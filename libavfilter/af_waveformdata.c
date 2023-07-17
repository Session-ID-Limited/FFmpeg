/*
 * Copyright (c) 2023 Tadas Dailyda
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <float.h>

#include "libavutil/ffmath.h"
#include "libavutil/opt.h"
#include "libavformat/avio.h"
#include "audio.h"
#include "avfilter.h"
#include "internal.h"

enum OutputBits {
    OUTPUT_BITS_8,
    OUTPUT_BITS_16
};

typedef struct ChannelStats {
    float min;
    float max;
} ChannelStats;

typedef struct WaveFormDataContext {
    const AVClass *class;

    double time_constant;
    int output_bits;
    char *file_str;

    AVIOContext* avio_context;
    ChannelStats *chstats;
    int nb_channels;
    int64_t tc_samples;
    int64_t window_pos;
    int64_t total_blocks;
} WaveFormDataContext;

#define OFFSET(x) offsetof(WaveFormDataContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption waveformdata_options[] = {
    { "length", "set the window length", OFFSET(time_constant), AV_OPT_TYPE_DOUBLE, {.dbl=3}, .01, 100, FLAGS }, // FIXME: min/max/default to match audiowaveform, also mut excl option for setting window length in samples?
    { "bits", "waveform data-point resolution", OFFSET(output_bits), AV_OPT_TYPE_INT, {.i64 = OUTPUT_BITS_16}, 0, OUTPUT_BITS_16, FLAGS, "bits" },
         { "8",  "8 bits", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_BITS_8},  .flags = FLAGS, .unit = "bits" },
         { "16", "16 bits", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_BITS_16}, .flags = FLAGS, .unit = "bits" },
    { "file", "set file for waveform data", OFFSET(file_str), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, FLAGS }, // FIXME: should it be required?
    { NULL }
};

AVFILTER_DEFINE_CLASS(waveformdata);

static int config_output(AVFilterLink *outlink)
{
    WaveFormDataContext *s = outlink->src->priv;

    s->chstats = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->chstats));
    if (!s->chstats)
        return AVERROR(ENOMEM);
    s->nb_channels = outlink->ch_layout.nb_channels;
    s->tc_samples = lrint(s->time_constant * outlink->sample_rate);
    s->window_pos = 0;
    s->total_blocks = 0;

    if (s->avio_context) {
        // write file headers
        // 32b: version 1|2
        avio_wl32(s->avio_context, s->nb_channels > 1 ? 2 : 1);
        // 32b: flags
        avio_wl32(s->avio_context, s->output_bits == OUTPUT_BITS_8 ? 1 : 0);
        // 32b: sample rate
        avio_wl32(s->avio_context, outlink->sample_rate);
        // 32b: samples per pixel
        avio_wl32(s->avio_context, s->tc_samples);
        // 32b: data-point count (size), to be filled in `uninit`
        avio_wl32(s->avio_context, 0);
        // 32b: channel count (if split_channels)
        if (s->nb_channels > 1)
            avio_wl32(s->avio_context, s->nb_channels);
    }

    return 0;
}

static void write_data_point(WaveFormDataContext *s, float min, float max)
{
    // FIXME: make this a function pointer in `s` using 3 functions: 16bit, 8bit, NOOP (or 2 if file opt is required)
    if (s->avio_context) {
        if (s->output_bits == OUTPUT_BITS_16) {
            avio_wl16(s->avio_context, (int16_t)roundf(min * SHRT_MAX));
            avio_wl16(s->avio_context, (int16_t)roundf(max * SHRT_MAX));
        } else {
            avio_w8(s->avio_context, (int8_t)roundf(min * CHAR_MAX));
            avio_w8(s->avio_context, (int8_t)roundf(max * CHAR_MAX));
        }
    }
}

static void finish_block(WaveFormDataContext *s)
{
    const int channels = s->nb_channels;

    for (int c = 0; c < channels; c++) {
        ChannelStats *p = &s->chstats[c];
        write_data_point(s, p->min, p->max);
        p->min = 0;
        p->max = 0;
    }
    s->window_pos = 0;
    s->total_blocks++;
}

static void update_stat(WaveFormDataContext *s, ChannelStats *p, float sample)
{
    if (sample > p->max)
        p->max = sample;
    if (sample < p->min)
        p->min = sample;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AVFilterContext *ctx = inlink->dst;
    WaveFormDataContext *s = ctx->priv;
    const int channels = s->nb_channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        for (int proc_smpl = 0; proc_smpl < buf->nb_samples; proc_smpl += s->tc_samples) {
            int window_end = av_clip(proc_smpl + s->tc_samples, 0, buf->nb_samples);

            for (int c = 0; c < channels; c++) {
                ChannelStats *p = &s->chstats[c];
                const float *src = ((const float *)buf->extended_data[c]) + proc_smpl;

                for (int i = proc_smpl; i < window_end; i++, src++)
                    update_stat(s, p, *src);
            }
            s->window_pos = window_end - proc_smpl;
            if (s->window_pos == s->tc_samples)
                finish_block(s);
        }
        break;
    case AV_SAMPLE_FMT_FLT: {
        const float *src = (const float *)buf->extended_data[0];

        for (int i = 0; i < buf->nb_samples; i++) {
            for (int c = 0; c < channels; c++, src++)
                update_stat(s, &s->chstats[c], *src);
            s->window_pos++;
            if (s->window_pos == s->tc_samples)
                finish_block(s);
        }}
        break;
    }

    return ff_filter_frame(inlink->dst->outputs[0], buf);
}

static av_cold int init(AVFilterContext *ctx)
{
    WaveFormDataContext *s = ctx->priv;

    s->avio_context = NULL;
    if (s->file_str) {
        int ret = avio_open(&s->avio_context, s->file_str, AVIO_FLAG_WRITE);

        if (ret < 0) {
            char buf[128];
            av_strerror(ret, buf, sizeof(buf));
            av_log(ctx, AV_LOG_ERROR, "Could not open %s: %s\n",
                   s->file_str, buf);
            return ret;
        }
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    WaveFormDataContext *s = ctx->priv;

    if (s->window_pos != 0)
        finish_block(s);
    if (s->avio_context) {
        // write total block count in headers (5th field after 4x32b)
        avio_seek(s->avio_context, 4 * 4, SEEK_SET);
        avio_wl32(s->avio_context, (uint32_t)s->total_blocks);
        avio_closep(&s->avio_context);
    }

    av_freep(&s->chstats);
}

static const AVFilterPad waveformdata_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad waveformdata_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const AVFilter ff_af_waveformdata = {
    .name          = "waveformdata",
    .description   = NULL_IF_CONFIG_SMALL("Generate peaks.js compatible waveform data file."),
    .priv_size     = sizeof(WaveFormDataContext),
    .priv_class    = &waveformdata_class,
    .init          = init,
    .uninit        = uninit,
    .flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(waveformdata_inputs),
    FILTER_OUTPUTS(waveformdata_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_FLT),
};
