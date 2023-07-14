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
    ChannelStats *chstats;
    int nb_channels;
    int64_t tc_samples;
    double time_constant;
    int split_channels;
    int output_bits;
    int64_t window_pos;
} WaveFormDataContext;

#define OFFSET(x) offsetof(WaveFormDataContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption waveformdata_options[] = {
    { "length", "set the window length", OFFSET(time_constant), AV_OPT_TYPE_DOUBLE, {.dbl=3}, .01, 100, FLAGS }, // FIXME: min/max/default to match audiowaveform?
    { "split_chan", "output multi-channel waveform data", OFFSET(split_channels), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, FLAGS },
    { "bits", "waveform data-point resolution", OFFSET(output_bits), AV_OPT_TYPE_INT, {.i64 = OUTPUT_BITS_16}, 0, OUTPUT_BITS_16, FLAGS, "bits" },
         { "8",  "8 bits", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_BITS_8},  .flags = FLAGS, .unit = "bits" },
         { "16", "16 bits", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_BITS_16}, .flags = FLAGS, .unit = "bits" },
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

    return 0;
}

static void write_data_point(AVFilterContext *ctx, float min, float max)
{
    WaveFormDataContext *s = ctx->priv;

    if (s->output_bits == OUTPUT_BITS_16) {
        int16_t min_shrt = (int16_t)lrint(min * SHRT_MAX);
        int16_t max_shrt = (int16_t)lrint(max * SHRT_MAX);
        av_log(ctx, AV_LOG_INFO, "Write: Min: %d Max: %d\n", min_shrt, max_shrt);
    } else {
        int8_t min_b = (int8_t)lrint(min * CHAR_MAX);
        int8_t max_b = (int8_t)lrint(max * CHAR_MAX);
        av_log(ctx, AV_LOG_INFO, "Write: Min: %d Max: %d\n", min_b, max_b);
    }
}

static void finish_block(AVFilterContext *ctx)
{
    WaveFormDataContext *s = ctx->priv;
    const int channels = s->nb_channels;
    float min_sum = 0, max_sum = 0;

    for (int c = 0; c < channels; c++) {
        ChannelStats *p = &s->chstats[c];
        av_log(ctx, AV_LOG_INFO, "Channel %d: Min: %g Max: %g\n", c + 1, p->min, p->max);
        if (s->split_channels)
            write_data_point(ctx, p->min, p->max);
        else {
            min_sum += p->min;
            max_sum += p->max;
        }
        p->min = 0;
        p->max = 0;
    }
    if (!s->split_channels)
        write_data_point(ctx, min_sum / channels, max_sum / channels);
    s->window_pos = 0;
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
        for (int proc_samples = 0; proc_samples < buf->nb_samples; proc_samples += s->tc_samples) {
            int window_end = av_clip(proc_samples + s->tc_samples, 0, buf->nb_samples);

            for (int c = 0; c < channels; c++) {
                ChannelStats *p = &s->chstats[c];
                const float *src = ((const float *)buf->extended_data[c]) + proc_samples;

                for (int i = proc_samples; i < window_end; i++, src++)
                    update_stat(s, p, *src);
            }
            s->window_pos = window_end - proc_samples;
            if (s->window_pos == s->tc_samples)
                finish_block(ctx);
        }
        break;
    case AV_SAMPLE_FMT_FLT: {
        const float *src = (const float *)buf->extended_data[0];

        for (int i = 0; i < buf->nb_samples; i++) {
            for (int c = 0; c < channels; c++, src++)
                update_stat(s, &s->chstats[c], *src);
            s->window_pos++;
            if (s->window_pos == s->tc_samples)
                finish_block(ctx);
        }}
        break;
    }

    return ff_filter_frame(inlink->dst->outputs[0], buf);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    WaveFormDataContext *s = ctx->priv;

    if (s->window_pos != 0)
        finish_block(ctx);
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
    .uninit        = uninit,
    .flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(waveformdata_inputs),
    FILTER_OUTPUTS(waveformdata_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_FLT),
};
