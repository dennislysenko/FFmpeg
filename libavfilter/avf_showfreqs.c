/*
 * Copyright (c) 2015 Paul B Mahol
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

#include <math.h>
#include <flam3.h>

#include "libavcodec/avfft.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "audio.h"
#include "video.h"
#include "avfilter.h"
#include "internal.h"
#include "window_func.h"

#if CONFIG_FLAM3
#include "flam3.h"

#define FLAM3_STRINGIFY(x) #x
char FLAM3_WREATH[] = FLAM3_STRINGIFY(
<flame name="wreath" version="Apophysis 2.09" size="480 360" center="0 0" scale="480" oversample="1" filter="0.2" quality="5" background="0 0 0" brightness="10.6913043478261" gamma="2.44" gamma_threshold="0.11" >
        <xform weight="16" color="0" opacity="0" diamond="1" coefs="1.061489 0 0.994422 1.074955 0 0" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 1 0" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.951057 0.309017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.809017 0.587785" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.587785 0.809017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.309017 0.951057" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0 1" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.309017 0.951057" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.587785 0.809017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.809017 0.587785" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.951057 0.309017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -1 0" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.951057 -0.309017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.809017 -0.587785" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.587785 -0.809017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 -0.309017 -0.951057" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0 -1" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.309017 -0.951057" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.587785 -0.809017" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.809017 -0.587785" />
        <xform weight="0.5" color="1" linear="0.25" coefs="1 0 0 1 0.951057 -0.309017" />
        <palette count="256" format="RGB">
        9007BB97039F9B176E9F2C3D8D41287C57138667278F7914
        96851197871099891095850F92810F907F0F8F7D0F746B0C
        70630D6C5C0E665809605505625506655507655A0A6B5C07
        6B5A0C735D0C7B610D76610B716109726604735E036E6008
        786908837208968B08A9A408A3AE049EB80087DD1C9ACF0F
        A89B008F82047669097166066D63045D70002C6C150BAE79
        1A9F6D2A91623D8944508126837C389F7F2CDC9624D4CF2D
        DFBB1FC3A41DA88D1CA48E20A190249E89149E8A1198850F
        93800E8E7C0E86750E7F6F0F7161096165113F6D2C10884A
        008867018A4A038D2E0C98153D870047970233900E0C8D79
        1A8066287354385E3D484927635A0B665706675805675805
        695B036D5C01725D008065008A77008F7B0B8E7C0C7C680F
        6B7D1F5A922F579640549A5233E43E0DEB0A1CCB024A9E08
        8C74148E7A1191800E95810E907F0D8E7C0E7A65125C5D0F
        445B142C5A1A10724113AC6C00CC8701FBA210E3AE11D38A
        38D41135BD0833A60067A70074B3048190007E6B04756900
        8071058C7A0A8F7D0F907F0D907E1089760F7D68157E571E
        98151A8814147127246C451A69570D675C0A6A5B08695609
        6956086956076A5B067556037154126F2C02594602504600
        594A055E500062560A5050063250080C7D001D89112A8317
        4A821D4D7B0F60600A655D0A74640D7A703279763F588193
        37A5C05F99FF2C4DC16A28C8871CDCC109F3E724FEE512DD
        F9079ABF0064F46B27FB8A2AFAE20CFAFF26DBFF33AAEE29
        9CE60DAE9C12A3950E9E8A119C87109B881098850F96870A
        928D0F96850F877C097C8109659A3244B55B32F37C55EE79
        47CB5383DA1CB0BF26D1B925C3B971C8BC72E17E9DF7177A
        C06425A6721FA367009A721D807B2A9F934B57DDFF44C1DD
        33F2FF30F1D02EF1A22BF66029FB1E19DF2009C32200E261
        00E49F00E7DD00C7C900A8B53A6D8275334F7F203D890E2B
        C60732D9132EED1F2BC333159947007F35126523242F7082
        3B899947A3B02399A5008F9A027B900568872D44707E089D
        CC04BEDF12CEF221DEDB20CDC51FBDB816CDAC0EDD8A0BD8
        </palette>
</flame>
);

char FLAM3_PABLO[] = FLAM3_STRINGIFY(
<flame name="Apophysis-160228-1" version="apophysis-j 2.9" size="100 100" center="0.4276471112825241 -0.07366013392303089" scale="32.93691281813009" oversample="1" filter="0.2" quality="5.0" background="0.0 0.0 0.0" brightness="4.0" gamma="4.0" vibrancy="1.0" >
        <xform weight="0.5" color="0.0" hyperbolic="0.04526427520602694" julia="0.9547357247939731" coefs="0.8577261877766948 0.1868437171181121 0.3308409593078538 0.8965508366269237 -0.31912619639074546 -0.3778000074350456"  />
        <xform weight="0.5" color="1.0" diamond="0.3113785542626345" waves="0.6886214457373655" coefs="-0.48836921272647715 0.8480483375500903 -0.6943247204829344 -0.07213206882587403 0.7636392841390722 -0.6621535566142094"  />
        <palette count="256" format="RGB">
        894B0E624A2E5B5550546172404F602C3D4F24394E1C364D
        51402E5F43276E47209B5917C86B0EBA7B3BAD8C69AA9174
        A89680B3DBFFA4CDF496C0EA82BBF46FB7FF6AB8FF66BAFF
        239DFF1D96FF1790FF2898FF39A0FF3D9CF84199F2379EFD
        28A4FE1398FF1698FF1998FF31A3FF49AFFF56B4FF63BAFF
        89CFFF90CFFE97CFFE83CBFD70C7FD6CC0FE69B9FF509EE9
        116FC9014E960143850238741961A9318ADE4F9AE46EABEA
        D2E8FDE8F3FDFFFFFDF9FBFEF4F7FFE0F2FFCDEEFFCBF3FB
        C5F1FEC6EBFEA9D4F88CBEF381AFDE77A1C96E7683605041
        111216092543013870014FA00166D0006EE00076F10089FF
        027DFE0164CA014A980231670724470C1828000A16010916
        21170D2F1E0D3E260E472D135035186E4319704319724317
        794410774B1E7C5326815B2E8B7357958B817688966D90AE
        47729C4C6B8A526478595C606155495E462A6A44174D3318
        3D1E011F0F00180B00110801090400000000000000010101
        00112B082C5410477D0F559A0F64B70A6FD70985FF2385E8
        4496EA5591CF5888B95C80A45C758B373F4A252422120F0A
        0201000100000000000000000000000704000C0800221101
        321A00593C1E5D462D62503C71666262707B5E6E7D636260
        40332A382E25312A201E1A191F2022012349003C760052A8
        0069D6008BFF1291FF2498FF2FA2FF3AA8FF40AAFE46A0F8
        2682D91F76C6186AB43065993E61813C41454D3318421F01
        3E1C003A1A013519012C1702241100271201281A0D352817
        3A3F453A434E3A47572B527B144677014B960657A8005BBA
        0D6ACA2587E849A1FA70C6FF90D8FEB5E1FEBBECFDBDEAFD
        BFE9FFC3E2FFC9F0FFE1F8FEFAFEFFE0F0FFCEE7FDB7DEFF
        92D0FF66A9ED217FDF0066CD0352A10E38621E2F3F0E1116
        00081501040D01040D00070F0A141E133455235C93236BB5
        64A1DA77A9DC70A5DB6F93B78173686353446A421F46290B
        401E023B1C00401F00532D067142167343126E4211462200
        311700170B00080300000000000000000000000000070300
        0B0602231C14322B23574D43666A6D7798BBBED6EE96B9DF
        </palette>
</flame>
);

enum Flame          { WREATH, PABLO, NB_FLAMES };
#endif

enum DisplayMode    { LINE, BAR, DOT, TWENTYBANDS, FLAM3, NB_MODES };
enum ChannelMode    { COMBINED, SEPARATE, NB_CMODES };
enum FrequencyScale { FS_LINEAR, FS_LOG, FS_RLOG, NB_FSCALES };
enum AmplitudeScale { AS_LINEAR, AS_SQRT, AS_CBRT, AS_LOG, NB_ASCALES };

#define NB_BANDS 20

typedef struct ShowFreqsContext {
    const AVClass *class;
    int w, h;
    int mode;
    int cmode;
    int fft_bits;
    int ascale, fscale;
    int avg;
    int win_func;
    FFTContext *fft;
    FFTComplex **fft_data;
    float **avg_data;
    float *window_func_lut;
    float overlap;
    int hop_size;
    int nb_channels;
    int nb_freq;
    int win_size;
    float scale;
    char *colors;
    AVAudioFifo *fifo;
    int64_t pts;
    double heights[NB_BANDS];
    double velocities[NB_BANDS];
#if CONFIG_FLAM3
    flam3_frame *frame;
    flam3_genome *cps;
#endif
    //int cps_counter;
    int ncps;
    int selected_flame;
} ShowFreqsContext;

#define OFFSET(x) offsetof(ShowFreqsContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption showfreqs_options[] = {
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "1024x512"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "1024x512"}, 0, 0, FLAGS },
    { "mode", "set display mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64=BAR}, 0, NB_MODES-1, FLAGS, "mode" },
        { "line", "show lines",  0, AV_OPT_TYPE_CONST, {.i64=LINE},   0, 0, FLAGS, "mode" },
        { "bar",  "show bars",   0, AV_OPT_TYPE_CONST, {.i64=BAR},    0, 0, FLAGS, "mode" },
        { "dot",  "show dots",   0, AV_OPT_TYPE_CONST, {.i64=DOT},    0, 0, FLAGS, "mode" },
        { "twentybands", "20-band bars", 0, AV_OPT_TYPE_CONST, {.i64=TWENTYBANDS}, 0, 0, FLAGS, "mode" },
#if CONFIG_FLAM3
        { "flam3", "fractal flame", 0, AV_OPT_TYPE_CONST, {.i64=FLAM3}, 0, 0, FLAGS, "mode" },
    { "flame", "set flam3 fractal flame", OFFSET(selected_flame), AV_OPT_TYPE_INT, {.i64=WREATH}, 0, NB_FLAMES-1, FLAGS, "flame" },
        { "wreath", "shows a circle of triangles that pulsate with 20-band frequences",  0, AV_OPT_TYPE_CONST, {.i64=WREATH},   0, 0, FLAGS, "flame" },
        { "pablo", "shows a circle of triangles that pulsate with 20-band frequences",  0, AV_OPT_TYPE_CONST, {.i64=PABLO},   0, 0, FLAGS, "flame" },
#endif
    { "ascale", "set amplitude scale", OFFSET(ascale), AV_OPT_TYPE_INT, {.i64=AS_LOG}, 0, NB_ASCALES-1, FLAGS, "ascale" },
        { "lin",  "linear",      0, AV_OPT_TYPE_CONST, {.i64=AS_LINEAR}, 0, 0, FLAGS, "ascale" },
        { "sqrt", "square root", 0, AV_OPT_TYPE_CONST, {.i64=AS_SQRT},   0, 0, FLAGS, "ascale" },
        { "cbrt", "cubic root",  0, AV_OPT_TYPE_CONST, {.i64=AS_CBRT},   0, 0, FLAGS, "ascale" },
        { "log",  "logarithmic", 0, AV_OPT_TYPE_CONST, {.i64=AS_LOG},    0, 0, FLAGS, "ascale" },
    { "fscale", "set frequency scale", OFFSET(fscale), AV_OPT_TYPE_INT, {.i64=FS_LINEAR}, 0, NB_FSCALES-1, FLAGS, "fscale" },
        { "lin",  "linear",              0, AV_OPT_TYPE_CONST, {.i64=FS_LINEAR}, 0, 0, FLAGS, "fscale" },
        { "log",  "logarithmic",         0, AV_OPT_TYPE_CONST, {.i64=FS_LOG},    0, 0, FLAGS, "fscale" },
        { "rlog", "reverse logarithmic", 0, AV_OPT_TYPE_CONST, {.i64=FS_RLOG},   0, 0, FLAGS, "fscale" },
    { "win_size", "set window size", OFFSET(fft_bits), AV_OPT_TYPE_INT, {.i64=11}, 4, 16, FLAGS, "fft" },
        { "w16",    0, 0, AV_OPT_TYPE_CONST, {.i64=4},  0, 0, FLAGS, "fft" },
        { "w32",    0, 0, AV_OPT_TYPE_CONST, {.i64=5},  0, 0, FLAGS, "fft" },
        { "w64",    0, 0, AV_OPT_TYPE_CONST, {.i64=6},  0, 0, FLAGS, "fft" },
        { "w128",   0, 0, AV_OPT_TYPE_CONST, {.i64=7},  0, 0, FLAGS, "fft" },
        { "w256",   0, 0, AV_OPT_TYPE_CONST, {.i64=8},  0, 0, FLAGS, "fft" },
        { "w512",   0, 0, AV_OPT_TYPE_CONST, {.i64=9},  0, 0, FLAGS, "fft" },
        { "w1024",  0, 0, AV_OPT_TYPE_CONST, {.i64=10}, 0, 0, FLAGS, "fft" },
        { "w2048",  0, 0, AV_OPT_TYPE_CONST, {.i64=11}, 0, 0, FLAGS, "fft" },
        { "w4096",  0, 0, AV_OPT_TYPE_CONST, {.i64=12}, 0, 0, FLAGS, "fft" },
        { "w8192",  0, 0, AV_OPT_TYPE_CONST, {.i64=13}, 0, 0, FLAGS, "fft" },
        { "w16384", 0, 0, AV_OPT_TYPE_CONST, {.i64=14}, 0, 0, FLAGS, "fft" },
        { "w32768", 0, 0, AV_OPT_TYPE_CONST, {.i64=15}, 0, 0, FLAGS, "fft" },
        { "w65536", 0, 0, AV_OPT_TYPE_CONST, {.i64=16}, 0, 0, FLAGS, "fft" },
    { "win_func", "set window function", OFFSET(win_func), AV_OPT_TYPE_INT, {.i64=WFUNC_HANNING}, 0, NB_WFUNC-1, FLAGS, "win_func" },
        { "rect",     "Rectangular",      0, AV_OPT_TYPE_CONST, {.i64=WFUNC_RECT},     0, 0, FLAGS, "win_func" },
        { "bartlett", "Bartlett",         0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BARTLETT}, 0, 0, FLAGS, "win_func" },
        { "hanning",  "Hanning",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_HANNING},  0, 0, FLAGS, "win_func" },
        { "hamming",  "Hamming",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_HAMMING},  0, 0, FLAGS, "win_func" },
        { "blackman", "Blackman",         0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BLACKMAN}, 0, 0, FLAGS, "win_func" },
        { "welch",    "Welch",            0, AV_OPT_TYPE_CONST, {.i64=WFUNC_WELCH},    0, 0, FLAGS, "win_func" },
        { "flattop",  "Flat-top",         0, AV_OPT_TYPE_CONST, {.i64=WFUNC_FLATTOP},  0, 0, FLAGS, "win_func" },
        { "bharris",  "Blackman-Harris",  0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BHARRIS},  0, 0, FLAGS, "win_func" },
        { "bnuttall", "Blackman-Nuttall", 0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BNUTTALL}, 0, 0, FLAGS, "win_func" },
        { "bhann",    "Bartlett-Hann",    0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BHANN},    0, 0, FLAGS, "win_func" },
        { "sine",     "Sine",             0, AV_OPT_TYPE_CONST, {.i64=WFUNC_SINE},     0, 0, FLAGS, "win_func" },
        { "nuttall",  "Nuttall",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_NUTTALL},  0, 0, FLAGS, "win_func" },
        { "lanczos",  "Lanczos",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_LANCZOS},  0, 0, FLAGS, "win_func" },
        { "gauss",    "Gauss",            0, AV_OPT_TYPE_CONST, {.i64=WFUNC_GAUSS},    0, 0, FLAGS, "win_func" },
        { "tukey",    "Tukey",            0, AV_OPT_TYPE_CONST, {.i64=WFUNC_TUKEY},    0, 0, FLAGS, "win_func" },
    { "overlap",  "set window overlap", OFFSET(overlap), AV_OPT_TYPE_FLOAT, {.dbl=1.}, 0., 1., FLAGS },
    { "averaging", "set time averaging", OFFSET(avg), AV_OPT_TYPE_INT, {.i64=1}, 0, INT32_MAX, FLAGS },
    { "colors", "set channels colors", OFFSET(colors), AV_OPT_TYPE_STRING, {.str = "red|green|blue|yellow|orange|lime|pink|magenta|brown" }, 0, 0, FLAGS },
    { "cmode", "set channel mode", OFFSET(cmode), AV_OPT_TYPE_INT, {.i64=COMBINED}, 0, NB_CMODES-1, FLAGS, "cmode" },
        { "combined", "show all channels in same window",  0, AV_OPT_TYPE_CONST, {.i64=COMBINED}, 0, 0, FLAGS, "cmode" },
        { "separate", "show each channel in own window",   0, AV_OPT_TYPE_CONST, {.i64=SEPARATE}, 0, 0, FLAGS, "cmode" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(showfreqs);

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts = NULL;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_RGBA, AV_PIX_FMT_NONE };
    int ret;

    /* set input audio formats */
    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &inlink->out_formats)) < 0)
        return ret;

    layouts = ff_all_channel_layouts();
    if ((ret = ff_channel_layouts_ref(layouts, &inlink->out_channel_layouts)) < 0)
        return ret;

    formats = ff_all_samplerates();
    if ((ret = ff_formats_ref(formats, &inlink->out_samplerates)) < 0)
        return ret;

    /* set output video format */
    formats = ff_make_format_list(pix_fmts);
    if ((ret = ff_formats_ref(formats, &outlink->in_formats)) < 0)
        return ret;

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    ShowFreqsContext *s = ctx->priv;

    s->pts = AV_NOPTS_VALUE;

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ShowFreqsContext *s = ctx->priv;
    float overlap;
    int i;

    s->nb_freq = 1 << (s->fft_bits - 1);
    s->win_size = s->nb_freq << 1;
    av_audio_fifo_free(s->fifo);
    av_fft_end(s->fft);
    s->fft = av_fft_init(s->fft_bits, 0);
    if (!s->fft) {
        av_log(ctx, AV_LOG_ERROR, "Unable to create FFT context. "
               "The window size might be too high.\n");
        return AVERROR(ENOMEM);
    }

//    if (s->bands_output != NULL) {
//        fclose(s->bands_output);
//        s->bands_output = NULL;
//    }

    /* FFT buffers: x2 for each (display) channel buffer.
     * Note: we use free and malloc instead of a realloc-like function to
     * make sure the buffer is aligned in memory for the FFT functions. */
    for (i = 0; i < s->nb_channels; i++) {
        av_freep(&s->fft_data[i]);
        av_freep(&s->avg_data[i]);
    }
    av_freep(&s->fft_data);
    av_freep(&s->avg_data);
    s->nb_channels = inlink->channels;

    s->fft_data = av_calloc(s->nb_channels, sizeof(*s->fft_data));
    if (!s->fft_data)
        return AVERROR(ENOMEM);
    s->avg_data = av_calloc(s->nb_channels, sizeof(*s->avg_data));
    if (!s->fft_data)
        return AVERROR(ENOMEM);
    for (i = 0; i < s->nb_channels; i++) {
        s->fft_data[i] = av_calloc(s->win_size, sizeof(**s->fft_data));
        s->avg_data[i] = av_calloc(s->nb_freq, sizeof(**s->avg_data));
        if (!s->fft_data[i] || !s->avg_data[i])
            return AVERROR(ENOMEM);
    }

    /* pre-calc windowing function */
    s->window_func_lut = av_realloc_f(s->window_func_lut, s->win_size,
                                      sizeof(*s->window_func_lut));

    av_log(ctx, AV_LOG_WARNING, "Using window size %d\n", s->win_size);

    if (!s->window_func_lut)
        return AVERROR(ENOMEM);
    ff_generate_window_func(s->window_func_lut, s->win_size, s->win_func, &overlap);
    if (s->overlap == 1.)
        s->overlap = overlap;
    s->hop_size = (1. - s->overlap) * s->win_size;
    if (s->hop_size < 1) {
        av_log(ctx, AV_LOG_ERROR, "overlap %f too big\n", s->overlap);
        return AVERROR(EINVAL);
    }

    for (s->scale = 0, i = 0; i < s->win_size; i++) {
        s->scale += s->window_func_lut[i] * s->window_func_lut[i];
    }

    outlink->frame_rate = av_make_q(inlink->sample_rate, s->win_size * (1.-s->overlap));
    outlink->sample_aspect_ratio = (AVRational){1,1};
    outlink->w = s->w;
    outlink->h = s->h;

    s->fifo = av_audio_fifo_alloc(inlink->format, inlink->channels, s->win_size);
    if (!s->fifo)
        return AVERROR(ENOMEM);
    return 0;
}

static inline void draw_dot(AVFrame *out, int x, int y, uint8_t fg[4])
{

    uint32_t color = AV_RL32(out->data[0] + y * out->linesize[0] + x * 4);

    if ((color & 0xffffff) != 0)
        AV_WL32(out->data[0] + y * out->linesize[0] + x * 4, AV_RL32(fg) | color);
    else
        AV_WL32(out->data[0] + y * out->linesize[0] + x * 4, AV_RL32(fg));
}

static int get_sx(ShowFreqsContext *s, int f)
{
    switch (s->fscale) {
    case FS_LINEAR:
        return (s->w/(float)s->nb_freq)*f;
    case FS_LOG:
        return s->w-pow(s->w, (s->nb_freq-f-1)/(s->nb_freq-1.));
    case FS_RLOG:
        return pow(s->w, f/(s->nb_freq-1.));
    }

    return 0;
}

static float get_bsize(ShowFreqsContext *s, int f)
{
    switch (s->fscale) {
    case FS_LINEAR:
        return s->w/(float)s->nb_freq;
    case FS_LOG:
        return pow(s->w, (s->nb_freq-f-1)/(s->nb_freq-1.))-
               pow(s->w, (s->nb_freq-f-2)/(s->nb_freq-1.));
    case FS_RLOG:
        return pow(s->w, (f+1)/(s->nb_freq-1.))-
               pow(s->w,  f   /(s->nb_freq-1.));
    }

    return 1.;
}

static inline void plot_freq(ShowFreqsContext *s, int ch,
                             double a, int f, uint8_t fg[4], int *prev_y,
                             AVFrame *out, AVFilterLink *outlink)
{
    const int w = s->w;
    const float avg = s->avg_data[ch][f];
    const float bsize = get_bsize(s, f);
    const int sx = get_sx(s, f);
    int end = outlink->h;
    int x, y, i;

    switch(s->ascale) {
    case AS_SQRT:
        a = 1.0 - sqrt(a);
        break;
    case AS_CBRT:
        a = 1.0 - cbrt(a);
        break;
    case AS_LOG:
        a = log(av_clipd(a, 1e-6, 1)) / log(1e-6);
        break;
    case AS_LINEAR:
        a = 1.0 - a;
        break;
    }

    switch (s->cmode) {
    case COMBINED:
        y = a * outlink->h - 1;
        break;
    case SEPARATE:
        end = (outlink->h / s->nb_channels) * (ch + 1);
        y = (outlink->h / s->nb_channels) * ch + a * (outlink->h / s->nb_channels) - 1;
        break;
    default:
        av_assert0(0);
    }
    if (y < 0)
        return;

    switch (s->avg) {
    case 0:
        y = s->avg_data[ch][f] = !outlink->frame_count ? y : FFMIN(avg, y);
        break;
    case 1:
        break;
    default:
        s->avg_data[ch][f] = avg + y * (y - avg) / (FFMIN(outlink->frame_count + 1, s->avg) * y);
        y = s->avg_data[ch][f];
        break;
    }

    switch(s->mode) {
    case LINE:
        if (*prev_y == -1) {
            *prev_y = y;
        }
        if (y <= *prev_y) {
            for (x = sx + 1; x < sx + bsize && x < w; x++)
                draw_dot(out, x, y, fg);
            for (i = y; i <= *prev_y; i++)
                draw_dot(out, sx, i, fg);
        } else {
            for (i = *prev_y; i <= y; i++)
                draw_dot(out, sx, i, fg);
            for (x = sx + 1; x < sx + bsize && x < w; x++)
                draw_dot(out, x, i - 1, fg);
        }
        *prev_y = y;
        break;
    case BAR:
        for (x = sx; x < sx + bsize && x < w; x++)
            for (i = y; i < end; i++)
                draw_dot(out, x, i, fg);
        break;
    case DOT:
        for (x = sx; x < sx + bsize && x < w; x++)
            draw_dot(out, x, y, fg);
        break;
    }
}

static int plot_freqs(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ShowFreqsContext *s = ctx->priv;
    const int win_size = s->win_size;
    char *colors, *color, *saveptr = NULL;
    AVFrame *out;
    int ch, n;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out)
        return AVERROR(ENOMEM);

    for (n = 0; n < outlink->h; n++)
        memset(out->data[0] + out->linesize[0] * n, 0, outlink->w * 4);

    /* fill FFT input with the number of samples available */
    for (ch = 0; ch < s->nb_channels; ch++) {
        const float *p = (float *)in->extended_data[ch];

        for (n = 0; n < in->nb_samples; n++) {
            s->fft_data[ch][n].re = p[n] * s->window_func_lut[n];
            s->fft_data[ch][n].im = 0;
        }
        for (; n < win_size; n++) {
            s->fft_data[ch][n].re = 0;
            s->fft_data[ch][n].im = 0;
        }
    }

    /* run FFT on each samples set */
    for (ch = 0; ch < s->nb_channels; ch++) {
        av_fft_permute(s->fft, s->fft_data[ch]);
        av_fft_calc(s->fft, s->fft_data[ch]);
    }

#define RE(x, ch) s->fft_data[ch][x].re
#define IM(x, ch) s->fft_data[ch][x].im
#define M(a, b) (sqrt((a) * (a) + (b) * (b)))

    colors = av_strdup(s->colors);
    if (!colors) {
        av_frame_free(&out);
        return AVERROR(ENOMEM);
    }

    uint8_t fg[4] = {0xff, 0xff, 0xff, 0xff};
    color = av_strtok(ch == 0 ? colors : NULL, " |", &saveptr);
    if (color)
        av_parse_color(fg, color, -1, ctx);

    if (s->mode == FLAM3 || s->mode == TWENTYBANDS) {
        const unsigned xscale[] = {0,1,2,3,4,5,6,7,8,11,15,20,27,
                                   36,47,62,82,107,141,184,255};
        float bar_size = s->w / (float)NB_BANDS;

        int i = 0;
        int j = 0;
        for (i = 0; i < NB_BANDS; i++) {
            double y = 0;

            // find the peaks in the different bands
            for (j = xscale[i]; j < xscale[i + 1]; j++) {
                for (ch = 0; ch < s->nb_channels; ch++) {
                    double a = av_clipd(M(RE(j, ch), IM(j, ch)) / s->scale, 0, 1);
                    if (a > y) {
                        y = a;
                    }
                }
            }

            // gradual falling:
            /*
            s->heights[i] -= 0.01;
            if (y > s->heights[i])
                s->heights[i] = y;
                */

            // averaging to make it more friendly:
            /*
            s->heights[i] = (s->heights[i] + y) / 2;
             */

            // Getting it onto a more moving scale
            if (y == 0)
                y = 0;
            else
                y = av_clipf(logf((float)y * 256.0f) / 8.0f, 0, 1);

            // Making it centered and less drastic
//            y = (y * 0.5f) + 0.25f;

            // Making it less drastic, giving it some kind of velocity
            double velocity = 0;
            double old_velocity = s->velocities[i];
            double old_height = s->heights[i];
            double diff = y - s->heights[i];

            velocity = FFSIGN(diff) * pow(diff, 2.0);

            if (FFSIGN(old_velocity) != FFSIGN(velocity)) {
                velocity = velocity * 0.1;
            }

            s->velocities[i] = velocity;
            s->heights[i] += velocity;
            s->heights[i] = av_clipd(s->heights[i], 0, 1);

            if (s->mode == TWENTYBANDS) {
                // if running twentybands, render the bar
                float start_x = bar_size * i;
                float end_x = start_x + bar_size;
                int cur_x = 0;
                const double max_y = (1 - s->heights[i]) * outlink->h;
                for (cur_x = (int) start_x; cur_x < (int) end_x; cur_x++) {
                    // trying to convert the fft result into a thing we can plot
                    // 0 was "ch"
                    int cur_y = 0;
                    for (cur_y = (int) max_y; cur_y < outlink->h; cur_y++) {
                        // colorized is lowest at the top and highest at the bottom, goes from 0 to 255
                        uint8_t colorized = (uint8_t) av_clipf(((float) cur_y / outlink->h) * 256, 0, 255);

#define GREEN_Y_START 96
#define RED_GRADIENT_SPEED 2.5
#define MIN_BAND_COLOR_CMP 20

                        uint8_t band_color[4];
                        band_color[0] = (uint8_t) av_clip((int) ((255 - colorized) * RED_GRADIENT_SPEED), MIN_BAND_COLOR_CMP, 255);
                        band_color[1] = (uint8_t) av_clip((int) ((colorized - GREEN_Y_START) * 2 * (255 / (255.0f - GREEN_Y_START))), MIN_BAND_COLOR_CMP, 255);
                        band_color[2] = MIN_BAND_COLOR_CMP;
                        // make sure no channel is at 0 so we can easily color key if we need to (thus all the 20s)

#undef GREEN_Y_START
#undef RED_GRADIENT_SPEED
#undef MIN_BAND_COLOR_CMP

                        draw_dot(out, cur_x, cur_y, band_color);
                    }
                }
            }
        }

#if CONFIG_FLAM3
        if (s->mode == FLAM3) {
            if (s->frame == NULL) {
                flam3_frame *frame = malloc(sizeof(flam3_frame));
                frame->pixel_aspect_ratio = outlink->sample_aspect_ratio.num / outlink->sample_aspect_ratio.den;

                // Load the right flame xml based on the selected flame
                switch (s->selected_flame) {
                    case WREATH:
                        s->cps = flam3_parse_xml2(FLAM3_WREATH, NULL, flam3_defaults_on, &s->ncps);
                        break;
                    case PABLO:
                        s->cps = flam3_parse_xml2(FLAM3_PABLO, NULL, flam3_defaults_on, &s->ncps);
                        break;
                        // write more cases here as you add them
                    default:
                        av_log(ctx, AV_LOG_ERROR, "unknown selected flame\n");
                }

                frame->ngenomes = 1;
                frame->verbose = 0;
                // TODO sizes under the 480x360 might be very weird/wrong in the visualization
                frame->bits = 64;
                frame->bytes_per_channel = 1;
                frame->earlyclip = 0;
                frame->time = 0.0;
                frame->progress = 0;
                frame->nthreads = 2;
                frame->sub_batch_size = 10000;

                s->frame = frame;
            }

            // go to the next flame in the render, but wrap around to zero so we don't go out of bounds.
//            s->cps_counter++;
//            s->cps_counter %= s->ncps;

            flam3_genome* genome = &s->cps[0];

            genome->width = out->width;
            genome->height = out->height;
            genome->pixels_per_unit = out->width;

            // Apply the audio band effects to the flame
            switch (s->selected_flame) {
                case WREATH: {
                    for (int band_index = 0;
                         band_index < NB_BANDS && (band_index + 1) < genome->num_xforms; band_index++) {
                        genome->xform[band_index + 1].var[0] = s->heights[band_index];
                    }
                    break;
                }
                case PABLO: {
                    double lowmax = 0;
                    double highmax = 0;
                    for (int bi = 0; bi < NB_BANDS; bi++) {
                        double height = s->heights[bi];
                        if (bi <= NB_BANDS/2) {
                            if (height > lowmax) {
                                lowmax = height;
                            }
                        } else {
                            if (height > highmax) {
                                highmax = height;
                            }
                        }
                    }

                    double y_x = genome->xform[0].c[1][0];
                    double y_y = genome->xform[0].c[1][1];
                    double mag = sqrt(y_x * y_x + y_y * y_y);
                    double target_mag = lowmax;

                    // Animate the mag of the y-point
                    genome->xform[0].c[1][0] = y_x / mag * target_mag;
                    genome->xform[0].c[1][1] = y_y / mag * target_mag;

//                    genome->xform[0].var[13] = lowmax;
//                    genome->xform[0].var[10] = 1 - lowmax;
//
//                    genome->xform[1].var[15] = highmax;
//                    genome->xform[1].var[11] = 1 - highmax;
                    break;
                };
            }

            // and load that flame into the frame
            s->frame->genomes = genome;

            av_log(ctx, AV_LOG_VERBOSE, "estimated memory %f\n", flam3_render_memory_required(s->frame));
            stat_struct stats;
            flam3_render(s->frame, out->data[0], flam3_field_both, 4, 0, &stats);
            av_log(ctx, AV_LOG_VERBOSE, "badvals=%0.3f, num_iters=%ld, render_seconds=%d\n", stats.badvals,
                   stats.num_iters, stats.render_seconds);
        }
#endif
    } else {
        for (ch = 0; ch < s->nb_channels; ch++) {

            int prev_y = -1, f;
            double a;

            a = av_clipd(M(RE(0, ch), 0) / s->scale, 0, 1);
            plot_freq(s, ch, a, 0, fg, &prev_y, out, outlink);

            for (f = 1; f < s->nb_freq; f++) {
                a = av_clipd(M(RE(f, ch), IM(f, ch)) / s->scale, 0, 1);

                plot_freq(s, ch, a, f, fg, &prev_y, out, outlink);
            }
        }
    }

    av_free(colors);
    out->pts = in->pts;
    return ff_filter_frame(outlink, out);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    ShowFreqsContext *s = ctx->priv;
    AVFrame *fin = NULL;
    int consumed = 0;
    int ret = 0;

    if (s->pts == AV_NOPTS_VALUE)
        s->pts = in->pts - av_audio_fifo_size(s->fifo);

    av_audio_fifo_write(s->fifo, (void **)in->extended_data, in->nb_samples);
    while (av_audio_fifo_size(s->fifo) >= s->win_size) {
        fin = ff_get_audio_buffer(inlink, s->win_size);
        if (!fin) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        fin->pts = s->pts + consumed;
        consumed += s->hop_size;
        ret = av_audio_fifo_peek(s->fifo, (void **)fin->extended_data, s->win_size);
        if (ret < 0)
            goto fail;

        ret = plot_freqs(inlink, fin);
        av_frame_free(&fin);
        av_audio_fifo_drain(s->fifo, s->hop_size);
        if (ret < 0)
            goto fail;
    }

fail:
    s->pts = AV_NOPTS_VALUE;
    av_frame_free(&fin);
    av_frame_free(&in);
    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ShowFreqsContext *s = ctx->priv;
    int i;

    av_fft_end(s->fft);
    for (i = 0; i < s->nb_channels; i++) {
        if (s->fft_data)
            av_freep(&s->fft_data[i]);
        if (s->avg_data)
            av_freep(&s->avg_data[i]);
    }
    av_freep(&s->fft_data);
    av_freep(&s->avg_data);
    av_freep(&s->window_func_lut);
    av_audio_fifo_free(s->fifo);

#if CONFIG_FLAM3
    if (s->frame != NULL) {
        free(s->frame);
    }

    if (s->cps != NULL) {
        free(s->cps);
    }
#endif
}

static const AVFilterPad showfreqs_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
    { NULL }
};

static const AVFilterPad showfreqs_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
    { NULL }
};

AVFilter ff_avf_showfreqs = {
    .name          = "showfreqs",
    .description   = NULL_IF_CONFIG_SMALL("Convert input audio to a frequencies video output."),
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,
    .priv_size     = sizeof(ShowFreqsContext),
    .inputs        = showfreqs_inputs,
    .outputs       = showfreqs_outputs,
    .priv_class    = &showfreqs_class,
};
