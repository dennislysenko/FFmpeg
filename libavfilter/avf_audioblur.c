#include "libavutil/imgutils.h"
#include "libavutil/eval.h"
#include "libavutil/opt.h"
#include "libavutil/pixfmt.h"
#include "avfilter.h"
#include "bufferqueue.h"
#include "formats.h"
#include "internal.h"
#include "dualinput.h"
#include "video.h"
#include "audio.h"
#include "libavutil/audio_fifo.h"
#include "libavcodec/avfft.h"
#include "window_func.h"
#include "libavutil/intreadwrite.h"

#define AUDIO 0
#define VIDEO 1

#define NB_BANDS 20

typedef struct AudioblurContext {
    FFFrameSync fs;
    double heights[NB_BANDS];
    double velocities[NB_BANDS];
    AVAudioFifo *fifo;
    FFTContext *fft;
    FFTComplex **fft_data;
    float **avg_data;
    float *window_func_lut;
    int hop_size;
    float scale;
    int nb_channels;
    int win_size;
    float overlap;
    int win_func;
    int nb_freq;
    int fft_bits;
    int hsub, vsub;
    uint8_t *temp[2]; ///< temporary buffer used in blur_power()
    int64_t pts;
} AudioblurContext;

#define OFFSET(x) offsetof(AudioblurContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption audioblur_options[] = {
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
//        { "shortest",    "force termination when the shortest input terminates", OFFSET(dinput.shortest), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
//        { "repeatlast",  "repeat last bottom frame", OFFSET(dinput.repeatlast), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
        { NULL }
};

AVFILTER_DEFINE_CLASS(audioblur);

static int read_freqs(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
//    AVFilterLink *outlink = ctx->outputs[0];
    AudioblurContext *s = ctx->priv;
    const int win_size = s->win_size;
    int ch, n;

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

    const unsigned xscale[] = {0,1,2,3,4,5,6,7,8,11,15,20,27,
                               36,47,62,82,107,141,184,255};

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

//        s->heights[i] = y;
//        av_log(ctx, AV_LOG_ERROR, "setting peak %d to %0.2f\n", i, y);

        // gradual falling:
        /*
        s->heights[i] -= 0.01;
        if (y > s->heights[i])
            s->heights[i] = y;
            */

        // averaging to make it more friendly:
        /*}
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
//        double velocity = 0;
//        double old_velocity = s->velocities[i];
//        double diff = y - s->heights[i];
//
//        velocity = FFSIGN(diff) * pow(diff, 2.0);
//
//        if (FFSIGN(old_velocity) != FFSIGN(velocity)) {
//            velocity = velocity * 0.1;
//        }
//
//        s->velocities[i] = velocity;
//        s->heights[i] += velocity;
//        s->heights[i] = av_clipd(s->heights[i], 0, 1);
        s->heights[i] = y;
    }

//    av_log(ctx, AV_LOG_ERROR, "first height is %0.2f\n", s->heights[0]);

    return 0;
}

static int filter_audio_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioblurContext *s = ctx->priv;
    AVFrame *fin = NULL;
    int consumed = 0;
    int ret = 0;

    if (s->pts == AV_NOPTS_VALUE)
        s->pts = in->pts - av_audio_fifo_size(s->fifo);

    ret = av_audio_fifo_write(s->fifo, (void **)in->extended_data, in->nb_samples);
//    av_log(ctx, AV_LOG_ERROR, "nb_samples=%d\n", in->nb_samples);
    if (ret < 0)
        goto fail;

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

        ret = read_freqs(inlink, fin);
//        av_frame_free(&fin);
        av_audio_fifo_drain(s->fifo, s->hop_size);
        if (ret < 0)
            goto fail;
    }

    fail:
    s->pts = AV_NOPTS_VALUE;
    // frame will be freed by framesync i think
//    av_frame_free(&fin);
//    av_frame_free(&in);
    return ret;
}

static inline uint8_t get(const uint8_t *buf, int buf_linesize, int x, int y) {
    return *(buf + y * buf_linesize + x/* * pixsize */);
}

static inline int sumrow(const uint8_t *buf, int buf_linesize, int center_x, int row_y, int radius, int w, int h) {
    int x, sum;
    sum = 0;
    int start = center_x - radius;
    if (start < 0) {
        start = 0;
    }
    int end = center_x + radius;
    if (end > h - 1) {
        end = h - 1;
    }
    for (x = start; x <= end; x++) {
        sum += get(buf, buf_linesize, x, row_y);
    }
    return sum;
}

static inline void depthblur(AVFilterContext *ctx, uint8_t *dst, int dst_linesize, const uint8_t *src, int src_linesize, int w, int h, double amount, int radius)
{
    int x, y;
    const int NUM_OTHER_PIXELS = (2 * radius + 1) * (2 * radius + 1) - 1;
    for (x = 0; x < w; x++) {
        int running = 0;

        for (y = 0; y < h + radius; y++) {
            running += sumrow(src, src_linesize, x, y, radius, w, h);

            int sum_being_removed;
            // we're basically doing a moving window average, so as we move down a row the row 2r-1 up gets removed from the running sum
            int row_being_removed = y - 2 * radius - 1;
            if (row_being_removed >= 0) {
                sum_being_removed = sumrow(src, src_linesize, x, row_being_removed, radius, w, h);
            } else {
                sum_being_removed = 0;
            }
            running -= sum_being_removed;

            int row_being_written = y - radius;
            if (row_being_written >= 0) {
                uint8_t value_being_set = get(src, src_linesize, x, row_being_written);
                double other = (double) (running - value_being_set) / NUM_OTHER_PIXELS;

//                av_log(ctx, AV_LOG_INFO, "blending %0.2f of %d (value_being_set) with %0.2f of %0.2f (other)\n", 1 - amount, value_being_set, amount, other);

                uint8_t result = (uint8_t)((1 - amount) * value_being_set + amount * other);
                *(dst + row_being_written * dst_linesize + x) = result;
            }
        }
    }
}
//
///* Naive boxblur would sum source pixels from x-radius .. x+radius
// * for destination pixel x. That would be O(radius*width).
// * If you now look at what source pixels represent 2 consecutive
// * output pixels, then you see they are almost identical and only
// * differ by 2 pixels, like:
// * src0       111111111
// * dst0           1
// * src1        111111111
// * dst1            1
// * src0-src1  1       -1
// * so when you know one output pixel you can find the next by just adding
// * and subtracting 1 input pixel.
// * The following code adopts this faster variant.
// */
//#define BLUR(type, depth)                                                   \
//static inline void blur ## depth(type *dst, int dst_step, const type *src,  \
//                                 int src_step, int len, int radius)         \
//{                                                                           \
//    const int length = radius*2 + 1;                                        \
//    const int inv = ((1<<16) + length/2)/length;                            \
//    int x, sum = src[radius*src_step];                                      \
//                                                                            \
//    for (x = 0; x < radius; x++)                                            \
//        sum += src[x*src_step]<<1;                                          \
//                                                                            \
//    sum = sum*inv + (1<<15);                                                \
//                                                                            \
//    for (x = 0; x <= radius; x++) {                                         \
//        sum += (src[(radius+x)*src_step] - src[(radius-x)*src_step])*inv;   \
//        dst[x*dst_step] = sum>>16;                                          \
//    }                                                                       \
//                                                                            \
//    for (; x < len-radius; x++) {                                           \
//        sum += (src[(radius+x)*src_step] - src[(x-radius-1)*src_step])*inv; \
//        dst[x*dst_step] = sum >>16;                                         \
//    }                                                                       \
//                                                                            \
//    for (; x < len; x++) {                                                  \
//        sum += (src[(2*len-radius-x-1)*src_step] - src[(x-radius-1)*src_step])*inv; \
//        dst[x*dst_step] = sum>>16;                                          \
//    }                                                                       \
//}
//
//BLUR(uint8_t,   8)
//BLUR(uint16_t, 16)
//
//#undef BLUR
//
//static inline void blur(uint8_t *dst, int dst_step, const uint8_t *src, int src_step,
//                        int len, int radius, int pixsize)
//{
//    if (pixsize == 1) blur8 (dst, dst_step   , src, src_step   , len, radius);
//    else              blur16((uint16_t*)dst, dst_step>>1, (const uint16_t*)src, src_step>>1, len, radius);
//}
//
//static inline void blur_power(uint8_t *dst, int dst_step, const uint8_t *src, int src_step,
//                              int len, int radius, int power, uint8_t *temp[2], int pixsize)
//{
//    uint8_t *a = temp[0], *b = temp[1];
//
//    if (radius && power) {
//        blur(a, pixsize, src, src_step, len, radius, pixsize);
//        for (; power > 2; power--) {
//            uint8_t *c;
//            blur(b, pixsize, a, pixsize, len, radius, pixsize);
//            c = a; a = b; b = c;
//        }
//        if (power > 1) {
//            blur(dst, dst_step, a, pixsize, len, radius, pixsize);
//        } else {
//            int i;
//            if (pixsize == 1) {
//                for (i = 0; i < len; i++)
//                    dst[i*dst_step] = a[i];
//            } else
//                for (i = 0; i < len; i++)
//                    *(uint16_t*)(dst + i*dst_step) = ((uint16_t*)a)[i];
//        }
//    } else {
//        int i;
//        if (pixsize == 1) {
//            for (i = 0; i < len; i++)
//                dst[i*dst_step] = src[i*src_step];
//        } else
//            for (i = 0; i < len; i++)
//                *(uint16_t*)(dst + i*dst_step) = *(uint16_t*)(src + i*src_step);
//    }
//}
//
//static void hblur(uint8_t *dst, int dst_linesize, const uint8_t *src, int src_linesize,
//                  int w, int h, int radius, int power, uint8_t *temp[2], int pixsize)
//{
//    int y;
//
//    if (radius == 0 && dst == src)
//        return;
//
//    for (y = 0; y < h; y++)
//        blur_power(dst + y*dst_linesize, pixsize, src + y*src_linesize, pixsize,
//                   w, radius, power, temp, pixsize);
//}
//
//static void vblur(uint8_t *dst, int dst_linesize, const uint8_t *src, int src_linesize,
//                  int w, int h, int radius, int power, uint8_t *temp[2], int pixsize)
//{
//    int x;
//
//    if (radius == 0 && dst == src)
//        return;
//
//    for (x = 0; x < w; x++)
//        blur_power(dst + x*pixsize, dst_linesize, src + x*pixsize, src_linesize,
//                   h, radius, power, temp, pixsize);
//}

static int process_fs_frame(struct FFFrameSync *fs) {
    AVFilterContext *ctx = fs->parent;
    AudioblurContext *s = fs->opaque;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *videolink = ctx->inputs[VIDEO];
    AVFrame *out, *audio, *video;
    int ret, plane;

    // some values for boxblur
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(videolink->format);
    s->hsub = desc->log2_chroma_w;
    s->vsub = desc->log2_chroma_h;
    const int depth = desc->comp[0].depth;
    const int pixsize = (depth+7)/8;

    // TODO: make these properties on the filter?
    const int max_radius = 20;
    const int max_power = 20;

    if ((ret = ff_framesync_get_frame(&s->fs, AUDIO, &audio,   0)) < 0 ||
        (ret = ff_framesync_get_frame(&s->fs, VIDEO, &video, 0)) < 0)
        return ret;

    // more values for boxblur--can only initialize these after getting video frame from framesync
    int cw = AV_CEIL_RSHIFT(videolink->w, s->hsub), ch = AV_CEIL_RSHIFT(video->height, s->vsub);
    int w[4] = { videolink->w, cw, cw, videolink->w };
    int h[4] = { video->height, ch, ch, video->height };

    if (ctx->is_disabled) {
        out = av_frame_clone(video);
        if (!out)
            return AVERROR(ENOMEM);
    } else {
        // Set up out-link to copy properties from video in-link
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out, video);
        out->format = video->format;

        filter_audio_frame(ctx->inputs[AUDIO], audio);

        const int BASS_BANDS = 5;
        double sum = 0;
        for (int band = 0; band < BASS_BANDS; band++) {
            sum += s->heights[band];
        }
        double bass_average = sum / BASS_BANDS;

        int radius = (int) (max_radius * bass_average);
        int power = 3;//(int) (max_power * bass_average);

//        av_log(ctx, AV_LOG_INFO, "bass average was %0.2f\n", bass_average);

        av_assert0(video->width == out->width);
        av_assert0(video->height == out->height);

        // Allocate temp buffer for blur function if needed
        // This was done in config_input in vf_boxblur, so maybe move it to such a function here eventually?
        if (s->temp[0] == NULL || s->temp[1] == NULL)
            if (!(s->temp[0] = av_malloc(2*FFMAX(videolink->w, videolink->h))) ||
                !(s->temp[1] = av_malloc(2*FFMAX(videolink->w, videolink->h))))
                return AVERROR(ENOMEM);

        double blur_depth = av_clipd(bass_average * 3, 0, 1);
        for (plane = 0; plane < 1 && video->data[plane] && video->linesize[plane]; plane++) {
            depthblur(ctx, out->data[plane], out->linesize[plane], video->data[plane], video->linesize[plane], w[plane], h[plane], blur_depth, 5);
        }

        for (plane = 1; plane < 4 && video->data[plane] && video->linesize[plane]; plane++) {
            depthblur(ctx, out->data[plane], out->linesize[plane], video->data[plane], video->linesize[plane], w[plane], h[plane], 0, 1);
        }
//        for (plane = 0; plane < 4 && video->data[plane] && video->linesize[plane]; plane++) {
//
//            hblur(out->data[plane], out->linesize[plane],
//                  video->data[plane], video->linesize[plane],
//                  w[plane], h[plane], radius, power,
//                  s->temp, pixsize);
//        }
//        for (plane = 0; plane < 4 && video->data[plane] && video->linesize[plane]; plane++) {
//            vblur(out->data[plane], out->linesize[plane],
//                  out->data[plane], out->linesize[plane],
//                  w[plane], h[plane], radius, power,
//                  s->temp, pixsize);
//        }
    }
    out->pts = av_rescale_q(audio->pts, s->fs.time_base, outlink->time_base);

    return ff_filter_frame(outlink, out);
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioblurContext *s = ctx->priv;

    s->pts = AV_NOPTS_VALUE;

    return 0;
}

static int query_video_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
            AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA420P,
            AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ422P,AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ411P,
            AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
            AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP, AV_PIX_FMT_GRAY8,
            AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
            AV_PIX_FMT_YUVA420P16, AV_PIX_FMT_YUVA422P16, AV_PIX_FMT_YUVA444P16,
            AV_PIX_FMT_GBRP16, AV_PIX_FMT_GRAY16,
            AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static int query_audio_formats(AVFilterContext *ctx)
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

static int query_formats(AVFilterContext *ctx) {
    int ret;

    ret = query_video_formats(ctx);
    if (ret < 0)
        return ret;

    ret = query_audio_formats(ctx);
    if (ret < 0)
        return ret;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    int i;
    AudioblurContext *s = ctx->priv;

    ff_framesync_uninit(&s->fs);

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

    av_freep(&s->temp[0]);
    av_freep(&s->temp[1]);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *audiolink = ctx->inputs[AUDIO];
    AVFilterLink *videolink = ctx->inputs[VIDEO];
    AudioblurContext *s = ctx->priv;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(videolink->format);
    float overlap;
    FFFrameSyncIn *in;
    int ret, is_16bit, i;

    outlink->w = videolink->w;
    outlink->h = videolink->h;
    outlink->time_base = videolink->time_base;
    outlink->sample_aspect_ratio = videolink->sample_aspect_ratio;
    outlink->frame_rate = videolink->frame_rate;


    is_16bit = pix_desc->comp[0].depth == 16;
    if ((ret = ff_framesync_init(&s->fs, ctx, 2)) < 0)
        return ret;
    in = s->fs.in;
    in[0].time_base = audiolink->time_base;
    in[1].time_base = videolink->time_base;

    // make the audio and video both at the same sync level so they simulcast frame events
    in[0].sync = 1;
    in[1].sync = 1;

    // don't extend the audio stream at all, that's just unwanted, if there are no more audio frames left, we'll cut the stream
    in[0].before = EXT_STOP;
    in[0].after = EXT_STOP;

    // but do extend the video stream since it's prob just an image
    in[1].before = EXT_INFINITY;
    in[1].after = EXT_INFINITY;

    s->fs.opaque = s;
    s->fs.on_event = process_fs_frame;

    if ((ret = ff_framesync_configure(&s->fs)) < 0)
        return ret;

    // FFT stuff:
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

    /* FFT buffers: x2 for each (display) channel buffer.
     * Note: we use free and malloc instead of a realloc-like function to
     * make sure the buffer is aligned in memory for the FFT functions. */
    for (i = 0; i < s->nb_channels; i++) {
        av_freep(&s->fft_data[i]);
        av_freep(&s->avg_data[i]);
    }
    av_freep(&s->fft_data);
    av_freep(&s->avg_data);
    s->nb_channels = audiolink->channels;

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

//    outlink->frame_rate = av_make_q(audiolink->sample_rate, s->win_size * (1.-s->overlap));

    s->fifo = av_audio_fifo_alloc(audiolink->format, audiolink->channels, s->win_size);
    if (!s->fifo)
        return AVERROR(ENOMEM);

    return 0;
}

static int request_frame(AVFilterLink *outlink)
{
    AudioblurContext *s = outlink->src->priv;

    return ff_framesync_request_frame(&s->fs, outlink);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AudioblurContext *s = inlink->dst->priv;

    return ff_framesync_filter_frame(&s->fs, inlink, buf);
}

static const AVFilterPad audioblur_inputs[] = {
        {
                .name          = "audio",
                .type          = AVMEDIA_TYPE_AUDIO,
                .filter_frame  = filter_frame,
        },{
                .name          = "video",
                .type          = AVMEDIA_TYPE_VIDEO,
                .filter_frame  = filter_frame,
        },
        { NULL }
};

static const AVFilterPad audioblur_outputs[] = {
        {
                .name          = "default",
                .type          = AVMEDIA_TYPE_VIDEO,
                .config_props  = config_output,
                .request_frame = request_frame,
        },
        { NULL }
};

AVFilter ff_avf_audioblur = {
        .name = "audioblur",
        .description = NULL_IF_CONFIG_SMALL("Blur video frame based on the low-band frequencies of co-timed audio frames."),
        .init = init,
        .uninit = uninit,
        .priv_size = sizeof(AudioblurContext),
        .query_formats = query_formats,
        .inputs = audioblur_inputs,
        .outputs = audioblur_outputs,
        .priv_class = &audioblur_class,
        .flags = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL | AVFILTER_FLAG_SLICE_THREADS,
};