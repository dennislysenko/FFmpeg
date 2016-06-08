/*
 * Copyright (c) 2013 Paul B Mahol
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

enum CoeffBlendMode {
    COEFFBLEND_UNSET = -1,
    COEFFBLEND_NORMAL,
    COEFFBLEND_ADDITION,
    COEFFBLEND_AND,
    COEFFBLEND_AVERAGE,
    COEFFBLEND_BURN,
    COEFFBLEND_DARKEN,
    COEFFBLEND_DIFFERENCE,
    COEFFBLEND_DIFFERENCE128,
    COEFFBLEND_DIVIDE,
    COEFFBLEND_DODGE,
    COEFFBLEND_EXCLUSION,
    COEFFBLEND_HARDLIGHT,
    COEFFBLEND_LIGHTEN,
    COEFFBLEND_MULTIPLY,
    COEFFBLEND_NEGATION,
    COEFFBLEND_OR,
    COEFFBLEND_OVERLAY,
    COEFFBLEND_PHOENIX,
    COEFFBLEND_PINLIGHT,
    COEFFBLEND_REFLECT,
    COEFFBLEND_SCREEN,
    COEFFBLEND_SOFTLIGHT,
    COEFFBLEND_SUBTRACT,
    COEFFBLEND_VIVIDLIGHT,
    COEFFBLEND_XOR,
    COEFFBLEND_HARDMIX,
    COEFFBLEND_LINEARLIGHT,
    COEFFBLEND_GLOW,
    COEFFBLEND_ADDITION128,
    COEFFBLEND_MULTIPLY128,
    COEFFBLEND_HEAT,
    COEFFBLEND_FREEZE,
    COEFFBLEND_PARTIAL_MASK,
    COEFFBLEND_COLORED_MASK,
    COEFFBLEND_RIFF_ADD,
    COEFFBLEND_NB
};

typedef struct CoeffBlendParam {
    enum CoeffBlendMode mode;
    double opacity;
    AVExpr *e;
    char *expr_str;
    void (*blend)(const uint8_t *top, ptrdiff_t top_linesize,
                  const uint8_t *bottom, ptrdiff_t bottom_linesize,
                  uint8_t *dst, ptrdiff_t dst_linesize,
                  ptrdiff_t width, ptrdiff_t height,
                  struct CoeffBlendParam *param, double *values, int starty);
    double coeff;
    double blend_factor;
} CoeffBlendParam;

void ff_coeffblend_init(CoeffBlendParam *param, int is_16bit);

#define TOP    0
#define BOTTOM 1

typedef struct CoeffBlendContext {
    const AVClass *class;
    FFDualInputContext dinput;
    int hsub, vsub;             ///< chroma subsampling values
    int nb_planes;
    char *all_expr;
    enum CoeffBlendMode all_mode;
    double all_opacity;
    double all_blend_factor;

    CoeffBlendParam params[4];
    int tblend;
    AVFrame *prev_frame;        /* only used with tblend */
} CoeffBlendContext;

static const char *const var_names[] = {   "X",   "Y",   "W",   "H",   "SW",   "SH",   "T",   "N",   "A",   "B",   "TOP",   "BOTTOM",   "K",        NULL };
enum                                   { VAR_X, VAR_Y, VAR_W, VAR_H, VAR_SW, VAR_SH, VAR_T, VAR_N, VAR_A, VAR_B, VAR_TOP, VAR_BOTTOM, VAR_K, VAR_VARS_NB };

typedef struct ThreadData {
    const AVFrame *top, *bottom;
    AVFrame *dst;
    AVFilterLink *inlink;
    int plane;
    int w, h;
    CoeffBlendParam *param;
} ThreadData;

#define COMMON_OPTIONS \
    { "c0_mode", "set component #0 blend mode", OFFSET(params[0].mode), AV_OPT_TYPE_INT, {.i64=0}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c1_mode", "set component #1 blend mode", OFFSET(params[1].mode), AV_OPT_TYPE_INT, {.i64=0}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c2_mode", "set component #2 blend mode", OFFSET(params[2].mode), AV_OPT_TYPE_INT, {.i64=0}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c3_mode", "set component #3 blend mode", OFFSET(params[3].mode), AV_OPT_TYPE_INT, {.i64=0}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c0_coeff", "set component #0 coefficient (constant that is usable in custom expressions, suggested range [0,1])", OFFSET(params[0].coeff), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c1_coeff", "set component #1 coefficient (constant that is usable in custom expressions, suggested range [0,1])", OFFSET(params[1].coeff), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c2_coeff", "set component #2 coefficient (constant that is usable in custom expressions, suggested range [0,1])", OFFSET(params[2].coeff), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "c3_coeff", "set component #3 coefficient (constant that is usable in custom expressions, suggested range [0,1])", OFFSET(params[3].coeff), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "all_mode", "set blend mode for all components", OFFSET(all_mode), AV_OPT_TYPE_INT, {.i64=-1},-1, COEFFBLEND_NB-1, FLAGS, "mode"},\
    { "addition",   "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_ADDITION},   0, 0, FLAGS, "mode" },\
    { "addition128", "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_ADDITION128}, 0, 0, FLAGS, "mode" },\
    { "and",        "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_AND},        0, 0, FLAGS, "mode" },\
    { "average",    "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_AVERAGE},    0, 0, FLAGS, "mode" },\
    { "burn",       "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_BURN},       0, 0, FLAGS, "mode" },\
    { "darken",     "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_DARKEN},     0, 0, FLAGS, "mode" },\
    { "difference", "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_DIFFERENCE}, 0, 0, FLAGS, "mode" },\
    { "difference128", "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_DIFFERENCE128}, 0, 0, FLAGS, "mode" },\
    { "divide",     "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_DIVIDE},     0, 0, FLAGS, "mode" },\
    { "dodge",      "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_DODGE},      0, 0, FLAGS, "mode" },\
    { "exclusion",  "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_EXCLUSION},  0, 0, FLAGS, "mode" },\
    { "freeze",     "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_FREEZE},     0, 0, FLAGS, "mode" },\
    { "glow",       "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_GLOW},       0, 0, FLAGS, "mode" },\
    { "hardlight",  "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_HARDLIGHT},  0, 0, FLAGS, "mode" },\
    { "hardmix",    "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_HARDMIX},    0, 0, FLAGS, "mode" },\
    { "heat",       "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_HEAT},       0, 0, FLAGS, "mode" },\
    { "lighten",    "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_LIGHTEN},    0, 0, FLAGS, "mode" },\
    { "linearlight","", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_LINEARLIGHT},0, 0, FLAGS, "mode" },\
    { "multiply",   "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_MULTIPLY},   0, 0, FLAGS, "mode" },\
    { "multiply128","", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_MULTIPLY128},0, 0, FLAGS, "mode" },\
    { "negation",   "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_NEGATION},   0, 0, FLAGS, "mode" },\
    { "normal",     "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_NORMAL},     0, 0, FLAGS, "mode" },\
    { "or",         "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_OR},         0, 0, FLAGS, "mode" },\
    { "overlay",    "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_OVERLAY},    0, 0, FLAGS, "mode" },\
    { "phoenix",    "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_PHOENIX},    0, 0, FLAGS, "mode" },\
    { "pinlight",   "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_PINLIGHT},   0, 0, FLAGS, "mode" },\
    { "reflect",    "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_REFLECT},    0, 0, FLAGS, "mode" },\
    { "screen",     "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_SCREEN},     0, 0, FLAGS, "mode" },\
    { "softlight",  "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_SOFTLIGHT},  0, 0, FLAGS, "mode" },\
    { "subtract",   "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_SUBTRACT},   0, 0, FLAGS, "mode" },\
    { "vividlight", "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_VIVIDLIGHT}, 0, 0, FLAGS, "mode" },\
    { "xor",        "", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_XOR},        0, 0, FLAGS, "mode" },\
    { "partialmask", "using A as a mask, darkens B by 50% relative to A's white value", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_PARTIAL_MASK}, 0, 0, FLAGS, "mode" },\
    { "coloredmask",  "color-keys pure black pixels out of A and overlays the remainder on top of B", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_COLORED_MASK}, 0, 0, FLAGS, "mode" },\
    { "riffadd",  "color-matrixes A based on channel coefficients, but combines A and B proportionally based on the value of A (if 255, uses only A, if 0, uses only B). like a maskedmerge, with only two inputs", 0, AV_OPT_TYPE_CONST, {.i64=COEFFBLEND_RIFF_ADD}, 0, 0, FLAGS, "mode" },\
    { "c0_expr",  "set color component #0 expression", OFFSET(params[0].expr_str), AV_OPT_TYPE_STRING, {.str=NULL}, CHAR_MIN, CHAR_MAX, FLAGS },\
    { "c1_expr",  "set color component #1 expression", OFFSET(params[1].expr_str), AV_OPT_TYPE_STRING, {.str=NULL}, CHAR_MIN, CHAR_MAX, FLAGS },\
    { "c2_expr",  "set color component #2 expression", OFFSET(params[2].expr_str), AV_OPT_TYPE_STRING, {.str=NULL}, CHAR_MIN, CHAR_MAX, FLAGS },\
    { "c3_expr",  "set color component #3 expression", OFFSET(params[3].expr_str), AV_OPT_TYPE_STRING, {.str=NULL}, CHAR_MIN, CHAR_MAX, FLAGS },\
    { "all_expr", "set expression for all color components", OFFSET(all_expr), AV_OPT_TYPE_STRING, {.str=NULL}, CHAR_MIN, CHAR_MAX, FLAGS },\
    { "c0_opacity",  "set color component #0 opacity", OFFSET(params[0].opacity), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "c1_opacity",  "set color component #1 opacity", OFFSET(params[1].opacity), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "c2_opacity",  "set color component #2 opacity", OFFSET(params[2].opacity), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "c3_opacity",  "set color component #3 opacity", OFFSET(params[3].opacity), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "all_opacity", "set opacity for all color components", OFFSET(all_opacity), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS},\
    { "c0_blend_factor",  "set color component #0 blend_factor", OFFSET(params[0].blend_factor), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "c1_blend_factor",  "set color component #1 blend_factor", OFFSET(params[1].blend_factor), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "c2_blend_factor",  "set color component #2 blend_factor", OFFSET(params[2].blend_factor), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "c3_blend_factor",  "set color component #3 blend_factor", OFFSET(params[3].blend_factor), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },\
    { "all_blend_factor", "set blend_factor for all color components", OFFSET(all_blend_factor), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS}

#define OFFSET(x) offsetof(CoeffBlendContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption coeffblend_options[] = {
    COMMON_OPTIONS,
    { "shortest",    "force termination when the shortest input terminates", OFFSET(dinput.shortest), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "repeatlast",  "repeat last bottom frame", OFFSET(dinput.repeatlast), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(coeffblend);

#define COPY(src)                                                            \
static void blend_copy ## src(const uint8_t *top, ptrdiff_t top_linesize,    \
                            const uint8_t *bottom, ptrdiff_t bottom_linesize,\
                            uint8_t *dst, ptrdiff_t dst_linesize,            \
                            ptrdiff_t width, ptrdiff_t height,               \
                            CoeffBlendParam *param, double *values, int starty) \
{                                                                            \
    av_image_copy_plane(dst, dst_linesize, src, src ## _linesize,            \
                        width, height);                                 \
}

COPY(top)
COPY(bottom)

#undef COPY

static void blend_normal_8bit(const uint8_t *top, ptrdiff_t top_linesize,
                              const uint8_t *bottom, ptrdiff_t bottom_linesize,
                              uint8_t *dst, ptrdiff_t dst_linesize,
                              ptrdiff_t width, ptrdiff_t height,
                              CoeffBlendParam *param, double *values, int starty)
{
    const double opacity = param->opacity;
    int i, j;

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            dst[j] = top[j] * opacity + bottom[j] * (1. - opacity);
        }
        dst    += dst_linesize;
        top    += top_linesize;
        bottom += bottom_linesize;
    }
}

static void blend_normal_16bit(const uint8_t *_top, ptrdiff_t top_linesize,
                                  const uint8_t *_bottom, ptrdiff_t bottom_linesize,
                                  uint8_t *_dst, ptrdiff_t dst_linesize,
                                  ptrdiff_t width, ptrdiff_t height,
                                  CoeffBlendParam *param, double *values, int starty)
{
    const uint16_t *top = (uint16_t*)_top;
    const uint16_t *bottom = (uint16_t*)_bottom;
    uint16_t *dst = (uint16_t*)_dst;
    const double opacity = param->opacity;
    int i, j;
    dst_linesize /= 2;
    top_linesize /= 2;
    bottom_linesize /= 2;

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            dst[j] = top[j] * opacity + bottom[j] * (1. - opacity);
        }
        dst    += dst_linesize;
        top    += top_linesize;
        bottom += bottom_linesize;
    }
}

#define DEFINE_COEFFBLEND8(name, expr)                                              \
static void blend_## name##_8bit(const uint8_t *top, ptrdiff_t top_linesize,         \
                                 const uint8_t *bottom, ptrdiff_t bottom_linesize,   \
                                 uint8_t *dst, ptrdiff_t dst_linesize,               \
                                 ptrdiff_t width, ptrdiff_t height,                \
                                 CoeffBlendParam *param, double *values, int starty) \
{                                                                              \
    double opacity = param->opacity;                                           \
    int i, j;                                                                  \
                                                                               \
    for (i = 0; i < height; i++) {                                             \
        for (j = 0; j < width; j++) {                                          \
            dst[j] = top[j] + ((expr) - top[j]) * opacity;                     \
        }                                                                      \
        dst    += dst_linesize;                                                \
        top    += top_linesize;                                                \
        bottom += bottom_linesize;                                             \
    }                                                                          \
}

#define DEFINE_COEFFBLEND16(name, expr)                                             \
static void blend_## name##_16bit(const uint8_t *_top, ptrdiff_t top_linesize,       \
                                  const uint8_t *_bottom, ptrdiff_t bottom_linesize, \
                                  uint8_t *_dst, ptrdiff_t dst_linesize,             \
                                  ptrdiff_t width, ptrdiff_t height,           \
                                  CoeffBlendParam *param, double *values, int starty)         \
{                                                                              \
    const uint16_t *top = (uint16_t*)_top;                                     \
    const uint16_t *bottom = (uint16_t*)_bottom;                               \
    uint16_t *dst = (uint16_t*)_dst;                                           \
    double opacity = param->opacity;                                           \
    int i, j;                                                                  \
    dst_linesize /= 2;                                                         \
    top_linesize /= 2;                                                         \
    bottom_linesize /= 2;                                                      \
                                                                               \
    for (i = 0; i < height; i++) {                                             \
        for (j = 0; j < width; j++) {                                          \
            dst[j] = top[j] + ((expr) - top[j]) * opacity;                     \
        }                                                                      \
        dst    += dst_linesize;                                                \
        top    += top_linesize;                                                \
        bottom += bottom_linesize;                                             \
    }                                                                          \
}

#define A top[j]
#define B bottom[j]
#define K param->coeff
#define F param->blend_factor

#define MULTIPLY(x, a, b) ((x) * (((a) * (b)) / 255))
#define SCREEN(x, a, b)   (255 - (x) * ((255 - (a)) * (255 - (b)) / 255))
#define BURN(a, b)        (((a) == 0) ? (a) : FFMAX(0, 255 - ((255 - (b)) << 8) / (a)))
#define DODGE(a, b)       (((a) == 255) ? (a) : FFMIN(255, (((b) << 8) / (255 - (a)))))

DEFINE_COEFFBLEND8(addition,   FFMIN(255, A + B))
DEFINE_COEFFBLEND8(addition128, av_clip_uint8(A + B - 128))
DEFINE_COEFFBLEND8(average,    (A + B) / 2)
DEFINE_COEFFBLEND8(subtract,   FFMAX(0, A - B))
DEFINE_COEFFBLEND8(multiply,   MULTIPLY(1, A, B))
DEFINE_COEFFBLEND8(multiply128,av_clip_uint8((A - 128) * B / 32. + 128))
DEFINE_COEFFBLEND8(negation,   255 - FFABS(255 - A - B))
DEFINE_COEFFBLEND8(difference, FFABS(A - B))
DEFINE_COEFFBLEND8(difference128, av_clip_uint8(128 + A - B))
DEFINE_COEFFBLEND8(screen,     SCREEN(1, A, B))
DEFINE_COEFFBLEND8(overlay,    (A < 128) ? MULTIPLY(2, A, B) : SCREEN(2, A, B))
DEFINE_COEFFBLEND8(hardlight,  (B < 128) ? MULTIPLY(2, B, A) : SCREEN(2, B, A))
DEFINE_COEFFBLEND8(hardmix,    (A < (255 - B)) ? 0: 255)
DEFINE_COEFFBLEND8(heat,       (A == 0) ? 0 : 255 - FFMIN(((255 - B) * (255 - B)) / A, 255))
DEFINE_COEFFBLEND8(freeze,     (B == 0) ? 0 : 255 - FFMIN(((255 - A) * (255 - A)) / B, 255))
DEFINE_COEFFBLEND8(darken,     FFMIN(A, B))
DEFINE_COEFFBLEND8(lighten,    FFMAX(A, B))
DEFINE_COEFFBLEND8(divide,     av_clip_uint8(B == 0 ? 255 : 255 * A / B))
DEFINE_COEFFBLEND8(dodge,      DODGE(A, B))
DEFINE_COEFFBLEND8(burn,       BURN(A, B))
DEFINE_COEFFBLEND8(softlight,  (A > 127) ? B + (255 - B) * (A - 127.5) / 127.5 * (0.5 - fabs(B - 127.5) / 255): B - B * ((127.5 - A) / 127.5) * (0.5 - fabs(B - 127.5)/255))
DEFINE_COEFFBLEND8(exclusion,  A + B - 2 * A * B / 255)
DEFINE_COEFFBLEND8(pinlight,   (B < 128) ? FFMIN(A, 2 * B) : FFMAX(A, 2 * (B - 128)))
DEFINE_COEFFBLEND8(phoenix,    FFMIN(A, B) - FFMAX(A, B) + 255)
DEFINE_COEFFBLEND8(reflect,    (B == 255) ? B : FFMIN(255, (A * A / (255 - B))))
DEFINE_COEFFBLEND8(glow,       (A == 255) ? A : FFMIN(255, (B * B / (255 - A))))
DEFINE_COEFFBLEND8(and,        A & B)
DEFINE_COEFFBLEND8(or,         A | B)
DEFINE_COEFFBLEND8(xor,        A ^ B)
DEFINE_COEFFBLEND8(vividlight, (A < 128) ? BURN(2 * A, B) : DODGE(2 * (A - 128), B))
DEFINE_COEFFBLEND8(linearlight,av_clip_uint8((B < 128) ? B + 2 * A - 255 : B + 2 * (A - 128)))
//DEFINE_COEFFBLEND8(partialmask,((B / 255.0) * (A / 255.0 * 0.5 + 0.5)) * 255)
DEFINE_COEFFBLEND8(partialmask,(B * A / 255 * (0.75 - 0.5 * F) + B * K * (0.25 + 0.5 * F)))
DEFINE_COEFFBLEND8(coloredmask,(A == 0 ? B : A))
DEFINE_COEFFBLEND8(riffadd,    (A * F * K + (1 - A * F / 255.0) * B))

#undef MULTIPLY
#undef SCREEN
#undef BURN
#undef DODGE

#define MULTIPLY(x, a, b) ((x) * (((a) * (b)) / 65535))
#define SCREEN(x, a, b)   (65535 - (x) * ((65535 - (a)) * (65535 - (b)) / 65535))
#define BURN(a, b)        (((a) == 0) ? (a) : FFMAX(0, 65535 - ((65535 - (b)) << 16) / (a)))
#define DODGE(a, b)       (((a) == 65535) ? (a) : FFMIN(65535, (((b) << 16) / (65535 - (a)))))

DEFINE_COEFFBLEND16(addition,   FFMIN(65535, A + B))
DEFINE_COEFFBLEND16(addition128, av_clip_uint16(A + B - 32768))
DEFINE_COEFFBLEND16(average,    (A + B) / 2)
DEFINE_COEFFBLEND16(subtract,   FFMAX(0, A - B))
DEFINE_COEFFBLEND16(multiply,   MULTIPLY(1, A, B))
DEFINE_COEFFBLEND16(multiply128, av_clip_uint16((A - 32768) * B / 8192. + 32768))
DEFINE_COEFFBLEND16(negation,   65535 - FFABS(65535 - A - B))
DEFINE_COEFFBLEND16(difference, FFABS(A - B))
DEFINE_COEFFBLEND16(difference128, av_clip_uint16(32768 + A - B))
DEFINE_COEFFBLEND16(screen,     SCREEN(1, A, B))
DEFINE_COEFFBLEND16(overlay,    (A < 32768) ? MULTIPLY(2, A, B) : SCREEN(2, A, B))
DEFINE_COEFFBLEND16(hardlight,  (B < 32768) ? MULTIPLY(2, B, A) : SCREEN(2, B, A))
DEFINE_COEFFBLEND16(hardmix,    (A < (65535 - B)) ? 0: 65535)
DEFINE_COEFFBLEND16(heat,       (A == 0) ? 0 : 65535 - FFMIN(((65535 - B) * (65535 - B)) / A, 65535))
DEFINE_COEFFBLEND16(freeze,     (B == 0) ? 0 : 65535 - FFMIN(((65535 - A) * (65535 - A)) / B, 65535))
DEFINE_COEFFBLEND16(darken,     FFMIN(A, B))
DEFINE_COEFFBLEND16(lighten,    FFMAX(A, B))
DEFINE_COEFFBLEND16(divide,     av_clip_uint16(B == 0 ? 65535 : 65535 * A / B))
DEFINE_COEFFBLEND16(dodge,      DODGE(A, B))
DEFINE_COEFFBLEND16(burn,       BURN(A, B))
DEFINE_COEFFBLEND16(softlight,  (A > 32767) ? B + (65535 - B) * (A - 32767.5) / 32767.5 * (0.5 - fabs(B - 32767.5) / 65535): B - B * ((32767.5 - A) / 32767.5) * (0.5 - fabs(B - 32767.5)/65535))
DEFINE_COEFFBLEND16(exclusion,  A + B - 2 * A * B / 65535)
DEFINE_COEFFBLEND16(pinlight,   (B < 32768) ? FFMIN(A, 2 * B) : FFMAX(A, 2 * (B - 32768)))
DEFINE_COEFFBLEND16(phoenix,    FFMIN(A, B) - FFMAX(A, B) + 65535)
DEFINE_COEFFBLEND16(reflect,    (B == 65535) ? B : FFMIN(65535, (A * A / (65535 - B))))
DEFINE_COEFFBLEND16(glow,       (A == 65535) ? A : FFMIN(65535, (B * B / (65535 - A))))
DEFINE_COEFFBLEND16(and,        A & B)
DEFINE_COEFFBLEND16(or,         A | B)
DEFINE_COEFFBLEND16(xor,        A ^ B)
DEFINE_COEFFBLEND16(vividlight, (A < 32768) ? BURN(2 * A, B) : DODGE(2 * (A - 32768), B))
DEFINE_COEFFBLEND16(linearlight,av_clip_uint16((B < 32768) ? B + 2 * A - 65535 : B + 2 * (A - 32768)))
//DEFINE_COEFFBLEND16(partialmask,((B / 65535) * (A / 65535 * 0.5 + 0.5)) * 65535)
DEFINE_COEFFBLEND16(partialmask,(B * A / 65535 * (0.75 - 0.5 * F) + B * K * (0.25 + 0.5 * F)))
DEFINE_COEFFBLEND16(coloredmask,(A == 0 ? B : A))
DEFINE_COEFFBLEND16(riffadd,    (A * F * K + (1 - A * F / 65535.0) * B))

#define DEFINE_COEFFBLEND_EXPR(type, name, div)                                     \
static void blend_expr_## name(const uint8_t *_top, ptrdiff_t top_linesize,          \
                               const uint8_t *_bottom, ptrdiff_t bottom_linesize,    \
                               uint8_t *_dst, ptrdiff_t dst_linesize,                \
                               ptrdiff_t width, ptrdiff_t height,              \
                               CoeffBlendParam *param, double *values, int starty) \
{                                                                              \
    const type *top = (type*)_top;                                             \
    const type *bottom = (type*)_bottom;                                       \
    type *dst = (type*)_dst;                                                   \
    AVExpr *e = param->e;                                                      \
    int y, x;                                                                  \
    dst_linesize /= div;                                                       \
    top_linesize /= div;                                                       \
    bottom_linesize /= div;                                                    \
                                                                               \
    for (y = 0; y < height; y++) {                                             \
        values[VAR_Y] = y + starty;                                            \
        for (x = 0; x < width; x++) {                                          \
            values[VAR_X]      = x;                                            \
            values[VAR_TOP]    = values[VAR_A] = top[x];                       \
            values[VAR_BOTTOM] = values[VAR_B] = bottom[x];                    \
            values[VAR_K] = param->coeff;                                      \
            dst[x] = av_expr_eval(e, values, NULL);                            \
        }                                                                      \
        dst    += dst_linesize;                                                \
        top    += top_linesize;                                                \
        bottom += bottom_linesize;                                             \
    }                                                                          \
}

DEFINE_COEFFBLEND_EXPR(uint8_t, 8bit, 1)
DEFINE_COEFFBLEND_EXPR(uint16_t, 16bit, 2)

static int filter_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    int slice_start = (td->h *  jobnr   ) / nb_jobs;
    int slice_end   = (td->h * (jobnr+1)) / nb_jobs;
    int height      = slice_end - slice_start;
    const uint8_t *top    = td->top->data[td->plane];
    const uint8_t *bottom = td->bottom->data[td->plane];
    uint8_t *dst    = td->dst->data[td->plane];
    double values[VAR_VARS_NB];

    values[VAR_N]  = td->inlink->frame_count;
    values[VAR_T]  = td->dst->pts == AV_NOPTS_VALUE ? NAN : td->dst->pts * av_q2d(td->inlink->time_base);
    values[VAR_W]  = td->w;
    values[VAR_H]  = td->h;
    values[VAR_SW] = td->w / (double)td->dst->width;
    values[VAR_SH] = td->h / (double)td->dst->height;

    td->param->blend(top + slice_start * td->top->linesize[td->plane],
                     td->top->linesize[td->plane],
                     bottom + slice_start * td->bottom->linesize[td->plane],
                     td->bottom->linesize[td->plane],
                     dst + slice_start * td->dst->linesize[td->plane],
                     td->dst->linesize[td->plane],
                     td->w, height, td->param, &values[0], slice_start);
    return 0;
}

static AVFrame *blend_frame(AVFilterContext *ctx, AVFrame *top_buf,
                            const AVFrame *bottom_buf)
{
    CoeffBlendContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *dst_buf;
    int plane;

    dst_buf = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!dst_buf)
        return top_buf;
    av_frame_copy_props(dst_buf, top_buf);

    for (plane = 0; plane < s->nb_planes; plane++) {
        int hsub = plane == 1 || plane == 2 ? s->hsub : 0;
        int vsub = plane == 1 || plane == 2 ? s->vsub : 0;
        int outw = AV_CEIL_RSHIFT(dst_buf->width,  hsub);
        int outh = AV_CEIL_RSHIFT(dst_buf->height, vsub);
        CoeffBlendParam *param = &s->params[plane];
        ThreadData td = { .top = top_buf, .bottom = bottom_buf, .dst = dst_buf,
                          .w = outw, .h = outh, .param = param, .plane = plane,
                          .inlink = inlink };

        ctx->internal->execute(ctx, filter_slice, &td, NULL, FFMIN(outh, ctx->graph->nb_threads));
    }

    if (!s->tblend)
        av_frame_free(&top_buf);

    return dst_buf;
}

static av_cold int init(AVFilterContext *ctx)
{
    CoeffBlendContext *s = ctx->priv;

    s->tblend = !strcmp(ctx->filter->name, "tblend");

    s->dinput.process = blend_frame;
    return 0;
}

static int query_formats(AVFilterContext *ctx)
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

static av_cold void uninit(AVFilterContext *ctx)
{
    CoeffBlendContext *s = ctx->priv;
    int i;

    ff_dualinput_uninit(&s->dinput);
    av_frame_free(&s->prev_frame);

    for (i = 0; i < FF_ARRAY_ELEMS(s->params); i++)
        av_expr_free(s->params[i].e);
}

void ff_coeffblend_init(CoeffBlendParam *param, int is_16bit)
{
    switch (param->mode) {
    case COEFFBLEND_ADDITION:   param->blend = is_16bit ? blend_addition_16bit   : blend_addition_8bit;   break;
    case COEFFBLEND_ADDITION128: param->blend = is_16bit ? blend_addition128_16bit : blend_addition128_8bit; break;
    case COEFFBLEND_AND:        param->blend = is_16bit ? blend_and_16bit        : blend_and_8bit;        break;
    case COEFFBLEND_AVERAGE:    param->blend = is_16bit ? blend_average_16bit    : blend_average_8bit;    break;
    case COEFFBLEND_BURN:       param->blend = is_16bit ? blend_burn_16bit       : blend_burn_8bit;       break;
    case COEFFBLEND_DARKEN:     param->blend = is_16bit ? blend_darken_16bit     : blend_darken_8bit;     break;
    case COEFFBLEND_DIFFERENCE: param->blend = is_16bit ? blend_difference_16bit : blend_difference_8bit; break;
    case COEFFBLEND_DIFFERENCE128: param->blend = is_16bit ? blend_difference128_16bit: blend_difference128_8bit; break;
    case COEFFBLEND_DIVIDE:     param->blend = is_16bit ? blend_divide_16bit     : blend_divide_8bit;     break;
    case COEFFBLEND_DODGE:      param->blend = is_16bit ? blend_dodge_16bit      : blend_dodge_8bit;      break;
    case COEFFBLEND_EXCLUSION:  param->blend = is_16bit ? blend_exclusion_16bit  : blend_exclusion_8bit;  break;
    case COEFFBLEND_FREEZE:     param->blend = is_16bit ? blend_freeze_16bit     : blend_freeze_8bit;     break;
    case COEFFBLEND_GLOW:       param->blend = is_16bit ? blend_glow_16bit       : blend_glow_8bit;       break;
    case COEFFBLEND_HARDLIGHT:  param->blend = is_16bit ? blend_hardlight_16bit  : blend_hardlight_8bit;  break;
    case COEFFBLEND_HARDMIX:    param->blend = is_16bit ? blend_hardmix_16bit    : blend_hardmix_8bit;    break;
    case COEFFBLEND_HEAT:       param->blend = is_16bit ? blend_heat_16bit       : blend_heat_8bit;       break;
    case COEFFBLEND_LIGHTEN:    param->blend = is_16bit ? blend_lighten_16bit    : blend_lighten_8bit;    break;
    case COEFFBLEND_LINEARLIGHT:param->blend = is_16bit ? blend_linearlight_16bit: blend_linearlight_8bit;break;
    case COEFFBLEND_MULTIPLY:   param->blend = is_16bit ? blend_multiply_16bit   : blend_multiply_8bit;   break;
    case COEFFBLEND_MULTIPLY128:param->blend = is_16bit ? blend_multiply128_16bit: blend_multiply128_8bit;break;
    case COEFFBLEND_NEGATION:   param->blend = is_16bit ? blend_negation_16bit   : blend_negation_8bit;   break;
    case COEFFBLEND_NORMAL:     param->blend = param->opacity == 1 ? blend_copytop :
                                          param->opacity == 0 ? blend_copybottom :
                                          is_16bit ? blend_normal_16bit     : blend_normal_8bit;     break;
    case COEFFBLEND_OR:         param->blend = is_16bit ? blend_or_16bit         : blend_or_8bit;         break;
    case COEFFBLEND_OVERLAY:    param->blend = is_16bit ? blend_overlay_16bit    : blend_overlay_8bit;    break;
    case COEFFBLEND_PHOENIX:    param->blend = is_16bit ? blend_phoenix_16bit    : blend_phoenix_8bit;    break;
    case COEFFBLEND_PINLIGHT:   param->blend = is_16bit ? blend_pinlight_16bit   : blend_pinlight_8bit;   break;
    case COEFFBLEND_REFLECT:    param->blend = is_16bit ? blend_reflect_16bit    : blend_reflect_8bit;    break;
    case COEFFBLEND_SCREEN:     param->blend = is_16bit ? blend_screen_16bit     : blend_screen_8bit;     break;
    case COEFFBLEND_SOFTLIGHT:  param->blend = is_16bit ? blend_softlight_16bit  : blend_softlight_8bit;  break;
    case COEFFBLEND_SUBTRACT:   param->blend = is_16bit ? blend_subtract_16bit   : blend_subtract_8bit;   break;
    case COEFFBLEND_VIVIDLIGHT: param->blend = is_16bit ? blend_vividlight_16bit : blend_vividlight_8bit; break;
    case COEFFBLEND_XOR:        param->blend = is_16bit ? blend_xor_16bit        : blend_xor_8bit;        break;
    case COEFFBLEND_PARTIAL_MASK: param->blend = is_16bit ? blend_partialmask_16bit : blend_partialmask_8bit; break;
    case COEFFBLEND_COLORED_MASK: param->blend = is_16bit ? blend_coloredmask_16bit : blend_coloredmask_8bit; break;
    case COEFFBLEND_RIFF_ADD:   param->blend = is_16bit ? blend_riffadd_16bit : blend_riffadd_8bit; break;
    }

    if (param->opacity == 0 && param->mode != COEFFBLEND_NORMAL) {
        param->blend = blend_copytop;
    }
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *toplink = ctx->inputs[TOP];
    CoeffBlendContext *s = ctx->priv;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(toplink->format);
    int ret, plane, is_16bit;

    if (!s->tblend) {
        AVFilterLink *bottomlink = ctx->inputs[BOTTOM];

        if (toplink->format != bottomlink->format) {
            av_log(ctx, AV_LOG_ERROR, "inputs must be of same pixel format\n");
            return AVERROR(EINVAL);
        }
        if (toplink->w                       != bottomlink->w ||
            toplink->h                       != bottomlink->h ||
            toplink->sample_aspect_ratio.num != bottomlink->sample_aspect_ratio.num ||
            toplink->sample_aspect_ratio.den != bottomlink->sample_aspect_ratio.den) {
            av_log(ctx, AV_LOG_ERROR, "First input link %s parameters "
                   "(size %dx%d, SAR %d:%d) do not match the corresponding "
                   "second input link %s parameters (%dx%d, SAR %d:%d)\n",
                   ctx->input_pads[TOP].name, toplink->w, toplink->h,
                   toplink->sample_aspect_ratio.num,
                   toplink->sample_aspect_ratio.den,
                   ctx->input_pads[BOTTOM].name, bottomlink->w, bottomlink->h,
                   bottomlink->sample_aspect_ratio.num,
                   bottomlink->sample_aspect_ratio.den);
            return AVERROR(EINVAL);
        }
    }

    outlink->w = toplink->w;
    outlink->h = toplink->h;
    outlink->time_base = toplink->time_base;
    outlink->sample_aspect_ratio = toplink->sample_aspect_ratio;
    outlink->frame_rate = toplink->frame_rate;

    s->hsub = pix_desc->log2_chroma_w;
    s->vsub = pix_desc->log2_chroma_h;

    is_16bit = pix_desc->comp[0].depth == 16;
    s->nb_planes = av_pix_fmt_count_planes(toplink->format);

    if (!s->tblend)
        if ((ret = ff_dualinput_init(ctx, &s->dinput)) < 0)
            return ret;

    for (plane = 0; plane < FF_ARRAY_ELEMS(s->params); plane++) {
        CoeffBlendParam *param = &s->params[plane];

        if (s->all_mode >= 0)
            param->mode = s->all_mode;
        if (s->all_opacity < 1)
            param->opacity = s->all_opacity;
        if (s->all_blend_factor < 1)
            param->blend_factor = s->all_blend_factor;

        ff_coeffblend_init(param, is_16bit);

        if (s->all_expr && !param->expr_str) {
            param->expr_str = av_strdup(s->all_expr);
            if (!param->expr_str)
                return AVERROR(ENOMEM);
        }
        if (param->expr_str) {
            ret = av_expr_parse(&param->e, param->expr_str, var_names,
                                NULL, NULL, NULL, NULL, 0, ctx);
            if (ret < 0)
                return ret;
            param->blend = is_16bit? blend_expr_16bit : blend_expr_8bit;
        }
    }

    return 0;
}

static int request_frame(AVFilterLink *outlink)
{
    CoeffBlendContext *s = outlink->src->priv;
    return ff_dualinput_request_frame(&s->dinput, outlink);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    CoeffBlendContext *s = inlink->dst->priv;
    return ff_dualinput_filter_frame(&s->dinput, inlink, buf);
}

static const AVFilterPad coeffblend_inputs[] = {
    {
        .name          = "top",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
    },{
        .name          = "bottom",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
    },
    { NULL }
};

static const AVFilterPad coeffblend_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
        .request_frame = request_frame,
    },
    { NULL }
};

AVFilter ff_vf_coeffblend = {
    .name          = "coeffblend",
    .description   = NULL_IF_CONFIG_SMALL("CoeffBlend two video frames into each other."),
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(CoeffBlendContext),
    .query_formats = query_formats,
    .inputs        = coeffblend_inputs,
    .outputs       = coeffblend_outputs,
    .priv_class    = &coeffblend_class,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL | AVFILTER_FLAG_SLICE_THREADS,
};
