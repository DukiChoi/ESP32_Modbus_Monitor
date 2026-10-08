#include "lv_app_font.h"
#include <string.h>

static const uint8_t micro_bitmap[] = {
    0x85, 0x0a, 0x14, 0x28, 0x50, 0xa1, 0x9a, 0xdb, 0x02, 0x04, 0x00
};

static const lv_font_t *font_for(uint32_t letter)
{
    if ((letter >= 0xAC00 && letter <= 0xD7A3) ||
        (letter >= 0x1100 && letter <= 0x11FF) ||
        (letter >= 0x3130 && letter <= 0x318F)) return &lv_font_galmuri9;
    return &lv_font_montserrat_16;
}

static bool glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *out,
                       uint32_t letter, uint32_t next)
{
    (void)font;
    if (letter == 0xB5) {
        memset(out, 0, sizeof(*out));
        out->adv_w = 8; out->box_w = 7; out->box_h = 11;
        out->ofs_y = -2; out->bpp = 1;
        return true;
    }
    const lv_font_t *base = font_for(letter);
    return lv_font_get_glyph_dsc(base, out, letter, font_for(next) == base ? next : 0);
}

static const uint8_t *glyph_bitmap(const lv_font_t *font, uint32_t letter)
{
    (void)font;
    if (letter == 0xB5) return micro_bitmap;
    return lv_font_get_glyph_bitmap(font_for(letter), letter);
}

const lv_font_t lv_app_font = {
    .get_glyph_dsc = glyph_dsc,
    .get_glyph_bitmap = glyph_bitmap,
    .line_height = 22,
    .base_line = 5,
    .subpx = LV_FONT_SUBPX_NONE,
};
