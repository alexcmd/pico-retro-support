#include <string.h>
#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "oled.h"
#include "font5x7.h"

/* ── internal state ─────────────────────────────────────────────── */
static uint8_t  s_addr  = 0x3C;
static uint8_t  s_buf[OLED_BUF_LEN];   /* framebuffer: 128×8 pages */

/* ── low-level I2C helpers ──────────────────────────────────────── */
static void cmd(uint8_t c) {
    uint8_t buf[2] = { 0x00, c };
    i2c_write_blocking(OLED_I2C, s_addr, buf, 2, false);
}

static void cmd2(uint8_t c0, uint8_t c1) {
    uint8_t buf[3] = { 0x00, c0, c1 };
    i2c_write_blocking(OLED_I2C, s_addr, buf, 3, false);
}

/* ── I2C scan ───────────────────────────────────────────────────── */
static uint8_t i2c_scan(void) {
    /* Common OLED addresses in order of likelihood */
    static const uint8_t candidates[] = { 0x3C, 0x3D, 0x78 >> 1, 0x7A >> 1 };
    uint8_t dummy;
    for (size_t i = 0; i < sizeof(candidates); i++) {
        if (i2c_read_blocking(OLED_I2C, candidates[i], &dummy, 1, false) >= 0)
            return candidates[i];
    }
    /* Full scan fallback */
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_read_blocking(OLED_I2C, addr, &dummy, 1, false) >= 0)
            return addr;
    }
    return 0;
}

/* ── SSD1306 init sequence ──────────────────────────────────────── */
static const uint8_t INIT_SEQ[] = {
    0xAE,               /* display off                */
    0xD5, 0x80,         /* clock divide ratio         */
    0xA8, 0x3F,         /* multiplex 1/64             */
    0xD3, 0x00,         /* display offset = 0         */
    0x40,               /* start line = 0             */
    0x8D, 0x14,         /* charge pump on             */
    0x20, 0x00,         /* horizontal addressing mode */
    0xA1,               /* seg remap: col127→SEG0     */
    0xC8,               /* COM scan: remapped         */
    0xDA, 0x12,         /* COM pins config            */
    0x81, 0xCF,         /* contrast                   */
    0xD9, 0xF1,         /* pre-charge period          */
    0xDB, 0x40,         /* VCOMH level                */
    0xA4,               /* output follows RAM         */
    0xA6,               /* normal (non-inverted)      */
    0xAF,               /* display on                 */
};

bool oled_init(void) {
    i2c_init(OLED_I2C, OLED_I2C_FREQ);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    sleep_ms(10);   /* power-on stabilise */

    s_addr = i2c_scan();
    if (s_addr == 0) {
        printf("[oled] no device found on I2C%d (SDA=GP%d SCL=GP%d)\r\n",
               i2c_hw_index(OLED_I2C), OLED_SDA_PIN, OLED_SCL_PIN);
        return false;
    }
    printf("[oled] found at 0x%02X\r\n", s_addr);

    for (size_t i = 0; i < sizeof(INIT_SEQ); i++)
        cmd(INIT_SEQ[i]);

    oled_clear();
    oled_flush();
    return true;
}

uint8_t oled_addr(void) { return s_addr; }

void oled_contrast(uint8_t c) { cmd2(0x81, c); }

void oled_invert(bool on) { cmd(on ? 0xA7 : 0xA6); }

/* ── flush framebuffer ──────────────────────────────────────────── */
void oled_flush(void) {
#ifdef OLED_SH1106
    /* SH1106: page-by-page writes with 2-column offset */
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        uint8_t buf[OLED_W + 1];
        /* set page address */
        cmd(0xB0 | page);
        /* set column (SH1106 starts at col 2) */
        cmd(0x02);          /* lower nibble  */
        cmd(0x10);          /* upper nibble  */
        buf[0] = 0x40;      /* data mode */
        memcpy(buf + 1, s_buf + page * OLED_W, OLED_W);
        i2c_write_blocking(OLED_I2C, s_addr, buf, OLED_W + 1, false);
    }
#else
    /* SSD1306: one shot in horizontal mode.
       Set Column Address (0x21) and Set Page Address (0x22) each require
       start+end in the same I2C transaction — separate cmd() calls would
       let 0x7F / 0x07 be parsed as unrelated commands (0x7F = "Set Display
       Start Line 63", visibly corrupting the display). */
    uint8_t col_addr[]  = { 0x00, 0x21, 0x00, 0x7F };
    uint8_t page_addr[] = { 0x00, 0x22, 0x00, 0x07 };
    i2c_write_blocking(OLED_I2C, s_addr, col_addr,  4, false);
    i2c_write_blocking(OLED_I2C, s_addr, page_addr, 4, false);

    /* send in 64-byte chunks to stay within I2C buffer limits */
    uint8_t chunk[65];
    chunk[0] = 0x40;
    for (int off = 0; off < OLED_BUF_LEN; off += 64) {
        memcpy(chunk + 1, s_buf + off, 64);
        i2c_write_blocking(OLED_I2C, s_addr, chunk, 65, false);
    }
#endif
}

void oled_clear(void) { memset(s_buf, 0, OLED_BUF_LEN); }

/* ── pixel primitives ───────────────────────────────────────────── */
void oled_pixel(int x, int y, bool on) {
    if ((unsigned)x >= OLED_W || (unsigned)y >= OLED_H) return;
    uint8_t *b = &s_buf[(y / 8) * OLED_W + x];
    if (on) *b |=  (1 << (y & 7));
    else    *b &= ~(1 << (y & 7));
}

void oled_hline(int x, int y, int len, bool on) {
    for (int i = 0; i < len; i++) oled_pixel(x + i, y, on);
}

void oled_vline(int x, int y, int len, bool on) {
    for (int i = 0; i < len; i++) oled_pixel(x, y + i, on);
}

void oled_rect(int x, int y, int w, int h, bool fill, bool on) {
    if (fill) {
        for (int row = 0; row < h; row++)
            oled_hline(x, y + row, w, on);
    } else {
        oled_hline(x,     y,         w, on);
        oled_hline(x,     y + h - 1, w, on);
        oled_vline(x,         y, h, on);
        oled_vline(x + w - 1, y, h, on);
    }
}

/* ── small font (5×8) ───────────────────────────────────────────── */
#define FONT_W   6   /* 5 pixels + 1 gap */
#define FONT_H   8

void oled_char(int x, int y, char c) {
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *col = FONT5X7[(uint8_t)(c - 0x20)];
    for (int cx = 0; cx < 5; cx++) {
        uint8_t bits = col[cx];
        for (int row = 0; row < 7; row++)
            oled_pixel(x + cx, y + row, (bits >> row) & 1);
    }
    /* gap column */
    for (int row = 0; row < FONT_H; row++)
        oled_pixel(x + 5, y + row, false);
}

void oled_str(int x, int y, const char *s) {
    while (*s) { oled_char(x, y, *s++); x += FONT_W; }
}

int oled_str_w(const char *s) {
    int n = 0;
    while (*s++) n++;
    return n * FONT_W;
}

/* ── large font (10×16): 2× scaled ─────────────────────────────── */
#define FONT_LG_W  12  /* 10 pixels + 2 gap */
#define FONT_LG_H  16

void oled_char_lg(int x, int y, char c) {
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *col = FONT5X7[(uint8_t)(c - 0x20)];
    for (int cx = 0; cx < 5; cx++) {
        uint8_t bits = col[cx];
        for (int row = 0; row < 7; row++) {
            bool on = (bits >> row) & 1;
            /* 2× scale: each pixel → 2×2 block */
            oled_pixel(x + cx*2,     y + row*2,     on);
            oled_pixel(x + cx*2 + 1, y + row*2,     on);
            oled_pixel(x + cx*2,     y + row*2 + 1, on);
            oled_pixel(x + cx*2 + 1, y + row*2 + 1, on);
        }
    }
}

void oled_str_lg(int x, int y, const char *s) {
    while (*s) { oled_char_lg(x, y, *s++); x += FONT_LG_W; }
}

int oled_str_lg_w(const char *s) {
    int n = 0;
    while (*s++) n++;
    return n * FONT_LG_W;
}
