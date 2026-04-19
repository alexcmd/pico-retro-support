#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* ── display geometry ──────────────────────────────────────────── */
#define OLED_W          128
#define OLED_H          64
#define OLED_PAGES      (OLED_H / 8)   /* 8  */
#define OLED_BUF_LEN    (OLED_W * OLED_PAGES)  /* 1024 */

/* ── I2C config ────────────────────────────────────────────────── */
#define OLED_I2C        i2c0
#define OLED_SDA_PIN    4
#define OLED_SCL_PIN    5
#define OLED_I2C_FREQ   400000          /* 400 kHz fast-mode */
#define OLED_ADDR_AUTO  0xFF            /* scan at init        */

/* ── controller quirk ──────────────────────────────────────────── */
/* Define OLED_SH1106 to add the 2-column offset SH1106 needs      */
/* #define OLED_SH1106 */

/* ── init / control ────────────────────────────────────────────── */
bool    oled_init(void);     /* returns false if no display found  */
uint8_t oled_addr(void);     /* detected I2C address               */
void    oled_flush(void);    /* push framebuffer to display        */
void    oled_clear(void);    /* zero framebuffer (no flush)        */
void    oled_contrast(uint8_t c);
void    oled_invert(bool on);

/* ── drawing ────────────────────────────────────────────────────── */
void oled_pixel(int x, int y, bool on);
void oled_hline(int x, int y, int len, bool on);
void oled_vline(int x, int y, int len, bool on);
void oled_rect(int x, int y, int w, int h, bool fill, bool on);

/* Small font (5×8): every printable ASCII */
void oled_char(int x, int y, char c);
void oled_str(int x, int y, const char *s);
int  oled_str_w(const char *s);   /* pixel width */

/* Large font (10×16): 2× scaled, digits + common symbols only     */
void oled_char_lg(int x, int y, char c);
void oled_str_lg(int x, int y, const char *s);
int  oled_str_lg_w(const char *s);
