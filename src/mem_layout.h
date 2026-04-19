#pragma once
#include <stdint.h>
#include <string.h>

/* ── region types ────────────────────────────────────────────────── */
typedef enum {
    REGION_NONE = 0,  /* unmapped — reads 0xFF, writes ignored  */
    REGION_RAM,
    REGION_ROM,       /* write-protected                        */
    REGION_VIA,       /* VIA 6522 register window               */
    REGION_VIDEO,     /* video RAM — r/w like RAM + dirty-tracks */
} region_type_t;

/* Video system identifier — tells Pico 2 how to render the framebuffer */
typedef enum {
    VIDEO_NONE = 0,
    VIDEO_SPECTRUM,   /* ZX Spectrum: 256×192 bitmap + 32×24 attrs */
    VIDEO_ZX81,       /* ZX81: D-file variable-length display       */
    VIDEO_C64,        /* C64: char screen + color map               */
} video_sys_t;

#define MAX_REGIONS 8

typedef struct {
    uint16_t      base;
    uint16_t      size;
    region_type_t type;
    char          label[8];
} mem_region_t;

typedef struct {
    char         name[16];
    char         desc[64];
    uint8_t      n_regions;
    mem_region_t regions[MAX_REGIONS];  /* checked in order — first match wins */
    video_sys_t  sys;                   /* VIDEO_NONE for non-video layouts */
} mem_layout_t;

/* ── predefined layouts ──────────────────────────────────────────── */
static const mem_layout_t LAYOUTS[] = {

    /* ── generic: sensible default for any 6502 project ────────── */
    {
        "generic",
        "48KB RAM | VIA $C000 | ROM $C010-$FFFF",
        3, {
            { 0x0000, 0xC000, REGION_RAM, "RAM"  },
            { 0xC000, 0x0010, REGION_VIA, "VIA"  },
            { 0xC010, 0x3FF0, REGION_ROM, "ROM"  },
        }
    },

    /* ── apple1: Wozniak Apple I ────────────────────────────────── */
    {
        "apple1",
        "Apple I: RAM $0-$CFFF | PIA $D010 | BASIC $E000 | Monitor $FF00",
        4, {
            { 0x0000, 0xD000, REGION_RAM, "RAM"   },
            { 0xD010, 0x0010, REGION_VIA, "PIA"   },  /* 6820 PIA via VIA stub */
            { 0xE000, 0x1000, REGION_ROM, "BASIC" },
            { 0xFF00, 0x0100, REGION_ROM, "MON"   },  /* Woz monitor           */
        }
    },

    /* ── c64: Commodore 64 (simplified) ────────────────────────── */
    {
        "c64",
        "C64: 40KB RAM | BASIC $A000 | CIA $DC00 | KERNAL $E000",
        4, {
            { 0x0000, 0xA000, REGION_RAM, "RAM"    },
            { 0xA000, 0x2000, REGION_ROM, "BASIC"  },
            { 0xDC00, 0x0010, REGION_VIA, "CIA"    },  /* CIA 1 via VIA stub    */
            { 0xE000, 0x2000, REGION_ROM, "KERNAL" },
        }
    },

    /* ── bbc: BBC Micro ─────────────────────────────────────────── */
    /* VIA listed before ROM so it wins within the OS ROM range     */
    {
        "bbc",
        "BBC: 32KB RAM | SWROM $8000 | sysVIA $FE40 | OS $C000",
        4, {
            { 0x0000, 0x8000, REGION_RAM, "RAM"   },
            { 0xFE40, 0x0010, REGION_VIA, "sVIA"  },  /* system VIA (priority) */
            { 0x8000, 0x4000, REGION_ROM, "SWROM" },
            { 0xC000, 0x4000, REGION_ROM, "OS"    },
        }
    },

    /* ── zx81: Sinclair ZX81 ────────────────────────────────────── */
    {
        "zx81",
        "ZX81: ROM $0000-$1FFF | RAM $4000-$FFFF (Z80 layout)",
        2, {
            { 0x0000, 0x2000, REGION_ROM, "ROM" },
            { 0x4000, 0xC000, REGION_RAM, "RAM" },
        }
    },

    /* ── spectrum: ZX Spectrum 48K ──────────────────────────────── */
    /* $4000-$5AFF = 6912B display file (bitmap + attrs) → video RAM */
    {
        "spectrum",
        "Spectrum 48K: ROM $0-$3FFF | VRAM $4000-$5AFF | RAM $5B00-$FFFF",
        3, {
            { 0x0000, 0x4000, REGION_ROM,   "ROM"  },
            { 0x4000, 0x1B00, REGION_VIDEO, "VRAM" },
            { 0x5B00, 0xA500, REGION_RAM,   "RAM"  },
        },
        VIDEO_SPECTRUM
    },

    /* ── minimal: small ROM + maximum RAM ──────────────────────── */
    {
        "minimal",
        "Minimal: 60KB RAM | VIA $F000 | 4KB ROM $F010-$FFFF",
        3, {
            { 0x0000, 0xF000, REGION_RAM, "RAM" },
            { 0xF000, 0x0010, REGION_VIA, "VIA" },
            { 0xF010, 0x0FF0, REGION_ROM, "ROM" },
        }
    },
};

#define N_LAYOUTS (int)(sizeof(LAYOUTS)/sizeof(LAYOUTS[0]))

/* ── helpers ─────────────────────────────────────────────────────── */
static inline region_type_t layout_region_for(const mem_layout_t *l, uint16_t addr) {
    for (int i = 0; i < l->n_regions; i++) {
        const mem_region_t *r = &l->regions[i];
        if (addr >= r->base && (uint32_t)addr < (uint32_t)r->base + r->size)
            return r->type;
    }
    return REGION_NONE;
}

static inline const char *layout_label_for(const mem_layout_t *l, uint16_t addr) {
    for (int i = 0; i < l->n_regions; i++) {
        const mem_region_t *r = &l->regions[i];
        if (addr >= r->base && (uint32_t)addr < (uint32_t)r->base + r->size)
            return r->label;
    }
    return "----";
}

static inline const char *region_type_str(region_type_t t) {
    switch (t) {
    case REGION_RAM:   return "RAM";
    case REGION_ROM:   return "ROM";
    case REGION_VIA:   return "VIA";
    case REGION_VIDEO: return "VIDEO";
    default:           return "---";
    }
}

/* Parse one layout from UART lines (call repeatedly until returns false).
   Format per line:  <type> <base_hex> <size_hex>
   Type tokens: ram rom via
   Returns true while more lines expected, false on "end" or error.      */
static inline bool layout_parse_line(mem_layout_t *l, const char *line) {
    while (*line == ' ') line++;
    if (!*line || line[0] == '#') return true;   /* blank / comment — keep going */

    if (strncmp(line, "end",  3) == 0) return false;
    if (strncmp(line, "name ", 5) == 0) {
        strncpy(l->name, line+5, sizeof(l->name)-1);
        /* strip trailing whitespace */
        for (int i=(int)strlen(l->name)-1; i>=0&&l->name[i]<=' '; i--) l->name[i]='\0';
        return true;
    }
    if (strncmp(line, "desc ", 5) == 0) {
        strncpy(l->desc, line+5, sizeof(l->desc)-1);
        return true;
    }

    if (strncmp(line, "sys ", 4) == 0) {
        const char *s = line + 4;
        while (*s == ' ') s++;
        if      (strncmp(s, "spectrum", 8) == 0) l->sys = VIDEO_SPECTRUM;
        else if (strncmp(s, "zx81",     4) == 0) l->sys = VIDEO_ZX81;
        else if (strncmp(s, "c64",      3) == 0) l->sys = VIDEO_C64;
        return true;
    }

    region_type_t type;
    uint8_t       tlen;
    if      (strncmp(line, "ram",   3) == 0) { type = REGION_RAM;   tlen = 3; }
    else if (strncmp(line, "rom",   3) == 0) { type = REGION_ROM;   tlen = 3; }
    else if (strncmp(line, "via",   3) == 0) { type = REGION_VIA;   tlen = 3; }
    else if (strncmp(line, "video", 5) == 0) { type = REGION_VIDEO; tlen = 5; }
    else return true;   /* unknown token — skip */

    if (l->n_regions >= MAX_REGIONS) return true;

    char *p = (char *)line + tlen;
    uint16_t base = (uint16_t)strtoul(p, &p, 16);
    uint16_t size = (uint16_t)strtoul(p, &p, 16);
    if (!size) return true;   /* malformed */

    mem_region_t *r = &l->regions[l->n_regions++];
    r->base  = base;
    r->size  = size;
    r->type  = type;
    /* auto-label from type + index */
    const char *ts = region_type_str(type);
    int idx = 0;
    for (int i = 0; i < l->n_regions-1; i++)
        if (l->regions[i].type == type) idx++;
    snprintf(r->label, sizeof(r->label), idx ? "%s%d" : "%s", ts, idx);
    return true;
}
