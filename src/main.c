/*
 * RP2350 — retro CPU clock + 64 KB memory + VIA 6522 emulator
 *
 * Core 0 : clock generator (GP2/PHI2), UART command shell
 * Core 1 : bus emulator — PHI2-synchronised tight loop, no interrupts
 *
 * ── Pin map ──────────────────────────────────────────────────────────
 *  GP0        UART0 TX  (stdio)
 *  GP1        UART0 RX  (stdio)
 *  GP2        PHI2 out  (PWM ≥9 Hz or timer toggle)
 *  GP3–GP18   A0–A15    address bus     (input from CPU)
 *  GP19–GP26  D0–D7     data bus        (bidirectional, contiguous)
 *  GP27       R/W       HIGH=read LOW=write (input from CPU)
 *  GP28       /IRQ      active-low out  (RP2350 → CPU)
 *  GP29       /RESET    active-low out  (RP2350 → CPU, release to run)
 *
 * ── Memory map ───────────────────────────────────────────────────────
 *  Runtime-configurable via 'layout' UART command.
 *  Default (generic): RAM $0000-$BFFF | VIA $C000 | ROM $C010-$FFFF
 *  Predefined layouts: generic apple1 c64 bbc zx81 minimal
 *  Custom layout: 'layout load' then paste lines, 'end' to finish.
 *
 * ── VIA 6522 registers ───────────────────────────────────────────────
 *  $00 ORB   $01 ORA   $02 DDRB  $03 DDRA
 *  $04 T1CL  $05 T1CH  $06 T1LL  $07 T1LH
 *  $08 T2CL  $09 T2CH  $0A SR    $0B ACR
 *  $0C PCR   $0D IFR   $0E IER   $0F ORA2
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "hardware/uart.h"
#include "uart_log.h"
#include "mem_layout.h"

/* ── pins ─────────────────────────────────────────────────────────── */
#define CLK_PIN      2
#define ADDR_PIN0    3
#define ADDR_BITS   16
#define DATA_PIN0   19
#define DATA_BITS    8
#define RW_PIN      27   /* input:  HIGH = CPU reading, LOW = CPU writing */
#define IRQ_PIN     28   /* output: drive LOW to interrupt CPU            */
#define RST_PIN     29   /* output: drive LOW to reset CPU                */

#define ADDR_MASK   (0xFFFFu   << ADDR_PIN0)
#define DATA_MASK   (0xFFu     << DATA_PIN0)

/* ── memory ───────────────────────────────────────────────────────── */
#define MEM_SIZE  0x10000u

static uint8_t     s_mem[MEM_SIZE];    /* flat 64 KB backing store */
static mem_layout_t s_layout;          /* active layout (Core 1 reads this) */

/* ── VIA 6522 state ───────────────────────────────────────────────── */
/* IFR / IER bit positions */
#define VIA_IRQ_T2  (1u << 5)
#define VIA_IRQ_T1  (1u << 6)
#define VIA_IRQ_ANY (1u << 7)   /* set by hw when any enabled flag set */

typedef struct {
    uint8_t  orb,  ora;     /* output registers          */
    uint8_t  ddrb, ddra;    /* data-direction registers   */
    uint16_t t1_ctr;        /* timer 1 counter            */
    uint16_t t1_latch;      /* timer 1 latch              */
    uint16_t t2_ctr;        /* timer 2 counter (one-shot) */
    uint8_t  t2_latch_lo;
    uint8_t  sr, acr, pcr;  /* shift-reg, aux-ctrl, periph-ctrl (stub) */
    uint8_t  ifr, ier;      /* interrupt flag / enable    */
    bool     t1_running;
    bool     t2_running;
} via_t;

static via_t s_via;

/* ── video link (UART0 GP0/GP1 @ 2Mbaud → Pico 2) ────────────────── */
#define VIDEO_UART        uart0
#define VIDEO_UART_TX_PIN 0
#define VIDEO_UART_RX_PIN 1
#define VIDEO_UART_BAUD   2000000

/* Spectrum display: 192 scanlines × 32 bytes + 24 attr rows × 32 bytes */
#define VID_LINES         192
#define VID_ATTR_ROWS      24
#define VID_PIXEL_BYTES   6144   /* 192×32 */
#define VID_ATTR_BYTES     768   /* 24×32  */
#define VID_TOTAL_BYTES   6912

/* dirty tracking: bit per scanline (192 bits = 6×uint32) and per attr row (24 bits) */
static volatile uint32_t s_dirty_lines[6];
static volatile uint32_t s_dirty_attrs;
static volatile uint16_t g_video_base = 0xFFFF;  /* address of VIDEO region, or 0xFFFF */

static uint32_t s_video_frame = 0;
static uint8_t  s_vid_build[7300];   /* frame build buffer (worst-case: 7163 bytes) */

/* ── bus transaction trace ────────────────────────────────────────── */
#define TRACE_CAP 512

typedef struct { uint8_t is_write; uint8_t data; uint16_t addr; } trace_t;

static trace_t         s_trace[TRACE_CAP];
static volatile uint32_t s_trace_wp     = 0;
static volatile uint32_t s_total_reads  = 0;
static volatile uint32_t s_total_writes = 0;

/* ── VIA register access ──────────────────────────────────────────── */
static uint8_t via_read(via_t *v, uint8_t reg) {
    switch (reg & 0x0F) {
    case 0x0: return (v->orb  & v->ddrb) | (0xFF & ~v->ddrb); /* ORB  */
    case 0x1: return (v->ora  & v->ddra) | (0xFF & ~v->ddra); /* ORA  */
    case 0x2: return v->ddrb;
    case 0x3: return v->ddra;
    case 0x4: v->ifr &= ~VIA_IRQ_T1; return (uint8_t)(v->t1_ctr & 0xFF); /* T1CL, clears flag */
    case 0x5: return (uint8_t)(v->t1_ctr >> 8);
    case 0x6: return (uint8_t)(v->t1_latch & 0xFF);
    case 0x7: return (uint8_t)(v->t1_latch >> 8);
    case 0x8: v->ifr &= ~VIA_IRQ_T2; return (uint8_t)(v->t2_ctr & 0xFF); /* T2CL, clears flag */
    case 0x9: return (uint8_t)(v->t2_ctr >> 8);
    case 0xA: return v->sr;
    case 0xB: return v->acr;
    case 0xC: return v->pcr;
    case 0xD: /* IFR: bit 7 set if any enabled flag */
        return v->ifr | ((v->ifr & v->ier & 0x7F) ? VIA_IRQ_ANY : 0);
    case 0xE: return v->ier | 0x80;   /* bit 7 always reads 1 */
    case 0xF: return (v->ora & v->ddra) | (0xFF & ~v->ddra);
    }
    return 0xFF;
}

static void via_write(via_t *v, uint8_t reg, uint8_t val) {
    switch (reg & 0x0F) {
    case 0x0: v->orb = val; break;
    case 0x1: case 0xF: v->ora = val; break;
    case 0x2: v->ddrb = val; break;
    case 0x3: v->ddra = val; break;
    case 0x4: case 0x6: v->t1_latch = (v->t1_latch & 0xFF00) | val; break;           /* T1LL */
    case 0x5:                                                                           /* T1CH: load & start */
        v->t1_latch  = (v->t1_latch & 0x00FF) | ((uint16_t)val << 8);
        v->t1_ctr    = v->t1_latch;
        v->ifr      &= ~VIA_IRQ_T1;
        v->t1_running = true;
        break;
    case 0x7: v->t1_latch = (v->t1_latch & 0x00FF) | ((uint16_t)val << 8); break;    /* T1LH */
    case 0x8: v->t2_latch_lo = val; break;                                             /* T2LL */
    case 0x9:                                                                           /* T2CH: load & start one-shot */
        v->t2_ctr    = ((uint16_t)val << 8) | v->t2_latch_lo;
        v->ifr      &= ~VIA_IRQ_T2;
        v->t2_running = true;
        break;
    case 0xA: v->sr  = val; break;
    case 0xB: v->acr = val; break;
    case 0xC: v->pcr = val; break;
    case 0xD: v->ifr &= ~(val & 0x7F); break;                /* write 1 to clear flag */
    case 0xE:                                                  /* set/clear enable bits */
        if (val & 0x80) v->ier |=  (val & 0x7F);
        else            v->ier &= ~(val & 0x7F);
        break;
    }
}

/* ── VIA tick: called once per PHI2 cycle ────────────────────────── */
static void via_tick(via_t *v) {
    if (v->t1_running) {
        v->t1_ctr--;
        if (v->t1_ctr == 0xFFFF) {   /* wrapped through 0 */
            v->ifr |= VIA_IRQ_T1;
            if (v->acr & 0x40) {      /* continuous mode */
                v->t1_ctr = v->t1_latch;
            } else {
                v->t1_running = false;
            }
        }
    }
    if (v->t2_running) {
        v->t2_ctr--;
        if (v->t2_ctr == 0xFFFF) {
            v->ifr |= VIA_IRQ_T2;
            v->t2_running = false;    /* always one-shot */
        }
    }
}

/* ── bus helpers ──────────────────────────────────────────────────── */
static uint8_t bus_read(uint16_t addr) {
    switch (layout_region_for(&s_layout, addr)) {
    case REGION_VIA:   return via_read(&s_via, (uint8_t)(addr & 0x0F));
    case REGION_VIDEO:
    case REGION_RAM:
    case REGION_ROM:   return s_mem[addr];
    default:           return 0xFF;   /* unmapped */
    }
}

static void bus_write(uint16_t addr, uint8_t data) {
    switch (layout_region_for(&s_layout, addr)) {
    case REGION_VIA:
        via_write(&s_via, (uint8_t)(addr & 0x0F), data);
        break;
    case REGION_VIDEO: {
        s_mem[addr] = data;
        uint16_t vb = g_video_base;
        if (vb != 0xFFFF) {
            uint16_t off = addr - vb;
            if (off < VID_PIXEL_BYTES) {
                /* Spectrum: addr encodes {block[1:0], pixel_row[2:0], char_row[2:0], col[4:0]} */
                uint8_t block     = (uint8_t)(off >> 11);
                uint8_t pixel_row = (uint8_t)((off >> 8) & 7);
                uint8_t char_row  = (uint8_t)((off >> 5) & 7);
                uint8_t line      = (block << 6) | (char_row << 3) | pixel_row;
                __atomic_fetch_or(&s_dirty_lines[line >> 5], 1u << (line & 31), __ATOMIC_RELAXED);
            } else if (off < VID_TOTAL_BYTES) {
                uint8_t row = (uint8_t)((off - VID_PIXEL_BYTES) >> 5);
                if (row < VID_ATTR_ROWS)
                    __atomic_fetch_or(&s_dirty_attrs, 1u << row, __ATOMIC_RELAXED);
            }
        }
        break;
    }
    case REGION_RAM: s_mem[addr] = data; break;
    case REGION_ROM: break;
    default:         break;
    }
}

static void update_irq(void) {
    bool assert = (s_via.ifr & s_via.ier & 0x7F) != 0;
    gpio_put(IRQ_PIN, !assert);   /* active low */
}

/* ── Core 1: bus emulator ─────────────────────────────────────────── */
static void core1_entry(void) {
    multicore_lockout_victim_init();

    /* address bus: inputs */
    for (int i = ADDR_PIN0; i < ADDR_PIN0 + ADDR_BITS; i++) {
        gpio_init(i); gpio_set_dir(i, GPIO_IN);
    }
    /* data bus: inputs by default */
    for (int i = DATA_PIN0; i < DATA_PIN0 + DATA_BITS; i++) {
        gpio_init(i); gpio_set_dir(i, GPIO_IN);
    }
    /* R/W: input with pull-up (default = read) */
    gpio_init(RW_PIN); gpio_set_dir(RW_PIN, GPIO_IN); gpio_pull_up(RW_PIN);

    /* /IRQ, /RESET: outputs, both released (high) */
    gpio_init(IRQ_PIN); gpio_set_dir(IRQ_PIN, GPIO_OUT); gpio_put(IRQ_PIN, true);
    gpio_init(RST_PIN); gpio_set_dir(RST_PIN, GPIO_OUT); gpio_put(RST_PIN, true);

    while (true) {
        /* wait for PHI2 rising edge (timeout handles stopped/step clock) */
        uint32_t spin = 0;
        while (!gpio_get(CLK_PIN)) {
            if (++spin > 200000) { via_tick(&s_via); update_irq(); spin = 0; }
        }

        /* ── sample bus at PHI2 high ─────────────────────────────── */
        uint32_t g    = gpio_get_all();
        uint16_t addr = (uint16_t)((g >> ADDR_PIN0) & 0xFFFF);
        bool reading  = !!(g & (1u << RW_PIN));

        if (reading) {
            /* READ: drive data bus before CPU samples it */
            uint8_t data = bus_read(addr);
            gpio_put_masked(DATA_MASK, (uint32_t)data << DATA_PIN0);
            gpio_set_dir_out_masked(DATA_MASK);

            /* record */
            uint32_t wp = s_trace_wp;
            s_trace[wp % TRACE_CAP] = (trace_t){ 0, data, addr };
            s_trace_wp = wp + 1;
            __atomic_fetch_add(&s_total_reads, 1, __ATOMIC_RELAXED);
        } else {
            /* WRITE: wait for data setup (~167 ns @ 150 MHz = 25 nops) */
            __asm volatile(
                "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
            uint8_t data = (uint8_t)((gpio_get_all() >> DATA_PIN0) & 0xFF);
            bus_write(addr, data);

            /* record */
            uint32_t wp = s_trace_wp;
            s_trace[wp % TRACE_CAP] = (trace_t){ 1, data, addr };
            s_trace_wp = wp + 1;
            __atomic_fetch_add(&s_total_writes, 1, __ATOMIC_RELAXED);
        }

        /* wait for PHI2 falling edge */
        while (gpio_get(CLK_PIN));

        /* release data bus */
        if (reading) gpio_set_dir_in_masked(DATA_MASK);

        /* ── end of cycle: tick VIA, update /IRQ ─────────────────── */
        via_tick(&s_via);
        update_irq();
    }
}

/* ── clock generator ──────────────────────────────────────────────── */
#define CLK_FREQ_MIN    1.0f
#define CLK_FREQ_MAX    30000000.0f
#define CLK_PWM_THRESH  9.0f

static const float PRESETS[] = {
     500000, 1000000, 2000000, 3579545, 4000000,
    5000000, 8000000,10000000,16000000,20000000,
};
static const char *PRESET_NAMES[] = {
    "500kHz","1MHz","2MHz","3.58MHz","4MHz",
    "5MHz","8MHz","10MHz","16MHz","20MHz",
};
static const float STEPS[]       = { 1,10,100,1e3,1e4,1e5,1e6,1e7 };
static const char *STEP_NAMES[]  = { "1Hz","10Hz","100Hz","1kHz","10kHz","100kHz","1MHz","10MHz" };
#define N_STEPS (int)(sizeof(STEPS)/sizeof(STEPS[0]))

volatile float  g_target_freq = 1000000.0f;
volatile float  g_actual_freq = 0.0f;
volatile bool   g_running     = false;  /* start stopped — let user load ROM first */
volatile bool   g_step_mode   = false;
volatile float  g_core_temp   = 0.0f;
volatile int    g_step_idx    = 5;

static uint    s_pwm_slice, s_pwm_chan;
static bool    s_use_pwm      = true;
static struct  repeating_timer s_timer;
static bool    s_timer_active = false;
static int32_t s_timer_half_us = 0;

static bool timer_cb(struct repeating_timer *t) {
    (void)t; gpio_xor_mask(1u << CLK_PIN); return true;
}

static void pwm_best(float hz, uint8_t *di, uint8_t *df, uint16_t *wrap) {
    uint32_t sys = clock_get_hz(clk_sys);
    float target = (float)sys / hz, best = 1e9f;
    *di=1; *df=0; *wrap=1;
    for (int d=1; d<=255; d++) for (int f=0; f<16; f++) {
        float div=d+f/16.0f, wf=target/div;
        if (wf<1||wf>65536) continue;
        uint32_t w=(uint32_t)(wf+0.5f);
        float e=fabsf((float)sys/(div*w)-hz)/hz;
        if (e<best){best=e;*di=(uint8_t)d;*df=(uint8_t)f;*wrap=(uint16_t)w;
                    if(e<1e-7f)return;}
    }
}

static void clk_stop(void) {
    if (s_timer_active) { cancel_repeating_timer(&s_timer); s_timer_active=false; }
    if (s_use_pwm) pwm_set_enabled(s_pwm_slice, false);
    gpio_set_function(CLK_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(CLK_PIN, GPIO_OUT);
    gpio_put(CLK_PIN, false);
}

static void clk_apply(float hz) {
    if (s_timer_active) { cancel_repeating_timer(&s_timer); s_timer_active=false; }
    if (s_use_pwm) pwm_set_enabled(s_pwm_slice, false);
    if (!g_running) {
        gpio_set_function(CLK_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(CLK_PIN, GPIO_OUT); gpio_put(CLK_PIN, false);
        g_actual_freq=hz; s_use_pwm=(hz>=CLK_PWM_THRESH); return;
    }
    if (hz >= CLK_PWM_THRESH) {
        uint8_t di,df; uint16_t wrap;
        pwm_best(hz, &di, &df, &wrap);
        gpio_set_function(CLK_PIN, GPIO_FUNC_PWM);
        pwm_set_clkdiv_int_frac(s_pwm_slice, di, df);
        pwm_set_wrap(s_pwm_slice, wrap-1);
        pwm_set_chan_level(s_pwm_slice, s_pwm_chan, wrap/2);
        pwm_set_enabled(s_pwm_slice, true);
        g_actual_freq=(float)clock_get_hz(clk_sys)/((di+df/16.0f)*wrap);
        s_use_pwm=true;
    } else {
        s_timer_half_us=(int32_t)(500000.0f/hz);
        gpio_set_function(CLK_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(CLK_PIN, GPIO_OUT); gpio_put(CLK_PIN, false);
        add_repeating_timer_us(-s_timer_half_us, timer_cb, NULL, &s_timer);
        s_timer_active=true;
        g_actual_freq=500000.0f/(float)s_timer_half_us;
        s_use_pwm=false;
    }
}

static void fmt_freq(char *b, size_t n, float hz) {
    if      (hz>=1e6f) snprintf(b,n,"%.6gMHz",(double)(hz/1e6f));
    else if (hz>=1e3f) snprintf(b,n,"%.6gkHz",(double)(hz/1e3f));
    else               snprintf(b,n,"%.4gHz", (double)hz);
}

static float read_core_temp(void) {
    adc_select_input(4);
    float v = adc_read()*(3.3f/4096.0f);
    return 27.0f-(v-0.706f)/0.001721f;
}

/* ── flash config ─────────────────────────────────────────────────── */
#define FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_MAGIC   0xC10CC10CU

typedef struct { uint32_t magic; float freq_hz; uint32_t crc; } flash_cfg_t;

static void flash_save(float hz) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    flash_cfg_t cfg = { FLASH_MAGIC, hz, FLASH_MAGIC^(uint32_t)hz };
    memcpy(page, &cfg, sizeof(cfg));
    multicore_lockout_start_blocking();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
    LOG_INFO("saved %.0f Hz", (double)hz);
}

static float flash_load(void) {
    const flash_cfg_t *c = (const flash_cfg_t *)(XIP_BASE+FLASH_OFFSET);
    if (c->magic!=FLASH_MAGIC) return 0;
    if (c->crc!=(FLASH_MAGIC^(uint32_t)c->freq_hz)) return 0;
    if (c->freq_hz<CLK_FREQ_MIN||c->freq_hz>CLK_FREQ_MAX) return 0;
    return c->freq_hz;
}

/* ── video helpers ────────────────────────────────────────────────── */
static void update_video_base(void) {
    g_video_base = 0xFFFF;
    for (int i = 0; i < s_layout.n_regions; i++) {
        if (s_layout.regions[i].type == REGION_VIDEO) {
            g_video_base = s_layout.regions[i].base;
            return;
        }
    }
}

/* Build and send one video frame over UART0.
 * Header: 0xFE 0xED | frame(2) | sys(1) | dirty_lines(24) | dirty_attrs(3)
 * Payload: for each dirty line: line_num(1) + 32 pixel bytes
 *          for each dirty attr row: row_num(1) + 32 attr bytes
 * Footer: 0xFD | xor_checksum(1)                                      */
static void video_send_frame(void) {
    if (s_layout.sys == VIDEO_NONE || g_video_base == 0xFFFF) return;

    /* snapshot + clear dirty flags (Core 1 may re-set during send) */
    uint32_t dl[6], da;
    for (int i = 0; i < 6; i++) { dl[i] = s_dirty_lines[i]; s_dirty_lines[i] = 0; }
    da = s_dirty_attrs; s_dirty_attrs = 0;

    uint8_t *p = s_vid_build;
    uint8_t  chk = 0;

#define PUT(b) do { uint8_t _b = (uint8_t)(b); *p++ = _b; chk ^= _b; } while(0)

    *p++ = 0xFE; *p++ = 0xED;   /* magic — not checksummed */
    PUT(s_video_frame); PUT(s_video_frame >> 8);
    PUT((uint8_t)s_layout.sys);

    for (int i = 0; i < 6; i++) {
        PUT(dl[i]); PUT(dl[i] >> 8); PUT(dl[i] >> 16); PUT(dl[i] >> 24);
    }
    PUT(da); PUT(da >> 8); PUT(da >> 16);

    for (int line = 0; line < VID_LINES; line++) {
        if (!(dl[line >> 5] & (1u << (line & 31)))) continue;
        PUT((uint8_t)line);
        /* Spectrum interleaved pixel address for this scanline */
        uint8_t  block     = (uint8_t)(line >> 6);
        uint8_t  char_row  = (uint8_t)((line >> 3) & 7);
        uint8_t  pixel_row = (uint8_t)(line & 7);
        uint16_t off       = (uint16_t)(block * 2048u + pixel_row * 256u + char_row * 32u);
        for (int j = 0; j < 32; j++) PUT(s_mem[g_video_base + off + j]);
    }

    for (int row = 0; row < VID_ATTR_ROWS; row++) {
        if (!(da & (1u << row))) continue;
        PUT((uint8_t)row);
        for (int j = 0; j < 32; j++) PUT(s_mem[g_video_base + VID_PIXEL_BYTES + row * 32 + j]);
    }

    *p++ = 0xFD;
    *p++ = chk;

#undef PUT

    uart_write_blocking(VIDEO_UART, s_vid_build, (size_t)(p - s_vid_build));
    s_video_frame++;
}

/* ── UART shell ───────────────────────────────────────────────────── */
static char s_line[128];
static int  s_linelen = 0;

static void print_status(void) {
    char t[20], a[20];
    fmt_freq(t, sizeof t, g_target_freq);
    fmt_freq(a, sizeof a, g_actual_freq);
    printf("[CLK %s] GP%d  tgt=%-10s act=%-10s %s  step=%s  T=%.1fC\r\n",
           g_step_mode?"STEP":(g_running?"RUN ":"STOP"),
           CLK_PIN, t, a, s_use_pwm?"PWM":"TMR",
           STEP_NAMES[g_step_idx], (double)g_core_temp);
}

static void hexdump(uint16_t base, uint16_t len) {
    for (uint16_t row=0; row<len; row+=16) {
        printf("%04X: ", (unsigned)(base+row));
        for (int i=0; i<16&&row+i<len; i++) printf("%02X ", s_mem[(base+row+i)&0xFFFF]);
        for (int i=(int)(len-row); i<16; i++) printf("   ");
        printf(" |");
        for (int i=0; i<16&&row+i<len; i++) {
            uint8_t c=s_mem[(base+row+i)&0xFFFF];
            printf("%c",(c>=0x20&&c<0x7F)?c:'.');
        }
        printf("|\r\n");
    }
}

static void print_layout(const mem_layout_t *l) {
    printf("Layout: %s — %s\r\n", l->name, l->desc);
    for (int i = 0; i < l->n_regions; i++) {
        const mem_region_t *r = &l->regions[i];
        uint32_t end = (uint32_t)r->base + r->size - 1;
        uint32_t kb  = r->size / 1024;
        if (kb)
            printf("  $%04X-$%04X  %2luKB  %s  [%s]\r\n",
                   r->base, (uint16_t)end, (unsigned long)kb,
                   region_type_str(r->type), r->label);
        else
            printf("  $%04X-$%04X  %3uB  %s  [%s]\r\n",
                   r->base, (uint16_t)end, r->size,
                   region_type_str(r->type), r->label);
    }
}

static void dump_via(void) {
    /* find VIA base from current layout */
    uint16_t via_base = 0;
    for (int i=0;i<s_layout.n_regions;i++)
        if (s_layout.regions[i].type==REGION_VIA) { via_base=s_layout.regions[i].base; break; }
    printf("=== VIA 6522 @ $%04X ===\r\n", via_base);
    printf("  ORA=%02X  DDRA=%02X  ORB=%02X  DDRB=%02X\r\n",
           s_via.ora, s_via.ddra, s_via.orb, s_via.ddrb);
    printf("  T1 ctr=%04X latch=%04X %s  ACR[6]=%d\r\n",
           s_via.t1_ctr, s_via.t1_latch,
           s_via.t1_running?"RUN":"stop", !!(s_via.acr&0x40));
    printf("  T2 ctr=%04X %s\r\n",
           s_via.t2_ctr, s_via.t2_running?"RUN":"stop");
    printf("  IFR=%02X  IER=%02X  active=%s\r\n",
           s_via.ifr, s_via.ier,
           (s_via.ifr & s_via.ier & 0x7F) ? "YES (IRQ asserted)" : "no");
    printf("  /IRQ pin: %s\r\n", gpio_get(IRQ_PIN) ? "released" : "ASSERTED");
}

static void process_command(const char *line) {
    while (*line==' ') line++;
    if (!*line) return;

    char cmd = line[0];
    const char *args = line+1;
    while (*args==' ') args++;

    switch (cmd) {

    case 'r': {
        uint16_t addr=(uint16_t)strtoul(args,NULL,16);
        printf("mem[%04X] = %02X\r\n", addr, s_mem[addr]);
        break;
    }
    case 'w': {
        char *end;
        uint16_t addr=(uint16_t)strtoul(args,&end,16);
        uint8_t  val =(uint8_t) strtoul(end, NULL,16);
        s_mem[addr]=val;
        printf("mem[%04X] <- %02X\r\n", addr, val);
        break;
    }
    case 'd': {
        char *end;
        uint16_t addr=(uint16_t)strtoul(args,&end, 16);
        uint16_t len =(*end)?(uint16_t)strtoul(end,NULL,16):256;
        hexdump(addr,len);
        break;
    }
    case 'f': {
        char *p;
        uint16_t addr=(uint16_t)strtoul(args,&p,16);
        uint16_t len =(uint16_t)strtoul(p,   &p,16);
        uint8_t  val =(uint8_t) strtoul(p,  NULL,16);
        for (uint32_t i=0;i<len;i++) s_mem[(uint16_t)(addr+i)]=val;
        printf("filled %04X..%04X with %02X\r\n",addr,(addr+len-1)&0xFFFF,val);
        break;
    }
    case 'l':
        printf("Paste Intel HEX (end with :00000001FF):\r\n");
        while (true) {
            char hex[128]; int n=0; int c;
            while ((c=getchar_timeout_us(5000000))!=PICO_ERROR_TIMEOUT) {
                if (c=='\r'||c=='\n'){hex[n]='\0';break;}
                if (n<(int)sizeof(hex)-1) hex[n++]=(char)c;
            }
            if (!hex[0]) continue;
            if (hex[0]!=':') continue;
            uint8_t  bc  =(uint8_t) strtoul((char[3]){hex[1],hex[2],0},NULL,16);
            uint16_t addr=(uint16_t)strtoul((char[5]){hex[3],hex[4],hex[5],hex[6],0},NULL,16);
            uint8_t  rt  =(uint8_t) strtoul((char[3]){hex[7],hex[8],0},NULL,16);
            if (rt==1){printf("HEX load done\r\n");break;}
            if (rt!=0) continue;
            for (int i=0;i<bc;i++) {
                uint8_t b=(uint8_t)strtoul((char[3]){hex[9+i*2],hex[10+i*2],0},NULL,16);
                s_mem[(addr+i)&0xFFFF]=b;
            }
            printf("  %d bytes @ %04X\r\n",bc,addr);
        }
        break;

    /* ── trace ─────────────────────────────────────────────────── */
    case 't': {
        if (line[1]=='c'||(line[1]==' '&&line[2]=='c')) {
            s_trace_wp=0; s_total_reads=0; s_total_writes=0;
            printf("trace cleared\r\n"); break;
        }
        int n=(*args)?(int)strtoul(args,NULL,10):32;
        uint32_t wp=s_trace_wp;
        uint32_t count=wp<TRACE_CAP?wp:TRACE_CAP;
        if ((uint32_t)n>count) n=(int)count;
        printf("last %d transactions (total R=%lu W=%lu):\r\n",
               n,(unsigned long)s_total_reads,(unsigned long)s_total_writes);
        for (int i=n-1;i>=0;i--) {
            trace_t *e=&s_trace[(wp-1-i+TRACE_CAP*2)%TRACE_CAP];
            const char *region = layout_label_for(&s_layout, e->addr);
            printf("  [%c] %04X (%s) = %02X\r\n",
                   e->is_write?'W':'R', e->addr, region, e->data);
        }
        break;
    }

    /* ── stats ─────────────────────────────────────────────────── */
    case 's': {
        char tf[20],af[20];
        fmt_freq(tf,sizeof tf,g_target_freq);
        fmt_freq(af,sizeof af,g_actual_freq);
        printf("=== Status ===\r\n");
        printf("CLK  : GP%d  %s → %s  %s  %s\r\n",
               CLK_PIN,tf,af,s_use_pwm?"PWM":"TMR",
               g_step_mode?"STEP":(g_running?"RUN":"STOP"));
        printf("Temp : %.1f C\r\n",(double)g_core_temp);
        printf("SYS  : %lu MHz\r\n",(unsigned long)(clock_get_hz(clk_sys)/1000000));
        print_layout(&s_layout);
        printf("Bus  : reads=%-8lu writes=%-8lu\r\n",
               (unsigned long)s_total_reads,(unsigned long)s_total_writes);
        uint16_t vec=(uint16_t)(s_mem[0xFFFC]|((uint16_t)s_mem[0xFFFD]<<8));
        printf("Reset vector: $%04X\r\n", vec);
        break;
    }

    /* ── VIA dump ──────────────────────────────────────────────── */
    case 'v':
        dump_via();
        break;

    /* ── CPU reset ─────────────────────────────────────────────── */
    case 'R':
        printf("Resetting CPU... ");
        gpio_put(RST_PIN, false);   /* assert /RESET */
        sleep_ms(10);
        gpio_put(RST_PIN, true);    /* release */
        printf("done (reset vector $%04X)\r\n",
               (uint16_t)(s_mem[0xFFFC]|((uint16_t)s_mem[0xFFFD]<<8)));
        s_total_reads=0; s_total_writes=0; s_trace_wp=0;
        memset(&s_via, 0, sizeof(s_via));   /* reset VIA state */
        break;

    /* ── manual /IRQ pulse ─────────────────────────────────────── */
    case 'I':
        gpio_put(IRQ_PIN, false);
        sleep_us(10);
        gpio_put(IRQ_PIN, true);
        printf("/IRQ pulsed\r\n");
        break;

    /* ── clock ─────────────────────────────────────────────────── */
    case 'c':
        if (line[1]=='l'&&line[2]=='k') {
            if (*args) {
                float hz=strtof(args,NULL);
                if (hz<CLK_FREQ_MIN) hz=CLK_FREQ_MIN;
                if (hz>CLK_FREQ_MAX) hz=CLK_FREQ_MAX;
                g_target_freq=hz;
                if (g_running) clk_apply(hz);
            }
            print_status();
        }
        break;

    case '0':case '1':case '2':case '3':case '4':
    case '5':case '6':case '7':case '8':case '9': {
        float hz=PRESETS[cmd-'0'];
        g_target_freq=hz; if(g_running) clk_apply(hz);
        print_status(); break;
    }
    case 'g':
        g_step_mode=false; g_running=true; clk_apply(g_target_freq);
        LOG_INFO("clock RUN"); print_status(); break;
    case 'p':
        g_step_mode=false; g_running=false; clk_stop();
        LOG_INFO("clock STOP"); print_status(); break;
    case 'm':
        g_step_mode=!g_step_mode;
        if (g_step_mode){
            g_running=false; clk_stop();
            gpio_set_function(CLK_PIN,GPIO_FUNC_SIO);
            gpio_set_dir(CLK_PIN,GPIO_OUT); gpio_put(CLK_PIN,false);
            LOG_INFO("step mode ON");
        } else { LOG_INFO("step mode OFF"); }
        break;
    case '.':
        if (g_step_mode) {
            gpio_put(CLK_PIN,true); sleep_us(1); gpio_put(CLK_PIN,false);
            LOG_INFO("clock pulse");
        }
        break;
    case 'W':
        flash_save(g_target_freq); break;

    /* ── layout ────────────────────────────────────────────────── */
    case 'L': {
        /* L              → show current layout
           L list         → list all predefined layouts
           L <name>       → switch to named layout (stops clock)
           L load         → enter custom layout via UART          */
        if (!*args || strcmp(args,"show")==0) {
            print_layout(&s_layout);
            break;
        }
        if (strcmp(args,"list")==0) {
            printf("Predefined layouts:\r\n");
            for (int i=0;i<N_LAYOUTS;i++)
                printf("  %-10s  %s%s\r\n", LAYOUTS[i].name, LAYOUTS[i].desc,
                       LAYOUTS[i].sys != VIDEO_NONE ? " [VIDEO]" : "");
            printf("  (also: 'L load' to enter a custom layout)\r\n");
            break;
        }
        if (strcmp(args,"load")==0) {
            mem_layout_t tmp;
            memset(&tmp,0,sizeof(tmp));
            strcpy(tmp.name,"custom");
            strcpy(tmp.desc,"user-defined");
            printf("Enter layout lines (<type> <base_hex> <size_hex>), then 'end':\r\n");
            char lbuf[64]; int llen=0;
            bool more=true;
            while (more) {
                int c=getchar_timeout_us(10000000);
                if (c==PICO_ERROR_TIMEOUT) break;
                if (c=='\r'||c=='\n') {
                    lbuf[llen]='\0';
                    if (llen>0) more=layout_parse_line(&tmp,lbuf);
                    llen=0;
                } else if (llen<(int)sizeof(lbuf)-1) lbuf[llen++]=(char)c;
            }
            if (tmp.n_regions==0){printf("no regions — layout unchanged\r\n");break;}
            g_running=false; clk_stop();
            s_layout=tmp; update_video_base();
            printf("Custom layout applied:\r\n"); print_layout(&s_layout);
            break;
        }
        /* switch by name */
        bool found=false;
        for (int i=0;i<N_LAYOUTS;i++) {
            if (strcmp(LAYOUTS[i].name,args)==0) {
                g_running=false; clk_stop();
                s_layout=LAYOUTS[i]; update_video_base();
                printf("Switched to '%s' (clock stopped — 'g' to run):\r\n",args);
                print_layout(&s_layout);
                found=true; break;
            }
        }
        if (!found) printf("unknown layout '%s' — try 'L list'\r\n",args);
        break;
    }

    case 'h': case '?':
        printf("\r\n=== RP2350 Retro CPU Support ===\r\n");
        printf(" Layout : %-10s  %s\r\n", s_layout.name, s_layout.desc);
        printf(" Pins   : A0-A15=GP3-18  D0-D7=GP19-26  R/W=GP27  /IRQ=GP28  /RST=GP29\r\n\r\n");
        printf(" Memory:  r/w/d/f <addr>  l=HEX load  t[n]/tc=trace  s=stats\r\n");
        printf(" Layout:  L show  L list  L <name>  L load\r\n");
        printf(" VIA:     v=dump registers\r\n");
        printf(" CPU:     R=reset  I=pulse /IRQ\r\n");
        printf(" Clock:   clk <hz>  0-9=presets  g=run  p=stop  m=step  .=pulse  W=save\r\n");
        printf(" Presets:");
        for (int i=0;i<10;i++) printf(" %d=%s",i,PRESET_NAMES[i]);
        printf("\r\n\r\n");
        break;

    default:
        printf("unknown '%c' — h for help\r\n", cmd);
    }
}

static void handle_uart(void) {
    int c=getchar_timeout_us(0);
    if (c==PICO_ERROR_TIMEOUT) return;
    if (c=='\r'||c=='\n') {
        s_line[s_linelen]='\0';
        if (s_linelen>0) process_command(s_line);
        s_linelen=0;
    } else if (c==0x7F||c=='\b') {
        if (s_linelen>0){s_linelen--;printf("\b \b");}
    } else if (s_linelen<(int)sizeof(s_line)-1) {
        s_line[s_linelen++]=(char)c; putchar(c);
    }
}

/* ── main ─────────────────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();
    sleep_ms(200);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    gpio_set_function(CLK_PIN, GPIO_FUNC_PWM);
    s_pwm_slice = pwm_gpio_to_slice_num(CLK_PIN);
    s_pwm_chan  = pwm_gpio_to_channel(CLK_PIN);

    float saved = flash_load();
    if (saved > 0.0f) { g_target_freq=saved; LOG_INFO("loaded %.0f Hz from flash",(double)saved); }

    memset(&s_via, 0, sizeof(s_via));
    memset(s_mem, 0xFF, sizeof(s_mem));   /* 0xFF = NOP-like default on many CPUs */
    s_layout = LAYOUTS[0];                /* default: generic */
    update_video_base();

    /* UART0 on GP0/GP1 @ 2Mbaud — video link to Pico 2 (stdio is USB CDC) */
    uart_init(VIDEO_UART, VIDEO_UART_BAUD);
    gpio_set_function(VIDEO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(VIDEO_UART_RX_PIN, GPIO_FUNC_UART);

    multicore_launch_core1(core1_entry);

    /* clock starts stopped — user loads ROM first, then 'R' to reset CPU */
    clk_apply(g_target_freq);

    uint32_t sys_mhz = clock_get_hz(clk_sys)/1000000;
    printf("\r\n================================\r\n");
    printf(" RP2350 Retro CPU Support\r\n");
    printf(" PHI2   : GP%d  (stopped — 'g' to run)\r\n", CLK_PIN);
    printf(" Address: GP%d–GP%d (A0–A15)\r\n", ADDR_PIN0, ADDR_PIN0+ADDR_BITS-1);
    printf(" Data   : GP%d–GP%d (D0–D7)\r\n", DATA_PIN0, DATA_PIN0+DATA_BITS-1);
    printf(" R/W    : GP%d   /IRQ: GP%d   /RST: GP%d\r\n", RW_PIN, IRQ_PIN, RST_PIN);
    printf(" Layout : %s — %s\r\n", s_layout.name, s_layout.desc);
    printf(" SYS    : %lu MHz\r\n", (unsigned long)sys_mhz);
    printf(" Video  : GP%d TX → Pico2 GP1 RX @ %d Mbaud\r\n",
           VIDEO_UART_TX_PIN, VIDEO_UART_BAUD/1000000);
    printf(" Workflow: 1) load ROM  2) 'g' run clock  3) 'R' reset CPU\r\n");
    printf(" Type 'h' for help\r\n");
    printf("================================\r\n\r\n");

    absolute_time_t next_status = make_timeout_time_ms(10000);
    absolute_time_t next_video  = make_timeout_time_ms(20);   /* 50 Hz */

    while (true) {
        handle_uart();
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(now, next_video) <= 0) {
            video_send_frame();
            next_video = make_timeout_time_ms(20);
        }
        if (absolute_time_diff_us(now, next_status) <= 0) {
            g_core_temp = read_core_temp();
            print_status();
            next_status = make_timeout_time_ms(10000);
        }
    }
}
