# RP2350 Retro CPU Support

Clock generator + 64 KB RAM/ROM emulator + VIA 6522, all on one Pico 2.
Core 0 runs the clock and UART shell. Core 1 runs the bus emulator in a
PHI2-synchronised tight loop with no interrupts.

---

## Pin map

| GPIO | Function | Direction |
|------|----------|-----------|
| GP0 | UART TX (stdio) | out |
| GP1 | UART RX (stdio) | in |
| GP2 | PHI2 / CLK out | out |
| GP3–GP18 | A0–A15 address bus | in (from CPU) |
| GP19–GP26 | D0–D7 data bus | bidirectional |
| GP27 | R/W (HIGH=read, LOW=write) | in (from CPU) |
| GP28 | /IRQ | out (active low, RP2350 → CPU) |
| GP29 | /RESET | out (active low, RP2350 → CPU) |

---

## Memory map

```
$0000–$7FFF   32 KB   RAM  (r/w)
$8000–$800F   16 B    VIA 6522 registers
$8010–$BFFF   ~16 KB  RAM  (r/w, contiguous with lower)
$C000–$FFFF   16 KB   ROM  (read-only — load with write_ram.py)
```

---

## VIA 6522 registers ($8000–$800F)

| Offset | Name | Function |
|--------|------|----------|
| $00 | ORB | Output Register B |
| $01 | ORA | Output Register A |
| $02 | DDRB | Data Direction B |
| $03 | DDRA | Data Direction A |
| $04 | T1CL | Timer 1 counter low (read clears IFR bit 6) |
| $05 | T1CH | Timer 1 counter high |
| $06 | T1LL | Timer 1 latch low |
| $07 | T1LH | Timer 1 latch high |
| $08 | T2CL | Timer 2 counter low (read clears IFR bit 5) |
| $09 | T2CH | Timer 2 counter high |
| $0B | ACR | Auxiliary Control (bit 6 = T1 continuous mode) |
| $0D | IFR | Interrupt Flag Register (write 1 to clear) |
| $0E | IER | Interrupt Enable (bit 7: 1=set, 0=clear) |

Timer 1 decrements each PHI2 cycle; sets IFR bit 6 on underflow.
In continuous mode (ACR bit 6) it reloads from latch automatically.
Timer 2 is always one-shot; sets IFR bit 5 on underflow.
/IRQ is asserted when `IFR & IER & 0x7F != 0`.

Port A/B are virtual (no spare GPIO pins for physical I/O).

---

## Workflow

```bash
# 1. Load ROM at $C000
./scripts/write_ram.py /dev/ttyACM1 rom.bin --base 0xC000 --verify

# 2. Open UART shell
cmake --build build --target uart

# 3. Set clock and run
clk 1000000      # 1 MHz
g                # start PHI2

# 4. Reset CPU (pulses /RESET, CPU reads reset vector at $FFFC/$FFFD)
R

# 5. Watch bus transactions
t 32             # last 32 reads/writes with region tag (RAM/ROM/VIA)

# 6. Inspect VIA state
v
```

---

## UART shell commands

### Memory
| Command | Action |
|---------|--------|
| `r ADDR` | read byte at hex address |
| `w ADDR VAL` | write byte |
| `d ADDR [LEN]` | hex dump (default 256 bytes) |
| `f ADDR LEN VAL` | fill range |
| `l` | load Intel HEX (paste or pipe) |

### Bus diagnostics
| Command | Action |
|---------|--------|
| `t [N]` | show last N bus transactions (default 32) — tagged RAM/ROM/VIA |
| `tc` | clear trace buffer and counters |
| `s` | status: clock, memory map, total R/W, reset vector |
| `v` | VIA 6522 register dump |

### CPU control
| Command | Action |
|---------|--------|
| `R` | pulse /RESET (resets CPU and VIA state) |
| `I` | pulse /IRQ manually |

### Clock
| Command | Action |
|---------|--------|
| `clk HZ` | set frequency |
| `0`–`9` | presets: 0=500kHz 1=1MHz 2=2MHz 3=3.58MHz 4=4MHz 5=5MHz 6=8MHz 7=10MHz 8=16MHz 9=20MHz |
| `g` | run clock |
| `p` | stop clock |
| `m` | toggle manual step mode |
| `.` | single PHI2 pulse (step mode) |
| `W` | save current frequency to flash |

---

## Build & flash

```bash
export PICO_SDK_PATH=~/pico-sdk
export PATH="$PATH:~/.cargo/bin"

# configure (once)
cmake --preset rp2350-debug

# build
cmake --build build

# flash via SWD
cmake --build build --target flash

# UART monitor
cmake --build build --target uart
```

Requires: ARM GNU Toolchain 14.2 at `~/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/`, probe-rs at `~/.cargo/bin/probe-rs`, pico-sdk at `~/pico-sdk`.

---

## write_ram.py

Load a binary file into RAM/ROM without reflashing the Pico:

```bash
# install dependency (once)
python3 -m venv ~/.pico-venv && ~/.pico-venv/bin/pip install pyserial

# load ROM at $C000
./scripts/write_ram.py /dev/ttyACM1 rom.bin --base 0xC000

# load + verify readback
./scripts/write_ram.py /dev/ttyACM1 rom.bin --base 0xC000 --verify

# load at custom address
./scripts/write_ram.py /dev/ttyACM1 data.bin --base 0x0200
```

---

## Notes

- Clock starts **stopped** at boot — load ROM first, then `g`, then `R`
- ROM region ($C000–$FFFF) is write-protected in software; writes are silently ignored
- Flash stores last clock frequency (magic + XOR CRC); auto-loaded on boot
- For CPUs >4 MHz, PIO state machines would be needed for reliable bus timing
- VIA Port A/B are virtual registers only — no physical GPIO lines available
