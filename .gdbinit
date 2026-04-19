# .gdbinit — RP2350 debug via probe-rs GDB server
# Start server first:  make gdb-server   (terminal A)
# Then run GDB:        make gdb          (terminal B)

set architecture armv8-m.main
set pagination off
set print pretty on
set print array on
set print array-indexes on
set disassemble-next-line auto
set mem inaccessible-by-default off

# RP2350 memory regions
mem 0x00000000 0x00003fff ro    # Boot ROM (16 KB)
mem 0x10000000 0x11ffffff ro    # Flash / XIP (up to 32 MB)
mem 0x20000000 0x20081fff rw    # SRAM (520 KB)
mem 0xd0000000 0xd0000fff rw    # SIO

# Connect to probe-rs GDB server
target extended-remote :3333

# Flash firmware and load symbols
load

# ── convenience commands ────────────────────────────────────────────

define rp_state
  printf "=== App state ===\n"
  print g_app_state
  print g_tick
  print g_led
  print g_core_temp
end
document rp_state
  Print application state variables (g_app_state, g_tick, g_led, g_core_temp).
end

define rp_regs
  printf "=== ARM core registers ===\n"
  info registers
end
document rp_regs
  Print all ARM core registers.
end

define rp_stack
  printf "=== Call stack ===\n"
  backtrace full
end
document rp_stack
  Print full call stack with local variables.
end

define rp_gpio
  printf "=== SIO GPIO_IN  @ 0xd0000004: ===\n"
  x/xw 0xd0000004
  printf "=== SIO GPIO_OUT @ 0xd0000010: ===\n"
  x/xw 0xd0000010
  printf "=== SIO GPIO_OE  @ 0xd0000020: ===\n"
  x/xw 0xd0000020
end
document rp_gpio
  Dump SIO GPIO input, output, and output-enable registers.
end

define rp_uart0
  printf "=== UART0 registers @ 0x40070000 ===\n"
  x/8xw 0x40070000
end
document rp_uart0
  Dump UART0 peripheral registers (DR, RSR, FR, IBRD, FBRD, LCR_H, CR, IMSC).
end

define rp_adc
  printf "=== ADC CS    @ 0x4004c000: ===\n"
  x/xw 0x4004c000
  printf "=== ADC RESULT@ 0x4004c004: ===\n"
  x/xw 0x4004c004
end
document rp_adc
  Dump ADC control/status and last result registers.
end

define rp_dump
  printf "\n========== RP2350 FULL DUMP ==========\n"
  rp_regs
  printf "\n"
  rp_gpio
  printf "\n"
  rp_state
  printf "======================================\n\n"
end
document rp_dump
  Full system dump: ARM registers + GPIO + app state.
end

define rp_mem
  if $argc != 2
    printf "Usage: rp_mem <addr> <count_words>\n"
  else
    x/$arg1xw $arg0
  end
end
document rp_mem
  Hex dump memory. Example: rp_mem 0x20000000 32
end

define rp_reset
  monitor reset halt
  printf "Target halted after reset.\n"
end
document rp_reset
  Reset target and halt at reset vector.
end

# ── initial break at main ────────────────────────────────────────────
break main
monitor reset halt
continue
