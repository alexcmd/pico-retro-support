cmake_minimum_required(VERSION 3.13)

# Must be set before project() — pico2 board implies rp2350 platform
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(demo-gui C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(demo-gui
    src/main.c
)

target_include_directories(demo-gui PRIVATE src)

target_link_libraries(demo-gui PRIVATE
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_watchdog
    hardware_pwm
    hardware_clocks
    hardware_flash
    hardware_sync
    hardware_uart
    pico_multicore
    pico_float
)

# USB CDC for stdio — UART0 (GP0/GP1) is reserved for video link to Pico 2
pico_enable_stdio_uart(demo-gui 0)
pico_enable_stdio_usb(demo-gui 1)

# Debug build: full symbols, no optimization
target_compile_options(demo-gui PRIVATE
    -g3
    -O0
    -fno-omit-frame-pointer
)

# Generate .uf2 .bin .hex .dis .map
pico_add_extra_outputs(demo-gui)

# ── tooling paths ────────────────────────────────────────────────────
set(PROBE_RS   probe-rs)
set(GDB        "$ENV{HOME}/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb")
set(CHIP       RP2350)
set(GDB_PORT   3333)
set(USB_DEV    /dev/ttyACM0)   # Pico USB CDC shell (adjust if needed)
set(UART_DEV   /dev/ttyACM1)   # debugprobe passthrough (now carries video link traffic)
set(UART_BAUD  115200)

# ── cmake --build build --target flash ───────────────────────────────
add_custom_target(flash
    COMMAND ${PROBE_RS} download --chip ${CHIP} $<TARGET_FILE:demo-gui>
    COMMAND ${PROBE_RS} reset --chip ${CHIP}
    DEPENDS demo-gui
    USES_TERMINAL
    COMMENT "Flashing demo-gui to ${CHIP} via SWD"
)

# ── cmake --build build --target gdb-server ──────────────────────────
add_custom_target(gdb-server
    COMMAND ${PROBE_RS} gdb --chip ${CHIP} $<TARGET_FILE:demo-gui> --port ${GDB_PORT}
    DEPENDS demo-gui
    USES_TERMINAL
    COMMENT "Starting probe-rs GDB server on :${GDB_PORT} — connect with: cmake --build build --target gdb"
)

# ── cmake --build build --target gdb ─────────────────────────────────
add_custom_target(gdb
    COMMAND ${GDB} -x ${CMAKE_SOURCE_DIR}/.gdbinit $<TARGET_FILE:demo-gui>
    DEPENDS demo-gui
    USES_TERMINAL
    COMMENT "Starting GDB session (requires gdb-server running in another terminal)"
)

# ── cmake --build build --target gdb-tui ─────────────────────────────
add_custom_target(gdb-tui
    COMMAND ${GDB} -tui -x ${CMAKE_SOURCE_DIR}/.gdbinit $<TARGET_FILE:demo-gui>
    DEPENDS demo-gui
    USES_TERMINAL
    COMMENT "Starting GDB TUI session"
)

# ── cmake --build build --target shell ───────────────────────────────
add_custom_target(shell
    COMMAND picocom -b 115200 --flow n ${USB_DEV}
    USES_TERMINAL
    COMMENT "Opening USB CDC shell on ${USB_DEV} (Ctrl+A X to quit)"
)

# ── cmake --build build --target uart ────────────────────────────────
add_custom_target(uart
    COMMAND stty -F ${UART_DEV} ${UART_BAUD} cs8 -cstopb -parenb raw
    COMMAND picocom -b ${UART_BAUD} --flow n ${UART_DEV}
    USES_TERMINAL
    COMMENT "Opening UART monitor on ${UART_DEV} @ ${UART_BAUD} (video link snooping)"
)
