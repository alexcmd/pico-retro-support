#!/home/deck/.pico-venv/bin/python3
"""
write_ram.py — stream a binary file into the RP2350 RAM emulator via UART.

The Pico firmware is NOT reflashed; data goes into the 64 KB SRAM that
Core 1 exposes as a RAM chip to the external CPU.

Usage:
    ./write_ram.py /dev/ttyACM1 rom.bin
    ./write_ram.py /dev/ttyACM1 rom.bin --base 0xC000      # load at $C000
    ./write_ram.py /dev/ttyACM1 rom.bin --base 0 --verify  # load + readback check

Requirements:  pip install pyserial
"""

import sys
import time
import argparse
import struct

# ── Intel HEX helpers ─────────────────────────────────────────────────────────

def _hex_record(rec_type: int, addr: int, data: bytes) -> str:
    row = bytes([len(data), (addr >> 8) & 0xFF, addr & 0xFF, rec_type]) + data
    checksum = (-sum(row)) & 0xFF
    return ':' + row.hex().upper() + f'{checksum:02X}'

def bin_to_ihex(data: bytes, base: int) -> list[str]:
    """Convert raw bytes to a list of Intel HEX record strings."""
    records = []
    for off in range(0, len(data), 16):
        chunk = data[off:off + 16]
        records.append(_hex_record(0x00, (base + off) & 0xFFFF, chunk))
    records.append(':00000001FF')   # EOF record
    return records

# ── hex dump parser (for --verify) ───────────────────────────────────────────

def parse_hexdump(text: str) -> dict[int, int]:
    """Parse 'AAAA: XX XX ... | ...' lines → {addr: byte}."""
    result = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or '|' not in line:
            continue
        try:
            addr_str, rest = line.split(':', 1)
            addr = int(addr_str.strip(), 16)
            hex_part = rest.split('|')[0].strip().split()
            for i, h in enumerate(hex_part):
                result[addr + i] = int(h, 16)
        except (ValueError, IndexError):
            pass
    return result

# ── serial helpers ────────────────────────────────────────────────────────────

def drain(ser, timeout: float = 0.3) -> str:
    """Read all pending bytes, waiting up to timeout seconds."""
    deadline = time.monotonic() + timeout
    buf = b''
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            deadline = time.monotonic() + 0.1   # extend if data is arriving
        else:
            time.sleep(0.02)
    return buf.decode(errors='replace')

def send_line(ser, text: str):
    ser.write((text + '\r\n').encode())

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Load a binary into the RP2350 RAM emulator over UART')
    ap.add_argument('port',    help='Serial port, e.g. /dev/ttyACM1')
    ap.add_argument('binfile', help='Binary file to load')
    ap.add_argument('--base',  default='0x0000',
                    help='Load address in hex (default: 0x0000)')
    ap.add_argument('--baud',  type=int, default=115200,
                    help='Baud rate (default: 115200)')
    ap.add_argument('--verify', action='store_true',
                    help='Read back RAM after load and compare')
    ap.add_argument('--inter-record-ms', type=int, default=5,
                    help='Delay between HEX records in ms (default: 5)')
    args = ap.parse_args()

    base = int(args.base, 16)

    with open(args.binfile, 'rb') as f:
        data = f.read()

    size = len(data)
    end  = (base + size - 1) & 0xFFFF
    print(f'File   : {args.binfile}  ({size} bytes)')
    print(f'Range  : {base:04X}..{end:04X}')
    print(f'Port   : {args.port} @ {args.baud}')
    print()

    try:
        import serial
    except ImportError:
        print('ERROR: pyserial not installed.  Run:  pip install pyserial')
        sys.exit(1)

    records = bin_to_ihex(data, base)
    delay   = args.inter_record_ms / 1000.0

    with serial.Serial(args.port, args.baud, timeout=2) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        # ── send 'l' to enter HEX load mode ──────────────────────────
        print('Entering HEX load mode...')
        send_line(ser, 'l')
        resp = drain(ser, 0.5)
        if 'Paste Intel HEX' not in resp:
            # retry once — Pico may have been mid-prompt
            send_line(ser, 'l')
            resp = drain(ser, 0.5)
        if 'Paste Intel HEX' not in resp:
            print(f'ERROR: unexpected response:\n{resp}')
            sys.exit(1)
        print('  ready')

        # ── stream records ────────────────────────────────────────────
        total = len(records)
        print(f'Sending {total} records ({size} bytes + EOF)...')
        t0 = time.monotonic()

        for i, rec in enumerate(records):
            send_line(ser, rec)
            if delay:
                time.sleep(delay)
            # show progress every 64 records
            if i % 64 == 0 or i == total - 1:
                pct  = (i + 1) * 100 // total
                done = i + 1
                bar  = '█' * (pct // 5) + '░' * (20 - pct // 5)
                print(f'  [{bar}] {pct:3d}%  {done}/{total}', end='\r')

        elapsed = time.monotonic() - t0
        print(f'\n  done in {elapsed:.1f}s  '
              f'({size / elapsed:.0f} bytes/s)')

        # ── collect final acknowledgement ─────────────────────────────
        resp = drain(ser, 0.5)
        for line in resp.splitlines():
            line = line.strip()
            if line:
                print(f'  Pico: {line}')

        # ── optional readback verification ────────────────────────────
        if args.verify:
            print()
            print('Verifying...')
            CHUNK = 256   # bytes per 'd' command
            errors = 0
            for off in range(0, size, CHUNK):
                chunk_addr = (base + off) & 0xFFFF
                chunk_len  = min(CHUNK, size - off)
                send_line(ser, f'd {chunk_addr:04X} {chunk_len:04X}')
                resp = drain(ser, 0.5)
                parsed = parse_hexdump(resp)
                for i in range(chunk_len):
                    addr = (chunk_addr + i) & 0xFFFF
                    expected = data[off + i]
                    actual   = parsed.get(addr)
                    if actual is None:
                        print(f'  WARN: no readback for addr {addr:04X}')
                    elif actual != expected:
                        print(f'  MISMATCH {addr:04X}: expected {expected:02X} got {actual:02X}')
                        errors += 1
                pct = min(off + CHUNK, size) * 100 // size
                print(f'  verified {pct:3d}%', end='\r')

            if errors == 0:
                print(f'\n  OK — {size} bytes verified clean')
            else:
                print(f'\n  FAILED — {errors} mismatches')
                sys.exit(1)

    print('\nDone.')

if __name__ == '__main__':
    main()
