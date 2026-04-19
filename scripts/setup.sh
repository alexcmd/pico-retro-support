#!/usr/bin/env bash
# One-time setup: install probe-rs, picocom, udev rules, GDB auto-load
set -euo pipefail

TOOLS_DIR="$HOME/.local/bin"
mkdir -p "$TOOLS_DIR"

# ── probe-rs ───────────────────────────────────────────────────────
if ! command -v probe-rs &>/dev/null; then
    echo "==> Installing probe-rs..."
    curl --proto '=https' --tlsv1.2 -LsSf \
        https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
    echo "probe-rs installed to ~/.cargo/bin/"
else
    echo "==> probe-rs already installed: $(probe-rs --version)"
fi

# ── picocom ────────────────────────────────────────────────────────
if ! command -v picocom &>/dev/null; then
    echo "==> Installing picocom (needs sudo)..."
    sudo pacman -S --noconfirm picocom
else
    echo "==> picocom already installed"
fi

# ── cmake + ninja ─────────────────────────────────────────────────
for pkg in cmake ninja; do
    if ! command -v "$pkg" &>/dev/null; then
        echo "==> Installing $pkg (needs sudo)..."
        sudo pacman -S --noconfirm "$pkg"
    else
        echo "==> $pkg already installed"
    fi
done

# ── udev rules for Debugprobe ──────────────────────────────────────
UDEV_FILE=/etc/udev/rules.d/99-debugprobe.rules
if [[ ! -f "$UDEV_FILE" ]]; then
    echo "==> Installing udev rules (needs sudo)..."
    sudo tee "$UDEV_FILE" > /dev/null << 'UDEV'
# Raspberry Pi Debugprobe (2e8a:000c) — allow non-root access
SUBSYSTEM=="usb",  ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000c", MODE="660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000c", MODE="660", GROUP="plugdev", TAG+="uaccess"
UDEV
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "==> udev rules installed"
else
    echo "==> udev rules already present"
fi

# ── plugdev group ──────────────────────────────────────────────────
if ! groups "$USER" | grep -q plugdev; then
    echo "==> Adding $USER to plugdev group (needs sudo)..."
    sudo usermod -aG plugdev "$USER"
    echo "    NOTE: log out and back in for group change to take effect"
else
    echo "==> $USER already in plugdev group"
fi

# ── GDB auto-load safe path ────────────────────────────────────────
GDBINIT="$HOME/.gdbinit"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SAFE_PATH_LINE="add-auto-load-safe-path $PROJECT_DIR/.gdbinit"
if ! grep -qF "$SAFE_PATH_LINE" "$GDBINIT" 2>/dev/null; then
    echo "$SAFE_PATH_LINE" >> "$GDBINIT"
    echo "==> GDB auto-load safe path added for $PROJECT_DIR"
fi

echo ""
echo "Setup complete. Re-plug the Debugprobe if this is the first run."
