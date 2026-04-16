#!/usr/bin/env bash
# deploy.sh — build firmware locally, flash via Pi4, update server
# Usage: ./deploy.sh [--firmware-only | --server-only]
set -e

PI="router@10.42.0.1"
REMOTE_DIR="/home/router/whip"

FIRMWARE=1
SERVER=1
if [[ "$1" == "--firmware-only" ]]; then SERVER=0; fi
if [[ "$1" == "--server-only"   ]]; then FIRMWARE=0; fi

# ── Firmware ──────────────────────────────────────────────────────────────────
if [[ $FIRMWARE -eq 1 ]]; then
  echo "▶ Building firmware..."
  pio run

  UF2=".pio/build/pico/firmware.uf2"
  if [[ ! -f "$UF2" ]]; then
    echo "ERROR: $UF2 not found — did the build succeed?"
    exit 1
  fi

  echo "▶ Copying firmware to Pi4..."
  scp "$UF2" "$PI:~/firmware.uf2"

  echo "▶ Flashing Pico via Pi4..."
  ssh "$PI" bash <<'REMOTE'
    set -e
    sudo systemctl stop whip || true
    sleep 1

    # Trigger BOOTSEL via 1200-baud touch (works without physical button)
    PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
    if [[ -n "$PORT" ]]; then
      echo "Triggering BOOTSEL via 1200-baud on $PORT..."
      stty -F "$PORT" 1200 2>/dev/null || true
      sleep 2
    fi

    if command -v picotool &>/dev/null; then
      picotool load -x ~/firmware.uf2 -f
      sleep 2
    else
      # Fallback: mount approach (user must have pressed BOOTSEL)
      MOUNT=$(findmnt -rno TARGET -S LABEL=RP2350 2>/dev/null \
           || findmnt -rno TARGET -S LABEL=RPI-RP2 2>/dev/null || true)
      if [[ -z "$MOUNT" ]]; then
        echo "Pico not in BOOTSEL mode and picotool not installed."
        echo "Install picotool:  sudo apt install picotool"
        exit 1
      fi
      cp ~/firmware.uf2 "$MOUNT/"
      sync
    fi
    echo "Firmware flashed. Waiting for Pico to reboot..."
    sleep 3
    sudo systemctl start whip
    echo "Service restarted."
REMOTE
fi

# ── Server ────────────────────────────────────────────────────────────────────
if [[ $SERVER -eq 1 ]]; then
  echo "▶ Copying server files to Pi4..."
  scp "Pi4/server.py" "Pi4/mobile.html" "$PI:$REMOTE_DIR/"

  echo "▶ Restarting whip service..."
  ssh "$PI" "sudo systemctl restart whip"
fi

echo "✓ Done."
