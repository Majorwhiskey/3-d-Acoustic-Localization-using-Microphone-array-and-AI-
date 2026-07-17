#!/usr/bin/env bash
#
# flash-fleet.sh - flash and customize multiple SD cards at once from a Pi 5
#
# ONE-TIME SETUP (do this before running the script):
#   1. In the Raspberry Pi Imager GUI, flash ONE card with your image and
#      full OS Customization (gear icon): username, password, SSH, WiFi,
#      locale, whatever you need on every board.
#   2. Before you boot that card, plug it back into the Pi 5 and copy its
#      firstrun.sh off the boot partition into ./firstrun-template.sh
#         cp /media/$USER/bootfs/firstrun.sh ./firstrun-template.sh
#   3. Open firstrun-template.sh, find the line that sets the hostname,
#      and replace the hardcoded value with the token __HOSTNAME__
#   4. Put your OS image (.img or .img.xz) in this same folder.
#
# USAGE:
#   sudo ./flash-fleet.sh raspios.img.xz pi-node
#
#   This flashes every detected USB SD card with raspios.img.xz and gives
#   them hostnames pi-node-01, pi-node-02, pi-node-03, and so on.

set -euo pipefail

IMAGE="${1:?Usage: sudo $0 <image.img[.xz]> <hostname-prefix>}"
PREFIX="${2:?Usage: sudo $0 <image.img[.xz]> <hostname-prefix>}"
TEMPLATE="./firstrun-template.sh"
WORKDIR="$(mktemp -d)"

if [[ $EUID -ne 0 ]]; then
  echo "Run this with sudo. rpi-imager needs elevated privileges to write disks."
  exit 1
fi

[[ -f "$IMAGE" ]] || { echo "Image not found: $IMAGE"; exit 1; }
[[ -f "$TEMPLATE" ]] || { echo "Missing $TEMPLATE. See the setup steps at the top of this script."; exit 1; }
command -v rpi-imager >/dev/null || { echo "rpi-imager not found. Run: sudo apt install rpi-imager"; exit 1; }
grep -q '__HOSTNAME__' "$TEMPLATE" || { echo "Warning: $TEMPLATE has no __HOSTNAME__ token. Every card will get the same hostname."; }

# Find candidate SD cards: USB-attached disks between 4GB and 256GB.
# The Pi 5's own boot media shows up as mmcblk0 or nvme0n1, never /dev/sd*,
# so filtering on TRAN=usb keeps this from ever touching the Pi 5 itself.
mapfile -t CARDS < <(lsblk -dpno NAME,TRAN,SIZE,TYPE | awk '$2=="usb" && $4=="disk" {print $1, $3}')

if [[ ${#CARDS[@]} -eq 0 ]]; then
  echo "No USB SD card readers detected. Check the hub and that cards are inserted."
  exit 1
fi

echo "Found ${#CARDS[@]} card(s):"
i=1
for line in "${CARDS[@]}"; do
  echo "  $i) $line"
  i=$((i + 1))
done
echo
echo "This will ERASE everything on the drives listed above."
read -rp "Type YES to continue: " CONFIRM
[[ "$CONFIRM" == "YES" ]] || { echo "Aborted, nothing was touched."; exit 1; }

# Build a per-card firstrun.sh and flash each card in the background.
i=1
PIDS=()
NAMES=()
for line in "${CARDS[@]}"; do
  DEV=$(awk '{print $1}' <<< "$line")
  HOST="${PREFIX}-$(printf '%02d' "$i")"
  SCRIPT="$WORKDIR/firstrun-$HOST.sh"
  sed "s/__HOSTNAME__/$HOST/g" "$TEMPLATE" > "$SCRIPT"

  echo "Flashing $DEV as $HOST ..."
  rpi-imager --cli --first-run-script "$SCRIPT" "$IMAGE" "$DEV" \
    > "$WORKDIR/log-$HOST.txt" 2>&1 &
  PIDS+=($!)
  NAMES+=("$HOST")
  i=$((i + 1))
done

# Wait for all flashes and report which ones failed, if any.
FAIL=0
for idx in "${!PIDS[@]}"; do
  if wait "${PIDS[$idx]}"; then
    echo "${NAMES[$idx]}: done"
  else
    echo "${NAMES[$idx]}: FAILED, see $WORKDIR/log-${NAMES[$idx]}.txt"
    FAIL=1
  fi
done

echo
echo "Logs kept at: $WORKDIR"
if [[ $FAIL -eq 0 ]]; then
  echo "All $((i - 1)) cards are flashed and ready. Insert them and power the boards on."
else
  echo "At least one card failed. Check the logs above before deploying."
fi
