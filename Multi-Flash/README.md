# flash-fleet

Flash and customize several SD cards at once from a Raspberry Pi 5. Plug in
your cards through a USB hub, run one command, and walk away with a stack
of ready-to-boot Pi boards, each with its own hostname.

## What's in this folder

- `flash-fleet.sh`, the script that detects your SD cards and flashes them
- `firstrun-template.sh`, you'll create this yourself in step 2 below
- your OS image file, you'll add this yourself in step 3

## Requirements

- A Raspberry Pi 5 (or any Linux machine) with `rpi-imager` version 1.8 or later
- A USB hub with SD card readers, one per card you want to flash
- Raspberry Pi Imager installed: `sudo apt install rpi-imager`

Check your version with `rpi-imager --version`. If it's older than 1.8, run
`sudo apt update && sudo apt install --only-upgrade rpi-imager` first. The
`--first-run-script` flag this tool depends on doesn't exist in older builds.

## Setup, do this once

**1. Flash one card by hand, with full customization.**

Open Raspberry Pi Imager, choose your OS and one SD card, then click the
gear icon in the bottom right before writing. Fill in hostname, username,
password, enable SSH, add WiFi details, and set locale if you need it.
Write the image.

**2. Copy the customization script off that card.**

Don't boot the card yet. Plug it into a reader on your Pi 5 and copy the
`firstrun.sh` file sitting in its boot partition:

```
cp /media/$USER/bootfs/firstrun.sh ./firstrun-template.sh
```

Your mount path may differ. Run `lsblk` if you're not sure where the card
mounted.

**3. Make the hostname reusable.**

Open `firstrun-template.sh` in a text editor and find the line that sets
the hostname. Replace the hardcoded value with the exact text
`__HOSTNAME__`. This is the only edit you need to make. Every other
setting, meaning username, password, SSH, WiFi, stays exactly as you
configured it in step 1.

**4. Add your OS image.**

Copy your image file (`.img` or `.img.xz`) into this same folder, next to
`flash-fleet.sh`.

Your folder should now look like this:

```
flash-fleet/
├── flash-fleet.sh
├── firstrun-template.sh
├── README.md
└── raspios.img.xz
```

## Running it

Insert all your SD cards, then run:

```
sudo ./flash-fleet.sh raspios.img.xz pi-node
```

Replace `raspios.img.xz` with your image filename and `pi-node` with
whatever hostname prefix you want. The script will:

1. List every USB-attached SD card it finds and ask you to confirm
2. Give each card a unique hostname (`pi-node-01`, `pi-node-02`, and so on)
3. Flash and customize all cards at the same time
4. Report which cards succeeded and which failed, with logs for each

Type `YES` at the confirmation prompt only after you've checked the device
list. This step erases every drive listed.

## Timing

Flashing 8 cards at once through a single USB hub typically takes 10 to 15
minutes total, depending on card speed and image size, since the cards
share the hub's bandwidth. That's still far faster than flashing one card
at a time.

If you want to save time and trust your cards, open `flash-fleet.sh` and
add `--disable-verify` to the `rpi-imager` command. It skips the read-back
check after writing, roughly cutting flash time in half, but it also won't
catch a bad card.

## Safety notes

The script only targets USB-attached drives (`TRAN=usb` in `lsblk`). A
Pi 5's own boot media shows up as `mmcblk0` or `nvme0n1`, never `/dev/sd*`,
so the script can't write to it even by accident. Still, always read the
confirmation list before typing `YES`. That's your last check before
anything gets erased.

## Troubleshooting

**"rpi-imager not found"**
Install it with `sudo apt install rpi-imager`.

**"No USB SD card readers detected"**
Run `lsblk -dpno NAME,TRAN,SIZE,TYPE` yourself and check that your readers
show up with `TRAN` set to `usb`. Some cheap hubs or readers report
differently, in which case you'll need to adjust the filter in the script.

**A card fails partway through**
Check its log file in the temp directory the script prints at the end.
Most failures come from a bad card or a flaky USB connection, not the
script itself. Re-run the script with just that one card connected.
