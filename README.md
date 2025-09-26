# Localized Sound Final

This repository contains a Jupyter Notebook for sound localization using signal processing and/or machine learning techniques.

## Files

- `Localized_Sound_final.ipynb`: The main notebook containing the implementation.
- `README.md`: This file.

## How to Run

1. Open the notebook in Jupyter or Google Colab.
2. Install necessary dependencies if prompted.
3. Follow the cells sequentially to replicate the results.

## Requirements

- Python 3.8+
- Jupyter Notebook
- Libraries: `numpy`, `matplotlib`, `librosa`, `scipy`, etc.

## Author

Amogh Ukkadagatri

# Raspberry Pi OS Setup Guide in Ubuntu

---

## Step 1: Format the SD Card

Before flashing the operating system, the SD card must be formatted to ensure it's ready for a new installation.

1.  **Connect the SD Card:** Insert the microSD card into a USB reader and connect it to your computer.

2.  **Identify the Device:** Use the `lsblk` command to identify the correct device name for your SD card. In my case, it was `/dev/sda`.

3.  **Format the Card:** Run the following command to format the entire device with a FAT32 filesystem. **Note:** This command will erase all data on the card.

```bash
sudo mkfs.vfat /dev/sda
```
## Step 2: Flash the Raspberry Pi OS with Linux Commands

After formatting your SD card, you can flash the Raspberry Pi OS image directly from the command line on Linux. This method is an alternative to the graphical Raspberry Pi Imager.

-----

### 1\. Download the OS Image

First, you'll need to download the `.img` or `.zip` file of the Raspberry Pi OS. The latest versions are available on the official Raspberry Pi website. You can use `wget` to download it directly from the terminal.

```bash
wget https://downloads.raspberrypi.com/raspios_lite_armhf/images/raspios_lite_armhf-2023-05-03/2023-05-03-raspios-bullseye-armhf-lite.img.xz
```

*Note: This is an example URL. Always use the latest URL from the official site.*

### 2\. Decompress the Image

If the downloaded file is a `.zip` or `.xz` compressed archive, you need to decompress it. For `.xz` files, use the `unxz` command.

```bash
unxz 2023-05-03-raspios-bullseye-armhf-lite.img.xz
```

This will create the `.img` file, which is the raw disk image you'll flash.

### 3\. Identify the SD Card Device

You already know the device name from the previous step. Confirm it's the correct device with `lsblk`. **This step is critical to avoid overwriting the wrong drive.**

```bash
lsblk
```

Look for your SD card, which might be `/dev/sda` or `/dev/sdb`.

### 4\. Flash the Image with `dd`

The `dd` (disk duplicator) command is a powerful tool for low-level data copying. It's used here to write the disk image to the SD card.

```bash
sudo dd bs=4M if=/path/to/your/image.img of=/dev/sdX conv=fsync status=progress
```

  * `sudo`: Executes the command with root privileges, which are required for writing to a disk device.
  * `dd`: The command itself.
  * `bs=4M`: Sets the block size to 4 megabytes. This makes the transfer faster than the default 512 bytes.
  * `if=/path/to/your/image.img`: The **i**nput **f**ile, which is the OS image you downloaded. Replace the path with the actual location of your image file.
  * `of=/dev/sdX`: The **o**utput **f**ile. **Be extremely careful here.** Replace `/dev/sdX` with the device name of your SD card (e.g., `/dev/sda`). Do not include a partition number (like `sda1`), as you're writing to the entire disk.
  * `conv=fsync`: Ensures all cached writes are physically written to the device, preventing corruption.
  * `status=progress`: Displays a real-time progress report.

-----

### 5\. Verify the Write (Optional but Recommended)

Once `dd` finishes, all data has been written to the card. It's a good idea to verify the write by comparing the checksum of the image file to the checksum of the written device.

```bash
sha256sum /path/to/your/image.img
```

```bash
sudo dd if=/dev/sdX | sha256sum
```

  * The `dd` command reads the entire SD card and pipes the output to `sha256sum` to generate a checksum.
  * Compare the output of both commands. They should be identical.

Once verified, you're ready to proceed to the next step.


 
 
 
 
 ## Setup for Simultaneous_REcording_on_booting

## Step 1: Create the Recording Script

First, save your Python recording script (or the three Linux commands) into a single executable shell script. Let's call it start_recording.sh and place it in your home directory.

Example content for start_recording.sh:
```Bash

#!/bin/bash
# Navigate to the correct directory and run the Python script
/usr/bin/python3 /home/pi/recording_script.py
```
After creating the file, make it executable:
```Bash

chmod +x /home/pi/start_recording.sh
```
## Step 2: Create the Systemd Service File

This file tells the system what to do when it boots. Create a new service file named recording.service in the /etc/systemd/system/ directory.
Bash
```
sudo nano /etc/systemd/system/recording.service
```
Paste the following content into the file:
Ini, TOML
```
[Unit]
Description=Start multi-channel audio recording on boot
After=network.target

[Service]
ExecStart=/home/pi/start_recording.sh
User=pi
Type=simple
Restart=on-failure
WorkingDirectory=/home/pi/

[Install]
WantedBy=multi-user.target
``` 
   Description: A brief description of the service.

   ExecStart: The full path to the script that will be executed.

   User: Specifies the user account (pi) to run the script as.

   WantedBy=multi-user.target: This is the key line that ensures the service starts automatically when the system boots up.

## Step 3: Enable and Start the Service

Now, tell systemd to reload its configuration and enable your new service.
```Bash

sudo systemctl daemon-reload
sudo systemctl enable recording.service
```

After running these commands, your Raspberry Pi will start the recording script every time it boots. You will need to perform this entire process on each of your **'n'** units
