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

# Software Installation 💻

The software setup requires you to install drivers and verify the hardware is working correctly. You'll need to use the terminal on your Raspberry Pi.

## 1. Update your Raspberry Pi OS

First, ensure your Raspberry Pi's operating system is up to date. Open a terminal and run the following commands:
```Bash
sudo apt update
sudo apt upgrade
sudo reboot
```
The reboot command will restart your Pi to apply all the changes.

## 2. Install the ReSpeaker Driver

Seeed Studio, the maker of ReSpeaker, provides an installation script for its voice cards.

Clone the driver repository from GitHub:
```Bash
git clone https://github.com/respeaker/seeed-voicecard.git
cd seeed-voicecard
```
Run the installation script. The script will automatically detect your Pi's architecture.
```Bash
sudo ./install.sh
```
If you're using a 64-bit version of Raspberry Pi OS, some sources suggest using sudo ./install_arm64.sh instead, but the main install script should work on most systems.

## 3. Reboot your Pi once the installation is complete.
```Bash
    sudo reboot
 ```
 



 
 
 # Setup for Simultaneous_REcording_on_booting

## Step 1: Create the Simplified Python Script

First, create a new Python file named startup_message.py. This script will only send the UDP message and then exit.
```Bash
sudo nano /home/pi/startup_message.py
```
Paste the following content into the file. Replace <center-ip> with the actual IP address of your command center.
```Python
import socket
import subprocess

# --- Configuration ---
CENTER_IP = "<center-ip>"  # Replace with the IP of your command center
UDP_PORT = 5018
MESSAGE = "Pi is awake"

def send_udp_message(ip, port, message):
    """Sends a simple UDP message."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(message.encode('utf-8'), (ip, port))
        print(f"Sent UDP message to {ip}:{port}")
    except Exception as e:
        print(f"Error sending UDP message: {e}")

if __name__ == "__main__":
    send_udp_message(CENTER_IP, UDP_PORT, MESSAGE)
    # The script will exit after sending the message
```
After creating the file, make it executable:
```Bash
sudo chmod +x /home/pi/startup_message.py
```
## Step 2: Update the Systemd Service File

Now, you need to modify your service file to run the new script.
```Bash
sudo nano /etc/systemd/system/recording.service
```
Change the ExecStart line to point to the new Python script:
```Ini, TOML

[Unit]
Description=Send UDP message on boot
After=network-online.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/startup_message.py
User=pi
Type=simple
Restart=on-failure
WorkingDirectory=/home/pi/

[Install]
WantedBy=multi-user.target
```
Finally, tell systemd to use this new service.
```Bash
sudo systemctl daemon-reload
sudo systemctl enable recording.service
```
After rebooting each of your Raspberry Pis, the startup_message.py script will run automatically, and it will only send the UDP message.
