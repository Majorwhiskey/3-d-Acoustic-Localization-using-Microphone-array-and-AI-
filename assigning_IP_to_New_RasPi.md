# 🧑‍💻 Headless Raspberry Pi Static IP Configuration

This guide outlines the process for locating a newly networked Raspberry Pi (Raspi) and setting a permanent static IP address and hostname, ensuring reliable SSH access for headless operation.
## Prerequisites

  Raspberry Pi running Raspberry Pi OS.

  The Pi is connected to a network (wired is highly recommended).

  A host machine (e.g., your laptop) connected to the same network.

## Step 1: Locate the Raspberry Pi's Current IP Address

Before you can SSH in, you need to find the address the Raspi was assigned by DHCP (Dynamic Host Configuration Protocol).

Run the nmap command on your host machine to scan the network subnet for active devices:
```bash
nmap -sP 10.42.0.1/24
```
Note: Replace 10.42.0.1/24 with your network's specific subnet if it's different (e.g., 192.168.1.1/24). Look for the device named raspberrypi or a similar host name in the output.

## Step 2: Establish Remote Access (SSH)

Once the IP is identified (e.g., 10.42.0.xxx), use the ssh command to connect. The default username is pi.
```bash
ssh pi@10.42.0.xxx
```
## Step 3: Configure Hostname (Optional but Recommended)

It's best practice to give your device a descriptive name.

### Replace NEWNAME with your desired hostname (e.g., 'respeaker-hub')

```bash
sudo hostnamectl set-hostname NEWNAME
```
## Step 4: Ensure dhcpcd is Active

The dhcpcd service is the standard tool on Raspberry Pi OS for managing static and dynamic IP addresses. Ensure it is installed and running.

```bash
sudo apt install dhcpcd dhcpcd5
sudo systemctl enable dhcpcd
sudo systemctl start dhcpcd
```

## Step 5: Configure the Static IP Address

Edit the configuration file for dhcpcd. This is where you override the dynamic IP assignment.
```bash
sudo nano /etc/dhcpcd.conf
```
Add the following block to the end of the file. This example sets the IP to 172.16.0.88, which is excellent for a private, dedicated network segment.
```IN THE FILE
interface eth0
    static ip_address=172.16.0.88/24
    static routers=172.16.0.1
    static domain_name_servers=8.8.8.8 8.8.4.4
```

  interface eth0: Specifies the wired network connection. Use wlan0 for Wi-Fi.

  static ip_address: The permanent address and subnet mask (/24).

  static routers: The gateway IP (your router).

  static domain_name_servers: Public DNS servers (Google DNS in this case).

Press Ctrl + X, then Y to save, and Enter to confirm.

## Step 6: Disable Conflicting Network Services (Crucial)

If you are setting a static IP using dhcpcd, it's vital to disable NetworkManager if it is present and running, as it can conflict and override your static configuration.
```bash
sudo systemctl stop NetworkManager
sudo systemctl disable NetworkManager
```

## Step 7: Apply and Verify

To make the changes permanent, reboot the device. Once rebooted, the Raspi will use the new static IP address.
```bash
sudo reboot
```
You can then verify the change by trying to SSH into the new static IP: ssh pi@172.16.0.88.
