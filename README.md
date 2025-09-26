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
