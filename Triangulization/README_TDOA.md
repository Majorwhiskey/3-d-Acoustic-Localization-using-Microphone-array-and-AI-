# TDOA Triangulation for Sound Source Localization

This script implements Time Difference of Arrival (TDOA) method for triangulating sound source locations using multi-channel audio data.

## Features

- **TDOA Calculation**: Computes time differences of arrival between all microphone pairs using cross-correlation
- **Triangulation**: Estimates sound source location using least squares method
- **Visualization**: Generates plots showing:
  - TDOA matrix heatmap
  - TDOA values for all microphone pairs
  - Cross-correlation examples
  - 2D and 3D triangulation results

## Installation

Install required packages:

```bash
pip install -r requirements.txt
```

Or install individually:

```bash
pip install numpy scipy matplotlib
```

## Usage

Run the script with your WAV file:

```bash
python3 tdoa_triangulation.py
```

The script will:
1. Load the audio file (`/home/head-node-5/resp7chn/138/resp.wav`)
2. Calculate TDOA between all microphone pairs
3. Triangulate the sound source location
4. Generate visualization plots

## Output Files

- `tdoa_results.png`: TDOA matrix, bar chart, and cross-correlation plots
- `triangulation_result.png`: 2D and 3D views of microphone array and estimated source location

## Configuration

You can modify the script to:
- Change the audio file path
- Specify custom microphone positions
- Adjust speed of sound (default: 343 m/s)
- Change maximum delay search range

## How It Works

1. **TDOA Calculation**: Uses cross-correlation to find time delays between microphone pairs
2. **Distance Differences**: Converts TDOA to distance differences using speed of sound
3. **Triangulation**: Solves system of equations to find source position that minimizes error

## Notes

- Default microphone array assumes a circular configuration for 7 channels
- For other channel counts, a linear array is assumed
- You can specify custom microphone positions if known

