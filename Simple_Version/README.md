# Simple Sound Localization - ESP32 + Condenser Microphones

## Overview
This is a simplified version of the sound source localization system using basic components:
- ESP32 Development Board
- 4x Condenser Microphones (standard electret microphones)
- Breadboard and jumper wires

## Key Differences from Full Version
- Uses **analog condenser microphones** instead of I2S digital microphones
- **Simplified circuit** with basic amplifier stages
- **Basic signal processing** optimized for ESP32 capabilities
- **Minimal components** for easy prototyping

## Hardware Requirements
- ESP32 Development Board
- 4x Condenser Microphones (electret microphones)
- 4x 10kΩ resistors (for microphone bias)
- 4x 100nF capacitors (for DC blocking)
- 4x LM358 op-amps (for signal amplification)
- Breadboard and jumper wires
- Power supply (5V, 1A)

## Features
- Real-time sound source localization
- 3D position estimation (azimuth and elevation)
- Serial data output for computer processing
- Simple calibration routine
- Basic visualization tools

## Quick Start
1. Build the circuit as per `circuit_diagram.md`
2. Upload the ESP32 code
3. Run the MATLAB processing script
4. Start localization and monitor results

## Limitations
- Lower accuracy compared to I2S microphones
- Requires manual calibration
- Limited to basic signal processing
- Suitable for educational and prototyping purposes
