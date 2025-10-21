# Sound Source Localization using INMP411 Microphone Array and ESP32

## Project Overview
This project implements real-time 3D sound source localization using an INMP411 microphone array connected to an ESP32 microcontroller. The system uses time-based triangulation to determine the azimuth and elevation angles of sound sources in 3D space.

## Hardware Requirements
- ESP32 Development Board
- 4x INMP411 MEMS Microphones
- MicroSD Card Module (for data logging)
- Breadboard and jumper wires
- 3D printed microphone array housing

## Project Structure
```
├── ESP32_Code/           # Arduino code for ESP32
├── MATLAB_Processing/    # MATLAB signal processing scripts
├── Hardware_Design/      # Circuit diagrams and 3D models
├── Documentation/        # Setup guides and theory
└── Test_Data/           # Sample audio files for testing
```

## System Architecture

### 1. Hardware Layer
- **Microphone Array**: 4 INMP411 microphones arranged in a tetrahedral configuration
- **ESP32**: Data acquisition and initial processing
- **Communication**: WiFi/Serial for data transmission

### 2. Signal Processing Pipeline
1. **Data Acquisition**: Simultaneous sampling from 4 microphones
2. **Preprocessing**: Filtering, normalization, and noise reduction
3. **Time Delay Estimation**: Cross-correlation analysis
4. **Triangulation**: 3D position calculation using TDOA
5. **Output**: Azimuth and elevation angles

### 3. Localization Algorithm
- **TDOA (Time Difference of Arrival)**: Calculate time delays between microphones
- **Triangulation**: Use known microphone positions and time delays
- **3D Positioning**: Convert to azimuth and elevation angles

## Quick Start
1. Upload the ESP32 Arduino code
2. Connect the microphone array as per wiring diagram
3. Run the MATLAB processing script
4. Calibrate the system using the provided calibration routine

## Features
- Real-time 3D sound source localization
- Configurable microphone array geometry
- Data logging and visualization
- Calibration routines for accurate positioning
- Compatible with ESP32's processing capabilities
