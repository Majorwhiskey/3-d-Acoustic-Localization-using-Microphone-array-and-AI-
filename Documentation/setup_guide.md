# Sound Source Localization Setup Guide

## Quick Start

### 1. Hardware Assembly

#### Required Components
- ESP32 Development Board
- 4x INMP411 MEMS Microphones
- Breadboard and jumper wires
- 3D printed microphone array housing
- Power supply (5V, 1A)
- Computer for data processing and storage

#### Wiring Instructions
1. **Connect microphones to ESP32:**
   ```
   INMP411 Pin → ESP32 Pin
   VDD → 3.3V
   GND → GND
   BCLK → GPIO 26 (shared)
   DOUT → GPIO 25/33/32/35 (Mic 1/2/3/4)
   ```

2. **Connect status LED:**
   ```
   LED Anode → GPIO 2 (with 220Ω resistor)
   LED Cathode → GND
   ```

### 2. Software Setup

#### ESP32 Arduino IDE Setup
1. Install Arduino IDE (latest version)
2. Install ESP32 board package:
   - Go to File → Preferences
   - Add URL: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Go to Tools → Board → Boards Manager
   - Search for "ESP32" and install

3. Install required libraries:
   - ESP32 I2S library (included)
   - WiFi library (included)

#### Upload Code
1. Open `ESP32_Code/sound_localization_esp32.ino`
2. Select board: ESP32 Dev Module
3. Set upload speed: 115200
4. Upload the code

#### MATLAB Setup
1. Install MATLAB (R2019b or later)
2. Required toolboxes:
   - Signal Processing Toolbox
   - Communications Toolbox (optional)
3. Add project folders to MATLAB path

### 3. System Calibration

#### Step 1: Hardware Calibration
1. **Measure microphone positions:**
   - Use calipers to measure exact positions
   - Record X, Y, Z coordinates in cm
   - Update `mic_positions` array in code

2. **Test audio connections:**
   - Upload test code to verify all microphones work
   - Check for proper audio levels
   - Adjust gain if needed

#### Step 2: Software Calibration
1. Run MATLAB calibration routine:
   ```matlab
   cd MATLAB_Processing
   run calibration_routine.m
   ```

2. Follow on-screen instructions:
   - Measure actual microphone positions
   - Test TDOA accuracy
   - Optimize parameters
   - Validate system performance

#### Step 3: System Validation
1. Test with known sound sources:
   - Place speaker at known positions
   - Verify localization accuracy
   - Adjust parameters if needed

### 4. Operation

#### Starting the System
1. **Power on ESP32**
2. **Connect to serial monitor** (115200 baud)
3. **Send commands:**
   ```
   START    - Start localization
   STATUS   - Check system status
   CALIBRATE - Recalibrate system
   STOP     - Stop localization
   ```

#### MATLAB Visualization
1. Run real-time visualization:
   ```matlab
   cd MATLAB_Processing
   real_time_visualization()
   ```

2. Monitor results:
   - 3D position tracking
   - Real-time plots
   - Performance metrics

#### Data Storage on Computer
1. **Serial data logging:**
   - ESP32 sends data via serial port
   - MATLAB captures and processes data
   - Data automatically saved to CSV files

2. **WiFi data transmission:**
   - ESP32 connects to your WiFi network
   - Data sent to computer via network
   - Real-time processing and storage

3. **Data export options:**
   ```matlab
   export_data()  % Exports to CSV files
   analyze_localization_performance()  % Performance analysis
   ```

## Troubleshooting

### Common Issues

#### No Audio Signal
- Check microphone connections
- Verify power supply (3.3V)
- Test with multimeter
- Check I2S configuration

#### Poor Localization Accuracy
- Recalibrate microphone positions
- Check for mechanical vibrations
- Verify array geometry
- Adjust correlation thresholds

#### ESP32 Crashes
- Check power supply (2A minimum)
- Reduce buffer size
- Add delay between operations
- Check for memory leaks

#### MATLAB Connection Issues
- Verify serial port settings
- Check baud rate (115200)
- Test with simple serial communication
- Update MATLAB serial drivers

### Performance Optimization

#### For Better Accuracy
1. **Improve microphone array:**
   - Use rigid mounting
   - Minimize vibrations
   - Equal cable lengths
   - Shielded cables

2. **Optimize processing:**
   - Increase sample rate
   - Use longer buffers
   - Apply better filtering
   - Use advanced algorithms

#### For Real-time Performance
1. **Reduce processing load:**
   - Smaller buffer sizes
   - Lower sample rates
   - Simplified algorithms
   - Hardware acceleration

2. **Optimize code:**
   - Use efficient data structures
   - Minimize memory allocation
   - Use DMA for I2S
   - Parallel processing

## Advanced Configuration

### Custom Microphone Array
To use a different array geometry:

1. **Update positions in ESP32 code:**
   ```cpp
   const float mic_positions[NUM_MICS][3] = {
     {x1, y1, z1},  // Mic 1
     {x2, y2, z2},  // Mic 2
     {x3, y3, z3},  // Mic 3
     {x4, y4, z4}   // Mic 4
   };
   ```

2. **Update MATLAB calibration:**
   ```matlab
   mic_positions = [
     x1, y1, z1;
     x2, y2, z2;
     x3, y3, z3;
     x4, y4, z4
   ];
   ```

### Custom Processing Parameters
Adjust these parameters for your application:

```cpp
// ESP32 parameters
#define SAMPLE_RATE 16000        // Higher = better accuracy
#define BUFFER_SIZE 1024         // Larger = better accuracy
#define MIN_CORRELATION 0.3     // Higher = more selective
```

```matlab
% MATLAB parameters
fs = 16000;                     % Sample rate
window_size = 512;              % FFT window size
overlap = 0.5;                  % Window overlap
min_correlation = 0.3;          % Correlation threshold
```

## Data Analysis

### Exporting Data
1. **From ESP32:**
   - Data saved to SD card
   - CSV format with timestamps
   - Includes TDOA values

2. **From MATLAB:**
   ```matlab
   export_data()  % Exports to CSV files
   ```

### Analysis Tools
1. **Performance analysis:**
   ```matlab
   analyze_localization_performance()
   ```

2. **Generate reports:**
   ```matlab
   generate_performance_report()
   ```

## Safety Considerations

### Electrical Safety
- Use proper power supplies
- Check voltage levels (3.3V for microphones)
- Avoid short circuits
- Use appropriate fuses

### Mechanical Safety
- Secure mounting to prevent falls
- Avoid sharp edges
- Use appropriate materials
- Check for loose connections

### Software Safety
- Implement error handling
- Use watchdog timers
- Add safety checks
- Monitor system health

## Support and Resources

### Documentation
- ESP32 I2S documentation
- INMP411 datasheet
- MATLAB signal processing guides
- Audio processing tutorials

### Community
- ESP32 forums
- Arduino community
- MATLAB Central
- Audio processing groups

### Updates
- Check for firmware updates
- Update MATLAB toolboxes
- Monitor for bug fixes
- Contribute improvements
