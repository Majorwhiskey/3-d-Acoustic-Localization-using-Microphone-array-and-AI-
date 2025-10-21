# Simple Setup Guide - ESP32 + Condenser Microphones

## Quick Start

### 1. Hardware Assembly

#### Required Components (Minimal)
- ESP32 Development Board
- 4x Condenser Microphones (electret microphones)
- 4x 10kΩ resistors (microphone bias)
- 4x 100nF capacitors (DC blocking)
- Breadboard and jumper wires
- Power supply (5V, 1A)

**Advantage**: No op-amps needed! Digital processing handles all signal conditioning.

#### Circuit Assembly (Simplified)
1. **Connect microphones directly to ESP32:**
   ```
   Condenser Mic → Breadboard
   Positive → 3.3V through 10kΩ resistor
   Negative → GND
   Output → 100nF capacitor → ESP32 ADC pin (direct connection)
   ```

2. **Connect to ESP32 (no op-amps needed):**
   ```
   Mic 1 → GPIO 36 (ADC1_CH0)
   Mic 2 → GPIO 39 (ADC1_CH3)
   Mic 3 → GPIO 34 (ADC1_CH6)
   Mic 4 → GPIO 35 (ADC1_CH7)
   ```

**Benefits of direct connection:**
- Reduced analog noise
- Fewer components
- Lower power consumption
- Digital processing handles amplification and filtering

### 2. Software Setup

#### ESP32 Arduino IDE Setup
1. Install Arduino IDE
2. Install ESP32 board package
3. Upload `simple_localization.ino`

#### MATLAB Setup
1. Install MATLAB (R2019b or later)
2. Run `simple_visualization.m`

### 3. System Calibration

#### Basic Calibration
1. **Test microphone signals:**
   - Send `TEST` command to ESP32
   - Check signal levels in serial monitor
   - Adjust op-amp gain if needed

2. **Position calibration:**
   - Place sound source at known positions
   - Record azimuth readings
   - Adjust microphone positions if needed

#### Calibration Commands
```
TEST - Test microphone signals
CALIBRATE - Start calibration routine
STATUS - Check system status
```

### 4. Operation

#### Starting the System
1. **Power on ESP32**
2. **Connect to serial monitor** (115200 baud)
3. **Send commands:**
   ```
   START    - Start localization
   STOP     - Stop localization
   STATUS   - Check system status
   CALIBRATE - Recalibrate system
   ```

#### MATLAB Visualization
1. Run visualization script:
   ```matlab
   cd Simple_Version/MATLAB_Processing
   simple_visualization()
   ```

2. Monitor results:
   - 2D position tracking
   - Real-time plots
   - Confidence monitoring

## Troubleshooting

### Common Issues

#### No Audio Signal
- Check microphone bias voltage (3.3V)
- Verify direct ADC connections
- Test with multimeter
- Check power supply
- Use `TEST` command to check ADC readings

#### Poor Localization Accuracy
- Recalibrate microphone positions
- Check for mechanical vibrations
- Verify array geometry
- Adjust signal levels

#### ESP32 Crashes
- Check power supply (1A minimum)
- Reduce buffer size
- Add delay between operations
- Check for loose connections

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
   - Shielded connections

2. **Optimize digital processing:**
   - Adjust filter coefficients
   - Increase ADC sampling
   - Improve calibration
   - Use advanced algorithms

#### For Real-time Performance
1. **Reduce processing load:**
   - Smaller buffer sizes
   - Lower sample rates
   - Simplified algorithms
   - Efficient data structures

2. **Optimize code:**
   - Use efficient ADC reading
   - Minimize memory allocation
   - Reduce serial output
   - Parallel processing

## Limitations

### Hardware Limitations
- **Lower accuracy** compared to I2S microphones
- **Analog noise** from op-amps
- **Limited frequency response** of condenser mics
- **Manual calibration** required

### Software Limitations
- **Basic algorithms** for ESP32 capabilities
- **2D localization** only (no elevation)
- **Simplified signal processing**
- **Limited real-time performance**

### Recommended Improvements
1. **Better microphones** (I2S digital)
2. **Improved amplification** (instrumentation amps)
3. **Advanced algorithms** (FFT-based processing)
4. **3D localization** (elevation angle)

## Data Storage

### Computer-Based Storage
1. **Serial data logging:**
   - ESP32 sends data via serial port
   - MATLAB captures and processes data
   - Data automatically saved to CSV files

2. **Data export options:**
   ```matlab
   exportData()  % Exports to CSV files
   createSummaryReport()  % Performance analysis
   ```

3. **Generated files:**
   - `simple_localization_data_YYYYMMDD_HHMMSS.csv`
   - `simple_localization_report_YYYYMMDD_HHMMSS.txt`

## Applications

### Suitable Applications
- **Educational purposes** - Learning sound localization
- **Prototyping** - Testing concepts
- **Basic research** - Simple experiments
- **Hobby projects** - DIY sound tracking

### Not Recommended For
- **High accuracy** applications
- **Professional** sound systems
- **Real-time** critical applications
- **Commercial** products

## Support

### Documentation
- Circuit diagrams with step-by-step instructions
- Code examples with comments
- Troubleshooting guides
- Performance optimization tips

### Community
- Arduino forums for ESP32 support
- MATLAB Central for signal processing
- Electronics forums for circuit design
- DIY communities for project sharing
