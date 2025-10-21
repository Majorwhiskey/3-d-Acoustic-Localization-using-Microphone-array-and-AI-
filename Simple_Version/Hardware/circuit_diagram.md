# Simple Circuit Design - ESP32 + Condenser Microphones

## Circuit Overview
This design uses analog condenser microphones with basic amplification stages, making it suitable for prototyping with limited components.

## Component List (Minimal Analog Components)
- ESP32 Development Board
- 4x Condenser Microphones (electret microphones)
- 4x 10kΩ resistors (microphone bias)
- 4x 100nF capacitors (DC blocking)
- Breadboard and jumper wires

**Note**: No op-amps needed! Digital processing handles amplification and filtering.

## Circuit Connections

### Microphone Connections (Direct to ESP32)
```
Condenser Microphone Wiring:
- Positive terminal → 3.3V through 10kΩ resistor
- Negative terminal → GND
- Output → 100nF capacitor → ESP32 ADC pin (direct connection)

No op-amps needed! Digital processing provides:
- Multiple ADC sampling for noise reduction
- Digital high-pass filtering
- Moving average filtering
- Bandpass filtering
- Signal normalization
```

### ESP32 ADC Connections
```
Microphone 1 → GPIO 36 (ADC1_CH0)
Microphone 2 → GPIO 39 (ADC1_CH3)
Microphone 3 → GPIO 34 (ADC1_CH6)
Microphone 4 → GPIO 35 (ADC1_CH7)
```

## Power Supply (Simplified)
- ESP32: 3.3V from USB or external supply
- Microphones: 3.3V through 10kΩ bias resistors
- **No op-amps needed** - reduced power consumption

## Digital Signal Processing
1. **DC Blocking**: 100nF capacitors remove DC offset
2. **Noise Reduction**: Multiple ADC samples averaged
3. **Digital Filtering**: High-pass, low-pass, and bandpass filters
4. **Signal Enhancement**: Digital amplification and normalization
5. **ADC Range**: 0-3.3V for ESP32 ADC with 12-bit resolution

## Microphone Array Geometry
```
Simple 2D Configuration (adjustable):
Mic 1 (Front):  (0, 0)
Mic 2 (Right):  (5, 0) cm
Mic 3 (Back):   (0, 5) cm  
Mic 4 (Left):   (-5, 0) cm
```

## Calibration Points
- **Front**: 0° azimuth
- **Right**: 90° azimuth
- **Back**: 180° azimuth
- **Left**: 270° azimuth

## Troubleshooting
- **No signal**: Check microphone bias voltage
- **Low signal**: Increase op-amp gain
- **Noise**: Add bypass capacitors
- **Distortion**: Reduce gain or check power supply
