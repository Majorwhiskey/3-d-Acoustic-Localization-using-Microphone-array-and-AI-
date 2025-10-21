# Hardware Setup - INMP411 Microphone Array with ESP32

## Circuit Connections

### INMP411 Microphone Pinout
```
INMP411 Pin Configuration:
Pin 1: VDD (3.3V)
Pin 2: L/R (Left/Right channel select - connect to GND for mono)
Pin 3: GND
Pin 4: WS (Word Select - not used in I2S mode)
Pin 5: BCLK (Bit Clock)
Pin 6: DOUT (Data Output)
Pin 7: SEL (Select - connect to GND)
Pin 8: GND
```

### ESP32 Connections

#### Microphone Array Configuration (4 INMP411)
```
Microphone 1 (Front):
- VDD → ESP32 3.3V
- GND → ESP32 GND
- BCLK → GPIO 26
- DOUT → GPIO 25

Microphone 2 (Right):
- VDD → ESP32 3.3V
- GND → ESP32 GND
- BCLK → GPIO 26 (shared)
- DOUT → GPIO 33

Microphone 3 (Back):
- VDD → ESP32 3.3V
- GND → ESP32 GND
- BCLK → GPIO 26 (shared)
- DOUT → GPIO 32

Microphone 4 (Left):
- VDD → ESP32 3.3V
- GND → ESP32 GND
- BCLK → GPIO 26 (shared)
- DOUT → GPIO 35
```

### Additional Components
```
Status LED:
- Anode → GPIO 2 (with 220Ω resistor)
- Cathode → GND

Optional: WiFi Antenna
- For better wireless communication range
```

## Microphone Array Geometry

### Tetrahedral Configuration
```
Microphone Positions (in cm from center):
Mic 1 (Front):  (0, 0, 2.5)
Mic 2 (Right):  (2.5, 0, 0)
Mic 3 (Back):   (0, 0, -2.5)
Mic 4 (Left):   (-2.5, 0, 0)
```

### Coordinate System
- X-axis: Left-Right (positive = right)
- Y-axis: Up-Down (positive = up)
- Z-axis: Front-Back (positive = front)

## Power Requirements
- ESP32: 3.3V, ~240mA
- 4x INMP411: 3.3V, ~4mA each
- Total: ~260mA (use 5V 1A power supply with 3.3V regulator)

## PCB Layout Recommendations
1. Keep microphone traces short and equal length
2. Use ground plane for noise reduction
3. Separate analog and digital power domains
4. Add decoupling capacitors (100nF) near each microphone
5. Use shielded cables for microphone connections if possible
