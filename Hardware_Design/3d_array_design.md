# 3D Microphone Array Design

## Physical Array Configuration

### Tetrahedral Geometry
The tetrahedral configuration provides optimal 3D coverage with 4 microphones:

```
        Mic 1 (Front)
           /|\
          / | \
         /  |  \
        /   |   \
   Mic 4    |    Mic 2
   (Left)   |    (Right)
        \   |   /
         \  |  /
          \ | /
           \|/
        Mic 3 (Back)
```

### Dimensions
- **Array radius**: 2.5 cm (adjustable based on application)
- **Microphone spacing**: 4.33 cm between adjacent mics
- **Height variation**: ±2.5 cm for 3D coverage

### 3D Printed Housing Design
```
Housing Specifications:
- Material: PLA or ABS
- Wall thickness: 2mm
- Microphone mounting: Press-fit or screw mount
- Cable management: Internal channels
- Mounting: Threaded insert for tripod mount
```

## Calibration Points
For accurate localization, define these reference points:
1. **Front**: 0° azimuth, 0° elevation
2. **Right**: 90° azimuth, 0° elevation  
3. **Back**: 180° azimuth, 0° elevation
4. **Left**: 270° azimuth, 0° elevation
5. **Up**: 0° azimuth, 90° elevation
6. **Down**: 0° azimuth, -90° elevation

## Environmental Considerations
- **Wind protection**: Foam windshields for outdoor use
- **Temperature stability**: Avoid direct sunlight
- **Vibration isolation**: Soft mounting for mechanical isolation
- **EMI shielding**: Metal housing for electromagnetic interference protection
