import os
from dataclasses import dataclass, field
from typing import Optional, List, Tuple


@dataclass
class AudioConfig:
    device_query: Optional[str] = "ReSpeaker"  # substring to match input device
    samplerate: int = 16000
    record_seconds: float = 2.0
    dtype: str = "float32"
    # If None, will use device's max input channels then slice channels_to_use
    requested_channels: Optional[int] = None
    # Use 1–4 only (exclude 0 and 5). Zero-based channel indices on device.
    channels_to_use: Tuple[int, int, int, int] = (1, 2, 3, 4)
    countdown_seconds: int = 3
    blocksize: int = 0  # 0 lets PortAudio choose optimal


@dataclass
class GeometryConfig:
    # Planar circular 4-mic subset; angles in degrees for channels 1–4
    # Radius in meters (tune to your hardware; ReSpeaker 6-mic circle ~4.3 cm)
    radius_m: float = 0.043
    mic_angles_deg: Tuple[float, float, float, float] = (0.0, 90.0, 180.0, 270.0)
    speed_of_sound: float = 343.0  # m/s at ~20°C


@dataclass
class SaveConfig:
    enable_save_audio: bool = True
    enable_save_results: bool = True
    output_dir: str = os.path.join(os.getcwd(), "outputs")


@dataclass
class PlotConfig:
    show_plots: bool = True
    interactive_3d: bool = False  # if True, use plotly; else use mpl 3d


@dataclass
class DetectionConfig:
    # Minimum correlation peak quality (peak / (mean + std))
    # Lower values = more sensitive, higher values = less false positives
    # Set very low to allow almost all detections - can filter later by confidence
    min_correlation_quality: float = 0.8
    
    # Minimum RMS energy threshold (normalized)
    # Lower values = more sensitive to quiet sounds
    # Set very low to allow almost all detections
    min_rms_energy: float = 0.0001
    
    # Minimum confidence threshold (0.0-1.0) to report a detection
    # Set to 0.0 to always report detections (dashboard can filter by confidence visually)
    # Lower values = more detections (including weak ones)
    # Higher values = only strong, clear detections
    min_confidence_threshold: float = 0.0
    
    # Always broadcast DOA updates even if confidence is low (let dashboard decide)
    always_broadcast: bool = True
    
    # Enable debug output to see signal quality metrics
    enable_debug: bool = True


@dataclass
class AppConfig:
    audio: AudioConfig = field(default_factory=AudioConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    saving: SaveConfig = field(default_factory=SaveConfig)
    plot: PlotConfig = field(default_factory=PlotConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)


