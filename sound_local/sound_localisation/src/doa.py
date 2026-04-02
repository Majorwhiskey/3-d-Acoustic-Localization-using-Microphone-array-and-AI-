from typing import Tuple, List, Optional
import numpy as np
from scipy.signal import fftconvolve
from scipy.optimize import least_squares
from math import atan2, degrees


def _gcc_phat(sig: np.ndarray, ref: np.ndarray, fs: int, max_tau: float, interp: int = 16) -> Tuple[float, float]:
    """
    GCC-PHAT between two signals to estimate time delay (seconds).
    Limits search to +/- max_tau.
    Returns (tau, peak_quality) where peak_quality is the normalized correlation peak value.
    """
    n = sig.shape[0] + ref.shape[0]
    nfft = 1
    while nfft < n:
        nfft <<= 1
    SIG = np.fft.rfft(sig, n=nfft)
    REF = np.fft.rfft(ref, n=nfft)
    R = SIG * np.conj(REF)
    R /= np.maximum(np.abs(R), 1e-12)
    cc = np.fft.irfft(R, n=nfft * interp)
    max_shift = int(interp * fs * max_tau)
    cc = np.concatenate((cc[-max_shift:], cc[: max_shift + 1]))
    cc_abs = np.abs(cc)
    peak_idx = np.argmax(cc_abs)
    peak_value = cc_abs[peak_idx]
    # Normalize peak quality: peak / (mean + std) to measure how distinct the peak is
    mean_cc = np.mean(cc_abs)
    std_cc = np.std(cc_abs)
    peak_quality = peak_value / max(mean_cc + std_cc, 1e-12)
    shift = peak_idx - max_shift
    tau = shift / float(interp * fs)
    return tau, peak_quality


def build_mic_positions(radius_m: float, angles_deg: Tuple[float, float, float, float]) -> np.ndarray:
    """
    Returns mic positions (4, 3) in meters on XY plane (z=0).
    """
    positions = []
    for ang in angles_deg:
        theta = np.deg2rad(ang)
        x = radius_m * np.cos(theta)
        y = radius_m * np.sin(theta)
        positions.append([x, y, 0.0])
    return np.array(positions, dtype=np.float64)


def compute_signal_quality(signals: np.ndarray) -> Tuple[float, float]:
    """
    Compute signal quality metrics.
    Returns (rms_energy, peak_to_mean_ratio)
    - rms_energy: RMS energy across all channels (normalized)
    - peak_to_mean_ratio: Ratio of peak signal to mean signal
    """
    # Compute RMS energy per channel
    rms_per_channel = np.sqrt(np.mean(np.square(signals), axis=0))
    rms_energy = float(np.mean(rms_per_channel))
    
    # Compute peak-to-mean ratio as a quality indicator
    abs_signals = np.abs(signals)
    peak_value = np.max(abs_signals)
    mean_value = np.mean(abs_signals)
    peak_to_mean = peak_value / max(mean_value, 1e-12)
    
    return rms_energy, float(peak_to_mean)


def estimate_doa_az_el(
    signals: np.ndarray,
    fs: int,
    mic_positions_m: np.ndarray,
    speed_of_sound: float = 343.0,
    min_correlation_quality: float = 0.8,
    min_rms_energy: float = 0.0001,
    enable_debug: bool = False,
) -> Tuple[float, float, float]:
    """
    Estimate azimuth and elevation using pairwise TDOA + LS direction fit.
    signals: (N, 4)
    mic_positions_m: (4, 3)
    min_correlation_quality: Minimum peak quality threshold (default 1.5)
    min_rms_energy: Minimum RMS energy threshold (default 0.001)
    Returns (azimuth_deg, elevation_deg, confidence) where confidence is 0.0-1.0
    """
    assert signals.shape[1] == 4, "Expected 4 channels"
    assert mic_positions_m.shape == (4, 3)

    # Check signal quality first
    rms_energy, peak_to_mean = compute_signal_quality(signals)
    
    # Debug output
    if enable_debug:
        print(f"[DEBUG] RMS Energy: {rms_energy:.6f} (threshold: {min_rms_energy:.6f}), Peak-to-Mean: {peak_to_mean:.2f}")
    
    # If signal is too weak, return low confidence but still compute DOA
    # This allows tracking even weak signals
    signal_too_weak = rms_energy < min_rms_energy
    if signal_too_weak and enable_debug:
        print(f"[DEBUG] Signal weak (RMS: {rms_energy:.6f} < {min_rms_energy:.6f}), but continuing...")

    # High-level: for all unique pairs, GCC-PHAT TDOA, then solve A u = b (LS), u=unit vector.
    pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
    max_mic_spacing = np.max(
        [np.linalg.norm(mic_positions_m[j] - mic_positions_m[i]) for i, j in pairs]
    )
    max_tau = max_mic_spacing / speed_of_sound

    tdoas = []
    A_rows = []
    correlation_qualities = []
    for i, j in pairs:
        tau_ij, peak_quality = _gcc_phat(signals[:, i], signals[:, j], fs, max_tau=max_tau)
        tdoas.append(tau_ij)
        correlation_qualities.append(peak_quality)
        diff = (mic_positions_m[j] - mic_positions_m[i]) / speed_of_sound  # shape (3,)
        A_rows.append(diff)
    
    # Check correlation quality - if peaks are too weak, likely just noise
    avg_correlation_quality = float(np.mean(correlation_qualities))
    if enable_debug:
        print(f"[DEBUG] Avg Correlation Quality: {avg_correlation_quality:.3f} (threshold: {min_correlation_quality:.3f})")
    
    # Don't reject based on correlation quality - compute DOA anyway and use confidence
    correlation_too_low = avg_correlation_quality < min_correlation_quality
    if correlation_too_low and enable_debug:
        print(f"[DEBUG] Correlation quality low ({avg_correlation_quality:.3f} < {min_correlation_quality:.3f}), but continuing...")
    
    A = np.vstack(A_rows)  # (num_pairs, 3)
    b = np.array(tdoas)  # (num_pairs,)

    # Least squares for direction vector
    u, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)  # shape (3,)
    if np.allclose(u, 0) or rank < 3:
        return 0.0, 0.0, 0.0
    u = u / np.maximum(np.linalg.norm(u), 1e-12)

    # Convert to spherical: azimuth in XY, elevation from XY plane
    azimuth_rad = atan2(u[1], u[0])
    azimuth_deg = (degrees(azimuth_rad) + 360.0) % 360.0
    # Elevation: angle above XY plane
    elevation_rad = atan2(u[2], np.linalg.norm(u[:2]))
    elevation_deg = degrees(elevation_rad)
    
    # Compute confidence based on correlation quality and signal energy
    # Always give some confidence, even if below thresholds (just lower)
    # Normalize correlation quality - use min_correlation_quality as baseline
    if avg_correlation_quality > min_correlation_quality:
        corr_confidence = min(1.0, (avg_correlation_quality - min_correlation_quality) / 2.0)
    else:
        # Below threshold, but still give some confidence (scaled down)
        corr_confidence = max(0.0, avg_correlation_quality / min_correlation_quality) * 0.3
    
    # Normalize RMS energy
    if rms_energy > min_rms_energy:
        energy_confidence = min(1.0, rms_energy / 0.01)
    else:
        # Below threshold, but still give some confidence (scaled down)
        energy_confidence = max(0.0, rms_energy / min_rms_energy) * 0.3
    
    # Combined confidence - give more weight to correlation (it's more reliable)
    confidence = (corr_confidence * 0.6 + energy_confidence * 0.4)
    # Always give at least a small confidence value so we can track
    confidence = max(confidence, 0.05)
    
    if enable_debug:
        print(f"[DEBUG] Confidence: {confidence:.3f} (corr: {corr_confidence:.3f}, energy: {energy_confidence:.3f})")
    
    return azimuth_deg, elevation_deg, float(confidence)


def rough_distance_from_energy(signals: np.ndarray, ref_db_at_1m: float = -20.0) -> float:
    """
    Extremely rough distance proxy from RMS level (dBFS-like).
    Requires calibration to be meaningful. Returns meters.
    """
    eps = 1e-12
    rms = np.sqrt(np.mean(np.square(signals), axis=0)).mean()
    db = 20.0 * np.log10(max(rms, eps))
    # Simple inverse square model: level drop 6 dB per doubling distance
    # distance = 1m * 10^((ref_db - db)/20)
    distance_m = 10 ** ((ref_db_at_1m - db) / 20.0)
    return float(max(distance_m, 0.1))


def _compute_tdoas_vs_ref(
    signals: np.ndarray, fs: int, ref_index: int, speed_of_sound: float, mic_positions_m: np.ndarray
) -> Tuple[np.ndarray, float, float]:
    """
    Compute GCC-PHAT TDOAs relative to reference mic.
    Returns:
      - tdoas: array of shape (num_mics,) where tdoas[ref_index] == 0
      - max_tau: bound based on maximum baseline
      - avg_correlation_quality: average correlation peak quality
    """
    num_mics = signals.shape[1]
    # Determine max_tau from geometry (largest mic spacing)
    pair_indices: List[Tuple[int, int]] = [(i, j) for i in range(num_mics) for j in range(i + 1, num_mics)]
    max_mic_spacing = np.max(
        [np.linalg.norm(mic_positions_m[j] - mic_positions_m[i]) for i, j in pair_indices]
    )
    max_tau = max_mic_spacing / speed_of_sound

    tdoas = np.zeros(num_mics, dtype=np.float64)
    correlation_qualities = []
    for i in range(num_mics):
        if i == ref_index:
            tdoas[i] = 0.0
        else:
            tau, peak_quality = _gcc_phat(signals[:, i], signals[:, ref_index], fs, max_tau=max_tau)
            tdoas[i] = tau
            correlation_qualities.append(peak_quality)
    avg_correlation_quality = float(np.mean(correlation_qualities)) if correlation_qualities else 0.0
    return tdoas, max_tau, avg_correlation_quality


def estimate_position_from_tdoa(
    signals: np.ndarray,
    fs: int,
    mic_positions_m: np.ndarray,
    speed_of_sound: float = 343.0,
    ref_index: int = 0,
    initial_guess: Optional[np.ndarray] = None,
    min_correlation_quality: float = 1.5,
) -> Optional[np.ndarray]:
    """
    Estimate 3D source position using TDOAs relative to a reference mic via nonlinear least squares.
    signals: (N, M) with M=4
    mic_positions_m: (M, 3)
    min_correlation_quality: Minimum correlation quality threshold
    Returns position np.ndarray shape (3,) or None if solve fails or quality is too low.
    """
    assert signals.shape[1] == mic_positions_m.shape[0], "Signal/mic count mismatch"
    num_mics = mic_positions_m.shape[0]
    assert num_mics >= 4, "Need at least 4 microphones for Position from TDOA"

    tdoas_meas, _, avg_correlation_quality = _compute_tdoas_vs_ref(
        signals=signals, fs=fs, ref_index=ref_index, speed_of_sound=speed_of_sound, mic_positions_m=mic_positions_m
    )
    
    # Check correlation quality before attempting position estimation
    if avg_correlation_quality < min_correlation_quality:
        return None

    # Initial guess: small height above array center or along estimated direction if available
    if initial_guess is None:
        initial_guess = np.array([0.0, 0.0, 0.1], dtype=np.float64)

    def residuals(src_pos: np.ndarray) -> np.ndarray:
        # Predicted delays vs ref: (|s - m_i| - |s - m_ref|)/c
        distances = np.linalg.norm(src_pos[None, :] - mic_positions_m, axis=1)
        tau_pred = (distances - distances[ref_index]) / speed_of_sound
        res = tdoas_meas - tau_pred
        res[ref_index] = 0.0
        return res

    try:
        result = least_squares(residuals, initial_guess, method="trf", max_nfev=200, ftol=1e-10, xtol=1e-10)
        if not result.success:
            return None
        return result.x.astype(np.float64)
    except Exception:
        return None


