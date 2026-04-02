#!/usr/bin/env python3
"""
Basic TDOA Triangulation for Sound Source Localization
========================================================
Simple implementation using Time Difference of Arrival (TDOA) method
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import correlate
from scipy.optimize import minimize
import os

# Load audio file
print("Loading audio file...")
sample_rate, audio_data = wavfile.read('resp.wav')

# Convert to float if needed
if np.issubdtype(audio_data.dtype, np.integer):
    max_val = np.iinfo(audio_data.dtype).max
    audio_data = audio_data.astype(np.float64) / max_val

# Handle channels
if audio_data.ndim == 1:
    audio_data = audio_data[:, np.newaxis]

num_channels = audio_data.shape[1]
print(f"Sample rate: {sample_rate} Hz")
print(f"Number of channels: {num_channels}")
print(f"Duration: {audio_data.shape[0]/sample_rate:.2f} seconds")

# Normalize channels
for ch in range(num_channels):
    channel = audio_data[:, ch]
    channel = channel - np.mean(channel)
    if np.std(channel) > 0:
        channel = channel / np.std(channel)
    audio_data[:, ch] = channel

# Speed of sound (m/s)
c = 343.0

# GCC-PHAT based TDOA estimation (robust to reverberation)
def estimate_tdoa_gcc_phat(sig1: np.ndarray, sig2: np.ndarray, fs: int, max_tau: float = 0.05) -> float:
    n = sig1.shape[0] + sig2.shape[0]
    # Next power of 2 for efficient FFT
    nfft = 1 << (n - 1).bit_length()
    SIG1 = np.fft.rfft(sig1, n=nfft)
    SIG2 = np.fft.rfft(sig2, n=nfft)
    R = SIG1 * np.conj(SIG2)
    denom = np.abs(R)
    denom[denom == 0] = 1.0
    R /= denom
    cc = np.fft.irfft(R, n=nfft)
    # Shift so zero lag is at center
    cc = np.concatenate((cc[-(nfft//2):], cc[:(nfft//2)]))
    max_lag = int(max_tau * fs)
    lags = np.arange(-nfft//2, nfft//2)
    # Restrict to +/- max_lag
    mask = (lags >= -max_lag) & (lags <= max_lag)
    lags = lags[mask]
    cc = cc[mask]
    best_lag = lags[np.argmax(cc)]
    return best_lag / fs

# Calculate TDOA between microphone pairs
print("\nCalculating TDOA for all microphone pair combinations...")
all_pairs = []  # list of dicts: {i, j, tdoa}

# Use first microphone as reference
ref_mic = 0
max_delay_samples = int(0.05 * sample_rate)  # 50ms max delay

# Use a window of audio for better correlation (first 10 seconds)
window_samples = min(int(10 * sample_rate), len(audio_data))
audio_window = audio_data[:window_samples, :]

for i in range(num_channels):
    for j in range(i + 1, num_channels):
        tdoa = estimate_tdoa_gcc_phat(audio_window[:, i], audio_window[:, j], sample_rate, max_tau=0.05)
        all_pairs.append({"i": i, "j": j, "tdoa": tdoa})
        note = " (may be unreliable)" if abs(tdoa) > 0.05 else ""
        print(f"  Mic {i} -> Mic {j}: TDOA = {tdoa*1000:.2f} ms{note}")

# Setup microphone positions (circular array for 6+ channels, linear for fewer)
if num_channels >= 6:
    radius = 0.5  # meters
    angles = np.linspace(0, 2*np.pi, num_channels, endpoint=False)
    mic_positions = np.array([
        [radius * np.cos(angle), radius * np.sin(angle), 0.0]
        for angle in angles
    ])
else:
    spacing = 0.1  # meters
    mic_positions = np.array([
        [i * spacing, 0.0, 0.0] for i in range(num_channels)
    ])

print(f"\nMicrophone positions:")
for i, pos in enumerate(mic_positions):
    print(f"  Mic {i}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

# Triangulate source location
print("\nTriangulating source location...")

def objective(source_pos):
    """Minimize error in distance differences"""
    residuals = []
    ref_pos = mic_positions[ref_mic]
    
    # Build TDOA map relative to reference mic with correct sign
    tdoa_ref = {}
    for pair in all_pairs:
        i, j, t = pair["i"], pair["j"], pair["tdoa"]
        if i == ref_mic:
            tdoa_ref[j] = t
        elif j == ref_mic:
            tdoa_ref[i] = -t

    for i in range(1, num_channels):
        if i not in tdoa_ref:
            continue
        tdoa = tdoa_ref[i]
        # Only use reliable TDOA values (within reasonable range)
        if abs(tdoa) < 1e-6 or abs(tdoa) > 0.05:
            continue
        
        dist_i = np.linalg.norm(source_pos - mic_positions[i])
        dist_ref = np.linalg.norm(source_pos - ref_pos)
        expected_diff = tdoa * c
        residual = (dist_i - dist_ref) - expected_diff
        residuals.append(residual)
    
    if len(residuals) == 0:
        return 1e10  # Large error if no valid measurements
    return np.sum(np.array(residuals)**2)

# Initial guess: center of array
initial_guess = np.mean(mic_positions, axis=0)
result = minimize(objective, initial_guess, method='BFGS')
source_position = result.x

print(f"Estimated source position: [{source_position[0]:.3f}, "
      f"{source_position[1]:.3f}, {source_position[2]:.3f}] meters")

# Create output folder
output_folder = 'triangulation_plots'
os.makedirs(output_folder, exist_ok=True)
print(f"\nCreating plots in folder: {output_folder}/")

# Create plots separately
print("\nGenerating plots...")

# Plot 1: TDOA values (all pairs)
fig1, ax1 = plt.subplots(figsize=(12, 6))
pair_labels = [f"M{p['i']}-M{p['j']}" for p in all_pairs]
pair_values_ms = [p['tdoa'] * 1000.0 for p in all_pairs]
bars = ax1.bar(range(len(all_pairs)), pair_values_ms, color='steelblue', alpha=0.7)
ax1.set_title('TDOA Values (All Microphone Pairs)', fontsize=16, fontweight='bold')
ax1.set_xlabel('Microphone Pair', fontsize=12)
ax1.set_ylabel('TDOA (milliseconds)', fontsize=12)
ax1.set_xticks(range(len(all_pairs)))
ax1.set_xticklabels(pair_labels, rotation=90, ha='right')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color='black', linestyle='-', linewidth=0.5)

# Add value labels
for i, (bar, val) in enumerate(zip(bars, pair_values_ms)):
    height = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width()/2., height,
            f'{val:.2f}',
            ha='center', va='bottom' if height >= 0 else 'top', fontsize=9)
plt.tight_layout()
plt.savefig(os.path.join(output_folder, '1_tdoa_values.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 1_tdoa_values.png")

# Plot 2: Cross-correlation example (first pair in list)
if len(all_pairs) > 0:
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    window_samples = min(int(10 * sample_rate), len(audio_data))
    # Build a PHAT-weighted correlation for visualization of first pair
    pair0 = all_pairs[0]
    s1 = audio_data[:window_samples, pair0['i']]
    s2 = audio_data[:window_samples, pair0['j']]
    n = s1.shape[0] + s2.shape[0]
    nfft = 1 << (n - 1).bit_length()
    S1 = np.fft.rfft(s1, n=nfft)
    S2 = np.fft.rfft(s2, n=nfft)
    R = S1 * np.conj(S2)
    denom = np.abs(R)
    denom[denom == 0] = 1.0
    R /= denom
    cc = np.fft.irfft(R, n=nfft)
    cc = np.concatenate((cc[-(nfft//2):], cc[:(nfft//2)]))
    lags = np.arange(-nfft//2, nfft//2)
    time_lags = lags / sample_rate
    mask = np.abs(time_lags) <= 0.05
    if np.any(mask):
        cc = cc[mask]
        time_lags = time_lags[mask]
        ax2.plot(time_lags*1000, cc, 'b-', linewidth=2)
        if abs(pair0['tdoa']) <= 0.05:
            ax2.axvline(pair0['tdoa']*1000, color='red', linestyle='--', 
                        linewidth=2, label=f"TDOA: {pair0['tdoa']*1000:.2f} ms")
    ax2.set_title(f"PHAT Cross-Correlation: Mic {pair0['i']} vs Mic {pair0['j']}", 
                 fontsize=16, fontweight='bold')
    ax2.set_xlabel('Time Lag (ms)', fontsize=12)
    ax2.set_ylabel('Correlation', fontsize=12)
    ax2.grid(True, alpha=0.3)
    if abs(pair0['tdoa']) <= 0.05:
        ax2.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, '2_cross_correlation.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("  Saved: 2_cross_correlation.png")

# Plot 3: TDOA Matrix (from all pairs)
fig3, ax3 = plt.subplots(figsize=(8, 7))
tdoa_matrix = np.zeros((num_channels, num_channels))
for p in all_pairs:
    i, j, t = p['i'], p['j'], p['tdoa']
    tdoa_matrix[i, j] = t
    tdoa_matrix[j, i] = -t
im = ax3.imshow(tdoa_matrix * 1000, cmap='RdBu_r', aspect='auto')
ax3.set_title('TDOA Matrix (ms)', fontsize=16, fontweight='bold')
ax3.set_xlabel('Microphone Index', fontsize=12)
ax3.set_ylabel('Microphone Index', fontsize=12)
plt.colorbar(im, ax=ax3, label='TDOA (ms)')
plt.tight_layout()
plt.savefig(os.path.join(output_folder, '3_tdoa_matrix.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 3_tdoa_matrix.png")

# Plot 4: Top View (XY plane)
fig4, ax4 = plt.subplots(figsize=(10, 10))
ax4.scatter(mic_positions[:, 0], mic_positions[:, 1], 
           s=200, c='blue', marker='s', label='Microphones', zorder=3)
ax4.scatter(source_position[0], source_position[1], 
           s=300, c='red', marker='*', label='Estimated Source', zorder=4)
for i, mic_pos in enumerate(mic_positions):
    ax4.plot([source_position[0], mic_pos[0]], 
            [source_position[1], mic_pos[1]], 
            'gray', linestyle='--', alpha=0.3, linewidth=1)
    ax4.text(mic_pos[0], mic_pos[1], f' M{i}', fontsize=10)
ax4.set_title('Top View (XY Plane)', fontsize=16, fontweight='bold')
ax4.set_xlabel('X (meters)', fontsize=12)
ax4.set_ylabel('Y (meters)', fontsize=12)
ax4.grid(True, alpha=0.3)
ax4.legend(fontsize=11)
ax4.axis('equal')
plt.tight_layout()
plt.savefig(os.path.join(output_folder, '4_top_view.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 4_top_view.png")

# Plot 4a: PHAT Cross-Correlation for all pairs (saved individually)
pair_corr_dir = os.path.join(output_folder, 'pairwise_correlations')
os.makedirs(pair_corr_dir, exist_ok=True)
for p in all_pairs:
    i, j, t = p['i'], p['j'], p['tdoa']
    figc, axc = plt.subplots(figsize=(10, 5))
    s1 = audio_data[:window_samples, i]
    s2 = audio_data[:window_samples, j]
    n = s1.shape[0] + s2.shape[0]
    nfft = 1 << (n - 1).bit_length()
    S1 = np.fft.rfft(s1, n=nfft)
    S2 = np.fft.rfft(s2, n=nfft)
    R = S1 * np.conj(S2)
    denom = np.abs(R)
    denom[denom == 0] = 1.0
    R /= denom
    cc = np.fft.irfft(R, n=nfft)
    cc = np.concatenate((cc[-(nfft//2):], cc[:(nfft//2)]))
    lags = np.arange(-nfft//2, nfft//2)
    time_lags = lags / sample_rate
    mask = np.abs(time_lags) <= 0.05
    if np.any(mask):
        axc.plot(time_lags[mask]*1000, cc[mask], 'b-', linewidth=1.5)
        if abs(t) <= 0.05:
            axc.axvline(t*1000, color='red', linestyle='--', linewidth=2,
                        label=f"TDOA: {t*1000:.2f} ms")
    axc.set_title(f'PHAT Cross-Correlation: Mic {i} vs Mic {j}', fontsize=14, fontweight='bold')
    axc.set_xlabel('Time Lag (ms)')
    axc.set_ylabel('Correlation (PHAT)')
    axc.grid(True, alpha=0.3)
    if abs(t) <= 0.05:
        axc.legend()
    plt.tight_layout()
    outp = os.path.join(pair_corr_dir, f'corr_M{i}_M{j}.png')
    plt.savefig(outp, dpi=300, bbox_inches='tight')
    plt.close()

# Plot 4b: TDOA over time for (ref, a neighbor) using GCC-PHAT
if num_channels > 1:
    fig4b, ax4b = plt.subplots(figsize=(12, 4))
    win_sec = 1.0
    hop_sec = 0.5
    win = int(win_sec * sample_rate)
    hop = int(hop_sec * sample_rate)
    times = []
    tdoa_series = []
    for start in range(0, audio_data.shape[0] - win, hop):
        end = start + win
        t = (start + win // 2) / sample_rate
        neighbor = 1 if ref_mic != 1 else 2
        neighbor = min(neighbor, num_channels - 1)
        td = estimate_tdoa_gcc_phat(audio_data[start:end, ref_mic], audio_data[start:end, neighbor], sample_rate, max_tau=0.02)
        times.append(t)
        tdoa_series.append(td * 1000.0)
    ax4b.plot(times, tdoa_series, 'k-', linewidth=1.5)
    ax4b.set_title(f'TDOA over Time (Mic {ref_mic} vs Mic {neighbor})', fontsize=14, fontweight='bold')
    ax4b.set_xlabel('Time (s)')
    ax4b.set_ylabel('TDOA (ms)')
    ax4b.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, '4b_tdoa_over_time.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("  Saved: 4b_tdoa_over_time.png")

# Plot 5: Side View (XZ plane)
ax5 = plt.subplot(2, 3, 5)
ax5.scatter(mic_positions[:, 0], mic_positions[:, 2], 
           s=200, c='blue', marker='s', label='Microphones', zorder=3)
ax5.scatter(source_position[0], source_position[2], 
           s=300, c='red', marker='*', label='Estimated Source', zorder=4)
for mic_pos in mic_positions:
    ax5.plot([source_position[0], mic_pos[0]], 
            [source_position[2], mic_pos[2]], 
            'gray', linestyle='--', alpha=0.3, linewidth=1)
ax5.set_title('Side View (XZ Plane)', fontsize=16, fontweight='bold')
ax5.set_xlabel('X (meters)', fontsize=12)
ax5.set_ylabel('Z (meters)', fontsize=12)
ax5.grid(True, alpha=0.3)
ax5.legend()

# Plot 6: 3D View
fig6 = plt.figure(figsize=(10, 10))
ax6 = fig6.add_subplot(111, projection='3d')
ax6.scatter(mic_positions[:, 0], mic_positions[:, 1], mic_positions[:, 2], 
           s=200, c='blue', marker='s', label='Microphones')
ax6.scatter(source_position[0], source_position[1], source_position[2], 
           s=300, c='red', marker='*', label='Estimated Source')
for mic_pos in mic_positions:
    ax6.plot([source_position[0], mic_pos[0]], 
            [source_position[1], mic_pos[1]], 
            [source_position[2], mic_pos[2]], 
            'gray', linestyle='--', alpha=0.3, linewidth=1)
ax6.set_title('3D View', fontsize=16, fontweight='bold')
ax6.set_xlabel('X (meters)', fontsize=12)
ax6.set_ylabel('Y (meters)', fontsize=12)
ax6.set_zlabel('Z (meters)', fontsize=12)
ax6.legend(fontsize=11)
plt.tight_layout()
plt.savefig(os.path.join(output_folder, '6_3d_view.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 6_3d_view.png")

# Save CSV summary of TDOA values (all pairs)
summary_csv = os.path.join(output_folder, 'tdoa_summary.csv')
with open(summary_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['pair', 'mic_i', 'mic_j', 'tdoa_s', 'tdoa_ms', 'distance_diff_m'])
    for p in all_pairs:
        dd = p['tdoa'] * c
        writer.writerow([f"M{p['i']}-M{p['j']}", p['i'], p['j'], p['tdoa'], p['tdoa']*1000.0, dd])
print(f"  Saved: {os.path.basename(summary_csv)}")

print("\n" + "="*60)
print("TRIANGULATION COMPLETE")
print("="*60)
print(f"Source position: [{source_position[0]:.3f}, {source_position[1]:.3f}, {source_position[2]:.3f}] m")
print(f"Distance from center: {np.linalg.norm(source_position):.3f} m")
print(f"\nAll plots saved in folder: {output_folder}/")
print("="*60)

