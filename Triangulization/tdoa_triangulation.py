#!/usr/bin/env python3
"""
TDOA Triangulation for Sound Source Localization
=================================================
Uses Time Difference of Arrival (TDOA) method to locate sound sources
using multiple microphone channels.

This script:
1. Loads multi-channel audio data
2. Calculates TDOA between all microphone pairs using cross-correlation
3. Triangulates the sound source location using TDOA values
4. Plots TDOA results and source location

Requirements:
- numpy
- scipy
- matplotlib

Install with: pip install numpy scipy matplotlib
"""

import sys
import os

# Try to import required packages
try:
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.io import wavfile
    from scipy.signal import correlate, find_peaks
    from scipy.optimize import minimize
    import warnings
    warnings.filterwarnings('ignore')
except ImportError as e:
    print("Error: Missing required packages.")
    print("Please install required packages:")
    print("  pip install numpy scipy matplotlib")
    print("\nOr use the virtual environment:")
    print("  source /home/head-node-5/project/venv/bin/activate")
    print("  pip install numpy scipy matplotlib")
    sys.exit(1)

class TDOATriangulation:
    def __init__(self, audio_file, mic_positions=None, speed_of_sound=343.0):
        """
        Initialize TDOA Triangulation
        
        Parameters:
        -----------
        audio_file : str
            Path to multi-channel WAV file
        mic_positions : array-like, optional
            Array of microphone positions [[x1, y1, z1], [x2, y2, z2], ...]
            If None, assumes a default 7-channel array configuration
        speed_of_sound : float
            Speed of sound in m/s (default: 343 m/s at 20°C)
        """
        self.audio_file = audio_file
        self.speed_of_sound = speed_of_sound
        
        # Load audio data
        print(f"Loading audio file: {audio_file}")
        self.sample_rate, self.audio_data = wavfile.read(audio_file)
        
        # Convert to float if integer
        if np.issubdtype(self.audio_data.dtype, np.integer):
            max_val = np.iinfo(self.audio_data.dtype).max
            self.audio_data = self.audio_data.astype(np.float64) / max_val
        
        # Handle mono vs multi-channel
        if self.audio_data.ndim == 1:
            self.audio_data = self.audio_data[:, np.newaxis]
            self.num_channels = 1
        else:
            self.num_channels = self.audio_data.shape[1]
        
        print(f"Sample rate: {self.sample_rate} Hz")
        print(f"Number of channels: {self.num_channels}")
        print(f"Duration: {self.audio_data.shape[0] / self.sample_rate:.2f} seconds")
        
        # Set microphone positions
        if mic_positions is None:
            # Default circular array configuration for 7 channels
            if self.num_channels == 7:
                radius = 0.5  # meters
                angles = np.linspace(0, 2*np.pi, 7, endpoint=False)
                self.mic_positions = np.array([
                    [radius * np.cos(angle), radius * np.sin(angle), 0.0]
                    for angle in angles
                ])
            else:
                # Default linear array for other channel counts
                spacing = 0.1  # meters
                self.mic_positions = np.array([
                    [i * spacing, 0.0, 0.0] for i in range(self.num_channels)
                ])
        else:
            self.mic_positions = np.array(mic_positions)
            if len(self.mic_positions) != self.num_channels:
                raise ValueError(f"Number of microphone positions ({len(self.mic_positions)}) "
                               f"does not match number of channels ({self.num_channels})")
        
        # Normalize audio channels
        self.normalize_audio()
        
        # Storage for TDOA results
        self.tdoa_matrix = None
        self.tdoa_pairs = []
        
    def normalize_audio(self):
        """Normalize each channel to zero mean and unit variance"""
        for ch in range(self.num_channels):
            channel = self.audio_data[:, ch]
            channel = channel - np.mean(channel)
            if np.std(channel) > 0:
                channel = channel / np.std(channel)
            self.audio_data[:, ch] = channel
    
    def calculate_tdoa(self, ch1, ch2, max_delay_samples=None):
        """
        Calculate Time Difference of Arrival between two channels
        
        Parameters:
        -----------
        ch1, ch2 : int
            Channel indices
        max_delay_samples : int, optional
            Maximum delay to search (in samples)
            
        Returns:
        --------
        tdoa : float
            Time difference of arrival in seconds (positive means ch2 arrives after ch1)
        correlation : array
            Cross-correlation values
        lags : array
            Time lags in seconds
        """
        sig1 = self.audio_data[:, ch1]
        sig2 = self.audio_data[:, ch2]
        
        # Perform cross-correlation
        correlation = correlate(sig1, sig2, mode='full')
        lags_samples = np.arange(-len(sig2) + 1, len(sig1))
        lags_time = lags_samples / self.sample_rate
        
        # Limit search range if specified
        if max_delay_samples is not None:
            mask = np.abs(lags_samples) <= max_delay_samples
            correlation = correlation[mask]
            lags_samples = lags_samples[mask]
            lags_time = lags_time[mask]
        
        # Find peak correlation
        peak_idx = np.argmax(np.abs(correlation))
        tdoa = lags_time[peak_idx]
        max_correlation = correlation[peak_idx]
        
        return tdoa, correlation, lags_time
    
    def calculate_all_tdoa(self, max_delay_ms=50):
        """
        Calculate TDOA between all microphone pairs
        
        Parameters:
        -----------
        max_delay_ms : float
            Maximum expected delay in milliseconds
        """
        print(f"\nCalculating TDOA for all microphone pairs...")
        max_delay_samples = int(max_delay_ms * self.sample_rate / 1000)
        
        # Initialize TDOA matrix
        self.tdoa_matrix = np.zeros((self.num_channels, self.num_channels))
        self.tdoa_pairs = []
        
        for i in range(self.num_channels):
            for j in range(i + 1, self.num_channels):
                tdoa, correlation, lags = self.calculate_tdoa(i, j, max_delay_samples)
                self.tdoa_matrix[i, j] = tdoa
                self.tdoa_matrix[j, i] = -tdoa
                
                # Calculate distance difference
                distance_diff = tdoa * self.speed_of_sound
                
                self.tdoa_pairs.append({
                    'mic1': i,
                    'mic2': j,
                    'tdoa': tdoa,
                    'distance_diff': distance_diff,
                    'correlation': correlation,
                    'lags': lags
                })
                
                print(f"  Mic {i} -> Mic {j}: TDOA = {tdoa*1000:.2f} ms, "
                      f"Distance diff = {distance_diff*100:.2f} cm")
    
    def triangulate_source(self, method='least_squares'):
        """
        Triangulate sound source location using TDOA values
        
        Parameters:
        -----------
        method : str
            Triangulation method: 'least_squares' or 'hyperbolic'
            
        Returns:
        --------
        source_position : array
            Estimated source position [x, y, z]
        """
        if self.tdoa_matrix is None:
            raise ValueError("Must calculate TDOA first. Call calculate_all_tdoa()")
        
        if method == 'least_squares':
            return self._triangulate_least_squares()
        elif method == 'hyperbolic':
            return self._triangulate_hyperbolic()
        else:
            raise ValueError(f"Unknown method: {method}")
    
    def _triangulate_least_squares(self):
        """
        Least squares triangulation using TDOA measurements
        """
        print(f"\nTriangulating source location using least squares method...")
        
        # Use first microphone as reference
        ref_mic = 0
        ref_pos = self.mic_positions[ref_mic]
        
        # Build system of equations: d_i - d_ref = c * tdoa_i
        # Where d_i is distance from source to mic i
        A = []
        b = []
        
        for i in range(1, self.num_channels):
            tdoa = self.tdoa_matrix[ref_mic, i]
            if abs(tdoa) < 1e-6:  # Skip if TDOA is too small
                continue
            
            mic_pos = self.mic_positions[i]
            distance_diff = tdoa * self.speed_of_sound
            
            # Equation: ||source - mic_i|| - ||source - ref_mic|| = distance_diff
            # Linearized around initial guess
            A.append(mic_pos - ref_pos)
            b.append(distance_diff)
        
        if len(A) < 2:
            print("Warning: Not enough TDOA measurements for 3D triangulation")
            return np.array([0.0, 0.0, 0.0])
        
        A = np.array(A)
        b = np.array(b)
        
        # Initial guess: center of microphone array
        initial_guess = np.mean(self.mic_positions, axis=0)
        
        # Objective function: minimize sum of squared residuals
        def objective(source_pos):
            residuals = []
            for i in range(1, self.num_channels):
                tdoa = self.tdoa_matrix[ref_mic, i]
                if abs(tdoa) < 1e-6:
                    continue
                
                dist_i = np.linalg.norm(source_pos - self.mic_positions[i])
                dist_ref = np.linalg.norm(source_pos - ref_pos)
                expected_diff = tdoa * self.speed_of_sound
                residual = (dist_i - dist_ref) - expected_diff
                residuals.append(residual)
            
            return np.sum(np.array(residuals)**2)
        
        # Minimize objective function
        result = minimize(objective, initial_guess, method='BFGS')
        source_position = result.x
        
        print(f"  Estimated source position: [{source_position[0]:.3f}, "
              f"{source_position[1]:.3f}, {source_position[2]:.3f}] meters")
        
        return source_position
    
    def _triangulate_hyperbolic(self):
        """
        Hyperbolic triangulation method
        """
        print(f"\nTriangulating source location using hyperbolic method...")
        # Simplified implementation - can be extended
        return self._triangulate_least_squares()
    
    def plot_tdoa_results(self, output_file='tdoa_results.png'):
        """
        Plot TDOA results and triangulation
        
        Parameters:
        -----------
        output_file : str
            Output filename for the plot
        """
        if self.tdoa_matrix is None:
            raise ValueError("Must calculate TDOA first. Call calculate_all_tdoa()")
        
        print(f"\nGenerating TDOA plots...")
        
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        
        # 1. TDOA Matrix Heatmap
        ax1 = plt.subplot(2, 3, 1)
        im = ax1.imshow(self.tdoa_matrix * 1000, cmap='RdBu_r', aspect='auto')
        ax1.set_title('TDOA Matrix (milliseconds)', fontsize=14, fontweight='bold')
        ax1.set_xlabel('Microphone Index')
        ax1.set_ylabel('Microphone Index')
        plt.colorbar(im, ax=ax1, label='TDOA (ms)')
        
        # 2. TDOA Values Bar Plot
        ax2 = plt.subplot(2, 3, 2)
        tdoa_values = []
        pair_labels = []
        for pair in self.tdoa_pairs:
            tdoa_values.append(pair['tdoa'] * 1000)
            pair_labels.append(f"M{pair['mic1']}-M{pair['mic2']}")
        
        bars = ax2.bar(range(len(tdoa_values)), tdoa_values, color='steelblue', alpha=0.7)
        ax2.set_title('TDOA Values for All Pairs', fontsize=14, fontweight='bold')
        ax2.set_xlabel('Microphone Pair')
        ax2.set_ylabel('TDOA (milliseconds)')
        ax2.set_xticks(range(len(pair_labels)))
        ax2.set_xticklabels(pair_labels, rotation=45, ha='right')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
        
        # Add value labels on bars
        for i, (bar, val) in enumerate(zip(bars, tdoa_values)):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val:.2f}',
                    ha='center', va='bottom' if height >= 0 else 'top', fontsize=8)
        
        # 3. Cross-correlation examples (first 3 pairs)
        num_examples = min(3, len(self.tdoa_pairs))
        for idx, pair in enumerate(self.tdoa_pairs[:num_examples]):
            ax = plt.subplot(2, 3, 3 + idx)
            ax.plot(pair['lags'] * 1000, pair['correlation'], 'b-', linewidth=1.5)
            ax.axvline(pair['tdoa'] * 1000, color='red', linestyle='--', 
                      linewidth=2, label=f"TDOA: {pair['tdoa']*1000:.2f} ms")
            ax.set_title(f'Cross-Correlation: Mic {pair["mic1"]} vs Mic {pair["mic2"]}', 
                        fontsize=12, fontweight='bold')
            ax.set_xlabel('Time Lag (ms)')
            ax.set_ylabel('Correlation')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.set_xlim([pair['tdoa']*1000 - 20, pair['tdoa']*1000 + 20])
        
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_file}")
        plt.close()
    
    def plot_triangulation(self, source_position, output_file='triangulation_result.png'):
        """
        Plot microphone array and triangulated source location
        
        Parameters:
        -----------
        source_position : array
            Estimated source position [x, y, z]
        output_file : str
            Output filename for the plot
        """
        print(f"\nGenerating triangulation plot...")
        
        fig = plt.figure(figsize=(14, 10))
        
        # 2D Top View (XY plane)
        ax1 = plt.subplot(2, 2, 1)
        ax1.scatter(self.mic_positions[:, 0], self.mic_positions[:, 1], 
                   s=200, c='blue', marker='s', label='Microphones', zorder=3)
        ax1.scatter(source_position[0], source_position[1], 
                   s=300, c='red', marker='*', label='Estimated Source', zorder=4)
        
        # Draw lines from source to microphones
        for i, mic_pos in enumerate(self.mic_positions):
            ax1.plot([source_position[0], mic_pos[0]], 
                    [source_position[1], mic_pos[1]], 
                    'gray', linestyle='--', alpha=0.3, linewidth=1)
            ax1.text(mic_pos[0], mic_pos[1], f' M{i}', 
                    fontsize=10, verticalalignment='bottom')
        
        ax1.set_title('Top View (XY Plane)', fontsize=14, fontweight='bold')
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.axis('equal')
        
        # 2D Side View (XZ plane)
        ax2 = plt.subplot(2, 2, 2)
        ax2.scatter(self.mic_positions[:, 0], self.mic_positions[:, 2], 
                   s=200, c='blue', marker='s', label='Microphones', zorder=3)
        ax2.scatter(source_position[0], source_position[2], 
                   s=300, c='red', marker='*', label='Estimated Source', zorder=4)
        
        for i, mic_pos in enumerate(self.mic_positions):
            ax2.plot([source_position[0], mic_pos[0]], 
                    [source_position[2], mic_pos[2]], 
                    'gray', linestyle='--', alpha=0.3, linewidth=1)
        
        ax2.set_title('Side View (XZ Plane)', fontsize=14, fontweight='bold')
        ax2.set_xlabel('X (meters)')
        ax2.set_ylabel('Z (meters)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # 3D View
        ax3 = plt.subplot(2, 2, (3, 4), projection='3d')
        ax3.scatter(self.mic_positions[:, 0], self.mic_positions[:, 1], 
                   self.mic_positions[:, 2], 
                   s=200, c='blue', marker='s', label='Microphones')
        ax3.scatter(source_position[0], source_position[1], source_position[2], 
                   s=300, c='red', marker='*', label='Estimated Source')
        
        # Draw lines from source to microphones
        for mic_pos in self.mic_positions:
            ax3.plot([source_position[0], mic_pos[0]], 
                    [source_position[1], mic_pos[1]], 
                    [source_position[2], mic_pos[2]], 
                    'gray', linestyle='--', alpha=0.3, linewidth=1)
        
        ax3.set_title('3D View', fontsize=14, fontweight='bold')
        ax3.set_xlabel('X (meters)')
        ax3.set_ylabel('Y (meters)')
        ax3.set_zlabel('Z (meters)')
        ax3.legend()
        
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_file}")
        plt.close()


def main():
    """Main function"""
    # Audio file path
    audio_file = '/home/head-node-5/resp7chn/138/resp.wav'
    
    print("=" * 80)
    print("TDOA TRIANGULATION FOR SOUND SOURCE LOCALIZATION")
    print("=" * 80)
    
    # Initialize TDOA triangulation
    # You can specify custom microphone positions if known
    # Example: mic_positions = [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], ...]
    tdoa_system = TDOATriangulation(audio_file, mic_positions=None)
    
    # Calculate TDOA for all microphone pairs
    tdoa_system.calculate_all_tdoa(max_delay_ms=50)
    
    # Plot TDOA results
    tdoa_system.plot_tdoa_results('tdoa_results.png')
    
    # Triangulate source location
    source_position = tdoa_system.triangulate_source(method='least_squares')
    
    # Plot triangulation results
    tdoa_system.plot_triangulation(source_position, 'triangulation_result.png')
    
    # Print summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"Estimated source position: [{source_position[0]:.3f}, "
          f"{source_position[1]:.3f}, {source_position[2]:.3f}] meters")
    print(f"Distance from array center: {np.linalg.norm(source_position):.3f} meters")
    print("\nGenerated files:")
    print("- tdoa_results.png")
    print("- triangulation_result.png")
    print("=" * 80)


if __name__ == "__main__":
    main()

