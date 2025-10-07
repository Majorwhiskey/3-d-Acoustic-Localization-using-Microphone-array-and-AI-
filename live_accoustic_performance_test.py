# Live Acoustic Performance Test Script
#
# Measures acoustic energy (RMS) in real-time, simulating Voice Activity Detection (VAD)
# and allowing you to validate range and detection thresholds live.
#
# Prerequisite: pip install numpy pyaudio matplotlib

import numpy as np
import pyaudio
import time
import matplotlib.pyplot as plt

# --- Configuration (Adjust as needed for your ReSpeaker USB 4 Mic Array V3.0) ---
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 4  # Try 2 if 4 gives an error (processed stereo output)
RATE = 16000
TEST_DURATION_SECONDS = 15 # Longer duration for live testing
DAMPING_FACTOR = 0.95      # EMA smoothing factor
THRESHOLD = 1500           # Initial VAD (Voice Activity Detection) threshold

def live_acoustic_test():
    """Streams audio live and tracks RMS energy for VAD/Range testing."""
    p = pyaudio.PyAudio()

    # 1. Recording Setup
    try:
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
    except Exception as e:
        print(f"Error opening audio stream with {CHANNELS} channels: {e}")
        print("Try changing CHANNELS to 2 in the script configuration.")
        p.terminate()
        return

    print(f"\n--- 3. LIVE Acoustic Performance (Range/Threshold) ---")
    print(f"ACTION: Test speaking at various distances during the {TEST_DURATION_SECONDS}s test.")
    print(f"Current VAD Threshold: {THRESHOLD}")
    
    energy_history = []
    times = []
    
    smoothed_energy = 0
    start_time = time.time()
    
    # 2. Real-Time Processing Loop
    try:
        for i in range(0, int(RATE / CHUNK * TEST_DURATION_SECONDS)):
            if (time.time() - start_time) > TEST_DURATION_SECONDS:
                break
                
            data = stream.read(CHUNK, exception_on_overflow=False)
            audio_data_chunk = np.frombuffer(data, dtype=np.int16)
            
            # Calculate Root Mean Square (RMS) energy
            current_energy = np.sqrt(np.mean(audio_data_chunk**2))

            # Exponential Moving Average (EMA) for smoothing
            smoothed_energy = DAMPING_FACTOR * smoothed_energy + (1 - DAMPING_FACTOR) * current_energy

            energy_history.append(smoothed_energy)
            times.append(time.time() - start_time)
            
            # Determine the detection status
            vad_status = "VOICE DETECTED" if smoothed_energy > THRESHOLD else "IDLE"
            
            print(f"   Time: {times[-1]:.1f}s | Energy: {int(smoothed_energy):<5} | Status: {vad_status}", end='\r')
            
    except KeyboardInterrupt:
        print("\nTest manually stopped.")
    finally:
        # 3. Cleanup and Plotting
        stream.stop_stream()
        stream.close()
        p.terminate()
        print("\n\nTest Finished.")
        
        print(f"INSTRUCTIONS: Note the distance and volume where the energy line exceeds the threshold line.")
        
        plt.figure(figsize=(10, 4))
        plt.plot(times, energy_history)
        plt.axhline(y=THRESHOLD, color='r', linestyle='--', label='Detection Threshold')
        plt.title("Live Acoustic Energy Tracking (VAD/Range Proxy)")
        plt.xlabel("Time (s)")
        plt.ylabel("Smoothed RMS Energy")
        plt.legend()
        plt.grid(True)
        plt.show() 

if __name__ == "__main__":
    live_acoustic_test()
