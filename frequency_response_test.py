# Frequency Response Test Script
#
# Records a short audio clip and performs Fast Fourier Transform (FFT) analysis
# to characterize the microphone array's sensitivity across different frequencies.
#
# Prerequisite: pip install numpy pyaudio matplotlib

import numpy as np
import pyaudio
import matplotlib.pyplot as plt

# --- Configuration (Adjust as needed for your ReSpeaker USB 4 Mic Array V3.0) ---
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 4  # Try 2 if 4 gives an error (processed stereo output)
RATE = 16000
RECORD_SECONDS = 5 # Increased duration for better frequency analysis

def record_and_analyze():
    """Records audio and performs FFT analysis."""
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

    print(f"[*] Recording for {RECORD_SECONDS} seconds at {RATE} Hz...")
    print("ACTION: Play a wide-band audio signal (like a tone sweep or music) during the recording.")
    frames = []
    
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK, exception_on_overflow=False) 
        frames.append(data)

    print("[*] Recording finished. Analyzing frequency content...")
    stream.stop_stream()
    stream.close()
    p.terminate()

    # Convert raw data to NumPy array
    audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
    
    # 2. FFT Analysis
    N = len(audio_data)
    windowed_data = audio_data * np.hamming(N)
    
    yf = np.fft.fft(windowed_data)
    yf_magnitude = 2.0/N * np.abs(yf[:N//2]) # Take only positive frequencies
    xf = np.fft.fftfreq(N, 1/RATE)[:N//2] # Frequency axis

    # 3. Plotting the result
    plt.figure(figsize=(10, 4))
    plt.plot(xf, yf_magnitude)
    plt.title("Frequency Spectrum of Recorded Audio Sample")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (Amplitude)")
    plt.grid(True)
    plt.xlim(20, RATE / 2)
    plt.ylim(bottom=0)
    plt.show() 

if __name__ == "__main__":
    record_and_analyze()
