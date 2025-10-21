/*
 * Advanced Sound Source Localization Algorithm
 * Enhanced ESP32 Implementation with Improved TDOA and Triangulation
 * 
 * Features:
 * - Cross-correlation based TDOA estimation
 * - 3D triangulation algorithm
 * - Kalman filtering for position tracking
 * - Real-time performance optimization
 */

#include <driver/i2s.h>
#include <WiFi.h>
#include <math.h>
#include <esp_dsp.h>

// Advanced configuration
#define SAMPLE_RATE 16000
#define FFT_SIZE 512
#define OVERLAP_FACTOR 0.5
#define NUM_MICS 4
#define MAX_SOURCES 3

// TDOA and triangulation structures
struct TDOA_Result {
  float delay_samples;
  float confidence;
  bool valid;
};

struct Position3D {
  float x, y, z;
  float azimuth, elevation, distance;
  float confidence;
  bool valid;
};

// Global variables for advanced processing
TDOA_Result tdoa_results[NUM_MICS][NUM_MICS];
Position3D source_positions[MAX_SOURCES];
float fft_buffer[FFT_SIZE * 2];
float correlation_buffer[FFT_SIZE];

// Microphone array geometry (optimized for ESP32)
const float mic_positions[NUM_MICS][3] = {
  {0.0, 0.0, 2.5},    // Mic 1 (Front)
  {2.5, 0.0, 0.0},    // Mic 2 (Right)  
  {0.0, 0.0, -2.5},   // Mic 3 (Back)
  {-2.5, 0.0, 0.0}    // Mic 4 (Left)
};

// Kalman filter state for position tracking
struct KalmanState {
  float x, y, z;           // Position
  float vx, vy, vz;        // Velocity
  float P[6][6];           // Covariance matrix
  bool initialized;
};

KalmanState kalman_state;

void setupAdvancedLocalization() {
  // Initialize DSP library
  esp_dsp_init();
  
  // Initialize Kalman filter
  initializeKalmanFilter();
  
  Serial.println("Advanced localization system initialized");
}

void initializeKalmanFilter() {
  kalman_state.initialized = false;
  
  // Initialize covariance matrix
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      kalman_state.P[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
}

TDOA_Result calculateTDOA_CrossCorrelation(int16_t* mic1_data, int16_t* mic2_data, int length) {
  TDOA_Result result;
  result.valid = false;
  result.delay_samples = 0.0;
  result.confidence = 0.0;
  
  // Preprocess signals
  float* signal1 = (float*)malloc(length * sizeof(float));
  float* signal2 = (float*)malloc(length * sizeof(float));
  
  // Convert to float and apply windowing
  for (int i = 0; i < length; i++) {
    signal1[i] = (float)mic1_data[i] * hamming_window(i, length);
    signal2[i] = (float)mic2_data[i] * hamming_window(i, length);
  }
  
  // Calculate cross-correlation using FFT
  float max_correlation = 0.0;
  int max_lag = 0;
  
  // Simplified cross-correlation (full implementation would use FFT)
  for (int lag = -length/4; lag <= length/4; lag++) {
    float correlation = 0.0;
    int count = 0;
    
    for (int i = 0; i < length - abs(lag); i++) {
      int idx1 = i;
      int idx2 = i + lag;
      if (idx2 >= 0 && idx2 < length) {
        correlation += signal1[idx1] * signal2[idx2];
        count++;
      }
    }
    
    if (count > 0) {
      correlation /= count;
      if (abs(correlation) > max_correlation) {
        max_correlation = abs(correlation);
        max_lag = lag;
      }
    }
  }
  
  // Convert lag to time delay
  result.delay_samples = (float)max_lag;
  result.confidence = max_correlation;
  result.valid = (max_correlation > 0.1); // Threshold for valid correlation
  
  free(signal1);
  free(signal2);
  
  return result;
}

float hamming_window(int index, int length) {
  return 0.54 - 0.46 * cos(2.0 * PI * index / (length - 1));
}

Position3D triangulate3D() {
  Position3D position;
  position.valid = false;
  
  // Calculate TDOA for all microphone pairs
  for (int i = 0; i < NUM_MICS; i++) {
    for (int j = i + 1; j < NUM_MICS; j++) {
      // This would use actual audio data - simplified here
      tdoa_results[i][j] = calculateTDOA_CrossCorrelation(
        audio_buffer[i], audio_buffer[j], BUFFER_SIZE);
    }
  }
  
  // Solve triangulation using least squares method
  // This is a simplified version - full implementation would solve the system of equations
  
  float A[6][3]; // Coefficient matrix
  float b[6];    // Right-hand side vector
  int equation_count = 0;
  
  // Build system of equations from TDOA measurements
  for (int i = 0; i < NUM_MICS; i++) {
    for (int j = i + 1; j < NUM_MICS; j++) {
      if (tdoa_results[i][j].valid) {
        float delay = tdoa_results[i][j].delay_samples / SAMPLE_RATE;
        float distance_diff = delay * 343.0 * 100; // Convert to cm
        
        // Equation: sqrt((x-xi)² + (y-yi)² + (z-zi)²) - sqrt((x-xj)² + (y-yj)² + (z-zj)²) = distance_diff
        A[equation_count][0] = 2 * (mic_positions[j][0] - mic_positions[i][0]);
        A[equation_count][1] = 2 * (mic_positions[j][1] - mic_positions[i][1]);
        A[equation_count][2] = 2 * (mic_positions[j][2] - mic_positions[i][2]);
        b[equation_count] = distance_diff;
        equation_count++;
      }
    }
  }
  
  if (equation_count >= 3) {
    // Solve using least squares (simplified)
    position = solveLeastSquares(A, b, equation_count);
    position.valid = true;
  }
  
  return position;
}

Position3D solveLeastSquares(float A[][3], float b[], int num_equations) {
  Position3D position;
  position.valid = false;
  
  // Simplified least squares solution
  // Full implementation would use proper matrix operations
  
  // For now, use a simple geometric approach
  float x = 0.0, y = 0.0, z = 0.0;
  
  // Weighted average of microphone positions based on TDOA
  float total_weight = 0.0;
  
  for (int i = 0; i < NUM_MICS; i++) {
    for (int j = i + 1; j < NUM_MICS; j++) {
      if (tdoa_results[i][j].valid) {
        float weight = tdoa_results[i][j].confidence;
        x += (mic_positions[i][0] + mic_positions[j][0]) * weight / 2.0;
        y += (mic_positions[i][1] + mic_positions[j][1]) * weight / 2.0;
        z += (mic_positions[i][2] + mic_positions[j][2]) * weight / 2.0;
        total_weight += weight;
      }
    }
  }
  
  if (total_weight > 0) {
    position.x = x / total_weight;
    position.y = y / total_weight;
    position.z = z / total_weight;
    
    // Convert to spherical coordinates
    float r = sqrt(position.x*position.x + position.y*position.y + position.z*position.z);
    position.azimuth = atan2(position.y, position.x) * 180.0 / PI;
    position.elevation = asin(position.z / r) * 180.0 / PI;
    position.distance = r;
    position.confidence = total_weight;
    position.valid = true;
  }
  
  return position;
}

void updateKalmanFilter(Position3D measurement) {
  if (!measurement.valid) return;
  
  if (!kalman_state.initialized) {
    // Initialize with first measurement
    kalman_state.x = measurement.x;
    kalman_state.y = measurement.y;
    kalman_state.z = measurement.z;
    kalman_state.vx = 0.0;
    kalman_state.vy = 0.0;
    kalman_state.vz = 0.0;
    kalman_state.initialized = true;
    return;
  }
  
  // Kalman filter prediction step
  float dt = 0.1; // Time step (100ms)
  
  // Predict position
  float x_pred = kalman_state.x + kalman_state.vx * dt;
  float y_pred = kalman_state.y + kalman_state.vy * dt;
  float z_pred = kalman_state.z + kalman_state.vz * dt;
  
  // Update covariance matrix (simplified)
  for (int i = 0; i < 6; i++) {
    kalman_state.P[i][i] += 0.1; // Process noise
  }
  
  // Kalman filter update step
  float innovation_x = measurement.x - x_pred;
  float innovation_y = measurement.y - y_pred;
  float innovation_z = measurement.z - z_pred;
  
  // Update state (simplified)
  float gain = 0.3; // Kalman gain
  kalman_state.x = x_pred + gain * innovation_x;
  kalman_state.y = y_pred + gain * innovation_y;
  kalman_state.z = z_pred + gain * innovation_z;
  
  // Update velocity
  kalman_state.vx = (kalman_state.x - x_pred) / dt;
  kalman_state.vy = (kalman_state.y - y_pred) / dt;
  kalman_state.vz = (kalman_state.z - z_pred) / dt;
}

void performAdvancedLocalization() {
  // Calculate TDOA for all microphone pairs
  for (int i = 0; i < NUM_MICS; i++) {
    for (int j = i + 1; j < NUM_MICS; j++) {
      tdoa_results[i][j] = calculateTDOA_CrossCorrelation(
        audio_buffer[i], audio_buffer[j], BUFFER_SIZE);
    }
  }
  
  // Perform 3D triangulation
  Position3D raw_position = triangulate3D();
  
  if (raw_position.valid) {
    // Update Kalman filter
    updateKalmanFilter(raw_position);
    
    // Output filtered position
    float filtered_azimuth = atan2(kalman_state.y, kalman_state.x) * 180.0 / PI;
    float filtered_elevation = asin(kalman_state.z / sqrt(kalman_state.x*kalman_state.x + 
                                                          kalman_state.y*kalman_state.y + 
                                                          kalman_state.z*kalman_state.z)) * 180.0 / PI;
    
    // Normalize angles
    if (filtered_azimuth < 0) filtered_azimuth += 360.0;
    if (filtered_elevation < -90) filtered_elevation = -90;
    if (filtered_elevation > 90) filtered_elevation = 90;
    
    // Output results
    Serial.print("Filtered Azimuth: ");
    Serial.print(filtered_azimuth, 1);
    Serial.print("°, Elevation: ");
    Serial.print(filtered_elevation, 1);
    Serial.print("°, Distance: ");
    Serial.print(sqrt(kalman_state.x*kalman_state.x + kalman_state.y*kalman_state.y + kalman_state.z*kalman_state.z), 1);
    Serial.println(" cm");
  }
}

// Utility functions for advanced processing
void applyHighPassFilter(int16_t* buffer, int length, float cutoff_freq) {
  static float prev_input = 0.0;
  static float prev_output = 0.0;
  
  float alpha = 1.0 / (1.0 + 2.0 * PI * cutoff_freq / SAMPLE_RATE);
  
  for (int i = 0; i < length; i++) {
    float input = (float)buffer[i];
    float output = alpha * (prev_output + input - prev_input);
    
    buffer[i] = (int16_t)output;
    prev_input = input;
    prev_output = output;
  }
}

void applyLowPassFilter(int16_t* buffer, int length, float cutoff_freq) {
  static float prev_output = 0.0;
  
  float alpha = 2.0 * PI * cutoff_freq / SAMPLE_RATE;
  
  for (int i = 0; i < length; i++) {
    float input = (float)buffer[i];
    float output = prev_output + alpha * (input - prev_output);
    
    buffer[i] = (int16_t)output;
    prev_output = output;
  }
}

void detectPeaks(int16_t* buffer, int length, int* peak_indices, int* num_peaks) {
  *num_peaks = 0;
  int min_distance = SAMPLE_RATE / 100; // Minimum 10ms between peaks
  int16_t threshold = 1000; // Adjust based on noise level
  
  for (int i = min_distance; i < length - min_distance; i++) {
    if (abs(buffer[i]) > threshold) {
      bool is_peak = true;
      
      // Check if it's a local maximum
      for (int j = i - min_distance/2; j <= i + min_distance/2; j++) {
        if (j != i && abs(buffer[j]) >= abs(buffer[i])) {
          is_peak = false;
          break;
        }
      }
      
      if (is_peak && *num_peaks < 10) {
        peak_indices[*num_peaks] = i;
        (*num_peaks)++;
      }
    }
  }
}
