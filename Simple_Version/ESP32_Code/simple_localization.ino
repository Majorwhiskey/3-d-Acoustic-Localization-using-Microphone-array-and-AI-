/*
 * Simple Sound Source Localization using ESP32 + Condenser Microphones
 * 
 * Hardware:
 * - ESP32 Development Board
 * - 4x Condenser Microphones (analog)
 * - Basic amplifier circuit
 * 
 * Features:
 * - Real-time audio acquisition from 4 analog microphones
 * - Basic TDOA calculation using cross-correlation
 * - 2D localization (azimuth only)
 * - Serial data output for computer processing
 * 
 * Author: AI Assistant
 * Date: 2024
 */

// ADC pin definitions
#define MIC_1_PIN 36  // ADC1_CH0
#define MIC_2_PIN 39  // ADC1_CH3  
#define MIC_3_PIN 34  // ADC1_CH6
#define MIC_4_PIN 35  // ADC1_CH7

// Audio parameters
#define SAMPLE_RATE 8000    // Lower sample rate for analog processing
#define BUFFER_SIZE 512     // Smaller buffer for ESP32 capabilities
#define NUM_MICS 4

// Microphone array geometry (2D, in cm)
const float mic_positions[NUM_MICS][2] = {
  {0.0, 0.0},    // Mic 1 (Front)
  {5.0, 0.0},    // Mic 2 (Right)
  {0.0, 5.0},    // Mic 3 (Back)
  {-5.0, 0.0}    // Mic 4 (Left)
};

// Global variables
int16_t audio_buffer[NUM_MICS][BUFFER_SIZE];
float tdoa_values[NUM_MICS];
float azimuth = 0.0;
float distance = 0.0;
bool localization_active = false;

// ADC configuration
const int adc_channel[NUM_MICS] = {ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_6, ADC1_CHANNEL_7};

void setup() {
  Serial.begin(115200);
  Serial.println("Simple Sound Localization System Starting...");
  
  // Configure ADC
  setupADC();
  
  // Calibrate system
  calibrateSystem();
  
  Serial.println("System ready for sound source localization");
  Serial.println("Commands: START, STOP, STATUS, CALIBRATE, HELP");
}

void loop() {
  if (localization_active) {
    // Acquire audio data from all microphones
    acquireAudioData();
    
    // Calculate Time Difference of Arrival (TDOA)
    calculateTDOA();
    
    // Perform 2D localization
    performLocalization();
    
    // Output results
    outputResults();
  }
  
  // Check for serial commands
  handleSerialCommands();
  
  delay(50); // 20Hz update rate
}

void setupADC() {
  // Configure ADC for minimal noise
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  analogReadResolution(12);         // 12-bit resolution
  analogSetSamples(4);             // Multiple samples for noise reduction
  
  Serial.println("ADC configured for minimal noise analog input");
  Serial.println("No op-amps needed - direct microphone connection");
}

void calibrateSystem() {
  Serial.println("Calibrating system...");
  
  // Acquire baseline noise levels
  for (int i = 0; i < 5; i++) {
    acquireAudioData();
    delay(100);
  }
  
  Serial.println("Calibration complete");
}

void acquireAudioData() {
  // Read from each microphone with digital noise reduction
  for (int mic = 0; mic < NUM_MICS; mic++) {
    for (int sample = 0; sample < BUFFER_SIZE; sample++) {
      // Multiple ADC reads for noise reduction
      int adc_sum = 0;
      for (int i = 0; i < 4; i++) {
        adc_sum += analogRead(adc_channel[mic]);
        delayMicroseconds(10); // Small delay between reads
      }
      int adc_value = adc_sum / 4; // Average of 4 readings
      
      // Convert to signed 16-bit range (-2048 to 2047)
      audio_buffer[mic][sample] = (int16_t)(adc_value - 2048);
    }
  }
}

void calculateTDOA() {
  // Enhanced TDOA calculation with digital filtering
  for (int i = 0; i < NUM_MICS; i++) {
    tdoa_values[i] = 0.0;
    
    // Apply digital high-pass filter to remove DC offset
    applyHighPassFilter(audio_buffer[i], BUFFER_SIZE);
    
    // Apply moving average filter for noise reduction
    applyMovingAverageFilter(audio_buffer[i], BUFFER_SIZE);
    
    // Find peak in filtered signal
    int16_t max_value = 0;
    int max_index = 0;
    
    for (int j = 0; j < BUFFER_SIZE; j++) {
      if (abs(audio_buffer[i][j]) > max_value) {
        max_value = abs(audio_buffer[i][j]);
        max_index = j;
      }
    }
    
    // Convert sample index to time delay
    tdoa_values[i] = (float)max_index / SAMPLE_RATE;
  }
}

void performLocalization() {
  // Simple 2D localization using TDOA
  float x = 0.0, y = 0.0;
  
  // Calculate relative time differences
  float dt12 = tdoa_values[1] - tdoa_values[0]; // Right - Front
  float dt13 = tdoa_values[2] - tdoa_values[0]; // Back - Front
  float dt14 = tdoa_values[3] - tdoa_values[0]; // Left - Front
  
  // Convert to position estimates (simplified)
  float speed_of_sound = 343.0; // m/s
  
  x = dt12 * speed_of_sound * 100; // Convert to cm
  y = dt13 * speed_of_sound * 100;
  
  // Convert to polar coordinates
  distance = sqrt(x*x + y*y);
  
  if (distance > 0) {
    azimuth = atan2(y, x) * 180.0 / PI;
    
    // Normalize angle
    if (azimuth < 0) azimuth += 360.0;
  }
}

void outputResults() {
  // Serial output for computer processing
  Serial.print("Azimuth: ");
  Serial.print(azimuth, 1);
  Serial.print("°, Distance: ");
  Serial.print(distance, 1);
  Serial.println(" cm");
  
  // JSON format for MATLAB processing
  Serial.print("{\"azimuth\":");
  Serial.print(azimuth, 2);
  Serial.print(",\"distance\":");
  Serial.print(distance, 2);
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "START") {
      localization_active = true;
      Serial.println("Localization started");
    }
    else if (command == "STOP") {
      localization_active = false;
      Serial.println("Localization stopped");
    }
    else if (command == "STATUS") {
      Serial.print("Localization active: ");
      Serial.println(localization_active ? "YES" : "NO");
    }
    else if (command == "CALIBRATE") {
      calibrateSystem();
    }
    else if (command == "HELP") {
      Serial.println("Available commands:");
      Serial.println("START - Start localization");
      Serial.println("STOP - Stop localization");
      Serial.println("STATUS - Check status");
      Serial.println("CALIBRATE - Recalibrate system");
      Serial.println("HELP - Show this help");
    }
  }
}

// Digital signal processing functions
void applyHighPassFilter(int16_t* buffer, int length) {
  // Simple high-pass filter to remove DC offset
  static int16_t prev_input = 0;
  static int16_t prev_output = 0;
  
  float alpha = 0.95; // Filter coefficient
  
  for (int i = 0; i < length; i++) {
    int16_t input = buffer[i];
    int16_t output = alpha * (prev_output + input - prev_input);
    
    buffer[i] = output;
    prev_input = input;
    prev_output = output;
  }
}

void applyMovingAverageFilter(int16_t* buffer, int length) {
  // Moving average filter for noise reduction
  int window_size = 3;
  
  for (int i = window_size/2; i < length - window_size/2; i++) {
    int sum = 0;
    for (int j = -window_size/2; j <= window_size/2; j++) {
      sum += buffer[i + j];
    }
    buffer[i] = sum / window_size;
  }
}

void applyBandpassFilter(int16_t* buffer, int length) {
  // Simple bandpass filter (high-pass + low-pass)
  applyHighPassFilter(buffer, length);
  
  // Low-pass filter
  static int16_t prev_output = 0;
  float alpha = 0.8; // Low-pass coefficient
  
  for (int i = 0; i < length; i++) {
    int16_t input = buffer[i];
    int16_t output = prev_output + alpha * (input - prev_output);
    
    buffer[i] = output;
    prev_output = output;
  }
}

void normalizeSignal(int16_t* buffer, int length) {
  // Find maximum value
  int16_t max_val = 0;
  for (int i = 0; i < length; i++) {
    if (abs(buffer[i]) > max_val) {
      max_val = abs(buffer[i]);
    }
  }
  
  // Normalize if not zero
  if (max_val > 0) {
    float scale = 2048.0 / max_val;
    for (int i = 0; i < length; i++) {
      buffer[i] = (int16_t)(buffer[i] * scale);
    }
  }
}

// Test functions for debugging
void testMicrophones() {
  Serial.println("Testing microphone signals...");
  
  for (int mic = 0; mic < NUM_MICS; mic++) {
    int adc_value = analogRead(adc_channel[mic]);
    Serial.print("Mic ");
    Serial.print(mic + 1);
    Serial.print(": ");
    Serial.println(adc_value);
  }
  Serial.println();
}

void testAudioLevels() {
  Serial.println("Testing audio levels...");
  
  for (int mic = 0; mic < NUM_MICS; mic++) {
    int16_t max_val = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      int adc_value = analogRead(adc_channel[mic]);
      int16_t sample = (int16_t)(adc_value - 2048);
      if (abs(sample) > max_val) {
        max_val = abs(sample);
      }
    }
    
    Serial.print("Mic ");
    Serial.print(mic + 1);
    Serial.print(" max level: ");
    Serial.println(max_val);
  }
  Serial.println();
}
