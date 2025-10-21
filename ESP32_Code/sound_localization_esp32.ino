/*
 * Sound Source Localization using INMP411 Microphone Array
 * ESP32 Implementation for Real-time 3D Localization
 * 
 * Hardware:
 * - 4x INMP411 MEMS Microphones
 * - ESP32 Development Board
 * - MicroSD Card Module (optional)
 * 
 * Author: AI Assistant
 * Date: 2024
 */

#include <driver/i2s.h>
#include <WiFi.h>
#include <math.h>

// Pin definitions
#define I2S_WS 27
#define I2S_BCLK 26
#define I2S_DOUT_1 25  // Mic 1 (Front)
#define I2S_DOUT_2 33  // Mic 2 (Right)
#define I2S_DOUT_3 32  // Mic 3 (Back)
#define I2S_DOUT_4 35  // Mic 4 (Left)

#define LED_PIN 2

// Audio parameters
#define SAMPLE_RATE 16000
#define BUFFER_SIZE 1024
#define NUM_MICS 4

// Microphone array geometry (in cm)
const float mic_positions[NUM_MICS][3] = {
  {0.0, 0.0, 2.5},    // Mic 1 (Front)
  {2.5, 0.0, 0.0},    // Mic 2 (Right)
  {0.0, 0.0, -2.5},   // Mic 3 (Back)
  {-2.5, 0.0, 0.0}    // Mic 4 (Left)
};

// Global variables
int16_t audio_buffer[NUM_MICS][BUFFER_SIZE];
float tdoa_values[NUM_MICS];
float azimuth = 0.0;
float elevation = 0.0;
bool localization_active = false;

// WiFi credentials
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

void setup() {
  Serial.begin(115200);
  Serial.println("Sound Source Localization System Starting...");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2S for microphone array
  setupI2S();
  
  // Connect to WiFi
  connectToWiFi();
  
  // Calibrate system
  calibrateSystem();
  
  Serial.println("System ready for sound source localization");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  if (localization_active) {
    // Acquire audio data from all microphones
    acquireAudioData();
    
    // Calculate Time Difference of Arrival (TDOA)
    calculateTDOA();
    
    // Perform 3D localization
    performLocalization();
    
    // Output results
    outputResults();
  }
  
  // Check for serial commands
  handleSerialCommands();
  
  delay(10); // Small delay for stability
}

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DOUT_1
  };
  
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed");
  }
}

void calibrateSystem() {
  Serial.println("Calibrating system...");
  
  // Acquire baseline noise levels
  for (int i = 0; i < 10; i++) {
    acquireAudioData();
    delay(100);
  }
  
  Serial.println("Calibration complete");
}

void acquireAudioData() {
  size_t bytes_read;
  
  // Read from each microphone sequentially
  for (int mic = 0; mic < NUM_MICS; mic++) {
    // Switch to appropriate microphone
    i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_DOUT_1 + mic
    };
    i2s_set_pin(I2S_NUM_0, &pin_config);
    
    // Read audio data
    i2s_read(I2S_NUM_0, audio_buffer[mic], BUFFER_SIZE * sizeof(int16_t), &bytes_read, portMAX_DELAY);
  }
}

void calculateTDOA() {
  // Calculate cross-correlation between microphone pairs
  // This is a simplified version - full implementation would use FFT
  
  for (int i = 0; i < NUM_MICS; i++) {
    tdoa_values[i] = 0.0; // Initialize
    
    // Simple peak detection for TDOA estimation
    int max_index = 0;
    int16_t max_value = 0;
    
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
  // Simplified 3D localization using TDOA
  // This is a basic implementation - full version would use proper triangulation
  
  float x = 0.0, y = 0.0, z = 0.0;
  
  // Calculate relative time differences
  float dt12 = tdoa_values[1] - tdoa_values[0]; // Right - Front
  float dt13 = tdoa_values[2] - tdoa_values[0]; // Back - Front
  float dt14 = tdoa_values[3] - tdoa_values[0]; // Left - Front
  
  // Convert to position estimates (simplified)
  float speed_of_sound = 343.0; // m/s
  
  x = dt12 * speed_of_sound * 100; // Convert to cm
  z = dt13 * speed_of_sound * 100;
  y = dt14 * speed_of_sound * 100;
  
  // Convert to spherical coordinates
  float r = sqrt(x*x + y*y + z*z);
  
  if (r > 0) {
    azimuth = atan2(y, x) * 180.0 / PI;
    elevation = asin(z / r) * 180.0 / PI;
    
    // Normalize angles
    if (azimuth < 0) azimuth += 360.0;
    if (elevation < -90) elevation = -90;
    if (elevation > 90) elevation = 90;
  }
}

void outputResults() {
  // Serial output for computer processing
  Serial.print("Azimuth: ");
  Serial.print(azimuth, 1);
  Serial.print("°, Elevation: ");
  Serial.print(elevation, 1);
  Serial.print("°, Distance: ");
  Serial.print(sqrt(azimuth*azimuth + elevation*elevation), 1);
  Serial.println(" cm");
  
  // JSON format for MATLAB processing
  Serial.print("{\"azimuth\":");
  Serial.print(azimuth, 2);
  Serial.print(",\"elevation\":");
  Serial.print(elevation, 2);
  Serial.print(",\"distance\":");
  Serial.print(sqrt(azimuth*azimuth + elevation*elevation), 2);
  Serial.print(",\"confidence\":");
  Serial.print(0.8, 2); // Placeholder confidence value
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}");
}

// Data logging function removed - data will be sent to computer via serial/WiFi

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

// Utility functions
float calculateDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
  return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2) + pow(z2-z1, 2));
}

void applyBandpassFilter(int16_t* buffer, int length, float low_freq, float high_freq) {
  // Simple bandpass filter implementation
  // This is a placeholder - full implementation would use proper DSP
  for (int i = 0; i < length; i++) {
    // Apply simple high-pass filter
    if (i > 0) {
      buffer[i] = buffer[i] - buffer[i-1] * 0.95;
    }
  }
}

void normalizeAudio(int16_t* buffer, int length) {
  // Find maximum value
  int16_t max_val = 0;
  for (int i = 0; i < length; i++) {
    if (abs(buffer[i]) > max_val) {
      max_val = abs(buffer[i]);
    }
  }
  
  // Normalize if not zero
  if (max_val > 0) {
    float scale = 32767.0 / max_val;
    for (int i = 0; i < length; i++) {
      buffer[i] = (int16_t)(buffer[i] * scale);
    }
  }
}
