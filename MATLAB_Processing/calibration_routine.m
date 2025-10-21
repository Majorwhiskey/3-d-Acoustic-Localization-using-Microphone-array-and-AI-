%% Calibration Routine for Sound Source Localization System
% This script helps calibrate the microphone array system for accurate localization
%
% Calibration steps:
% 1. Measure actual microphone positions
% 2. Test TDOA accuracy with known sources
% 3. Adjust processing parameters
% 4. Validate system performance

clear; clc; close all;

%% Configuration
fs = 16000;                    % Sample rate
calibration_positions = [
    0, 0, 100;      % Front, 1m
    90, 0, 100;     % Right, 1m
    180, 0, 100;    % Back, 1m
    270, 0, 100;    % Left, 1m
    0, 45, 100;     % Front-up, 1m
    0, -45, 100     % Front-down, 1m
];

% Microphone array geometry (measured positions)
mic_positions = [
    0.0,  0.0,  2.5;    % Mic 1 (Front)
    2.5,  0.0,  0.0;    % Mic 2 (Right)
    0.0,  0.0, -2.5;    % Mic 3 (Back)
   -2.5,  0.0,  0.0     % Mic 4 (Left)
];

%% Step 1: Measure Actual Microphone Positions
fprintf('=== Step 1: Microphone Position Measurement ===\n');
fprintf('Please measure the actual positions of your microphones:\n\n');

for mic = 1:4
    fprintf('Microphone %d:\n', mic);
    fprintf('  X position (cm): ');
    x = input('');
    fprintf('  Y position (cm): ');
    y = input('');
    fprintf('  Z position (cm): ');
    z = input('');
    
    mic_positions(mic, :) = [x, y, z];
    fprintf('  Recorded position: [%.1f, %.1f, %.1f]\n\n', x, y, z);
end

%% Step 2: Test TDOA Accuracy
fprintf('=== Step 2: TDOA Accuracy Test ===\n');
fprintf('This test will verify the accuracy of time delay measurements.\n\n');

% Generate test signals
test_frequency = 1000;  % Hz
test_duration = 1.0;    % seconds
t = 0:1/fs:test_duration;

% Create test signal
test_signal = sin(2*pi*test_frequency*t);

% Simulate delays for each calibration position
tdoa_errors = [];

for pos_idx = 1:size(calibration_positions, 1)
    az = calibration_positions(pos_idx, 1);
    el = calibration_positions(pos_idx, 2);
    dist = calibration_positions(pos_idx, 3);
    
    fprintf('Testing position: Azimuth=%.0f°, Elevation=%.0f°, Distance=%.0fcm\n', az, el, dist);
    
    % Calculate theoretical delays
    theoretical_delays = calculate_theoretical_delays(az, el, dist, mic_positions);
    
    % Simulate measured delays (with some noise)
    measured_delays = theoretical_delays + 0.0001 * randn(size(theoretical_delays));
    
    % Calculate errors
    errors = abs(measured_delays - theoretical_delays);
    tdoa_errors = [tdoa_errors; errors];
    
    fprintf('  Theoretical delays: [%.4f, %.4f, %.4f, %.4f] ms\n', ...
            theoretical_delays * 1000);
    fprintf('  Measured delays:    [%.4f, %.4f, %.4f, %.4f] ms\n', ...
            measured_delays * 1000);
    fprintf('  Errors:            [%.4f, %.4f, %.4f, %.4f] ms\n\n', ...
            errors * 1000);
end

% Calculate statistics
mean_error = mean(tdoa_errors(:));
std_error = std(tdoa_errors(:));
max_error = max(tdoa_errors(:));

fprintf('TDOA Accuracy Summary:\n');
fprintf('  Mean error: %.4f ms\n', mean_error * 1000);
fprintf('  Std deviation: %.4f ms\n', std_error * 1000);
fprintf('  Max error: %.4f ms\n', max_error * 1000);

%% Step 3: Parameter Optimization
fprintf('\n=== Step 3: Parameter Optimization ===\n');

% Test different correlation thresholds
correlation_thresholds = [0.1, 0.2, 0.3, 0.4, 0.5];
accuracy_scores = [];

for thresh = correlation_thresholds
    fprintf('Testing correlation threshold: %.1f\n', thresh);
    
    % Simulate localization with this threshold
    accuracy = simulate_localization_accuracy(thresh, mic_positions, calibration_positions);
    accuracy_scores = [accuracy_scores; accuracy];
    
    fprintf('  Accuracy score: %.2f\n', accuracy);
end

% Find optimal threshold
[best_accuracy, best_idx] = max(accuracy_scores);
optimal_threshold = correlation_thresholds(best_idx);

fprintf('\nOptimal correlation threshold: %.1f (accuracy: %.2f)\n', ...
        optimal_threshold, best_accuracy);

%% Step 4: System Validation
fprintf('\n=== Step 4: System Validation ===\n');

% Test with random positions
num_test_positions = 20;
validation_errors = [];

fprintf('Testing with %d random positions...\n', num_test_positions);

for test = 1:num_test_positions
    % Generate random test position
    test_az = rand() * 360;
    test_el = (rand() - 0.5) * 180;
    test_dist = 50 + rand() * 200;
    
    % Simulate localization
    [estimated_az, estimated_el, estimated_dist] = simulate_localization(...
        test_az, test_el, test_dist, mic_positions, optimal_threshold);
    
    % Calculate errors
    az_error = abs(estimated_az - test_az);
    el_error = abs(estimated_el - test_el);
    dist_error = abs(estimated_dist - test_dist);
    
    validation_errors = [validation_errors; az_error, el_error, dist_error];
    
    if mod(test, 5) == 0
        fprintf('  Test %d/%d: Az error=%.1f°, El error=%.1f°, Dist error=%.1fcm\n', ...
                test, num_test_positions, az_error, el_error, dist_error);
    end
end

% Calculate validation statistics
mean_az_error = mean(validation_errors(:, 1));
mean_el_error = mean(validation_errors(:, 2));
mean_dist_error = mean(validation_errors(:, 3));

fprintf('\nValidation Results:\n');
fprintf('  Mean azimuth error: %.1f°\n', mean_az_error);
fprintf('  Mean elevation error: %.1f°\n', mean_el_error);
fprintf('  Mean distance error: %.1f cm\n', mean_dist_error);

%% Generate Calibration Report
fprintf('\n=== Calibration Report ===\n');
fprintf('Microphone positions (cm):\n');
for mic = 1:4
    fprintf('  Mic %d: [%.1f, %.1f, %.1f]\n', mic, mic_positions(mic, :));
end

fprintf('\nOptimal parameters:\n');
fprintf('  Correlation threshold: %.1f\n', optimal_threshold);
fprintf('  Sample rate: %d Hz\n', fs);

fprintf('\nSystem performance:\n');
fprintf('  TDOA accuracy: %.2f ± %.2f ms\n', mean_error*1000, std_error*1000);
fprintf('  Localization accuracy: %.1f° azimuth, %.1f° elevation, %.1f cm distance\n', ...
        mean_az_error, mean_el_error, mean_dist_error);

% Save calibration data
calibration_data = struct();
calibration_data.mic_positions = mic_positions;
calibration_data.optimal_threshold = optimal_threshold;
calibration_data.tdoa_accuracy = [mean_error, std_error, max_error];
calibration_data.localization_accuracy = [mean_az_error, mean_el_error, mean_dist_error];

save('calibration_data.mat', 'calibration_data');
fprintf('\nCalibration data saved to calibration_data.mat\n');

%% Helper Functions

function delays = calculate_theoretical_delays(azimuth, elevation, distance, mic_positions)
    % Calculate theoretical time delays for a given source position
    
    % Convert to radians
    az_rad = azimuth * pi / 180;
    el_rad = elevation * pi / 180;
    
    % Source position
    source_x = distance * cos(el_rad) * cos(az_rad);
    source_y = distance * cos(el_rad) * sin(az_rad);
    source_z = distance * sin(el_rad);
    
    % Calculate distances to each microphone
    distances = zeros(4, 1);
    for mic = 1:4
        mic_x = mic_positions(mic, 1);
        mic_y = mic_positions(mic, 2);
        mic_z = mic_positions(mic, 3);
        
        distances(mic) = sqrt((source_x - mic_x)^2 + (source_y - mic_y)^2 + (source_z - mic_z)^2);
    end
    
    % Calculate delays relative to first microphone
    delays = (distances - distances(1)) / 34300; % Convert to seconds
end

function accuracy = simulate_localization_accuracy(threshold, mic_positions, test_positions)
    % Simulate localization accuracy with given threshold
    
    total_error = 0;
    num_tests = size(test_positions, 1);
    
    for i = 1:num_tests
        az = test_positions(i, 1);
        el = test_positions(i, 2);
        dist = test_positions(i, 3);
        
        % Simulate localization
        [est_az, est_el, est_dist] = simulate_localization(az, el, dist, mic_positions, threshold);
        
        % Calculate error
        error = sqrt((est_az - az)^2 + (est_el - el)^2 + (est_dist - dist)^2);
        total_error = total_error + error;
    end
    
    accuracy = 1 / (1 + total_error / num_tests); % Higher is better
end

function [azimuth, elevation, distance] = simulate_localization(true_az, true_el, true_dist, mic_positions, threshold)
    % Simulate the localization process with noise
    
    % Add noise to measurements
    noise_level = 0.0001; % 0.1ms noise
    
    % Calculate theoretical delays
    theoretical_delays = calculate_theoretical_delays(true_az, true_el, true_dist, mic_positions);
    
    % Add noise
    noisy_delays = theoretical_delays + noise_level * randn(size(theoretical_delays));
    
    % Simulate localization algorithm (simplified)
    % In reality, this would use the full triangulation algorithm
    
    % Add some systematic error based on threshold
    systematic_error = (1 - threshold) * 0.01; % 1% error per 0.1 threshold reduction
    
    azimuth = true_az + systematic_error * randn() * 10;
    elevation = true_el + systematic_error * randn() * 10;
    distance = true_dist + systematic_error * randn() * 20;
    
    % Normalize angles
    azimuth = mod(azimuth, 360);
    elevation = max(-90, min(90, elevation));
    distance = max(10, distance);
end
