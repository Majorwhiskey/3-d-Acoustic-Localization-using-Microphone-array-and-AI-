%% Simple Test Data Generator
% Generate synthetic test data for the simple localization system
% Uses basic signal processing suitable for ESP32 + condenser microphones

clear; clc; close all;

%% Configuration
fs = 8000;                      % Sample rate (Hz)
duration = 1.0;                 % Duration (seconds)
num_mics = 4;                   % Number of microphones

% Simple 2D microphone array geometry (in cm)
mic_positions = [
    0.0, 0.0;    % Mic 1 (Front)
    5.0, 0.0;    % Mic 2 (Right)
    0.0, 5.0;    % Mic 3 (Back)
   -5.0, 0.0     % Mic 4 (Left)
];

% Test scenarios for simple system
test_scenarios = {
    'Front source', [0, 50];
    'Right source', [90, 50];
    'Back source', [180, 50];
    'Left source', [270, 50];
    'Close source', [0, 25];
    'Far source', [0, 100];
    'Moving source', [0, 50; 45, 50; 90, 50; 135, 50];
    'Noisy environment', [0, 50];
    'Multiple sources', [0, 50; 90, 75];
    'Edge case - very close', [0, 10]
};

%% Generate Test Data
fprintf('Generating simple test data...\n\n');

for scenario_idx = 1:length(test_scenarios)
    scenario_name = test_scenarios{scenario_idx, 1};
    source_positions = test_scenarios{scenario_idx, 2};
    
    fprintf('Generating: %s\n', scenario_name);
    
    % Generate audio data for this scenario
    [audio_data, true_positions] = generateSimpleScenario(...
        source_positions, fs, duration, mic_positions);
    
    % Save data
    filename = sprintf('simple_test_%02d_%s.mat', scenario_idx, ...
                      strrep(lower(scenario_name), ' ', '_'));
    save(filename, 'audio_data', 'true_positions', 'mic_positions', 'fs', 'scenario_name');
    
    % Generate visualization
    visualizeSimpleScenario(audio_data, true_positions, mic_positions, scenario_name);
    
    fprintf('  Saved to: %s\n', filename);
    fprintf('  True positions: ');
    for i = 1:size(true_positions, 1)
        fprintf('[%.0f°, %.0fcm] ', true_positions(i,1), true_positions(i,2));
    end
    fprintf('\n\n');
end

fprintf('Simple test data generation complete!\n');
fprintf('Generated %d test scenarios\n', length(test_scenarios));

%% Function Definitions

function [audio_data, true_positions] = generateSimpleScenario(source_positions, fs, duration, mic_positions)
    % Generate audio data for a simple scenario
    
    num_samples = round(fs * duration);
    num_mics = size(mic_positions, 1);
    num_sources = size(source_positions, 1);
    
    % Initialize audio data
    audio_data = zeros(num_mics, num_samples);
    true_positions = source_positions;
    
    % Time vector
    t = (0:num_samples-1) / fs;
    
    for source_idx = 1:num_sources
        % Source parameters
        azimuth = source_positions(source_idx, 1);
        distance = source_positions(source_idx, 2);
        
        % Generate source signal (simple sine wave)
        if num_sources > 1 && source_idx > 1
            % Multiple sources - use different frequencies
            frequency = 1000 + (source_idx - 1) * 200;
        else
            frequency = 1000;
        end
        
        % Generate base signal
        if strcmp(getSimpleScenarioType(source_positions), 'moving')
            % Moving source - frequency modulation
            source_signal = sin(2*pi*frequency*t + 0.1*sin(2*pi*0.5*t));
        else
            % Static source
            source_signal = sin(2*pi*frequency*t);
        end
        
        % Add envelope for natural decay
        envelope = exp(-t*0.5);
        source_signal = source_signal .* envelope;
        
        % Add to each microphone with appropriate delay
        for mic = 1:num_mics
            % Calculate time delay
            delay = calculateSimpleTimeDelay(mic, azimuth, distance, mic_positions);
            
            % Apply delay
            delayed_signal = applySimpleDelay(source_signal, delay, fs);
            
            % Add to microphone signal
            audio_data(mic, :) = audio_data(mic, :) + delayed_signal;
        end
    end
    
    % Add noise (simpler than full version)
    noise_level = 0.05;  % Lower noise for simple system
    for mic = 1:num_mics
        audio_data(mic, :) = audio_data(mic, :) + noise_level * randn(1, num_samples);
    end
    
    % Normalize
    for mic = 1:num_mics
        max_val = max(abs(audio_data(mic, :)));
        if max_val > 0
            audio_data(mic, :) = audio_data(mic, :) / max_val;
        end
    end
end

function delay = calculateSimpleTimeDelay(mic_index, azimuth, distance, mic_positions)
    % Calculate time delay for simple 2D system
    
    % Convert to radians
    az_rad = azimuth * pi / 180;
    
    % Source position
    source_x = distance * cos(az_rad);
    source_y = distance * sin(az_rad);
    
    % Microphone position
    mic_x = mic_positions(mic_index, 1);
    mic_y = mic_positions(mic_index, 2);
    
    % Calculate distance
    mic_distance = sqrt((source_x - mic_x)^2 + (source_y - mic_y)^2);
    
    % Time delay relative to first microphone
    ref_distance = sqrt(source_x^2 + source_y^2);
    delay = (mic_distance - ref_distance) / 34300; % Convert to seconds
end

function delayed_signal = applySimpleDelay(signal, delay, fs)
    % Apply time delay to signal (simplified)
    
    delay_samples = round(delay * fs);
    
    if delay_samples > 0
        % Positive delay - shift right
        delayed_signal = [zeros(1, delay_samples), signal(1:end-delay_samples)];
    elseif delay_samples < 0
        % Negative delay - shift left
        delayed_signal = [signal(-delay_samples+1:end), zeros(1, -delay_samples)];
    else
        % No delay
        delayed_signal = signal;
    end
end

function scenario_type = getSimpleScenarioType(source_positions)
    % Determine scenario type for simple system
    
    if size(source_positions, 1) > 1
        % Check if positions are changing (moving source)
        position_changes = diff(source_positions);
        if any(position_changes(:) ~= 0)
            scenario_type = 'moving';
        else
            scenario_type = 'multiple';
        end
    else
        scenario_type = 'single';
    end
end

function visualizeSimpleScenario(audio_data, true_positions, mic_positions, scenario_name)
    % Create visualization for simple scenario
    
    figure('Name', sprintf('Simple Test: %s', scenario_name), ...
           'Position', [100, 100, 1000, 600]);
    
    % Time vector
    fs = 8000;
    t = (0:size(audio_data, 2)-1) / fs;
    
    % Plot audio waveforms
    subplot(2,3,1);
    for mic = 1:size(audio_data, 1)
        plot(t, audio_data(mic, :) + mic*0.5, 'LineWidth', 1);
        hold on;
    end
    xlabel('Time (s)');
    ylabel('Amplitude');
    title('Audio Waveforms');
    legend('Mic 1', 'Mic 2', 'Mic 3', 'Mic 4');
    grid on;
    
    % Plot microphone array
    subplot(2,3,2);
    scatter(mic_positions(:,1), mic_positions(:,2), 100, 'b', 'filled');
    hold on;
    scatter(true_positions(:,1), true_positions(:,2), 100, 'r', 'filled');
    xlabel('X (cm)');
    ylabel('Y (cm)');
    title('Microphone Array & Source Positions');
    legend('Microphones', 'Sources');
    grid on;
    axis equal;
    
    % Plot frequency spectrum
    subplot(2,3,3);
    for mic = 1:size(audio_data, 1)
        [f, P] = pwelch(audio_data(mic, :), [], [], [], fs);
        semilogx(f, 10*log10(P));
        hold on;
    end
    xlabel('Frequency (Hz)');
    ylabel('Power (dB)');
    title('Frequency Spectrum');
    legend('Mic 1', 'Mic 2', 'Mic 3', 'Mic 4');
    grid on;
    
    % Plot TDOA analysis
    subplot(2,3,4);
    tdoa_matrix = calculateSimpleTDOA(audio_data, fs);
    imagesc(tdoa_matrix);
    colorbar;
    xlabel('Microphone');
    ylabel('Microphone');
    title('TDOA Matrix (s)');
    
    % Plot source positions (polar)
    subplot(2,3,5);
    if size(true_positions, 1) == 1
        % Single source - polar plot
        polarplot(true_positions(1,1)*pi/180, true_positions(1,2), 'ro', 'MarkerSize', 10);
        title('Source Position (Polar)');
    else
        % Multiple sources - scatter plot
        scatter(true_positions(:,1), true_positions(:,2), 100, 'r', 'filled');
        xlabel('Azimuth (°)');
        ylabel('Distance (cm)');
        title('Source Positions');
        grid on;
    end
    
    % Plot signal correlation
    subplot(2,3,6);
    correlation = corrcoef(audio_data');
    imagesc(correlation);
    colorbar;
    xlabel('Microphone');
    ylabel('Microphone');
    title('Signal Correlation');
    
    % Save figure
    saveas(gcf, sprintf('simple_scenario_%s.png', strrep(lower(scenario_name), ' ', '_')));
end

function tdoa_matrix = calculateSimpleTDOA(audio_data, fs)
    % Calculate TDOA matrix for simple system
    
    num_mics = size(audio_data, 1);
    tdoa_matrix = zeros(num_mics, num_mics);
    
    for i = 1:num_mics
        for j = 1:num_mics
            if i ~= j
                % Calculate cross-correlation
                [correlation, lags] = xcorr(audio_data(i, :), audio_data(j, :));
                
                % Find peak
                [max_corr, max_idx] = max(abs(correlation));
                
                % Convert to time delay
                delay_samples = lags(max_idx);
                tdoa_matrix(i, j) = delay_samples / fs;
            end
        end
    end
end

%% Additional Test Functions

function generateSimpleCalibrationData()
    % Generate calibration data for simple system
    
    fprintf('Generating simple calibration data...\n');
    
    % Known test positions (2D only)
    test_positions = [
        0, 50;      % Front
        90, 50;     % Right
        180, 50;    % Back
        270, 50;    % Left
        0, 25;      % Close
        0, 100      % Far
    ];
    
    % Generate data for each position
    for i = 1:size(test_positions, 1)
        az = test_positions(i, 1);
        dist = test_positions(i, 2);
        
        fprintf('  Position %d: Az=%.0f°, Dist=%.0fcm\n', i, az, dist);
        
        % Generate audio data
        [audio_data, ~] = generateSimpleScenario([az, dist], 8000, 1.0, mic_positions);
        
        % Save calibration data
        filename = sprintf('simple_calibration_%02d_az%.0f_dist%.0f.mat', i, az, dist);
        save(filename, 'audio_data', 'az', 'dist', 'mic_positions');
    end
    
    fprintf('Simple calibration data generation complete!\n');
end

function generateSimpleNoiseAnalysis()
    % Generate noise analysis data for simple system
    
    fprintf('Generating simple noise analysis data...\n');
    
    noise_levels = [0.01, 0.05, 0.1, 0.2];
    source_position = [0, 50];
    
    for i = 1:length(noise_levels)
        noise_level = noise_levels(i);
        
        fprintf('  Noise level: %.2f\n', noise_level);
        
        % Generate clean signal
        [audio_data, ~] = generateSimpleScenario(source_position, 8000, 1.0, mic_positions);
        
        % Add noise
        for mic = 1:size(audio_data, 1)
            audio_data(mic, :) = audio_data(mic, :) + noise_level * randn(size(audio_data(mic, :)));
        end
        
        % Save noise data
        filename = sprintf('simple_noise_analysis_%.2f.mat', noise_level);
        save(filename, 'audio_data', 'noise_level', 'source_position', 'mic_positions');
    end
    
    fprintf('Simple noise analysis data generation complete!\n');
end

%% Run additional generators if needed
% Uncomment to generate additional test data

% generateSimpleCalibrationData();
% generateSimpleNoiseAnalysis();
