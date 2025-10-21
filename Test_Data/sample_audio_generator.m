%% Sample Audio Data Generator for Testing
% This script generates synthetic audio data for testing the localization system
% without requiring actual hardware

clear; clc; close all;

%% Configuration
fs = 16000;                    % Sample rate (Hz)
duration = 2.0;                % Duration (seconds)
num_mics = 4;                  % Number of microphones

% Microphone array geometry (in cm)
mic_positions = [
    0.0,  0.0,  2.5;    % Mic 1 (Front)
    2.5,  0.0,  0.0;    % Mic 2 (Right)
    0.0,  0.0, -2.5;    % Mic 3 (Back)
   -2.5,  0.0,  0.0     % Mic 4 (Left)
];

% Test scenarios
test_scenarios = {
    'Single source - Front', [0, 0, 100];
    'Single source - Right', [90, 0, 100];
    'Single source - Back', [180, 0, 100];
    'Single source - Left', [270, 0, 100];
    'Single source - Up', [0, 45, 100];
    'Single source - Down', [0, -45, 100];
    'Multiple sources', [0, 0, 100; 90, 0, 150];
    'Moving source', [0, 0, 100; 45, 0, 100; 90, 0, 100; 135, 0, 100];
    'Noisy environment', [0, 0, 100];
    'Close source', [0, 0, 50];
    'Far source', [0, 0, 300]
};

%% Generate Test Data
fprintf('Generating test audio data...\n\n');

for scenario_idx = 1:length(test_scenarios)
    scenario_name = test_scenarios{scenario_idx, 1};
    source_positions = test_scenarios{scenario_idx, 2};
    
    fprintf('Generating: %s\n', scenario_name);
    
    % Generate audio data for this scenario
    [audio_data, true_positions] = generate_scenario_audio(...
        source_positions, fs, duration, mic_positions);
    
    % Save data
    filename = sprintf('test_data_%02d_%s.mat', scenario_idx, ...
                      strrep(lower(scenario_name), ' ', '_'));
    save(filename, 'audio_data', 'true_positions', 'mic_positions', 'fs', 'scenario_name');
    
    % Generate visualization
    visualize_scenario(audio_data, true_positions, mic_positions, scenario_name);
    
    fprintf('  Saved to: %s\n', filename);
    fprintf('  True positions: ');
    for i = 1:size(true_positions, 1)
        fprintf('[%.0f°, %.0f°, %.0fcm] ', true_positions(i,1), true_positions(i,2), true_positions(i,3));
    end
    fprintf('\n\n');
end

fprintf('Test data generation complete!\n');
fprintf('Generated %d test scenarios\n', length(test_scenarios));

%% Function Definitions

function [audio_data, true_positions] = generate_scenario_audio(source_positions, fs, duration, mic_positions)
    % Generate audio data for a specific scenario
    
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
        elevation = source_positions(source_idx, 2);
        distance = source_positions(source_idx, 3);
        
        % Generate source signal
        if num_sources > 1 && source_idx > 1
            % Multiple sources - use different frequencies
            frequency = 1000 + (source_idx - 1) * 200;
        else
            frequency = 1000;
        end
        
        % Generate base signal
        if strcmp(get_scenario_type(source_positions), 'moving')
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
            delay = calculate_time_delay(mic, azimuth, elevation, distance, mic_positions);
            
            % Apply delay
            delayed_signal = apply_delay(source_signal, delay, fs);
            
            % Add to microphone signal
            audio_data(mic, :) = audio_data(mic, :) + delayed_signal;
        end
    end
    
    % Add noise
    noise_level = 0.1;
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

function delay = calculate_time_delay(mic_index, azimuth, elevation, distance, mic_positions)
    % Calculate time delay for a specific microphone
    
    % Convert to radians
    az_rad = azimuth * pi / 180;
    el_rad = elevation * pi / 180;
    
    % Source position
    source_x = distance * cos(el_rad) * cos(az_rad);
    source_y = distance * cos(el_rad) * sin(az_rad);
    source_z = distance * sin(el_rad);
    
    % Microphone position
    mic_x = mic_positions(mic_index, 1);
    mic_y = mic_positions(mic_index, 2);
    mic_z = mic_positions(mic_index, 3);
    
    % Calculate distance
    mic_distance = sqrt((source_x - mic_x)^2 + (source_y - mic_y)^2 + (source_z - mic_z)^2);
    
    % Time delay relative to first microphone
    ref_distance = sqrt(source_x^2 + source_y^2 + (source_z - 2.5)^2);
    delay = (mic_distance - ref_distance) / 34300; % Convert to seconds
end

function delayed_signal = apply_delay(signal, delay, fs)
    % Apply time delay to signal
    
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

function scenario_type = get_scenario_type(source_positions)
    % Determine scenario type based on source positions
    
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

function visualize_scenario(audio_data, true_positions, mic_positions, scenario_name)
    % Create visualization for the scenario
    
    figure('Name', sprintf('Test Scenario: %s', scenario_name), ...
           'Position', [100, 100, 1200, 800]);
    
    % Time vector
    fs = 16000;
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
    scatter3(mic_positions(:,1), mic_positions(:,2), mic_positions(:,3), ...
             100, 'b', 'filled');
    hold on;
    scatter3(true_positions(:,1), true_positions(:,2), true_positions(:,3), ...
             100, 'r', 'filled');
    xlabel('X (cm)');
    ylabel('Y (cm)');
    zlabel('Z (cm)');
    title('Microphone Array & Source Positions');
    legend('Microphones', 'Sources');
    grid on;
    
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
    tdoa_matrix = calculate_tdoa_matrix(audio_data, fs);
    imagesc(tdoa_matrix);
    colorbar;
    xlabel('Microphone');
    ylabel('Microphone');
    title('TDOA Matrix (s)');
    
    % Plot source positions
    subplot(2,3,5);
    if size(true_positions, 1) == 1
        % Single source - polar plot
        polarplot(true_positions(1,1)*pi/180, true_positions(1,3), 'ro', 'MarkerSize', 10);
        title('Source Position (Polar)');
    else
        % Multiple sources - scatter plot
        scatter(true_positions(:,1), true_positions(:,2), 100, 'r', 'filled');
        xlabel('Azimuth (°)');
        ylabel('Elevation (°)');
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
    saveas(gcf, sprintf('scenario_%s.png', strrep(lower(scenario_name), ' ', '_')));
end

function tdoa_matrix = calculate_tdoa_matrix(audio_data, fs)
    % Calculate TDOA matrix for visualization
    
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

function generate_calibration_data()
    % Generate calibration data for system testing
    
    fprintf('Generating calibration data...\n');
    
    % Known test positions
    test_positions = [
        0, 0, 100;      % Front
        90, 0, 100;     % Right
        180, 0, 100;    % Back
        270, 0, 100;    % Left
        0, 45, 100;     % Front-up
        0, -45, 100     % Front-down
    ];
    
    % Generate data for each position
    for i = 1:size(test_positions, 1)
        az = test_positions(i, 1);
        el = test_positions(i, 2);
        dist = test_positions(i, 3);
        
        fprintf('  Position %d: Az=%.0f°, El=%.0f°, Dist=%.0fcm\n', i, az, el, dist);
        
        % Generate audio data
        [audio_data, ~] = generate_scenario_audio([az, el, dist], 16000, 1.0, mic_positions);
        
        % Save calibration data
        filename = sprintf('calibration_%02d_az%.0f_el%.0f_dist%.0f.mat', i, az, el, dist);
        save(filename, 'audio_data', 'az', 'el', 'dist', 'mic_positions');
    end
    
    fprintf('Calibration data generation complete!\n');
end

function generate_noise_analysis()
    % Generate data for noise analysis
    
    fprintf('Generating noise analysis data...\n');
    
    noise_levels = [0.01, 0.05, 0.1, 0.2, 0.5];
    source_position = [0, 0, 100];
    
    for i = 1:length(noise_levels)
        noise_level = noise_levels(i);
        
        fprintf('  Noise level: %.2f\n', noise_level);
        
        % Generate clean signal
        [audio_data, ~] = generate_scenario_audio(source_position, 16000, 1.0, mic_positions);
        
        % Add noise
        for mic = 1:size(audio_data, 1)
            audio_data(mic, :) = audio_data(mic, :) + noise_level * randn(size(audio_data(mic, :)));
        end
        
        % Save noise data
        filename = sprintf('noise_analysis_%.2f.mat', noise_level);
        save(filename, 'audio_data', 'noise_level', 'source_position', 'mic_positions');
    end
    
    fprintf('Noise analysis data generation complete!\n');
end

%% Run additional generators if needed
% Uncomment to generate additional test data

% generate_calibration_data();
% generate_noise_analysis();
