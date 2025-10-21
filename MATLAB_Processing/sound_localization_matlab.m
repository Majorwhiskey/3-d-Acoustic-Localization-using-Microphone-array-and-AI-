%% Sound Source Localization using MATLAB
% Advanced signal processing and visualization for ESP32 microphone array
% 
% This script processes audio data from ESP32 and performs:
% - Advanced TDOA estimation using cross-correlation
% - 3D triangulation and localization
% - Real-time visualization
% - Performance analysis and calibration

clear; clc; close all;

%% Configuration Parameters
% Audio parameters
fs = 16000;                    % Sample rate (Hz)
buffer_size = 1024;            % Buffer size
num_mics = 4;                  % Number of microphones

% Microphone array geometry (in cm)
mic_positions = [
    0.0,  0.0,  2.5;    % Mic 1 (Front)
    2.5,  0.0,  0.0;    % Mic 2 (Right)
    0.0,  0.0, -2.5;    % Mic 3 (Back)
   -2.5,  0.0,  0.0     % Mic 4 (Left)
];

% Processing parameters
window_size = 512;             % FFT window size
overlap = 0.5;                 % Window overlap
min_correlation = 0.3;         % Minimum correlation threshold
max_distance = 500;            % Maximum detection distance (cm)
speed_of_sound = 34300;        % Speed of sound (cm/s)

%% Initialize Variables
% Audio buffers
audio_data = zeros(num_mics, buffer_size);
tdoa_matrix = zeros(num_mics, num_mics);
position_history = [];
confidence_history = [];

% Visualization setup
figure('Position', [100, 100, 1200, 800]);
subplot(2,3,1); % 3D position plot
subplot(2,3,2); % Azimuth plot
subplot(2,3,3); % Elevation plot
subplot(2,3,4); % TDOA matrix
subplot(2,3,5); % Audio waveforms
subplot(2,3,6); % Confidence plot

%% Main Processing Loop
fprintf('Starting sound source localization...\n');
fprintf('Press Ctrl+C to stop\n\n');

try
    while true
        % Simulate data acquisition (replace with actual ESP32 communication)
        audio_data = simulate_audio_data(fs, buffer_size, num_mics);
        
        % Process audio data
        [azimuth, elevation, distance, confidence] = process_audio_data(...
            audio_data, mic_positions, fs, window_size, overlap, ...
            min_correlation, max_distance, speed_of_sound);
        
        % Update history
        if confidence > 0.5
            position_history = [position_history; azimuth, elevation, distance];
            confidence_history = [confidence_history; confidence];
            
            % Keep only last 100 measurements
            if size(position_history, 1) > 100
                position_history = position_history(end-99:end, :);
                confidence_history = confidence_history(end-99:end);
            end
        end
        
        % Update visualizations
        update_visualizations(audio_data, position_history, confidence_history, ...
                            mic_positions, azimuth, elevation, distance, confidence);
        
        % Display results
        if confidence > 0.5
            fprintf('Azimuth: %.1f°, Elevation: %.1f°, Distance: %.1f cm, Confidence: %.2f\n', ...
                    azimuth, elevation, distance, confidence);
        end
        
        pause(0.1); % 100ms update rate
    end
    
catch ME
    if strcmp(ME.identifier, 'MATLAB:interrupted')
        fprintf('\nLocalization stopped by user\n');
    else
        rethrow(ME);
    end
end

%% Function Definitions

function audio_data = simulate_audio_data(fs, buffer_size, num_mics)
    % Simulate audio data from microphone array
    % In real implementation, this would read from ESP32 via serial/WiFi
    
    t = (0:buffer_size-1) / fs;
    audio_data = zeros(num_mics, buffer_size);
    
    % Simulate a sound source at random position
    source_azimuth = rand() * 360;
    source_elevation = (rand() - 0.5) * 180;
    source_distance = 50 + rand() * 200; % 50-250 cm
    
    % Add some noise
    noise_level = 0.1;
    
    for mic = 1:num_mics
        % Calculate time delay for this microphone
        delay = calculate_time_delay(mic, source_azimuth, source_elevation, source_distance);
        
        % Generate delayed signal
        signal = sin(2*pi*1000*t) .* exp(-t*2); % 1kHz tone with decay
        delayed_signal = [zeros(1, round(delay*fs)), signal(1:end-round(delay*fs))];
        
        % Add noise
        audio_data(mic, :) = delayed_signal + noise_level * randn(1, buffer_size);
    end
end

function delay = calculate_time_delay(mic_index, azimuth, elevation, distance)
    % Calculate theoretical time delay for a microphone
    % This is used for simulation - in real system, delays are measured
    
    % Convert to radians
    az_rad = azimuth * pi / 180;
    el_rad = elevation * pi / 180;
    
    % Source position
    source_x = distance * cos(el_rad) * cos(az_rad);
    source_y = distance * cos(el_rad) * sin(az_rad);
    source_z = distance * sin(el_rad);
    
    % Microphone positions (hardcoded for simulation)
    mic_pos = [
        0, 0, 2.5;
        2.5, 0, 0;
        0, 0, -2.5;
        -2.5, 0, 0
    ];
    
    % Calculate distance from source to microphone
    mic_x = mic_pos(mic_index, 1);
    mic_y = mic_pos(mic_index, 2);
    mic_z = mic_pos(mic_index, 3);
    
    mic_distance = sqrt((source_x - mic_x)^2 + (source_y - mic_y)^2 + (source_z - mic_z)^2);
    
    % Time delay relative to first microphone
    ref_distance = sqrt(source_x^2 + source_y^2 + (source_z - 2.5)^2);
    delay = (mic_distance - ref_distance) / 34300; % Convert to seconds
end

function [azimuth, elevation, distance, confidence] = process_audio_data(...
    audio_data, mic_positions, fs, window_size, overlap, min_correlation, ...
    max_distance, speed_of_sound)
    
    % Initialize outputs
    azimuth = 0;
    elevation = 0;
    distance = 0;
    confidence = 0;
    
    % Preprocess audio data
    processed_data = preprocess_audio(audio_data, fs);
    
    % Calculate TDOA matrix
    tdoa_matrix = calculate_tdoa_matrix(processed_data, fs, window_size, overlap);
    
    % Find valid TDOA measurements
    valid_pairs = find_valid_tdoa_pairs(tdoa_matrix, min_correlation);
    
    if size(valid_pairs, 1) < 3
        return; % Not enough valid measurements
    end
    
    % Perform 3D triangulation
    [azimuth, elevation, distance, confidence] = triangulate_3d(...
        tdoa_matrix, mic_positions, valid_pairs, speed_of_sound, max_distance);
end

function processed_data = preprocess_audio(audio_data, fs)
    % Preprocess audio data: filtering, normalization, etc.
    
    [num_mics, buffer_size] = size(audio_data);
    processed_data = zeros(size(audio_data));
    
    for mic = 1:num_mics
        % High-pass filter to remove DC offset
        [b, a] = butter(2, 50/(fs/2), 'high');
        processed_data(mic, :) = filter(b, a, audio_data(mic, :));
        
        % Normalize
        max_val = max(abs(processed_data(mic, :)));
        if max_val > 0
            processed_data(mic, :) = processed_data(mic, :) / max_val;
        end
    end
end

function tdoa_matrix = calculate_tdoa_matrix(audio_data, fs, window_size, overlap)
    % Calculate TDOA matrix using cross-correlation
    
    [num_mics, buffer_size] = size(audio_data);
    tdoa_matrix = zeros(num_mics, num_mics);
    
    for i = 1:num_mics
        for j = 1:num_mics
            if i ~= j
                % Calculate cross-correlation
                [correlation, lags] = xcorr(audio_data(i, :), audio_data(j, :));
                
                % Find peak
                [max_corr, max_idx] = max(abs(correlation));
                
                % Convert lag to time delay
                delay_samples = lags(max_idx);
                tdoa_matrix(i, j) = delay_samples / fs;
            end
        end
    end
end

function valid_pairs = find_valid_tdoa_pairs(tdoa_matrix, min_correlation)
    % Find valid TDOA measurement pairs
    
    [num_mics, ~] = size(tdoa_matrix);
    valid_pairs = [];
    
    for i = 1:num_mics
        for j = i+1:num_mics
            if abs(tdoa_matrix(i, j)) > min_correlation
                valid_pairs = [valid_pairs; i, j, tdoa_matrix(i, j)];
            end
        end
    end
end

function [azimuth, elevation, distance, confidence] = triangulate_3d(...
    tdoa_matrix, mic_positions, valid_pairs, speed_of_sound, max_distance)
    
    % Initialize outputs
    azimuth = 0;
    elevation = 0;
    distance = 0;
    confidence = 0;
    
    if size(valid_pairs, 1) < 3
        return;
    end
    
    % Set up system of equations for triangulation
    num_equations = size(valid_pairs, 1);
    A = zeros(num_equations, 3);
    b = zeros(num_equations, 1);
    
    for eq = 1:num_equations
        i = valid_pairs(eq, 1);
        j = valid_pairs(eq, 2);
        tdoa = valid_pairs(eq, 3);
        
        % Distance difference
        distance_diff = tdoa * speed_of_sound;
        
        % Microphone positions
        mic_i = mic_positions(i, :);
        mic_j = mic_positions(j, :);
        
        % Equation coefficients
        A(eq, :) = 2 * (mic_j - mic_i);
        b(eq) = distance_diff;
    end
    
    % Solve using least squares
    if rank(A) >= 3
        x = A \ b;
        
        % Convert to spherical coordinates
        distance = norm(x);
        if distance > 0 && distance <= max_distance
            azimuth = atan2(x(2), x(1)) * 180 / pi;
            elevation = asin(x(3) / distance) * 180 / pi;
            
            % Calculate confidence based on residual
            residual = norm(A * x - b);
            confidence = max(0, 1 - residual / max_distance);
        end
    end
end

function update_visualizations(audio_data, position_history, confidence_history, ...
                             mic_positions, azimuth, elevation, distance, confidence)
    
    % Clear subplots
    for i = 1:6
        subplot(2,3,i);
        cla;
    end
    
    % 3D position plot
    subplot(2,3,1);
    if ~isempty(position_history)
        scatter3(position_history(:,1), position_history(:,2), position_history(:,3), ...
                50, confidence_history, 'filled');
        colorbar;
        xlabel('Azimuth (°)');
        ylabel('Elevation (°)');
        zlabel('Distance (cm)');
        title('3D Position History');
        grid on;
    end
    
    % Azimuth plot
    subplot(2,3,2);
    if ~isempty(position_history)
        plot(1:length(position_history), position_history(:,1), 'b-', 'LineWidth', 2);
        xlabel('Time');
        ylabel('Azimuth (°)');
        title('Azimuth vs Time');
        grid on;
        ylim([0, 360]);
    end
    
    % Elevation plot
    subplot(2,3,3);
    if ~isempty(position_history)
        plot(1:length(position_history), position_history(:,2), 'r-', 'LineWidth', 2);
        xlabel('Time');
        ylabel('Elevation (°)');
        title('Elevation vs Time');
        grid on;
        ylim([-90, 90]);
    end
    
    % TDOA matrix
    subplot(2,3,4);
    tdoa_matrix = calculate_tdoa_matrix(audio_data, 16000, 512, 0.5);
    imagesc(tdoa_matrix);
    colorbar;
    xlabel('Microphone');
    ylabel('Microphone');
    title('TDOA Matrix (s)');
    
    % Audio waveforms
    subplot(2,3,5);
    t = (0:size(audio_data,2)-1) / 16000;
    for mic = 1:size(audio_data,1)
        plot(t, audio_data(mic,:) + mic*0.5, 'LineWidth', 1);
        hold on;
    end
    xlabel('Time (s)');
    ylabel('Amplitude');
    title('Audio Waveforms');
    legend('Mic 1', 'Mic 2', 'Mic 3', 'Mic 4');
    grid on;
    
    % Confidence plot
    subplot(2,3,6);
    if ~isempty(confidence_history)
        plot(1:length(confidence_history), confidence_history, 'g-', 'LineWidth', 2);
        xlabel('Time');
        ylabel('Confidence');
        title('Confidence vs Time');
        grid on;
        ylim([0, 1]);
    end
    
    drawnow;
end
