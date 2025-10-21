# Sound Source Localization: Theory and Algorithms

## Overview

Sound source localization using microphone arrays is based on the principle that sound waves arrive at different microphones at slightly different times. By measuring these time differences (Time Difference of Arrival - TDOA), we can determine the direction and distance of the sound source.

## Mathematical Foundation

### 1. Time Difference of Arrival (TDOA)

The TDOA between two microphones is given by:

```
τ_ij = (d_i - d_j) / c
```

Where:
- `τ_ij` = Time difference between microphones i and j
- `d_i` = Distance from source to microphone i
- `d_j` = Distance from source to microphone j  
- `c` = Speed of sound (343 m/s at 20°C)

### 2. Cross-Correlation Method

The most common method for TDOA estimation is cross-correlation:

```
R_ij(τ) = ∫ x_i(t) x_j(t + τ) dt
```

The TDOA is found at the peak of the cross-correlation function:

```
τ_ij = argmax(R_ij(τ))
```

### 3. 3D Triangulation

Given TDOA measurements, we can solve for the source position using the system of equations:

```
√((x - x_i)² + (y - y_i)² + (z - z_i)²) - √((x - x_j)² + (y - y_j)² + (z - z_j)²) = c·τ_ij
```

This is a nonlinear system that can be solved using least squares methods.

## Algorithm Implementation

### 1. Signal Preprocessing

#### High-Pass Filtering
Remove DC offset and low-frequency noise:

```matlab
[b, a] = butter(2, 50/(fs/2), 'high');
filtered_signal = filter(b, a, signal);
```

#### Windowing
Apply window function to reduce spectral leakage:

```matlab
windowed_signal = signal .* hamming(length(signal));
```

#### Normalization
Scale signals to prevent overflow:

```matlab
normalized_signal = signal / max(abs(signal));
```

### 2. TDOA Estimation

#### Cross-Correlation Algorithm
```matlab
function tdoa = estimate_tdoa(signal1, signal2, fs)
    % Calculate cross-correlation
    [correlation, lags] = xcorr(signal1, signal2);
    
    % Find peak
    [max_corr, max_idx] = max(abs(correlation));
    
    % Convert to time delay
    tdoa = lags(max_idx) / fs;
end
```

#### Generalized Cross-Correlation (GCC)
Improved method using frequency domain:

```matlab
function tdoa = gcc_phat(signal1, signal2, fs)
    % FFT of signals
    X1 = fft(signal1);
    X2 = fft(signal2);
    
    % Cross-power spectral density
    G12 = X1 .* conj(X2);
    
    % Phase transform
    G12_phat = G12 ./ abs(G12);
    
    % Inverse FFT
    correlation = ifft(G12_phat);
    
    % Find peak
    [max_corr, max_idx] = max(abs(correlation));
    tdoa = (max_idx - 1) / fs;
end
```

### 3. 3D Triangulation

#### Least Squares Solution
```matlab
function [x, y, z] = triangulate_3d(tdoa_matrix, mic_positions, speed_of_sound)
    % Build system of equations
    num_equations = 0;
    A = [];
    b = [];
    
    for i = 1:size(mic_positions, 1)
        for j = i+1:size(mic_positions, 1)
            if abs(tdoa_matrix(i,j)) > threshold
                % Distance difference
                distance_diff = tdoa_matrix(i,j) * speed_of_sound;
                
                % Equation coefficients
                A(num_equations+1, :) = 2 * (mic_positions(j,:) - mic_positions(i,:));
                b(num_equations+1) = distance_diff;
                num_equations = num_equations + 1;
            end
        end
    end
    
    % Solve using least squares
    if rank(A) >= 3
        solution = A \ b;
        x = solution(1);
        y = solution(2);
        z = solution(3);
    end
end
```

#### Spherical Coordinates Conversion
```matlab
function [azimuth, elevation, distance] = cartesian_to_spherical(x, y, z)
    distance = sqrt(x^2 + y^2 + z^2);
    azimuth = atan2(y, x) * 180 / pi;
    elevation = asin(z / distance) * 180 / pi;
    
    % Normalize angles
    if azimuth < 0
        azimuth = azimuth + 360;
    end
    if elevation < -90
        elevation = -90;
    elseif elevation > 90
        elevation = 90;
    end
end
```

## Advanced Algorithms

### 1. Kalman Filtering for Tracking

#### State Model
```matlab
% State vector: [x, y, z, vx, vy, vz]
% State transition matrix
F = [1, 0, 0, dt, 0,  0;
     0, 1, 0, 0,  dt, 0;
     0, 0, 1, 0,  0,  dt;
     0, 0, 0, 1,  0,  0;
     0, 0, 0, 0,  1,  0;
     0, 0, 0, 0,  0,  1];

% Measurement matrix
H = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];
```

#### Kalman Filter Implementation
```matlab
function [x_est, P_est] = kalman_filter(x_pred, P_pred, measurement, H, R)
    % Innovation
    innovation = measurement - H * x_pred;
    
    % Innovation covariance
    S = H * P_pred * H' + R;
    
    % Kalman gain
    K = P_pred * H' / S;
    
    % Updated state
    x_est = x_pred + K * innovation;
    
    % Updated covariance
    P_est = (eye(6) - K * H) * P_pred;
end
```

### 2. Multiple Source Localization

#### Source Separation
```matlab
function sources = separate_sources(audio_data, num_sources)
    % Use Independent Component Analysis (ICA)
    [sources, A, W] = fastica(audio_data, 'numOfIC', num_sources);
end
```

#### Clustering Algorithm
```matlab
function clusters = cluster_tdoa_measurements(tdoa_measurements)
    % Use k-means clustering
    [clusters, centroids] = kmeans(tdoa_measurements, num_sources);
end
```

### 3. Robust Estimation

#### RANSAC Algorithm
```matlab
function [best_model, inliers] = ransac_tdoa(tdoa_measurements, threshold)
    best_model = [];
    max_inliers = 0;
    
    for iteration = 1:max_iterations
        % Random sample
        sample = random_sample(tdoa_measurements, 3);
        
        % Fit model
        model = fit_model(sample);
        
        % Count inliers
        inliers = count_inliers(tdoa_measurements, model, threshold);
        
        if length(inliers) > max_inliers
            max_inliers = length(inliers);
            best_model = model;
        end
    end
end
```

## Performance Optimization

### 1. Computational Efficiency

#### FFT-based Correlation
```matlab
function correlation = fft_correlation(signal1, signal2)
    % Zero-pad signals
    N = length(signal1) + length(signal2) - 1;
    signal1_padded = [signal1, zeros(1, N - length(signal1))];
    signal2_padded = [signal2, zeros(1, N - length(signal2))];
    
    % FFT correlation
    correlation = ifft(fft(signal1_padded) .* conj(fft(signal2_padded)));
end
```

#### Parallel Processing
```matlab
% Use parallel computing for multiple TDOA calculations
parfor i = 1:num_mics
    for j = i+1:num_mics
        tdoa_matrix(i,j) = estimate_tdoa(audio_data(i,:), audio_data(j,:), fs);
    end
end
```

### 2. Real-time Processing

#### Circular Buffer
```matlab
classdef CircularBuffer < handle
    properties
        buffer
        head
        tail
        size
    end
    
    methods
        function obj = CircularBuffer(size)
            obj.buffer = zeros(size, 1);
            obj.head = 1;
            obj.tail = 1;
            obj.size = size;
        end
        
        function push(obj, data)
            obj.buffer(obj.head) = data;
            obj.head = mod(obj.head, obj.size) + 1;
        end
        
        function data = pop(obj)
            data = obj.buffer(obj.tail);
            obj.tail = mod(obj.tail, obj.size) + 1;
        end
    end
end
```

#### Adaptive Processing
```matlab
function [azimuth, elevation] = adaptive_localization(audio_data, confidence_threshold)
    if confidence > confidence_threshold
        % Use full algorithm
        [azimuth, elevation] = full_localization(audio_data);
    else
        % Use simplified algorithm
        [azimuth, elevation] = simplified_localization(audio_data);
    end
end
```

## Error Analysis

### 1. Sources of Error

#### Measurement Errors
- **Quantization noise**: Limited ADC resolution
- **Jitter**: Clock instability
- **Noise**: Environmental and electronic noise
- **Interference**: Electromagnetic interference

#### Algorithmic Errors
- **Approximation errors**: Numerical methods
- **Model errors**: Simplified physical models
- **Calibration errors**: Incorrect microphone positions
- **Processing errors**: Round-off and truncation

### 2. Error Mitigation

#### Signal Processing
```matlab
% Noise reduction
denoised_signal = wiener_filter(noisy_signal, noise_estimate);

% Outlier detection
valid_measurements = detect_outliers(tdoa_measurements, threshold);

% Smoothing
smoothed_position = kalman_filter(position_measurements);
```

#### Calibration
```matlab
% Automatic calibration
function calibration_data = auto_calibrate(mic_positions, test_sources)
    for source = test_sources
        % Measure TDOA
        measured_tdoa = measure_tdoa(source);
        
        % Calculate theoretical TDOA
        theoretical_tdoa = calculate_theoretical_tdoa(source, mic_positions);
        
        % Calculate error
        error = measured_tdoa - theoretical_tdoa;
        
        % Update calibration
        calibration_data = update_calibration(calibration_data, error);
    end
end
```

## Validation and Testing

### 1. Accuracy Metrics

#### Angular Accuracy
```matlab
function accuracy = calculate_angular_accuracy(true_azimuth, estimated_azimuth)
    error = abs(true_azimuth - estimated_azimuth);
    accuracy = 1 - error / 180; % Normalized accuracy
end
```

#### Distance Accuracy
```matlab
function accuracy = calculate_distance_accuracy(true_distance, estimated_distance)
    error = abs(true_distance - estimated_distance);
    accuracy = 1 - error / true_distance; % Relative accuracy
end
```

### 2. Performance Benchmarks

#### Real-time Performance
```matlab
function benchmark_results = performance_benchmark()
    % Test processing time
    tic;
    [azimuth, elevation] = localize_sound(audio_data);
    processing_time = toc;
    
    % Test accuracy
    accuracy = test_accuracy(test_positions);
    
    % Test robustness
    robustness = test_robustness(noise_levels);
    
    benchmark_results = struct('processing_time', processing_time, ...
                              'accuracy', accuracy, ...
                              'robustness', robustness);
end
```

This comprehensive theory and algorithm guide provides the mathematical foundation and implementation details for sound source localization using microphone arrays. The algorithms are optimized for ESP32 implementation while maintaining accuracy and real-time performance.
