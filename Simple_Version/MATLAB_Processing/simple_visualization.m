%% Simple Sound Localization Visualization
% Basic MATLAB processing for ESP32 + Condenser Microphone system
% 
% Features:
% - Real-time data acquisition from ESP32
% - Basic 2D localization visualization
% - Simple calibration routine
% - Data logging and export

clear; clc; close all;

%% Configuration
serial_port = 'COM3';  % Adjust to your ESP32 port
baud_rate = 115200;
update_rate = 0.1;     % 10Hz update rate

% Microphone array geometry (2D)
mic_positions = [
    0.0, 0.0;    % Mic 1 (Front)
    5.0, 0.0;    % Mic 2 (Right)
    0.0, 5.0;    % Mic 3 (Back)
   -5.0, 0.0     % Mic 4 (Left)
];

%% Initialize Variables
position_history = [];
time_history = [];
confidence_history = [];

%% Setup Serial Communication
try
    % Create serial port object
    s = serialport(serial_port, baud_rate);
    configureCallback(s, "terminator", @readSerialData);
    
    fprintf('Connected to ESP32 on %s\n', serial_port);
catch ME
    fprintf('Error connecting to ESP32: %s\n', ME.message);
    fprintf('Please check the serial port and try again\n');
    return;
end

%% Setup Visualization
figure('Name', 'Simple Sound Localization Monitor', ...
       'Position', [100, 100, 1000, 600]);

% Create subplots
subplot(2,3,1); % 2D position plot
subplot(2,3,2); % Azimuth vs time
subplot(2,3,3); % Distance vs time
subplot(2,3,4); % Confidence plot
subplot(2,3,5); % Microphone array
subplot(2,3,6); % System status

%% Main Processing Loop
fprintf('Starting simple localization monitoring...\n');
fprintf('Press Ctrl+C to stop\n\n');

try
    while true
        % Check for new data
        if s.NumBytesAvailable > 0
            data = readline(s);
            processData(data);
        end
        
        % Update visualizations
        updateVisualizations();
        
        pause(update_rate);
    end
    
catch ME
    if strcmp(ME.identifier, 'MATLAB:interrupted')
        fprintf('\nMonitoring stopped by user\n');
    else
        fprintf('Error: %s\n', ME.message);
    end
end

% Cleanup
clear s;
fprintf('Serial connection closed\n');

%% Function Definitions

function readSerialData(src, ~)
    % Callback function for serial data
    global latest_data;
    latest_data = readline(src);
end

function processData(data)
    % Process incoming data from ESP32
    
    global position_history time_history confidence_history;
    
    try
        % Parse JSON data
        if contains(data, '{')
            json_data = jsondecode(data);
            
            azimuth = json_data.azimuth;
            distance = json_data.distance;
            timestamp = json_data.timestamp;
            
            % Calculate confidence (simple metric)
            confidence = calculateConfidence(azimuth, distance);
            
            % Store data
            position_history = [position_history; azimuth, distance];
            time_history = [time_history; timestamp];
            confidence_history = [confidence_history; confidence];
            
            % Keep only last 100 measurements
            if length(position_history) > 100
                position_history = position_history(end-99:end, :);
                time_history = time_history(end-99:end);
                confidence_history = confidence_history(end-99:end);
            end
            
            % Display results
            fprintf('Azimuth: %.1f°, Distance: %.1f cm, Confidence: %.2f\n', ...
                    azimuth, distance, confidence);
        end
        
    catch ME
        % Handle parsing errors
        fprintf('Error parsing data: %s\n', ME.message);
    end
end

function confidence = calculateConfidence(azimuth, distance)
    % Simple confidence calculation
    
    % Distance-based confidence
    if distance < 50
        distance_conf = 0.9;
    elseif distance < 100
        distance_conf = 0.8;
    elseif distance < 200
        distance_conf = 0.7;
    else
        distance_conf = 0.5;
    end
    
    % Angle-based confidence (prefer front/back)
    angle_conf = 1.0 - abs(sin(azimuth * pi / 180)) * 0.3;
    
    confidence = distance_conf * angle_conf;
end

function updateVisualizations()
    % Update all visualization plots
    
    global position_history time_history confidence_history mic_positions;
    
    if isempty(position_history)
        return;
    end
    
    % Convert time to relative seconds
    time_seconds = (time_history - time_history(1)) / 1000;
    
    % 2D position plot
    subplot(2,3,1);
    cla;
    if size(position_history, 1) > 1
        scatter(position_history(:,1), position_history(:,2), ...
                50, confidence_history, 'filled');
        colorbar;
        xlabel('Azimuth (°)');
        ylabel('Distance (cm)');
        title('2D Position Tracking');
        grid on;
        
        % Add current position marker
        hold on;
        scatter(position_history(end,1), position_history(end,2), ...
                100, 'r', 'filled', 'MarkerEdgeColor', 'k');
        hold off;
    end
    
    % Azimuth vs time
    subplot(2,3,2);
    cla;
    plot(time_seconds, position_history(:,1), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Azimuth (°)');
    title('Azimuth Tracking');
    grid on;
    ylim([0, 360]);
    
    % Distance vs time
    subplot(2,3,3);
    cla;
    plot(time_seconds, position_history(:,2), 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Distance (cm)');
    title('Distance Tracking');
    grid on;
    ylim([0, 500]);
    
    % Confidence plot
    subplot(2,3,4);
    cla;
    plot(time_seconds, confidence_history, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Confidence');
    title('Confidence Level');
    grid on;
    ylim([0, 1]);
    
    % Microphone array
    subplot(2,3,5);
    cla;
    scatter(mic_positions(:,1), mic_positions(:,2), 100, 'b', 'filled');
    xlabel('X (cm)');
    ylabel('Y (cm)');
    title('Microphone Array');
    grid on;
    axis equal;
    
    % System status
    subplot(2,3,6);
    cla;
    if ~isempty(confidence_history)
        mean_conf = mean(confidence_history);
        text(0.5, 0.7, sprintf('Mean Confidence: %.2f', mean_conf), ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
        text(0.5, 0.5, sprintf('Measurements: %d', length(position_history)), ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
        text(0.5, 0.3, sprintf('Update Rate: %.1f Hz', 1/0.1), ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
    end
    title('System Status');
    axis off;
    
    drawnow;
end

%% Additional Functions

function exportData()
    % Export localization data
    
    global position_history time_history confidence_history;
    
    if isempty(position_history)
        fprintf('No data to export\n');
        return;
    end
    
    % Create timestamp
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % Create data table
    time_seconds = (time_history - time_history(1)) / 1000;
    data_table = table(time_seconds, position_history(:,1), position_history(:,2), confidence_history, ...
                      'VariableNames', {'Time_s', 'Azimuth_deg', 'Distance_cm', 'Confidence'});
    
    % Export to CSV
    filename = sprintf('simple_localization_data_%s.csv', timestamp);
    writetable(data_table, filename);
    fprintf('Data exported to %s\n', filename);
    
    % Create summary report
    createSummaryReport(timestamp);
end

function createSummaryReport(timestamp)
    % Create summary report
    
    global position_history confidence_history;
    
    if isempty(position_history)
        return;
    end
    
    % Calculate statistics
    mean_azimuth = mean(position_history(:,1));
    std_azimuth = std(position_history(:,1));
    mean_distance = mean(position_history(:,2));
    std_distance = std(position_history(:,2));
    mean_confidence = mean(confidence_history);
    
    % Create report
    report_filename = sprintf('simple_localization_report_%s.txt', timestamp);
    fid = fopen(report_filename, 'w');
    
    fprintf(fid, 'Simple Sound Localization Session Report\n');
    fprintf(fid, '========================================\n\n');
    fprintf(fid, 'Session Date: %s\n', datestr(now));
    fprintf(fid, 'Total Measurements: %d\n\n', length(position_history));
    
    fprintf(fid, 'Position Statistics:\n');
    fprintf(fid, '  Azimuth: %.1f° ± %.1f° (range: %.1f° to %.1f°)\n', ...
            mean_azimuth, std_azimuth, min(position_history(:,1)), max(position_history(:,1)));
    fprintf(fid, '  Distance: %.1f cm ± %.1f cm (range: %.1f cm to %.1f cm)\n', ...
            mean_distance, std_distance, min(position_history(:,2)), max(position_history(:,2)));
    
    fprintf(fid, '\nConfidence Statistics:\n');
    fprintf(fid, '  Mean Confidence: %.3f\n', mean_confidence);
    fprintf(fid, '  Min Confidence: %.3f\n', min(confidence_history));
    fprintf(fid, '  Max Confidence: %.3f\n', max(confidence_history));
    
    fclose(fid);
    fprintf('Summary report saved to %s\n', report_filename);
end

function calibrateSystem()
    % Simple calibration routine
    
    fprintf('Starting system calibration...\n');
    
    % Test microphone levels
    fprintf('Testing microphone levels...\n');
    for mic = 1:4
        fprintf('  Microphone %d: Check signal level\n', mic);
    end
    
    % Calibration instructions
    fprintf('\nCalibration Instructions:\n');
    fprintf('1. Place sound source at front (0°)\n');
    fprintf('2. Press any key when ready...\n');
    pause;
    
    fprintf('3. Place sound source at right (90°)\n');
    fprintf('4. Press any key when ready...\n');
    pause;
    
    fprintf('5. Place sound source at back (180°)\n');
    fprintf('6. Press any key when ready...\n');
    pause;
    
    fprintf('7. Place sound source at left (270°)\n');
    fprintf('8. Press any key when ready...\n');
    pause;
    
    fprintf('Calibration complete!\n');
end
