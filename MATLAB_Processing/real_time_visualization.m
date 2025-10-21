%% Real-time Visualization for Sound Source Localization
% Advanced visualization tools for monitoring and analyzing localization results
%
% Features:
% - 3D position tracking
% - Real-time plots
% - Performance metrics
% - Data logging and analysis

function real_time_visualization()
    % Initialize visualization system
    setup_visualization();
    
    % Start real-time monitoring
    monitor_localization();
end

function setup_visualization()
    % Create main figure with subplots
    fig = figure('Name', 'Sound Source Localization Monitor', ...
                 'Position', [100, 100, 1400, 900], ...
                 'NumberTitle', 'off');
    
    % Create subplots
    subplot(2,4,1); % 3D position plot
    subplot(2,4,2); % Azimuth vs time
    subplot(2,4,3); % Elevation vs time
    subplot(2,4,4); % Distance vs time
    subplot(2,4,5); % Confidence plot
    subplot(2,4,6); % TDOA matrix
    subplot(2,4,7); % Audio spectrum
    subplot(2,4,8); % Performance metrics
    
    % Initialize data storage
    global position_data confidence_data time_data performance_data;
    position_data = [];
    confidence_data = [];
    time_data = [];
    performance_data = [];
    
    % Set up timer for real-time updates
    timer_obj = timer('ExecutionMode', 'fixedRate', ...
                     'Period', 0.1, ...
                     'TimerFcn', @update_visualization);
    start(timer_obj);
    
    fprintf('Real-time visualization started. Press Ctrl+C to stop.\n');
end

function monitor_localization()
    % Main monitoring loop
    try
        while true
            % Read data from ESP32 (simulate for now)
            [azimuth, elevation, distance, confidence, tdoa_matrix, audio_spectrum] = read_esp32_data();
            
            % Update data storage
            update_data_storage(azimuth, elevation, distance, confidence);
            
            % Update visualizations
            update_all_plots(azimuth, elevation, distance, confidence, tdoa_matrix, audio_spectrum);
            
            pause(0.1); % 100ms update rate
        end
    catch ME
        if strcmp(ME.identifier, 'MATLAB:interrupted')
            fprintf('\nVisualization stopped by user\n');
        else
            rethrow(ME);
        end
    end
end

function [azimuth, elevation, distance, confidence, tdoa_matrix, audio_spectrum] = read_esp32_data()
    % Read data from ESP32 via serial communication
    % This is a simulation - replace with actual serial communication
    
    persistent serial_port;
    
    % Initialize serial port if needed
    if isempty(serial_port)
        try
            serial_port = serialport('COM3', 115200); % Adjust port as needed
            configureCallback(serial_port, "terminator", @read_serial_data);
        catch
            % Use simulation if serial port not available
            serial_port = 'simulation';
        end
    end
    
    if strcmp(serial_port, 'simulation')
        % Simulate data
        azimuth = 45 + 10 * randn();
        elevation = 15 + 5 * randn();
        distance = 100 + 20 * randn();
        confidence = 0.7 + 0.2 * rand();
        
        % Simulate TDOA matrix
        tdoa_matrix = 0.001 * randn(4, 4);
        tdoa_matrix = tdoa_matrix - diag(diag(tdoa_matrix)); % Zero diagonal
        
        % Simulate audio spectrum
        audio_spectrum = rand(1, 256);
    else
        % Read from actual serial port
        if serial_port.NumBytesAvailable > 0
            data = readline(serial_port);
            % Parse JSON data
            json_data = jsondecode(data);
            azimuth = json_data.azimuth;
            elevation = json_data.elevation;
            distance = json_data.distance;
            confidence = json_data.confidence;
        else
            % Use previous values
            azimuth = 0;
            elevation = 0;
            distance = 0;
            confidence = 0;
        end
    end
end

function read_serial_data(src, ~)
    % Callback function for serial data
    global latest_data;
    latest_data = readline(src);
end

function update_data_storage(azimuth, elevation, distance, confidence)
    % Update global data storage
    global position_data confidence_data time_data performance_data;
    
    current_time = now;
    
    % Store position data
    position_data = [position_data; azimuth, elevation, distance];
    confidence_data = [confidence_data; confidence];
    time_data = [time_data; current_time];
    
    % Calculate performance metrics
    if length(position_data) > 1
        % Calculate position stability
        position_variance = var(position_data(end-10:end, :));
        stability_score = 1 / (1 + mean(position_variance));
        
        % Calculate confidence trend
        if length(confidence_data) > 5
            confidence_trend = mean(confidence_data(end-4:end)) - mean(confidence_data(end-9:end-5));
        else
            confidence_trend = 0;
        end
        
        performance_data = [performance_data; stability_score, confidence_trend];
    end
    
    % Keep only last 1000 measurements
    max_history = 1000;
    if length(position_data) > max_history
        position_data = position_data(end-max_history+1:end, :);
        confidence_data = confidence_data(end-max_history+1:end);
        time_data = time_data(end-max_history+1:end);
        performance_data = performance_data(end-max_history+1:end, :);
    end
end

function update_all_plots(azimuth, elevation, distance, confidence, tdoa_matrix, audio_spectrum)
    % Update all visualization plots
    
    global position_data confidence_data time_data performance_data;
    
    if isempty(position_data)
        return;
    end
    
    % Convert time to relative seconds
    time_seconds = (time_data - time_data(1)) * 24 * 3600;
    
    % 3D position plot
    subplot(2,4,1);
    cla;
    if size(position_data, 1) > 1
        scatter3(position_data(:,1), position_data(:,2), position_data(:,3), ...
                50, confidence_data, 'filled');
        colorbar;
        xlabel('Azimuth (°)');
        ylabel('Elevation (°)');
        zlabel('Distance (cm)');
        title('3D Position Tracking');
        grid on;
        
        % Add current position marker
        hold on;
        scatter3(azimuth, elevation, distance, 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
        hold off;
    end
    
    % Azimuth vs time
    subplot(2,4,2);
    cla;
    plot(time_seconds, position_data(:,1), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Azimuth (°)');
    title('Azimuth Tracking');
    grid on;
    ylim([0, 360]);
    
    % Elevation vs time
    subplot(2,4,3);
    cla;
    plot(time_seconds, position_data(:,2), 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Elevation (°)');
    title('Elevation Tracking');
    grid on;
    ylim([-90, 90]);
    
    % Distance vs time
    subplot(2,4,4);
    cla;
    plot(time_seconds, position_data(:,3), 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Distance (cm)');
    title('Distance Tracking');
    grid on;
    ylim([0, 500]);
    
    % Confidence plot
    subplot(2,4,5);
    cla;
    plot(time_seconds, confidence_data, 'm-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Confidence');
    title('Confidence Level');
    grid on;
    ylim([0, 1]);
    
    % TDOA matrix
    subplot(2,4,6);
    cla;
    imagesc(tdoa_matrix);
    colorbar;
    xlabel('Microphone');
    ylabel('Microphone');
    title('TDOA Matrix (s)');
    colormap('jet');
    
    % Audio spectrum
    subplot(2,4,7);
    cla;
    plot(1:length(audio_spectrum), audio_spectrum, 'k-', 'LineWidth', 1);
    xlabel('Frequency Bin');
    ylabel('Magnitude');
    title('Audio Spectrum');
    grid on;
    
    % Performance metrics
    subplot(2,4,8);
    cla;
    if ~isempty(performance_data)
        plot(time_seconds(1:length(performance_data)), performance_data(:,1), 'c-', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Stability Score');
        title('System Performance');
        grid on;
        ylim([0, 1]);
    end
    
    % Update display
    drawnow;
end

function update_visualization(~, ~)
    % Timer callback for automatic updates
    % This function is called by the timer
    try
        [azimuth, elevation, distance, confidence, tdoa_matrix, audio_spectrum] = read_esp32_data();
        update_data_storage(azimuth, elevation, distance, confidence);
        update_all_plots(azimuth, elevation, distance, confidence, tdoa_matrix, audio_spectrum);
    catch ME
        fprintf('Error in visualization update: %s\n', ME.message);
    end
end

%% Additional Analysis Functions

function analyze_localization_performance()
    % Analyze the performance of the localization system
    
    global position_data confidence_data time_data performance_data;
    
    if isempty(position_data)
        fprintf('No data available for analysis\n');
        return;
    end
    
    fprintf('=== Localization Performance Analysis ===\n\n');
    
    % Position statistics
    fprintf('Position Statistics:\n');
    fprintf('  Azimuth: %.1f° ± %.1f° (range: %.1f° to %.1f°)\n', ...
            mean(position_data(:,1)), std(position_data(:,1)), ...
            min(position_data(:,1)), max(position_data(:,1)));
    fprintf('  Elevation: %.1f° ± %.1f° (range: %.1f° to %.1f°)\n', ...
            mean(position_data(:,2)), std(position_data(:,2)), ...
            min(position_data(:,2)), max(position_data(:,2)));
    fprintf('  Distance: %.1f cm ± %.1f cm (range: %.1f cm to %.1f cm)\n', ...
            mean(position_data(:,3)), std(position_data(:,3)), ...
            min(position_data(:,3)), max(position_data(:,3)));
    
    % Confidence statistics
    fprintf('\nConfidence Statistics:\n');
    fprintf('  Mean confidence: %.3f\n', mean(confidence_data));
    fprintf('  Min confidence: %.3f\n', min(confidence_data));
    fprintf('  Max confidence: %.3f\n', max(confidence_data));
    fprintf('  High confidence (>0.8): %.1f%%\n', ...
            100 * sum(confidence_data > 0.8) / length(confidence_data));
    
    % System stability
    if ~isempty(performance_data)
        fprintf('\nSystem Stability:\n');
        fprintf('  Mean stability score: %.3f\n', mean(performance_data(:,1)));
        fprintf('  Current stability: %.3f\n', performance_data(end,1));
    end
    
    % Generate performance report
    generate_performance_report();
end

function generate_performance_report()
    % Generate a comprehensive performance report
    
    global position_data confidence_data time_data performance_data;
    
    % Create report figure
    fig = figure('Name', 'Localization Performance Report', ...
                 'Position', [200, 200, 1200, 800]);
    
    % Position distribution
    subplot(2,3,1);
    histogram(position_data(:,1), 20);
    xlabel('Azimuth (°)');
    ylabel('Frequency');
    title('Azimuth Distribution');
    grid on;
    
    subplot(2,3,2);
    histogram(position_data(:,2), 20);
    xlabel('Elevation (°)');
    ylabel('Frequency');
    title('Elevation Distribution');
    grid on;
    
    subplot(2,3,3);
    histogram(position_data(:,3), 20);
    xlabel('Distance (cm)');
    ylabel('Frequency');
    title('Distance Distribution');
    grid on;
    
    % Confidence analysis
    subplot(2,3,4);
    histogram(confidence_data, 20);
    xlabel('Confidence');
    ylabel('Frequency');
    title('Confidence Distribution');
    grid on;
    
    % Time series analysis
    subplot(2,3,5);
    time_seconds = (time_data - time_data(1)) * 24 * 3600;
    plot(time_seconds, confidence_data, 'b-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Confidence');
    title('Confidence Over Time');
    grid on;
    
    % Performance metrics
    subplot(2,3,6);
    if ~isempty(performance_data)
        plot(time_seconds(1:length(performance_data)), performance_data(:,1), 'r-', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Stability Score');
        title('System Stability');
        grid on;
    end
    
    % Save report
    saveas(fig, 'localization_performance_report.png');
    fprintf('Performance report saved as localization_performance_report.png\n');
end

function export_data()
    % Export localization data for further analysis
    
    global position_data confidence_data time_data performance_data;
    
    if isempty(position_data)
        fprintf('No data to export\n');
        return;
    end
    
    % Create timestamp for unique filenames
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % Create data table
    time_seconds = (time_data - time_data(1)) * 24 * 3600;
    
    data_table = table(time_seconds, position_data(:,1), position_data(:,2), position_data(:,3), confidence_data, ...
                      'VariableNames', {'Time_s', 'Azimuth_deg', 'Elevation_deg', 'Distance_cm', 'Confidence'});
    
    % Export to CSV with timestamp
    filename = sprintf('localization_data_%s.csv', timestamp);
    writetable(data_table, filename);
    fprintf('Data exported to %s\n', filename);
    
    % Export performance data if available
    if ~isempty(performance_data)
        perf_table = table(time_seconds(1:length(performance_data)), performance_data(:,1), performance_data(:,2), ...
                          'VariableNames', {'Time_s', 'Stability_Score', 'Confidence_Trend'});
        perf_filename = sprintf('performance_data_%s.csv', timestamp);
        writetable(perf_table, perf_filename);
        fprintf('Performance data exported to %s\n', perf_filename);
    end
    
    % Create summary report
    create_summary_report(timestamp);
end

function create_summary_report(timestamp)
    % Create a summary report of the localization session
    
    global position_data confidence_data time_data performance_data;
    
    if isempty(position_data)
        return;
    end
    
    % Calculate statistics
    time_seconds = (time_data - time_data(1)) * 24 * 3600;
    session_duration = time_seconds(end);
    num_measurements = length(position_data);
    
    % Position statistics
    mean_azimuth = mean(position_data(:,1));
    std_azimuth = std(position_data(:,1));
    mean_elevation = mean(position_data(:,2));
    std_elevation = std(position_data(:,2));
    mean_distance = mean(position_data(:,3));
    std_distance = std(position_data(:,3));
    mean_confidence = mean(confidence_data);
    
    % Create report
    report_filename = sprintf('localization_report_%s.txt', timestamp);
    fid = fopen(report_filename, 'w');
    
    fprintf(fid, 'Sound Source Localization Session Report\n');
    fprintf(fid, '========================================\n\n');
    fprintf(fid, 'Session Date: %s\n', datestr(now));
    fprintf(fid, 'Session Duration: %.1f seconds\n', session_duration);
    fprintf(fid, 'Total Measurements: %d\n\n', num_measurements);
    
    fprintf(fid, 'Position Statistics:\n');
    fprintf(fid, '  Azimuth: %.1f° ± %.1f° (range: %.1f° to %.1f°)\n', ...
            mean_azimuth, std_azimuth, min(position_data(:,1)), max(position_data(:,1)));
    fprintf(fid, '  Elevation: %.1f° ± %.1f° (range: %.1f° to %.1f°)\n', ...
            mean_elevation, std_elevation, min(position_data(:,2)), max(position_data(:,2)));
    fprintf(fid, '  Distance: %.1f cm ± %.1f cm (range: %.1f cm to %.1f cm)\n', ...
            mean_distance, std_distance, min(position_data(:,3)), max(position_data(:,3)));
    
    fprintf(fid, '\nConfidence Statistics:\n');
    fprintf(fid, '  Mean Confidence: %.3f\n', mean_confidence);
    fprintf(fid, '  Min Confidence: %.3f\n', min(confidence_data));
    fprintf(fid, '  Max Confidence: %.3f\n', max(confidence_data));
    fprintf(fid, '  High Confidence (>0.8): %.1f%%\n', ...
            100 * sum(confidence_data > 0.8) / length(confidence_data));
    
    if ~isempty(performance_data)
        fprintf(fid, '\nSystem Performance:\n');
        fprintf(fid, '  Mean Stability Score: %.3f\n', mean(performance_data(:,1)));
        fprintf(fid, '  Current Stability: %.3f\n', performance_data(end,1));
    end
    
    fprintf(fid, '\nData Files Generated:\n');
    fprintf(fid, '  - localization_data_%s.csv\n', timestamp);
    if ~isempty(performance_data)
        fprintf(fid, '  - performance_data_%s.csv\n', timestamp);
    end
    fprintf(fid, '  - localization_report_%s.txt (this file)\n', timestamp);
    
    fclose(fid);
    fprintf('Summary report saved to %s\n', report_filename);
end
