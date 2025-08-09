%% Analysis Tools for Drone Simulation Results
% This script provides comprehensive analysis tools for post-simulation
% data analysis and visualization

fprintf('Loading Drone Simulation Analysis Tools\n');
fprintf('======================================\n');

%% Load required data if not available
if ~exist('sim_config', 'var')
    % Get the project root directory (go up one level from scripts)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    config_file = fullfile(project_root, 'data', 'simulation_config.mat');
    
    if exist(config_file, 'file')
        load(config_file);
        fprintf('Simulation configuration loaded from: %s\n', config_file);
    else
        fprintf('Warning: Simulation configuration not found at: %s\n', config_file);
        fprintf('Run simulation_setup.m first.\n');
    end
end

%% Analysis Functions

function analyze_flight_performance(sim_results)
    % Comprehensive flight performance analysis
    fprintf('Analyzing flight performance...\n');
    
    if isempty(sim_results) || ~isfield(sim_results, 'sim_output')
        fprintf('Error: Invalid simulation results.\n');
        return;
    end
    
    data = sim_results.sim_output;
    
    % Create analysis figure
    figure('Name', sprintf('Flight Analysis - %s', sim_results.scenario_name), ...
           'Position', [100, 100, 1200, 800]);
    
    % Position tracking
    subplot(3, 3, 1);
    if isfield(data, 'position')
        pos_data = data.position.Data;
        plot(data.tout, pos_data(:, 1), 'r-', 'LineWidth', 1.5); hold on;
        plot(data.tout, pos_data(:, 2), 'g-', 'LineWidth', 1.5);
        plot(data.tout, pos_data(:, 3), 'b-', 'LineWidth', 1.5);
        
        if isfield(data, 'position_ref')
            pos_ref = data.position_ref.Data;
            plot(data.tout, pos_ref(:, 1), 'r--', 'LineWidth', 1);
            plot(data.tout, pos_ref(:, 2), 'g--', 'LineWidth', 1);
            plot(data.tout, pos_ref(:, 3), 'b--', 'LineWidth', 1);
        end
        
        xlabel('Time (s)'); ylabel('Position (m)');
        title('Position Tracking');
        legend('X', 'Y', 'Z', 'X_{ref}', 'Y_{ref}', 'Z_{ref}', 'Location', 'best');
        grid on;
    end
    
    % Attitude tracking
    subplot(3, 3, 2);
    if isfield(data, 'attitude')
        att_data = rad2deg(data.attitude.Data);
        plot(data.tout, att_data(:, 1), 'r-', 'LineWidth', 1.5); hold on;
        plot(data.tout, att_data(:, 2), 'g-', 'LineWidth', 1.5);
        plot(data.tout, att_data(:, 3), 'b-', 'LineWidth', 1.5);
        
        if isfield(data, 'attitude_ref')
            att_ref = rad2deg(data.attitude_ref.Data);
            plot(data.tout, att_ref(:, 1), 'r--', 'LineWidth', 1);
            plot(data.tout, att_ref(:, 2), 'g--', 'LineWidth', 1);
            plot(data.tout, att_ref(:, 3), 'b--', 'LineWidth', 1);
        end
        
        xlabel('Time (s)'); ylabel('Attitude (deg)');
        title('Attitude Tracking');
        legend('Roll', 'Pitch', 'Yaw', 'Roll_{ref}', 'Pitch_{ref}', 'Yaw_{ref}', 'Location', 'best');
        grid on;
    end
    
    % 3D trajectory
    subplot(3, 3, 3);
    if isfield(data, 'position')
        pos_data = data.position.Data;
        plot3(pos_data(:, 1), pos_data(:, 2), pos_data(:, 3), 'b-', 'LineWidth', 2);
        hold on;
        
        % Mark start and end points
        plot3(pos_data(1, 1), pos_data(1, 2), pos_data(1, 3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot3(pos_data(end, 1), pos_data(end, 2), pos_data(end, 3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        if isfield(data, 'position_ref')
            pos_ref = data.position_ref.Data;
            plot3(pos_ref(:, 1), pos_ref(:, 2), pos_ref(:, 3), 'k--', 'LineWidth', 1);
        end
        
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('3D Trajectory');
        legend('Actual', 'Start', 'End', 'Reference', 'Location', 'best');
        grid on; axis equal;
    end
    
    % Motor commands
    subplot(3, 3, 4);
    if isfield(data, 'motor_commands')
        motor_data = data.motor_commands.Data;
        plot(data.tout, motor_data(:, 1), 'r-', 'LineWidth', 1.5); hold on;
        plot(data.tout, motor_data(:, 2), 'g-', 'LineWidth', 1.5);
        plot(data.tout, motor_data(:, 3), 'b-', 'LineWidth', 1.5);
        plot(data.tout, motor_data(:, 4), 'm-', 'LineWidth', 1.5);
        
        xlabel('Time (s)'); ylabel('Motor Command');
        title('Motor Commands');
        legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Location', 'best');
        grid on;
    end
    
    % Velocity tracking
    subplot(3, 3, 5);
    if isfield(data, 'velocity')
        vel_data = data.velocity.Data;
        plot(data.tout, vel_data(:, 1), 'r-', 'LineWidth', 1.5); hold on;
        plot(data.tout, vel_data(:, 2), 'g-', 'LineWidth', 1.5);
        plot(data.tout, vel_data(:, 3), 'b-', 'LineWidth', 1.5);
        
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        title('Velocity');
        legend('V_x', 'V_y', 'V_z', 'Location', 'best');
        grid on;
    end
    
    % Angular velocity
    subplot(3, 3, 6);
    if isfield(data, 'angular_velocity')
        omega_data = rad2deg(data.angular_velocity.Data);
        plot(data.tout, omega_data(:, 1), 'r-', 'LineWidth', 1.5); hold on;
        plot(data.tout, omega_data(:, 2), 'g-', 'LineWidth', 1.5);
        plot(data.tout, omega_data(:, 3), 'b-', 'LineWidth', 1.5);
        
        xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
        title('Angular Velocity');
        legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best');
        grid on;
    end
    
    % Position error
    subplot(3, 3, 7);
    if isfield(data, 'position') && isfield(data, 'position_ref')
        pos_error = data.position.Data - data.position_ref.Data;
        error_magnitude = sqrt(sum(pos_error.^2, 2));
        
        plot(data.tout, pos_error(:, 1), 'r-', 'LineWidth', 1); hold on;
        plot(data.tout, pos_error(:, 2), 'g-', 'LineWidth', 1);
        plot(data.tout, pos_error(:, 3), 'b-', 'LineWidth', 1);
        plot(data.tout, error_magnitude, 'k-', 'LineWidth', 2);
        
        xlabel('Time (s)'); ylabel('Position Error (m)');
        title('Position Tracking Error');
        legend('e_x', 'e_y', 'e_z', '|e|', 'Location', 'best');
        grid on;
    end
    
    % Battery state
    subplot(3, 3, 8);
    if isfield(data, 'battery_soc')
        soc_data = data.battery_soc.Data;
        plot(data.tout, soc_data, 'b-', 'LineWidth', 2);
        
        xlabel('Time (s)'); ylabel('State of Charge (%)');
        title('Battery State');
        grid on;
        ylim([0, 100]);
    end
    
    % Power consumption
    subplot(3, 3, 9);
    if isfield(data, 'power_consumption')
        power_data = data.power_consumption.Data;
        plot(data.tout, power_data, 'r-', 'LineWidth', 1.5);
        
        xlabel('Time (s)'); ylabel('Power (W)');
        title('Power Consumption');
        grid on;
    elseif isfield(data, 'motor_commands')
        % Estimate power from motor commands
        motor_data = data.motor_commands.Data;
        estimated_power = sum(motor_data.^2, 2) * 10; % Rough estimation
        plot(data.tout, estimated_power, 'r-', 'LineWidth', 1.5);
        
        xlabel('Time (s)'); ylabel('Estimated Power (W)');
        title('Estimated Power Consumption');
        grid on;
    end
    
    fprintf('Flight performance analysis completed.\n');
end

function metrics = calculate_detailed_metrics(sim_results)
    % Calculate comprehensive performance metrics
    fprintf('Calculating detailed performance metrics...\n');
    
    if isempty(sim_results) || ~isfield(sim_results, 'sim_output')
        fprintf('Error: Invalid simulation results.\n');
        metrics = [];
        return;
    end
    
    data = sim_results.sim_output;
    time = data.tout;
    dt = mean(diff(time));
    
    metrics = struct();
    metrics.scenario = sim_results.scenario_name;
    metrics.simulation_time = time(end);
    
    % Position tracking metrics
    if isfield(data, 'position') && isfield(data, 'position_ref')
        pos_actual = data.position.Data;
        pos_ref = data.position_ref.Data;
        pos_error = pos_actual - pos_ref;
        
        % RMS error
        metrics.position.rms_error = sqrt(mean(sum(pos_error.^2, 2)));
        metrics.position.rms_error_xyz = sqrt(mean(pos_error.^2, 1));
        
        % Maximum error
        error_magnitude = sqrt(sum(pos_error.^2, 2));
        metrics.position.max_error = max(error_magnitude);
        metrics.position.max_error_xyz = max(abs(pos_error), [], 1);
        
        % Settling time (within 5% of final value)
        final_error = error_magnitude(end);
        settling_threshold = 0.05 * (max(error_magnitude) - final_error) + final_error;
        settling_indices = find(error_magnitude <= settling_threshold);
        if ~isempty(settling_indices)
            metrics.position.settling_time = time(settling_indices(1));
        else
            metrics.position.settling_time = NaN;
        end
        
        % Steady-state error
        steady_state_window = round(0.1 / dt); % Last 10% of simulation
        if length(error_magnitude) > steady_state_window
            metrics.position.steady_state_error = mean(error_magnitude(end-steady_state_window+1:end));
        else
            metrics.position.steady_state_error = final_error;
        end
        
        % Overshoot
        if ~isempty(pos_ref) && length(unique(pos_ref(:, 3))) > 1 % Check for step input
            ref_final = pos_ref(end, 3);
            ref_initial = pos_ref(1, 3);
            step_size = abs(ref_final - ref_initial);
            
            if step_size > 0.01 % Minimum step size
                actual_z = pos_actual(:, 3);
                overshoot_value = max(actual_z) - ref_final;
                metrics.position.overshoot_percent = 100 * overshoot_value / step_size;
            else
                metrics.position.overshoot_percent = 0;
            end
        else
            metrics.position.overshoot_percent = 0;
        end
    end
    
    % Attitude tracking metrics
    if isfield(data, 'attitude') && isfield(data, 'attitude_ref')
        att_actual = data.attitude.Data;
        att_ref = data.attitude_ref.Data;
        att_error = att_actual - att_ref;
        
        metrics.attitude.rms_error = sqrt(mean(sum(att_error.^2, 2)));
        metrics.attitude.rms_error_rpy = sqrt(mean(att_error.^2, 1));
        metrics.attitude.max_error = max(sqrt(sum(att_error.^2, 2)));
        metrics.attitude.max_error_rpy = max(abs(att_error), [], 1);
    end
    
    % Control effort metrics
    if isfield(data, 'motor_commands')
        motor_cmds = data.motor_commands.Data;
        
        metrics.control.total_variation = sum(sum(abs(diff(motor_cmds, 1, 1))));
        metrics.control.max_command = max(motor_cmds(:));
        metrics.control.min_command = min(motor_cmds(:));
        metrics.control.mean_command = mean(motor_cmds(:));
        metrics.control.std_command = std(motor_cmds(:));
        
        % Control smoothness (rate of change)
        cmd_rate = diff(motor_cmds, 1, 1) / dt;
        metrics.control.max_rate = max(abs(cmd_rate(:)));
        metrics.control.mean_rate = mean(abs(cmd_rate(:)));
    end
    
    % Energy consumption
    if isfield(data, 'battery_soc')
        soc = data.battery_soc.Data;
        metrics.energy.battery_consumed = soc(1) - soc(end);
        metrics.energy.consumption_rate = metrics.energy.battery_consumed / time(end);
        
        % Estimated flight time
        if metrics.energy.consumption_rate > 0
            metrics.energy.estimated_flight_time = soc(end) / metrics.energy.consumption_rate;
        else
            metrics.energy.estimated_flight_time = Inf;
        end
    end
    
    % Stability metrics
    if isfield(data, 'position')
        pos_data = data.position.Data;
        
        % Position variance (stability indicator)
        metrics.stability.position_variance = var(pos_data, 1, 1);
        
        % Velocity smoothness
        if size(pos_data, 1) > 1
            velocity = diff(pos_data, 1, 1) / dt;
            metrics.stability.velocity_variance = var(velocity, 1, 1);
        end
    end
    
    % Frequency domain analysis
    if isfield(data, 'position') && length(time) > 100
        try
            pos_data = data.position.Data;
            fs = 1 / dt;
            
            for axis = 1:3
                [psd, freq] = pwelch(pos_data(:, axis), [], [], [], fs);
                
                % Dominant frequency
                [~, max_idx] = max(psd);
                metrics.frequency.dominant_freq(axis) = freq(max_idx);
                
                % Bandwidth (frequency where power drops to -3dB)
                max_power = max(psd);
                bw_threshold = max_power / 2; % -3dB point
                bw_indices = find(psd >= bw_threshold);
                if ~isempty(bw_indices)
                    metrics.frequency.bandwidth(axis) = freq(bw_indices(end)) - freq(bw_indices(1));
                else
                    metrics.frequency.bandwidth(axis) = NaN;
                end
            end
        catch
            fprintf('Warning: Frequency analysis failed.\n');
        end
    end
    
    fprintf('Detailed metrics calculation completed.\n');
end

function compare_scenarios(results_array)
    % Compare performance across multiple scenarios
    fprintf('Comparing scenario performance...\n');
    
    if isempty(results_array)
        fprintf('Error: No results to compare.\n');
        return;
    end
    
    % Extract metrics from all scenarios
    scenario_names = {};
    rms_errors = [];
    max_errors = [];
    settling_times = [];
    control_efforts = [];
    
    for i = 1:length(results_array)
        if ~isempty(results_array{i}) && isfield(results_array{i}, 'metrics')
            scenario_names{end+1} = results_array{i}.scenario_name;
            
            metrics = results_array{i}.metrics;
            
            if isfield(metrics, 'position')
                rms_errors(end+1) = metrics.position.rms_error;
                max_errors(end+1) = metrics.position.max_error;
                settling_times(end+1) = metrics.position.settling_time;
            else
                rms_errors(end+1) = NaN;
                max_errors(end+1) = NaN;
                settling_times(end+1) = NaN;
            end
            
            if isfield(metrics, 'control')
                control_efforts(end+1) = metrics.control.total_variation;
            else
                control_efforts(end+1) = NaN;
            end
        end
    end
    
    % Create comparison plots
    figure('Name', 'Scenario Comparison', 'Position', [200, 200, 1000, 600]);
    
    subplot(2, 2, 1);
    bar(rms_errors);
    set(gca, 'XTickLabel', scenario_names);
    xlabel('Scenario'); ylabel('RMS Position Error (m)');
    title('Position Tracking Accuracy');
    grid on; xtickangle(45);
    
    subplot(2, 2, 2);
    bar(max_errors);
    set(gca, 'XTickLabel', scenario_names);
    xlabel('Scenario'); ylabel('Max Position Error (m)');
    title('Maximum Position Error');
    grid on; xtickangle(45);
    
    subplot(2, 2, 3);
    bar(settling_times);
    set(gca, 'XTickLabel', scenario_names);
    xlabel('Scenario'); ylabel('Settling Time (s)');
    title('Response Time');
    grid on; xtickangle(45);
    
    subplot(2, 2, 4);
    bar(control_efforts);
    set(gca, 'XTickLabel', scenario_names);
    xlabel('Scenario'); ylabel('Total Control Variation');
    title('Control Effort');
    grid on; xtickangle(45);
    
    % Summary table
    fprintf('\nScenario Comparison Summary:\n');
    fprintf('%-20s %-12s %-12s %-12s %-12s\n', 'Scenario', 'RMS Error', 'Max Error', 'Settling T', 'Control Eff');
    fprintf('%s\n', repmat('-', 1, 80));
    
    for i = 1:length(scenario_names)
        fprintf('%-20s %-12.4f %-12.4f %-12.4f %-12.2f\n', ...
            scenario_names{i}, rms_errors(i), max_errors(i), settling_times(i), control_efforts(i));
    end
end

function export_results(sim_results, filename)
    % Export simulation results to various formats
    fprintf('Exporting results to %s...\n', filename);
    
    try
        [~, ~, ext] = fileparts(filename);
        
        switch lower(ext)
            case '.mat'
                save(filename, 'sim_results');
                
            case '.csv'
                % Export time series data to CSV
                if isfield(sim_results, 'sim_output')
                    data = sim_results.sim_output;
                    
                    % Combine all time series data
                    export_data = [data.tout];
                    headers = {'Time'};
                    
                    if isfield(data, 'position')
                        export_data = [export_data, data.position.Data];
                        headers = [headers, {'Pos_X', 'Pos_Y', 'Pos_Z'}];
                    end
                    
                    if isfield(data, 'attitude')
                        export_data = [export_data, rad2deg(data.attitude.Data)];
                        headers = [headers, {'Roll', 'Pitch', 'Yaw'}];
                    end
                    
                    if isfield(data, 'motor_commands')
                        export_data = [export_data, data.motor_commands.Data];
                        headers = [headers, {'Motor1', 'Motor2', 'Motor3', 'Motor4'}];
                    end
                    
                    % Write to CSV
                    writematrix([headers; num2cell(export_data)], filename);
                end
                
            case '.xlsx'
                % Export to Excel with multiple sheets
                if isfield(sim_results, 'sim_output')
                    data = sim_results.sim_output;
                    
                    % Position data
                    if isfield(data, 'position')
                        pos_table = table(data.tout, data.position.Data(:,1), data.position.Data(:,2), data.position.Data(:,3), ...
                            'VariableNames', {'Time', 'X', 'Y', 'Z'});
                        writetable(pos_table, filename, 'Sheet', 'Position');
                    end
                    
                    % Attitude data
                    if isfield(data, 'attitude')
                        att_table = table(data.tout, rad2deg(data.attitude.Data(:,1)), rad2deg(data.attitude.Data(:,2)), rad2deg(data.attitude.Data(:,3)), ...
                            'VariableNames', {'Time', 'Roll', 'Pitch', 'Yaw'});
                        writetable(att_table, filename, 'Sheet', 'Attitude');
                    end
                    
                    % Metrics summary
                    if isfield(sim_results, 'metrics')
                        metrics_struct = sim_results.metrics;
                        % Convert struct to table (simplified)
                        summary_data = {'Scenario', sim_results.scenario_name; ...
                                       'Simulation Time', metrics_struct.simulation_time};
                        
                        if isfield(metrics_struct, 'position')
                            summary_data = [summary_data; {'RMS Position Error', metrics_struct.position.rms_error}];
                            summary_data = [summary_data; {'Max Position Error', metrics_struct.position.max_error}];
                        end
                        
                        summary_table = table(summary_data(:,1), summary_data(:,2), ...
                            'VariableNames', {'Metric', 'Value'});
                        writetable(summary_table, filename, 'Sheet', 'Summary');
                    end
                end
                
            otherwise
                fprintf('Unsupported file format: %s\n', ext);
                return;
        end
        
        fprintf('Results exported successfully to %s\n', filename);
        
    catch ME
        fprintf('Export failed: %s\n', ME.message);
    end
end

function generate_report(sim_results, report_title)
    % Generate comprehensive analysis report
    fprintf('Generating analysis report: %s\n', report_title);
    
    % Create report figure
    fig = figure('Name', report_title, 'Position', [50, 50, 1400, 900]);
    
    % Calculate metrics
    metrics = calculate_detailed_metrics(sim_results);
    
    % Create main analysis plots
    analyze_flight_performance(sim_results);
    
    % Add text summary
    subplot(3, 3, 9);
    axis off;
    
    summary_text = sprintf('PERFORMANCE SUMMARY\n\n');
    summary_text = [summary_text, sprintf('Scenario: %s\n', sim_results.scenario_name)];
    summary_text = [summary_text, sprintf('Duration: %.1f s\n\n', metrics.simulation_time)];
    
    if isfield(metrics, 'position')
        summary_text = [summary_text, sprintf('POSITION TRACKING:\n')];
        summary_text = [summary_text, sprintf('RMS Error: %.4f m\n', metrics.position.rms_error)];
        summary_text = [summary_text, sprintf('Max Error: %.4f m\n', metrics.position.max_error)];
        summary_text = [summary_text, sprintf('Settling Time: %.2f s\n\n', metrics.position.settling_time)];
    end
    
    if isfield(metrics, 'control')
        summary_text = [summary_text, sprintf('CONTROL PERFORMANCE:\n')];
        summary_text = [summary_text, sprintf('Max Command: %.3f\n', metrics.control.max_command)];
        summary_text = [summary_text, sprintf('Control Variation: %.2f\n\n', metrics.control.total_variation)];
    end
    
    if isfield(metrics, 'energy')
        summary_text = [summary_text, sprintf('ENERGY CONSUMPTION:\n')];
        summary_text = [summary_text, sprintf('Battery Used: %.1f%%\n', metrics.energy.battery_consumed)];
        summary_text = [summary_text, sprintf('Est. Flight Time: %.1f min\n', metrics.energy.estimated_flight_time/60)];
    end
    
    text(0.05, 0.95, summary_text, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'FontName', 'FixedWidth', 'FontSize', 10);
    
    fprintf('Report generation completed.\n');
end

%% Main Analysis Interface
function analysis_interface()
    % Interactive analysis interface
    fprintf('\nDrone Simulation Analysis Tools\n');
    fprintf('===============================\n');
    fprintf('1. Analyze flight performance\n');
    fprintf('2. Calculate detailed metrics\n');
    fprintf('3. Compare multiple scenarios\n');
    fprintf('4. Export results\n');
    fprintf('5. Generate comprehensive report\n');
    fprintf('6. Load and analyze existing results\n');
    fprintf('0. Exit\n\n');
    
    choice = input('Select analysis option: ');
    
    switch choice
        case 1
            results_var = input('Enter results variable name: ', 's');
            if evalin('base', sprintf('exist(''%s'', ''var'')', results_var))
                results = evalin('base', results_var);
                analyze_flight_performance(results);
            else
                fprintf('Variable %s not found in workspace.\n', results_var);
            end
            
        case 2
            results_var = input('Enter results variable name: ', 's');
            if evalin('base', sprintf('exist(''%s'', ''var'')', results_var))
                results = evalin('base', results_var);
                metrics = calculate_detailed_metrics(results);
                assignin('base', 'detailed_metrics', metrics);
                fprintf('Detailed metrics saved to workspace as ''detailed_metrics''.\n');
            else
                fprintf('Variable %s not found in workspace.\n', results_var);
            end
            
        case 3
            results_array_var = input('Enter results array variable name: ', 's');
            if evalin('base', sprintf('exist(''%s'', ''var'')', results_array_var))
                results_array = evalin('base', results_array_var);
                compare_scenarios(results_array);
            else
                fprintf('Variable %s not found in workspace.\n', results_array_var);
            end
            
        case 4
            results_var = input('Enter results variable name: ', 's');
            filename = input('Enter output filename: ', 's');
            if evalin('base', sprintf('exist(''%s'', ''var'')', results_var))
                results = evalin('base', results_var);
                export_results(results, filename);
            else
                fprintf('Variable %s not found in workspace.\n', results_var);
            end
            
        case 5
            results_var = input('Enter results variable name: ', 's');
            report_title = input('Enter report title: ', 's');
            if evalin('base', sprintf('exist(''%s'', ''var'')', results_var))
                results = evalin('base', results_var);
                generate_report(results, report_title);
            else
                fprintf('Variable %s not found in workspace.\n', results_var);
            end
            
        case 6
            filename = input('Enter results file path: ', 's');
            if exist(filename, 'file')
                loaded_data = load(filename);
                assignin('base', 'loaded_results', loaded_data);
                fprintf('Results loaded as ''loaded_results'' in workspace.\n');
            else
                fprintf('File %s not found.\n', filename);
            end
            
        case 0
            fprintf('Exiting analysis interface.\n');
            return;
            
        otherwise
            fprintf('Invalid option. Try again.\n');
            analysis_interface();
    end
end

%% Run interface if called directly
if ~exist('skip_interface', 'var')
    analysis_interface();
end

fprintf('Analysis tools loaded successfully!\n');
