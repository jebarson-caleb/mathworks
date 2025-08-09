%% Simulation Setup and Configuration Script
% This script configures simulation parameters and environments
% for different test scenarios

fprintf('Simulation Setup and Configuration\n');
fprintf('=================================\n');

%% Load parameters if not already loaded
if ~exist('drone', 'var')
    fprintf('Loading drone parameters...\n');
    run('initialize_drone.m');
end

%% Simulation Environment Setup

% Create simulation configuration structure
sim_config = struct();

%% Scenario 1: Hovering Test
sim_config.hover.name = 'Hovering Stability Test';
sim_config.hover.duration = 20;  % seconds
sim_config.hover.sample_time = 0.001;
sim_config.hover.reference.position = [0; 0; -2];  % 2m altitude
sim_config.hover.reference.attitude = [0; 0; 0];   % Level attitude
sim_config.hover.disturbances.wind = [0.5; 0.3; 0.1];  % Light wind
sim_config.hover.disturbances.sensor_noise = true;
sim_config.hover.initial_conditions = init;

%% Scenario 2: Trajectory Following
sim_config.trajectory.name = 'Trajectory Following Test';
sim_config.trajectory.duration = 60;
sim_config.trajectory.sample_time = 0.001;

% Define waypoints for figure-8 trajectory
t_traj = linspace(0, 60, 1000);
sim_config.trajectory.reference.position = [
    3 * sin(0.1 * t_traj);                    % X trajectory
    2 * sin(0.2 * t_traj);                    % Y trajectory
    -2 - 0.5 * sin(0.05 * t_traj)            % Z trajectory (varying altitude)
];
sim_config.trajectory.reference.yaw = 0.1 * t_traj;  % Slow rotation

sim_config.trajectory.disturbances.wind = [1.0; 0.5; 0.2];
sim_config.trajectory.disturbances.turbulence = true;
sim_config.trajectory.initial_conditions = init;

%% Scenario 3: Disturbance Rejection
sim_config.disturbance.name = 'Wind Disturbance Rejection';
sim_config.disturbance.duration = 30;
sim_config.disturbance.sample_time = 0.001;
sim_config.disturbance.reference.position = [0; 0; -3];
sim_config.disturbance.reference.attitude = [0; 0; 0];

% Strong wind gusts
sim_config.disturbance.disturbances.wind = [3.0; 2.0; 1.0];
sim_config.disturbance.disturbances.gusts.enable = true;
sim_config.disturbance.disturbances.gusts.amplitude = [4; 3; 2];
sim_config.disturbance.disturbances.gusts.frequency = [0.2; 0.15; 0.1];
sim_config.disturbance.initial_conditions = init;

%% Scenario 4: Payload Drop Mission
sim_config.payload.name = 'Payload Drop Mission';
sim_config.payload.duration = 45;
sim_config.payload.sample_time = 0.001;

% Mission waypoints
waypoints = [
    0,  0, -1;   % Takeoff
    5,  0, -3;   % Navigate to drop zone
    5,  0, -1;   % Descend for drop
    5,  0, -3;   % Ascend after drop
    0,  0, -1    % Return home
];
sim_config.payload.waypoints = waypoints;
sim_config.payload.payload_mass = 0.02;  % 20g payload
sim_config.payload.drop_time = 25;       % Drop at 25 seconds
sim_config.payload.initial_conditions = init;

%% Scenario 5: Motor Failure Test
sim_config.failure.name = 'Motor Failure Recovery';
sim_config.failure.duration = 25;
sim_config.failure.sample_time = 0.001;
sim_config.failure.reference.position = [0; 0; -2];
sim_config.failure.reference.attitude = [0; 0; 0];
sim_config.failure.motor_failure.motor_id = 2;      % Motor 2 fails
sim_config.failure.motor_failure.failure_time = 10; % Failure at 10s
sim_config.failure.motor_failure.severity = 0.3;    % 30% thrust loss
sim_config.failure.initial_conditions = init;

%% Scenario 6: Battery Depletion
sim_config.battery.name = 'Battery Depletion Test';
sim_config.battery.duration = 300;  % 5 minutes
sim_config.battery.sample_time = 0.01;  % Longer sample time for efficiency
sim_config.battery.reference.position = [0; 0; -2];
sim_config.battery.initial_soc = 100;  % Start with full battery
sim_config.battery.low_battery_threshold = 20;  % 20% threshold
sim_config.battery.auto_land_enabled = true;
sim_config.battery.initial_conditions = init;

%% Monte Carlo Simulation Setup
sim_config.monte_carlo.name = 'Monte Carlo Robustness Analysis';
sim_config.monte_carlo.num_runs = 100;
sim_config.monte_carlo.duration = 30;
sim_config.monte_carlo.sample_time = 0.01;

% Parameter variations (% of nominal)
sim_config.monte_carlo.variations.mass = 0.1;           % ±10% mass variation
sim_config.monte_carlo.variations.inertia = 0.15;       % ±15% inertia variation
sim_config.monte_carlo.variations.thrust_coeff = 0.05;  % ±5% thrust variation
sim_config.monte_carlo.variations.sensor_noise = 2.0;   % 2x sensor noise variation

%% Hardware-in-the-Loop Setup
sim_config.hil.name = 'Hardware-in-the-Loop Test';
sim_config.hil.target = 'Speedgoat';  % or 'Arduino', 'Raspberry Pi'
sim_config.hil.sample_time = 0.001;
sim_config.hil.real_time_factor = 1.0;  % Real-time execution
sim_config.hil.communication.protocol = 'UDP';
sim_config.hil.communication.port = 14550;  % MAVLink port
sim_config.hil.communication.rate = 1000;   % 1kHz update rate

%% Data Logging Configuration
logging_config = struct();
logging_config.enable = true;
logging_config.signals = {
    'Position', 'Velocity', 'Attitude', 'Angular_Velocity', ...
    'Motor_Commands', 'Sensor_Data', 'Controller_Output', ...
    'Battery_State', 'Wind_Disturbance'
};
logging_config.format = 'Dataset';  % or 'Array', 'Structure'
logging_config.decimation = 1;      % Log every sample
logging_config.limit_data_points = false;

%% Visualization Setup
visualization = struct();
visualization.enable_3d_animation = true;
visualization.enable_realtime_plots = true;
visualization.update_rate = 30;  % 30 FPS for animation
visualization.trail_length = 100;  % Number of position points in trail
visualization.view_angle = [-45, 30];  % Default 3D view

%% Performance Metrics Setup
metrics = struct();
metrics.position_error_threshold = 0.1;     % 10cm position accuracy
metrics.attitude_error_threshold = 0.087;   % 5 degrees attitude accuracy
metrics.settling_time_threshold = 2.0;      % 2 second settling time
metrics.overshoot_threshold = 0.1;          % 10% overshoot limit
metrics.steady_state_error_threshold = 0.05; % 5cm steady-state error

%% Functions for Simulation Management

function results = run_simulation_scenario(scenario_name)
    % Run a specific simulation scenario
    fprintf('Running simulation scenario: %s\n', scenario_name);
    
    if ~isfield(sim_config, scenario_name)
        error('Scenario %s not found!', scenario_name);
    end
    
    scenario = sim_config.(scenario_name);
    
    % Configure Simulink model
    model_name = 'main_drone_model';
    
    try
        % Load model if not already loaded
        if ~bdIsLoaded(model_name)
            load_system(model_name);
        end
        
        % Set simulation parameters
        set_param(model_name, 'StopTime', num2str(scenario.duration));
        set_param(model_name, 'FixedStep', num2str(scenario.sample_time));
        set_param(model_name, 'Solver', 'FixedStepDiscrete');
        
        % Configure logging
        if logging_config.enable
            set_param(model_name, 'SignalLogging', 'on');
            set_param(model_name, 'SignalLoggingName', 'logsout');
        end
        
        % Run simulation
        fprintf('Starting simulation...\n');
        tic;
        sim_output = sim(model_name);
        sim_time = toc;
        
        fprintf('Simulation completed in %.2f seconds\n', sim_time);
        
        % Extract results
        results.scenario_name = scenario_name;
        results.simulation_time = sim_time;
        results.sim_output = sim_output;
        
        if exist('logsout', 'var')
            results.logged_data = logsout;
        end
        
        % Calculate performance metrics
        results.metrics = calculate_performance_metrics(sim_output);
        
    catch ME
        fprintf('Simulation failed: %s\n', ME.message);
        results = [];
    end
end

function metrics_results = calculate_performance_metrics(sim_output)
    % Calculate performance metrics from simulation results
    metrics_results = struct();
    
    try
        % Extract time series data
        time = sim_output.tout;
        
        % Position tracking error (if reference exists)
        if isfield(sim_output, 'position') && isfield(sim_output, 'position_ref')
            pos_error = sim_output.position.Data - sim_output.position_ref.Data;
            metrics_results.rms_position_error = sqrt(mean(sum(pos_error.^2, 2)));
            metrics_results.max_position_error = max(sqrt(sum(pos_error.^2, 2)));
        end
        
        % Attitude tracking error
        if isfield(sim_output, 'attitude') && isfield(sim_output, 'attitude_ref')
            att_error = sim_output.attitude.Data - sim_output.attitude_ref.Data;
            metrics_results.rms_attitude_error = sqrt(mean(sum(att_error.^2, 2)));
            metrics_results.max_attitude_error = max(sqrt(sum(att_error.^2, 2)));
        end
        
        % Control effort
        if isfield(sim_output, 'motor_commands')
            motor_commands = sim_output.motor_commands.Data;
            metrics_results.total_control_effort = sum(sum(abs(motor_commands)));
            metrics_results.max_motor_command = max(motor_commands(:));
        end
        
        % Battery consumption
        if isfield(sim_output, 'battery_soc')
            soc = sim_output.battery_soc.Data;
            metrics_results.battery_consumption = soc(1) - soc(end);
            metrics_results.flight_time = time(end);
        end
        
        fprintf('Performance metrics calculated successfully.\n');
        
    catch ME
        fprintf('Metrics calculation failed: %s\n', ME.message);
        metrics_results = struct();
    end
end

function run_monte_carlo_analysis()
    % Run Monte Carlo analysis for robustness testing
    fprintf('Starting Monte Carlo analysis...\n');
    
    mc_config = sim_config.monte_carlo;
    results_array = cell(mc_config.num_runs, 1);
    
    % Save nominal parameters
    nominal_params = struct();
    nominal_params.mass = drone.mass;
    nominal_params.inertia = drone.I;
    nominal_params.thrust_coeff = motor.thrust_coeff;
    
    for run = 1:mc_config.num_runs
        fprintf('Monte Carlo run %d/%d\n', run, mc_config.num_runs);
        
        % Generate random parameter variations
        mass_var = 1 + mc_config.variations.mass * (2*rand - 1);
        inertia_var = 1 + mc_config.variations.inertia * (2*rand - 1);
        thrust_var = 1 + mc_config.variations.thrust_coeff * (2*rand - 1);
        
        % Apply variations
        drone.mass = nominal_params.mass * mass_var;
        drone.I = nominal_params.inertia * inertia_var;
        motor.thrust_coeff = nominal_params.thrust_coeff * thrust_var;
        
        % Run simulation
        try
            results_array{run} = run_simulation_scenario('hover');
        catch ME
            fprintf('Run %d failed: %s\n', run, ME.message);
            results_array{run} = [];
        end
    end
    
    % Restore nominal parameters
    drone.mass = nominal_params.mass;
    drone.I = nominal_params.inertia;
    motor.thrust_coeff = nominal_params.thrust_coeff;
    
    % Analyze results
    analyze_monte_carlo_results(results_array);
    
    fprintf('Monte Carlo analysis completed.\n');
end

function analyze_monte_carlo_results(results_array)
    % Analyze Monte Carlo simulation results
    valid_results = results_array(~cellfun(@isempty, results_array));
    num_valid = length(valid_results);
    
    if num_valid == 0
        fprintf('No valid Monte Carlo results to analyze.\n');
        return;
    end
    
    fprintf('Analyzing %d valid Monte Carlo results...\n', num_valid);
    
    % Extract metrics
    rms_errors = zeros(num_valid, 1);
    max_errors = zeros(num_valid, 1);
    
    for i = 1:num_valid
        if isfield(valid_results{i}.metrics, 'rms_position_error')
            rms_errors(i) = valid_results{i}.metrics.rms_position_error;
        end
        if isfield(valid_results{i}.metrics, 'max_position_error')
            max_errors(i) = valid_results{i}.metrics.max_position_error;
        end
    end
    
    % Statistical analysis
    fprintf('Position Tracking Performance:\n');
    fprintf('RMS Error - Mean: %.4f, Std: %.4f\n', mean(rms_errors), std(rms_errors));
    fprintf('Max Error - Mean: %.4f, Std: %.4f\n', mean(max_errors), std(max_errors));
    
    % Create histograms
    figure('Name', 'Monte Carlo Analysis Results');
    
    subplot(2, 1, 1);
    histogram(rms_errors, 20);
    xlabel('RMS Position Error (m)');
    ylabel('Frequency');
    title('Distribution of RMS Position Error');
    grid on;
    
    subplot(2, 1, 2);
    histogram(max_errors, 20);
    xlabel('Max Position Error (m)');
    ylabel('Frequency');
    title('Distribution of Maximum Position Error');
    grid on;
end

%% Save all simulation configurations
fprintf('Saving simulation configurations...\n');
save('data/simulation_config.mat', 'sim_config', 'logging_config', ...
     'visualization', 'metrics');

fprintf('Simulation setup completed successfully!\n');
fprintf('Available scenarios:\n');
scenario_names = fieldnames(sim_config);
for i = 1:length(scenario_names)
    if isstruct(sim_config.(scenario_names{i})) && isfield(sim_config.(scenario_names{i}), 'name')
        fprintf('  %s: %s\n', scenario_names{i}, sim_config.(scenario_names{i}).name);
    end
end

fprintf('\nTo run a scenario, use: run_simulation_scenario(''scenario_name'')\n');
