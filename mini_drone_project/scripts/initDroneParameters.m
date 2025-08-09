function initDroneParameters()
%INITDRONEPARAMETERS Initialize all drone parameters and create data directory
%   This function creates the data directory if it doesn't exist and
%   initializes all drone parameters for the Mini Drone Simulink project

fprintf('Initializing drone parameters...\n');

% Get current directory (should be project root)
project_root = pwd;
data_dir = fullfile(project_root, 'data');

% Ensure data directory exists
if ~exist(data_dir, 'dir')
    try
        mkdir(data_dir);
        fprintf('✓ Created data directory: %s\n', data_dir);
    catch ME
        fprintf('✗ Failed to create data directory: %s\n', ME.message);
        error('Cannot create data directory. Check permissions.');
    end
else
    fprintf('✓ Data directory exists: %s\n', data_dir);
end

%% Physical Properties
% Mass properties
drone.mass = 0.087;  % Total mass in kg (typical for mini drone)
drone.arm_length = 0.046;  % Distance from center to motor in meters

% Moments of inertia (kg⋅m²) - estimated for small quadcopter
drone.Ixx = 1.4e-5;  % Roll inertia
drone.Iyy = 1.4e-5;  % Pitch inertia  
drone.Izz = 2.17e-5; % Yaw inertia

% Cross products of inertia (assumed negligible for symmetric design)
drone.Ixy = 0;
drone.Ixz = 0;
drone.Iyz = 0;

% Inertia matrix
drone.I = [drone.Ixx, drone.Ixy, drone.Ixz;
           drone.Ixy, drone.Iyy, drone.Iyz;
           drone.Ixz, drone.Iyz, drone.Izz];

%% Motor and Propeller Properties
% Motor specifications (typical coreless motor)
motor.max_thrust = 0.3;  % Maximum thrust per motor in Newtons
motor.thrust_coeff = 1.69e-6;  % Thrust coefficient (N⋅s²/rad²)
motor.drag_coeff = 2.9e-8;   % Drag coefficient (N⋅m⋅s²/rad²)
motor.time_constant = 0.02;   % Motor time constant in seconds
motor.max_rpm = 31000;        % Maximum RPM

% Propeller specifications
prop.diameter = 0.046;  % Propeller diameter in meters
prop.pitch = 0.024;     % Propeller pitch in meters
prop.num_blades = 2;    % Number of blades

%% Environmental Parameters
env.gravity = 9.81;     % Gravitational acceleration (m/s²)
env.air_density = 1.225; % Air density at sea level (kg/m³)
env.temperature = 20;    % Temperature in Celsius
env.pressure = 101325;   % Atmospheric pressure in Pa

% Wind model parameters
env.wind.enable = true;
env.wind.constant = [0; 0; 0];  % Constant wind velocity [m/s]
env.wind.gust_amplitude = 2;     % Wind gust amplitude [m/s]
env.wind.gust_frequency = 0.1;   % Wind gust frequency [Hz]

%% Sensor Parameters
% IMU specifications
sensors.imu.accelerometer_noise = 0.01;   % m/s² std
sensors.imu.gyroscope_noise = 0.001;      % rad/s std
sensors.imu.magnetometer_noise = 0.1;     % µT std
sensors.imu.sample_rate = 1000;           % Hz

% GPS specifications  
sensors.gps.position_noise = 0.5;         % m std
sensors.gps.velocity_noise = 0.1;         % m/s std
sensors.gps.sample_rate = 10;             % Hz

% Barometer specifications
sensors.barometer.altitude_noise = 0.1;   % m std
sensors.barometer.sample_rate = 50;       % Hz

%% Battery Parameters
battery.voltage_nominal = 3.7;   % Nominal voltage (V)
battery.voltage_max = 4.2;       % Maximum voltage (V)
battery.voltage_min = 3.0;       % Minimum voltage (V)
battery.capacity = 1100;         % Capacity (mAh)
battery.internal_resistance = 0.08; % Internal resistance (Ohm)

%% Flight Mode Parameters
flight_modes.manual.max_angle = deg2rad(30);     % Max tilt angle (rad)
flight_modes.altitude_hold.enabled = true;
flight_modes.position_hold.enabled = true;
flight_modes.return_to_home.enabled = true;

%% Control Parameters
% PID Controller gains (outer loop - position)
control.position.kp = [0.8, 0.8, 1.2];  % [x, y, z] proportional gains
control.position.ki = [0.1, 0.1, 0.4];  % [x, y, z] integral gains
control.position.kd = [0.3, 0.3, 0.6];  % [x, y, z] derivative gains

% PID Controller gains (inner loop - attitude)
control.attitude.kp = [6.0, 6.0, 4.0];  % [roll, pitch, yaw] proportional gains
control.attitude.ki = [0.5, 0.5, 0.2];  % [roll, pitch, yaw] integral gains
control.attitude.kd = [0.8, 0.8, 0.4];  % [roll, pitch, yaw] derivative gains

% Control limits
control.limits.max_tilt_angle = deg2rad(30);     % Maximum tilt angle (rad)
control.limits.max_yaw_rate = deg2rad(180);      % Maximum yaw rate (rad/s)
control.limits.max_vertical_speed = 3.0;         % Maximum vertical speed (m/s)

%% PID Controller Parameters (detailed)
% Position PID
pid.position_x = struct('kp', 0.8, 'ki', 0.1, 'kd', 0.3, 'max_output', deg2rad(15));
pid.position_y = struct('kp', 0.8, 'ki', 0.1, 'kd', 0.3, 'max_output', deg2rad(15));
pid.position_z = struct('kp', 1.2, 'ki', 0.4, 'kd', 0.6, 'max_output', 2.0);

% Attitude PID
pid.roll = struct('kp', 6.0, 'ki', 0.5, 'kd', 0.8, 'max_output', 1.0);
pid.pitch = struct('kp', 6.0, 'ki', 0.5, 'kd', 0.8, 'max_output', 1.0);
pid.yaw = struct('kp', 4.0, 'ki', 0.2, 'kd', 0.4, 'max_output', 1.0);

%% LQR Controller Parameters
lqr.Q = diag([10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1]); % State weighting matrix
lqr.R = diag([1, 1, 1, 1]);                              % Control weighting matrix
lqr.enabled = true;

%% MPC Controller Parameters
mpc.prediction_horizon = 10;     % Prediction horizon
mpc.control_horizon = 3;         % Control horizon
mpc.sample_time = 0.1;          % Sample time (s)
mpc.enabled = false;             % Disabled by default

%% Simulation Parameters
sim.simulation_time = 30;        % Total simulation time (s)
sim.sample_time = 0.01;         % Simulation sample time (s)
sim.solver = 'ode45';           % ODE solver
sim.max_step = 0.01;            % Maximum step size (s)

%% Initial Conditions
init.position = [0; 0; 0];           % Initial position [x, y, z] (m)
init.velocity = [0; 0; 0];           % Initial velocity [vx, vy, vz] (m/s)
init.attitude = [0; 0; 0];           % Initial attitude [roll, pitch, yaw] (rad)
init.angular_velocity = [0; 0; 0];   % Initial angular velocity [p, q, r] (rad/s)

%% Save all parameters to workspace and file
fprintf('Saving parameters to workspace and file...\n');

try
    % Ensure we're in the right directory and data folder exists
    project_root = pwd;
    data_dir = fullfile(project_root, 'data');
    
    % Create data directory if it doesn't exist
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
        fprintf('Created data directory: %s\n', data_dir);
    end
    
    % Create full file path
    parameter_file = fullfile(data_dir, 'drone_parameters.mat');
    fprintf('Saving to: %s\n', parameter_file);
    
    % Save all parameters
    save(parameter_file, 'drone', 'motor', 'prop', 'env', 'sensors', ...
         'battery', 'flight_modes', 'control', 'pid', 'lqr', 'mpc', 'sim', 'init');
    
    % Also assign to base workspace for immediate use
    assignin('base', 'drone', drone);
    assignin('base', 'motor', motor);
    assignin('base', 'prop', prop);
    assignin('base', 'env', env);
    assignin('base', 'sensors', sensors);
    assignin('base', 'battery', battery);
    assignin('base', 'flight_modes', flight_modes);
    assignin('base', 'control', control);
    assignin('base', 'pid', pid);
    assignin('base', 'lqr', lqr);
    assignin('base', 'mpc', mpc);
    assignin('base', 'sim', sim);
    assignin('base', 'init', init);
    
    fprintf('✓ Parameters saved successfully to: %s\n', parameter_file);
    
catch ME
    fprintf('✗ Error saving parameters: %s\n', ME.message);
    error('Failed to save drone parameters. Check directory permissions.');
end

%% Display summary
fprintf('\n==========================================\n');
fprintf('Drone Parameters Initialized Successfully!\n');
fprintf('==========================================\n');
fprintf('Mass: %.3f kg\n', drone.mass);
fprintf('Arm length: %.3f m\n', drone.arm_length);
fprintf('Max thrust per motor: %.2f N\n', motor.max_thrust);
fprintf('Battery capacity: %d mAh\n', battery.capacity);
fprintf('Simulation time: %.1f s\n', sim.simulation_time);
fprintf('Parameter file: %s\n', parameter_file);
fprintf('==========================================\n');

end
