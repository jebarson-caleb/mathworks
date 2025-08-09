%% Drone Parameters Initialization
% This script defines all physical and control parameters for the mini drone
% Based on typical mini quadcopter specifications (e.g., DJI Tello, Parrot Mambo)

fprintf('Loading drone physical parameters...\n');

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
env.wind.turbulence_intensity = 0.1; % Turbulence intensity

%% Sensor Parameters
% IMU (Inertial Measurement Unit)
sensors.imu.accel_noise_std = 0.01;      % Accelerometer noise std dev (m/s²)
sensors.imu.gyro_noise_std = 0.001;      % Gyroscope noise std dev (rad/s)
sensors.imu.accel_bias_std = 0.005;      % Accelerometer bias std dev (m/s²)
sensors.imu.gyro_bias_std = 0.0005;      % Gyroscope bias std dev (rad/s)
sensors.imu.sample_rate = 1000;          % IMU sample rate (Hz)

% Barometer
sensors.baro.noise_std = 0.1;            % Pressure altitude noise std dev (m)
sensors.baro.bias_std = 0.5;             % Pressure altitude bias std dev (m)
sensors.baro.sample_rate = 50;           % Barometer sample rate (Hz)

% GPS (if equipped)
sensors.gps.position_noise_std = 1.0;    % Position noise std dev (m)
sensors.gps.velocity_noise_std = 0.1;    % Velocity noise std dev (m/s)
sensors.gps.sample_rate = 10;            % GPS sample rate (Hz)
sensors.gps.availability = 0.95;         % GPS availability probability

% Optical flow sensor
sensors.optflow.noise_std = 0.01;        % Optical flow noise std dev (rad/s)
sensors.optflow.max_range = 4.0;         % Maximum sensing range (m)
sensors.optflow.fov = 42;                % Field of view (degrees)

%% Battery Model Parameters
battery.nominal_voltage = 3.7;           % Nominal voltage (V)
battery.capacity = 1100;                 % Capacity (mAh)
battery.internal_resistance = 0.08;      % Internal resistance (Ohm)
battery.discharge_curve = [4.2, 3.7, 3.4, 3.0]; % Voltage curve points
battery.soc_points = [100, 50, 20, 0];   % State of charge points (%)

%% Control System Parameters
% Flight modes
flight_modes.manual = 1;
flight_modes.stabilize = 2;
flight_modes.altitude_hold = 3;
flight_modes.position_hold = 4;
flight_modes.auto_mission = 5;

% Control allocation matrix (thrust to motor commands)
% Motors arranged in X configuration: 1(FL), 2(FR), 3(RL), 4(RR)
control.allocation_matrix = [
    1,  1,  1,  1;           % Total thrust
    0,  drone.arm_length,  0, -drone.arm_length;  % Roll moment
    drone.arm_length,  0, -drone.arm_length,  0;  % Pitch moment
    -motor.drag_coeff/motor.thrust_coeff,  motor.drag_coeff/motor.thrust_coeff, ...
    -motor.drag_coeff/motor.thrust_coeff,  motor.drag_coeff/motor.thrust_coeff   % Yaw moment
];

%% PID Controller Parameters (Initial Values)
% Position Control (Outer Loop)
pid.pos.P = [2.0, 2.0, 4.0];    % Position P gains [x, y, z]
pid.pos.I = [0.1, 0.1, 0.5];    % Position I gains [x, y, z]
pid.pos.D = [0.5, 0.5, 1.0];    % Position D gains [x, y, z]

% Velocity Control
pid.vel.P = [3.0, 3.0, 3.0];    % Velocity P gains [x, y, z]
pid.vel.I = [0.5, 0.5, 0.5];    % Velocity I gains [x, y, z]
pid.vel.D = [0.1, 0.1, 0.1];    % Velocity D gains [x, y, z]

% Attitude Control (Inner Loop)
pid.att.P = [8.0, 8.0, 4.0];    % Attitude P gains [roll, pitch, yaw]
pid.att.I = [0.2, 0.2, 0.1];    % Attitude I gains [roll, pitch, yaw]
pid.att.D = [0.15, 0.15, 0.05]; % Attitude D gains [roll, pitch, yaw]

% Angular Rate Control
pid.rate.P = [0.15, 0.15, 0.25]; % Rate P gains [roll, pitch, yaw]
pid.rate.I = [0.1, 0.1, 0.05];   % Rate I gains [roll, pitch, yaw]
pid.rate.D = [0.003, 0.003, 0.001]; % Rate D gains [roll, pitch, yaw]

%% LQR Controller Parameters
% State: [x, y, z, roll, pitch, yaw, vx, vy, vz, p, q, r]
lqr.Q = diag([10, 10, 10, 5, 5, 5, 1, 1, 1, 0.1, 0.1, 0.1]); % State weights
lqr.R = diag([1, 1, 1, 1]);  % Control weights (4 motors)
lqr.N = zeros(12, 4);         % Cross-coupling weights

%% MPC Controller Parameters
mpc.prediction_horizon = 20;   % Prediction horizon
mpc.control_horizon = 5;       % Control horizon
mpc.sample_time = 0.02;        % Sample time (s)

% Constraints
mpc.constraints.u_min = [0, 0, 0, 0];           % Minimum motor commands
mpc.constraints.u_max = [1, 1, 1, 1];           % Maximum motor commands
mpc.constraints.attitude_max = [30, 30, 180];   % Max attitudes (degrees)
mpc.constraints.velocity_max = [5, 5, 3];       % Max velocities (m/s)

%% Simulation Parameters
sim.sample_time = 0.001;       % Base sample time (s)
sim.simulation_time = 30;       % Total simulation time (s)
sim.solver = 'ode45';          % Solver type
sim.relative_tolerance = 1e-6;  % Relative tolerance
sim.absolute_tolerance = 1e-8;  % Absolute tolerance

%% Initial Conditions
init.position = [0; 0; 0];      % Initial position [x, y, z] (m)
init.velocity = [0; 0; 0];      % Initial velocity [vx, vy, vz] (m/s)
init.attitude = [0; 0; 0];      % Initial attitude [roll, pitch, yaw] (rad)
init.angular_velocity = [0; 0; 0]; % Initial angular velocity [p, q, r] (rad/s)

%% Save all parameters
fprintf('Saving parameters to workspace and file...\n');

% Save to .mat file for easy loading
save('data/drone_parameters.mat', 'drone', 'motor', 'prop', 'env', 'sensors', ...
     'battery', 'flight_modes', 'control', 'pid', 'lqr', 'mpc', 'sim', 'init');

% Display summary
fprintf('Drone parameters loaded successfully!\n');
fprintf('Mass: %.3f kg\n', drone.mass);
fprintf('Arm length: %.3f m\n', drone.arm_length);
fprintf('Max thrust per motor: %.2f N\n', motor.max_thrust);
fprintf('Battery capacity: %d mAh\n', battery.capacity);
fprintf('Simulation time: %.1f s\n', sim.simulation_time);
