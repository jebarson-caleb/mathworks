%% Comprehensive Drone Parameters Database
% Complete parameter set for mini quadcopter simulation
% Based on real mini drone specifications and literature values

% Save this data to a .mat file for use in Simulink models
fprintf('Generating comprehensive drone parameters database...\n');

%% ==========================================
%% PHYSICAL PROPERTIES
%% ==========================================

% Basic drone characteristics (based on DJI Tello, Parrot Mambo)
drone = struct();
drone.name = 'Mini Quadcopter';
drone.category = 'Racing/Hobby';
drone.mass = 0.087;                    % Total mass [kg]
drone.arm_length = 0.046;              % Center to motor distance [m]
drone.frame_width = 0.092;             % Frame width [m]
drone.frame_height = 0.041;            % Frame height [m]

% Mass distribution
drone.cg_offset = [0; 0; 0.005];       % CG offset from geometric center [m]
drone.payload_capacity = 0.020;         % Maximum payload [kg]

% Moments of inertia [kg⋅m²]
drone.Ixx = 1.4e-5;                    % Roll moment of inertia
drone.Iyy = 1.4e-5;                    % Pitch moment of inertia
drone.Izz = 2.17e-5;                   % Yaw moment of inertia
drone.Ixy = 0;                         % Cross product (symmetric design)
drone.Ixz = 0;                         % Cross product
drone.Iyz = 0;                         % Cross product

% Full inertia matrix
drone.I = [drone.Ixx, drone.Ixy, drone.Ixz;
           drone.Ixy, drone.Iyy, drone.Iyz;
           drone.Ixz, drone.Iyz, drone.Izz];

% Aerodynamic properties
drone.drag_coeffs = [0.1, 0.1, 0.15];  % [Cx, Cy, Cz] body drag coefficients
drone.reference_area = 0.01;           % Reference area for drag calculation [m²]

%% ==========================================
%% MOTOR AND PROPELLER SPECIFICATIONS
%% ==========================================

% Motor specifications (coreless brushed motor typical for mini drones)
motor = struct();
motor.type = 'Coreless Brushed';
motor.nominal_voltage = 3.7;           % Nominal voltage [V]
motor.max_voltage = 4.2;               % Maximum voltage [V]
motor.stall_current = 1.2;             % Stall current [A]
motor.no_load_current = 0.05;          % No-load current [A]
motor.resistance = 2.5;                % Motor resistance [Ω]
motor.inductance = 8e-6;               % Motor inductance [H]
motor.time_constant = 0.018;           % Mechanical time constant [s]
motor.max_rpm = 31000;                 % Maximum RPM
motor.max_power = 4.5;                 % Maximum power [W]

% Thrust and torque characteristics
motor.thrust_coeff = 1.69e-6;          % Thrust coefficient [N⋅s²/rad²]
motor.drag_coeff = 2.9e-8;             % Drag coefficient [N⋅m⋅s²/rad²]
motor.max_thrust = 0.29;               % Maximum thrust per motor [N]
motor.hover_thrust = 0.067;            % Thrust per motor at hover [N]

% Motor efficiency curve (RPM vs efficiency)
motor.efficiency_rpm = [0, 5000, 10000, 15000, 20000, 25000, 31000];
motor.efficiency_values = [0, 0.65, 0.78, 0.82, 0.85, 0.80, 0.70];

% Propeller specifications
prop = struct();
prop.diameter = 0.046;                 % Propeller diameter [m]
prop.pitch = 0.024;                    % Propeller pitch [m]
prop.chord = 0.008;                    % Average chord length [m]
prop.num_blades = 2;                   % Number of blades
prop.blade_angle = 15;                 % Blade angle at 75% radius [deg]
prop.material = 'Plastic';             % Blade material
prop.mass = 0.0008;                    % Propeller mass [kg]
prop.moment_inertia = 2.1e-7;          % Propeller moment of inertia [kg⋅m²]

%% ==========================================
%% BATTERY AND POWER SYSTEM
%% ==========================================

battery = struct();
battery.type = 'LiPo';
battery.nominal_voltage = 3.7;         % Nominal cell voltage [V]
battery.cells_series = 1;              % Number of cells in series
battery.total_voltage = 3.7;           % Total battery voltage [V]
battery.capacity = 1100;               % Capacity [mAh]
battery.energy = 4.07;                 % Energy content [Wh]
battery.max_discharge_rate = 10;       % C-rate
battery.internal_resistance = 0.08;    % Internal resistance [Ω]
battery.mass = 0.024;                  % Battery mass [kg]

% Discharge characteristics
battery.voltage_curve = [4.2, 3.85, 3.7, 3.5, 3.3, 3.0]; % Voltage points [V]
battery.soc_points = [100, 80, 50, 20, 10, 0];            % State of charge [%]
battery.capacity_curve = [1100, 1080, 1050, 1000, 950, 900]; % Available capacity [mAh]

% Temperature effects
battery.temp_derating = [-20, 0, 25, 45, 60];             % Temperature [°C]
battery.capacity_factor = [0.6, 0.8, 1.0, 0.95, 0.85];   % Capacity factor

%% ==========================================
%% SENSOR SPECIFICATIONS
%% ==========================================

sensors = struct();

% Inertial Measurement Unit (IMU)
sensors.imu = struct();
sensors.imu.type = 'MEMS 6-DOF';
sensors.imu.sample_rate = 1000;                     % Sample rate [Hz]
sensors.imu.bandwidth = 250;                        % Bandwidth [Hz]

% Accelerometer
sensors.imu.accel_range = 8;                        % Full scale range [g]
sensors.imu.accel_resolution = 16;                  % Resolution [bits]
sensors.imu.accel_noise_density = 150e-6;          % Noise density [g/√Hz]
sensors.imu.accel_bias_stability = 0.02e-3;        % Bias stability [g]
sensors.imu.accel_scale_factor_error = 0.5;        % Scale factor error [%]
sensors.imu.accel_cross_axis_sensitivity = 1;      % Cross-axis sensitivity [%]

% Gyroscope  
sensors.imu.gyro_range = 2000;                      % Full scale range [°/s]
sensors.imu.gyro_resolution = 16;                   % Resolution [bits]
sensors.imu.gyro_noise_density = 0.01;             % Noise density [°/s/√Hz]
sensors.imu.gyro_bias_stability = 10;              % Bias stability [°/h]
sensors.imu.gyro_scale_factor_error = 0.2;         % Scale factor error [%]

% Magnetometer (optional)
sensors.mag = struct();
sensors.mag.present = false;                       % Not typically used in mini drones
sensors.mag.range = 1000;                          % Range [µT]
sensors.mag.resolution = 12;                       % Resolution [bits]
sensors.mag.noise = 2;                             % RMS noise [µT]

% Barometer
sensors.baro = struct();
sensors.baro.present = true;
sensors.baro.sample_rate = 50;                     % Sample rate [Hz]
sensors.baro.pressure_range = [300, 1100];         % Pressure range [hPa]
sensors.baro.altitude_resolution = 0.06;           % Altitude resolution [m]
sensors.baro.pressure_noise = 0.1;                 % Pressure noise RMS [hPa]
sensors.baro.altitude_noise = 0.8;                 % Altitude noise RMS [m]
sensors.baro.response_time = 0.02;                 % Response time [s]

% Optical flow sensor (common in mini drones)
sensors.optflow = struct();
sensors.optflow.present = true;
sensors.optflow.sample_rate = 100;                 % Sample rate [Hz]
sensors.optflow.fov = 42;                          % Field of view [degrees]
sensors.optflow.max_range = 4.0;                   % Maximum range [m]
sensors.optflow.min_range = 0.08;                  % Minimum range [m]
sensors.optflow.resolution = 0.1;                  % Flow resolution [rad/s]
sensors.optflow.noise_std = 0.05;                  % Flow noise std dev [rad/s]

% GPS (if present - not common in mini indoor drones)
sensors.gps = struct();
sensors.gps.present = false;                       % Usually not present
sensors.gps.sample_rate = 10;                      % Sample rate [Hz]
sensors.gps.horizontal_accuracy = 3.0;             % Horizontal accuracy [m]
sensors.gps.vertical_accuracy = 5.0;               % Vertical accuracy [m]
sensors.gps.velocity_accuracy = 0.1;               % Velocity accuracy [m/s]

%% ==========================================
%% ENVIRONMENTAL PARAMETERS
%% ==========================================

env = struct();
env.gravity = 9.81;                                % Gravitational acceleration [m/s²]
env.air_density = 1.225;                          % Air density at sea level [kg/m³]
env.air_viscosity = 1.81e-5;                      % Dynamic viscosity [Pa⋅s]
env.speed_of_sound = 343;                         % Speed of sound [m/s]
env.temperature = 20;                             % Standard temperature [°C]
env.pressure = 101325;                            % Standard pressure [Pa]

% Wind model parameters
env.wind = struct();
env.wind.turbulence_intensity = 0.1;             % Turbulence intensity
env.wind.turbulence_scale = 10;                  % Turbulence length scale [m]
env.wind.gust_factor = 1.5;                      % Gust factor
env.wind.reference_height = 10;                   % Reference height [m]
env.wind.surface_roughness = 0.1;                % Surface roughness [m]

%% ==========================================
%% FLIGHT CONTROLLER PARAMETERS
%% ==========================================

% Flight modes
flight_modes = struct();
flight_modes.manual = 1;                          % Manual control
flight_modes.stabilize = 2;                      % Attitude stabilization
flight_modes.altitude_hold = 3;                  % Altitude hold
flight_modes.position_hold = 4;                  % Position hold
flight_modes.auto_mission = 5;                   % Autonomous mission
flight_modes.return_to_launch = 6;               % Return to home
flight_modes.land = 7;                           % Auto landing

% Control frequencies
control = struct();
control.attitude_loop_freq = 1000;               % Attitude control frequency [Hz]
control.position_loop_freq = 50;                 % Position control frequency [Hz]
control.sensor_fusion_freq = 400;                % Sensor fusion frequency [Hz]
control.mission_update_freq = 10;                % Mission update frequency [Hz]

% Control allocation matrix (X configuration)
% Motors: 1=Front-Right, 2=Back-Right, 3=Back-Left, 4=Front-Left
control.allocation_matrix = [
    1,  1,  1,  1;                                % Total thrust
    0,  drone.arm_length,  0, -drone.arm_length; % Roll moment
    drone.arm_length,  0, -drone.arm_length,  0; % Pitch moment
    -motor.drag_coeff/motor.thrust_coeff,  motor.drag_coeff/motor.thrust_coeff, ...
    -motor.drag_coeff/motor.thrust_coeff,  motor.drag_coeff/motor.thrust_coeff   % Yaw moment
];

%% ==========================================
%% CONTROL SYSTEM GAINS (BASELINE VALUES)
%% ==========================================

% PID controller gains
pid = struct();

% Position control gains (outer loop)
pid.pos_p = [4.0, 4.0, 6.0];                     % Position P gains [x, y, z]
pid.pos_i = [0.4, 0.4, 1.0];                     % Position I gains [x, y, z]
pid.pos_d = [0.8, 0.8, 1.5];                     % Position D gains [x, y, z]
pid.pos_imax = [2.0, 2.0, 3.0];                  % Position I limits [x, y, z]

% Velocity control gains
pid.vel_p = [5.0, 5.0, 5.0];                     % Velocity P gains [x, y, z]
pid.vel_i = [1.0, 1.0, 1.0];                     % Velocity I gains [x, y, z]
pid.vel_d = [0.2, 0.2, 0.2];                     % Velocity D gains [x, y, z]
pid.vel_imax = [3.0, 3.0, 3.0];                  % Velocity I limits [x, y, z]

% Attitude control gains (middle loop)
pid.att_p = [8.0, 8.0, 4.0];                     % Attitude P gains [roll, pitch, yaw]
pid.att_i = [0.3, 0.3, 0.15];                    % Attitude I gains [roll, pitch, yaw]
pid.att_d = [0.15, 0.15, 0.08];                  % Attitude D gains [roll, pitch, yaw]
pid.att_imax = [30, 30, 30];                     % Attitude I limits [deg]

% Angular rate control gains (inner loop)
pid.rate_p = [0.18, 0.18, 0.25];                 % Rate P gains [roll, pitch, yaw]
pid.rate_i = [0.12, 0.12, 0.08];                 % Rate I gains [roll, pitch, yaw]
pid.rate_d = [0.004, 0.004, 0.002];              % Rate D gains [roll, pitch, yaw]
pid.rate_imax = [0.3, 0.3, 0.3];                 % Rate I limits [rad/s]

% Control limits
control.limits = struct();
control.limits.max_tilt_angle = deg2rad(45);      % Maximum tilt angle [rad]
control.limits.max_climb_rate = 3.0;              % Maximum climb rate [m/s]
control.limits.max_descent_rate = 2.0;            % Maximum descent rate [m/s]
control.limits.max_horizontal_speed = 5.0;        % Maximum horizontal speed [m/s]
control.limits.max_yaw_rate = deg2rad(90);        % Maximum yaw rate [rad/s]

%% ==========================================
%% SIMULATION PARAMETERS
%% ==========================================

sim = struct();
sim.sample_time = 0.001;                         % Base sample time [s]
sim.simulation_time = 30;                        % Default simulation time [s]
sim.solver = 'ode45';                            % Solver type
sim.relative_tolerance = 1e-6;                   % Relative tolerance
sim.absolute_tolerance = 1e-8;                   % Absolute tolerance
sim.max_step = 0.01;                             % Maximum step size [s]
sim.initial_step = 1e-6;                         % Initial step size [s]

%% ==========================================
%% INITIAL CONDITIONS
%% ==========================================

init = struct();
init.position = [0; 0; 0];                       % Initial position [x, y, z] [m]
init.velocity = [0; 0; 0];                       % Initial velocity [vx, vy, vz] [m/s]
init.attitude = [0; 0; 0];                       % Initial attitude [roll, pitch, yaw] [rad]
init.angular_velocity = [0; 0; 0];               % Initial angular velocity [p, q, r] [rad/s]
init.motor_speeds = [0; 0; 0; 0];                % Initial motor speeds [rad/s]
init.battery_soc = 100;                          % Initial battery state of charge [%]

%% ==========================================
%% SAFETY AND LIMITS
%% ==========================================

safety = struct();
safety.geofence_radius = 50;                     % Maximum horizontal distance [m]
safety.max_altitude = 30;                        % Maximum altitude [m]
safety.min_altitude = 0.1;                       % Minimum altitude [m]
safety.low_battery_threshold = 20;               % Low battery warning [%]
safety.critical_battery_threshold = 10;          % Critical battery level [%]
safety.max_tilt_angle = deg2rad(60);             % Emergency tilt limit [rad]
safety.failsafe_descent_rate = 1.0;              % Emergency descent rate [m/s]

%% ==========================================
%% COMMUNICATION PARAMETERS
%% ==========================================

comm = struct();
comm.radio_frequency = 2.4e9;                    % Radio frequency [Hz]
comm.max_range = 100;                            % Maximum communication range [m]
comm.update_rate = 50;                           % Command update rate [Hz]
comm.latency = 0.02;                             % Communication latency [s]
comm.packet_loss_rate = 0.01;                    % Packet loss rate [%]

%% ==========================================
%% SAVE PARAMETERS TO FILE
%% ==========================================

% Create timestamp for version control
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

% Save all parameters
parameter_file = 'drone_parameters_complete.mat';  % Save in current directory (data/)
save(parameter_file, 'drone', 'motor', 'prop', 'battery', 'sensors', 'env', ...
     'flight_modes', 'control', 'pid', 'sim', 'init', 'safety', 'comm', 'timestamp');

% Also save a backup with timestamp
backup_file = sprintf('drone_parameters_%s.mat', timestamp);
save(backup_file, 'drone', 'motor', 'prop', 'battery', 'sensors', 'env', ...
     'flight_modes', 'control', 'pid', 'sim', 'init', 'safety', 'comm', 'timestamp');

%% ==========================================
%% PARAMETER VALIDATION
%% ==========================================

fprintf('Parameter validation:\n');
fprintf('====================\n');

% Check hover capability
total_max_thrust = 4 * motor.max_thrust;
hover_thrust_margin = total_max_thrust / (drone.mass * env.gravity);
fprintf('Hover thrust margin: %.2f (should be > 2.0)\n', hover_thrust_margin);

% Check battery endurance
hover_power_per_motor = motor.hover_thrust^2 / motor.thrust_coeff * motor.nominal_voltage / 1000; % Rough estimate
total_hover_power = 4 * hover_power_per_motor;
estimated_hover_time = battery.energy / total_hover_power * 60; % minutes
fprintf('Estimated hover time: %.1f minutes\n', estimated_hover_time);

% Check control authority
max_roll_moment = 2 * motor.max_thrust * drone.arm_length;
max_roll_acceleration = max_roll_moment / drone.Ixx;
fprintf('Maximum roll acceleration: %.1f rad/s²\n', max_roll_acceleration);

% Check weight distribution
weight_per_motor_hover = drone.mass * env.gravity / 4;
motor_utilization_hover = weight_per_motor_hover / motor.max_thrust * 100;
fprintf('Motor utilization at hover: %.1f%%\n', motor_utilization_hover);

fprintf('\nParameter database generated successfully!\n');
fprintf('Main file: %s\n', parameter_file);
fprintf('Backup file: %s\n', backup_file);
fprintf('Total parameters: %d structures\n', 13);
