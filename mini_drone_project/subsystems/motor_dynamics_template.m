% Motor Dynamics Subsystem Template
% Models brushless motor and propeller dynamics

% Model: motor_dynamics.slx
% Description: Individual motor and propeller model with realistic dynamics

% ==========================================
% MOTOR MODEL OVERVIEW
% ==========================================

% This subsystem models:
% 1. Brushless DC motor electrical dynamics
% 2. Propeller aerodynamics  
% 3. Motor-propeller coupling
% 4. Motor limitations and saturation

% ==========================================
% ELECTRICAL MOTOR MODEL
% ==========================================

% Simplified BLDC motor equations:
% 
% Voltage equation:
% V = R*I + L*dI/dt + Ke*ω
%
% Where:
% V = applied voltage [V]
% I = motor current [A]  
% R = motor resistance [Ω]
% L = motor inductance [H]
% Ke = back-EMF constant [V⋅s/rad]
% ω = motor angular velocity [rad/s]

% Torque equation:
% τm = Kt*I - τfriction - τload
%
% Where:
% τm = motor torque [N⋅m]
% Kt = torque constant [N⋅m/A]
% τfriction = bearing friction [N⋅m]
% τload = propeller load torque [N⋅m]

% Mechanical equation:
% J*dω/dt = τm
%
% Where:
% J = rotor inertia [kg⋅m²]

% ==========================================
% PROPELLER AERODYNAMICS
% ==========================================

% Thrust generation:
% T = Ct * ρ * n² * D⁴
%
% Where:
% T = thrust [N]
% Ct = thrust coefficient [-]
% ρ = air density [kg/m³]
% n = propeller rotation rate [rev/s] 
% D = propeller diameter [m]

% Torque generation:
% Q = Cq * ρ * n² * D⁵  
%
% Where:
% Q = propeller torque [N⋅m]
% Cq = torque coefficient [-]

% Relationship between angular velocity and rotation rate:
% ω = 2π * n [rad/s]

% Simplified model (commonly used):
% T = kt * ω²
% Q = kq * ω²
%
% Where:
% kt = thrust coefficient [N⋅s²/rad²]
% kq = drag coefficient [N⋅m⋅s²/rad²]

% ==========================================
% FIRST-ORDER APPROXIMATION
% ==========================================

% For control design, motors are often modeled as first-order systems:
%
% Transfer function:
% G(s) = K / (τs + 1)
%
% Where:
% K = steady-state gain
% τ = time constant [s]

% Step response:
% ω(t) = ωss * (1 - e^(-t/τ))

% Typical values for mini drone motors:
% τ = 0.01 - 0.05 s (20-100 Hz bandwidth)
% K = depends on motor and ESC characteristics

% ==========================================
% MOTOR LIMITATIONS
% ==========================================

% Voltage Saturation:
% Vmin ≤ V ≤ Vmax
% Typically: 0V ≤ V ≤ Battery_voltage

% Current Saturation:  
% 0A ≤ I ≤ Imax
% Protects motor from overcurrent

% Speed Saturation:
% 0 ≤ ω ≤ ωmax
% Maximum RPM limit

% Thrust Saturation:
% 0N ≤ T ≤ Tmax
% Physical thrust limit

% ==========================================
% TEMPERATURE EFFECTS
% ==========================================

% Motor parameters vary with temperature:
% R(T) = R₀ * (1 + α*(T - T₀))
% Kt(T) = Kt₀ * (1 + β*(T - T₀))
%
% Where:
% T = motor temperature [°C]
% T₀ = reference temperature [°C]
% α, β = temperature coefficients

% Thermal model:
% C*dT/dt = P_loss - h*(T - Tamb)
%
% Where:
% C = thermal capacity [J/°C]
% P_loss = I²*R (copper losses) [W]
% h = heat transfer coefficient [W/°C]
% Tamb = ambient temperature [°C]

% ==========================================
% SIMULINK IMPLEMENTATION
% ==========================================

% INPUT PORTS:
% 1. Motor Command [0-1] - ESC PWM signal
% 2. Air Density [kg/m³] - environmental condition
% 3. Ambient Temperature [°C] - thermal effects

% OUTPUT PORTS:
% 1. Motor Thrust [N] - generated thrust
% 2. Motor Torque [N⋅m] - propeller drag torque  
% 3. Motor RPM [rpm] - rotational speed
% 4. Motor Current [A] - electrical current
% 5. Motor Temperature [°C] - thermal state

% ==========================================
% SUBSYSTEM STRUCTURE
% ==========================================

% 1. COMMAND CONDITIONING:
%    - Input validation (0-1 range)
%    - Command filtering (remove noise)
%    - Rate limiting (prevent step changes)

% 2. ESC MODEL:
%    - PWM to voltage conversion
%    - ESC dynamics (typically fast)
%    - Protection features

% 3. MOTOR ELECTRICAL MODEL:
%    - First-order lag: 1/(τs+1)
%    - Current limiting
%    - Back-EMF effects

% 4. PROPELLER MODEL:
%    - Thrust: T = kt*ω²
%    - Drag torque: Q = kq*ω²
%    - Efficiency characteristics

% 5. THERMAL MODEL:
%    - Heat generation from losses
%    - Temperature rise calculation
%    - Thermal protection

% ==========================================
% PARAMETER VALUES (Typical Mini Drone)
% ==========================================

% Motor Specifications:
motor_params = struct(...
    'resistance', 0.1, ...          % Motor resistance [Ω]
    'inductance', 50e-6, ...        % Motor inductance [H]
    'ke_constant', 0.001, ...       % Back-EMF constant [V⋅s/rad]
    'kt_constant', 0.001, ...       % Torque constant [N⋅m/A]
    'rotor_inertia', 1e-6, ...      % Rotor inertia [kg⋅m²]
    'time_constant', 0.02, ...      % Overall time constant [s]
    'max_current', 3.0, ...         % Maximum current [A]
    'max_rpm', 31000, ...           % Maximum RPM
    'max_voltage', 11.1 ...         % Maximum voltage [V]
);

% Propeller Specifications:
prop_params = struct(...
    'diameter', 0.046, ...          % Propeller diameter [m]
    'pitch', 0.024, ...             % Propeller pitch [m]
    'thrust_coeff', 1.69e-6, ...    % Thrust coefficient [N⋅s²/rad²]
    'drag_coeff', 2.9e-8, ...       % Drag coefficient [N⋅m⋅s²/rad²]
    'efficiency', 0.85 ...          % Propeller efficiency [-]
);

% ESC Specifications:
esc_params = struct(...
    'pwm_frequency', 400, ...       % PWM frequency [Hz]
    'response_time', 0.002, ...     % ESC response time [s]
    'voltage_range', [6, 12], ...   % Operating voltage range [V]
    'current_limit', 10 ...         % Current limit [A]
);

% ==========================================
% VALIDATION AND TESTING
% ==========================================

% Static Tests:
% 1. Steady-state thrust vs RPM
% 2. Power consumption vs thrust
% 3. Efficiency curves

% Dynamic Tests:
% 1. Step response (command to thrust)
% 2. Frequency response
% 3. Saturation behavior

% Comparison with Real Data:
% 1. Bench test validation
% 2. Flight test correlation
% 3. Parameter identification

% ==========================================
% ADVANCED FEATURES
% ==========================================

% Blade Flapping (for larger drones):
% - Coriolis and gyroscopic effects
% - Blade flexibility
% - Dynamic inflow

% Ground Effect:
% - Increased thrust near ground
% - Modified inflow characteristics
% - Height-dependent efficiency

% Motor Aging:
% - Performance degradation over time
% - Magnet strength reduction
% - Bearing wear effects

% ==========================================
% SIMULINK BLOCKS USED
% ==========================================

% Transfer Functions:
% - First-order lag: tf([K], [tau, 1])
% - Lead-lag compensator for ESC

% Math Operations:
% - Square function (for thrust calculation)
% - Square root (for RPM calculation)
% - Lookup tables (for efficiency curves)

% Saturation Blocks:
% - Motor command limits [0, 1]
% - Current limits [0, Imax]
% - RPM limits [0, RPMmax]

% Signal Conditioning:
% - Rate limiters
% - Low-pass filters
% - Deadband functions

fprintf('Motor dynamics subsystem template loaded.\n');
fprintf('This template provides detailed motor and propeller modeling.\n');
fprintf('Key features:\n');
fprintf('1. Realistic motor electrical dynamics\n');
fprintf('2. Propeller aerodynamics\n');
fprintf('3. Thermal effects\n');
fprintf('4. Saturation and protection\n');
fprintf('5. Environmental dependencies\n');
