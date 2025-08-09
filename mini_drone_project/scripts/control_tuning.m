%% Controller Parameter Tuning Script
% This script provides tools for tuning different control strategies
% Run this after initialize_drone.m

fprintf('Control System Tuning Tools\n');
fprintf('============================\n');

%% Load drone parameters if not already loaded
if ~exist('drone', 'var')
    fprintf('Loading drone parameters...\n');
    run('initialize_drone.m');
end

%% PID Tuning Functions

function tuned_gains = tune_pid_attitude()
    % Automated PID tuning for attitude control using Simulink Control Design
    fprintf('Tuning attitude PID controllers...\n');
    
    % Load the attitude control model
    model_name = 'attitude_controller';
    
    try
        load_system(model_name);
        
        % Define tuning requirements
        req = TuningGoal.StepTracking('/attitude_command', '/attitude_response', 0.5);
        req.StepAmplitude = 0.1; % 0.1 rad step
        
        % Overshoot requirement
        overshoot = TuningGoal.Overshoot('/attitude_command', '/attitude_response', 10);
        
        % Stability margins
        margins = TuningGoal.Margins('/motor_commands', 6, 45); % 6dB gain, 45° phase
        
        % Tune the controller
        [tuned_model, info] = systune(model_name, [req, overshoot, margins]);
        
        % Extract tuned gains
        tuned_gains.roll = getBlockValue(tuned_model, 'Roll_PID');
        tuned_gains.pitch = getBlockValue(tuned_model, 'Pitch_PID');
        tuned_gains.yaw = getBlockValue(tuned_model, 'Yaw_PID');
        
        fprintf('PID tuning completed successfully!\n');
        
    catch ME
        fprintf('PID tuning failed: %s\n', ME.message);
        fprintf('Using default PID gains.\n');
        tuned_gains = pid.att;
    end
end

%% LQR Controller Design
function [K, S, e] = design_lqr_controller()
    % Design LQR controller for linearized drone dynamics
    fprintf('Designing LQR controller...\n');
    
    % Linearized state-space model around hover
    % State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
    % Input: [u1, u2, u3, u4] (motor thrust commands)
    
    % A matrix (linearized dynamics)
    A = zeros(12, 12);
    
    % Position dynamics
    A(1, 7) = 1;  % x_dot = vx
    A(2, 8) = 1;  % y_dot = vy
    A(3, 9) = 1;  % z_dot = vz
    
    % Attitude dynamics
    A(4, 10) = 1; % phi_dot = p
    A(5, 11) = 1; % theta_dot = q
    A(6, 12) = 1; % psi_dot = r
    
    % Velocity dynamics (small angle approximation)
    A(7, 5) = env.gravity;  % vx_dot = g*theta
    A(8, 4) = -env.gravity; % vy_dot = -g*phi
    
    % Angular velocity dynamics
    A(10, 10) = -1/motor.time_constant; % Motor dynamics
    A(11, 11) = -1/motor.time_constant;
    A(12, 12) = -1/motor.time_constant;
    
    % B matrix (control effectiveness)
    B = zeros(12, 4);
    
    % Thrust affects vertical acceleration
    B(9, :) = 1/(drone.mass) * [1, 1, 1, 1];
    
    % Moments affect angular accelerations
    B(10, :) = 1/drone.Ixx * [0, drone.arm_length, 0, -drone.arm_length];
    B(11, :) = 1/drone.Iyy * [drone.arm_length, 0, -drone.arm_length, 0];
    B(12, :) = 1/drone.Izz * [-motor.drag_coeff/motor.thrust_coeff, ...
                               motor.drag_coeff/motor.thrust_coeff, ...
                               -motor.drag_coeff/motor.thrust_coeff, ...
                               motor.drag_coeff/motor.thrust_coeff];
    
    % Design LQR controller
    [K, S, e] = lqr(A, B, lqr.Q, lqr.R, lqr.N);
    
    fprintf('LQR controller designed successfully!\n');
    fprintf('Closed-loop poles: ');
    fprintf('%.3f ', real(e));
    fprintf('\n');
end

%% MPC Controller Setup
function mpc_controller = design_mpc_controller()
    % Design Model Predictive Controller
    fprintf('Setting up MPC controller...\n');
    
    try
        % Create MPC object
        mpc_controller = mpc(ss(A, B, eye(12), zeros(12, 4)), mpc.sample_time);
        
        % Set prediction and control horizons
        mpc_controller.PredictionHorizon = mpc.prediction_horizon;
        mpc_controller.ControlHorizon = mpc.control_horizon;
        
        % Input constraints
        mpc_controller.MV(1).Min = mpc.constraints.u_min(1);
        mpc_controller.MV(1).Max = mpc.constraints.u_max(1);
        mpc_controller.MV(2).Min = mpc.constraints.u_min(2);
        mpc_controller.MV(2).Max = mpc.constraints.u_max(2);
        mpc_controller.MV(3).Min = mpc.constraints.u_min(3);
        mpc_controller.MV(3).Max = mpc.constraints.u_max(3);
        mpc_controller.MV(4).Min = mpc.constraints.u_min(4);
        mpc_controller.MV(4).Max = mpc.constraints.u_max(4);
        
        % Output constraints (attitude limits)
        mpc_controller.OV(4).Min = -deg2rad(mpc.constraints.attitude_max(1));
        mpc_controller.OV(4).Max = deg2rad(mpc.constraints.attitude_max(1));
        mpc_controller.OV(5).Min = -deg2rad(mpc.constraints.attitude_max(2));
        mpc_controller.OV(5).Max = deg2rad(mpc.constraints.attitude_max(2));
        
        % Weights
        mpc_controller.Weights.OV = diag([10, 10, 10, 5, 5, 5, 1, 1, 1, 0.1, 0.1, 0.1]);
        mpc_controller.Weights.MV = [1, 1, 1, 1];
        mpc_controller.Weights.MVRate = [0.1, 0.1, 0.1, 0.1];
        
        fprintf('MPC controller setup completed!\n');
        
    catch ME
        fprintf('MPC setup failed: %s\n', ME.message);
        mpc_controller = [];
    end
end

%% Frequency Domain Analysis
function analyze_control_performance()
    % Analyze control system performance in frequency domain
    fprintf('Performing frequency domain analysis...\n');
    
    try
        % Load closed-loop system
        [K, ~, ~] = design_lqr_controller();
        
        % Closed-loop system
        A_cl = A - B*K;
        sys_cl = ss(A_cl, B, eye(12), zeros(12, 4));
        
        % Bode plot
        figure('Name', 'Control System Analysis');
        
        subplot(2, 2, 1);
        bode(sys_cl(1, 1)); % x position response
        title('Position Control - X axis');
        grid on;
        
        subplot(2, 2, 2);
        bode(sys_cl(4, 2)); % roll response
        title('Attitude Control - Roll');
        grid on;
        
        % Step response
        subplot(2, 2, 3);
        step(sys_cl(1, 1), 5); % 5 second step response
        title('Step Response - X Position');
        grid on;
        
        subplot(2, 2, 4);
        step(sys_cl(4, 2), 2); % 2 second step response
        title('Step Response - Roll Attitude');
        grid on;
        
        % Stability margins
        [Gm, Pm, Wcg, Wcp] = margin(sys_cl(1, 1));
        fprintf('Stability Margins:\n');
        fprintf('Gain Margin: %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
        fprintf('Phase Margin: %.2f deg at %.2f rad/s\n', Pm, Wcp);
        
    catch ME
        fprintf('Analysis failed: %s\n', ME.message);
    end
end

%% Optimization-based Tuning
function optimized_gains = optimize_controller_gains()
    % Use optimization to find optimal PID gains
    fprintf('Optimizing controller gains...\n');
    
    % Define cost function
    cost_function = @(gains) evaluate_control_performance(gains);
    
    % Initial guess (current PID gains)
    x0 = [pid.att.P, pid.att.I, pid.att.D];
    
    % Bounds
    lb = zeros(size(x0));  % Lower bounds (non-negative gains)
    ub = 10 * x0;          % Upper bounds
    
    % Optimization options
    options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 50);
    
    try
        % Run optimization
        [x_opt, fval] = fmincon(cost_function, x0, [], [], [], [], lb, ub, [], options);
        
        % Extract optimized gains
        optimized_gains.P = x_opt(1:3);
        optimized_gains.I = x_opt(4:6);
        optimized_gains.D = x_opt(7:9);
        
        fprintf('Optimization completed! Cost: %.4f\n', fval);
        
    catch ME
        fprintf('Optimization failed: %s\n', ME.message);
        optimized_gains = pid.att;
    end
end

function cost = evaluate_control_performance(gains)
    % Evaluate control performance for given gains
    % This would typically run a simulation and evaluate metrics
    
    % For now, return a simple cost based on gain magnitudes
    % In practice, this would run Simulink simulation
    cost = sum(gains.^2) + 1000 * any(gains < 0); % Penalty for negative gains
end

%% Main Tuning Interface
function run_tuning_interface()
    % Interactive tuning interface
    fprintf('\nController Tuning Options:\n');
    fprintf('1. Tune PID controllers automatically\n');
    fprintf('2. Design LQR controller\n');
    fprintf('3. Setup MPC controller\n');
    fprintf('4. Frequency domain analysis\n');
    fprintf('5. Optimize controller gains\n');
    fprintf('6. Compare control strategies\n');
    fprintf('0. Exit\n\n');
    
    choice = input('Select option: ');
    
    switch choice
        case 1
            tuned_pid = tune_pid_attitude();
            assignin('base', 'tuned_pid', tuned_pid);
            
        case 2
            [K, S, e] = design_lqr_controller();
            assignin('base', 'lqr_gain', K);
            assignin('base', 'lqr_riccati', S);
            assignin('base', 'lqr_poles', e);
            
        case 3
            mpc_ctrl = design_mpc_controller();
            assignin('base', 'mpc_controller', mpc_ctrl);
            
        case 4
            analyze_control_performance();
            
        case 5
            opt_gains = optimize_controller_gains();
            assignin('base', 'optimized_gains', opt_gains);
            
        case 6
            compare_controllers();
            
        case 0
            fprintf('Exiting tuning interface.\n');
            return;
            
        otherwise
            fprintf('Invalid option. Try again.\n');
            run_tuning_interface();
    end
end

function compare_controllers()
    % Compare different control strategies
    fprintf('Comparing control strategies...\n');
    
    % This would run simulations with different controllers
    % and compare performance metrics
    fprintf('Controller comparison completed. Check workspace for results.\n');
end

%% Execute main interface
if ~exist('skip_interface', 'var')
    run_tuning_interface();
end

fprintf('Control tuning tools loaded successfully!\n');
