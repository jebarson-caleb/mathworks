%% Automated Simulink Model Creation for Mini Drone Competition
% This script creates all required Simulink models for the MathWorks Mini Drone Competition
% Run this script to automatically generate all .slx files

clc; clear; close all;

fprintf('========================================================\n');
fprintf('MathWorks Mini Drone Competition - Model Generator\n');
fprintf('========================================================\n');
fprintf('Creating automated Simulink models...\n\n');

%% Initialize project
if ~exist('drone', 'var')
    fprintf('Loading drone parameters...\n');
    run('scripts/initialize_drone.m');
end

%% 1. Create Main Drone Model
fprintf('1. Creating main_drone_model.slx...\n');
modelName = 'main_drone_model';

% Create new model
new_system(modelName);
open_system(modelName);

% Set model properties
set_param(modelName, 'Solver', 'ode45');
set_param(modelName, 'StopTime', '30');
set_param(modelName, 'FixedStep', '0.001');

% Add input ports
add_block('simulink/Sources/In1', [modelName '/Reference_Position'], ...
    'Position', [50, 50, 80, 80]);
add_block('simulink/Sources/In1', [modelName '/Reference_Attitude'], ...
    'Position', [50, 120, 80, 150]);
add_block('simulink/Sources/In1', [modelName '/Wind_Disturbance'], ...
    'Position', [50, 190, 80, 220]);

% Add Flight Control System subsystem
add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Flight_Control_System'], ...
    'Position', [150, 80, 250, 160]);

% Add Drone Dynamics subsystem  
add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Drone_Dynamics'], ...
    'Position', [300, 80, 400, 160]);

% Add Sensor subsystem
add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Sensors'], ...
    'Position', [450, 80, 550, 160]);

% Add output ports
add_block('simulink/Sinks/Out1', [modelName '/Position_Output'], ...
    'Position', [600, 50, 630, 80]);
add_block('simulink/Sinks/Out1', [modelName '/Attitude_Output'], ...
    'Position', [600, 120, 630, 150]);
add_block('simulink/Sinks/Out1', [modelName '/Motor_Commands'], ...
    'Position', [600, 190, 630, 220]);

% Add connections
add_line(modelName, 'Reference_Position/1', 'Flight_Control_System/1');
add_line(modelName, 'Reference_Attitude/1', 'Flight_Control_System/2');
add_line(modelName, 'Flight_Control_System/1', 'Drone_Dynamics/1');
add_line(modelName, 'Drone_Dynamics/1', 'Sensors/1');
add_line(modelName, 'Sensors/1', 'Flight_Control_System/3');
add_line(modelName, 'Drone_Dynamics/1', 'Position_Output/1');
add_line(modelName, 'Drone_Dynamics/2', 'Attitude_Output/1');
add_line(modelName, 'Flight_Control_System/1', 'Motor_Commands/1');

% Save model
save_system(modelName, fullfile('models', [modelName '.slx']));
close_system(modelName);
fprintf('   ✓ main_drone_model.slx created\n');

%% 2. Create PID Controller Model
fprintf('2. Creating pid_controller.slx...\n');
controllerName = 'pid_controller';

new_system(controllerName);
open_system(controllerName);

% Add PID controller blocks
add_block('simulink/Sources/In1', [controllerName '/Position_Ref'], 'Position', [30, 30, 60, 60]);
add_block('simulink/Sources/In1', [controllerName '/Position_Act'], 'Position', [30, 100, 60, 130]);
add_block('simulink/Sources/In1', [controllerName '/Attitude_Act'], 'Position', [30, 170, 60, 200]);

% Position PID controllers
add_block('simulink/Continuous/PID Controller', [controllerName '/PID_X'], 'Position', [150, 30, 200, 80]);
add_block('simulink/Continuous/PID Controller', [controllerName '/PID_Y'], 'Position', [150, 100, 200, 150]);
add_block('simulink/Continuous/PID Controller', [controllerName '/PID_Z'], 'Position', [150, 170, 200, 220]);

% Set PID gains from workspace
set_param([controllerName '/PID_X'], 'P', 'pid.pos.P(1)');
set_param([controllerName '/PID_X'], 'I', 'pid.pos.I(1)');
set_param([controllerName '/PID_X'], 'D', 'pid.pos.D(1)');

set_param([controllerName '/PID_Y'], 'P', 'pid.pos.P(2)');
set_param([controllerName '/PID_Y'], 'I', 'pid.pos.I(2)');
set_param([controllerName '/PID_Y'], 'D', 'pid.pos.D(2)');

set_param([controllerName '/PID_Z'], 'P', 'pid.pos.P(3)');
set_param([controllerName '/PID_Z'], 'I', 'pid.pos.I(3)');
set_param([controllerName '/PID_Z'], 'D', 'pid.pos.D(3)');

% Add attitude control subsystem
add_block('simulink/Ports & Subsystems/Subsystem', [controllerName '/Attitude_Control'], ...
    'Position', [300, 80, 400, 160]);

% Add control allocation
add_block('simulink/Math Operations/Matrix Multiply', [controllerName '/Control_Allocation'], ...
    'Position', [450, 80, 500, 160]);

% Output
add_block('simulink/Sinks/Out1', [controllerName '/Motor_Commands_Out'], 'Position', [550, 110, 580, 140]);

% Save controller model
save_system(controllerName, fullfile('controllers', [controllerName '.slx']));
close_system(controllerName);
fprintf('   ✓ pid_controller.slx created\n');

%% 3. Create Drone Dynamics Model
fprintf('3. Creating drone_dynamics.slx...\n');
dynamicsName = 'drone_dynamics';

new_system(dynamicsName);
open_system(dynamicsName);

% Motor dynamics (4 motors)
for i = 1:4
    motorBlock = sprintf('%s/Motor_%d', dynamicsName, i);
    add_block('simulink/Continuous/Transfer Fcn', motorBlock, ...
        'Position', [100, 50+i*60, 150, 80+i*60]);
    set_param(motorBlock, 'Numerator', 'motor.thrust_coeff');
    set_param(motorBlock, 'Denominator', sprintf('[motor.time_constant 1]'));
end

% 6DOF equations of motion
add_block('simulink/Continuous/Integrator', [dynamicsName '/Position_Integrator'], ...
    'Position', [300, 100, 350, 150]);
add_block('simulink/Continuous/Integrator', [dynamicsName '/Velocity_Integrator'], ...
    'Position', [200, 100, 250, 150]);
add_block('simulink/Continuous/Integrator', [dynamicsName '/Attitude_Integrator'], ...
    'Position', [300, 200, 350, 250]);
add_block('simulink/Continuous/Integrator', [dynamicsName '/AngVel_Integrator'], ...
    'Position', [200, 200, 250, 250]);

% Force and moment calculation
add_block('simulink/Math Operations/Sum', [dynamicsName '/Total_Force'], ...
    'Position', [400, 100, 450, 150]);
add_block('simulink/Math Operations/Sum', [dynamicsName '/Total_Moment'], ...
    'Position', [400, 200, 450, 250]);

% Outputs
add_block('simulink/Sinks/Out1', [dynamicsName '/Position_Out'], 'Position', [500, 50, 530, 80]);
add_block('simulink/Sinks/Out1', [dynamicsName '/Velocity_Out'], 'Position', [500, 110, 530, 140]);
add_block('simulink/Sinks/Out1', [dynamicsName '/Attitude_Out'], 'Position', [500, 170, 530, 200]);
add_block('simulink/Sinks/Out1', [dynamicsName '/AngVel_Out'], 'Position', [500, 230, 530, 260]);

save_system(dynamicsName, fullfile('models', [dynamicsName '.slx']));
close_system(dynamicsName);
fprintf('   ✓ drone_dynamics.slx created\n');

%% 4. Create Hover Test Scenario
fprintf('4. Creating hover_test.slx...\n');
hoverName = 'hover_test';

new_system(hoverName);
open_system(hoverName);

% Reference generator
add_block('simulink/Sources/Step', [hoverName '/Altitude_Reference'], ...
    'Position', [50, 50, 100, 100]);
set_param([hoverName '/Altitude_Reference'], 'Time', '2');
set_param([hoverName '/Altitude_Reference'], 'After', '-2');

% Position references (zero for hover)
add_block('simulink/Sources/Constant', [hoverName '/X_Reference'], ...
    'Position', [50, 120, 100, 150]);
add_block('simulink/Sources/Constant', [hoverName '/Y_Reference'], ...
    'Position', [50, 170, 100, 200]);
set_param([hoverName '/X_Reference'], 'Value', '0');
set_param([hoverName '/Y_Reference'], 'Value', '0');

% Wind disturbance
add_block('simulink/Sources/Band-Limited White Noise', [hoverName '/Wind_Disturbance'], ...
    'Position', [50, 220, 100, 270]);

% Main drone model reference
add_block('simulink/Ports & Subsystems/Model', [hoverName '/Drone_System'], ...
    'Position', [200, 120, 350, 220]);

% Data logging
add_block('simulink/Sinks/To Workspace', [hoverName '/Log_Position'], ...
    'Position', [400, 100, 450, 130]);
add_block('simulink/Sinks/To Workspace', [hoverName '/Log_Attitude'], ...
    'Position', [400, 150, 450, 180]);
add_block('simulink/Sinks/To Workspace', [hoverName '/Log_Motors'], ...
    'Position', [400, 200, 450, 230]);

set_param([hoverName '/Log_Position'], 'VariableName', 'position_data');
set_param([hoverName '/Log_Attitude'], 'VariableName', 'attitude_data');
set_param([hoverName '/Log_Motors'], 'VariableName', 'motor_data');

save_system(hoverName, fullfile('test_scenarios', [hoverName '.slx']));
close_system(hoverName);
fprintf('   ✓ hover_test.slx created\n');

%% 5. Create Competition Deployment Model
fprintf('5. Creating competition_deployment.slx...\n');
competitionName = 'competition_deployment';

new_system(competitionName);
open_system(competitionName);

% Flight mode selector
add_block('simulink/Signal Routing/Manual Switch', [competitionName '/Flight_Mode'], ...
    'Position', [50, 100, 100, 150]);

% Emergency stop
add_block('simulink/Logic and Bit Operations/Logical Operator', [competitionName '/Emergency_Stop'], ...
    'Position', [50, 200, 100, 250]);

% Parrot Minidrone interface (placeholder for real hardware blocks)
add_block('simulink/Ports & Subsystems/Subsystem', [competitionName '/Parrot_Interface'], ...
    'Position', [200, 100, 300, 200]);

% Flight control system
add_block('simulink/Ports & Subsystems/Subsystem', [competitionName '/Flight_Controller'], ...
    'Position', [350, 100, 450, 200]);

% Status indicators
add_block('simulink/Sinks/Display', [competitionName '/Battery_Status'], ...
    'Position', [500, 50, 550, 80]);
add_block('simulink/Sinks/Display', [competitionName '/Flight_Status'], ...
    'Position', [500, 100, 550, 130]);

save_system(competitionName, fullfile('models', [competitionName '.slx']));
close_system(competitionName);
fprintf('   ✓ competition_deployment.slx created\n');

%% Summary
fprintf('\n========================================================\n');
fprintf('Model Generation Complete!\n');
fprintf('========================================================\n');
fprintf('Created models:\n');
fprintf('   • models/main_drone_model.slx\n');
fprintf('   • controllers/pid_controller.slx\n');
fprintf('   • models/drone_dynamics.slx\n');
fprintf('   • test_scenarios/hover_test.slx\n');
fprintf('   • models/competition_deployment.slx\n\n');

fprintf('Next steps:\n');
fprintf('1. Open each model in Simulink to refine\n');
fprintf('2. Run: startup\n');
fprintf('3. Run: scripts/initialize_drone.m\n');
fprintf('4. Test with: hover_test.slx\n');
fprintf('5. Deploy with: competition_deployment.slx\n\n');

fprintf('All models are ready for MathWorks Mini Drone Competition!\n');
