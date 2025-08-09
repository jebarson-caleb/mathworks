%% Test Parameter Initialization
% This script tests the drone parameter initialization function

fprintf('==============================================\n');
fprintf('Testing Drone Parameter Initialization\n');
fprintf('==============================================\n');

% Clear workspace
clear all;

% Change to project directory
project_dir = '/home/jebarson/Documents/mathworks/mini_drone_project';
if exist(project_dir, 'dir')
    cd(project_dir);
    fprintf('✓ Changed to project directory: %s\n', pwd);
else
    fprintf('✗ Project directory not found: %s\n', project_dir);
    return;
end

% Test directory creation
fprintf('\nTesting directory creation...\n');
required_dirs = {'data', 'models', 'scripts', 'controllers', 'subsystems', 'test_scenarios'};
for i = 1:length(required_dirs)
    if ~exist(required_dirs{i}, 'dir')
        mkdir(required_dirs{i});
        fprintf('✓ Created directory: %s\n', required_dirs{i});
    else
        fprintf('✓ Directory exists: %s\n', required_dirs{i});
    end
end

% Test parameter initialization
fprintf('\nTesting parameter initialization...\n');
try
    addpath('scripts');
    initDroneParameters();
    fprintf('✓ Parameter initialization successful!\n');
    
    % Verify parameter file exists
    param_file = fullfile('data', 'drone_parameters.mat');
    if exist(param_file, 'file')
        fprintf('✓ Parameter file created: %s\n', param_file);
        
        % Load and verify parameters
        loaded_params = load(param_file);
        fprintf('✓ Parameter file loaded successfully\n');
        fprintf('  - Drone mass: %.3f kg\n', loaded_params.drone.mass);
        fprintf('  - Motor max thrust: %.2f N\n', loaded_params.motor.max_thrust);
        fprintf('  - Battery capacity: %d mAh\n', loaded_params.battery.capacity);
        
    else
        fprintf('✗ Parameter file not found: %s\n', param_file);
    end
    
catch ME
    fprintf('✗ Parameter initialization failed: %s\n', ME.message);
    fprintf('Error details:\n');
    disp(ME);
end

fprintf('\n==============================================\n');
fprintf('Test Complete\n');
fprintf('==============================================\n');
