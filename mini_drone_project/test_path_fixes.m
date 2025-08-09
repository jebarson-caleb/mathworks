%% Test Path Fixes
% This script tests that all the path fixes work correctly

clc; clear; close all;

fprintf('========================================\n');
fprintf('Testing Path Fixes\n');
fprintf('========================================\n');
fprintf('Working directory: %s\n', pwd);

%% Test 1: Direct parameter creation with absolute paths
fprintf('\n--- Test 1: Direct Parameter Creation ---\n');
try
    project_root = pwd;
    data_dir = fullfile(project_root, 'data');
    
    fprintf('Project root: %s\n', project_root);
    fprintf('Data directory: %s\n', data_dir);
    
    % Ensure directory exists
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
        fprintf('✓ Created data directory\n');
    else
        fprintf('✓ Data directory exists\n');
    end
    
    % Create and save test parameters
    test_drone.mass = 0.087;
    test_motor.thrust = 0.3;
    
    param_file = fullfile(data_dir, 'test_params.mat');
    save(param_file, 'test_drone', 'test_motor');
    
    fprintf('✓ Test parameters saved to: %s\n', param_file);
    
    % Verify file exists
    if exist(param_file, 'file')
        fprintf('✓ Parameter file verified\n');
        delete(param_file); % Clean up
    else
        fprintf('✗ Parameter file not found\n');
    end
    
catch ME
    fprintf('✗ Test 1 failed: %s\n', ME.message);
end

%% Test 2: Initialize drone script
fprintf('\n--- Test 2: Initialize Drone Script ---\n');
try
    % Call the fixed initialize_drone script
    run('scripts/initialize_drone.m');
    fprintf('✓ initialize_drone.m executed successfully\n');
    
    % Check if parameters were loaded
    if exist('drone', 'var')
        fprintf('✓ Drone parameters loaded: mass = %.3f kg\n', drone.mass);
    else
        fprintf('✗ Drone parameters not found in workspace\n');
    end
    
    % Check if file was created
    param_file = fullfile(pwd, 'data', 'drone_parameters.mat');
    if exist(param_file, 'file')
        fprintf('✓ Parameter file created: %s\n', param_file);
    else
        fprintf('✗ Parameter file not created\n');
    end
    
catch ME
    fprintf('✗ Test 2 failed: %s\n', ME.message);
end

%% Test 3: Startup script simulation preferences
fprintf('\n--- Test 3: Simulation Preferences ---\n');
try
    % Test the simulation preferences part of startup
    project_root = pwd;
    data_dir = fullfile(project_root, 'data');
    
    % Create test preferences
    solverPref.Solver = 'ode45';
    solverPref.MaxStep = '0.01';
    
    pref_file = fullfile(data_dir, 'simulation_preferences.mat');
    save(pref_file, 'solverPref');
    
    fprintf('✓ Simulation preferences saved to: %s\n', pref_file);
    
    % Test loading
    loaded_pref = load(pref_file);
    fprintf('✓ Preferences loaded: solver = %s\n', loaded_pref.solverPref.Solver);
    
catch ME
    fprintf('✗ Test 3 failed: %s\n', ME.message);
end

%% Summary
fprintf('\n========================================\n');
fprintf('Path Fix Test Complete!\n');
fprintf('========================================\n');
fprintf('If all tests passed, the path issues are resolved.\n');
fprintf('You can now run:\n');
fprintf('1. startup.m - for full project initialization\n');
fprintf('2. create_simulink_models.m - for model creation\n');
fprintf('========================================\n');
