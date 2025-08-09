%% Mini Drone Project Startup Script
% This script initializes the workspace for the mini drone Simulink project
% Run this script before opening any Simulink models

clc; clear; close all;

%% Project Information
fprintf('==========================================\n');
fprintf('Mini Drone Simulink Project Initialization\n');
fprintf('==========================================\n');
fprintf('Date: %s\n', datestr(now));
fprintf('Project Version: 1.0\n\n');

%% Set Project Paths
project_root = pwd;

% Create required directories if they don't exist
required_dirs = {'data', 'models', 'scripts', 'controllers', 'subsystems', 'test_scenarios'};
fprintf('Checking and creating required directories...\n');
for i = 1:length(required_dirs)
    if ~exist(required_dirs{i}, 'dir')
        mkdir(required_dirs{i});
        fprintf('Created directory: %s\n', required_dirs{i});
    end
end

addpath(genpath(project_root));

% Add specific folders to path (only if they exist)
folders_to_add = {'models', 'scripts', 'data', 'controllers', 'subsystems', 'test_scenarios'};
for i = 1:length(folders_to_add)
    folder_path = fullfile(project_root, folders_to_add{i});
    if exist(folder_path, 'dir')
        addpath(folder_path);
    else
        fprintf('Note: Directory %s does not exist, skipping.\n', folders_to_add{i});
    end
end

fprintf('Project paths added successfully.\n');

%% Load Simulink Project (if available)
try
    % Check if we're in a directory with a .prj file
    prj_files = dir('*.prj');
    if ~isempty(prj_files)
        project_obj = simulinkproject(pwd);
        fprintf('Simulink project loaded successfully.\n');
    else
        fprintf('No Simulink project file found. Running in standalone mode.\n');
    end
catch ME
    if contains(ME.message, 'simulinkproject')
        fprintf('Simulink project functionality not available. Running in standalone mode.\n');
    else
        fprintf('Project loading failed: %s\n', ME.message);
    end
    fprintf('This won''t affect the core functionality.\n');
end

%% Initialize Drone Parameters
fprintf('Initializing drone parameters...\n');
try
    % Use the robust initialization function
    initDroneParameters();
    fprintf('✓ Drone parameters initialized successfully!\n');
catch ME
    fprintf('✗ Parameter initialization failed: %s\n', ME.message);
    % Try fallback initialization
    fprintf('Attempting fallback initialization...\n');
    try
        % Ensure data directory exists
        if ~exist('data', 'dir')
            mkdir('data');
        end
        
        % Run the original initialization script
        run('initialize_drone.m');
        fprintf('✓ Fallback initialization successful!\n');
    catch ME2
        fprintf('✗ Fallback initialization also failed: %s\n', ME2.message);
        fprintf('You may need to manually create the data directory and check file permissions.\n');
    end
end

%% Set Simulink Preferences (with error handling)
try
    % Set character encoding if parameter exists
    if isprop(get_param(0, 'ObjectParameters'), 'CharacterEncoding')
        set_param(0, 'CharacterEncoding', 'UTF-8');
    end
    
    % Set data tips if parameter exists (newer MATLAB versions)
    if isprop(get_param(0, 'ObjectParameters'), 'LineBranchDataTip')
        set_param(0, 'LineBranchDataTip', 'on');
    end
    
    % Alternative for older versions
    try
        set_param(0, 'DataTipDisplay', 'on');
    catch
        % Parameter doesn't exist in this version, continue
    end
    
    fprintf('Simulink preferences configured successfully.\n');
catch ME
    fprintf('Warning: Some Simulink preferences could not be set: %s\n', ME.message);
    fprintf('This is normal for some MATLAB versions and won''t affect functionality.\n');
end

%% Load Required Toolboxes Check
fprintf('Checking required toolboxes...\n');
required_toolboxes = {
    'Simulink',
    'Aerospace Blockset',
    'Control System Toolbox',
    'DSP System Toolbox',
    'Simulink Control Design'
};

missing_toolboxes = {};
for i = 1:length(required_toolboxes)
    if license('test', strrep(required_toolboxes{i}, ' ', '_'))
        fprintf('✓ %s - Available\n', required_toolboxes{i});
    else
        fprintf('✗ %s - Not Available\n', required_toolboxes{i});
        missing_toolboxes{end+1} = required_toolboxes{i};
    end
end

% Provide guidance for missing toolboxes
if ~isempty(missing_toolboxes)
    fprintf('\nNote: Some advanced features may not be available without:\n');
    for i = 1:length(missing_toolboxes)
        switch missing_toolboxes{i}
            case 'Control System Toolbox'
                fprintf('  - %s: LQR controller will use basic implementation\n', missing_toolboxes{i});
            case 'DSP System Toolbox'
                fprintf('  - %s: Using basic filter implementations\n', missing_toolboxes{i});
            otherwise
                fprintf('  - %s\n', missing_toolboxes{i});
        end
    end
    fprintf('Basic drone simulation will still work with available toolboxes.\n');
end

%% Set up model callbacks and preferences
fprintf('\nSetting up Simulink environment...\n');

% Configure solver preferences
solverPref.solver = 'ode45';
solverPref.StopTime = '30';
solverPref.MaxStep = '0.01';
solverPref.RelTol = '1e-6';

% Save preferences
save('data/simulation_preferences.mat', 'solverPref');

%% Create data logging setup
fprintf('Setting up data logging configuration...\n');

% Signal logging configuration
try
    % Check if Simulation Data Inspector is available
    if exist('Simulink.sdi.setSubPlotLayout', 'file')
        Simulink.sdi.setSubPlotLayout(2,2);
        Simulink.sdi.setAutoArrangeSubPlots(true);
        fprintf('Data logging configured successfully.\n');
    else
        fprintf('Simulation Data Inspector not available. Using basic logging.\n');
        % Use basic Simulink logging instead
        try
            set_param(0, 'DataLogging', 'on');
            set_param(0, 'DataLoggingToFile', 'on');
            fprintf('Basic data logging configured.\n');
        catch
            fprintf('Using default logging settings.\n');
        end
    end
catch ME
    fprintf('Data logging setup failed: %s\n', ME.message);
    fprintf('Continuing with default logging settings.\n');
end

%% Display completion message
fprintf('\n==========================================\n');
fprintf('Initialization Complete!\n');
fprintf('==========================================\n');
fprintf('You can now:\n');
fprintf('1. Open main_drone_model.slx for simulation\n');
fprintf('2. Explore controller models in controllers/\n');
fprintf('3. Run test scenarios from test_scenarios/\n');
fprintf('4. Modify parameters using scripts/control_tuning.m\n\n');

%% Open main model (optional - uncomment if desired)
% fprintf('Opening main drone model...\n');
% open_system('models/main_drone_model');

fprintf('Project ready for simulation!\n');
