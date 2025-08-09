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
addpath(genpath(project_root));

% Add specific folders to path
addpath(fullfile(project_root, 'models'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'data'));
addpath(fullfile(project_root, 'controllers'));
addpath(fullfile(project_root, 'subsystems'));
addpath(fullfile(project_root, 'test_scenarios'));

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
run('initialize_drone.m');

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

for i = 1:length(required_toolboxes)
    if license('test', strrep(required_toolboxes{i}, ' ', '_'))
        fprintf('✓ %s - Available\n', required_toolboxes{i});
    else
        fprintf('✗ %s - Not Available\n', required_toolboxes{i});
    end
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
Simulink.sdi.setSubPlotLayout(2,2);
Simulink.sdi.setAutoArrangeSubPlots(true);

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
