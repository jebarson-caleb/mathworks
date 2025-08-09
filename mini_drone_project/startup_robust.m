function startup_robust()
%STARTUP_ROBUST Robust startup function with comprehensive error handling
%   This function handles all initialization tasks with proper error handling

fprintf('==============================================\n');
fprintf('Mini Drone Simulink Project - Robust Startup\n');
fprintf('==============================================\n');
fprintf('Date: %s\n', datestr(now));
fprintf('MATLAB Version: %s\n', version);
fprintf('Working Directory: %s\n', pwd);
fprintf('Platform: %s\n', computer);

%% Step 1: Verify and Create Project Structure
fprintf('\n--- Step 1: Project Structure ---\n');
project_root = pwd;
required_dirs = {'data', 'models', 'scripts', 'controllers', 'subsystems', 'test_scenarios'};

for i = 1:length(required_dirs)
    dir_path = fullfile(project_root, required_dirs{i});
    fprintf('Checking: %s\n', dir_path);
    
    if ~exist(dir_path, 'dir')
        try
            [success, msg, msgid] = mkdir(dir_path);
            if success
                fprintf('  ✓ Created directory: %s\n', required_dirs{i});
            else
                fprintf('  ✗ Failed to create %s: %s (ID: %s)\n', required_dirs{i}, msg, msgid);
            end
        catch ME
            fprintf('  ✗ Exception creating %s: %s\n', required_dirs{i}, ME.message);
        end
    else
        fprintf('  ✓ Directory exists: %s\n', required_dirs{i});
    end
    
    % Verify directory is accessible
    if exist(dir_path, 'dir')
        try
            % Test write permission by creating a temporary file
            temp_file = fullfile(dir_path, '.test_write_permission');
            fid = fopen(temp_file, 'w');
            if fid > 0
                fclose(fid);
                delete(temp_file);
                fprintf('  ✓ Write permission confirmed for %s\n', required_dirs{i});
            else
                fprintf('  ✗ No write permission for %s\n', required_dirs{i});
            end
        catch ME
            fprintf('  ✗ Write permission test failed for %s: %s\n', required_dirs{i}, ME.message);
        end
    end
end

%% Step 2: Add Paths
fprintf('\n--- Step 2: Adding Paths ---\n');
try
    addpath(genpath(project_root));
    fprintf('✓ Added project paths recursively\n');
catch ME
    fprintf('✗ Failed to add paths: %s\n', ME.message);
end

%% Step 3: Initialize Parameters
fprintf('\n--- Step 3: Parameter Initialization ---\n');

% Method 1: Try the robust initialization function
try
    if exist(fullfile('scripts', 'initDroneParameters.m'), 'file')
        addpath('scripts');
        initDroneParameters();
        fprintf('✓ Parameters initialized using initDroneParameters\n');
        return; % Success, exit function
    else
        fprintf('! initDroneParameters.m not found\n');
    end
catch ME
    fprintf('✗ initDroneParameters failed: %s\n', ME.message);
end

% Method 2: Direct parameter creation and saving
fprintf('Trying direct parameter creation...\n');
try
    % Ensure data directory exists
    data_dir = fullfile(project_root, 'data');
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
    end
    
    % Create basic parameters
    drone = struct();
    drone.mass = 0.087;
    drone.arm_length = 0.046;
    
    motor = struct();
    motor.max_thrust = 0.3;
    motor.time_constant = 0.02;
    
    battery = struct();
    battery.capacity = 1100;
    battery.voltage_nominal = 3.7;
    
    env = struct();
    env.gravity = 9.81;
    
    % Save using absolute path
    param_file = fullfile(data_dir, 'drone_parameters.mat');
    save(param_file, 'drone', 'motor', 'battery', 'env');
    
    fprintf('✓ Basic parameters created and saved to: %s\n', param_file);
    
    % Load to verify
    test_load = load(param_file);
    fprintf('✓ Parameter file verified: mass = %.3f kg\n', test_load.drone.mass);
    
catch ME
    fprintf('✗ Direct parameter creation failed: %s\n', ME.message);
    fprintf('Full error: %s\n', ME.getReport());
end

%% Step 4: Configuration
fprintf('\n--- Step 4: Basic Configuration ---\n');
try
    % Set some basic Simulink preferences if available
    if exist('simulink', 'file')
        fprintf('✓ Simulink available\n');
        
        % Try to set basic preferences
        try
            set_param(0, 'DefaultBlockFontName', 'Arial');
            fprintf('✓ Simulink preferences configured\n');
        catch
            fprintf('! Some Simulink preferences not available\n');
        end
    else
        fprintf('! Simulink not available\n');
    end
catch ME
    fprintf('✗ Configuration failed: %s\n', ME.message);
end

fprintf('\n==============================================\n');
fprintf('Robust Startup Complete!\n');
fprintf('==============================================\n');
fprintf('Next steps:\n');
fprintf('1. Run: load(''data/drone_parameters.mat'')\n');
fprintf('2. Check: dir(''data'')\n');
fprintf('3. Test: create_simulink_models\n');
fprintf('==============================================\n');

end
