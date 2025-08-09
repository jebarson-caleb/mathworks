%% Simplified Startup Script for Testing
% This is a minimal version to test the core initialization

fprintf('=== Simplified Startup Test ===\n');
fprintf('Date: %s\n', datestr(now));
fprintf('Working directory: %s\n', pwd);

%% Step 1: Create directories
fprintf('\n1. Creating directories...\n');
required_dirs = {'data', 'models', 'scripts', 'controllers', 'subsystems', 'test_scenarios'};

for i = 1:length(required_dirs)
    dir_name = required_dirs{i};
    if ~exist(dir_name, 'dir')
        try
            mkdir(dir_name);
            fprintf('  ✓ Created: %s\n', dir_name);
        catch ME
            fprintf('  ✗ Failed to create %s: %s\n', dir_name, ME.message);
        end
    else
        fprintf('  ✓ Exists: %s\n', dir_name);
    end
end

%% Step 2: Test parameter initialization
fprintf('\n2. Testing parameter initialization...\n');
try
    % Ensure data directory exists
    if ~exist('data', 'dir')
        mkdir('data');
    end
    
    % Create simple parameters
    drone.mass = 0.087;
    drone.arm_length = 0.046;
    motor.max_thrust = 0.3;
    battery.capacity = 1100;
    
    % Test saving
    param_file = fullfile('data', 'test_params.mat');
    save(param_file, 'drone', 'motor', 'battery');
    
    fprintf('  ✓ Successfully saved parameters to: %s\n', param_file);
    
    % Test loading
    loaded = load(param_file);
    fprintf('  ✓ Successfully loaded parameters\n');
    fprintf('    - Drone mass: %.3f kg\n', loaded.drone.mass);
    fprintf('    - Motor thrust: %.1f N\n', loaded.motor.max_thrust);
    
    % Clean up test file
    delete(param_file);
    
catch ME
    fprintf('  ✗ Parameter test failed: %s\n', ME.message);
end

%% Step 3: Test full initialization function
fprintf('\n3. Testing initDroneParameters function...\n');
try
    addpath('scripts');
    initDroneParameters();
    fprintf('  ✓ initDroneParameters executed successfully\n');
catch ME
    fprintf('  ✗ initDroneParameters failed: %s\n', ME.message);
    fprintf('     Error details: %s\n', ME.getReport());
end

fprintf('\n=== Simplified Startup Complete ===\n');
