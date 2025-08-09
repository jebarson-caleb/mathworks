%% Competition Model Validation Script
% Validates all models and configurations for MathWorks Mini Drone Competition

clc; clear; close all;

fprintf('========================================================\n');
fprintf('MathWorks Mini Drone Competition - Model Validation\n');
fprintf('========================================================\n\n');

%% Test 1: Parameter Loading
fprintf('TEST 1: Parameter Loading\n');
fprintf('-------------------------\n');

try
    run('scripts/initialize_drone.m');
    fprintf('✓ Drone parameters loaded successfully\n');
    fprintf('  - Mass: %.3f kg\n', drone.mass);
    fprintf('  - Arm length: %.3f m\n', drone.arm_length);
    fprintf('  - Max thrust: %.2f N per motor\n', motor.max_thrust);
    test1_pass = true;
catch ME
    fprintf('✗ Parameter loading failed: %s\n', ME.message);
    test1_pass = false;
end

%% Test 2: Model File Existence
fprintf('\nTEST 2: Model File Existence\n');
fprintf('----------------------------\n');

required_models = {
    'models/main_drone_model.slx',
    'controllers/pid_controller.slx',
    'models/drone_dynamics.slx',
    'test_scenarios/hover_test.slx',
    'models/competition_deployment.slx'
};

test2_pass = true;
for i = 1:length(required_models)
    if exist(required_models{i}, 'file')
        fprintf('✓ %s exists\n', required_models{i});
    else
        fprintf('✗ %s missing\n', required_models{i});
        test2_pass = false;
    end
end

%% Test 3: Model Loading
fprintf('\nTEST 3: Model Loading\n');
fprintf('--------------------\n');

test3_pass = true;
for i = 1:length(required_models)
    try
        if exist(required_models{i}, 'file')
            load_system(required_models{i});
            fprintf('✓ %s loads successfully\n', required_models{i});
            close_system(required_models{i});
        end
    catch ME
        fprintf('✗ %s failed to load: %s\n', required_models{i}, ME.message);
        test3_pass = false;
    end
end

%% Test 4: Control System Validation
fprintf('\nTEST 4: Control System Validation\n');
fprintf('---------------------------------\n');

test4_pass = true;
try
    % Check PID gains
    if exist('pid', 'var')
        fprintf('✓ PID gains configured\n');
        fprintf('  - Position P gains: [%.1f, %.1f, %.1f]\n', pid.pos.P);
        fprintf('  - Attitude P gains: [%.1f, %.1f, %.1f]\n', pid.att.P);
    else
        fprintf('✗ PID gains not found\n');
        test4_pass = false;
    end
    
    % Check control allocation matrix
    if exist('control', 'var') && isfield(control, 'allocation_matrix')
        fprintf('✓ Control allocation matrix configured\n');
        fprintf('  - Matrix size: %dx%d\n', size(control.allocation_matrix));
    else
        fprintf('✗ Control allocation matrix not found\n');
        test4_pass = false;
    end
    
catch ME
    fprintf('✗ Control system validation failed: %s\n', ME.message);
    test4_pass = false;
end

%% Test 5: Simulation Test
fprintf('\nTEST 5: Basic Simulation Test\n');
fprintf('-----------------------------\n');

test5_pass = true;
try
    if exist('test_scenarios/hover_test.slx', 'file')
        % Set short simulation time for test
        open_system('test_scenarios/hover_test.slx');
        set_param('hover_test', 'StopTime', '5');  % 5 second test
        
        % Run simulation
        fprintf('Running 5-second hover test...\n');
        sim_output = sim('hover_test');
        
        close_system('hover_test');
        
        fprintf('✓ Hover simulation completed successfully\n');
        fprintf('  - Simulation time: %.1f seconds\n', sim_output.tout(end));
        
        % Basic performance check
        if isfield(sim_output, 'position_data')
            final_pos = sim_output.position_data.Data(end, :);
            fprintf('  - Final position: [%.2f, %.2f, %.2f] m\n', final_pos);
        end
        
    else
        fprintf('✗ Hover test model not found\n');
        test5_pass = false;
    end
    
catch ME
    fprintf('✗ Simulation test failed: %s\n', ME.message);
    test5_pass = false;
end

%% Test 6: Competition Requirements Check
fprintf('\nTEST 6: Competition Requirements\n');
fprintf('-------------------------------\n');

test6_pass = true;

% Check MathWorks competition requirements
requirements = {
    'Drone mass < 100g', drone.mass < 0.1,
    'Control frequency ≥ 50Hz', control.attitude_loop_freq >= 50,
    'Battery monitoring', exist('battery', 'var'),
    'Safety limits configured', exist('safety', 'var'),
    'Deployment model ready', exist('models/competition_deployment.slx', 'file')
};

for i = 1:2:length(requirements)
    req_name = requirements{i};
    req_met = requirements{i+1};
    
    if req_met
        fprintf('✓ %s\n', req_name);
    else
        fprintf('✗ %s\n', req_name);
        test6_pass = false;
    end
end

%% Test 7: Hover Performance Analysis
fprintf('\nTEST 7: Hover Performance Analysis\n');
fprintf('----------------------------------\n');

test7_pass = true;
try
    if exist('sim_output', 'var')
        % Calculate basic performance metrics
        if isfield(sim_output, 'position_data')
            pos_data = sim_output.position_data.Data;
            
            % Check if drone reaches target altitude
            target_altitude = -2.0;  % 2m above ground
            final_altitude = pos_data(end, 3);
            altitude_error = abs(final_altitude - target_altitude);
            
            if altitude_error < 0.2  % Within 20cm of target
                fprintf('✓ Altitude tracking: %.2f m (target: %.1f m)\n', final_altitude, target_altitude);
            else
                fprintf('✗ Poor altitude tracking: %.2f m (target: %.1f m)\n', final_altitude, target_altitude);
                test7_pass = false;
            end
            
            % Check position stability
            x_std = std(pos_data(end-100:end, 1));  % Last 100 samples
            y_std = std(pos_data(end-100:end, 2));
            
            if x_std < 0.1 && y_std < 0.1  % Within 10cm standard deviation
                fprintf('✓ Position stability: σx=%.3f m, σy=%.3f m\n', x_std, y_std);
            else
                fprintf('✗ Poor position stability: σx=%.3f m, σy=%.3f m\n', x_std, y_std);
                test7_pass = false;
            end
        else
            fprintf('✗ No position data available for analysis\n');
            test7_pass = false;
        end
    else
        fprintf('! Skipping performance analysis (no simulation data)\n');
        test7_pass = true;  % Don't fail if no sim data
    end
    
catch ME
    fprintf('✗ Performance analysis failed: %s\n', ME.message);
    test7_pass = false;
end

%% Overall Results
fprintf('\n========================================================\n');
fprintf('VALIDATION RESULTS SUMMARY\n');
fprintf('========================================================\n');

tests = {
    'Parameter Loading', test1_pass,
    'Model File Existence', test2_pass,
    'Model Loading', test3_pass,
    'Control System', test4_pass,
    'Basic Simulation', test5_pass,
    'Competition Requirements', test6_pass,
    'Performance Analysis', test7_pass
};

passed_tests = 0;
total_tests = length(tests) / 2;

for i = 1:2:length(tests)
    test_name = tests{i};
    test_result = tests{i+1};
    
    if test_result
        fprintf('✓ %s: PASS\n', test_name);
        passed_tests = passed_tests + 1;
    else
        fprintf('✗ %s: FAIL\n', test_name);
    end
end

fprintf('\nOVERALL SCORE: %d/%d tests passed (%.1f%%)\n', ...
    passed_tests, total_tests, 100*passed_tests/total_tests);

if passed_tests == total_tests
    fprintf('\n🎉 ALL TESTS PASSED! Your project is ready for competition! 🏆\n');
elseif passed_tests >= total_tests * 0.8
    fprintf('\n✅ MOSTLY READY! Minor issues to fix before competition.\n');
else
    fprintf('\n⚠️  NEEDS WORK! Please fix failed tests before proceeding.\n');
end

%% Recommendations
fprintf('\n========================================================\n');
fprintf('RECOMMENDATIONS\n');
fprintf('========================================================\n');

if ~test1_pass
    fprintf('• Fix parameter loading by running: startup\n');
end

if ~test2_pass
    fprintf('• Create missing models by running: create_simulink_models.m\n');
end

if ~test3_pass
    fprintf('• Check Simulink installation and model syntax\n');
end

if ~test4_pass
    fprintf('• Configure control system by running: scripts/control_tuning.m\n');
end

if ~test5_pass
    fprintf('• Debug simulation issues in hover_test.slx\n');
end

if ~test6_pass
    fprintf('• Review competition requirements and update configuration\n');
end

if ~test7_pass
    fprintf('• Tune control gains for better performance\n');
end

if passed_tests == total_tests
    fprintf('• Your project is competition-ready!\n');
    fprintf('• Consider running additional test scenarios\n');
    fprintf('• Test hardware deployment when ready\n');
end

fprintf('\n========================================================\n');
fprintf('Next steps: Follow IMPLEMENTATION_GUIDE.md for details\n');
fprintf('========================================================\n');
