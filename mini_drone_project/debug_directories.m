%% Debug Directory Creation
% Simple test to verify directory creation works

fprintf('=== Debug Directory Creation ===\n');
fprintf('Current directory: %s\n', pwd);

% Test directory creation
test_dirs = {'data', 'models', 'scripts'};

for i = 1:length(test_dirs)
    dir_name = test_dirs{i};
    fprintf('\nTesting directory: %s\n', dir_name);
    
    % Check if exists
    if exist(dir_name, 'dir')
        fprintf('  ✓ Directory exists\n');
    else
        fprintf('  ✗ Directory does not exist\n');
        
        % Try to create
        try
            mkdir(dir_name);
            fprintf('  ✓ Successfully created directory\n');
        catch ME
            fprintf('  ✗ Failed to create directory: %s\n', ME.message);
        end
    end
    
    % Verify creation
    if exist(dir_name, 'dir')
        fprintf('  ✓ Final check: Directory exists\n');
        
        % Test file creation in directory
        test_file = fullfile(dir_name, 'test.txt');
        try
            fid = fopen(test_file, 'w');
            if fid > 0
                fprintf(fid, 'Test file');
                fclose(fid);
                fprintf('  ✓ Can write files to directory\n');
                delete(test_file); % Clean up
            else
                fprintf('  ✗ Cannot write files to directory\n');
            end
        catch ME
            fprintf('  ✗ File write test failed: %s\n', ME.message);
        end
    else
        fprintf('  ✗ Final check: Directory still does not exist\n');
    end
end

fprintf('\n=== Test Complete ===\n');
