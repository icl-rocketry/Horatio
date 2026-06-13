clc;
clear;

% c-compiled file
binary_file = pwd;

% Add path to search path
addpath(binary_file);
fprintf('Successfully linked the compiled binaries from: %s\n', binary_file);

% Evaluation ponts
x_current = [pi * 2; 2.0];
u_current = [pi * 6, 3.0];

% Evaluate matrices
try
    % Call c file casADI function
    [f0_sparse, J_x_sparse, J_u_sparse] = Continuous_Linear_mats(x_current, u_current);
catch ME
    error('Execution Fault: The binary eval_vehicle_jacobians could not be executed. Verify the MEX file exists for your OS architecture.');
end

% Outputs
J_x = full(J_x_sparse);
J_u = full(J_u_sparse);
f0 = full(f0_sparse);

% Display the exact evaluated derivative matrices
fprintf(' Evaluated State Jacobian (J_x) \n');
disp(J_x);

fprintf(' Evaluated Control Jacobian (J_u) \n');
disp(J_u);

fprintf(' Evaluated function (f0) \n');
disp(f0);