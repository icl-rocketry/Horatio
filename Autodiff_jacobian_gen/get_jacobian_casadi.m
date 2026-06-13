clc;
clear;

% Import casADI
import casadi.*

% Setup sim environment
nx = 2;
nu = 2;
% ...

% Create symbolic primatives
x = SX.sym('x', nx, 1);
u = SX.sym('u', nu, 1);

% Setup computation expression
f = dyn_fn(x, u); % change once dynamics function is finished

% Compute Jacobians
J_x = jacobian(f, x);
J_u = jacobian(f, u);

% Check matrices
disp(J_x)
disp(J_u)

% Save to C encoded format to remove casADI dependencies
get_jacobians = Function('get_jacobians', {x, u}, {f, J_x, J_u});
opts = struct('mex', true);
get_jacobians.generate('Continuous_Linear_mats.c', opts);

% example function. Delete later
function x_dot = dyn_fn(x, u)
    x_dot1 = 2 * x .^ 2;
    x_dot2 = 8 * u .^ 2;
    x_dot = [x_dot1; x_dot2];
end