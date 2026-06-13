clear
clc

%tuning variables
%bounds and log vs integer all arbitrary for now
tuning_variables = [ ...
    optimizableVariable('terminal_pos_error',   [1e1, 1e5], 'Transform','log'); ...
    optimizableVariable('terminal_vel_error', [1e-2, 1e2],'Transform','log'); ...
    optimizableVariable('terminal_quat_error',   [1e-1, 1e3],'Transform','log'); ...
    optimizableVariable('terminal_rotrate_error',   [1e-1, 1e3],'Transform','log'); ...
    optimizableVariable('terminal_mass_error',      [10, 40], 'Type','integer'); ...
    optimizableVariable('gridfin_cost',      [10, 40], 'Type','integer');
    optimizableVariable('TVC_cost',      [10, 40], 'Type','integer');
    optimizableVariable('w_ep',      [10, 40], 'Type','integer');
    optimizableVariable('w_prox',      [10, 40], 'Type','integer');
    ];

%range of ics
% need to define variable values
n_ic = 10;
%nominal ic
rx_ic_nom = 0.0; ry_ic_nom = 0.0; rz_ic_nom = 3000;
vx_ic_nom = 0.0; vy_ic_nom = 0.0; vz_ic_nom = 0.0;
qw_ic_nom = 1.0; qx_ic_nom = 0.0; qy_ic_nom = 0.0; qz_ic_nom = 0.0;
wx_ic_nom = 0.0; wy_ic_nom = 0.0; wz_ic_nom = 0.0;
m_ic_nom = 500.0; 
%ic deviation max magnitudes
rx_dev = 10.0; ry_dev = 10.0; rz_dev = 10.0;
vx_dev = 1.0; vy_dev = 1.0; vz_dev = 1.0;
qw_dev = 0.01; qx_dev = 0.05; qy_dev = 0.05; qz_dev = 0.05;
wx_dev = 0.01; wy_dev = 0.01; wz_dev = 0.01;
m_dev = 10.0;
x_ic_nominal = [rx_ic_nom; ry_ic_nom; rz_ic_nom; vx_ic_nom; vy_ic_nom; vz_ic_nom; qw_ic_nom; qx_ic_nom; qy_ic_nom; qz_ic_nom; wx_ic_nom; wy_ic_nom; wz_ic_nom; m_ic_nom];
x_max_deviation = [rx_dev; ry_dev; rz_dev; vx_dev; vy_dev; vz_dev; qw_dev; qx_dev; qy_dev; qz_dev; wx_dev; wy_dev; wz_dev; m_dev];
%perturb to define matrix of initial conditions
rng('default')
ic_matrix = x_ic_nominal + (2*rand(14, n_ic) - 1) .* x_max_deviation;
ic_matrix(7:10,:) = ic_matrix(7:10,:) ./ vecnorm(ic_matrix(7:10,:)); %renormalise quaternions

%Objective function setup:
cost_vector = zeros(n_ic, 1);
rx_land_nom = 0.0; ry_land_nom = 0.0; rz_land_nom = 0.0;
vx_land_nom = 0.0; vy_land_nom = 0.0; vz_land_nom = 0.0;
qw_land_nom = 1.0; qx_land_nom = 0.0; qy_land_nom = 0.0; qz_land_nom = 0.0;
wx_land_nom = 0.0; wy_land_nom = 0.0; wz_land_nom = 0.0;
m_land_nom = 500.0;
x_landing_nominal = [rx_land_nom; ry_land_nom; rz_land_nom; vx_land_nom; vy_land_nom; vz_land_nom; qw_land_nom; qx_land_nom; qy_land_nom; qz_land_nom; wx_land_nom; wy_land_nom; wz_land_nom; m_land_nom]; 

%define hard constraint boundary values 
r_xy_crit = 5;
r_z_crit = 0.5;
w_x_crit = 0.05;
w_zy_crit = 0.01;
tilt_crit = 5; 
n_constraints = 5; 
constraint_matrix = zeros(n_constraints, n_ic);

%put all key parameters in a struct to pass to objective function
parameters.ic_matrix         = ic_matrix;
parameters.x_landing_nominal = x_landing_nominal;
parameters.n_ic              = n_ic;
parameters.n_constraints     = n_constraints;
parameters.r_xy_crit = r_xy_crit;  parameters.r_z_crit  = r_z_crit;
parameters.tilt_crit = tilt_crit;  parameters.w_x_crit  = w_x_crit;  parameters.w_zy_crit = w_zy_crit;

function [objective, constraint_vector] = objectivefun(tuning_variables, parameters)

    error_vector = zeros(14,1);
    cost_vector = zeros(parameters.n_ic, 1);
    constraint_matrix = zeros(parameters.n_constraints, parameters.n_ic);

    %run the sim with params and across range of ic
    for i = 1:parameters.n_ic
        try
            %run the sim with given tuning variables and parameters, SAM
            x_landing = 

            %find error_vector vector
            %scalars:
            error_vector(1:6) = x_landing(1:6) - parameters.x_landing_nominal(1:6);
            error_vector(11:14) = x_landing(11:14) - parameters.x_landing_nominal(11:14);
            %quaternions: 
            error_vector(7:10) = %quaternion error_vector, SAM script - translate q error_vector to degrees? - easier to tune weights - 
            %add some way of finding tilt from vertical via quaternion error_vector
            tilt = 
            
            %evaluate scalar cost:
            cost_vector(i) = norm(error_vector(4:6)); %magnitude of velocity error_vector
            %evaluate constraint vector corresponding to ic 
            constraint_matrix(:,i) = [norm(error_vector(1:2)) - parameters.r_xy_crit;... %x-y plane constraint
                                            abs(error_vector(3)) - parameters.r_z_crit; ... %z constraint
                                            abs(tilt) - parameters.tilt_crit;... %attitude constraint
                                            abs(error_vector(11)) - parameters.w_x_crit; ... %roll constraint
                                            norm(error_vector(12:13)) - parameters.w_zy_crit]; %yaw/pitch combined constraint  

        catch
            cost_vector(i) = NaN; %specifically an error rather than just big cost
            constraint_matrix(:,i) = 1e2 * ones(parameters.n_constraints,1);  % mark infeasible
        end                            
    end

    %cost  is average across all ic:
    objective = mean(cost_vector);
    %constraint values are max across all ic:
    constraint_vector = max(constraint_matrix, [], 2);
end

%running BO:
results = bayesopt(@(tuning_variables) objectivefun(tuning_variables, parameters), tuning_variables, ...
    'NumCoupledConstraints', n_constraints, ...
    'AreCoupledConstraintsDeterministic', true(1,n_constraints), ...
    'AcquisitionFunctionName','expected-improvement-plus', ...
    'NumSeedPoints',          10, ...     
    'MaxObjectiveEvaluations',60, ...
    'IsObjectiveDeterministic', true);
%NumSeedPoints and MaxObjectiveEvaluations arbitrary for now

bestParams = bestPoint(results);