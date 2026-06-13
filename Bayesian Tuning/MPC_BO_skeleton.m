clear
clc

%tuning variable
%bounds and log vs integer all arbitrary for now
vars = [ ...
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
rx_nom = 0.0;
ry_nom = 0.0;
rz_nom = 0.0;
v_x_nom = 0.0;
v_y_nom = 0.0;
v_z_nom = 0.0;
q_w_nom = 1.0; 
q_x_nom = 0.0;
q_y_nom = 0.0;
q_z_nom = 0.0;
w_x_nom = 0.0;
w_y_nom = 0.0;
w_z_nom = 0.0;
m_nom = 500.0; 

rx_dev = 10.0;
ry_dev = 10.0;
rz_dev = 10.0;
v_x_dev = 1.0;
v_y_dev = 1.0;
v_z_dev = 1.0;
q_w_dev = 0.01;
q_x_dev = 0.05;
q_y_dev = 0.05;
q_z_dev = 0.05;
w_x_dev = 0.01;
w_y_dev = 0.01;
w_z_dev = 0.01;
m_dev = 5.0;
x_ic_nominal = [rx_ic_nom; ry_ic_nom; rz_ic_nom; vx_ic_nom; vy_ic_nom; vz_ic_nom; qw_ic_nom; qx_ic_nom; qy_ic_nom; qz_ic_nom; wx_ic_nom; wy_ic_nom; wz_ic_nom; m_ic_nom];
x_max_deviation = [rx_dev; ry_dev; rz_dev; v_x_dev; v_y_dev; v_z_dev; q_w_dev; q_x_dev; q_y_dev; q_z_dev; w_x_dev; w_y_dev; w_z_dev; m_dev];
%perturb to define matrix of initial conditions
ic_matrix = x_ic_nominal + (2*rand(14, n_ic) - 1) .* x_max_deviation;
ic_matrix(7:10,:) = ic_matrix(7:10,:) ./ vecnorm(ic_matrix(7:10,:)); %renormalise quaternions?

%Objective function
%x_landing_matrix = zeros(14, n_ic);

cost_vector = zeros(n_ic, 1);
x_landing_nominal = [rx_land_nom; ry_land_nom; rz_land_nom; vx_land_nom; vy_land_nom; vz_land_nom; qw_land_nom; qx_land_nom; qy_land_nom; qz_land_nom; wx_land_nom; wy_land_nom; wz_land_nom; m_land_nom];
error = zeros(14,1);
x_landing = zeros(14,1);

function objective = objectivefun(params)
    %pass the tuning params into simulink
    % SAM

    %run the sim with params and across range of ic
    for i = 1:n_ic

        x_ic = ic_matrix(:,i);
        try
            %run the sim, SAM
        catch
            objective = 1e6;   % large finite penalty for crash - what value?
            return
        end
    
        %Extract landing position from simulink run, SAM 
        x_landing = zeros(14,1);

        %find error
        %scalars:
        error(1:6) = abs(x_landing(1:6) - x_landing_nominal(1:6));
        error(11:14) = abs(x_landing(11:14) - x_landing_nominal(11:14));
        %quaternions: 
        error(7:10) = %quaternion error, SAM script - translate q error to degrees? - easier to tune weights

        %normalise error vector into scalar cost




    end

    %cost is average 

    
 
end


%NumSeedPoints and MaxObjectiveEvaluations arbitrary for now
%deterministic depends whether we vary ic each time - look into this
%running BO:
results = bayesopt(@objectivefun, vars, ...
    'AcquisitionFunctionName','expected-improvement-plus', ...
    'NumSeedPoints',          10, ...     
    'MaxObjectiveEvaluations',60, ...
    'IsObjectiveDeterministic', true);

bestParams = results.XAtMinObjective;