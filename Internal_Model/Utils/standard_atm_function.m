function [rho, T, P] = standard_atm_function(altitude)
% Uses standard atmospheric relation to calculate the density of air at the current altitude

    % constants from literature
    g = 9.81;
    lambda = -0.0065;
    P0 = 101325;
    T0 = 288.15;
    rho0 = 1.225;
    R = 287.15;
    
    % using relations to obtain density, pressure and temperature
    T = T0 + lambda * altitude;
    P = P0 * (T0 / (T0 + lambda * altitude)) ^ (g / (R * lambda));
    rho = (T0 / T) * (P / P0) * rho0;
end