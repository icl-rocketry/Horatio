function Mach_num = get_mach(alt, velocity)
% obtains the mach number of the rocket from the altitude and velocity. 
    % define constants
    gamma = 1.4;
    R = 287.1;

    % get atmospheric parameters
    [~, T, ~] = standard_atm_function(alt);

    % get mach number
    Mach_num = velocity / sqrt(gamma * R * T);
end