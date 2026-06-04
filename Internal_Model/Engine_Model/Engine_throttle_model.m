function [F_thrust, engine_is_lit] = Engine_throttle_model(Fmax, throttle, min_throttle, engine_is_lit)
% Basic Model of the engines response to throttle commands
    if engine_is_lit == true
        if throttle < min_throttle
            % if engine is throttled below 
            F_thrust = 0.0;  
            engine_is_lit = false;
        else
            % linear throttling, this will be replaced later once data is obtained
            F_thrust = Fmax * throttle;
        end 
    else
        % if engine isnt lit, output no thrust
        F_thrust = 0.0;
    end
end