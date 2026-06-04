function [Mass_ROC] = Mass_change_rate(Thrust, Isp, g)
% Obtain mass ROC wrt time.
    % Obtains the mass flow rate of the engine
    Mass_ROC = - Thrust / (Isp * g);
end