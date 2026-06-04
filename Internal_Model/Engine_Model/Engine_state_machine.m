classdef Engine_state_machine < matlab.System
% State machine for rocket engine

    % Public, tunable properties
    properties(DiscreteState)
        phase;
        t_ignition;
        has_relit;
    end

    % Pre-computed constants or internal states
    properties (Constant)
        T_min = 100;
        t_spool_req = 2.5;
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Phase = 1; % set to initial phase in config
            obj.t_ignition = 0;
        end

        function [Engine_Phase, t_ignition] = stepImpl(obj, t, u, Engine_trigger)
            
            % State Machine Logic
            switch obj.Phase
                case 1 % Unlit
                    % Engine triggered
                    if Engine_trigger == 1
                        obj.Phase = 2;
                        obj.t_ignition = t;
                    else
                        obj.phase = 1;
                    end
                    
                case 2 % Relight
                    % Engine finished spool up
                    if (t - obj.t_ignition) >= obj.t_spool_req
                        obj.Phase = 3;
                    else 
                        obj.Phase = 2;
                    end
                    
                case 3 % Lit
                    % Flameout
                    if u < obj.u_min
                        obj.Phase = 1;
                    else
                        obj.Phase = 3;
                    end
            end
            
            % Output phase
            Engine_Phase = obj.Phase;
            t_ignition = obj.t_ignition;
        end
    end
end
