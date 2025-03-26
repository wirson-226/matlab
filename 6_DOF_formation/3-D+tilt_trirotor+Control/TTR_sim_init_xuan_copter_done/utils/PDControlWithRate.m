classdef PDControlWithRate
    properties
        kp      % Proportional gain
        kd      % Derivative gain
        limit   % Saturation limit
    end
    
    methods
        % Constructor
        function obj = PDControlWithRate(kp, kd, limit)
            if nargin > 0
                obj.kp = kp;
                obj.kd = kd;
                obj.limit = limit;
            end
        end
        
        % Update function for PD control with rate
        function u_sat = update(obj, y_ref, y, ydot)
            % PD control law: u = kp*(yref - y) - kd*ydot
            u = obj.kp * (y_ref - y) - obj.kd * ydot;
            
            % Saturate the control output
            u_sat = obj.saturate(u);
        end
        
        % Saturation function for control output
        function u_sat = saturate(obj, u)
            % Saturate u at +/- limit
            if u >= obj.limit
                u_sat = obj.limit;
            elseif u <= -obj.limit
                u_sat = -obj.limit;
            else
                u_sat = u;
            end
        end
    end
end
