classdef PIControl
    properties
        kp           % Proportional gain
        ki           % Integral gain
        Ts           % Sampling time
        limit        % Saturation limit
        integrator   % Integrator state
        error_delay_1 % Delayed error from previous step
    end
    
    methods
        % Constructor
        function obj = PIControl(kp, ki, Ts, limit)
            if nargin > 0
                obj.kp = kp;
                obj.ki = ki;
                obj.Ts = Ts;
                obj.limit = limit;
                obj.integrator = 0.0;
                obj.error_delay_1 = 0.0;
            end
        end
        
        % Update function for PI control
        function u_sat = update(obj, y_ref, y)
            % Compute the error
            error = y_ref - y;
            
            % Update the integrator using trapezoidal rule
            obj.integrator = obj.integrator + (obj.Ts / 2) * (error + obj.error_delay_1);
            
            % PI control
            u = obj.kp * error + obj.ki * obj.integrator;
            
            % Saturate the PI control output
            u_sat = obj.saturate(u);
            
            % Integral anti-windup
            if abs(obj.ki) > 1e-4
                obj.integrator = obj.integrator + (obj.Ts / obj.ki) * (u_sat - u);
            end
            
            % Update the delayed error
            obj.error_delay_1 = error;
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
