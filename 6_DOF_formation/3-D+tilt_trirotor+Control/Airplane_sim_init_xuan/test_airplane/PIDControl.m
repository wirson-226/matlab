classdef PIDControl
    properties
        kp         % Proportional gain
        ki         % Integral gain
        kd         % Derivative gain
        Ts         % Sampling time
        limit      % Control output saturation limit
        integrator % Integrator state
        error_delay_1 % Error delayed by one time step
        error_dot_delay_1 % Derivative error delayed by one time step
        a1          % Differentiator filter coefficient
        a2          % Differentiator filter coefficient
    end
    
    methods
        % Constructor
        function obj = PIDControl(kp, ki, kd, Ts, sigma, limit)
            if nargin > 0
                obj.kp = kp;
                obj.ki = ki;
                obj.kd = kd;
                obj.Ts = Ts;
                obj.limit = limit;
                obj.integrator = 0;
                obj.error_delay_1 = 0;
                obj.error_dot_delay_1 = 0;
                % Initialize differentiator filter coefficients
                obj.a1 = (2*sigma - Ts) / (2*sigma + Ts);
                obj.a2 = 2 / (2*sigma + Ts);
            end
        end
        
        % Update function for PID control
        function [u_sat, obj] = update(obj, y_ref, y, reset_flag)
            if nargin < 3
                reset_flag = false;
            end
            
            if reset_flag
                % Reset the internal states
                obj.integrator = 0;
                obj.error_delay_1 = 0;
                obj.error_dot_delay_1 = 0;
            end
            
            % Compute error
            error = y_ref - y;
            
            % Update the integrator using trapezoidal rule
            obj.integrator = obj.integrator + (obj.Ts / 2) * (error + obj.error_delay_1);
            
            % Compute derivative using low-pass filter
            error_dot = obj.a1 * obj.error_dot_delay_1 + obj.a2 * (error - obj.error_delay_1);
            
            % Compute the PID control output
            u = obj.kp * error + obj.ki * obj.integrator + obj.kd * error_dot;
            
            % Saturate the control output
            u_sat = obj.saturate(u);
            
            % Integral anti-windup
            if abs(obj.ki) > 1e-4
                obj.integrator = obj.integrator + (obj.Ts / obj.ki) * (u_sat - u);
            end
            
            % Update the delayed error and error_dot
            obj.error_delay_1 = error;
            obj.error_dot_delay_1 = error_dot;
        end
        
        % Update function with rate of change of output (for rate-based control)
        function [u_sat, obj] = updateWithRate(obj, y_ref, y, ydot, reset_flag)
            if nargin < 4
                reset_flag = false;
            end
            
            if reset_flag
                % Reset the internal states
                obj.integrator = 0;
                obj.error_delay_1 = 0;
            end
            
            % Compute error
            error = y_ref - y;
            
            % Update the integrator using trapezoidal rule
            obj.integrator = obj.integrator + (obj.Ts / 2) * (error + obj.error_delay_1);
            
            % Compute the PID control output
            u = obj.kp * error + obj.ki * obj.integrator - obj.kd * ydot;
            
            % Saturate the control output
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
            % Saturate the control output to within [-limit, limit]
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
