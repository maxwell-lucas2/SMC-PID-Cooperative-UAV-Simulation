% Maxwell Lucas, 11/19/2025
% University of Vermont, Honors College Thesis

classdef PIDController < handle
    % PID control with integral clamping
    
    properties
        % Controller gains
        Kp % Proportional gain
        Ki % Integral gain
        Kd % Derivative gain
        
        % Anti-windup
        int_max    
        
        integral % Accumulated integral
        
        % Controller type
        type % 'altitude', 'lateral', 'attitude'
    end
    
    methods
        function obj = PIDController(type, params)
            
            obj.type = type;
            obj.Kp = params.Kp;
            obj.Ki = params.Ki;
            obj.Kd = params.Kd;
            obj.int_max = params.int_max;
            
            obj.integral = 0;
        end
        
        function u = compute(obj, error, derivative, dt)
            
            % Update integral with anti-windup
            obj.integral = obj.integral + error * dt;
            obj.integral = max(-obj.int_max, min(obj.int_max, obj.integral));
            
            % PID output
            u = obj.Kp * error + obj.Ki * obj.integral - obj.Kd * derivative;
        end
        
        function u = computeWithDerivative(obj, error, error_dot, dt)
            
            % Update integral with anti-windup
            obj.integral = obj.integral + error * dt;
            obj.integral = max(-obj.int_max, min(obj.int_max, obj.integral));
            
            % PID output
            u = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * error_dot;
        end
        
        function reset(obj)
            % Reset controller state
            obj.integral = 0;
        end
        
        function setGains(obj, Kp, Ki, Kd, int_max)
            
            if nargin >= 2 && ~isempty(Kp)
                obj.Kp = Kp;
            end
            if nargin >= 3 && ~isempty(Ki)
                obj.Ki = Ki;
            end
            if nargin >= 4 && ~isempty(Kd)
                obj.Kd = Kd;
            end
            if nargin >= 5 && ~isempty(int_max)
                obj.int_max = int_max;
            end
        end
    end
end
