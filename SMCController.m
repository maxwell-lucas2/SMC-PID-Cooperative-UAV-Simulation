% Maxwell Lucas, 11/20/2025
% University of Vermont, Honors College Thesis

classdef SMCController < handle
    % Sliding Mode Controller with saturation
    % Implements SMC with boundary layer for chattering reduction
    
    properties
        % Controller gains
        lambda % Sliding surface slope
        eta % Switching gain
        phi % Boundary layer thickness
        
        % State tracking
        s_prev % Previous sliding surface value
        s_current % Current sliding surface value
        
        % Controller type identifier for M vs J
        type % 'altitude', 'lateral', 'attitude'
    end
    
    methods
        function obj = SMCController(type, params)
            
            obj.type = type;
            obj.lambda = params.lambda;
            obj.eta = params.eta;
            obj.phi = params.phi;
            
            obj.s_prev = 0;
            obj.s_current = 0;
        end
        
        function [u, s] = compute(obj, e, e_dot, dd_des)
            % Calculate control output
            dd_des = 0;
            
            % Sliding surface
            s = e_dot + obj.lambda * e;
            
            % Equivalent control 
            u_eq = dd_des + obj.lambda * e_dot;
            
            % Switching control with saturation 
            if abs(s) <= obj.phi
                sat_s = s / obj.phi;  
            else
                sat_s = sign(s);      
            end
            
            u_sw = obj.eta * sat_s;
            
            % Total control
            u = u_eq + u_sw;
            
            % Update stored values
            obj.s_prev = obj.s_current;
            obj.s_current = s;
        end
        
        
        function setGains(obj, lambda, eta, phi)
            % Update controller gains
            
            if nargin >= 2 && ~isempty(lambda)
                obj.lambda = lambda;
            end
            if nargin >= 3 && ~isempty(eta)
                obj.eta = eta;
            end
            if nargin >= 4 && ~isempty(phi)
                obj.phi = phi;
            end
        end
    end
end
