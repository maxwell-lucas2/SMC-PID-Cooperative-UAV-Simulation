% Maxwell Lucas, 11/20/2025
% University of Vermont, Honors College Thesis

classdef Tether < handle
    % Class for spring-damper tether connection
    % Computes forces between two attachment points
    
    properties
        k       % Spring constant [N/m]
        c       % Damping coefficient [N*s/m]
        L0      % Natural length [m]
        
        current_length      % Current tether length [m]
        current_force_mag   % Current force magnitude [N]
        
        id
    end
    
    properties (Dependent)
        stretch     % Current stretch amount [m]
        is_taut     % If tether is under tension
    end
    
    methods
        function obj = Tether(id, params)
            
            obj.id = id;
            obj.k = params.k;
            obj.c = params.c;
            obj.L0 = params.L0;
            
            obj.current_length = params.L0;
            obj.current_force_mag = 0;
        end
        
        % Get dependent properties
        function val = get.stretch(obj)
            val = max(0, obj.current_length - obj.L0);
        end
        
        function val = get.is_taut(obj)
            val = obj.current_length > obj.L0;
        end
        
        function [F_point1, F_point2, length, force_mag] = computeForces(obj, pos1, vel1, pos2, vel2)
            % Calculate tether forces on both attachment points

            % Vector from quad to mass
            r = pos1 - pos2;
            L = norm(r);
            
            % Store length
            obj.current_length = L;
            
            % Apply force if stretched
            if L > obj.L0 && L > 1e-6

                % Unit vector along tether
                n = r / L;
                
                % Stretch amount
                stretch_val = L - obj.L0;
                
                % Relative velocity along tether
                v_rel = vel1 - vel2;
                v_rel_tether = dot(v_rel, n);
                
                % Spring-damper force magnitude
                F_mag = obj.k * stretch_val + obj.c * v_rel_tether;
                F_mag = max(0, F_mag); 
                
                % Store force
                obj.current_force_mag = F_mag;
                
                % Get forces for int 
                F_point1 = -F_mag * n;  % Force on quad
                F_point2 = F_mag * n;   % Force on mass 
                
                force_mag = F_mag;
            else
                % Slack tether
                F_point1 = [0; 0];
                F_point2 = [0; 0];
                force_mag = 0;
                obj.current_force_mag = 0;
                obj.current_length = max(L, obj.L0);
            end
            
            length = obj.current_length;
        end
    end
end
