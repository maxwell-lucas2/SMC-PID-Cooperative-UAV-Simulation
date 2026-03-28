% Maxwell Lucas, 11/20/2025
% University of Vermont, Honors College Thesis

classdef VFormationPlanner < handle
    %   Given desired mass position, calculates where each quad should be
    
    properties
        % Formation geometry
        half_angle      % Half angle of V from vertical [rad]
        tether_length   % Tether length [m]
        
        % Number of quads
        num_quads
    end
    
    properties (Dependent)
        full_angle      % Full V angle [rad]
        separation      % Horizontal separation between quads [m]
    end
    
    methods
        function obj = VFormationPlanner(params)
            
            obj.half_angle = deg2rad(params.half_angle_deg);
            obj.tether_length = params.tether_length;
            obj.num_quads = params.num_quads;
        end
        
        % Get dependent properties
        function val = get.full_angle(obj)
            val = 2 * obj.half_angle;
        end
        
        function val = get.separation(obj)
            val = 2 * obj.tether_length * sin(obj.half_angle);
        end
        
        function positions = computeQuadPositions(obj, mass_pos)
            % Calculate quad positions for V-formation
            
            x_mass = mass_pos(1);
            z_mass = mass_pos(2);
            
            positions = cell(obj.num_quads, 1);
            
            
            % V-formation with 2 quads
            % Quad 1 
            positions{1} = [x_mass - obj.tether_length * sin(obj.half_angle);
                            z_mass + obj.tether_length * cos(obj.half_angle)];
                
            % Quad 2 
            positions{2} = [x_mass + obj.tether_length * sin(obj.half_angle);
                            z_mass + obj.tether_length * cos(obj.half_angle)];
            
                
        end
        
        function [x_des, z_des] = getQuadSetpoint(obj, quad_idx, mass_pos)
            % Get setpoint for each quad
            
            positions = obj.computeQuadPositions(mass_pos);
            pos = positions{quad_idx};
            x_des = pos(1);
            z_des = pos(2);
        end
    end
end
