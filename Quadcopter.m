% Maxwell Lucas, 11/20/2025
% University of Vermont, Honors College Thesis

classdef Quadcopter < handle
    % Planar quadcopter class
    
    properties
        m  % Mass [kg]
        J  % Moment of inertia [kg*m^2]
        L  % Arm length [m]
        g  % Gravity [m/s^2]
        
        % State vector [x, dx, z, dz, theta, dtheta]
        state

        id
    end
    
    properties (Dependent)
        x % [m]
        dx % [m/s]
        z % [m]
        dz % [m/s]
        theta % [rad]
        dtheta % [rad/s]
        position % [x; z] 
        velocity % [dx; dz] 
    end
    
    methods
        function obj = Quadcopter(id, params)
            
            obj.id = id;
            obj.m = params.m;
            obj.J = params.J;
            obj.L = params.L;
            obj.g = params.g;
            obj.state = params.initial_state(:);
        end
        
        % Get dependent properties
        function val = get.x(obj)
            val = obj.state(1);
        end
        
        function val = get.dx(obj)
            val = obj.state(2);
        end
        
        function val = get.z(obj)
            val = obj.state(3);
        end
        
        function val = get.dz(obj)
            val = obj.state(4);
        end
        
        function val = get.theta(obj)
            val = obj.state(5);
        end
        
        function val = get.dtheta(obj)
            val = obj.state(6);
        end
        
        function val = get.position(obj)
            val = [obj.state(1); obj.state(3)];
        end
        
        function val = get.velocity(obj)
            val = [obj.state(2); obj.state(4)];
        end
        
        function dstate = dynamics(obj, state, F1, F2, F_external)
            % Compute state derivatives
            
            theta_val = state(5);
            dtheta_val = state(6);
            
            F_thrust = F1 + F2;
            
            % Forces in world frame
            Fx = -F_thrust * sin(theta_val) + F_external(1);
            Fz = F_thrust * cos(theta_val) - obj.m * obj.g + F_external(2);
            
            % Torque from differential thrust
            tau = (F1 - F2) * obj.L;
            
            % Accelerations
            ddx = Fx / obj.m;
            ddz = Fz / obj.m;
            ddtheta = tau / obj.J;
            
            dstate = [state(2); ddx; state(4); ddz; dtheta_val; ddtheta];
        end
        
        function update(obj, F1, F2, F_external, dt)
            % Integrate dynamics using RK4
            k1 = obj.dynamics(obj.state, F1, F2, F_external);
            k2 = obj.dynamics(obj.state + 0.5*dt*k1, F1, F2, F_external);
            k3 = obj.dynamics(obj.state + 0.5*dt*k2, F1, F2, F_external);
            k4 = obj.dynamics(obj.state + dt*k3, F1, F2, F_external);
            
            obj.state = obj.state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        end
        
        function [x_body, z_body] = getBodyPoints(obj)
            
            x_body = obj.x + [-obj.L*cos(obj.theta), obj.L*cos(obj.theta)];
            z_body = obj.z + [-obj.L*sin(obj.theta), obj.L*sin(obj.theta)];
        end
    end
end
