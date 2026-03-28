% Maxwell Lucas, 11/20/2025
% University of Vermont, Honors College Thesis

classdef TetheredMass < handle
    % Class for a point mass with tether attachment
    
    properties
        m  % [kg]
        g  % Gravity [m/s*s]
        
        % State vector [x, dx, z, dz]
        state

        z_min   % Minimum altitude for ground plane [m]
    end
    
    properties (Dependent)
        x % [m]
        dx % [m/s]
        z % [m]
        dz % [m/s]
        position % [x; z] 
        velocity % [dx; dz]
    end
    
    methods
        function obj = TetheredMass(params)
            
            obj.m = params.m;
            obj.g = params.g;
            obj.state = params.initial_state(:);
            

            obj.z_min = 0.05;
            
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
        
        function val = get.position(obj)
            val = [obj.state(1); obj.state(3)];
        end
        
        function val = get.velocity(obj)
            val = [obj.state(2); obj.state(4)];
        end
        
        function dstate = dynamics(obj, state, F_external)
            % Compute derivatives
            
            Fx = F_external(1);
            Fz = F_external(2) - obj.m * obj.g;
            
            ddx = Fx / obj.m;
            ddz = Fz / obj.m;
            
            dstate = [state(2); ddx; state(4); ddz];
        end
        
        function update(obj, F_external, dt)
            % Integrate with RK4
            
            k1 = obj.dynamics(obj.state, F_external);
            k2 = obj.dynamics(obj.state + 0.5*dt*k1, F_external);
            k3 = obj.dynamics(obj.state + 0.5*dt*k2, F_external);
            k4 = obj.dynamics(obj.state + dt*k3, F_external);
            
            obj.state = obj.state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
            
            % Ground constraint
            obj.applyGroundConstraint();
        end
        
        function applyGroundConstraint(obj)
            % Stop mass from going below ground
            
            if obj.state(3) < obj.z_min
                obj.state(3) = obj.z_min;
                obj.state(4) = max(0, obj.state(4));
            end
        end
    end
end
