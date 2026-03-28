% Maxwell Lucas, 11/20/2025
% University of Vermont, Honors College Thesis

classdef WaypointManager < handle
    % Manages waypoint navigation for trajectory following
    % Waypoint tolerance and hold time
    
    properties
        waypoints       % Array of [x, z] waypoints
        num_waypoints   % Total number of waypoints
        
        % Parameters
        tolerance       % Distance tolerance to reach waypoint [m]
        hold_time       % Time to hold at waypoint [s]
        
        current_idx     % Current waypoint index
        timer           % Time spent at current waypoint [s]
        completed       
    end
    
    properties (Dependent)
        current_waypoint    % [x; z]
        x_des               % desired x
        z_des               % desired z
        progress            % Progress through waypoints
    end
    
    methods
        function obj = WaypointManager(waypoints, params)
            
            obj.waypoints = waypoints;
            obj.num_waypoints = size(waypoints, 1);
            
            obj.tolerance = params.tolerance;
            obj.hold_time = params.hold_time;
            
            obj.current_idx = 1;
            obj.timer = 0;
            obj.completed = false;
        end
        
        % Get dependent properties
        function val = get.current_waypoint(obj)
            val = obj.waypoints(obj.current_idx, :)';
        end
        
        function val = get.x_des(obj)
            val = obj.waypoints(obj.current_idx, 1);
        end
        
        function val = get.z_des(obj)
            val = obj.waypoints(obj.current_idx, 2);
        end
        
        function val = get.progress(obj)
            val = (obj.current_idx - 1) / obj.num_waypoints;
        end
        
        function update(obj, current_pos, dt)
            % Update waypoint state based on current position

            if obj.completed
                return;
            end
            
            % Calculate distance to current waypoint
            target = obj.current_waypoint;
            distance = norm(current_pos - target);
            
            % Check if within tolerance
            if distance < obj.tolerance
                obj.timer = obj.timer + dt;
            else
                obj.timer = 0;
            end
            
            % Check if hold time satisfied
            if obj.timer >= obj.hold_time
                if obj.current_idx < obj.num_waypoints
                    obj.current_idx = obj.current_idx + 1;
                    obj.timer = 0;
                else
                    obj.completed = true;
                end
            end
        end
        
        function reset(obj)
            % Reset to first waypoint
            obj.current_idx = 1;
            obj.timer = 0;
            obj.completed = false;
        end
        
        function setWaypoints(obj, waypoints)
            % Set new waypoint list
            obj.waypoints = waypoints;
            obj.num_waypoints = size(waypoints, 1);
            obj.reset();
        end
    end
end
