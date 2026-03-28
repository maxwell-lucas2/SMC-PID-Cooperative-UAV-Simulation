% Maxwell Lucas — University of Vermont, Honors College Thesis
% PID Controller wrapper for dual-quad tethered payload system

classdef PIDQuadController < handle

    properties
        pid_alt % Altitude PID
        pid_lat % Lateral PID
        pid_att % Attitude PID

        quad % Quadcopter

        theta_max % Max pitch angle [rad]
        use_feedforward % Enable tether FF

        F_tether_z % Vertical tether force for FF

        % Safety limits
        a_z_max      
        a_x_max           
        ff_max             
    end

    methods
        function obj = PIDQuadController(quad, pid_params)

            obj.quad = quad;

            obj.pid_alt = PIDController('altitude', pid_params.alt);
            obj.pid_lat = PIDController('lateral',  pid_params.lat);
            obj.pid_att = PIDController('attitude', pid_params.att);

            obj.theta_max = pi/4;
            obj.use_feedforward = true;
            obj.F_tether_z = 0;

            % Acceleration limits 
            obj.a_z_max = 20.0;  % [m/s^2]
            obj.a_x_max = 15.0;  % [m/s^2]
            obj.ff_max  = 15.0;  % [m/s^2] 
        end

        function setTetherForce(obj, F_tether)
            obj.F_tether_z = F_tether(2);
        end

        function [F1, F2, info] = computeThrust(obj, x_des, z_des, dt)

            x      = obj.quad.x;
            dx     = obj.quad.dx;
            z      = obj.quad.z;
            dz     = obj.quad.dz;
            theta  = obj.quad.theta;
            dtheta = obj.quad.dtheta;

            % Altitude PID
            e_z = z_des - z;
            a_z_cmd = obj.pid_alt.compute(e_z, dz, dt);
            a_z_cmd = max(-obj.a_z_max, min(obj.a_z_max, a_z_cmd));

            % Feed-forward with saturation
            if obj.use_feedforward
                a_z_ff  = -obj.F_tether_z / obj.quad.m;
                a_z_ff  = max(-obj.ff_max, min(obj.ff_max, a_z_ff));
                a_z_cmd = a_z_cmd + a_z_ff;
            end

            % Lateral PID
            e_x = x_des - x;
            a_x_cmd = obj.pid_lat.compute(e_x, dx, dt);
            a_x_cmd = max(-obj.a_x_max, min(obj.a_x_max, a_x_cmd));

            % Force mapping
            F_des_x   = obj.quad.m * a_x_cmd;
            F_des_z   = obj.quad.m * (a_z_cmd + obj.quad.g);
            theta_cmd = atan2(-F_des_x, F_des_z);
            theta_cmd = max(-obj.theta_max, min(obj.theta_max, theta_cmd));
            F_total   = sqrt(F_des_x^2 + F_des_z^2);

            % Attitude PID
            e_theta = theta_cmd - theta;
            tau     = obj.pid_att.compute(e_theta, dtheta, dt);

            % Motor mixing
            F1 = max(0, 0.5 * (F_total + tau / obj.quad.L));
            F2 = max(0, 0.5 * (F_total - tau / obj.quad.L));

            % Logging
            info.F_total   = F_total;
            info.tau       = tau;
            info.e_z       = e_z;
            info.e_x       = e_x;
            info.e_theta   = e_theta;
            info.theta_cmd = theta_cmd;

            info.s_alt = 0;
            info.s_lat = 0;
            info.s_att = 0;
        end

        function reset(obj)
            obj.pid_alt.reset();
            obj.pid_lat.reset();
            obj.pid_att.reset();
            obj.F_tether_z = 0;
        end
    end
end
