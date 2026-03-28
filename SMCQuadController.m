% Maxwell Lucas — University of Vermont, Honors College Thesis
% SMC Controller wrapper for dual-quad tethered payload system

classdef SMCQuadController < handle

    properties
        smc_alt     % Altitude SMC
        smc_lat     % Lateral SMC
        smc_att     % Attitude SMC

        quad        % Quadcopter reference

        theta_max           % Max pitch [rad]
        use_feedforward     % Enable tether FF

        F_tether_z  % Vertical tether force for FF

        % Sliding surface values
        s_alt
        s_lat
        s_att
    end

    methods
        function obj = SMCQuadController(quad, smc_params)

            obj.quad = quad;

            obj.smc_alt = SMCController('altitude', smc_params.alt);
            obj.smc_lat = SMCController('lateral',  smc_params.lat);
            obj.smc_att = SMCController('attitude', smc_params.att);

            obj.theta_max = pi/4;
            obj.use_feedforward = true;
            obj.F_tether_z = 0;

            obj.s_alt = 0;
            obj.s_lat = 0;
            obj.s_att = 0;
        end

        function setTetherForce(obj, F_tether)
            obj.F_tether_z = F_tether(2);
        end

        function [F1, F2, info] = computeThrust(obj, x_des, z_des, ~)

            x = obj.quad.x;
            dx = obj.quad.dx;
            z = obj.quad.z;
            dz = obj.quad.dz;
            theta = obj.quad.theta;
            dtheta = obj.quad.dtheta;

            % Altitude SMC
            e_z = z_des - z;
            e_dot_z = 0 - dz;
            [a_z_cmd, obj.s_alt] = obj.smc_alt.compute(e_z, e_dot_z, 0);

            if obj.use_feedforward
                a_z_cmd = a_z_cmd - obj.F_tether_z / obj.quad.m;
            end

            % Lateral SMC
            e_x = x_des - x;
            e_dot_x = 0 - dx;
            [a_x_cmd, obj.s_lat] = obj.smc_lat.compute(e_x, e_dot_x, 0);

            % Force mapping
            F_des_x   = obj.quad.m * a_x_cmd;
            F_des_z   = obj.quad.m * (a_z_cmd + obj.quad.g);
            theta_cmd = atan2(-F_des_x, F_des_z);
            theta_cmd = max(-obj.theta_max, min(obj.theta_max, theta_cmd));
            F_total   = sqrt(F_des_x^2 + F_des_z^2);

            % Attitude SMC
            e_theta     = theta_cmd - theta;
            e_dot_theta = 0 - dtheta;
            [ddtheta_cmd, obj.s_att] = obj.smc_att.compute(e_theta, e_dot_theta, 0);
            tau = obj.quad.J * ddtheta_cmd;

            % Motor mixing
            F1 = max(0, 0.5 * (F_total + tau / obj.quad.L));
            F2 = max(0, 0.5 * (F_total - tau / obj.quad.L));

            % Info for logging
            info.F_total = F_total;
            info.tau = tau;
            info.e_z = e_z;
            info.e_x = e_x;
            info.e_theta = e_theta;
            info.theta_cmd = theta_cmd;
            info.s_alt = obj.s_alt;
            info.s_lat = obj.s_lat;
            info.s_att = obj.s_att;
        end
    end
end
