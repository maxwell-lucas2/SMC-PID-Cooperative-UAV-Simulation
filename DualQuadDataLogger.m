% Maxwell Lucas, University of Vermont, Honors College Thesis
% Unified DataLogger for dual-quad tethered payload simulations

classdef DualQuadDataLogger < handle

    properties
        t % Time vector

        % Quad 1
        x1;  z1;  theta1
        F1_q1;  F2_q1
        x1_des;  z1_des
        e_z1;  e_x1
        s_alt1;  s_lat1

        % Quad 2
        x2;  z2;  theta2
        F1_q2;  F2_q2
        x2_des;  z2_des
        e_z2;  e_x2
        s_alt2;  s_lat2

        % Payload
        x_mass;  z_mass
        x_mass_des;  z_mass_des

        % Tethers
        tether1_length;  tether2_length
        tether1_force;   tether2_force

        % Control effort
        F_total1;  F_total2
        tau1;      tau2

        idx;  N
    end

    methods
        function obj = DualQuadDataLogger(N, dt)
            obj.N   = N;
            obj.idx = 0;
            obj.t   = (0:N-1) * dt;

            z = zeros(N,1);
            obj.x1 = z;  obj.z1 = z;  obj.theta1 = z;
            obj.F1_q1 = z;  obj.F2_q1 = z;
            obj.x1_des = z;  obj.z1_des = z;
            obj.e_z1 = z;  obj.e_x1 = z;
            obj.s_alt1 = z;  obj.s_lat1 = z;

            obj.x2 = z;  obj.z2 = z;  obj.theta2 = z;
            obj.F1_q2 = z;  obj.F2_q2 = z;
            obj.x2_des = z;  obj.z2_des = z;
            obj.e_z2 = z;  obj.e_x2 = z;
            obj.s_alt2 = z;  obj.s_lat2 = z;

            obj.x_mass = z;  obj.z_mass = z;
            obj.x_mass_des = z;  obj.z_mass_des = z;

            obj.tether1_length = z;  obj.tether2_length = z;
            obj.tether1_force = z;   obj.tether2_force = z;

            obj.F_total1 = z;  obj.F_total2 = z;
            obj.tau1 = z;      obj.tau2 = z;
        end

        function log(obj, d)
            obj.idx = obj.idx + 1;
            i = obj.idx;

            obj.x1(i) = d.x1;  obj.z1(i) = d.z1;  obj.theta1(i) = d.theta1;
            obj.F1_q1(i) = d.F1_q1;  obj.F2_q1(i) = d.F2_q1;
            obj.x1_des(i) = d.x1_des;  obj.z1_des(i) = d.z1_des;
            obj.e_z1(i) = d.e_z1;  obj.e_x1(i) = d.e_x1;
            obj.s_alt1(i) = d.s_alt1;  obj.s_lat1(i) = d.s_lat1;
            obj.F_total1(i) = d.F_total1;  obj.tau1(i) = d.tau1;

            obj.x2(i) = d.x2;  obj.z2(i) = d.z2;  obj.theta2(i) = d.theta2;
            obj.F1_q2(i) = d.F1_q2;  obj.F2_q2(i) = d.F2_q2;
            obj.x2_des(i) = d.x2_des;  obj.z2_des(i) = d.z2_des;
            obj.e_z2(i) = d.e_z2;  obj.e_x2(i) = d.e_x2;
            obj.s_alt2(i) = d.s_alt2;  obj.s_lat2(i) = d.s_lat2;
            obj.F_total2(i) = d.F_total2;  obj.tau2(i) = d.tau2;

            obj.x_mass(i) = d.x_mass;  obj.z_mass(i) = d.z_mass;
            obj.x_mass_des(i) = d.x_mass_des;  obj.z_mass_des(i) = d.z_mass_des;

            obj.tether1_length(i) = d.tether1_length;
            obj.tether2_length(i) = d.tether2_length;
            obj.tether1_force(i)  = d.tether1_force;
            obj.tether2_force(i)  = d.tether2_force;
        end
    end
end
