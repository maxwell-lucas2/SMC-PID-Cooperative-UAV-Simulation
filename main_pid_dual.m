%% Dual Quadcopter V-Configuration Tethered Payload with PID Control
%  Maxwell Lucas, University of Vermont, Honors College Thesis
clear; close all; clc;

%% Parameters

sim.dt    = 0.001;
sim.t_end = 25;
sim.t     = 0:sim.dt:sim.t_end;
sim.N     = length(sim.t);

% Payload waypoints
waypoints = [0  1.0;
             0  1.5;
             1  2.0;
            -1  2.5;
             0  3.0;
             0  2.5];

wp_params.tolerance = 0.15; % [m]
wp_params.hold_time = 1.0; % [s]
waypoint_mgr = WaypointManager(waypoints, wp_params);

% V-Formation
formation_params.half_angle_deg = 30;
formation_params.tether_length  = 1.0;
formation_params.num_quads      = 2;
formation = VFormationPlanner(formation_params);

% Quadcopter parameters 
quad_params.m = 0.5; % Mass [kg]
quad_params.J = 0.01; % Moment of inertia [kg*m^2]
quad_params.L = 0.25; % Arm length [m]
quad_params.g = 9.81; % Gravity [m/s^2]

mass_init_pos  = [waypoints(1,1); waypoints(1,2)];
init_positions = formation.computeQuadPositions(mass_init_pos);

quad_params.initial_state = [init_positions{1}(1); 0; init_positions{1}(2); 0; 0; 0];
quad1 = Quadcopter(1, quad_params);

quad_params.initial_state = [init_positions{2}(1); 0; init_positions{2}(2); 0; 0; 0];
quad2 = Quadcopter(2, quad_params);

% Tethered payload 
mass_params.m = 0.15; % Payload mass [kg]
mass_params.g = 9.81;
mass_params.initial_state = [mass_init_pos(1); 0; mass_init_pos(2); 0];
mass_params.z_min = 0.05;
payload = TetheredMass(mass_params);

% Tether parameters
tether_params.k  = 80; % [N/m]
tether_params.c  = 5.0; % [N*s/m]
tether_params.L0 = 1.0; % [m]
tether1 = Tether(1, tether_params);
tether2 = Tether(2, tether_params);

%% PID gains
pid_params.alt = struct('Kp', 20.0, 'Ki', 1.0, 'Kd', 10.0, 'int_max', 5);
pid_params.lat = struct('Kp',  4.0, 'Ki', 0.3, 'Kd',  5.0, 'int_max', 2);
pid_params.att = struct('Kp', 30.0, 'Ki', 0.0, 'Kd', 10.0, 'int_max', 10);

controller1 = PIDQuadController(quad1, pid_params);
controller2 = PIDQuadController(quad2, pid_params);

%%  Data logging
logger = DualQuadDataLogger(sim.N, sim.dt);

%% MAIN SIMULATION LOOP
for i = 1:sim.N

    % Waypoint management 
    waypoint_mgr.update(payload.position, sim.dt);
    x_mass_des = waypoint_mgr.x_des;
    z_mass_des = waypoint_mgr.z_des;

    % V-formation setpoints
    [x1_des, z1_des] = formation.getQuadSetpoint(1, [x_mass_des; z_mass_des]);
    [x2_des, z2_des] = formation.getQuadSetpoint(2, [x_mass_des; z_mass_des]);

    % Tether forces
    [F_quad1, F_mass1, L1, F1_mag] = tether1.computeForces(quad1.position, quad1.velocity, payload.position, payload.velocity);
    [F_quad2, F_mass2, L2, F2_mag] = tether2.computeForces(quad2.position, quad2.velocity, payload.position, payload.velocity);
    F_mass_total = F_mass1 + F_mass2;

    % Feed-forward tether compensation
    controller1.setTetherForce(F_quad1);
    controller2.setTetherForce(F_quad2);

    % Compute control
    [F1_q1, F2_q1, info1] = controller1.computeThrust(x1_des, z1_des, sim.dt);
    [F1_q2, F2_q2, info2] = controller2.computeThrust(x2_des, z2_des, sim.dt);

    % Log
    d = struct();
    d.x1 = quad1.x;  d.z1 = quad1.z;  d.theta1 = quad1.theta;
    d.F1_q1 = F1_q1;  d.F2_q1 = F2_q1;
    d.x1_des = x1_des;  d.z1_des = z1_des;
    d.e_z1 = info1.e_z;  d.e_x1 = info1.e_x;
    d.s_alt1 = info1.s_alt;  d.s_lat1 = info1.s_lat;
    d.F_total1 = info1.F_total;  d.tau1 = info1.tau;

    d.x2 = quad2.x;  d.z2 = quad2.z;  d.theta2 = quad2.theta;
    d.F1_q2 = F1_q2;  d.F2_q2 = F2_q2;
    d.x2_des = x2_des;  d.z2_des = z2_des;
    d.e_z2 = info2.e_z;  d.e_x2 = info2.e_x;
    d.s_alt2 = info2.s_alt;  d.s_lat2 = info2.s_lat;
    d.F_total2 = info2.F_total;  d.tau2 = info2.tau;

    d.x_mass = payload.x;  d.z_mass = payload.z;
    d.x_mass_des = x_mass_des;  d.z_mass_des = z_mass_des;
    d.tether1_length = L1;  d.tether2_length = L2;
    d.tether1_force = F1_mag;  d.tether2_force = F2_mag;
    logger.log(d);

    % Integrate dynamics
    quad1.update(F1_q1, F2_q1, F_quad1, sim.dt);
    quad2.update(F1_q2, F2_q2, F_quad2, sim.dt);
    payload.update(F_mass_total, sim.dt);
end

%% Metrics
t = logger.t;
N = sim.N;

fprintf('\nDual-Quad PID Metrics\n');

% Payload tracking RMSE
rmse_z_mass = sqrt(mean((logger.z_mass_des - logger.z_mass).^2));
rmse_x_mass = sqrt(mean((logger.x_mass_des - logger.x_mass).^2));
fprintf('Payload altitude RMSE: %.4f m\n', rmse_z_mass);
fprintf('Payload lateral  RMSE: %.4f m\n', rmse_x_mass);

% Steady-state error
ss = t >= (sim.t_end - 2);
ss_err_z = mean(abs(logger.z_mass_des(ss) - logger.z_mass(ss)));
ss_err_x = mean(abs(logger.x_mass_des(ss) - logger.x_mass(ss)));
fprintf('Payload altitude SS err: %.4f m\n', ss_err_z);
fprintf('Payload lateral  SS err: %.4f m\n', ss_err_x);

% Max payload tracking error
max_err_z = max(abs(logger.z_mass_des - logger.z_mass));
max_err_x = max(abs(logger.x_mass_des - logger.x_mass));
fprintf('Max payload alt error:   %.4f m\n', max_err_z);
fprintf('Max payload lat error:   %.4f m\n', max_err_x);

% Total control effort
effort_q1 = trapz(t, logger.F1_q1 + logger.F2_q1);
effort_q2 = trapz(t, logger.F1_q2 + logger.F2_q2);
total_effort = effort_q1 + effort_q2;
fprintf('Total control effort:    %.2f N·s\n', total_effort);

% Thrust smoothness 
TV_q1 = sum(abs(diff(logger.F1_q1))) + sum(abs(diff(logger.F2_q1)));
TV_q2 = sum(abs(diff(logger.F1_q2))) + sum(abs(diff(logger.F2_q2)));
fprintf('Thrust TV (Q1): %.2f N\n', TV_q1);
fprintf('Thrust TV (Q2): %.2f N\n', TV_q2);

% Tether statistics
fprintf('Max tether 1 force: %.2f N\n', max(logger.tether1_force));
fprintf('Max tether 2 force: %.2f N\n', max(logger.tether2_force));
fprintf('Mean tether 1 force: %.2f N\n', mean(logger.tether1_force));
fprintf('Mean tether 2 force: %.2f N\n', mean(logger.tether2_force));

% Max pitch
fprintf('Max |pitch| Q1: %.2f deg\n', max(abs(rad2deg(logger.theta1))));
fprintf('Max |pitch| Q2: %.2f deg\n', max(abs(rad2deg(logger.theta2))));

% Quad tracking RMSE
rmse_z_q1 = sqrt(mean((logger.z1_des - logger.z1).^2));
rmse_x_q1 = sqrt(mean((logger.x1_des - logger.x1).^2));
rmse_z_q2 = sqrt(mean((logger.z2_des - logger.z2).^2));
rmse_x_q2 = sqrt(mean((logger.x2_des - logger.x2).^2));
fprintf('Quad 1 RMSE (z/x): %.4f / %.4f m\n', rmse_z_q1, rmse_x_q1);
fprintf('Quad 2 RMSE (z/x): %.4f / %.4f m\n', rmse_z_q2, rmse_x_q2);

%% Plots

% Tracking performance
figure('Position', [50 50 1400 900], 'Name', 'PID Dual-Quad Tracking');

subplot(3,2,1);
plot(t, logger.z_mass, 'b-', 'LineWidth', 1.5); hold on;
plot(t, logger.z_mass_des, 'r--', 'LineWidth', 1.2);
plot(t, logger.z1, 'g-', 'LineWidth', 0.8);
plot(t, logger.z2, 'm-', 'LineWidth', 0.8);
grid on; xlabel('Time [s]'); ylabel('Altitude [m]');
title('Altitude Tracking');
legend('Payload','Reference','Quad 1','Quad 2','Location','best');

subplot(3,2,2);
plot(t, logger.x_mass, 'b-', 'LineWidth', 1.5); hold on;
plot(t, logger.x_mass_des, 'r--', 'LineWidth', 1.2);
plot(t, logger.x1, 'g-', 'LineWidth', 0.8);
plot(t, logger.x2, 'm-', 'LineWidth', 0.8);
grid on; xlabel('Time [s]'); ylabel('Position [m]');
title('Lateral Position');
legend('Payload','Reference','Quad 1','Quad 2','Location','best');

subplot(3,2,3);
plot(t, logger.z_mass_des - logger.z_mass, 'b-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Error [m]');
title('Payload Altitude Error'); yline(0,'k--');

subplot(3,2,4);
plot(t, logger.x_mass_des - logger.x_mass, 'b-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Error [m]');
title('Payload Lateral Error'); yline(0,'k--');

subplot(3,2,5);
plot(t, rad2deg(logger.theta1), 'g-', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(logger.theta2), 'm-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Pitch Angles'); legend('Quad 1','Quad 2','Location','best');

subplot(3,2,6);
plot(t, logger.tether1_force, 'g-', 'LineWidth', 1.2); hold on;
plot(t, logger.tether2_force, 'm-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Force [N]');
title('Tether Forces'); legend('Tether 1','Tether 2','Location','best');

sgtitle('PID Controller — Dual-Quad Tracking Performance', 'FontSize', 14, 'FontWeight', 'bold');

% Control inputs
figure('Position', [100 100 1200 500], 'Name', 'PID Dual-Quad Control');

subplot(1,3,1);
plot(t, logger.F1_q1 + logger.F2_q1, 'g-', 'LineWidth', 1.2); hold on;
plot(t, logger.F1_q2 + logger.F2_q2, 'm-', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Total Thrust [N]');
title('Total Thrust per Quad'); legend('Quad 1','Quad 2');

subplot(1,3,2);
plot(t, logger.tau1, 'g-', 'LineWidth', 1.2); hold on;
plot(t, logger.tau2, 'm-', 'LineWidth', 1.2);
yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Torque [N·m]');
title('Control Torque'); legend('Quad 1','Quad 2');

subplot(1,3,3);
cum_q1 = cumtrapz(t, logger.F1_q1 + logger.F2_q1);
cum_q2 = cumtrapz(t, logger.F1_q2 + logger.F2_q2);
plot(t, cum_q1, 'g-', 'LineWidth', 1.5); hold on;
plot(t, cum_q2, 'm-', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Cumulative [N·s]');
title('Cumulative Effort'); legend('Quad 1','Quad 2');

sgtitle('PID Controller — Control Inputs', 'FontSize', 14, 'FontWeight', 'bold');

%Trajectory
figure('Position', [150 150 700 600], 'Name', 'PID Dual-Quad Trajectory');
plot(logger.x1, logger.z1, 'g-', 'LineWidth', 1.5); hold on;
plot(logger.x2, logger.z2, 'm-', 'LineWidth', 1.5);
plot(logger.x_mass, logger.z_mass, 'b-', 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), 'r^', 'MarkerSize', 10, 'MarkerFaceColor','r');
plot([-3 3], [0 0], 'k-', 'LineWidth', 2);
axis equal; grid on; xlabel('X [m]'); ylabel('Z [m]');
title('PID — Flight Path (XZ Plane)', 'FontSize', 13);
legend('Quad 1','Quad 2','Payload','Waypoints','Ground','Location','best');

% Tether lengths
figure('Position', [200 200 800 400], 'Name', 'PID Tether Lengths');
plot(t, logger.tether1_length, 'g-', 'LineWidth', 1.2); hold on;
plot(t, logger.tether2_length, 'm-', 'LineWidth', 1.2);
yline(tether_params.L0, 'r--', 'LineWidth', 1);
grid on; xlabel('Time [s]'); ylabel('Length [m]');
title('PID — Tether Lengths');
legend('Tether 1','Tether 2','Natural Length','Location','best');

pid_metrics.rmse_z_mass  = rmse_z_mass;
pid_metrics.rmse_x_mass  = rmse_x_mass;
pid_metrics.ss_err_z     = ss_err_z;
pid_metrics.ss_err_x     = ss_err_x;
pid_metrics.max_err_z    = max_err_z;
pid_metrics.max_err_x    = max_err_x;
pid_metrics.total_effort = total_effort;
pid_metrics.TV_q1        = TV_q1;
pid_metrics.TV_q2        = TV_q2;
pid_metrics.max_tether1  = max(logger.tether1_force);
pid_metrics.max_tether2  = max(logger.tether2_force);
pid_metrics.max_pitch_q1 = max(abs(rad2deg(logger.theta1)));
pid_metrics.max_pitch_q2 = max(abs(rad2deg(logger.theta2)));
pid_metrics.rmse_z_q1    = rmse_z_q1;
pid_metrics.rmse_x_q1    = rmse_x_q1;
pid_metrics.rmse_z_q2    = rmse_z_q2;
pid_metrics.rmse_x_q2    = rmse_x_q2;

save('dual_pid_results.mat', 'logger', 'pid_metrics', ...
     'waypoints', 'quad_params', 'mass_params', 'tether_params', ...
     'formation_params', 'pid_params', 'sim');

fprintf('\nResults saved to dual_pid_results.mat\n');
