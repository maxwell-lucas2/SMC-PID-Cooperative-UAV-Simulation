%% Dual-Quad PID vs SMC Controller Comparison
%  Maxwell Lucas, University of Vermont, Honors College Thesis
%  Run after both main_pid_dual.m and main_smc_dual.m
clear; close all; clc;

% Load results
pid = load('dual_pid_results.mat');
smc = load('dual_smc_results.mat');

t = pid.logger.t;

% Verify parameter match
assert(pid.quad_params.m == smc.quad_params.m, 'Quad mass mismatch!');
assert(pid.mass_params.m == smc.mass_params.m, 'Payload mass mismatch!');
assert(pid.formation_params.half_angle_deg == smc.formation_params.half_angle_deg, ...
    'V-angle mismatch!');
fprintf('Parameter check passed: m_q=%.2f kg, m_p=%.2f kg, V_angle=%d°\n', ...
    pid.quad_params.m, pid.mass_params.m, pid.formation_params.half_angle_deg);

% Shorthand
pl = pid.logger;  sl = smc.logger;
clr_smc = [0.85 0.33 0.1];  % Orange for SMC

% PAYLOAD TRACKING
figure('Position', [50 50 1400 900], 'Name', 'Dual-Quad Tracking Comparison');

subplot(3,2,1);
plot(t, pl.z_mass, 'b-', 'LineWidth', 1.5); hold on;
plot(t, sl.z_mass, 'Color', clr_smc, 'LineWidth', 1.5);
plot(t, pl.z_mass_des, 'k--', 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Altitude [m]');
title('Payload Altitude');
legend('PID','SMC','Reference','Location','best');

subplot(3,2,2);
plot(t, pl.x_mass, 'b-', 'LineWidth', 1.5); hold on;
plot(t, sl.x_mass, 'Color', clr_smc, 'LineWidth', 1.5);
plot(t, pl.x_mass_des, 'k--', 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Position [m]');
title('Payload Lateral Position');
legend('PID','SMC','Reference','Location','best');

subplot(3,2,3);
plot(t, pl.z_mass_des - pl.z_mass, 'b-', 'LineWidth', 1.2); hold on;
plot(t, sl.z_mass_des - sl.z_mass, 'Color', clr_smc, 'LineWidth', 1.2);
yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Error [m]');
title('Payload Altitude Error');
legend('PID','SMC','Location','best');

subplot(3,2,4);
plot(t, pl.x_mass_des - pl.x_mass, 'b-', 'LineWidth', 1.2); hold on;
plot(t, sl.x_mass_des - sl.x_mass, 'Color', clr_smc, 'LineWidth', 1.2);
yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Error [m]');
title('Payload Lateral Error');
legend('PID','SMC','Location','best');

subplot(3,2,5);
plot(t, rad2deg(pl.theta1), 'b-', 'LineWidth', 1.0); hold on;
plot(t, rad2deg(sl.theta1), 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Quad 1 Pitch Angle');
legend('PID','SMC','Location','best');

subplot(3,2,6);
plot(t, pl.tether1_force, 'b-', 'LineWidth', 1.0); hold on;
plot(t, sl.tether1_force, 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Force [N]');
title('Tether 1 Force');
legend('PID','SMC','Location','best');

sgtitle('PID vs SMC — Dual-Quad Payload Tracking', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ======================== FIGURE 2: CONTROL EFFORT ======================
figure('Position', [100 100 1400 500], 'Name', 'Dual-Quad Control Comparison');

subplot(1,3,1);
plot(t, pl.F1_q1 + pl.F2_q1, 'b-', 'LineWidth', 1.0); hold on;
plot(t, sl.F1_q1 + sl.F2_q1, 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Total Thrust [N]');
title('Quad 1 Total Thrust'); legend('PID','SMC');

subplot(1,3,2);
plot(t, pl.tau1, 'b-', 'LineWidth', 1.0); hold on;
plot(t, sl.tau1, 'Color', clr_smc, 'LineWidth', 1.0);
yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Torque [N·m]');
title('Quad 1 Control Torque'); legend('PID','SMC');

subplot(1,3,3);
pid_cum = cumtrapz(t, pl.F1_q1 + pl.F2_q1 + pl.F1_q2 + pl.F2_q2);
smc_cum = cumtrapz(t, sl.F1_q1 + sl.F2_q1 + sl.F1_q2 + sl.F2_q2);
plot(t, pid_cum, 'b-', 'LineWidth', 1.5); hold on;
plot(t, smc_cum, 'Color', clr_smc, 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Cumulative [N·s]');
title('Total System Effort'); legend('PID','SMC');

sgtitle('PID vs SMC — Control Effort', 'FontSize', 14, 'FontWeight', 'bold');

%% Trajectory Overlay
figure('Position', [150 150 800 650], 'Name', 'Dual-Quad Path Comparison');

% PID
plot(pl.x1, pl.z1, 'b-', 'LineWidth', 1.0); hold on;
plot(pl.x2, pl.z2, 'b--', 'LineWidth', 1.0);
plot(pl.x_mass, pl.z_mass, 'b-', 'LineWidth', 2.0);

% SMC
plot(sl.x1, sl.z1, '-', 'Color', clr_smc, 'LineWidth', 1.0);
plot(sl.x2, sl.z2, '--', 'Color', clr_smc, 'LineWidth', 1.0);
plot(sl.x_mass, sl.z_mass, '-', 'Color', clr_smc, 'LineWidth', 2.0);

% Waypoints
wp = pid.waypoints;
plot(wp(:,1), wp(:,2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', [0.3 0.3 0.3]);
plot([-3 3], [0 0], 'k-', 'LineWidth', 2);

axis equal; grid on; xlabel('X [m]'); ylabel('Z [m]');
title('Flight Path Comparison (XZ Plane)', 'FontSize', 13);
legend('PID Q1','PID Q2','PID Payload', ...
       'SMC Q1','SMC Q2','SMC Payload', ...
       'Waypoints','Ground','Location','best');

%% Tether Comparison
figure('Position', [200 200 1200 400], 'Name', 'Dual-Quad Tether Comparison');

subplot(1,2,1);
plot(t, pl.tether1_force, 'b-', 'LineWidth', 1.0); hold on;
plot(t, sl.tether1_force, 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Force [N]');
title('Tether 1 Force'); legend('PID','SMC');

subplot(1,2,2);
plot(t, pl.tether2_force, 'b-', 'LineWidth', 1.0); hold on;
plot(t, sl.tether2_force, 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Force [N]');
title('Tether 2 Force'); legend('PID','SMC');

sgtitle('PID vs SMC — Tether Force Comparison', 'FontSize', 14, 'FontWeight', 'bold');

%% Comparison Table
pm = pid.pid_metrics;
sm = smc.smc_metrics;

fprintf('\n');
fprintf('========================================================================\n');
fprintf('    DUAL-QUAD  PID vs SMC  —  PERFORMANCE COMPARISON TABLE\n');
fprintf('========================================================================\n');
fprintf('%-35s | %12s | %12s\n', 'Metric', 'PID', 'SMC');
fprintf('%s\n', repmat('-', 1, 65));
fprintf('%-35s | %12.4f | %12.4f\n', 'Payload Alt RMSE [m]',       pm.rmse_z_mass,  sm.rmse_z_mass);
fprintf('%-35s | %12.4f | %12.4f\n', 'Payload Lat RMSE [m]',       pm.rmse_x_mass,  sm.rmse_x_mass);
fprintf('%-35s | %12.4f | %12.4f\n', 'Payload Alt SS Error [m]',   pm.ss_err_z,     sm.ss_err_z);
fprintf('%-35s | %12.4f | %12.4f\n', 'Payload Lat SS Error [m]',   pm.ss_err_x,     sm.ss_err_x);
fprintf('%-35s | %12.4f | %12.4f\n', 'Max Payload Alt Error [m]',  pm.max_err_z,    sm.max_err_z);
fprintf('%-35s | %12.4f | %12.4f\n', 'Max Payload Lat Error [m]',  pm.max_err_x,    sm.max_err_x);
fprintf('%-35s | %12.2f | %12.2f\n', 'Total Ctrl Effort [N·s]',    pm.total_effort, sm.total_effort);
fprintf('%-35s | %12.2f | %12.2f\n', 'Thrust TV Q1 [N]',           pm.TV_q1,        sm.TV_q1);
fprintf('%-35s | %12.2f | %12.2f\n', 'Thrust TV Q2 [N]',           pm.TV_q2,        sm.TV_q2);
fprintf('%-35s | %12.2f | %12.2f\n', 'Max Tether 1 Force [N]',     pm.max_tether1,  sm.max_tether1);
fprintf('%-35s | %12.2f | %12.2f\n', 'Max Tether 2 Force [N]',     pm.max_tether2,  sm.max_tether2);
fprintf('%-35s | %12.2f | %12.2f\n', 'Max |Pitch| Q1 [deg]',       pm.max_pitch_q1, sm.max_pitch_q1);
fprintf('%-35s | %12.2f | %12.2f\n', 'Max |Pitch| Q2 [deg]',       pm.max_pitch_q2, sm.max_pitch_q2);
fprintf('%-35s | %12.4f | %12.4f\n', 'Quad 1 Alt RMSE [m]',        pm.rmse_z_q1,    sm.rmse_z_q1);
fprintf('%-35s | %12.4f | %12.4f\n', 'Quad 1 Lat RMSE [m]',        pm.rmse_x_q1,    sm.rmse_x_q1);
fprintf('%-35s | %12.4f | %12.4f\n', 'Quad 2 Alt RMSE [m]',        pm.rmse_z_q2,    sm.rmse_z_q2);
fprintf('%-35s | %12.4f | %12.4f\n', 'Quad 2 Lat RMSE [m]',        pm.rmse_x_q2,    sm.rmse_x_q2);
fprintf('========================================================================\n');
fprintf('\nShared parameters:\n');
fprintf('  Quads: m=%.2f kg, J=%.3f, L=%.2f m\n', ...
    pid.quad_params.m, pid.quad_params.J, pid.quad_params.L);
fprintf('  Payload: m=%.2f kg\n', pid.mass_params.m);
fprintf('  Tether: k=%.0f N/m, c=%.1f N·s/m, L0=%.1f m\n', ...
    pid.tether_params.k, pid.tether_params.c, pid.tether_params.L0);
fprintf('  V-angle: %d° half-angle\n', pid.formation_params.half_angle_deg);
fprintf('  Simulation: dt=%.4f s, T=%.0f s\n', pid.sim.dt, pid.sim.t_end);
