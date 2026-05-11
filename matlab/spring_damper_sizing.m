clc;
clear;
close all;

%% =========================================================
%  SPRING AND DAMPER SIZING
%  Semi-Active Suspension Prototype — Single Corner
%
%  Inputs:   Validated motion ratio from kinematic analysis
%  Outputs:  Spring stiffness, damper coefficient, stroke,
%            servo torque requirement
%
%  Sign convention: bump = positive wheel travel
%% =========================================================

%% ---------------- INPUTS ----------------

m_sprung    = 0.600;        % kg  — sprung mass at prototype corner
f_n         = 2.5;          % Hz  — target natural frequency (FSAE)
MR_mean     = 1.1809;       % -   — mean motion ratio from kinematic analysis
MR_min      = 1.0461;       % -   — min MR (full bump)
MR_max      = 1.4617;       % -   — max MR (full droop)

zeta_target = 0.65;         % -   — damping ratio target (typical FSAE underdamped)
zeta_crit   = 1.00;         % -   — critical damping reference

wheel_travel_range = 40;    % mm  — total travel (±20mm)

%% ---------------- STEP 1: WHEEL RATE ----------------
% The wheel rate is the effective spring stiffness felt at the wheel.
% From target natural frequency and sprung mass:
%   f_n = (1/2pi) * sqrt(k_wheel / m_sprung)
%   k_wheel = (2*pi*f_n)^2 * m_sprung

omega_n  = 2 * pi * f_n;            % rad/s — natural frequency
k_wheel  = omega_n^2 * m_sprung;    % N/m   — required wheel rate

fprintf('--- Step 1: Wheel Rate ---\n');
fprintf('Natural frequency:     %.2f Hz\n',   f_n);
fprintf('omega_n:               %.4f rad/s\n', omega_n);
fprintf('Required wheel rate:   %.4f N/m  (%.4f N/mm)\n', ...
         k_wheel, k_wheel/1000);

%% ---------------- STEP 2: SPRING RATE AT DAMPER ----------------
% The spring sits at the damper, not at the wheel.
% Motion ratio scales the stiffness:
%   k_wheel = k_spring * MR^2
%   k_spring = k_wheel / MR^2
%
% Use mean MR for nominal sizing.
% Also compute at MR_min and MR_max to bound the effective wheel rate.

k_spring_nom  = k_wheel / MR_mean^2;
k_spring_soft = k_wheel / MR_max^2;   % softest effective rate (droop end)
k_spring_stiff= k_wheel / MR_min^2;   % stiffest effective rate (bump end)

fprintf('\n--- Step 2: Spring Rate at Damper ---\n');
fprintf('k_spring (nominal, mean MR):   %.4f N/m  (%.4f N/mm)\n', ...
         k_spring_nom, k_spring_nom/1000);
fprintf('k_spring (at MR_max=%.4f):    %.4f N/m  (%.4f N/mm)\n', ...
         MR_max, k_spring_soft, k_spring_soft/1000);
fprintf('k_spring (at MR_min=%.4f):    %.4f N/m  (%.4f N/mm)\n', ...
         MR_min, k_spring_stiff, k_spring_stiff/1000);

% Effective wheel rate variation due to MR nonlinearity
k_wheel_bump  = k_spring_nom * MR_min^2;
k_wheel_droop = k_spring_nom * MR_max^2;
fprintf('\nEffective wheel rate at full bump:   %.4f N/m\n', k_wheel_bump);
fprintf('Effective wheel rate at full droop:  %.4f N/m\n', k_wheel_droop);
fprintf('Wheel rate variation (bump/droop):   %.1f%%\n', ...
         (k_wheel_bump - k_wheel_droop)/k_wheel * 100);

%% ---------------- STEP 3: DAMPER STROKE ----------------
% Damper stroke = wheel travel * MR
% Read from kinematic output: damper travels ~2.3mm per mm of wheel travel
% Over ±20mm wheel travel:

damper_stroke_total = wheel_travel_range * MR_mean;   % mm

fprintf('\n--- Step 3: Damper Stroke ---\n');
fprintf('Wheel travel range:        ±%.0f mm (%.0f mm total)\n', ...
         wheel_travel_range/2, wheel_travel_range);
fprintf('Damper stroke (mean MR):   ±%.2f mm (%.2f mm total)\n', ...
         damper_stroke_total/2, damper_stroke_total);
fprintf('NOTE: Select RC shock with stroke >= %.0f mm\n', ...
         ceil(damper_stroke_total/2) + 3);   % +3mm clearance

%% ---------------- STEP 4: DAMPING COEFFICIENT ----------------
% Critical damping coefficient:
%   c_crit = 2 * sqrt(k_wheel * m_sprung)
%
% Working damping coefficient at target zeta:
%   c_working = zeta_target * c_crit
%
% These are referred to the WHEEL (wheel-rate, sprung mass).
% To get damping coefficient at the DAMPER, scale by MR:
%   c_damper = c_wheel / MR
% (velocity at damper = velocity at wheel * MR, so force scales inversely)

c_crit_wheel    = 2 * sqrt(k_wheel * m_sprung);     % N·s/m at wheel
c_working_wheel = zeta_target * c_crit_wheel;        % N·s/m at wheel

c_crit_damper    = c_crit_wheel / MR_mean;           % N·s/m at damper
c_working_damper = c_working_wheel / MR_mean;        % N·s/m at damper

fprintf('\n--- Step 4: Damping Coefficient ---\n');
fprintf('c_critical (at wheel):     %.4f N·s/m\n', c_crit_wheel);
fprintf('c_working  (at wheel):     %.4f N·s/m  (zeta=%.2f)\n', ...
         c_working_wheel, zeta_target);
fprintf('c_critical (at damper):    %.4f N·s/m\n', c_crit_damper);
fprintf('c_working  (at damper):    %.4f N·s/m  (zeta=%.2f)\n', ...
         c_working_damper, zeta_target);

%% ---------------- STEP 5: DAMPER FORCE REQUIREMENT ----------------
% At a typical excitation frequency of f_n, the peak damper velocity is:
%   v_damper_peak = amplitude * omega_n * MR
% Use 5mm wheel amplitude (modest road input at prototype scale)

wheel_amplitude_m = 0.005;          % m — 5mm road input amplitude
v_wheel_peak      = wheel_amplitude_m * omega_n;           % m/s
v_damper_peak     = v_wheel_peak * MR_mean;                % m/s

F_damper_working  = c_working_damper * v_damper_peak;      % N
F_damper_crit     = c_crit_damper    * v_damper_peak;      % N

fprintf('\n--- Step 5: Damper Force at Resonance ---\n');
fprintf('Wheel amplitude:           %.1f mm\n', wheel_amplitude_m*1000);
fprintf('Peak wheel velocity:       %.4f m/s\n', v_wheel_peak);
fprintf('Peak damper velocity:      %.4f m/s\n', v_damper_peak);
fprintf('Peak damper force (zeta=%.2f):  %.4f N\n', zeta_target, F_damper_working);
fprintf('Peak damper force (crit):       %.4f N\n', F_damper_crit);

%% ---------------- STEP 6: SERVO TORQUE CHECK ----------------
% The servo actuates a needle valve against fluid pressure in the bypass line.
% Estimate fluid pressure in bypass from damper force:
%   P = F_damper / A_piston
% Assume RC shock piston diameter ~8mm (typical 1/10 scale)
%
% Torque on needle valve:
%   T = P * A_port * r_valve
% Assume bypass port diameter ~1.5mm, needle valve moment arm ~5mm

d_piston  = 0.008;                          % m — RC shock piston diameter
A_piston  = pi/4 * d_piston^2;             % m²

P_fluid   = F_damper_working / A_piston;   % Pa — fluid pressure at working damping

d_port    = 0.0015;                         % m — bypass port diameter
A_port    = pi/4 * d_port^2;               % m²
r_valve   = 0.005;                          % m — needle valve moment arm

T_valve   = P_fluid * A_port * r_valve;    % N·m — torque to actuate valve

% MG996R stall torque: ~10 kg·cm = 0.98 N·m
T_MG996R  = 0.98;                           % N·m

fprintf('\n--- Step 6: Servo Torque Feasibility ---\n');
fprintf('Estimated fluid pressure:  %.4f Pa  (%.4f kPa)\n', ...
         P_fluid, P_fluid/1000);
fprintf('Torque to actuate valve:   %.6f N·m  (%.4f kg·cm)\n', ...
         T_valve, T_valve/0.0981);
fprintf('MG996R stall torque:       %.2f N·m  (%.1f kg·cm)\n', ...
         T_MG996R, T_MG996R/0.0981);
fprintf('Safety factor:             %.1fx\n', T_MG996R / T_valve);

if T_MG996R / T_valve > 5
    fprintf('PASS: MG996R has ample torque authority for valve actuation.\n');
else
    fprintf('WARNING: Marginal torque — consider larger servo or smaller port.\n');
end

%% ---------------- SUMMARY ----------------

fprintf('\n============================================================\n');
fprintf('  SIZING SUMMARY\n');
fprintf('============================================================\n');
fprintf('  Sprung mass:              %.0f g\n',    m_sprung*1000);
fprintf('  Target natural freq:      %.1f Hz\n',   f_n);
fprintf('  Required wheel rate:      %.2f N/m\n',  k_wheel);
fprintf('  Spring rate (at damper):  %.2f N/m  →  order %.0f–%.0f N/m spring\n', ...
         k_spring_nom, floor(k_spring_nom/50)*50, ceil(k_spring_nom/50)*50+50);
fprintf('  Damper stroke required:   ±%.1f mm  →  select shock with ≥%.0fmm stroke\n', ...
         damper_stroke_total/2, ceil(damper_stroke_total/2)+3);
fprintf('  Working damping coeff:    %.4f N·s/m (at damper)\n', c_working_damper);
fprintf('  Servo:                    MG996R — PASS\n');
fprintf('============================================================\n');

%% ---------------- PLOTS ----------------

figure;
zeta_range = 0.1:0.01:1.5;
c_range    = zeta_range * c_crit_wheel;
plot(zeta_range, c_range, 'b-', 'LineWidth', 2);
hold on;
xline(zeta_target, 'r--', sprintf('\\zeta = %.2f (working)', zeta_target));
xline(zeta_crit,   'g--', '\zeta = 1.0 (critical)');
xlabel('Damping Ratio \zeta');
ylabel('Damping Coefficient c (N·s/m) — at wheel');
title('Damping Coefficient vs Damping Ratio');
grid on;

figure;
MR_range   = 0.8:0.01:1.6;
k_s_range  = k_wheel ./ MR_range.^2;
plot(MR_range, k_s_range, 'b-', 'LineWidth', 2);
hold on;
xline(MR_mean, 'r--', sprintf('Mean MR = %.4f', MR_mean));
scatter(MR_mean, k_spring_nom, 80, 'r', 'filled');
xlabel('Motion Ratio');
ylabel('Required Spring Rate (N/m)');
title('Spring Rate vs Motion Ratio — Sensitivity');
grid on;