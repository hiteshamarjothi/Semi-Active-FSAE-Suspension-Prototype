clc;
clear;
close all;

%% =========================================================
%  PUSHROD-ROCKER MOTION RATIO ANALYSIS
%  2D Kinematic Model
%  Coordinate system:
%  X = lateral
%  Y = vertical (up)
%  Pivot located at origin
%% =========================================================

%% ---------------- GEOMETRY ----------------
% All units in mm

pivot              = [0, 0];

% Pushrod attachment on bellcrank
pushrod_bellcrank  = [-55.0, -0.9];

% Damper attachment on bellcrank
damper_bellcrank   = [10.2, 64.2];

% Damper chassis mount
damper_chassis     = [175.3, -9.4];

% Pushrod outboard point (upright)
pushrod_outboard   = [-230.8, -317.5];

%% ---------------- BELLCRANK GEOMETRY ----------------

push_vec = pushrod_bellcrank - pivot;
damp_vec = damper_bellcrank  - pivot;

% Arm lengths
L_push = norm(push_vec);
L_damp = norm(damp_vec);

% Initial angles
theta_push = atan2(push_vec(2), push_vec(1));
theta_damp = atan2(damp_vec(2), damp_vec(1));

% Fixed pushrod length
L_pushrod = norm(pushrod_bellcrank - pushrod_outboard);

% Arm angle validation
arm_angle = acos(dot(push_vec, damp_vec) / (L_push * L_damp));

%% ---------------- GEOMETRY SUMMARY ----------------

fprintf('--- Geometry Summary ---\n');
fprintf('L_push:            %.3f mm\n', L_push);
fprintf('L_damp:            %.3f mm\n', L_damp);
fprintf('L_pushrod:         %.3f mm\n', L_pushrod);
fprintf('theta_push:        %.2f deg\n', rad2deg(theta_push));
fprintf('theta_damp:        %.2f deg\n', rad2deg(theta_damp));
fprintf('Arm angle:         %.2f deg\n', rad2deg(arm_angle));

%% =========================================================
%  WHEEL TRAVEL SWEEP
%% =========================================================

% Reduced range to stay within physical geometry
wheel_travel = -20 : 0.5 : 20;

damper_length  = nan(1, length(wheel_travel));
bellcrank_angle = nan(1, length(wheel_travel));

% Store previous valid solution to prevent branch jumping
prev_inboard = pushrod_bellcrank;

%% ---------------- MAIN LOOP ----------------

for i = 1:length(wheel_travel)

    % Move outboard point vertically
    current_outboard = pushrod_outboard + [0, wheel_travel(i)];

    % Distance between pivot and outboard point
    d = norm(current_outboard - pivot);

    %% ---------- INTERSECTION CHECK ----------

    if d > (L_push + L_pushrod) || d < abs(L_push - L_pushrod)

        warning('No solution at wheel travel = %.1f mm', ...
                 wheel_travel(i));

        continue;
    end

    %% ---------- CIRCLE-CIRCLE INTERSECTION ----------

    % Circle 1:
    % Center = pivot
    % Radius = bellcrank pushrod arm length

    % Circle 2:
    % Center = outboard point
    % Radius = fixed pushrod length

    a = (L_push^2 - L_pushrod^2 + d^2) / (2*d);

    h_sq = L_push^2 - a^2;

    % Numerical protection
    h_sq = max(h_sq, 0);

    h = sqrt(h_sq);

    % Unit vector from pivot to outboard
    u = (current_outboard - pivot) / d;

    % Perpendicular vector
    v = [-u(2), u(1)];

    % Two possible intersection solutions
    P1 = pivot + a*u + h*v;
    P2 = pivot + a*u - h*v;

    %% ---------- CONTINUITY SELECTION ----------
    % Choose solution closest to previous timestep
    % Prevents branch flipping/singularity jumps

    if norm(P1 - prev_inboard) < norm(P2 - prev_inboard)
        inboard = P1;
    else
        inboard = P2;
    end

    % Update previous point
    prev_inboard = inboard;

    %% ---------- BELLCRANK ROTATION ----------

    current_theta_push = atan2(inboard(2), inboard(1));

    delta_theta = current_theta_push - theta_push;

    bellcrank_angle(i) = delta_theta;

    %% ---------- NEW DAMPER POSITION ----------

    current_theta_damp = theta_damp + delta_theta;

    new_damper_bellcrank = pivot + ...
        L_damp * [cos(current_theta_damp), ...
                  sin(current_theta_damp)];

    %% ---------- DAMPER LENGTH ----------

    damper_length(i) = ...
        norm(new_damper_bellcrank - damper_chassis);

end

bellcrank_angle = unwrap(bellcrank_angle);

%% =========================================================
%  MOTION RATIO
%% =========================================================

motion_ratio = abs(diff(damper_length) ./ diff(wheel_travel));

wheel_travel_mid = ...
    (wheel_travel(1:end-1) + wheel_travel(2:end)) / 2;

%% ---------------- REMOVE INVALID VALUES ----------------

valid = ~isnan(motion_ratio);

motion_ratio_valid = motion_ratio(valid);
travel_valid       = wheel_travel_mid(valid);

%% =========================================================
%  VALIDATION
%% =========================================================

fprintf('\n--- Motion Ratio Validation ---\n');

fprintf('Range:             %.4f to %.4f\n', ...
        min(motion_ratio_valid), ...
        max(motion_ratio_valid));

fprintf('Mean:              %.4f\n', ...
        mean(motion_ratio_valid));

fprintf('Peak-to-peak var:  %.2f%%\n', ...
        (max(motion_ratio_valid) - min(motion_ratio_valid)) ...
        / mean(motion_ratio_valid) * 100);

zero_crossings = sum(diff(sign(motion_ratio_valid)) ~= 0);

fprintf('Zero crossings:    %d\n', zero_crossings);

%% ---------- BASIC VALIDATION ----------

if max(motion_ratio_valid) > 2.0

    warning('MR exceeds typical FSAE range');

elseif min(motion_ratio_valid) < 0.3

    warning('MR below typical FSAE range');

else

    fprintf('PASS: Motion ratio within plausible range\n');

end

%% =========================================================
%  PLOTS
%% =========================================================

%% ---------- MOTION RATIO PLOT ----------

figure;

plot(travel_valid, motion_ratio_valid, ...
    'b-', 'LineWidth', 2);

hold on;

yline(mean(motion_ratio_valid), ...
    'r--', 'Mean MR');

xlabel('Wheel Travel (mm)');
ylabel('Motion Ratio (damper / wheel)');

title('Motion Ratio vs Wheel Travel — Pushrod-Rocker');

grid on;

%% ---------- BELLCRANK ROTATION ----------

figure;

plot(wheel_travel, ...
     rad2deg(bellcrank_angle), ...
     'LineWidth', 2);

xlabel('Wheel Travel (mm)');
ylabel('Bellcrank Rotation (deg)');

title('Bellcrank Rotation vs Wheel Travel');

grid on;

%% =========================================================
%  OPTIONAL: DAMPER LENGTH PLOT
%% =========================================================

figure;

plot(wheel_travel, ...
     damper_length, ...
     'LineWidth', 2);

xlabel('Wheel Travel (mm)');
ylabel('Damper Length (mm)');

title('Damper Length vs Wheel Travel');

grid on;