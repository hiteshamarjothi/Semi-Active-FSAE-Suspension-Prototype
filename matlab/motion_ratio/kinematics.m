function out = kinematics(cfg)

pivot = cfg.pivot;

push_b = cfg.pushrod_bellcrank;
damp_b = cfg.damper_bellcrank;

push_out = cfg.pushrod_outboard;
damp_ch  = cfg.damper_chassis;

L_push = norm(push_b - pivot);
L_damp = norm(damp_b - pivot);
L_pushrod = norm(push_b - push_out);

theta_push = atan2(push_b(2), push_b(1));
theta_damp = atan2(damp_b(2), damp_b(1));

wheel_travel = cfg.wheel_travel;

damper_length = nan(size(wheel_travel));
bell_angle    = nan(size(wheel_travel));

prev_inboard = push_b;

for i = 1:length(wheel_travel)

    current_out = push_out + [0 wheel_travel(i)];

    d = norm(current_out - pivot);

    if d > (L_push + L_pushrod) || d < abs(L_push - L_pushrod)
        continue;
    end

    a = (L_push^2 - L_pushrod^2 + d^2) / (2*d);
    h = sqrt(max(L_push^2 - a^2, 0));

    u = (current_out - pivot) / d;
    v = [-u(2), u(1)];

    P1 = pivot + a*u + h*v;
    P2 = pivot + a*u - h*v;

    if norm(P1 - prev_inboard) < norm(P2 - prev_inboard)
        inboard = P1;
    else
        inboard = P2;
    end

    prev_inboard = inboard;

    theta = atan2(inboard(2), inboard(1));

    dtheta = theta - theta_push;

    bell_angle(i) = dtheta;

    new_theta_damp = theta_damp + dtheta;

    damper_rb = pivot + L_damp * [cos(new_theta_damp), sin(new_theta_damp)];

    damper_length(i) = norm(damper_rb - damp_ch);

end

out.damper_length = damper_length;
out.bell_angle = unwrap(bell_angle);
out.wheel_travel = wheel_travel;

end