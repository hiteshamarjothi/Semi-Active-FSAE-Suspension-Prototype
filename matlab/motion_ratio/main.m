clc; clear; close all;

cfg = config();

out = kinematics(cfg);

mr = compute_motion_ratio(out.damper_length, out.wheel_travel);

validation(mr);

plots(out.wheel_travel, out.damper_length, mr);