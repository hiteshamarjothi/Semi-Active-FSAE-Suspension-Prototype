function mr = compute_motion_ratio(damper_length, wheel_travel)

mr = abs(diff(damper_length) ./ diff(wheel_travel));

end