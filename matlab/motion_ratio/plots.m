function plots(wheel_travel, damper_length, mr)

figure;
plot(wheel_travel, damper_length, 'LineWidth', 2);
xlabel('Wheel Travel (mm)');
ylabel('Damper Length (mm)');
title('Damper Length vs Wheel Travel');
grid on;

figure;
plot(wheel_travel(1:end-1), mr, 'LineWidth', 2);
xlabel('Wheel Travel (mm)');
ylabel('Motion Ratio');
title('Motion Ratio vs Wheel Travel');
grid on;

end