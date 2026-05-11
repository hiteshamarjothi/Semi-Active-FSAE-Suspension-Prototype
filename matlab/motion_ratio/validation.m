function validation(mr)

fprintf('\n--- Motion Ratio Validation ---\n');

fprintf('Range: %.4f - %.4f\n', min(mr), max(mr));
fprintf('Mean:  %.4f\n', mean(mr));

variation = (max(mr)-min(mr))/mean(mr)*100;

fprintf('Variation: %.2f%%\n', variation);

if max(mr) > 2
    warning('MR too high');
elseif min(mr) < 0.3
    warning('MR too low');
else
    fprintf('PASS: MR within range\n');
end

end