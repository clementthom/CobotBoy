%PID with Interactive Slider

clear all; clc;

pkg load control; pkg load qt;

% Constants
R = 3.8;

% Transfer function
num = [0.03];
denum = [4.8e-5 1];
Gp = tf(num, denum);
Hcap = R;

% Closed-loop without PID
figure(1); step(feedback(Gp, Hcap)); grid on;
title('Closed-loop response without PID');

% Initial PID parameters
Kp = 1;
Ki = 0;
Kd = 1;

% Create figure and slider
f = figure('Name', 'PID Tuner', 'Position', [100, 100, 600, 400]);
ax = axes('Parent', f, 'Position', [0.1, 0.3, 0.8, 0.6]);
slider = uicontrol('Parent', f, 'Style', 'slider', ...
                   'Position', [100, 20, 400, 20], ...
                   'Min', 0, 'Max', 3, 'Value', Kd, ...
                   'Callback', @(src,~) update_plot(src, Gp, Hcap, Kp, Ki));

% Initial plot
update_plot(slider, Gp, Hcap, Kp, Ki);

function update_plot(slider, Gp, Hcap, Kp, Ki)
    Kd = get(slider, 'Value');
    Hc = pid(Kp, Ki, Kd);
    CLwithPID = feedback(Gp*Hc, Hcap);
    [y, t] = step(CLwithPID);
    axes(ax);
    plot(t, y); grid on;
    title(['Closed-loop response with PID (Kd = ', num2str(Kd, '%.2f'), ')']);
    xlabel('Time (s)'); ylabel('Amplitude');
end
