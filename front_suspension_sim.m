% front_suspension_sim.m
% Simulate front suspension (1-DOF mass-spring-damper) response to terrain input

clear; clc;

%% Parameters
m = 20;             % Mass [kg] (fork + partial rider)
k = 8000;           % Spring constant [N/m]
c = 1200;           % Damping coefficient [Ns/m]

%% Terrain Input Type
input_type = 'sine';  % Options: 'step', 'sine'

switch input_type
    case 'step'
        y = @(t) 0.05 * (t >= 1);       % 5 cm step
        dy = @(t) 0;
    case 'sine'
        A = 0.01; f = 2;                % 1 cm amplitude, 2 Hz
        y = @(t) A * sin(2*pi*f*t);
        dy = @(t) A * 2*pi*f * cos(2*pi*f*t);
end
%% State-space ODE system
% z = [x; x_dot], where x is suspension displacement
odefun = @(t, z) [
    z(2);
    (-c*(z(2) - dy(t)) - k*(z(1) - y(t))) / m
];

%% Simulation
tspan = [0 5];
z0 = [0; 0];                     % Initial conditions
[t, z] = ode45(odefun, tspan, z0);

x = z(:,1);                      % Suspension displacement

%% Export results to CSV
export = true;
if export
    output = table(t, x, 'VariableNames', {'Time_s', 'Displacement_m'});
    writetable(output, 'suspension_response.csv');
end

%% Plot
figure;
plot(t, x*1000, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Suspension Response to Step Input');
grid on;

hold on
plot(t, y(t)*1000, '--r', 'LineWidth', 1)
legend('Suspension Displacement', 'Terrain Input')