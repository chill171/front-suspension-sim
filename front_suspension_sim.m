% front_suspension_sim.m
% Simulate front suspension (1-DOF mass-spring-damper) response to terrain input

clear; clc;

%% Parameters
m = 20;             % Mass [kg] (fork + partial rider)
k = 8000;           % Spring constant [N/m]
c = 1200;           % Damping coefficient [Ns/m]

%% Terrain input (step bump at t=1s)
y = @(t) 0.05 * (t >= 1);        % 5 cm bump
dy = @(t) 0;                     % Derivative of step is zero

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

%% Plot
figure;
plot(t, x*1000, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Suspension Response to Step Input');
grid on;