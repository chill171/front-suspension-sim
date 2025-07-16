% front_suspension_sim.m
% Simulate front suspension (1-DOF mass-spring-damper) response to terrain input

clear; clc;

%% Parameters
m = 20;             % Mass [kg] (fork + partial rider)
k = 8000;           % Spring constant [N/m]
c = 1200;           % Damping coefficient [Ns/m]
k1 = 8000;           % linear term [N/m]
k2 = 5e8;         % cubic term [N/m^3] — controls ramp-up
%% Fork Type: 'coil' or 'air'
fork_type = 'air';  % change to 'air' to simulate nonlinear spring

%% Terrain Input Type
input_type = 'sine';  % Options: 'step', 'sine'

switch input_type
    case 'step'
        y = @(t) 0.05 * (t >= 1);       % 5 cm step
        dy = @(t) 0;
    case 'sine'
        A = 0.05; f = 2;                % 1 cm amplitude, 2 Hz
        y = @(t) A * sin(2*pi*f*t);
        dy = @(t) A * 2*pi*f * cos(2*pi*f*t);
end

%% Inline spring force function 
spring_force = @(x, t) ...
    strcmp(fork_type, 'coil') * (k * (x - y(t))) + ...
    strcmp(fork_type, 'air')  * (k1 * (x - y(t)) + k2 * (x - y(t))^3);

%% ODE System with nonlinear spring (air fork support)
odefun = @(t, z) [
    z(2);
    (-c*(z(2) - dy(t)) - spring_force(z(1), t)) / m
];
%% ODE System with nonlinear spring (air fork support)

% Define ODE using anonymous function (inherits from workspace)
odefun = @(t, z) [
    z(2);
    (-c*(z(2) - dy(t)) - spring_force(z(1), t)) / m
];

% Inline spring force using fork type logic
spring_force = @(x, t) ...
    strcmp(fork_type, 'coil') * (k * (x - y(t))) + ...
    strcmp(fork_type, 'air')  * (k1 * (x - y(t)) + k2 * (x - y(t)).^3);

%% Simulation
tspan = [0 5];
z0 = [0; 0];                     % Initial conditions
[t, z] = ode45(odefun, tspan, z0);

x = z(:,1);                      % Suspension displacement

v = z(:,2);                        % Suspension velocity
a = diff(v) ./ diff(t);           % Suspension acceleration
t_a = t(1:end-1);                 % Time vector for acceleration

spring_f = spring_force(x, t);             % Spring force over time
damper_f = c * (v - dy(t));                % Damper force over time

travel_limit = 0.08;                      % 80 mm max travel
max_travel = max(x) - min(x);            % Peak-to-peak
bottom_out = any(abs(x) >= travel_limit);

F_total = spring_f + damper_f;   % Net force acting on the suspension

if bottom_out
    disp('Bottom-out occurred!');
else
    disp('No bottom-out.');
end

%% Export results to CSV
export = true;
if export
    output = table(t, x, v, spring_f, damper_f, F_total, ...
    'VariableNames', {'Time_s', 'Displacement_m', 'Velocity_mps', ...
                      'SpringForce_N', 'DamperForce_N', 'TotalForce_N'});
end

%% Plot
figure;
plot(t, x*1000, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Displacement [mm]');
title(['Suspension Response - ', upper(fork_type), ' Fork']);
grid on;

hold on
plot(t, y(t)*1000, '--r', 'LineWidth', 1)
legend('Suspension Displacement', 'Terrain Input');