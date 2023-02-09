clc;
clear;

% Define simulation parameters
dt = 0.01; % Time step
tf = 10; % Final time
t = 0:dt:tf; % Time vector
N = length(t); % Number of time steps

% Define initial state
x0 = [0; 0; 0; 0]; % [position; velocity; acceleration]

% Define control input
u = zeros(1,N); % Constant control input

% Initialize state and control history
x_history = zeros(4,N);
u_history = zeros(1,N);

% Define sliding mode control parameters
k = 0.1; % Control gain

% Simulation loop
for i = 1:N
    % Store current state and control input
    x_history(:,i) = x0;
    u_history(i) = u(i);
    
    % Calculate sliding mode control
    error = x_history(3,i) - 9.8;
    u(i) = -k * sign(error);
    
    % Simulate drone dynamics
    x_dot = drone_dynamics(t(i), x0, u(i));
    x0 = x0 + x_dot * dt;
end

% Plot results
subplot(2,2,1);
plot(t, x_history(1,:));
legend('X Position (m)');
xlabel('Time (s)');
ylabel('X position (m)');

subplot(2,2,2);
plot(t, x_history(2,:));
legend('Velocity');
xlabel('Time (s)');
ylabel('X velocity (m/s)');

subplot(2,2,3);
plot(t, x_history(3,:));
legend('Acceleration');
xlabel('Time (s)');
ylabel('X acceleration (m/s^2)');

subplot(2,2,4);
plot(t, u_history);
legend('Control input');
xlabel('Time (s)');
ylabel('Control input (m/s^2)');

% Drone dynamics function
function x_dot = drone_dynamics(t, x, u)
    x_dot = [x(2); 9.8 + u; x(4); u];
end