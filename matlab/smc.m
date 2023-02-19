% Initialize variables
dt = 0.01;                  % Time step (s)
t = 0:dt:10;                % Time vector (s)
N = length(t);              % Number of time steps
m = 0.5;                    % Mass of drone (kg)
g = 9.81;                   % Acceleration due to gravity (m/s^2)
k = 3;                      % Spring constant (N/m)
b = 0.5;                    % Damping coefficient (N*s/m)

x_history = zeros(2, N);    % Position and velocity history
u_history = zeros(1, N);    % Control input history
F_history = zeros(1, N);    % External force history

% Initial conditions
x = [0; 0];                 % Initial position and velocity
u = 0;                      % Initial control input
F = 0;                      % Initial external force

% Simulate the system
for i = 1:N
    % Calculate the net force on the drone
    net_force = u - m*g - k*x(1) - b*x(2) + F;
    
    % Update the position and velocity
    x = x + dt*[x(2); net_force/m];
    
    % Update the control input and external force
    if t(i) > 2
        u = 5;
    end
    if t(i) > 4
        F = 2;
    end
    
    % Save the data to the history arrays
    x_history(:, i) = x;
    u_history(i) = u;
    F_history(i) = F;
end

% Plot the results
subplot(2,1,1);
plot(t, x_history(1,:));
xlabel('Time (s)');
ylabel('Position (m)');
title('Drone Position');

subplot(2,1,2);
plot(t, x_history(2,:));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Drone Velocity');
