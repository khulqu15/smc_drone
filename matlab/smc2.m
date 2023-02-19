% Define constants
g = 9.81;           % Acceleration due to gravity (m/s^2)
dt = 0.1;           % Sampling time (s)
m = 0.5;            % Mass of drone (kg)
k = 3;              % Spring constant (N/m)
b = 0.5;            % Damping coefficient (N*s/m)
F = [0; 0; 0];      % External force (N)

% Initialize state variables
x = [0; 0; 0];      % Initial position
v = [0; 0; 0];      % Initial velocity

% Initialize history variables
x_history = zeros(3, 100);
v_history = zeros(3, 100);

% Simulate the system
for i = 1:100
    % Calculate the net force on the drone
    net_force = F - m*g*[0; 0; 1] - k*x - b*v;
    
    % Update the position and velocity
    v = v + dt*net_force/m;
    x = x + dt*v;
    
    % Save the data to the history arrays
    x_history(:, i) = x;
    v_history(:, i) = v;
    
    % Update the external force (for testing)
    if i == 30
        F = [0; 0; 2];
    end
end

% Plot the results
figure;
subplot(2,3,1);
plot(1:100, x_history(1,:));
xlabel('Time (s)');
ylabel('Position (m)');
title('Drone X Position');

subplot(2,3,2);
plot(1:100, x_history(2,:));
xlabel('Time (s)');
ylabel('Position (m)');
title('Drone Y Position');

subplot(2,3,3);
plot(1:100, x_history(3,:));
xlabel('Time (s)');
ylabel('Position (m)');
title('Drone Z Position');

subplot(2,3,4);
plot(1:100, v_history(1,:));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Drone X Velocity');

subplot(2,3,5);
plot(1:100, v_history(2,:));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Drone Y Velocity');

subplot(2,3,6);
plot(1:100, v_history(3,:));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Drone Z Velocity');
