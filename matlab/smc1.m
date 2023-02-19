% Constants
g = 9.81;           % Acceleration due to gravity (m/s^2)
dt = 0.1;           % Sampling time (s)

% System matrices
A = [1 dt; 0 1];    % State transition matrix
B = [dt^2/2; dt];   % Input matrix
C = [1 0; 0 1];     % Output matrix
D = [0; 0];         % Feedthrough matrix

% Process noise covariance
Q = [0.1 0; 0 0.1];

% Measurement noise covariance
R = [1 0; 0 1];

% Initial state and covariance
x = [0; 0];      % Initial state estimate

% Sliding mode controller gains
k1 = 1;
k2 = 1;

% Time vector
t = 0:dt:10;

% Input signal
u = 5*ones(size(t));    % Step input

% True state vector
x_true = [0; 0];

% State and measurement history
x_history = zeros(2,length(t));
z_history = zeros(2,length(t));
pitch_history = zeros(1,length(t));
roll_history = zeros(1,length(t));
yaw_history = zeros(1,length(t));
inertia_history = zeros(3,length(t));

% Simulate the system
for i = 1:length(t)
    % Calculate the net force on the drone
    net_force = u(i) - m*g - k*x(1) - b*x(2) + F;
    
    % Update the position and velocity
    x = x + dt*[x(2); net_force/m];
    
    % Update the control input and external force
    if t(i) > 2
        u(i) = 5;
    end
    if t(i) > 4
        F = 2;
    end
    
    % Save state and measurement history
    x_history(:,i) = x;
    z_history(:,i) = [x(1); x(2)];
    pitch_history(i) = 0;       % Placeholder for pitch data
    roll_history(i) = 0;        % Placeholder for roll data
    yaw_history(i) = 0;         % Placeholder for yaw data
    inertia_history(:,i) = [1; 1; 1];    % Placeholder for inertia data
end

% Sliding mode control loop
for i = 1:length(t)
    % Control input
    r = [cos(t(i)); sin(t(i))];     % Reference trajectory
    e = x - r;                      % Tracking error
    v = -k1*sign(e(1)) - k2*sign(e(2));
    
    % Apply control input to the system
    net_force = v - m*g - k*x(1) - b*x(2) + F;
    x = x + dt*[x(2); net_force/m];
    
    % Save state and measurement history
    x_history(:,i) = x;
    z_history(:,i) = [x(1); x(2)];
    pitch_history(i) = 0;       % Placeholder for pitch data
    roll_history(i) = 0;        % Placeholder for roll data
    yaw_history(i) = 0;         % Placeholder for yaw data
    inertia_history(:,i) = [1; 1; 1];    % Placeholder for inertia data
end

% Plot the results
figure;
subplot(3,2,1);
plot(t,x_history(1,:),t,z_history(1,:));
xlabel('Time (s)');
ylabel('Position (m)');
legend('True Position','Measured Position');

subplot(3,2,2);
plot(t,x_history(2,:),t,xhat(2)*ones(size(t)));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('True Velocity','Estimated Velocity');

subplot(3,2,3);
plot(t,pitch_history);
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');

subplot(3,2,4);
plot(t,roll_history);
xlabel('Time (s)');
ylabel('Roll Angle (rad)');

subplot(3,2,5);
plot(t,yaw_history);
xlabel('Time (s)');
ylabel('Yaw Angle (rad)');

subplot(3,2,6);
plot(t,inertia_history(1,:),t,inertia_history(2,:),t,inertia_history(3,:));
xlabel('Time (s)');
ylabel('Inertia');
legend('X-axis','Y-axis','Z-axis');
