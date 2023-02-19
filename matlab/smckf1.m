% Constants
g = 9.81;           % Acceleration due to gravity (m/s^2)
dt = 0.01;           % Sampling time (s)

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
xhat = [0; 0];      % Initial state estimate
P = [1 0; 0 1];     % Initial covariance estimate

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

% Kalman filter loop
for i = 1:length(t)
    % True state update
    x_true = A*x_true + B*u(i) + [0; -g*dt^2/2];
    
    % Measurement update
    z = C*x_true + sqrt(R)*randn(2,1);
    
    % Kalman filter prediction step
    xhat_minus = A*xhat + B*u(i);
    P_minus = A*P*A' + Q;
    
    % Kalman filter correction step
    K = P_minus*C'/(C*P_minus*C' + R);
    xhat = xhat_minus + K*(z - C*xhat_minus);
    P = (eye(2) - K*C)*P_minus;
    
    % Save state and measurement history
    x_history(:,i) = x_true;
    z_history(:,i) = z;
    pitch_history(i) = 0;       % Placeholder for pitch data
    roll_history(i) = 0;        % Placeholder for roll data
    yaw_history(i) = 0;         % Placeholder for yaw data
    inertia_history(:,i) = [1; 1; 1];    % Placeholder for inertia data
end

% Sliding mode control loop
for i = 1:length(t)
    % Control input
    r = [cos(t(i)); sin(t(i))];     % Reference trajectory
    e = xhat - r;                   % Tracking error
    v = -k1*sign(e(1)) - k2*sign(e(2));
    
    % Apply control input to the system
    x_true = A*x_true + B*v + [0; -g*dt^2/2];
    z = C*x_true + sqrt(R)*randn(2,1);
    
    % Save state and measurement history
    x_history(:,i) = x_true;
    z_history(:,i) = z;
    pitch_history(i) = 0;       % Placeholder for pitch data
    roll_history(i) = 0;        % Placeholder for roll
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
