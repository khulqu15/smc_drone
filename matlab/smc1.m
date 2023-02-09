% Define the state transition matrix
A = [1 dt; 0 1];

% Define the control input matrix
B = [dt^2/2; dt];

% Define the measurement matrix
C = [1 0];

% Define the initial state
x = [0; 0];

% Define the desired position and velocity
des_pos = 5;
des_vel = 0;

% Define the control input
u = 0;

% Define the sampling time
dt = 0.1;

% Define the number of steps
N = 100;

% Pre-allocate memory for the arrays
x_array = zeros(2,N);
y_array = zeros(1,N);

% Main loop
for i = 1:N
    % Generate the process noise
    w = sqrt(Q) * randn(2,1);
    
    % Generate the measurement noise
    v = sqrt(R) * randn(1,1);
    
    % Simulate the process
    x = A * x + B * u + w;
    
    % Simulate the measurement
    y = C * x + v;
    
    % Compute the control input using sliding mode control
    e_pos = des_pos - x(1);
    e_vel = des_vel - x(2);
    u = u + sign(e_pos) * dt;
    
    % Save the results
    x_array(:,i) = x;
    y_array(i) = y;
end

% Plot the results
figure;
subplot(2,1,1);
plot(x_array(1,:), 'LineWidth', 2);
hold on;
plot(y_array, 'LineWidth', 2);
legend('Estimated position', 'Measured position');
xlabel('Time step');
ylabel('Position');

subplot(2,1,2);
plot(x_array(2,:), 'LineWidth', 2);
xlabel('Time step');
ylabel('Velocity');