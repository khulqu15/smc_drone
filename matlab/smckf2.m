% Konstanta dan variabel
g = 9.81;           % Percepatan gravitasi (m/s^2)
dt = 0.1;           % Waktu sampling (s)
m = 0.5;            % Massa drone (kg)
k = 3;              % Konstanta pegas (N/m)
b = 0.5;            % Koefisien redaman (N*s/m)
t = 0:dt:10;        % Vektor waktu (s)

% Inisialisasi variabel
x_history = zeros(2,length(t));
y_history = zeros(2,length(t));
z_history = zeros(2,length(t));
pitch_history = zeros(1,length(t));
roll_history = zeros(1,length(t));
yaw_history = zeros(1,length(t));
inertia_x_history = zeros(2,length(t));
inertia_y_history = zeros(2,length(t));
inertia_z_history = zeros(2,length(t));
xhat_history = zeros(2,length(t));
Phat_history = zeros(4,4,length(t));

% Inisialisasi variabel animasi
fig = figure;
ax = axes('XLim', [-10 10], 'YLim', [-10 10], 'ZLim', [-10 10]);
view(3);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;

% Gambar drone
drone_size = 1;
drone_color = 'b';
drone_vertices = drone_size * [0.5 -0.5 -0.5 0.5 0.5 0.5 -0.5 -0.5 0.5 0.5 -0.5 -0.5;                                 0.5 0.5 -0.5 -0.5 0.5 0.5 0.5 -0.5 -0.5 0.5 0.5 -0.5;                                 0 0 0 0 0.5 -0.5 -0.5 0.5 0.5 -0.5 -0.5 0.5];
drone_faces = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 5 6 7 8];

drone = patch('Vertices', drone_vertices', 'Faces', drone_faces, 'FaceColor', drone_color);

% Kalman filter variabel
A = [1 dt; 0 1];     % State transition matrix
B = [dt^2/2; dt];    % Input matrix
C = [1 0; 0 1];      % Output matrix
D = [0; 0];          % Feedthrough matrix
Q = [0.1 0; 0 0.1];  % Process noise covariance
R = [1 0; 0 1];      % Measurement noise covariance
xhat = [0; 0];       % Initial state estimate
P = [1 0; 0 1];      % Initial covariance estimate

% Simulasi
x = [0; 0];         % Kecepatan awal dan posisi awal
u = 0;              % Input kontrol awal
F = 0;              % Gaya
for i = 1:length(t)
    % Hitung gaya bersih pada drone
    net_force = u - m*g - k*x(1) - b*x(2) + F;
    
    % Update posisi dan kecepatan
    x = x + dt*[x(2); net_force/m];
    
    % Update input kontrol dan gaya eksternal
    if t(i) > 2
        u = 5;
    end
    if t(i) > 4
        F = 2;
    end
    
    % Kalman filter prediction step
    xhat_minus = A*xhat + B*u;
    P_minus = A*P*A' + Q;
    
    % Kalman filter correction step
    y = C*[x(1); x(2)];
    K = P_minus*C'/(C*P_minus*C' + R);
    xhat = xhat_minus + K*(y - C*xhat_minus);
    P = (eye(2) - K*C)*P_minus;
    
    % Save history of data
    x_history(:,i) = x;
    y_history(:,i) = [0; 0];    % Placeholder for y data
    z_history(:,i) = [0; 0];    % Placeholder for z data
    pitch_history(i) = 0;       % Placeholder for pitch data
    roll_history(i) = 0;        % Placeholder for roll data
    yaw_history(i) = 0;         % Placeholder for yaw data
    inertia_x_history(:,i) = [1; 1];   % Placeholder for inertia x data
    inertia_y_history(:,i) = [1; 1];   % Placeholder for inertia y data
    inertia_z_history(:,i) = [1; 1];   % Placeholder for inertia z data
    xhat_history(:,i) = xhat;
    Phat_history(:,:,i) = [P, zeros(2,2); zeros(2,2), P];
    
    % Update posisi drone
    drone_vertices_new = drone_vertices' + repmat([x(1), x(2), 0], size(drone_vertices, 2), 1);
    set(drone, 'Vertices', drone_vertices_new);
    
    % Tampilkan animasi
    pause(0.01);
end

% Plot hasil simulasi
figure;
subplot(3, 3, 1);
plot(t, x_history(1,:), t, xhat_history(1,:));
xlabel('Waktu (s)');
ylabel('Posisi (m)');
legend('Posisi Sebenarnya', 'Posisi Terukur');

subplot(3, 3, 2);
plot(t, y_history(1,:), t, y_history(2,:));
xlabel('Waktu (s)');
ylabel('y (m)');
legend('Placeholder y1', 'Placeholder y2');

subplot(3, 3, 3);
plot(t, z_history(1,:), t, z_history(2,:));
xlabel('Waktu (s)');
ylabel('z (m)');
legend('Placeholder z1', 'Placeholder z2');

subplot(3, 3, 4);
plot(t, pitch_history);
xlabel('Waktu (s)');
ylabel('Pitch (rad)');

subplot(3, 3, 5);
plot(t, roll_history);
xlabel('Waktu (s)');
ylabel('Roll (rad)');

subplot(3, 3, 6);
plot(t, yaw_history);
xlabel('Waktu (s)');
ylabel('Yaw (rad)');

subplot(3, 3, 7);
Phat_11 = squeeze(Phat_history(1,1,:));
Phat_12 = squeeze(Phat_history(1,2,:));
Phat_21 = squeeze(Phat_history(2,1,:));
Phat_22 = squeeze(Phat_history(2,2,:));
plot(t, Phat_11, t, Phat_12, t, Phat_21, t, Phat_22);
xlabel('Waktu (s)');
ylabel('Covariance Matrix');
legend('P_{11}', 'P_{12}', 'P_{21}', 'P_{22}');

subplot(3, 3, 8);
plot(t, inertia_x_history(1,:), t, inertia_x_history(2,:));
xlabel('Waktu (s)');
ylabel('Inertia X');
legend('Placeholder inertia x1', 'Placeholder inertia x2');

subplot(3, 3, 9);
plot(t, inertia_y_history(1,:), t, inertia_y_history(2,:));
xlabel('Waktu (s)');
ylabel('Inertia Y');
legend('Placeholder inertia y1', 'Placeholder inertia y2');
