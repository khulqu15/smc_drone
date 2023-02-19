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

% Inisialisasi variabel Kalman filter
xhat = [0; 0];      % Inisialisasi estimasi posisi dan kecepatan awal
P = [1 0; 0 1];     % Inisialisasi covariance matrix awal
Q = [0.1 0; 0 0.1]; % Covariance matrix untuk noise proses
R = [1 0; 0 1];     % Covariance matrix untuk noise pengukuran

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

% Simulasi
x = [0; 0];         % Kecepatan awal dan posisi awal
u = 0;              % Input kontrol awal
F = 0;              % Gaya eksternal awal

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
    
    % Kalman filter
    % Predict
    xhat = A*xhat + B*u;
    P = A*P*A' + Q;
    % Update
    y = [x(1); x(2)] - C*xhat;
    S = C*P*C' + R;
    K = P*C'*inv(S);
    xhat = xhat + K*y;
    P = (eye(2) - K*C)*P;
    
    % Update posisi drone
    drone_vertices_new = drone_vertices' + repmat([xhat(1), xhat(2), 0], size(drone_vertices, 2), 1);
    set(drone, 'Vertices', drone_vertices_new);
    
    % Tampilkan animasi
    pause(0.01);
end

% Plot hasil simulasi
figure;
subplot(3, 3, 1);
plot(t, x_history(1,:), t, x_history(2,:));
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
plot(t, inertia_x_history(1,:), t, inertia_x_history(2,:));
xlabel('Waktu (s)');
ylabel('Inertia X');
legend('Placeholder inertia x1', 'Placeholder inertia x2');

subplot(3, 3, 8);
plot(t, inertia_y_history(1,:), t, inertia_y_history(2,:));
xlabel('Waktu (s)');
ylabel('Inertia Y');
legend('Placeholder inertia y1', 'Placeholder inertia y2');

subplot(3, 3, 9);
plot(t, inertia_z_history(1,:), t, inertia_z_history(2,:));
xlabel('Waktu (s)');
ylabel('Inertia Z');
legend('Placeholder inertia z1', 'Placeholder inertia z2');

