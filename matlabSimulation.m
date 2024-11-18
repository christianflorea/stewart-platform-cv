clc;

radius = 0.035; % Path Diameter
num_points = 100; % #Points in Path
delay = 0.075; % Time delay between each point

theta_values = linspace(0, 30*pi, num_points); % Rotation Amount
x_circle = radius * cos(theta_values); 
y_circle = radius * sin(theta_values); 

x_goal = 0;
y_goal = 0; 

error_x_last = 0; 
error_y_last = 0; 

integral_x = 0;
integral_y = 0;

time_between = 0;

% lpf_coef = 1;

figure;

% Loop circular path & start of PID code
for index = 1:num_points

x_ball = x_circle(index);
y_ball = y_circle(index);

kp = 1; % Tune
kd = 1; % Tune
ki = 1; % Tune

error_x = x_goal - x_ball;
error_y = y_goal - y_ball;

if index > 1
    time_between = toc;
end

tic;

if index > 1 
    derivative_x = (error_x-error_x_last) / time_between;
    derivative_y = (error_y-error_y_last) / time_between;
else
    derivative_x = 0;
    derivative_y = 0;
end

integral_x = integral_x + (error_x * time_between);
integral_y = integral_y + (error_y * time_between);

x_pid = (kp * error_x) + (kd * derivative_x) + (ki * integral_x);
y_pid = (kp * error_y) + (kd * derivative_y) + (ki * integral_y);

%x_pid = lpf_coef * x_pid + (1-lpf_coef) * error_x_last;
%y_pid = lpf_coef * y_pid + (1-lpf_coef) * error_y_last;

error_x_last = error_x;
error_y_last = error_y;

% Start of regular code

if x_pid == 0
    x_pid = 0.001;
end
if y_pid == 0
    y_pid = 0.001;
end

if x_pid > 0.10206
    x_pid = 0.10206;
end
if y_pid > 0.10206
    y_pid = 0.10206;
end
if x_pid < -0.10206
    x_pid = -0.10206;
end
if y_pid < -0.10206
    y_pid = -0.10206;
end

m_ball = 0.04593; % Golf ball mass in kg
g = 9.81;

V = sqrt((x_pid)^2+(y_pid)^2);
theta = (rad2deg(atan(y_pid/x_pid)));
disp(['V ', num2str(V)]);
disp(['theta ', num2str(theta)]);

% will need to check which quadrant ball is in and use appropriate eqn

phi = rad2deg((asin((2*V)/(m_ball*g)))/2);
phi = max(-15, min(15, phi));
disp(['phi ', num2str(phi)]);

V_x = abs(V*(cosd(theta))/(cosd(phi)));
V_y = abs(V*(sind(theta))/(cosd(phi))); 

phi_x = rad2deg(acos(x_pid/V_x));
%phi_x = rad2deg(atan((sind(phi))/(cosd(theta)))); %abs?----------------
%phi_x = max(-15, min(15, phi_x));
phi_y = rad2deg(acos(y_pid/V_y));
%phi_y = rad2deg(atan(sind(phi)/sind(theta))); %abs?--------------------
%phi_y = max(-15, min(15, phi_y));

% Check if either x_pid or y_pid is zero
if x_pid == 0 || y_pid == 0
    % If either coordinate is zero, do not change Vx or Vy
else
    if x_pid > 0 && y_pid > 0  % Quadrant 1
        V_x = -V_x;
        V_y = -V_y;
    elseif x_pid > 0 && y_pid < 0  % Quadrant 2
        V_x = -V_x;
        % Vy remains the same
    elseif x_pid < 0 && y_pid < 0  % Quadrant 3
        % Vx and Vy remain the same
    elseif x_pid < 0 && y_pid > 0  % Quadrant 4
        % Vx remains the same
        V_y = -V_y;
    end
end

%Vx = [V_x * cosd(phi_x), 0, V_x * sind(phi_x)];
%Vy = [0, V_y*cosd(phi_y), V_y*sin(phi_y)];

%normalV = cross(Vx, Vy);

%N_x = normalV(1); % i-component
%N_y = normalV(2); % j-component
%N_z = normalV(3); % k-component

% Normal Vector
N_x = -V_x*V_y*cosd(phi_y)*sind(phi_x);
N_y = V_x*V_y*cosd(phi_x)*sind(phi_y);
N_z = V_x*V_y*cosd(phi_x)*cosd(phi_y);

%Defines alpha beta and gamma components used later
alpha = abs(N_x);
beta = abs(N_y);
gamma = abs(N_z);

% Defines the sign of alpha beta and gamma depending on the quadrent the
% point is in.
if x_pid > 0 && y_pid > 0
    % First quadrant
    alpha = -alpha;
    beta = -beta;

elseif x_pid > 0 && y_pid < 0
    % Second quadrant
    alpha = -alpha;

elseif x_pid < 0 && y_pid < 0
    % Third quadrant
    % All three are positive

elseif x_pid < 0 && y_pid > 0
    % Fourth quadrant
    beta = -beta;
end

disp(['alpha ', num2str(alpha)]);
disp(['beta ', num2str(beta)]);
disp(['gamma ', num2str(gamma)]);

%alpha = deg2rad((rad2deg(acos((-sin(deg2rad(phi_x)) * ...
    %cos(deg2rad(phi_y))) / sqrt(sin(deg2rad(phi_x))^2 * ...
    %cos(deg2rad(phi_y))^2 + cos(deg2rad(phi_x))^2)))));
%alpha = atan(tan(deg2rad(phi_y)));

%beta = deg2rad((rad2deg(acos((cos(deg2rad(phi_x)) * ...
    %sin(deg2rad(phi_y))) / sqrt((sin(deg2rad(phi_x))^2 * ...
    %cos(deg2rad(phi_y))^2) + (cos(deg2rad(phi_x))^2))))));
%beta = atan(-tan(deg2rad(phi_x)));

%gamma = deg2rad((rad2deg(acos((cos(deg2rad(phi_x)) * ...
    %cos(deg2rad(phi_y))) / sqrt((sin(deg2rad(phi_x))^2 * ...
    %cos(deg2rad(phi_y))^2) + (cos(deg2rad(phi_x))^2))))));
%gamma = atan(-(1 ./ tan(deg2rad(phi_x))) * tan(deg2rad(phi_y)));    

% Ball Joint Position Calculations
% Initializing More Variables
L = 0.10206; %Dist to plate center in m
h = (121.922 /1000);

%Leg 1 Calculations
x1 = sqrt(L^2 - (h - (h - (L * alpha) / sqrt(gamma^2 + alpha^2)))^2);
disp(['x1: ', num2str(x1)]);
y1 = 0;
disp(['y1: ', num2str(y1)]);
z1 = h - ((L * alpha) / (sqrt(gamma^2 + alpha^2)));
disp(['z1: ', num2str(z1)]);

%Leg 2 Calculations
x2 = (-L * gamma) / sqrt(4 * gamma^2 + alpha^2 - 2 * sqrt(3) * ...
    alpha * beta + 3 * beta^2);
disp(['x2: ', num2str(x2)]);
y2 = (sqrt(3) * L * gamma) / sqrt(4 * gamma^2 + alpha^2 - 2 * ...
    sqrt(3) * alpha * beta + 3 * beta^2);
disp(['y2: ', num2str(y2)]);
z2 = h + ((L * (alpha - sqrt(3) * beta)) / sqrt(4 * gamma^2 + ...
    alpha^2 - 2 * sqrt(3) * alpha * beta + 3 * beta^2));
disp(['z2: ', num2str(z2)]);

%Leg 3 Calculations
x3 = (-L * gamma) / sqrt(4 * gamma^2 + alpha^2 + 2 * ...
    sqrt(3) * alpha * beta + 3 * beta^2);
disp(['x3: ', num2str(x3)]);
y3 = (-sqrt(3) * L * gamma) / sqrt(4 * gamma^2 + alpha^2 + 2 * ...
    sqrt(3) * alpha * beta + 3 * beta^2);
disp(['y3: ', num2str(y3)]);
z3 = h + ((L * (alpha + sqrt(3) * beta)) / sqrt(4 * gamma^2 + alpha^2 + ...
    2 * sqrt(3) * alpha * beta + 3 * beta^2));
disp(['z3: ', num2str(z3)]);

%Initializing Values for Theta Angles
L_1 = 0.06; % Bottom arm
L_2 = 0.0805; % Top arm
L_m = 0.10206;

%Link 1 Theta Calculations
L_a1 = sqrt(x1^2 + y1^2 + z1^2);


theta_1 = rad2deg(((acos((-L_2^2 + (x1^2 + y1^2 + z1^2) + 2*L_m*L_a1 ...
    *(cos(acos(z1/L_a1) + 1.5708)) + L_m^2)/ L_1))-2 * L_a1 * ...
    (acos(z1 / L_a1) + 1.5708))/(2 * L_a1 + L_m - 2 * L_1));


theta_1 = max(90, min(179,  theta_1));
%theta_1 = 135;
disp(['theta_1: ', num2str(theta_1)]);

%Link 2 Theta Calculations
L_a2 = sqrt(x2^2 + y2^2 + z2^2);


theta_2 = rad2deg((acos((-L_2^2 + (x2^2 + y2^2 + z2^2) + 2*L_m*L_a2 ...
    *cos(acos(z2/L_a2) + 1.5708) + L_m^2)/ L_1))-2 * L_a2 * ...
    (acos(z2 / L_a2) + 1.5708))/(2 * L_a2 + L_m - 2 * L_1);


theta_2 = max(90, min(179, theta_2));
%theta_2 = 135;
disp(['theta_2: ', num2str(theta_2)]);

%Link 3 Theta Calculations
L_a3 = sqrt(x3^2 + y3^2 + z3^2);


theta_3 = rad2deg((acos((-L_2^2 + (x3^2 + y3^2 + z3^2) + 2*L_m*L_a3 ...
    *cos(acos(z3/L_a3) + 1.5708) + L_m^2)/ L_1))-2 * L_a3 * ...
    (acos(z3 / L_a3) + 1.5708))/(2 * L_a3 + L_m - 2 * L_1);


theta_3 = max(90, min(179, theta_3));
%theta_3 = 135;
disp(['theta_3: ', num2str(theta_3)]);

% Define the base points
base_points = [
    0.10206, 0, 0;
    -0.05103, 0.088386, 0;
    -0.05103, -0.088386, 0;
];

% Calculate the centroid of the base triangle
centroid = mean(base_points(:, 1:2), 1);

% Inward directions in the XY plane for each link at theta = 0
inward_directions = [
    (centroid - base_points(1, 1:2)), 0;
    (centroid - base_points(2, 1:2)), 0;
    (centroid - base_points(3, 1:2)), 0
];

% Normalize inward directions
for i = 1:3
    inward_directions(i,1:2) = inward_directions(i,1:2) / ...
    norm(inward_directions(i,1:2));
end

% Calculate the end points of each green link (first arm) based on theta
link_end_points_1 = zeros(3, 3);
theta = [theta_1, theta_2, theta_3];
for i = 1:3
    theta_val = theta(i);
    % Calculate the end point of the green link in the XY plane
    link_end_points_1(i, 1:2) = base_points(i, 1:2) + L_1 * ...
    cosd(theta_val) * inward_directions(i, 1:2);
    link_end_points_1(i, 3) = L_1 * sind(theta_val); % Z comp
    
    %disp('Link End Points (3x3 matrix):');
    %disp(link_end_points_1);
end

% Define the end points of each pink link (second arm) based on the 
% calculated ball joint coordinates
link_end_points_2 = [
    x1, y1, z1;
    x2, y2, z2;
    x3, y3, z3
];

clf;

% Plot the base of the Stewart platform

plot3(base_points(:,1), base_points(:,2), ...
base_points(:,3), 'c', 'LineWidth', 2);

% Connect the points to form a closed triangle
hold on;
plot3([base_points(1,1), base_points(2,1)], [base_points(1,2), ...
    base_points(2,2)], [base_points(1,3), base_points(2,3)], ...
    'c', 'LineWidth', 2);
plot3([base_points(2,1), base_points(3,1)], [base_points(2,2), ...
    base_points(3,2)], [base_points(2,3), base_points(3,3)], ...
    'c', 'LineWidth', 2);
plot3([base_points(3,1), base_points(1,1)], [base_points(3,2), ...
    base_points(1,2)], [base_points(3,3), base_points(1,3)], ...
    'c', 'LineWidth', 2);

% Plot the green links (first arms)
for i = 1:3
    plot3([base_points(i,1), link_end_points_1(i,1)], ...
          [base_points(i,2), link_end_points_1(i,2)], ...
          [base_points(i,3), link_end_points_1(i,3)], 'g', ...
          'LineWidth', 2);
end

% Plot the pink links (second arms)
for i = 1:3
    plot3([link_end_points_1(i,1), link_end_points_2(i,1)], ...
          [link_end_points_1(i,2), link_end_points_2(i,2)], ...
          [link_end_points_1(i,3), link_end_points_2(i,3)], 'm', ...
          'LineWidth', 2);
end

% Draw the red shaded triangle connecting the endpoints of the pink links
fill3(link_end_points_2(:,1), link_end_points_2(:,2), ...
    link_end_points_2(:,3), 'r', 'FaceAlpha', 0.5);

% Set axis limits, labels, and reverse the X-axis direction
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Stewart Platform');
set(gca, 'XDir', 'reverse'); % Flip the X-axis direction
axis([-0.25 0.25 -0.25 0.25 -0.25 0.25]);
camzoom(2);

disp('----------------------------------------------');
pause(delay);
end

