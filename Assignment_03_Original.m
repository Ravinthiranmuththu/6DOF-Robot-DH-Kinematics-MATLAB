%% E7215/EC7215: Robotics and Automation 
%%Assignment #3: DH Conventions
% EG/2020/4164
%Q2:
% Clear workspace and figure
clear; clc; close all;


% Initialize joint angles (in radians)
theta1 = deg2rad(0);
theta2 = deg2rad(0);
theta3 = deg2rad(0);
theta4 = deg2rad(0);
theta5 = deg2rad(0);
theta6 = deg2rad(0);

%% Define relative Homogeneous Transformation Matrices for Each Joint
HTMbase_1 = [1 0 0 0; 0 1 0 0; 0 0 1 220; 0 0 0 1;];

% Existing matrices 
HTM_12 = [cos(theta1) 0  sin(theta1)  0.320e3*cos(theta1) + 0.125e3*sin(theta1);
          sin(theta1) 0 -cos(theta1)  0.320e3*sin(theta1) - 0.125e3*cos(theta1);
          0          1  0             460;
          0          0  0             1];

HTM_23 = [cos(theta2) -sin(theta2) 0  -0.975e3*sin(theta2);
          sin(theta2)  cos(theta2) 0   0.975e3*cos(theta2);
          0            0           1   85;
          0            0           0   1];

HTM_34 = [-sin(theta3) 0 cos(theta3)  0.187e3*cos(theta3) - 0.200e3*sin(theta3);
           cos(theta3) 0 sin(theta3)  0.187e3*sin(theta3) + 0.200e3*cos(theta3);
           0           1 0           -210;
           0           0 0            1];


d4 = 887;  % mm ( DH row for joint 4)
HTM_45 = [cos(theta4)  0  -sin(theta4)  0;
          sin(theta4)  0   cos(theta4)  0;
          0           -1   0            d4;
          0            0   0            1];

% Joint 5 DH row: a5=0, d5=0, alpha5=+90
HTM_56 = [cos(theta5)  0  sin(theta5)  0;
          sin(theta5)  0 -cos(theta5)  0;
          0            1  0            0;
          0            0  0            1];

% Joint 6 DH row: a6=0, d6=200, alpha6 = -90  
d6 = 200;
HTM_6E = [cos(theta6)  0  -sin(theta6)  0;
          sin(theta6)  0   cos(theta6)  0;
          0           -1   0            d6;
          0            0   0            1];

% Store each joint's transformation in a cell array.
T_cell = {HTMbase_1, HTM_12, HTM_23, HTM_34, HTM_45, HTM_56, HTM_6E};
%% ComputATION O Cumulative Transformation and Extract Joint Positions
numJoints = length(T_cell);
T_total = eye(4);
positions = zeros(numJoints+1, 3); 
T_all = cell(numJoints+1, 1);
T_all{1} = T_total;
positions(1,:) = T_total(1:3,4)';

for i = 1:numJoints
    T_total = T_total * T_cell{i};
    T_all{i+1} = T_total;
    positions(i+1,:) = T_total(1:3,4)';
end

%% Plot the Robot Arm
figure;
hold on;
nPoints = 100;  
arrowScale = 200; 

% Plot each link as a smooth line (orange color)
for i = 1:size(positions,1)-1
    x_line = linspace(positions(i,1), positions(i+1,1), nPoints);
    y_line = linspace(positions(i,2), positions(i+1,2), nPoints);
    z_line = linspace(positions(i,3), positions(i+1,3), nPoints);
    plot3(x_line, y_line, z_line, 'Color', [1, 0.5, 0], 'LineWidth', 10);
end

% Plot joint positions (black circles)
plot3(positions(:,1), positions(:,2), positions(:,3), 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k');

% Visualize joint orientations using arrows and axis colors
for i = 1:length(T_all)
    T_current = T_all{i};
    origin = T_current(1:3,4);
    x_axis = T_current(1:3,1);
    y_axis = T_current(1:3,2);
    z_axis = T_current(1:3,3);
    
    % Plot arrows for x (red), y (green), and z (blue) axes
    quiver3(origin(1), origin(2), origin(3), arrowScale*x_axis(1), arrowScale*x_axis(2), arrowScale*x_axis(3), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    quiver3(origin(1), origin(2), origin(3), arrowScale*y_axis(1), arrowScale*y_axis(2), arrowScale*y_axis(3), 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 1);
    quiver3(origin(1), origin(2), origin(3), arrowScale*z_axis(1), arrowScale*z_axis(2), arrowScale*z_axis(3), 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
end

%% Formatting the Plot
xlim([-100 1700]);
ylim([-300 600]);
zlim([-100 2600]);

xlabel('X-axis','Color','g','FontSize',12);
ylabel('Y-axis','Color','r','FontSize',12);
zlabel('Z-axis','Color','b','FontSize',12);
title('EE7215 4164 ASSIGNMENT03','FontSize',14,'FontWeight','bold');
axis equal;
grid on;

% --- Isometric view settings ---
view(3);            % ensure 3D view
view(45, 35.264);   % true isometric angles
camproj('orthographic');  % remove perspective distortion (typical for isometric)
axis vis3d;         % keep aspect ratio fixed during rotation/zoom
hold off;