% Static zero-configuration demonstration (Assignment 03 baseline).
clear; clc; close all;
q = zeros(1,6); % radians
[T_all, positions] = robotTransforms6DOF(q);
disp('End-effector homogeneous transform:'); disp(T_all{end});
disp('Frame origins in millimetres:'); disp(positions);
plotRobot6DOF(q);
