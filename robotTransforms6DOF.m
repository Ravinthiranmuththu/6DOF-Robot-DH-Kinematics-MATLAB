function [T_all, positions] = robotTransforms6DOF(q)
% ROBOTTRANSFORMS6DOF Original Assignment 03 transforms, dimensions in mm.
% q: six joint angles in radians, row or column vector.
% T_all: eight frames (base, offset, six successive transformations).
% positions: corresponding 8-by-3 frame origins in mm.
validateattributes(q, {'numeric'}, {'real','finite','vector','numel',6});
q = q(:); t1=q(1); t2=q(2); t3=q(3);
t4=q(4); t5=q(5); t6=q(6);
B = [1 0 0 0; 0 1 0 0; 0 0 1 220; 0 0 0 1];
A1 = [cos(t1) 0 sin(t1) 320*cos(t1)+125*sin(t1); ...
      sin(t1) 0 -cos(t1) 320*sin(t1)-125*cos(t1); ...
      0 1 0 460; 0 0 0 1];
A2 = [cos(t2) -sin(t2) 0 -975*sin(t2); ...
      sin(t2) cos(t2) 0 975*cos(t2); ...
      0 0 1 85; 0 0 0 1];
A3 = [-sin(t3) 0 cos(t3) 187*cos(t3)-200*sin(t3); ...
      cos(t3) 0 sin(t3) 187*sin(t3)+200*cos(t3); ...
      0 1 0 -210; 0 0 0 1];
A4 = [cos(t4) 0 -sin(t4) 0; sin(t4) 0 cos(t4) 0; ...
      0 -1 0 887; 0 0 0 1];
A5 = [cos(t5) 0 sin(t5) 0; sin(t5) 0 -cos(t5) 0; ...
      0 1 0 0; 0 0 0 1];
A6 = [cos(t6) 0 -sin(t6) 0; sin(t6) 0 cos(t6) 0; ...
      0 -1 0 200; 0 0 0 1];
relative = {B,A1,A2,A3,A4,A5,A6};
T_all = cell(8,1); T_all{1}=eye(4);
positions = zeros(8,3);
for k=1:7
    T_all{k+1}=T_all{k}*relative{k};
    positions(k+1,:)=T_all{k+1}(1:3,4).';
end
end
