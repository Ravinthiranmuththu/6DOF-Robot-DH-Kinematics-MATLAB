function [fig, ax] = plotRobot6DOF(q)
% PLOTROBOT6DOF Static wireframe and local coordinate frames (mm).
if nargin==0, q=zeros(1,6); end
[T, P] = robotTransforms6DOF(q);
fig=figure('Color','w','Name','6-DOF DH Kinematics');
ax=axes('Parent',fig); hold(ax,'on');
plot3(ax,P(:,1),P(:,2),P(:,3),'-o','Color',[0.94 0.42 0.10], ...
    'LineWidth',4,'MarkerSize',7,'MarkerFaceColor',[0.1 0.1 0.1], ...
    'MarkerEdgeColor',[0.1 0.1 0.1]);
scale=160;
for k=1:numel(T)
    origin=T{k}(1:3,4); R=T{k}(1:3,1:3);
    quiver3(ax,origin(1),origin(2),origin(3),scale*R(1,1),scale*R(2,1),scale*R(3,1),0,'r','LineWidth',1);
    quiver3(ax,origin(1),origin(2),origin(3),scale*R(1,2),scale*R(2,2),scale*R(3,2),0,'g','LineWidth',1);
    quiver3(ax,origin(1),origin(2),origin(3),scale*R(1,3),scale*R(2,3),scale*R(3,3),0,'b','LineWidth',1);
end
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
title(ax,'6-DOF Robot | DH Forward Kinematics');
grid(ax,'on'); axis(ax,'equal'); view(ax,45,30);
hold(ax,'off');
end
