% ANIMATE6DOF Illustrative joint-motion demonstration, not planned motion.
% Run from MATLAB/ folder. Set exportVideo=false to preview only.
clear; clc; close all;
exportVideo = true;
outputFile = '6DOF_DH_Kinematics_Demo.mp4';
fps = 24; durationSeconds = 12;
nFrames = fps * durationSeconds;
times = linspace(0,durationSeconds,nFrames);
% Smooth periodic example trajectories, in radians (NOT from assignment).
qFun = @(t) [0.55*sin(2*pi*t/durationSeconds), ...
    0.35*sin(2*pi*t/durationSeconds+0.5), ...
    0.40*sin(2*pi*t/durationSeconds+1.0), ...
    0.65*sin(4*pi*t/durationSeconds), ...
    0.40*sin(2*pi*t/durationSeconds+1.5), ...
    0.70*sin(4*pi*t/durationSeconds+0.3)];
% Determine stable axis bounds from every pose.
allP=zeros(nFrames*8,3);
for k=1:nFrames
    [~, P] = robotTransforms6DOF(qFun(times(k)));
    allP((k-1)*8+(1:8),:)=P;
end
lo=min(allP,[],1)-250; hi=max(allP,[],1)+250;
center=(lo+hi)/2; span=max(hi-lo)/2;
fig=figure('Color','w','Position',[100 100 1280 720], ...
    'Name','6-DOF Robot DH Kinematics Animation');
ax=axes('Parent',fig,'Position',[0.07 0.12 0.62 0.78]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlim(ax,center(1)+[-span span]);
ylim(ax,center(2)+[-span span]);
zlim(ax,center(3)+[-span span]);
view(ax,45,25); axis(ax,'manual');
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
title(ax,'6-DOF Robot | Forward Kinematics');
arm=plot3(ax,nan,nan,nan,'-o','Color',[0.95 0.45 0.1], ...
    'LineWidth',4,'MarkerSize',7,'MarkerFaceColor','k');
trace=plot3(ax,nan,nan,nan,'--','Color',[0.2 0.45 0.8],'LineWidth',1.4);
frameAxes=gobjects(8,3);
colors={'r','g','b'}; scale=150;
for j=1:8
    for a=1:3
        frameAxes(j,a)=quiver3(ax,0,0,0,0,0,0,0,colors{a},'LineWidth',1);
    end
end
textPanel=annotation(fig,'textbox',[0.73 0.25 0.25 0.50], ...
    'String','', 'FontSize',12,'EdgeColor','none','Interpreter','none');
annotation(fig,'textbox',[0.72 0.78 0.26 0.10], ...
    'String','Joint angles (degrees)','FontWeight','bold', ...
    'FontSize',14,'EdgeColor','none');
trail=zeros(nFrames,3);
if exportVideo
    try
        writer=VideoWriter(outputFile,'MPEG-4'); writer.FrameRate=fps;
        open(writer);
    catch ME
        error('MP4 export unavailable: %s. Set exportVideo=false to preview.',ME.message);
    end
end
for k=1:nFrames
    q=qFun(times(k)); [T,P]=robotTransforms6DOF(q);
    set(arm,'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
    trail(k,:)=P(end,:);
    set(trace,'XData',trail(1:k,1),'YData',trail(1:k,2), ...
        'ZData',trail(1:k,3));
    for j=1:8
        o=T{j}(1:3,4); R=T{j}(1:3,1:3);
        for a=1:3
            set(frameAxes(j,a),'XData',o(1),'YData',o(2),'ZData',o(3), ...
                'UData',scale*R(1,a),'VData',scale*R(2,a), ...
                'WData',scale*R(3,a));
        end
    end
    angles=rad2deg(q);
    textPanel.String=sprintf(['Time: %5.2f s\n\n' ...
        'q1: %+6.1f deg\nq2: %+6.1f deg\nq3: %+6.1f deg\n' ...
        'q4: %+6.1f deg\nq5: %+6.1f deg\nq6: %+6.1f deg\n\n' ...
        'Illustrative joint trajectories'],times(k),angles);
    drawnow;
    if exportVideo, writeVideo(writer,getframe(fig)); end
end
if exportVideo
    close(writer);
    fprintf('Saved video: %s\n',fullfile(pwd,outputFile));
end
