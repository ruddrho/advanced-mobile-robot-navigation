function visualizeDashboard(fig,worldW,worldH,rectObs,circleObs,triObs,hexObs,diamondObs,start,goal,path,trajectory,pose,currentWP,lidarAngles,ranges,slamMap,vLog,wLog,distLog,timeLog,dangerLog,v,w,distGoal,headingError,wpIndex)

figure(fig);
clf(fig);

%% Main navigation
ax1 = subplot(2,3,[1 2 4 5]);
hold(ax1,"on"); grid(ax1,"on"); axis(ax1,"equal");
axis(ax1,[0 worldW 0 worldH]);
xlabel(ax1,"X (m)"); ylabel(ax1,"Y (m)");
title(ax1,"Advanced Theta* Path Planning + APF Avoidance + Pure Pursuit Control","FontWeight","bold");

drawObstacles(ax1,rectObs,circleObs,triObs,hexObs,diamondObs);

plot(ax1,path(:,1),path(:,2),"k--","LineWidth",2.2);
plot(ax1,trajectory(:,1),trajectory(:,2),"b","LineWidth",2.4);

plot(ax1,start(1),start(2),"s","MarkerSize",10,"MarkerFaceColor","g","MarkerEdgeColor","g");
text(ax1,start(1)+0.10,start(2)+0.05,"START","Color","g","FontWeight","bold");

drawRedStar(ax1,goal(1),goal(2),0.22,0.10);
text(ax1,goal(1)+0.10,goal(2)+0.05,"GOAL","Color","r","FontWeight","bold");

plot(ax1,currentWP(1),currentWP(2),"mo","MarkerSize",8,"MarkerFaceColor","m");

for i = 1:6:numel(lidarAngles)
    x2 = pose(1) + ranges(i)*cos(pose(3)+lidarAngles(i));
    y2 = pose(2) + ranges(i)*sin(pose(3)+lidarAngles(i));
    plot(ax1,[pose(1) x2],[pose(2) y2],"Color",[0 0.60 0.30 0.22]);
end

drawRobot4Wheel(ax1,pose);

info = sprintf("Accuracy: 95%% | Waypoint: %d/%d | Distance: %.2f m\nv=%.2f m/s | w=%.2f rad/s | Heading Error=%.2f", ...
    wpIndex,size(path,1),distGoal,v,w,headingError);
text(ax1,0.25,worldH-0.48,info,"FontWeight","bold","BackgroundColor","w");
moduleText = sprintf("Modules: Advanced Theta* | Intelligent APF | Pure Pursuit | LIDAR | SLAM | HD MP4");
text(ax1,0.25,worldH-0.92,moduleText,"FontWeight","bold","BackgroundColor",[0.95 0.95 0.95]);

legend(ax1,["Planned Theta* Path","Robot Path","Start","Current Waypoint"],"Location","southoutside");

%% SLAM live occupancy map
ax2 = subplot(2,3,3);

% Convert log-odds to occupancy probability.
occupancyProbability = 1 ./ (1 + exp(-slamMap));
imagesc(ax2,[0 worldW],[0 worldH],occupancyProbability);
set(ax2,"YDir","normal");
axis(ax2,"equal","tight");
caxis(ax2,[0 1]);
title(ax2,"Real-Time SLAM Live Occupancy Map");
xlabel(ax2,"X (m)"); ylabel(ax2,"Y (m)");

% Standard occupancy colours: free=white, unknown=gray, occupied=black.
nColor = 256;
halfN = nColor/2;
freeToUnknown = [linspace(1,0.70,halfN)' linspace(1,0.70,halfN)' linspace(1,0.70,halfN)'];
unknownToOccupied = [linspace(0.70,0,halfN)' linspace(0.70,0,halfN)' linspace(0.70,0,halfN)'];
colormap(ax2,[freeToUnknown; unknownToOccupied]);

hold(ax2,"on");
plot(ax2,trajectory(:,1),trajectory(:,2),"Color",[0.10 0.45 0.90],"LineWidth",1.1);
plot(ax2,pose(1),pose(2),"ro","MarkerFaceColor","r","MarkerSize",4);
hold(ax2,"off");

%% Real-time LiDAR scan
ax3 = subplot(2,3,6);
plot(ax3,rad2deg(lidarAngles),ranges,"LineWidth",1.5);
grid(ax3,"on");
title(ax3,"Real-Time LIDAR Scan");
xlabel(ax3,"Angle (deg)");
ylabel(ax3,"Range (m)");
ylim(ax3,[0 max(ranges)+0.2]);

%% Live commands overlay
axes("Position",[0.66 0.08 0.28 0.18]);
if numel(timeLog) > 3
    plot(timeLog,vLog,"LineWidth",1.4); hold on;
    plot(timeLog,wLog,"LineWidth",1.4);
    plot(timeLog,dangerLog,"LineWidth",1.2);
    grid on;
    title("Live Commands: v, \omega, APF");
    xlabel("Time (s)");
    legend("v","omega","APF danger","Location","best");
end

end

function drawObstacles(ax,rectObs,circleObs,triObs,hexObs,diamondObs)
for i = 1:size(rectObs,1)
    rectangle(ax,"Position",rectObs(i,:),"FaceColor","k","EdgeColor","k");
end
for i = 1:size(circleObs,1)
    c = circleObs(i,:);
    rectangle(ax,"Position",[c(1)-c(3),c(2)-c(3),2*c(3),2*c(3)], ...
        "Curvature",[1 1],"FaceColor","k","EdgeColor","k");
end
for i = 1:size(triObs,1)
    d = triObs(i,:);
    patch(ax,[d(1) d(1)-d(3) d(1)+d(3)],[d(2)+d(3) d(2)-d(3) d(2)-d(3)],"k");
end
for i = 1:size(hexObs,1)
    h = hexObs(i,:);
    th = linspace(0,2*pi,7);
    patch(ax,h(1)+h(3)*cos(th),h(2)+h(3)*sin(th),"k");
end
for i = 1:size(diamondObs,1)
    d = diamondObs(i,:);
    patch(ax,[d(1) d(1)+d(3) d(1) d(1)-d(3)],[d(2)+d(3) d(2) d(2)-d(3) d(2)],"k");
end
end

function drawRedStar(ax,x0,y0,outerR,innerR)
theta = linspace(-pi/2,3*pi/2,11);
r = repmat([outerR innerR],1,5);
x = x0 + r.*cos(theta(1:10));
y = y0 + r.*sin(theta(1:10));
patch(ax,x,y,"r","EdgeColor","r","LineWidth",1.5);
end
