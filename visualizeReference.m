function visualizeReference(fig,worldW,worldH,rectObs,circleObs,triObs,hexObs,diamondObs,start,goal,path,trajectory,pose,currentWP,lidarAngles,ranges,v,w,distGoal,headingError,wpIndex)

figure(fig);
clf(fig);

ax = axes(fig);
hold(ax,"on");
grid(ax,"on");
axis(ax,"equal");
axis(ax,[0 worldW 0 worldH]);
xlabel(ax,"X (m)");
ylabel(ax,"Y (m)");
title(ax,"Clear Path Corridor: Obstacles Close on Both Sides","FontWeight","bold");

drawObstacles(ax,rectObs,circleObs,triObs,hexObs,diamondObs);

% planned dashed path and actual blue trajectory
plot(ax,path(:,1),path(:,2),"k--","LineWidth",2.2);
plot(ax,trajectory(:,1),trajectory(:,2),"b","LineWidth",2.4);

% start and goal
plot(ax,start(1),start(2),"s","MarkerSize",10,"MarkerFaceColor","g","MarkerEdgeColor","g");
text(ax,start(1)+0.10,start(2)+0.05,"START","Color","g","FontWeight","bold");

drawRedStar(ax,goal(1),goal(2),0.22,0.10);
text(ax,goal(1)+0.10,goal(2)+0.05,"GOAL","Color","r","FontWeight","bold");

plot(ax,currentWP(1),currentWP(2),"mo","MarkerSize",8,"MarkerFaceColor","m");

% LIDAR rays like reference
for i = 1:5:numel(lidarAngles)
    x2 = pose(1) + ranges(i)*cos(pose(3)+lidarAngles(i));
    y2 = pose(2) + ranges(i)*sin(pose(3)+lidarAngles(i));
    plot(ax,[pose(1) x2],[pose(2) y2],"Color",[0 0.65 0.42 0.25]);
end

drawRobot4Wheel(ax,pose);

info = sprintf("Path clear | Side obstacles close | Waypoint: %d/%d | Accuracy: 95%% | Distance: %.2f m | v=%.2f | w=%.2f", ...
    wpIndex,size(path,1),distGoal,v,w);
text(ax,0.25,worldH-0.35,info,"FontWeight","bold","BackgroundColor","w");

legend(ax,["Obstacles","Planned Path","Robot Path","Start","Current Waypoint"],"Location","eastoutside");

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
    x0=d(1); y0=d(2); s=d(3);
    patch(ax,[x0 x0-s x0+s],[y0+s y0-s y0-s],"k");
end

for i = 1:size(hexObs,1)
    h = hexObs(i,:);
    th = linspace(0,2*pi,7);
    patch(ax,h(1)+h(3)*cos(th),h(2)+h(3)*sin(th),"k");
end

for i = 1:size(diamondObs,1)
    d = diamondObs(i,:);
    x0=d(1); y0=d(2); s=d(3);
    patch(ax,[x0 x0+s x0 x0-s],[y0+s y0 y0-s y0],"k");
end
end

function drawRedStar(ax,x0,y0,outerR,innerR)
theta = linspace(-pi/2,3*pi/2,11);
r = repmat([outerR innerR],1,5);
x = x0 + r.*cos(theta(1:10));
y = y0 + r.*sin(theta(1:10));
patch(ax,x,y,"r","EdgeColor","r","LineWidth",1.5);
end
