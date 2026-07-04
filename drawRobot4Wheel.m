function drawRobot4Wheel(ax,pose)
% Draw 4-wheel robot.

L = 0.48;
W = 0.34;
x = pose(1);
y = pose(2);
th = pose(3);

R = [cos(th) -sin(th); sin(th) cos(th)];

body = [
     L/2  W/2
     L/2 -W/2
    -L/2 -W/2
    -L/2  W/2
]';

bodyW = R*body + [x;y];
patch(ax,bodyW(1,:),bodyW(2,:),[0.00 0.25 1.00],"EdgeColor","k","LineWidth",1.1);

wheelL = 0.16;
wheelW = 0.06;
centers = [
     L/3  W/2+0.04
     L/3 -W/2-0.04
    -L/3  W/2+0.04
    -L/3 -W/2-0.04
];

for i = 1:4
    c = centers(i,:)';
    wheel = [
         wheelL/2  wheelW/2
         wheelL/2 -wheelW/2
        -wheelL/2 -wheelW/2
        -wheelL/2  wheelW/2
    ]';
    p = R*(wheel+c) + [x;y];
    patch(ax,p(1,:),p(2,:),"k");
end

quiver(ax,x,y,0.45*cos(th),0.45*sin(th),0,"r","LineWidth",2);
end
