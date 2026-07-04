function ranges = simulateLidar(pose,rectObs,circleObs,triObs,hexObs,diamondObs,angles,maxRange,worldW,worldH)
% Ray-cast LiDAR against all obstacle shapes.

ranges = maxRange*ones(size(angles));

for i = 1:numel(angles)
    th = pose(3) + angles(i);

    for r = 0:0.025:maxRange
        x = pose(1) + r*cos(th);
        y = pose(2) + r*sin(th);

        if x <= 0 || y <= 0 || x >= worldW || y >= worldH
            ranges(i) = r;
            break;
        end

        p = [x y];
        if pointInRectangles(p,rectObs) || pointInCircles(p,circleObs) || ...
           pointInTriangles(p,triObs) || pointInHexagons(p,hexObs) || pointInDiamonds(p,diamondObs)
            ranges(i) = r;
            break;
        end
    end
end
end

function inside = pointInRectangles(p,rects)
inside = false;
for j = 1:size(rects,1)
    x=rects(j,1); y=rects(j,2); w=rects(j,3); h=rects(j,4);
    if p(1)>=x && p(1)<=x+w && p(2)>=y && p(2)<=y+h
        inside = true; return;
    end
end
end

function inside = pointInCircles(p,circles)
inside = false;
for j = 1:size(circles,1)
    if norm(p - circles(j,1:2)) <= circles(j,3)
        inside = true; return;
    end
end
end

function inside = pointInTriangles(p,tris)
inside = false;
for j = 1:size(tris,1)
    x0=tris(j,1); y0=tris(j,2); s=tris(j,3);
    xv=[x0 x0-s x0+s];
    yv=[y0+s y0-s y0-s];
    if inpolygon(p(1),p(2),xv,yv)
        inside = true; return;
    end
end
end

function inside = pointInHexagons(p,hexs)
inside = false;
for j = 1:size(hexs,1)
    x0=hexs(j,1); y0=hexs(j,2); r=hexs(j,3);
    th=linspace(0,2*pi,7);
    if inpolygon(p(1),p(2),x0+r*cos(th),y0+r*sin(th))
        inside = true; return;
    end
end
end

function inside = pointInDiamonds(p,diamonds)
inside = false;
for j = 1:size(diamonds,1)
    x0=diamonds(j,1); y0=diamonds(j,2); s=diamonds(j,3);
    if inpolygon(p(1),p(2),[x0 x0+s x0 x0-s],[y0+s y0 y0-s y0])
        inside = true; return;
    end
end
end
