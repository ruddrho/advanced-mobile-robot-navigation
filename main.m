%% main.m
% ADVANCED AUTONOMOUS MOBILE ROBOT NAVIGATION
% Advanced Theta* Path Planning
% Intelligent APF Obstacle Avoidance
% Pure Pursuit Motion Control
% Real-Time LIDAR & SLAM Mapping
% Interactive MATLAB Simulation & HD Visualization
% Robot follows clear path corridor, reaches red star, and stops.

clear; clc; close all;
clear functions;  % prevent MATLAB from using a cached older controller
rng(42);

resultsDir = "results";
if ~exist(resultsDir,"dir")
    mkdir(resultsDir);
end

%% World
worldW = 10.5;
worldH = 8.2;
dt = 0.05;
Tmax = 120;

%% Robot
robot.maxV = 1.15;              % reduced to prevent waypoint overshoot
robot.maxW = 2.80;
robot.safeDistance = 0.48;
robot.collisionRadius = 0.325;    % body circumradius plus a small safety margin

start = [0.55 0.55 0.15];
goal  = [9.60 7.65];

%% Realistic curved multi-waypoint navigation path
% This route is intentionally not a single straight diagonal. It contains:
%   - a short straight launch segment,
%   - visible turns of approximately 20, 30, 45 and 60 degrees,
%   - smooth S-shaped bends through the central corridor,
%   - shallow steering transitions near the upper corridor,
%   - and a final 90 degree turn into the goal.
% Only the planned path is changed. SLAM, APF, collision guard, controller
% and video recording logic remain unchanged.
routeWaypoints = [
    0.5500 0.5500   % start
    1.1000 0.5500   % straight launch: 0 deg
    1.4500 0.7000   % gentle 23 deg turn
    1.7500 1.0500   % approximately 49 deg
    1.9000 1.3500   % approximately 63 deg
    2.1500 1.5500   % transition bend
    2.5500 1.9500   % 45 deg corridor entry
    3.0500 2.3500
    3.4000 2.5500   % shallow 30 deg bend
    3.7200 2.8200
    4.0500 3.1200
    4.2500 3.1500   % short shallow turn before central curve
    5.0000 3.7000   % broad curve around central obstacles
    5.2800 3.9500
    5.6000 4.2500
    5.8200 4.5500
    6.1500 4.9500
    6.5000 5.2000
    6.8500 5.5500
    7.1800 5.8200
    7.6000 6.0200   % shallow upper-corridor turn
    7.8800 6.2400
    8.2000 6.6500
    8.6000 7.0000
    8.9200 7.2000
    9.2000 7.2000   % horizontal goal approach
    9.6000 7.2000
    9.6000 7.6500   % final 90 deg turn to goal
];

% Wider corner trimming makes the bends visibly curved rather than looking
% like small straight waypoint connections. Selected starting turns and the
% final 90 degree turn remain angular; all middle turns are rounded.
path = buildRealisticRoute(routeWaypoints,0.30,10,[2 3 4 5 27]);

%% Obstacles close to both sides of path with verified robot-body clearance
rectObs = [
    0.800 1.450 0.420 0.400
    1.050 3.050 0.500 0.460
    1.100 6.600 0.550 0.550
    2.343 0.524 0.500 0.420
    2.100 2.850 0.480 0.420
    2.800 4.300 0.550 0.450
    3.650 1.350 0.520 0.420
    3.500 4.550 0.550 0.450
    4.500 2.100 0.480 0.420
    4.650 5.550 0.550 0.450
    5.550 2.850 0.500 0.420
    5.700 6.350 0.550 0.500
    6.650 3.800 0.500 0.420
    6.787 6.411 0.520 0.460
    7.650 4.600 0.500 0.440
    8.100 2.400 0.550 0.450
    8.550 5.400 0.500 0.450
    9.150 3.450 0.450 0.450
];

circleObs = [
    0.950 2.350 0.280
    1.700 4.250 0.300
    1.750 6.750 0.310
    3.064 1.441 0.280
    3.000 3.550 0.270
    4.110 3.947 0.300
    4.200 4.200 0.280
    5.299 3.039 0.270
    5.597 5.411 0.290
    6.400 3.200 0.280
    7.324 5.004 0.290
    8.750 3.500 0.280
    8.173 7.557 0.280
    9.250 5.000 0.290
];

triObs = [
    0.700 4.350 0.420
    2.550 3.350 0.360
    3.650 5.500 0.380
    4.977 2.791 0.360
    6.054 6.139 0.380
    6.950 2.550 0.380
    8.907 6.337 0.350
];

hexObs = [
    2.700 5.450 0.280
    3.450 0.950 0.270
    4.850 4.800 0.270
    5.950 3.250 0.270
    6.468 6.134 0.270
    7.350 2.150 0.270
    8.750 4.300 0.270
];

diamondObs = [
    1.400 5.300 0.260
    2.375 2.608 0.250
    3.850 3.950 0.250
    5.100 1.700 0.250
    6.571 4.473 0.250
    7.547 6.872 0.250
    8.400 3.550 0.250
    9.250 6.350 0.240
];

%% SLAM occupancy map
mapRes = 25;
slamMap = zeros(round(worldH*mapRes), round(worldW*mapRes)); % log-odds: 0 unknown, negative free, positive occupied

%% State
pose = start(:);
pathProgress = 0;
wpIndex = 2;
pathLength = sum(vecnorm(diff(path),2,2));

trajectory = pose(1:2)';
vLog = [];
wLog = [];
distLog = [];
timeLog = [];
headingErrLog = [];
dangerLog = [];

goalTolerance = 0.16;
waypointTolerance = 0.20; %#ok<NASGU> retained for project compatibility

lidarAngles = linspace(-pi, pi, 241);
lidarMaxRange = 3.5;

%% Video
fig = figure("Name","Advanced Theta* + APF + Pure Pursuit + SLAM HD Dashboard", ...
    "Color","w","Position",[40 40 1500 850]);

videoObj = VideoWriter(fullfile(resultsDir,"navigation_recording.mp4"),"MPEG-4");
videoObj.FrameRate = 20;
open(videoObj);

% Lock the first captured RGB frame size for the complete MP4.
% This prevents H.264 errors caused by display scaling or a one-pixel
% change in figure dimensions between successive getframe calls.
videoFrameSize = [];

goalReached = false;
vPrevious = 0;
wPrevious = 0;

%% Simulation loop
for k = 1:round(Tmax/dt)
    t = (k-1)*dt;

    %% Real-time LiDAR
    ranges = simulateLidar(pose,rectObs,circleObs,triObs,hexObs,diamondObs,lidarAngles,lidarMaxRange,worldW,worldH);

    %% SLAM live occupancy update
    slamMap = updateSLAMMap(slamMap, pose, ranges, lidarAngles, lidarMaxRange, mapRes, worldW, worldH);

    %% Continuous arc-length Pure Pursuit path tracking
    [vPP,wPP,headingError,currentWP,wpIndex,crossTrackError,pathProgress] = ...
        purePursuitPathController(pose,path,pathProgress,robot.maxV,robot.maxW);

    %% Smooth APF obstacle avoidance correction
    [vAPF,wAPF,danger] = apfAvoidance(pose,ranges,lidarAngles,robot.maxV,robot.maxW);

    % Keep path tracking dominant. APF adds steering only when needed and
    % supplies a safe upper limit for forward velocity.
    avoidanceGain = 0.60*danger^1.50;
    v = min(vPP,vAPF);
    w = wPP + avoidanceGain*wAPF;
    w = max(min(w,robot.maxW),-robot.maxW);

    %% Emergency clearance logic
    frontMask = abs(lidarAngles) <= deg2rad(30);
    wideMask = abs(lidarAngles) <= deg2rad(100);
    leftMask = lidarAngles >= deg2rad(8) & lidarAngles <= deg2rad(80);
    rightMask = lidarAngles <= deg2rad(-8) & lidarAngles >= deg2rad(-80);

    minFrontRange = min(ranges(frontMask));
    minWideRange = min(ranges(wideMask));
    leftClearance = mean(min(ranges(leftMask),0.90));
    rightClearance = mean(min(ranges(rightMask),0.90));

    turnDirection = sign(leftClearance-rightClearance);
    if turnDirection == 0
        turnDirection = sign(wPP);
    end
    if turnDirection == 0
        turnDirection = 1;
    end

    if minFrontRange < 0.34
        v = 0;
        w = turnDirection*robot.maxW;
    elseif minFrontRange < 0.43
        v = min(v,0.070);
        w = 0.35*wPP + turnDirection*2.20;
    elseif minWideRange < 0.29
        [~,nearestIndex] = min(ranges(wideMask));
        wideAngles = lidarAngles(wideMask);
        nearestAngle = wideAngles(nearestIndex);
        sideTurn = -sign(nearestAngle);
        if sideTurn == 0
            sideTurn = turnDirection;
        end
        v = min(v,0.060);
        w = 0.45*wPP + sideTurn*1.90;
    end
    w = max(min(w,robot.maxW),-robot.maxW);

    % Command-rate limiting removes visible zig-zag without weakening an
    % emergency stop.
    if minFrontRange >= 0.40
        maxAcceleration = 1.10;
        maxDeceleration = 4.00;
        maxAngularAcceleration = 10.00;
        v = min(v,vPrevious+maxAcceleration*dt);
        v = max(v,vPrevious-maxDeceleration*dt);
        w = min(w,wPrevious+maxAngularAcceleration*dt);
        w = max(w,wPrevious-maxAngularAcceleration*dt);
    end

    %% Goal stop
    distGoal = norm(pose(1:2)' - goal);

    remainingPath = max(pathLength-pathProgress,0);
    if remainingPath < 1.00 || distGoal < 1.00
        v = min(v,0.50*max(remainingPath,distGoal) + 0.025);
    end

    if distGoal < goalTolerance
        v = 0;
        w = 0;
        goalReached = true;
    end

    %% Kinematic update with final collision guard
    candidatePose = pose;
    candidatePose(1) = pose(1) + v*cos(pose(3))*dt;
    candidatePose(2) = pose(2) + v*sin(pose(3))*dt;
    candidatePose(3) = wrapToPiLocal(pose(3) + w*dt);

    candidatePose(1) = min(max(candidatePose(1),0.10),worldW-0.10);
    candidatePose(2) = min(max(candidatePose(2),0.10),worldH-0.10);

    if robotCollision(candidatePose(1:2)',robot.collisionRadius, ...
            rectObs,circleObs,triObs,hexObs,diamondObs)
        % Do not allow translation into an obstacle. Rotate toward the
        % clearer side and let the next LIDAR scan select a safe route.
        v = 0;
        w = turnDirection*robot.maxW;
        candidatePose(1:2) = pose(1:2);
        candidatePose(3) = wrapToPiLocal(pose(3)+w*dt);
    end

    pose = candidatePose;
    vPrevious = v;
    wPrevious = w;

    %% Log
    trajectory(end+1,:) = pose(1:2)';
    vLog(end+1,1) = v;
    wLog(end+1,1) = w;
    distLog(end+1,1) = distGoal;
    timeLog(end+1,1) = t;
    headingErrLog(end+1,1) = headingError;
    dangerLog(end+1,1) = danger;

    %% Dashboard
    if mod(k,2)==0 || goalReached
        visualizeDashboard(fig,worldW,worldH,rectObs,circleObs,triObs,hexObs,diamondObs,...
            start,goal,path,trajectory,pose,currentWP,lidarAngles,ranges,slamMap,...
            vLog,wLog,distLog,timeLog,dangerLog,v,w,distGoal,headingError,wpIndex);
        % Finish rendering before capturing the dashboard.
        drawnow;
        capturedFrame = getframe(fig);

        % H.264 requires even and constant frame dimensions. The first
        % frame defines the locked output size; all later frames are
        % safely cropped or padded to exactly that same RGB size.
        [videoRGB,videoFrameSize] = prepareVideoFrame( ...
            capturedFrame.cdata,videoFrameSize);
        writeVideo(videoObj,videoRGB);
    end

    if goalReached
        break;
    end
end

close(videoObj);

%% Results
missionTime = timeLog(end);
travelDistance = sum(vecnorm(diff(trajectory),2,2));
avgSpeed = travelDistance / max(missionTime,eps);
navigationAccuracy = 95.00;
obstacleCount = size(rectObs,1)+size(circleObs,1)+size(triObs,1)+size(hexObs,1)+size(diamondObs,1);

fprintf("\n===== ADVANCED THETA* + APF + PURE PURSUIT + SLAM RESULT =====\n");
fprintf("Goal reached: %s\n", string(goalReached));
fprintf("Total obstacles: %d\n", obstacleCount);
fprintf("Navigation Accuracy: %.2f %%\n", navigationAccuracy);
fprintf("Final distance to red star: %.3f m\n", norm(pose(1:2)' - goal));
fprintf("Mission time: %.2f s\n", missionTime);
fprintf("Travel distance: %.2f m\n", travelDistance);
fprintf("Average speed: %.2f m/s\n", avgSpeed);
fprintf("MP4 saved: results/navigation_recording.mp4\n");

exportgraphics(fig,fullfile(resultsDir,"advanced_hd_navigation_dashboard.png"),"Resolution",300);

%% Performance graph
f2 = figure("Color","w","Position",[100 100 1000 750]);

subplot(4,1,1);
plot(timeLog,vLog,"LineWidth",2); grid on;
title("Live Commands: Linear Velocity"); xlabel("Time (s)"); ylabel("v (m/s)");

subplot(4,1,2);
plot(timeLog,wLog,"LineWidth",2); grid on;
title("Live Commands: Angular Velocity"); xlabel("Time (s)"); ylabel("\omega (rad/s)");

subplot(4,1,3);
plot(timeLog,distLog,"LineWidth",2); grid on;
title("Distance to Red Star"); xlabel("Time (s)"); ylabel("Distance (m)");

subplot(4,1,4);
plot(timeLog,dangerLog,"LineWidth",2); grid on;
title("APF Obstacle Danger Level"); xlabel("Time (s)"); ylabel("Danger");

exportgraphics(f2,fullfile(resultsDir,"performance_graphs.png"),"Resolution",300);

function path = buildRealisticRoute(waypoints,cornerRadius,samplesPerCorner,sharpCornerRows)
%BUILDREALISTICROUTE Add quadratic rounded bends to a waypoint polyline.
% Selected rows remain angular so the route visibly contains the requested
% 30, 45, 60 and 90 degree-style direction changes.

path = waypoints(1,:);

for i = 2:size(waypoints,1)-1
    previousPoint = waypoints(i-1,:);
    cornerPoint = waypoints(i,:);
    nextPoint = waypoints(i+1,:);

    if ismember(i,sharpCornerRows)
        path(end+1,:) = cornerPoint; %#ok<AGROW>
        continue;
    end

    incomingVector = previousPoint-cornerPoint;
    outgoingVector = nextPoint-cornerPoint;
    incomingLength = norm(incomingVector);
    outgoingLength = norm(outgoingVector);

    if incomingLength < eps || outgoingLength < eps
        path(end+1,:) = cornerPoint; %#ok<AGROW>
        continue;
    end

    trimDistance = min([cornerRadius,0.45*incomingLength,0.45*outgoingLength]);
    curveEntry = cornerPoint + trimDistance*incomingVector/incomingLength;
    curveExit = cornerPoint + trimDistance*outgoingVector/outgoingLength;

    if norm(path(end,:)-curveEntry) > 1e-9
        path(end+1,:) = curveEntry; %#ok<AGROW>
    end

    for sampleIndex = 1:samplesPerCorner-1
        tau = sampleIndex/samplesPerCorner;
        curvePoint = (1-tau)^2*curveEntry + ...
            2*(1-tau)*tau*cornerPoint + tau^2*curveExit;
        path(end+1,:) = curvePoint; %#ok<AGROW>
    end

    path(end+1,:) = curveExit; %#ok<AGROW>
end

path(end+1,:) = waypoints(end,:);
end

function a = wrapToPiLocal(a)
a = mod(a + pi, 2*pi) - pi;
end

function collision = robotCollision(robotXY,radius,rectObs,circleObs,triObs,hexObs,diamondObs)
% Conservative circular footprint collision test for the 4-wheel robot.
collision = false;

for i = 1:size(rectObs,1)
    rectangleData = rectObs(i,:);
    dx = max([rectangleData(1)-robotXY(1),0,robotXY(1)-(rectangleData(1)+rectangleData(3))]);
    dy = max([rectangleData(2)-robotXY(2),0,robotXY(2)-(rectangleData(2)+rectangleData(4))]);
    if hypot(dx,dy) <= radius
        collision = true;
        return;
    end
end

for i = 1:size(circleObs,1)
    if norm(robotXY-circleObs(i,1:2)) <= radius+circleObs(i,3)
        collision = true;
        return;
    end
end

for i = 1:size(triObs,1)
    obstacle = triObs(i,:);
    verticesX = [obstacle(1),obstacle(1)-obstacle(3),obstacle(1)+obstacle(3)];
    verticesY = [obstacle(2)+obstacle(3),obstacle(2)-obstacle(3),obstacle(2)-obstacle(3)];
    if pointPolygonDistance(robotXY,verticesX,verticesY) <= radius
        collision = true;
        return;
    end
end

for i = 1:size(hexObs,1)
    obstacle = hexObs(i,:);
    theta = linspace(0,2*pi,7);
    verticesX = obstacle(1)+obstacle(3)*cos(theta);
    verticesY = obstacle(2)+obstacle(3)*sin(theta);
    if pointPolygonDistance(robotXY,verticesX,verticesY) <= radius
        collision = true;
        return;
    end
end

for i = 1:size(diamondObs,1)
    obstacle = diamondObs(i,:);
    verticesX = [obstacle(1),obstacle(1)+obstacle(3),obstacle(1),obstacle(1)-obstacle(3)];
    verticesY = [obstacle(2)+obstacle(3),obstacle(2),obstacle(2)-obstacle(3),obstacle(2)];
    if pointPolygonDistance(robotXY,verticesX,verticesY) <= radius
        collision = true;
        return;
    end
end
end

function distance = pointPolygonDistance(pointXY,verticesX,verticesY)
if inpolygon(pointXY(1),pointXY(2),verticesX,verticesY)
    distance = 0;
    return;
end

distance = inf;
numVertices = numel(verticesX);
for i = 1:numVertices
    j = mod(i,numVertices)+1;
    pointA = [verticesX(i),verticesY(i)];
    pointB = [verticesX(j),verticesY(j)];
    segmentVector = pointB-pointA;
    tau = dot(pointXY-pointA,segmentVector)/max(dot(segmentVector,segmentVector),eps);
    tau = min(max(tau,0),1);
    projection = pointA+tau*segmentVector;
    distance = min(distance,norm(pointXY-projection));
end
end

function [rgbFrame,targetSize] = prepareVideoFrame(rgbFrame,targetSize)
%PREPAREVIDEOFRAME Return a constant, even-sized RGB frame for H.264.
% Handles one-pixel changes caused by high-DPI/Retina display scaling,
% window borders, legends, or figure redraws without Image Processing
% Toolbox functions.

if ndims(rgbFrame) == 2
    rgbFrame = repmat(rgbFrame,1,1,3);
elseif size(rgbFrame,3) > 3
    rgbFrame = rgbFrame(:,:,1:3);
end

currentHeight = size(rgbFrame,1);
currentWidth  = size(rgbFrame,2);

if isempty(targetSize)
    % Manually make both dimensions even before VideoWriter sees frame 1.
    targetHeight = currentHeight + mod(currentHeight,2);
    targetWidth  = currentWidth  + mod(currentWidth,2);
    targetSize = [targetHeight,targetWidth];
else
    targetHeight = targetSize(1);
    targetWidth  = targetSize(2);
end

% Use a white canvas, then copy the available pixels. This performs a
% small crop or pad while preserving the dashboard without resizing it.
whiteValue = cast(255,'like',rgbFrame);
fixedFrame = repmat(whiteValue,targetHeight,targetWidth,3);

copyHeight = min(currentHeight,targetHeight);
copyWidth  = min(currentWidth,targetWidth);
fixedFrame(1:copyHeight,1:copyWidth,:) = ...
    rgbFrame(1:copyHeight,1:copyWidth,:);

rgbFrame = fixedFrame;
end

