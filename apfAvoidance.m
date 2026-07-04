function [v,w,danger] = apfAvoidance(~,ranges,angles,maxV,maxW)
%APFAVOIDANCE Stable front-focused LIDAR obstacle avoidance.
%
% Side obstacles do not unnecessarily pull the robot away from the global
% path. Repulsive steering becomes active mainly when an obstacle enters
% the forward driving corridor.

safeDistance = 0.68;
criticalDistance = 0.34;

ranges = max(ranges,0.03);
avoidanceMask = abs(angles) <= deg2rad(78);
avoidanceAngles = angles(avoidanceMask);
avoidanceRanges = ranges(avoidanceMask);

proximity = max(0,(safeDistance-avoidanceRanges)/(safeDistance-criticalDistance));
frontFocus = cos(max(min(avoidanceAngles,pi/2),-pi/2)).^2;
weights = (proximity.^2).*frontFocus./max(avoidanceRanges,0.08);

if sum(weights) > eps
    % Obstacle on the left produces a negative correction (turn right).
    turnMoment = sum(-sin(avoidanceAngles).*weights)/sum(weights);
else
    turnMoment = 0;
end

frontMask = abs(angles) <= deg2rad(32);
leftMask = angles >= deg2rad(8) & angles <= deg2rad(75);
rightMask = angles <= deg2rad(-8) & angles >= deg2rad(-75);

minFront = min(ranges(frontMask));
leftClearance = mean(min(ranges(leftMask),safeDistance));
rightClearance = mean(min(ranges(rightMask),safeDistance));

% A directly-ahead obstacle creates little left/right moment. Select the
% clearer side to prevent hesitation or a local minimum.
if minFront < 0.52 && abs(turnMoment) < 0.10
    clearanceDifference = leftClearance-rightClearance;
    if abs(clearanceDifference) < 0.01
        clearanceDifference = 1; % deterministic tie-break: turn left
    end
    turnMoment = 0.85*sign(clearanceDifference);
end

danger = (safeDistance-minFront)/(safeDistance-criticalDistance);
danger = min(max(danger,0),1);

w = maxW*tanh(2.0*turnMoment);

% Safe forward-speed ceiling.
v = maxV*(1-0.82*danger);
if minFront < criticalDistance
    v = 0;
elseif minFront < 0.43
    v = min(v,0.08);
elseif minFront < 0.52
    v = min(v,0.25*maxV);
end
v = max(0,min(v,maxV));
end
