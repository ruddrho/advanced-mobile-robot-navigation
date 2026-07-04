function [v,w,headingError,targetPoint,wpIndex,crossTrackError] = waypointController(pose,path,wpIndex,maxV,maxW)
%WAYPOINTCONTROLLER Stable polyline look-ahead path tracker.
%
% Instead of chasing one waypoint at full speed, the controller:
%   1) projects the robot onto the nearby path segments,
%   2) selects a smooth look-ahead target on the polyline,
%   3) slows down for heading error, curvature and cross-track error.
%
% This greatly reduces waypoint overshoot and zig-zag motion.

robotXY = pose(1:2)';
numPoints = size(path,1);

if numPoints < 2
    targetPoint = path(1,:);
    headingError = 0;
    crossTrackError = 0;
    v = 0;
    w = 0;
    wpIndex = 1;
    return;
end

wpIndex = min(max(round(wpIndex),2),numPoints);
searchStart = max(1,wpIndex-2);
searchEnd = min(numPoints-1,wpIndex+4);

bestDistance = inf;
bestSegment = max(1,wpIndex-1);
bestProjection = path(bestSegment,:);

% Find the closest projection on a small forward path window.
for segmentIndex = searchStart:searchEnd
    pointA = path(segmentIndex,:);
    pointB = path(segmentIndex+1,:);
    segmentVector = pointB-pointA;
    segmentLengthSquared = dot(segmentVector,segmentVector);

    if segmentLengthSquared < eps
        continue;
    end

    tau = dot(robotXY-pointA,segmentVector)/segmentLengthSquared;
    tau = min(max(tau,0),1);
    projection = pointA + tau*segmentVector;
    projectionDistance = norm(robotXY-projection);

    if projectionDistance < bestDistance
        bestDistance = projectionDistance;
        bestSegment = segmentIndex;
        bestProjection = projection;
    end
end

% Never command the robot back to an already completed path section.
activeSegment = max(bestSegment,wpIndex-1);
activeSegment = min(activeSegment,numPoints-1);

if activeSegment ~= bestSegment
    pointA = path(activeSegment,:);
    pointB = path(activeSegment+1,:);
    segmentVector = pointB-pointA;
    tau = dot(robotXY-pointA,segmentVector)/max(dot(segmentVector,segmentVector),eps);
    tau = min(max(tau,0),1);
    bestProjection = pointA + tau*segmentVector;
end

wpIndex = max(wpIndex,activeSegment+1);

% Signed cross-track error relative to the active path segment.
activeVector = path(activeSegment+1,:)-path(activeSegment,:);
activeLength = max(norm(activeVector),eps);
robotOffset = robotXY-bestProjection;
crossTrackError = (activeVector(1)*robotOffset(2)-activeVector(2)*robotOffset(1))/activeLength;

% Adaptive look-ahead: shorter while recovering, longer on a clean path.
lookAhead = 0.48 + 0.12*min(maxV/1.5,1);
lookAhead = max(0.38,lookAhead-0.22*min(abs(crossTrackError),0.60));

% Walk forward on the polyline to obtain the look-ahead target.
targetPoint = bestProjection;
remainingDistance = lookAhead;
segmentIndex = activeSegment;
segmentStart = bestProjection;

while segmentIndex <= numPoints-1
    segmentEnd = path(segmentIndex+1,:);
    availableDistance = norm(segmentEnd-segmentStart);

    if remainingDistance <= availableDistance && availableDistance > eps
        targetPoint = segmentStart + (remainingDistance/availableDistance)*(segmentEnd-segmentStart);
        break;
    end

    remainingDistance = remainingDistance-availableDistance;
    targetPoint = segmentEnd;
    segmentIndex = segmentIndex+1;

    if segmentIndex <= numPoints-1
        segmentStart = path(segmentIndex,:);
    end
end

wpIndex = max(wpIndex,min(numPoints,segmentIndex+1));

dx = targetPoint(1)-pose(1);
dy = targetPoint(2)-pose(2);
desiredHeading = atan2(dy,dx);
headingError = wrapToPiLocal(desiredHeading-pose(3));

% Pure-pursuit curvature with additional heading damping.
curvature = 2*sin(headingError)/max(lookAhead,0.20);

headingScale = max(0.12,cos(min(abs(headingError),pi/2))^2);
curvatureScale = 1/(1+0.75*abs(curvature));
trackScale = max(0.42,1-0.85*min(abs(crossTrackError),0.65));

v = maxV*headingScale*curvatureScale*trackScale;

if abs(headingError) > 1.10
    v = min(v,0.10);
elseif abs(headingError) > 0.72
    v = min(v,0.38*maxV);
end

% Smooth final approach.
finalDistance = norm(robotXY-path(end,:));
if finalDistance < 0.90
    v = min(v,0.58*finalDistance+0.035);
end

v = max(0.045,min(v,maxV));
w = v*curvature + 1.35*headingError;
w = max(min(w,maxW),-maxW);
end

function angle = wrapToPiLocal(angle)
angle = mod(angle+pi,2*pi)-pi;
end
