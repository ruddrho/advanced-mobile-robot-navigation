function [v,w,headingError,targetPoint,pathIndex,crossTrackError,pathProgress] = ...
    purePursuitPathController(pose,path,pathProgress,maxV,maxW)
%PUREPURSUITPATHCONTROLLER Continuous arc-length Pure Pursuit tracker.
%
% The controller never aims directly at the final goal. It projects the
% robot onto the planned polyline, advances monotonically along path arc
% length, selects a look-ahead point on the same path, and generates linear
% and angular velocity commands that recover cross-track error.
%
% Inputs
%   pose         [x;y;theta]
%   path         N-by-2 planned path
%   pathProgress travelled progress along path in metres
%   maxV,maxW    command limits
%
% Outputs
%   targetPoint      Pure Pursuit look-ahead point on path
%   pathIndex        active displayed path point
%   crossTrackError  signed lateral path error
%   pathProgress     updated monotonic arc-length progress

robotXY = pose(1:2)';
numPoints = size(path,1);

if numPoints < 2
    targetPoint = path(1,:);
    pathIndex = 1;
    pathProgress = 0;
    crossTrackError = 0;
    headingError = 0;
    v = 0;
    w = 0;
    return;
end

segmentVectors = diff(path,1,1);
segmentLengths = vecnorm(segmentVectors,2,2);
segmentLengths = max(segmentLengths,eps);
cumulativeLength = [0; cumsum(segmentLengths)];
totalLength = cumulativeLength(end);
pathProgress = min(max(pathProgress,0),totalLength);

% Locate the segment corresponding to the previous arc-length progress.
currentSegment = find(cumulativeLength <= pathProgress,1,'last');
currentSegment = min(max(currentSegment,1),numPoints-1);

% Search behind only slightly, but look well ahead. This permits recovery
% after obstacle avoidance while preventing jumps to distant path sections.
searchStart = max(1,currentSegment-5);
searchEnd = min(numPoints-1,currentSegment+60);
backtrackAllowance = 0.12;
forwardJumpLimit = 2.20;

bestDistance = inf;
bestSegment = currentSegment;
bestTau = 0;
bestProjection = path(currentSegment,:);
bestProgress = pathProgress;

for segmentIndex = searchStart:searchEnd
    pointA = path(segmentIndex,:);
    segmentVector = segmentVectors(segmentIndex,:);
    segmentLengthSquared = dot(segmentVector,segmentVector);

    tau = dot(robotXY-pointA,segmentVector)/max(segmentLengthSquared,eps);
    tau = min(max(tau,0),1);
    projection = pointA+tau*segmentVector;
    candidateProgress = cumulativeLength(segmentIndex)+tau*segmentLengths(segmentIndex);

    if candidateProgress < pathProgress-backtrackAllowance || ...
            candidateProgress > pathProgress+forwardJumpLimit
        continue;
    end

    projectionDistance = norm(robotXY-projection);
    progressPenalty = 0.015*max(candidateProgress-pathProgress,0);
    candidateCost = projectionDistance+progressPenalty;

    if candidateCost < bestDistance
        bestDistance = candidateCost;
        bestSegment = segmentIndex;
        bestTau = tau;
        bestProjection = projection;
        bestProgress = candidateProgress;
    end
end

% Fallback for an unusually large disturbance: find the nearest valid
% forward segment without allowing progress to move backwards.
if isinf(bestDistance)
    for segmentIndex = currentSegment:numPoints-1
        pointA = path(segmentIndex,:);
        segmentVector = segmentVectors(segmentIndex,:);
        tau = dot(robotXY-pointA,segmentVector)/max(dot(segmentVector,segmentVector),eps);
        tau = min(max(tau,0),1);
        projection = pointA+tau*segmentVector;
        projectionDistance = norm(robotXY-projection);

        if projectionDistance < bestDistance
            bestDistance = projectionDistance;
            bestSegment = segmentIndex;
            bestTau = tau;
            bestProjection = projection;
            bestProgress = cumulativeLength(segmentIndex)+tau*segmentLengths(segmentIndex);
        end
    end
end

pathProgress = max(pathProgress,bestProgress);
pathProgress = min(pathProgress,totalLength);
pathIndex = min(numPoints,bestSegment+1);

% Signed cross-track error. Positive means the robot is to the left of the
% path tangent; the feedback below therefore steers to the right.
pathTangent = segmentVectors(bestSegment,:)/segmentLengths(bestSegment);
robotOffset = robotXY-bestProjection;
crossTrackError = pathTangent(1)*robotOffset(2)-pathTangent(2)*robotOffset(1);

% Estimate upcoming path curvature from headings separated along arc length.
headingNow = atan2(pathTangent(2),pathTangent(1));
headingAheadPoint = pointAtArcLength(path,cumulativeLength, ...
    min(pathProgress+0.70,totalLength));
headingNearPoint = pointAtArcLength(path,cumulativeLength, ...
    min(pathProgress+0.20,totalLength));
upcomingHeading = atan2(headingAheadPoint(2)-headingNearPoint(2), ...
    headingAheadPoint(1)-headingNearPoint(1));
turnSeverity = abs(wrapToPiLocal(upcomingHeading-headingNow));

% Short look-ahead on curves and during path recovery; longer on straights.
lookAhead = 0.55-0.16*min(turnSeverity/(pi/2),1) ...
    -0.14*min(abs(crossTrackError)/0.55,1);
lookAhead = min(max(lookAhead,0.30),0.58);

targetProgress = min(pathProgress+lookAhead,totalLength);
targetPoint = pointAtArcLength(path,cumulativeLength,targetProgress);

% Pure Pursuit geometry in the robot frame.
dx = targetPoint(1)-pose(1);
dy = targetPoint(2)-pose(2);
targetDistance = max(hypot(dx,dy),0.08);
targetBearing = atan2(dy,dx);
headingError = wrapToPiLocal(targetBearing-pose(3));
purePursuitCurvature = 2*sin(headingError)/targetDistance;

% Tangent-heading damping and cross-track recovery keep the robot centred
% on the route after local APF avoidance.
pathHeadingError = wrapToPiLocal(headingNow-pose(3));

headingScale = max(0.10,cos(min(abs(headingError),pi/2))^2);
curveScale = 1/(1+1.40*abs(purePursuitCurvature)+0.90*turnSeverity);
trackScale = max(0.30,1-1.10*min(abs(crossTrackError),0.65));

v = maxV*headingScale*curveScale*trackScale;

if abs(headingError) > 1.15
    v = min(v,0.07);
elseif abs(headingError) > 0.75
    v = min(v,0.30*maxV);
end

remainingPath = totalLength-pathProgress;
finalDistance = norm(robotXY-path(end,:));
if remainingPath < 1.10 || finalDistance < 1.00
    v = min(v,0.55*max(remainingPath,finalDistance)+0.025);
end

v = max(0.035,min(v,maxV));

crossTrackCorrection = -1.55*crossTrackError;
w = v*purePursuitCurvature + 0.85*pathHeadingError + crossTrackCorrection;
w = max(min(w,maxW),-maxW);
end

function point = pointAtArcLength(path,cumulativeLength,queryLength)
%POINTATARCLENGTH Linear interpolation on the planned polyline.
queryLength = min(max(queryLength,0),cumulativeLength(end));
segmentIndex = find(cumulativeLength <= queryLength,1,'last');
segmentIndex = min(max(segmentIndex,1),size(path,1)-1);

segmentStartLength = cumulativeLength(segmentIndex);
segmentEndLength = cumulativeLength(segmentIndex+1);
tau = (queryLength-segmentStartLength)/max(segmentEndLength-segmentStartLength,eps);
point = path(segmentIndex,:)+tau*(path(segmentIndex+1,:)-path(segmentIndex,:));
end

function angle = wrapToPiLocal(angle)
angle = mod(angle+pi,2*pi)-pi;
end
