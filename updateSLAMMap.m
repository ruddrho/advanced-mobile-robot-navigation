function slamMap = updateSLAMMap(slamMap, pose, ranges, angles, maxRange, res, worldW, worldH)
%UPDATESLAMMAP Robust live occupancy-grid update using log-odds evidence.
%
% slamMap stores log-odds values:
%   0          = unknown
%   negative   = increasingly likely free
%   positive   = increasingly likely occupied
%
% This prevents an occupied cell from being erased immediately by a
% neighbouring LiDAR ray, which caused flickering and an unstable map in
% the previous binary (-1/0/1) implementation.

[H,W] = size(slamMap);

% Inverse sensor-model increments.
logOddsFree = -0.40;
logOddsOcc  =  1.10;
logOddsMin  = -4.00;
logOddsMax  =  4.00;

% Use approximately half-cell ray steps for continuous-looking free space.
rayStep = max(0.5/res, 0.015);
endpointGap = max(1.5/res, 0.055);
rayStride = 2;

for i = 1:rayStride:numel(angles)
    rMeasured = min(max(ranges(i),0),maxRange);
    th = pose(3) + angles(i);

    % A return shorter than max range is treated as a hit. This includes
    % obstacles and the simulated world boundary.
    hasHit = rMeasured < (maxRange - rayStep);

    if hasHit
        freeEnd = max(0, rMeasured - endpointGap);
    else
        freeEnd = rMeasured;
    end

    %% Mark traversed cells as free
    if freeEnd > 0
        rFree = 0:rayStep:freeEnd;
        xFree = pose(1) + rFree*cos(th);
        yFree = pose(2) + rFree*sin(th);

        valid = xFree >= 0 & xFree < worldW & yFree >= 0 & yFree < worldH;
        xFree = xFree(valid);
        yFree = yFree(valid);

        if ~isempty(xFree)
            cols = floor(xFree*res) + 1;
            rows = floor(yFree*res) + 1;
            cols = min(max(cols,1),W);
            rows = min(max(rows,1),H);

            freeIdx = unique(sub2ind([H W],rows,cols));
            slamMap(freeIdx) = max(logOddsMin,slamMap(freeIdx) + logOddsFree);
        end
    end

    %% Mark the measured endpoint as occupied
    if hasHit
        xHit = pose(1) + rMeasured*cos(th);
        yHit = pose(2) + rMeasured*sin(th);

        if xHit >= 0 && xHit < worldW && yHit >= 0 && yHit < worldH
            hitCol = min(max(floor(xHit*res)+1,1),W);
            hitRow = min(max(floor(yHit*res)+1,1),H);

            % A tiny footprint makes thin obstacle edges visible without
            % unrealistically inflating them.
            [dc,dr] = meshgrid(-1:1,-1:1);
            keep = (dc.^2 + dr.^2) <= 2;
            hitCols = hitCol + dc(keep);
            hitRows = hitRow + dr(keep);
            validHit = hitCols >= 1 & hitCols <= W & hitRows >= 1 & hitRows <= H;
            hitCols = hitCols(validHit);
            hitRows = hitRows(validHit);

            hitIdx = unique(sub2ind([H W],hitRows,hitCols));
            slamMap(hitIdx) = min(logOddsMax,slamMap(hitIdx) + logOddsOcc);
        end
    end
end

% Numerical safety.
slamMap = min(max(slamMap,logOddsMin),logOddsMax);
end
