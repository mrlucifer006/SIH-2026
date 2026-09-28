function [command, tracks, path, debug] = sih_autonomy_step(egoState, cameraDetections, lidarDetections, radarDetections, staticObstacles, goal, params)
%SIH_AUTONOMY_STEP Closed-loop perception, planning and control step.
%
% This is a drop-in MATLAB Function block implementation. All positions are
% expressed in the ego/world XY frame (metres), with +X pointing forward.
% Detection rows are:
%   camera: [x y confidence]
%   lidar:  [x y radius confidence]
%   radar:  [x y vx vy confidence]
%
% egoState is [x y yaw speed]. staticObstacles are [x y radius]. goal is [x y].
% The function deliberately uses base MATLAB syntax so it can also run in a
% MATLAB Function block. Replace the camera adapter with your trained detector
% when the camera output is an image rather than detections.

% A no-argument call is useful for quickly checking the block from the MATLAB
% command window. It runs one safe demo step and prints the resulting command.
if nargin == 0
    egoState = [0 0 0 2];
    cameraDetections = [12 1 0.9];
    lidarDetections = [12 1 1.0 0.95];
    radarDetections = [12 1 -1.0 0 0.9];
    staticObstacles = [8 -3 1.0; 18 3 1.2];
    goal = [30 0];
    params = struct('maxSpeed', 6);
    [command, tracks, path, debug] = sih_autonomy_step(egoState, ...
        cameraDetections, lidarDetections, radarDetections, ...
        staticObstacles, goal, params);
    fprintf('SIH autonomy demo completed.\n');
    fprintf('Command [steering throttle brake] = [%.3f %.3f %.3f]\n', command);
    fprintf('Tracks: %d | Path points: %d | Nearest clearance: %.2f m\n', ...
        size(tracks,1), size(path,1), debug.nearestRisk);
    return;
end

if nargin < 7 || isempty(params), params = struct; end
p = defaults(params);

ego = double(egoState(:));
if numel(ego) < 4, error('egoState must be [x y yaw speed].'); end
goal = double(goal(:));

tracks = fuseDetections(cameraDetections, lidarDetections, radarDetections, p);
predicted = tracks(:,1:2) + p.predictionHorizon * tracks(:,3:4);

% Include predicted moving objects and static obstacles in the local map.
obstacles = [predicted, max(tracks(:,5), p.minimumRadius) + p.vehicleRadius + p.safetyMargin];
if ~isempty(staticObstacles)
    so = double(staticObstacles);
    obstacles = [obstacles; so(:,1:2), so(:,3) + p.vehicleRadius + p.safetyMargin]; %#ok<AGROW>
end

path = astarPath(ego(1:2), goal(1:2), obstacles, p);
if isempty(path)
    path = [ego(1:2)'; goal(1:2)'];
end

[steer, desiredSpeed, nearestRisk] = purePursuit(ego, path, obstacles, p);
accel = max(-p.maxBrake, min(p.maxAccel, p.speedGain * (desiredSpeed - ego(4))));
if nearestRisk < p.emergencyDistance
    accel = -p.maxBrake;
    desiredSpeed = 0;
end

command = [steer; max(0, accel); max(0, -accel - p.maxAccel)]; % steering, throttle, brake
debug = struct('desiredSpeed', desiredSpeed, 'nearestRisk', nearestRisk, ...
    'replanRequired', true, 'numTracks', size(tracks,1));
end

function p = defaults(in)
p = struct('resolution',0.5,'mapHalfWidth',25,'mapHalfLength',35, ...
    'vehicleRadius',1.2,'safetyMargin',0.8,'minimumRadius',0.5, ...
    'predictionHorizon',0.8,'clusterDistance',2.0,'maxSpeed',8.0, ...
    'lookahead',5.0,'speedGain',1.5,'maxAccel',2.0,'maxBrake',6.0, ...
    'emergencyDistance',4.0,'maxSteer',0.55);
names = fieldnames(in);
for k = 1:numel(names), p.(names{k}) = in.(names{k}); end
end

function tracks = fuseDetections(cam, lidar, radar, p)
% Track columns: [x y vx vy radius confidence]. Greedy clustering is stable,
% lightweight, and suitable for a first Simulink integration.
all = zeros(0,6);
if ~isempty(cam)
    c = double(cam); all = [all; c(:,1:2), zeros(size(c,1),2), ...
        p.minimumRadius*ones(size(c,1),1), col(c,3)]; %#ok<AGROW>
end
if ~isempty(lidar)
    l = double(lidar); all = [all; l(:,1:2), zeros(size(l,1),2), ...
        max(l(:,3),p.minimumRadius), col(l,4)]; %#ok<AGROW>
end
if ~isempty(radar)
    r = double(radar); all = [all; r(:,1:2), r(:,3:4), ...
        p.minimumRadius*ones(size(r,1),1), col(r,5)]; %#ok<AGROW>
end
if isempty(all), tracks = all; return; end

used = false(size(all,1),1); tracks = zeros(0,6);
for i = 1:size(all,1)
    if used(i), continue; end
    members = find(~used & vecnorm(all(:,1:2)-all(i,1:2),2,2) <= p.clusterDistance);
    w = max(all(members,6), 0.05);
    pos = sum(all(members,1:2).*w,1)/sum(w);
    vel = sum(all(members,3:4).*w,1)/sum(w);
    rad = max(all(members,5)); conf = min(1, max(all(members,6)));
    tracks(end+1,:) = [pos vel rad conf]; %#ok<AGROW>
    used(members) = true;
end
end

function x = col(a, idx)
if size(a,2) >= idx, x = a(:,idx); else, x = ones(size(a,1),1); end
end

function path = astarPath(start, goal, obstacles, p)
% Grid A* in a local ego-centred map.
res = p.resolution; nx = floor(2*p.mapHalfLength/res)+1; ny = floor(2*p.mapHalfWidth/res)+1;
origin = [start(1)-p.mapHalfLength, start(2)-p.mapHalfWidth];
toCell = @(q) max([1 1], min([nx ny], round((q-origin)/res)+1));
fromCell = @(c) origin + (c-1)*res;
occ = false(ny,nx);
for k = 1:size(obstacles,1)
    cc = toCell(obstacles(k,1:2)); rr = max(1,ceil(obstacles(k,3)/res));
    x1=max(1,cc(1)-rr); x2=min(nx,cc(1)+rr); y1=max(1,cc(2)-rr); y2=min(ny,cc(2)+rr);
    for yy0=y1:y2, for xx0=x1:x2
        if norm(([xx0 yy0]-cc)) <= rr, occ(yy0,xx0)=true; end
    end, end
end
s = toCell(start); g = toCell(goal); occ(s(2),s(1))=false; occ(g(2),g(1))=false;
open = s; gScore = inf(ny,nx); fScore = inf(ny,nx); came = zeros(ny,nx,2); gScore(s(2),s(1))=0; fScore(s(2),s(1))=norm(s-g);
dirs = [-1 -1;0 -1;1 -1;-1 0;1 0;-1 1;0 1;1 1];
while ~isempty(open)
    openLinear = sub2ind([ny nx], open(:,2), open(:,1));
    [~,ix] = min(fScore(openLinear)); cur=open(ix,:); open(ix,:)=[];
    if isequal(cur,g), break; end
    for d=1:8
        nb=cur+dirs(d,:); if any(nb<1)||nb(1)>nx||nb(2)>ny||occ(nb(2),nb(1)), continue; end
        tentative=gScore(cur(2),cur(1))+norm(dirs(d,:));
        if tentative<gScore(nb(2),nb(1))
            came(nb(2),nb(1),:) = cur; gScore(nb(2),nb(1))=tentative; fScore(nb(2),nb(1))=tentative+norm(nb-g);
            if ~ismember(nb,open,'rows'), open(end+1,:)=nb; end %#ok<AGROW>
        end
    end
end
if isinf(gScore(g(2),g(1))), path=[]; return; end
cells=g; while ~isequal(cells(1,:),s), prev=squeeze(came(cells(end,2),cells(end,1),:))'; cells=[cells;prev]; end
cells=flipud(cells); path=zeros(size(cells,1),2); for k=1:size(cells,1), path(k,:)=fromCell(cells(k,:)); end
end

function [steer, speed, risk] = purePursuit(ego, path, obstacles, p)
d = vecnorm(path-ego(1:2)',2,2); idx=find(d>=p.lookahead,1); if isempty(idx), idx=size(path,1); end
target=path(idx,:); a=atan2(target(2)-ego(2),target(1)-ego(1))-ego(3); a=atan2(sin(a),cos(a));
steer=max(-p.maxSteer,min(p.maxSteer,atan2(2*2.7*sin(a),p.lookahead)));
speed=p.maxSpeed; risk=inf;
if ~isempty(obstacles), risk=min(vecnorm(obstacles(:,1:2)-ego(1:2)',2,2)-obstacles(:,3)); end
if risk<10, speed=p.maxSpeed*max(0,min(1,(risk-p.emergencyDistance)/6)); end
end
