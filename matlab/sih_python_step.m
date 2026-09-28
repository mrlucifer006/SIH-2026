function [command, tracks, path, debug] = sih_python_step(egoState, cameraDetections, lidarDetections, radarDetections, staticObstacles, goal)
%SIH_PYTHON_STEP Execute the existing sih_sim Python pipeline from Simulink.
% Use this function from an Interpreted MATLAB Function block. The Python
% environment must contain the project package and its dependencies.

% Command-window smoke test. In Simulink, pass the six block signals instead.
if nargin == 0
    egoState = [0 0 0 2];
    cameraDetections = [12 1 0.9];
    lidarDetections = [12 1 1.0 0.95];
    radarDetections = [12 1 -1 0 0.9];
    staticObstacles = [8 -3 1.0];
    goal = [30 0];
end

request = struct('egoState', double(egoState(:).'), ...
    'cameraDetections', double(cameraDetections), ...
    'lidarDetections', double(lidarDetections), ...
    'radarDetections', double(radarDetections), ...
    'staticObstacles', double(staticObstacles), ...
    'goal', double(goal(:).'), 'device', 'auto');

jsonText = jsonencode(request);
projectRoot = fileparts(fileparts(mfilename('fullpath')));
py.sys.path.insert(int32(0), projectRoot);
responseText = char(py.sih_sim.matlab_bridge.step_json(jsonText));
response = jsondecode(responseText);
command = double(response.command(:));
tracks = double(response.tracks);
path = double(response.path);
debug = response.debug;
end
