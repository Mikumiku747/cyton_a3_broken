% Cyton Access Demo
% Daniel Selmes 2019

%% Initialisation

% Load the joint positions
load('cyton_q.mat');

%% Robot Creation

DHParams = [
%   z       x       alpha       range       offset
    0.120   0       90.00       300.0       0
    0       0       -90.00      210.0       0
    0.141   0       90.00       300.0       0
    0       0.072   -90.00      210.0       90.00
    0       0.072   90.00       210.0       0
    0       0       90.00       210.0       90.00
    0.130   0       180.0       300.0       180.0
];
numJoints = size(DHParams,1);
robotJoints = Link.empty(7,0);
for i = 1:numJoints
    lims = [-deg2rad(DHParams(i,4)/2), deg2rad(DHParams(i,4)/2)];
    robotJoints(i) = Link('d',DHParams(i,1), 'a', DHParams(i,2), ...
        'alpha', deg2rad(DHParams(i,3)), 'offset', deg2rad(DHParams(i,5)), ...
        'qlim', lims);
end
robot = SerialLink(robotJoints, 'name','Hans Cute Robot');

%% Running the example data

% Continously plot the values from cyton_q
robot.plot(cyton_q(1,:));
speed = 3;
for i = 1:speed:size(cyton_q,1)
    robot.animate(cyton_q(i,:));
end
