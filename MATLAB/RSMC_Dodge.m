% RSMC Object Avoidance
% Moves at a constant speed from one destination to another, whilst
% swerving the end effector out of the way of a labelled point.

robot = CytonGamma300('Hans Cutie Robot');

% Setup the points to move between
point1 = transl(-0.2, 0.3, 0.1)*trotx(-pi/2)*troty(pi);
point2 = transl( 0.2, 0.3, 0.1)*trotx(-pi/2)*troty(pi);
approxQ = [0, pi/2, -pi/2, pi/4, 0, pi/4, 0];

avoidPos = transl(0, 0.15, 0.1);

% Setup the robot in a configuration close to the desired one
initialQ = robot.robotModel.ikcon(point1, approxQ);
robot.joints = initialQ;
robot.plot()

% The point we're moving towards
target = point2;

while true
    robotPos = robot.getEndEffectorTransform();
    % Alternate the target between point 1 and 2
    while distance(robotPos, target) > 0.05
        % Get the positional change to the target
        robotPos = robot.getEndEffectorTransform();
        toTarget = target(1:3,4) - robotPos(1:3,4);
        dist = distance(robotPos, target);
        % Make this a unit vector of length 0.05
        toolPosVel = toTarget / norm(toTarget) * 0.005;
        % Add the rotational components as well
        toolVel = [toolPosVel' [0 0 0]]';
        % Use Moore-Penrose Pseudo inverse to get the smallest solution
        jointVel = pinv(robot.getJacobian()) * toolVel;
        % Attenuate the joint velocities
        for i = 1:robot.nJoints
            if jointVel(i) > pi/12
                reductionFactor = (pi/12) / jointVel(i);
                disp 'Reduced joint velocities due to excessive motion'
                jointVel = jointVel .* reductionFactor;
            end
        end
        % Apply to the joints
        robot.joints = robot.joints + jointVel';
        % Update the plot
        robot.animate();
    end
    disp 'Reached target'
    if target == point1
        target = point2;
    else
        target = point1;
    end
end

function d = distance(t1, t2)
    p1 = t1(1:3,4);
    p2 = t2(1:3,4);
    d = norm(p2 - p1);
end