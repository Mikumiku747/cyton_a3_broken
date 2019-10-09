classdef HansCute < handle
    % Han's robotics cute robot class.
    % For use with Peter Corke's Robotics, Vision and Control Toolbox.
    % We inherit from handle because this allows us to modify the robots
    % state and keep those changes preserved in class methods (for example,
    % moveJ will actually change the robot's joints).
    
    properties (Constant)
        DHParams = ...  % DH Parameters describing the robot geometry
        [
        %   z       x       alpha       range       offset
            0.120   0       90.00       300.0       0
            0       0       -90.00      210.0       0
            0.141   0       90.00       300.0       0
            0       0.072   -90.00      210.0       90.00
            0       0.072   90.00       210.0       0
            0       0       90.00       210.0       90.00
            0.130   0       180.0       300.0       180.0
        ];    
        nJoints = 7;    % Number of joints in the robot;
        moveJFrequency = 20;    % Rate at which joint moves should run
        moveLFrequency = 20;    % Rate at which tool moves should run
        maxJointVel = ...       % Largest movement the robot can make in one time step
            (pi/2) / 20;
        end
    properties
        robotModel      % SerialLink object describing the robot
        joints          % Joint Positions
        realRobotHAL    % HAL for the actual robot itself
        moveRealRobot   % Hardware State (simulation or actual)
    end
    
    methods
        
        function obj = set.joints(obj, joints)
            % Validation that the new joint position is within joint limits
            for i = 1:obj.nJoints
                if joints(i) > deg2rad(obj.DHParams(i,4)/2) || ...
                   joints(i) < deg2rad(-obj.DHParams(i,4)/2)
                    error("Joint values exceed joint limits.");
                end
            end
            obj.joints = joints;
        end
        
        function obj = HansCute(name)
            % Creates a new Cyton 300 Robot Object. Default pose is 0
            if nargin < 1
                name = "Hans Cute Robot";
            end
            links = Link.empty(obj.nJoints, 0);
            for i = 1:size(obj.DHParams,1)
                links(i) = Link('d', obj.DHParams(i,1), ...
                    'a', obj.DHParams(i,2), 'alpha', deg2rad(obj.DHParams(i,3)), ...
                    'qlim', deg2rad([-obj.DHParams(i,4)/2, obj.DHParams(i,4)/2]), ...
                    'offset', deg2rad(obj.DHParams(i,5)));
            end
            obj.robotModel = SerialLink(links, 'name', name);
            obj.joints = zeros(1, obj.nJoints);
            obj.moveRealRobot = false;
        end
        
        function connectToHW(obj)
            % Connects and initialises an actual robot
            obj.realRobotHAL = HansCuteHAL();
            obj.realRobotHAL.homeRobot();
            obj.syncHW();
        end
        
        function syncHW(obj)
            % Synchronises the model of the robot with the actual running
            % robot.
            % TODO read the robot position
        end
        
        function teach(obj)
            % Opens a figure with the robot for teaching new poses
            obj.robotModel.teach(obj.joints);
        end
        
        function joints = getJoints(obj)
            % Gets the current joint positions of the robot
            joints = obj.robotModel.getpos();
        end
        
        function transform = getEndEffectorTransform(obj, joints)
            % Uses forward kinematics to get the transform of the end
            % effector.
            if nargin < 2
                joints = obj.joints;
            end
            transform = obj.robotModel.fkine(joints);
        end
        
        function position = getEndEffectorPosition(obj, joints)
            % Uses forward kinematics to get the position of the end
            % effector.
            if nargin < 2
                joints = obj.joints;
            end
            transform = obj.robotModel.fkine(joints);
            position = transform(1:3,4);
        end
        
        function jacobian = getJacobian(obj, joints)
            % Gets the jacobian for a given robot joint position. 
            % If no position is supplied, the current pose is used.
            if nargin < 2
                joints = obj.joints;
            end
            jacobian = obj.robotModel.jacob0(joints);
        end
        
        function plot(obj)
            % Plots the robot
            obj.robotModel.plot(obj.joints);
        end
        
        function animate(obj)
            % Updates the plot of a robot
            obj.robotModel.animate(obj.joints);
        end
        
        function jogMode(obj)
            % Puts the robot into jog mode under controller navigation
            % Runs until the stop button is pressed.
            control = joystickJog(1);
            control.runFrequency = obj.moveLFrequency;
            % Rate controller lets us run at 20Hz
            rateLimiter = rateControl(obj.moveLFrequency);
            rateLimiter.reset();
            obj.plot();
            while ~control.getStopStatus()
                % Read the controller Input
                toolVel = control.generateJogMovements();
                % Generate joint velocities using inverse jacobian
                jointVel = toolVel * pinv(obj.getJacobian)';
                % Apply to robot model and animate
                obj.joints = obj.joints + jointVel;
                obj.animate();
                rateLimiter.waitfor();
            end
        end
        
        function moveJTraj(obj, trajectory)
            % Moves the robot through a set of joint positions
            % The rate limiter means our robot will run at a realistic
            % speed
            rateLimiter = rateControl(obj.moveJFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.joints = trajectory(1,:);
            obj.plot();
            rateLimiter.reset();
            for i = 2:size(trajectory,1)
                obj.joints = trajectory(i,:);
                obj.animate();
                rateLimiter.waitfor();
            end
        end
        
        function traj = planJTraj(obj, destTrans, duration)
            % Plans a jointspace movement and produces a trajectory
            
            % Solve the destination position and interpolate over to it
            destJoints = obj.robotModel.ikcon(destTrans, obj.getJoints);
            traj = jtraj(obj.joints, destJoints, duration * obj.moveJFrequency);
        end
        
        function moveJ(obj, dest, duration)
            % Plans and moves through a trajectory
            traj = obj.planJTraj(dest, duration);
            obj.moveJTraj(traj);
        end
        
        function moveLTraj(obj, trajectory)
            % Move through a set of joint velocities
            
            % A rate limiter allows us to run our loop at the desired
            % frequency
            rateLimiter = rateControl(obj.moveLFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.plot();
            rateLimiter.reset()
            for i = 1:size(trajectory,1)
                obj.joints = obj.joints + trajectory(i,:);
                obj.animate();
                rateLimiter.waitfor();
            end
        end
        
        function traj = planLTraj(obj, destPos, duration, accuracy)
            % Moves the robot to the given position (maintains orientation)
            % through cartesian space. 
            
            % Compute the speed to move at based on duration
            moveSpeed = obj.distanceTo(destPos) / duration;
            % Setup the trajectory
            traj = [];
            % Begin to move towards that in step increments until we reach
            % the desired destination
            plannedJoints = obj.joints;
            diff = destPos - obj.getEndEffectorPosition(plannedJoints);
            while (norm(diff) > accuracy)
                % Compute the step in the right direction
                diff = destPos - obj.getEndEffectorPosition(plannedJoints);
                diffDir = diff / norm(diff);
                % Compute the velocity for this timestep
                positionVelocities = diffDir .* (moveSpeed / obj.moveLFrequency);
                velocities = [positionVelocities' [0 0 0]]'; 
                % Use the jacobian to get the required joint velocities
                jointVelocities = pinv(obj.getJacobian) * velocities;
                % Stop if the joint velocities are too high
                for i = 1:obj.nJoints
                    if jointVelocities(i) > obj.maxJointVel
                        error('Stopping due to excessive joint velocities, check for singularities.');
                    end
                end
                % Store the velocity in the trajectory
                traj = [traj' jointVelocities']';
                % Apply the joint velocities
                plannedJoints = plannedJoints + jointVelocities';
            end
        end
        
        function moveL(obj, destPos, duration, accuracy)
            % Plans and moves the robot through toolspace
            if (nargin < 4)
                accuracy = 0.005;
            end
            traj = obj.planLTraj(destPos, duration, accuracy);
            obj.moveLTraj(traj);
        end
        
        function dist = distanceTo(obj, pos)
            % Gives the distance from the end effector to a given position.
            robotTrans = obj.getEndEffectorTransform;
            robotPos = robotTrans(1:3,4);
            dist = norm(pos - robotPos);
        end
    end    
end