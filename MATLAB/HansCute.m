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
        maxJointVel = pi/20;    % Largest movement the robot can make in one time step
        moveJFrequency = 20;    % Rate at which joint moves should run
        moveLFrequency = 20;    % Rate at which tool moves should run
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
                % TODO: Validation here
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
        end
        
        function connectToHW(obj)
            % Connects and initialises an actual robot
            % TODO Not finished yet
            obj.realRobotHAL = HansCuteHAL();
            obj.realRobotHAL.homeRobot();
            obj.syncHW();
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
        
        function moveJTraj(obj, trajectory)
            % Animates the robot along a given trajectory
            obj.joints = trajectory(1,:);
            % The rate limiter means our robot will run at a realistic
            % speed
            rateLimiter = rateControl(obj.moveJFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.robotModel.plot(trajectory(1,:));
            rateLimiter.reset();
            for i = 2:size(trajectory,1)
                obj.joints = trajectory(i,:);
                obj.robotModel.animate(trajectory(i,:));
                rateLimiter.waitfor();
            end
        end
        
        function moveJ(obj, destTrans, duration)
            % Moves the robot to the given transform through joint space
            destJoints = obj.robotModel.ikcon(destTrans, obj.getJoints);
            traj = jtraj(obj.joints, destJoints, duration * obj.moveJFrequency);
            obj.moveJTraj(traj);
        end
        
        function moveL(obj, destPos, duration, accuracy)
            % Moves the robot to the given position (maintains orientation)
            % through cartesian space. 
            if (nargin < 4)
                accuracy = 0.005;
            end
            % Compute the speed to move at based on duration
            moveSpeed = obj.distanceTo(destPos) / duration;
            % A rate limiter allows us to run our loop at the desired
            % frequency
            rateLimiter = rateControl(obj.moveLFrequency);
            rateLimiter.OverrunAction = 'slip';
            % Begin to move towards that in step increments until we reach
            % the desired destination
            obj.plot();
            rateLimiter.reset()
            while (obj.distanceTo(destPos) > accuracy)
                % Compute the step in the right direction
                diff = destPos - obj.getEndEffectorPosition;
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
                % Apply the joint velocities (including plot)
                obj.joints = obj.joints + jointVelocities';
                obj.animate();
                rateLimiter.waitfor();
            end
            
        end
        
        function dist = distanceTo(obj, pos)
            % Gives the distance from the end effector to a given position.
            robotTrans = obj.getEndEffectorTransform;
            robotPos = robotTrans(1:3,4);
            dist = norm(pos - robotPos);
        end
    end    
end