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
            0.130   0       0.0         300.0       180.0
        ];    
        nJoints = 7;    % Number of joints in the robot;
        moveJFrequency = 20;    % Rate at which joint moves should run
        moveLFrequency = 20;    % Rate at which tool moves should run
        moveLStepSize = 0.05;   % Toolspace constant velocity
        maxJointVel = ...       % Largest movement the robot can make in one time step
            (pi/2) / 20;
        q0 = ...                % Home Position 
            [0 0 0 0 0 0 0];    % (all zeroes)
        cancelFrequency = ...   % Rate at which the controller is sampled to cancel a movement
            20;                 % 20 HZ
    end
    
    properties
        robotModel      % SerialLink object describing the robot
        joints          % Joint Positions
        realRobotHAL    % HAL for the actual robot itself
        controller      % Gamepad controller (JoystickJog)
        moveRealRobot   % Hardware State (simulation or actual)
    end
    
    methods
        
        function set.joints(obj, joints)
            % Validate joints on assignment
            obj.validateJoints(joints);
            obj.joints = joints;
        end
        
        function validateJoints(obj, joints)
            % Validation that the new joint position is within joint limits
            for i = 1:obj.nJoints
                if joints(i) > deg2rad(obj.DHParams(i,4)/1.8) || ...
                   joints(i) < deg2rad(-obj.DHParams(i,4)/1.8)
                    error("Joint values exceed joint limits.");
                end
            end
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
            obj.controller = JoystickJog;
            obj.controller.runFrequency = obj.moveLFrequency;
        end
        
        function connectToHW(obj)
            % Connects and initialises an actual robot
            obj.realRobotHAL = HansCuteHAL();
            %obj.realRobotHAL.homeRobot();
            obj.syncHW();
            obj.moveRealRobot = true;
        end
        
        function syncHW(obj)
            % Synchronises the model of the robot with the actual running
            % robot.
            obj.joints = obj.realRobotHAL.getActualJoints;
            obj.animate;
        end
        
        function teach(obj)
            % Opens a figure with the robot for teaching new poses
            obj.robotModel.teach(obj.joints);
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
            
            % Rate controller lets us run at 20Hz
            rateLimiter = rateControl(obj.moveLFrequency);
            rateLimiter.reset();
            obj.plot();
            while ~obj.controller.getStopStatus()
                % Read the controller Input
                toolVel = obj.controller.generateJogMovements();
                % Generate joint velocities using inverse jacobian
                jointVel = toolVel * pinv(obj.getJacobian)';
                % Apply to robot model and animate
                obj.joints = obj.joints + jointVel;
                obj.animate();
                % Move the real robot as well
                if obj.moveRealRobot == true
                    instantTraj = obj.joints;
                    obj.realRobotHAL.movePTraj(instantTraj, obj.controller.runFrequency);
                end
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
        
        function traj = planJTraj(obj, destTrans, duration, accuracy)
            % Plans a jointspace movement and produces a trajectory
            
            % Solve the destination position and interpolate over to it
            destJoints = obj.robotModel.ikcon(destTrans, obj.joints);
            traj = jtraj(obj.joints, destJoints, duration * obj.moveJFrequency);
            % If the requested position does not satisfy accuracy
            % requirements
            finalTrans = obj.robotModel.fkine(traj(size(traj,1),:));
            pError = norm(destTrans(1:3,4) - finalTrans(1:3,4));
            rError = norm(destTrans(1:3,1:3) - finalTrans(1:3,1:3));
            if pError > accuracy
                error('Calculated destination has too much error:\nPositional: %s m\nRotational: %s', pError, rError);
            end
        end
        
        function traj = planQTraj(obj, destQ, duration)
            % Plans a movement directly to another joint position
            traj = jtraj(obj.joints, destQ, duration * obj.moveJFrequency);
        end
        
        function moveJ(obj, dest, duration, accuracy)
            % Plans and moves through a trajectory
            traj = obj.planJTraj(dest, duration, accuracy);
            % Move the simulation first
            disp 'Performing simulation move.'
            obj.moveJTraj(traj);
            % Once the simulation has successfully moved, move the real
            % robot too
            if obj.moveRealRobot
                disp 'Performing real robot move.'
                obj.realRobotHAL.movePTraj(traj, obj.moveJFrequency);
                disp 'Waiting for real robot to complete movement'
                pause(duration);
                obj.joints = obj.realRobotHAL.getActualJoints();
                obj.animate();
            end
        end
        
        function moveQ(obj, dest, duration)
            % Performs a jointspace move on the virtual and real robot
            traj = obj.planQTraj(dest, duration);
            % Move the simulation first and wait for it to succeed
            disp 'Performing simulation move.'
            obj.moveJTraj(traj);
            % Move the real robot now that the virtual one has finished
            if obj.moveRealRobot
                disp 'Performing real robot move.'
                obj.realRobotHAL.movePTraj(traj, obj.moveJFrequency);
                % Wait for the actual robot to finish its movements
                disp 'Waiting for real robot to complete movement'
                pause(duration);
                obj.joints = obj.realRobotHAL.getActualJoints();
                obj.animate();
            end
        end

%         REMOVED: Since the real robot can no longer move with velocity,
%         we don't need to bother with it for this one.
%         
%         function moveLTraj(obj, trajectory)
%             % Move through a set of joint velocities
%             
%             % A rate limiter allows us to run our loop at the desired
%             % frequency
%             rateLimiter = rateControl(obj.moveLFrequency);
%             rateLimiter.OverrunAction = 'slip';
%             obj.plot();
%             rateLimiter.reset()
%             for i = 1:size(trajectory,1)
%                 obj.joints = obj.joints + trajectory(i,:);
%                 obj.animate();
%                 rateLimiter.waitfor();
%             end
%         end
        
        function traj = planLTraj(obj, destPos, duration, accuracy)
            % Moves the robot to the given position (maintains orientation)
            % through cartesian space. The trajectory is a set of
            % POSITIONS, not velocities as you'd expect...
            
            % Compute the speed to move at based on duration
            moveSpeed = obj.moveLStepSize;
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
                % Apply the joint velocities
                plannedJoints = plannedJoints + jointVelocities';
                % Verify the joints fall within limits
                obj.validateJoints(plannedJoints);
                % Store the positions in the trajectory
                traj = [traj' plannedJoints']';
            end
        end
        
        function moveL(obj, destPos, duration, accuracy)
            % Plans and moves the robot through toolspace
            if (nargin < 4)
                accuracy = 0.005;
            end
            traj = obj.planLTraj(destPos, duration, accuracy);
            disp 'Performing simulated robot move'
            obj.moveJTraj(traj);
            % Move the real robot now that the virtual one has finished
            if obj.moveRealRobot
                disp 'Performing real robot move.'
                obj.realRobotHAL.movePTraj(traj, obj.moveJFrequency);
                % Wait for the actual robot to finish its movements
                disp 'Waiting for real robot to complete movement'
                pause(duration);
                obj.joints = obj.realRobotHAL.getActualJoints();
                obj.animate();
            end
        end
        
        function dist = distanceTo(obj, pos)
            % Gives the distance from the end effector to a given position.
            robotTrans = obj.getEndEffectorTransform;
            robotPos = robotTrans(1:3,4);
            dist = norm(pos - robotPos);
        end
        
        function waitOrCancel(obj, time, freq)
            % Waits for a set time, stops if the stop button is pressed
            if nargin < 3
                freq = obj.cancelFrequency;
            end
            rateLimiter = rateControl(freq);
            count = 0.0;
            while count < time
                if obj.controller.getStopStatus()
                    % TODO: Use the cancel service to send a cancel command
                    disp('Robot motion interrupted by stop button!')
                end
                count = count + 1/freq;
                rateLimiter.waitfor()
            end
        end
    end    
end