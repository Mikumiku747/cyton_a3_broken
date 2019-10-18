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
            0.150   0       90.00       300.0       0
            0       0       -90.00      210.0       0
            0.125   0       90.00       300.0       0
            0       0.065   -90.00      210.0       90.00
            0       0.068   90.00       210.0       0
            0       0       90.00       210.0       90.00
            0.158   0       0.0         300.0       180.0
        ];    
        nJoints = 7;    % Number of joints in the robot;
        maxJointVel = ...       % Largest movement the robot can make in one time step
            (pi);
        q0 = ...                % Home Position 
            [0 0 0 0 0 0 0];    % (all zeroes)
        cancelFrequency = ...   % Rate at which the controller is sampled to cancel a movement
            10;                 % 10 HZ
        RMRCWeights = ...       % Weighting for the RMRC Solution
            diag([1 1 1 0.5 0.5 0.5]);
        linearSpeed = ...       % Linear speed during L trajectories
            0.2;                % in m/s.
        angularSpeed = ...      % Angular speed during L trajectories (tool rotation speed)
            pi/2;               % in rad/s.
        workspace = [-1 1 -1 1 -0.3 1];   % Robot workspace
    end
    
    properties
        robotModel      % SerialLink object describing the robot
        joints          % Joint Positions
        realRobotHAL    % HAL for the actual robot itself
        controller      % Gamepad controller (JoystickJog)
        moveRealRobot   % Hardware State (simulation or actual)
        useRealController   % Controller State (keyboard or controller)
        moveJFrequency	% Rate at which joint moves should run
        moveLFrequency	% Rate at which tool moves should run
    end
    
    methods
        function plotModel(self)   %robot,workspace
            for linkIndex = 0:self.robotModel.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] =  ...
                    plyread(['HansLink',num2str(linkIndex),'.ply'],'tri');
                
                self.robotModel.faces{linkIndex + 1} = faceData;
                self.robotModel.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.robotModel.plot3d(zeros(1,self.robotModel.n),...
                'noarrow','workspace',self.workspace);
            hold all
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.robotModel.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.robotModel.n
                handles = findobj('Tag', self.robotModel.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = ...
                        [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        function set.joints(obj, joints)
            % Validate joints on assignment
            obj.validateJoints(joints);
            obj.joints = joints;
        end
        
        function valid = validateJoints(obj, joints, crash)
            % Validation that the new joint position is within joint limits
            % If no joints are provided the robot's current joints are
            % used. If crash is set to false, the validity of the joints
            % will be returned instead.
            if nargin < 2
                joints = obj.joints;
            end
            if nargin < 3
                crash = true;
            end
            if norm(double(abs(joints) > deg2rad(obj.DHParams(:,4))'/2))
                if crash
                    disp('Joint limit overshoot')
                    rad2deg(abs(joints) - deg2rad(obj.DHParams(:,4)'/2))
                    error('Joints exceed joint limits!');
                else
                    valid = false;
                    return
                end
            else
                valid = true;
                return
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
    %        obj.controller = JoystickJog();
            obj.moveRealRobot = false;
            obj.moveJFrequency = 15;
            obj.moveLFrequency = 15;
        end
        
        function connectToHW(obj)
            % Connects and initialises an actual robot
            
            disp 'Connecting to real robot.'
            obj.realRobotHAL = HansCuteHAL();
            disp 'Enabling joints.'
            obj.realRobotHAL.enableRobot();
            disp 'Homing robot.'
            obj.realRobotHAL.homeRobot();
            disp 'Starting Claw'
            obj.realRobotHAL.enableClaw();
            obj.realRobotHAL.setClaw(20);
            disp 'Synchronizing plot and simulation.'
            obj.plotModel();
            obj.syncHW();
            disp 'Robot ready to operate.'
            obj.moveRealRobot = true;
            obj.realRobotHAL.controllerHandle = obj.controller;
        end
        
        function syncHW(obj)
            % Synchronises the model of the robot with the actual running
            % robot.
            obj.joints = obj.realRobotHAL.getActualJoints();
            %obj.animate;
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
            pause(0.1)
        end
        
        function jogMode(obj)
            % Puts the robot into jog mode under controller navigation
            % Runs until the stop button is pressed.
            % Rate controller lets us run at a constant rate
            rateLimiter = rateControl(obj.moveLFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.plotModel();
            rateLimiter.reset();
            % Profile the timing on this function to see if we can keep
            % up...
            p = RateProfiler();
            p.start();
            count = 0;
            while ~obj.controller.getStopStatus()
                count = count + 1;
                % Read the controller Input
                toolVel = obj.controller.generateJogMovements();
                % Generate joint velocities using inverse jacobian
                jointVel = toolVel * pinv(obj.getJacobian)';
                % Validate the joints
                if obj.validateJoints(obj.joints + jointVel, false)
                    % Move the real robot as well
                    if obj.moveRealRobot == true
                        obj.realRobotHAL.sendV(jointVel);
                        obj.syncHW();
                    else
                        obj.joints = obj.joints + jointVel/obj.moveLFrequency;
                        obj.animate();
                    end
                else
                    warning(['Step %d: \nMotion abandoned, would exceed'...
                        'joint limits.'], count);
                end
                rateLimiter.waitfor();
            end
            % Profiling Results
            p.toc()
            p.calcStats(count, obj.moveLFrequency);
            p.showStats();
        end
        
        function q = teachPosition(obj)
            % Allows the robot to be hand-guided by disabling the motors.
            % When the guide button is pressed, the robot is de-energized,
            % and then once it is pressed again, it is re-energized and the
            % resulting position is returned. 
            
            disp(['Preparing for teach mode. Press Square to switch off'...
                ' the motors and move the robot by hand.']);
            
            % Wait for the button to be pressed
            while ~obj.controller.getTeachStatus
            end
            obj.realRobotHAL.disableRobot();
            % Give time for the button to be released
            pause(0.5);
            disp(['The robot is now in teach mode, press Square again ' ...
                'to engage the motors and locate the robot.']);
            % Wait for the button to be pressed again
            while ~obj.controller.getTeachStatus()
            end
            obj.realRobotHAL.enableRobot();
            obj.syncHW();
            disp('Position locked.');
            q = obj.joints;
        end
        
        function moveJTraj(obj, trajectory)
            % Moves the robot through a set of joint positions
            % The rate limiter means our robot will run at a realistic
            % speed
            rateLimiter = rateControl(obj.moveJFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.joints = trajectory(1,:);
            obj.plotModel();
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
            if obj.moveRealRobot
                % Move the real robot
                disp 'Performing real robot move.'
                obj.realRobotHAL.movePTraj(traj, obj.moveJFrequency, accuracy);
                obj.joints = obj.realRobotHAL.getActualJoints();
                obj.animate();
                if (obj.controller.getStopStatus())
                    return
                end
            else
                % Move the simulation
                disp 'Performing simulation move.'
                obj.moveJTraj(traj);
            end
        end
        
        function moveQ(obj, dest, duration, accuracy)
            % Performs a jointspace move on the virtual and real robot
            traj = obj.planQTraj(dest, duration);
            % Move the real robot now that the virtual one has finished
            if obj.moveRealRobot
                disp 'Performing real robot move.'
                obj.realRobotHAL.movePTraj(traj, obj.moveJFrequency, accuracy);
                obj.joints = obj.realRobotHAL.getActualJoints();
                obj.animate();
                if obj.controller.getStopStatus()
                    return
                end
            else
                % Move the simulation
                disp 'Performing simulation move.'
                obj.moveJTraj(traj);
            end
        end
        
        function moveLTraj(obj, trajectory)
            % Move through a set of joint velocities
            
            % A rate limiter allows us to run our loop at the desired
            % frequency
            rateLimiter = rateControl(obj.moveLFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.plotModel();
            rateLimiter.reset()
            for i = 1:size(trajectory,1)
                obj.joints = obj.joints + (trajectory(i,:) / obj.moveLFrequency);
                obj.animate();
                rateLimiter.waitfor();
            end
        end
        
        function traj = planLTraj(obj, destTrans, accuracy)
            % Moves the robot to the given position (maintains orientation)
            % through cartesian space. The trajectory is a set of
            % velocities, since the navigation is done in velocity space
            
            % Store the velocity trajectory in here
            traj = [];
            % Plan our trajectory's joints with this variable
            plannedJoints = obj.joints;
            % Get the inital error
            currentTrans = obj.getEndEffectorTransform(plannedJoints);
            pError = destTrans(1:3,4)' - currentTrans(1:3,4)';
            % Move towards the destination until the error is within
            % our accuracy range (we only count for position).
            count = 0;
            while (norm(pError) > accuracy)
                count = count + 1;
                % Re-Compute the error
                currentTrans = obj.getEndEffectorTransform(plannedJoints);
                pError = destTrans(1:3,4)' - currentTrans(1:3,4)';
                rError = tr2rpy(destTrans(1:3,1:3)) - tr2rpy(currentTrans(1:3,1:3));
                % Clamp the rotational error between -pi & pi.
                rError = mod(rError+pi, 2*pi)-pi;
                % Normalise it and set it to the tool speed
                pUnit = (pError / norm(pError)) * obj.linearSpeed;
                rUnit = (rError / norm(rError)) * obj.angularSpeed;
                % If needed, scale the speed down so that we don't
                % overshoot
                pStepSize = norm(pUnit / obj.moveLFrequency);
                if pStepSize > norm(pError)
                    pVel = pUnit * (norm(pError) / pStepSize);
                else
                    pVel = pUnit;
                end
                rStepSize = norm(rUnit / obj.moveLFrequency);
                if rStepSize > norm(rError)
                    rVel = rUnit * (norm(rError) / rStepSize);
                else
                    rVel = rUnit;
                end
                % Combine these into the final delta transform (with
                % weighting
                tVel = [pVel rVel] * obj.RMRCWeights;
                % Use the jacobian and least squares inverse to get joint
                % velocities (pinv does the least squares part)
                qVel = (pinv(obj.getJacobian(plannedJoints)) * tVel')';
                % Check these don't bring the robot past joint limits
                obj.validateJoints(plannedJoints + qVel / obj.moveLFrequency);
                % Check they don't exceed the maximum joint velocity
                % allowed
                if norm(double(...
                        qVel > ones(1,obj.nJoints)*obj.maxJointVel)) > 0
                    error('Joints exceed max allowable joint velocity.');
                end
                % Save to the trajectory
                traj(count,:) = qVel;
                % Increment the planned joint positions
                plannedJoints = plannedJoints + qVel / obj.moveLFrequency;
            end
        end
        
        function moveL(obj, destTrans, accuracy)
            % Plans and moves the robot through toolspace
            if (nargin < 4)
                accuracy = 0.005;
            end
            traj = obj.planLTraj(destTrans, accuracy);
            
            % Move the real robot now that the virtual one has finished
            if obj.moveRealRobot
                disp 'Performing real robot move.'
                obj.realRobotHAL.moveVTraj(traj, obj.moveLFrequency);
                obj.joints = obj.realRobotHAL.getActualJoints();
                obj.animate();
                if obj.controller.getStopStatus()
                    return
                end
            else
                disp 'Performing simulated robot move'
                obj.moveLTraj(traj);
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