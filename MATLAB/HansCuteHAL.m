classdef HansCuteHAL < handle
    % Hans Cute Robot Hardware Abstraction Layer
    % This class encapsulates the common functions for using actual
    % hardware for the hans robot. This includes connection, status,
    % converting trajectories to a format which the robot understands, and
    % power controls. 
    
    properties
        positionPub     % Position Publisher (writer)
        positionMsg     % Position Message (for the above writer)
        velocityPub     % Velocity Publisher (writer) 
        velocityMsg     % Velocity Message (for the above writer)
        clawPub         % Claw Publisher (writer)
        clawSub         % Claw Subscriber (reader)
        stateSub        % Joint State Subscriber (reader)
        enableCli       % Motor Enable Client (caller)
        homeCli         % Home Robot Client (caller)
    end
    properties (Constant)
        maxJointVel = ...   % Max value for joint velocities (sanity check)
            pi/2;           % Max value of 90 degrees per second
    end
    
    methods
        function obj = HansCuteHAL()
            % Creates a new hardware connection
            obj.positionPub = rospublisher('/cyton_position_commands');
            obj.velocityPub = rospublisher('/cyton_velocity_commands');
            obj.clawPub = rospublisher('/claw_controller/command');
            obj.clawSub = rospublisher('/claw_controller/state');
            obj.stateSub = rossubscriber('/joint_states');
            obj.enableCli = rossvcclient('/enableCyton');
            obj.homeCli = rossvcclient('/goHome');
            obj.positionMsg = rosmessage(obj.positionPub);
            obj.velocityMsg = rosmessage(obj.velocityPub);
        end
        
        function enableRobot(obj)
            % Enables the motors in the robot.
            enableMotorsMsg = rosmessage(obj.enableCli);
            enableMotorsMsg.TorqueEnable = true;
            obj.enableCli.call(enableMotorsMsg);
        end
        
        function disableRobot(obj)
            % Disables the motors in the robot.
            % WATCH OUT, the robot may fall if motors are disabled.
            disableMotorsMsg = rosmessage(obj.enableCli);
            disableMotorsMsg.TorqueEnable = false;
            obj.enableCli.call(disableMotorsMsg);
        end
        
        function enableClaw(obj)
            % Enables the claw motor
            % TODO
        end
        
        function disableClaw(obj)
            % Disables the claw motor
            % TODO
        end
        
        function setClaw(obj, dist)
            % Sets the claw to the specified jaw distance
            msg = rosmessage(obj.clawPub);
            msg.Data = dist;
            obj.clawPub.send(msg);
        end
        
        function homeRobot(obj)
            % Moves the robot into the home position
            homeRobotMsg = rosmessage(obj.homeCli);
            obj.homeCli.call(homeRobotMsg);
        end

%         DISABLED: We never ended up using this, running trajectories
%         worked just fine
%         function autoMove(obj, joints, duration)
%             % Automatically moves the robot to the given position.
%             % Does not do trajectory planning or collision avoidance, used
%             % for setting the robot to specific positions in a safe
%             % situation.
%             % Create the move command
%             autoMoveMsg = rosmessage(obj.commandPub);
%             autoMoveMsg.Points(1) = ros.msggen.trajectory_msgs.JointTrajectoryPoint;
%             autoMoveMsg.Points.Positions = joints';
%             autoMoveMsg.Points.TimeFromStart.Sec = duration;
%             % Send the message to the robot
%             obj.commandPub.send(autoMoveMsg);
%         end
        
        function movePTraj(obj, traj, frequency, accuracy)
            % Moves the robot through the list of joint positions
            % Frequency is the frequency at which the robot should move
            % through the points provided (positions per second).
            
            % Create the messages to send
            positionMsg = rosmessage(obj.positionPub);
            % Use a rate limiter to send the messages consistently
            rateLimiter = rateControl(frequency);
            rateLimiter.OverrunAction = 'slip';
            % Send each point in the trajectory
            rateLimiter.reset();
            for i = 1:(size(traj,1))
                % Load the joint positions into the message
                if i < size(traj,1)
                    positionMsg.Data = traj(i,:);
                else
                    positionMsg.Data = traj(size(traj,1),:);
                end
                % Send the trajectory to the robot
                obj.positionPub.send(positionMsg);
                rateLimiter.waitfor();
            end
            % Since the cyton driver is dodgy and takes a while to enter
            % position, keep sending until we're close enough 
            error = norm(traj(size(traj,1),:) - obj.getActualJoints);
            while error > accuracy
                obj.positionPub.send(positionMsg);
                rateLimiter.waitfor();
                error = norm(traj(size(traj,1),:) - obj.getActualJoints);
            end
        end
        
        function sendP(obj, jointPos)
            % Sends directly to the position publisher
            % This is for when you really need to optimise the speed you
            % send data to the robot at.
            obj.positionMsg.Data = jointPos;
            obj.positionPub.send(obj.positionMsg);
        end
        
        function sendV(obj, jointVel)
            % Sends directly to the velocity publisher
            % This is for when you really need to optimise the speed you
            % send data to the robot at
            obj.velocityMsg.Data = jointVel;
            obj.velocityPub.send(obj.velocityMsg);
        end
        
        function moveVTraj(obj, traj, frequency)
           % Apply the given velocities to the robot.
           % traj is a set of velocities each lasting for 1/frequency
           % seconds. Note that a zero velocity message will be appened to
           % the end of the list but you should include your own one for
           % sanity reasons.
           
           % Create the messages to send
           velocityMsg = rosmessage(obj.velocityPub);
           % Make the rate limiter to control the rate we send messages
           rateLimiter = rateControl(frequency);
           rateLimiter.OverrunAction = 'slip';
           % Load each of the trajectory points into the message
           rateLimiter.reset();
           for i = 1:size(traj,1)
                % Check to see if any of the joint velocities are over the
                % max.
                if (norm(double(traj(i,:) >= obj.maxJointVel)) > 0)
                    error("Large joint velocity detected");
                end
                % Load the point in the trajetory
                velocityMsg.Data = traj(i,:);
                % Send the trajectory to the robot.
                obj.velocityPub.send(velocityMsg);
                % Rate limiting
                rateLimiter.waitfor();
           end
           velocityMsg.Data = zeros(1, size(traj,2));
           % Append to the end of the trajectory
           obj.velocityPub.send(velocityMsg);
        end
        
        function joints = getActualJoints(obj)
            % Gets the joint values of the real robot
            jMsg = obj.stateSub.receive;
            joints = jMsg.Position(1:7)';
        end
    end
end