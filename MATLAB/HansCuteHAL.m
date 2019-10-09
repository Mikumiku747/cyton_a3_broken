classdef HansCuteHAL < handle
    % Hans Cute Robot Hardware Abstraction Layer
    % This class encapsulates the common functions for using actual
    % hardware for the hans robot. This includes connection, status,
    % converting trajectories to a format which the robot understands, and
    % power controls. 
    
    properties
        commandPub      % Position Command Publisher (writer)
        stateSub        % Position State Subscriber (reader)
        enableMotorsCli % Motor Control Client (caller)
        homeRobotCli    % Home Robot Client (caller)
    end
    properties (Constant)
        jointNames = ...    % Joint Names for use in trajectory messages
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',...
            'joint7'];
        maxJointVel = ...   % Max value for joint velocities (sanity check)
            pi/2;           % Max value of 90 degrees per second
    end
    
    methods
        function obj = HansCuteHAL()
            % Creates a new hardware connection
            % You should only make one of these
            obj.commandPub = rospublisher('/cute_arm_controller/command');
            obj.stateSub = rossubscriber('/cute_arm_controller/state');
            obj.enableMotorsCli = rossvcclient('/cute_torque_enable');
            obj.homeRobotCli = rossvcclient('/cute_go_home');
        end
        
        function enableRobot(obj)
            % Enables the motors in the robot.
            enableMotorsMsg = rosmsg(obj.enableMotorsCli);
            obj.enableMotorsCli.call(enableMotorsMsg, 'data', true);
        end
        
        function disableRobot(obj)
            % Disables the motors in the robot.
            % WATCH OUT, the robot may fall if motors are disabled.
            disableMotorsMsg = rosmsg(obj.enableMotorsCli);
            obj.enableMotorsCli.call(disableMotorsMsg, 'data', false);
        end
        
        function homeRobot(obj)
            % Moves the robot into the home position
            homeRobotMsg = rosmsg(obj.homeRobotCli);
            ojb.homeRobotCli.call(homeRobotMsg, 'data', true);
        end
        
        function autoMove(obj, joints, duration)
            % Automatically moves the robot to the given position.
            % Does not do trajectory planning or collision avoidance, used
            % for setting the robot to specific positions in a safe
            % situation.
            % Create the move command
            autoMoveMsg = rosmessage(obj.commandPub);
            autoMoveMsg.Points = ros.msggen.trajectory_msgs.JointTrajectoryPoint;
            autoMoveMsg.Points.Positions = joints';
            autoMoveMsg.Points.TimeFromStart.Sec = duration;
            % Send the message to the robot
            obj.commandPub.send(autoMoveMsg);
        end
        
        function movePTraj(obj, traj, frequency)
            % Moves the robot through the list of joints
            % Frequency is the frequency at which the robot should move
            % through the points provided (positions per second).
            
            % Compute the time between joints
            period = 1/frequency;
            % Create the messages to send
            moveMsg = rosmsg(obj.commandPub);
            moveMsg.JointNames = obj.jointNames;
            % Create the trajectory
            for i = 1:size(traj,1)
                % Create the trajectory point
                moveMsg.Points(i) = ros.msggen.trajectory_msgs.JointTrajectoryPoint;
                % Mark the time when this point should be reached
                [s, ns] = splitTime(period * (i / size(traj,1)));
                moveMsg.Points(i).TimeFromStart.Sec = s;
                moveMsg.Points(i).TimeFromStart.Nsec = ns;
                % Load the point in the trajetory
                moveMsg.Points(i).Positions = traj(i,:);
            end
            % Send the trajectory to the robot
            obj.commandPub.send(moveMsg);
        end
        
        function moveVTraj(obj, traj, frequency)
           % Apply the given velocities to the robot.
           % traj is a set of velocities each lasting for 1/frequency
           % seconds. Note that a zero velocity message will be appened to
           % the end of the list but you should include your own one for
           % sanity reasons.
           
           % Compute the time between samples
           period = 1/frequency;
           % Create the messages to send
           moveMsg = rosmsg(obj.commandPub);
           moveMsg.JointNames = obj.jointNames;
           % Load each of the trajectory points into the message
           for i = 1:size(traj,1)
                % Create the trajectory point
                moveMsg.Points(i) = ros.msggen.trajectory_msgs.JointTrajectoryPoint;
                % Mark the time when this point should be reached
                [s, ns] = splitTime(period * (i / size(traj,1)));
                moveMsg.Points(i).TimeFromStart.Sec = s;
                moveMsg.Points(i).TimeFromStart.Nsec = ns;
                % Check to see if any of the joint velocities are over the
                % max.
                if (norm(double(traj(i,:) >= obj.maxJointVel / frequency)) > 0)
                    error("Large joint velocity detected");
                end
                % Load the point in the trajetory
                moveMsg.Points(i).Velocities = traj(i,:);
           end
           % Add a zero velocity move to the end
           lasti = size(traj,1) + 1;
           pt = ros.msggen.trajectory_msgs.JointTrajectoryPoint;
           [s, ns] = splitTime(period * lasti);
           pt.TimeFromStart.Sec = s;
           pt.TimeFromStart.Nsec = ns;
           % Load the final zero velocity set
           pt.Velocities = zeros(1,size(traj,2));
           % Append to the end of the trajectory
           moveMsg.Points(lasti) = pt;
           % Send the trajectory to the robot.
           obj.commandPub.send(moveMsg);
        end
    end
end

function [sec, nanosec] = splitTime(time)
    % Splits the time into whole and nanoseconds
    sec = floor(time);
    nanosec = (time - floor(time)) * 1000000;
end