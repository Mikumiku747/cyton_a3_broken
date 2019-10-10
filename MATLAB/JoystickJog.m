classdef JoystickJog < handle
    % Reads input from a joystick for the purpose of jogging a robot.
    
    properties
        joystick                % Joystick Object
        runFrequency = 20;      % Estimated freqeuncy of control updates. (Hz)
        jogLinearSpeed = 0.1;   % Linear move speed (m/s)
        jogRotarySpeed = pi/4;  % Rotary move speed (rad/s)
    end
    properties (Constant)
        xAxisID = 1;            % X Axis ID
        yAxisID = 2;            % Y Axis ID
        rxAxisID = 5;           % Rotation X Axis ID
        ryAxisID = 4;           % Rotation Y Axis ID
        rzAxisPlus = 6;         % Rotation Z axis positive ID.
        rzAxisMinus = 3;        % Rotation Z axis negative ID.
        ZupButton = 6;          % Z Axis Positive Button
        zDownButton = 5;        % Z Axis Negative Button
        stopButton = 2;
    end
    
    methods
        function obj = JoystickJog(id)
            % Initialises the joystick
            if nargin < 1
                id = 1;
            end
            obj.joystick = vrjoystick(id);
        end
        
        function velocities = generateJogMovements(obj)
            % Generates tool velocities for jogging based on input.
            [axes, buttons, ~] = read(obj.joystick);
            % The X and Y movement axes are simple
            xVel = axes(obj.xAxisID) * obj.jogLinearSpeed / obj.runFrequency;
            yVel = -axes(obj.yAxisID) * obj.jogLinearSpeed / obj.runFrequency;
            % We have to convert the buttons to double to use them in
            % maths, and we also have to take the difference between the up
            % and down buttons to get the robot moving properly.
            zVel = double(buttons(obj.ZupButton)) * 0.5 ...
                * obj.jogLinearSpeed / obj.runFrequency ...
                - double(buttons(obj.zDownButton)) * 0.5 ...
                * obj.jogLinearSpeed / obj.runFrequency;
            % The X and Y rotary axes are simple as well
            xRot = -axes(obj.rxAxisID) * obj.jogRotarySpeed / obj.runFrequency;
            yRot = axes(obj.ryAxisID) * obj.jogRotarySpeed / obj.runFrequency;
            % We have to do some processing with the triggers for the Z
            % rotation, since each trigger starts at -1 for released and
            % goes to 1 for pressed. We scale them down and then take the
            % difference of the positive and negative triggers.
            zRot = 0.5 * (axes(obj.rzAxisPlus) + 1) ...
                * obj.jogRotarySpeed / obj.runFrequency ...
                - 0.5 * (axes(obj.rzAxisMinus) + 1) ...
                * obj.jogRotarySpeed / obj.runFrequency;
            % Construct the final output vector
            velocities = [xVel yVel zVel xRot yRot zRot];
        end
        
        function status = getStopStatus(obj)
            % Gets the status of the STOP button (Circle on DualShock4)
            [~, button, ~] = read(obj.joystick);
            status = button(obj.stopButton);
        end
        
        function closeController(obj)
            % Disconnects the controller
            close(obj.joystick);
            clear obj.joystick;
        end
    end
end