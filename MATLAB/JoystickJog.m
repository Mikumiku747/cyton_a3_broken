classdef JoystickJog < handle
    % Reads input from a joystick for the purpose of jogging a robot.
    
    properties
        joystick                % Joystick Object
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
        stopButton = 2;         % Stop Robot Button
        teachButton = 4;        % For teaching the robot positions
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
            xVel = axes(obj.xAxisID) * obj.jogLinearSpeed;
            yVel = -axes(obj.yAxisID) * obj.jogLinearSpeed;
            % We have to convert the buttons to double to use them in
            % maths, and we also have to take the difference between the up
            % and down buttons to get the robot moving properly.
            zVel = double(buttons(obj.ZupButton)) * 0.5 ...
                * obj.jogLinearSpeed ...
                - double(buttons(obj.zDownButton)) * 0.5 ...
                * obj.jogLinearSpeed;
            % The X and Y rotary axes are simple as well
            xRot = -axes(obj.rxAxisID) * obj.jogRotarySpeed;
            yRot = axes(obj.ryAxisID) * obj.jogRotarySpeed;
            % We have to do some processing with the triggers for the Z
            % rotation, since each trigger starts at -1 for released and
            % goes to 1 for pressed. We scale them down and then take the
            % difference of the positive and negative triggers.
            zRot = 0.5 * (axes(obj.rzAxisPlus) + 1) ...
                * obj.jogRotarySpeed ...
                - 0.5 * (axes(obj.rzAxisMinus) + 1) ...
                * obj.jogRotarySpeed;
            % Construct the final output vector
            velocities = [xVel yVel zVel xRot yRot zRot];
        end
        
        function status = getStopStatus(obj)
            % Gets the status of the STOP button (Circle on DualShock4)
            [~, button, ~] = read(obj.joystick);
            status = button(obj.stopButton);
        end
        
        function status = getTeachStatus(obj)
            % Gets the status of the Teach button (Square on the DS4)
            [~, button, ~] = read(obj.joystick);
            status = button(obj.teachButton);
        end
        
        function closeController(obj)
            % Disconnects the controller
            close(obj.joystick);
            clear obj.joystick;
        end
    end
end