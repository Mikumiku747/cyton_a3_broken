classdef AutoClean < handle
    % Automated surface cleaning using a camera and a robot
    
    properties
        % General properties
        robot       % The robot this class is using
        GUI         % The GUI used to interact with the class
        plane       % The plane used as a reference for the camera
        camera      % The camera used to take
        im          % The image in the camera
        objects     % Objects ID'd by the camera
        % Camera calibration
        cal_base    % Base Point
        cal_xAxis   % X Axis Point
        cal_yAxis   % Y Axis Point
        cam_scale   % Scaling Information
    end
    
    methods
        
        function obj = AutoClean()
            % Starts the autocleaner

            % Plot for the robot
            figure(1)
            obj.robot = HansCute();
            obj.robot.plot()
            obj.robot.connectToHW();
            % Plot for the camera
            figure(2);
            obj.camera = webcam('EyeToy USB camera Namtai');
            obj.im = obj.camera.snapshot();
            imshow(obj.im);
            figure(1);
        end

        function CalibrateScale(obj)
            % TODO: Camera scaling calibration
            obj.cam_scale = [1 1];
        end

        function CalibratePlane(obj)
            % Calibrate the plane, requires scale

            % Get the base point
            disp('Please calibrate the base point using the robot in teach mode:');
            obj.cal_base = obj.robot.teachPosition();
            disp('Please calibrate the X Axis endpoint.');
            obj.cal_xAxis = obj.robot.teachPosition();
            disp('Please calibrate the Y Axis endpoint.');
            obj.cal_yAxis = obj.robot.teachPosition();
            
            % Create the plane
            obj.plane = PlaneReference(obj.cal_base, obj.cal_xAxis, ...
                obj.cal_yAxis, obj.cam_scale);
        end
    end

end