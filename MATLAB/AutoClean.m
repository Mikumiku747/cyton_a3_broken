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
        cam_base    % Corresponding base point in image
    end
    
    methods
        
        function obj = AutoClean()
            % Starts the autocleaner

            % Start up the GUI
            addpath('./app');
            obj.GUI = assignment_2_app;
            obj.GUI.cleanerHandle = obj;
            % Plot for the robot
            figure(1)
            obj.robot = HansCute();
            obj.robot.plotModel();
            % Plot for the camera
            figure(2);
            obj.camera = webcam('EyeToy USB camera Namtai');
            obj.im = obj.camera.snapshot();
            imshow(obj.im);
            figure(1);
        end
        
        function connectRealHardware(obj)
            % Connects the hardware 
            obj.robot.connectToHW();
        end

        function scaleFactor = CalibrateScale(obj)
            % Calibrates the scale of objects on the screen
            uiwait(msgbox('The camera scaling factor will be calibrated.'));
            obj.processImage();
            bpID = inputdlg("Enter the ID of the first scale point: ");
            rpID = inputdlg("Enter the ID of the second scale point: ");
            bp = obj.objects(str2num(bpID{1})).Centroid;
            rp = obj.objects(str2num(rpID{1})).Centroid;
            dist = norm(bp - rp);
            uiwait(msgbox(sprintf('The points are %.1f pixels apart.\n', dist)));
            trueDistText = inputdlg("Enter true distance in m: ");
            trueDist = str2double(trueDistText{1});
            scaleFactor = (dist / trueDist);
            obj.cam_scale = [1 1] * scaleFactor;
            uiwait(msgbox('Calibration complete!'));
        end

        function CalibratePlane(obj)
            % Calibrate the plane, requires scale

            % Get the base point
            uiwait(msgbox('Please calibrate the base point using the robot in teach mode:'));
            obj.cal_base = obj.robot.getEndEffectorTransform(obj.robot.teachPosition());
            uiwait(msgbox('Please calibrate the X Axis endpoint.'));
            obj.cal_xAxis = obj.robot.getEndEffectorTransform(obj.robot.teachPosition());
            uiwait(msgbox('Please calibrate the Y Axis endpoint.'));
            obj.cal_yAxis = obj.robot.getEndEffectorTransform(obj.robot.teachPosition());
            
            % Create the plane
            obj.plane = PlaneReference(obj.cal_base, obj.cal_xAxis, ...
                obj.cal_yAxis, obj.cam_scale);
            uiwait(msgbox('Plane has been successfully calibrated.'));
        end
        
        function bp = CalibrateBasePoint(obj)
            % Allows selection of the base point when calibrating the
            % camera
            uiwait(msgbox('The camera base point will be selected.'));
            obj.processImage();
            bpIDText = inputdlg('Enter the number of the base point used for plane calibration: ', 'Base point Calibration');
            bpID = str2double(bpIDText{1});
            bp = obj.objects(bpID).Centroid;
            obj.cam_base = bp;
            uiwait(msgbox('Camera Base Point Calibration Complete'));
        end
        
        function transform = camToWorld(obj, camcoords)
            % Covnerts a set of camera coordinates to world coordinates
            transform = obj.cal_base;
            planeCoords = obj.cam_base - camcoords;
            transform(1:3,4) = obj.plane.convertFrom(planeCoords);
        end
        
        function objs = processImage(obj)
            % Processes what is seen by the camera
            obj.im = obj.camera.snapshot();
            figure(2);
            hold off
            imshow(obj.im);
            hold on;
            objs = colorDetect(obj.im);
            obj.objects = objs;
            renderObjData(objs);
            figure(1);
            hold off;
        end
        
        function cleanobject(obj, ID)
            % Clean one of the objects pictured in the image
            cleanPoint = obj.camToWorld(obj.objects(ID).Centroid);
            wayPoints = cell.empty(6,0);
            wayPoints{1} = cleanPoint*transl(0,0,-0.04);
            wayPoints{2} = cleanPoint;
            wayPoints{3} = cleanPoint*transl(-0.025, 0, 0);
            wayPoints{4} = cleanPoint*transl(0.025, 0, 0);
            wayPoints{5} = cleanPoint*transl(0, -0.025, 0);
            wayPoints{6} = cleanPoint*transl(0, 0.025, 0);
            wayPoints{7} = wayPoints{1};
            for i = 1:7
                obj.robot.moveJ(wayPoints{i}, 1, 0.01);
            end
            obj.robot.moveQ(obj.robot.q0, 3, 0.0075);
        end
        
        function cleanObjectAuto(obj)
            % Same as the cleaning function above but automatically works
            % on points.
            while (~obj.robot.controller.getStopStatus)
                obj.processImage();
                if (size(obj.objects,1) > 0)
                    obj.cleanobject(1);
                end
            end
        end
    end
end