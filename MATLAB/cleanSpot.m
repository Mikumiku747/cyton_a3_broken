function cleanSpot()
    
    accuracy = 0.010;  % default accuracy
    cleanMove = 0.050;  % Size of the area to wipe around

    % Init the robot
    robot = HansCute;
    robot.connectToHW;
    
    % Take and annotate the picture
    c = webcam('EyeToy USB camera Namtai');
    im = c.snapshot();
    imshow(im);
    hold on
    objs = colorDetect(im);
    renderObjData(objs);
    hold off
    pause(3);
    
    
    % Teach the position
    robot.plot();
    robot.teachPosition;
    baseSpot = robot.getEndEffectorTransform
    pause(3);
    
    % Move above the spot
    above = baseSpot * transl(0, 0, -0.050);
    robot.moveJ(above, 3, accuracy);
    
    % Generate the cleaning points
    cPoints = {};
    cPoints{1} = baseSpot * transl(cleanMove/2, 0, 0);
    cPoints{2} = baseSpot * transl(-cleanMove/2, 0, 0);
    cPoints{3} = baseSpot * transl(0, cleanMove/2, 0) * trotz(pi/2);
    cPoints{4} = baseSpot * transl(0, -cleanMove/2, 0) * trotz(pi/2);
    
    % Do the cleaning procedure
    robot.moveJ(baseSpot, 0.75, accuracy);
    for i = 1:4
%         robot.moveL(cPoints{i}, accuracy);
        robot.moveJ(cPoints{i}, 0.25, accuracy);
    end
    robot.moveJ(above, 3, accuracy);
    
end