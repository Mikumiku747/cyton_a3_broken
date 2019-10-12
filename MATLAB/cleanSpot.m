function cleanSpot()
    
    accuracy = 0.010;  % default accuracy
    cleanMove = 0.050;  % Size of the area to wipe around

    % Init the robot
    robot = HansCute;
    robot.connectToHW;
    
    close 1
    
    % Take and annotate the picture
    disp 'Getting Picture';
    hold off
    c = webcam('EyeToy USB camera Namtai');
    im = c.snapshot();
    imshow(im);
    hold on
    objs = colorDetect(im);
    renderObjData(objs);
    hold off
    
    % Wait for the teach button to be pressed again
    while ~robot.controller.getTeachStatus
    end
    
    % Teach the position
    disp 'Learning cleaning location';
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
        robot.moveJ(cPoints{i}, 0.2, accuracy);
    end
    robot.moveJ(above, 3, accuracy);
    
end