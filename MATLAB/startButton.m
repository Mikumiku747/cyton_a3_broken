% This function run when the startButton in the app is pushed
function startButton(app)

switch app.mode
    case 1  % AutoClean Mode
        app.cleanerHandle.cleanObjectAuto();
        
    case 2  % AutoMove Mode
        x = app.XEditField.value();
        y = app.YEditField.value();
        z = app.ZEditField.value();
        trans = app.cleanerHandle.robot.robotModel.fkine(app.cleanerHandle.robot.joints());
        trans(1:3,4) = [x y z]';
        app.cleanerHandle.robot.moveJ(trans);
        
    case 3  % Controller
        app.cleanerHandle.robot.jogMode();
        
    otherwise
end

