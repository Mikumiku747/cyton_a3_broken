robot1 = UR3('robot1');
robot2 = UR3('robot2');
robot1.SetBaseLocation(-0.22,0.22,0,0);
robot2.SetBaseLocation(0.22,0.22,0,0);
startPos = [0,-pi/2,0,pi,pi/2,0];
%%
% Table(0,0,0);
% hold all
% Cage(0,0,0);

robot1.PlotAndColourRobot
robot1.model.animate(startPos)
hold on
%robot2.PlotAndColourRobot
%robot2.model.animate(startPos)
