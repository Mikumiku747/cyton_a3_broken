classdef UR3 < handle
    properties
        %> Robot model
        model;
        %> Num of steps
        steps = 40;
        %qMatrix = ones(1,3,step.steps);
        %> workspace
        workspace = [-1 1 -1 1 -0.3 1];   
               
    end

    
    methods
        %% 
        % ======================================================================
        %> @brief Class constructor
        %>
        %> Create a UR3 
        %>
        %> @param name Robot name
        % ======================================================================
        function self = UR3(name)
            pause(0.001);
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0); % was 'offset',pi/2
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0); % was 'offset',pi/2
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0); 
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);                     
        end

        %%
        % ======================================================================
        %> @brief Class constructor
        %>
        %> Create a UR3 
        %>
        %> @param name Robot name
        % ======================================================================
        function SetBaseLocation(self,transX,transY,transZ,rot)
            
            self.model.base = eye*transl(transX,transY,transZ)*trotz(rot)
            
        end
        
        %% PlotAndColourRobot
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            hold all
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %% Move
        function Move(self,q1,q2)
            qMatrix = jtraj(q1,q2,self.steps);
            s = lspb(0,1,self.steps);               % First, create the scalar function
            qMatrix = nan(self.steps,self.model.n);            % Create memory allocation for variables
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;      % Generate interpolated joint angles
            end
                self.model.animate(qMatrix)
        end
        %% MoveModel
        function MoveModel(self,q1,q2)
            qMatrix = jtraj(q1,q2,self.steps);
            
            s = lspb(0,1,self.steps);               
            qMatrix = nan(self.steps,6);            
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;      
            end
            for i=1:self.steps
                self.model.fkine(qMatrix(i,:))
                self.model.animate(qMatrix(i,:))
                pause(0.1)
            end
        end
        %%
%         function fkine = GetFkine(self,qtemp)
%                 fkine = self.model.fkine((qtemp.qMatrix))
%         end   
        %% GetIkine
        function ikine = GetIkine(self,T,q)
            ikine = self.model.ikine(T,q,[1,1,1,1,0,0]);
        end
    end
end