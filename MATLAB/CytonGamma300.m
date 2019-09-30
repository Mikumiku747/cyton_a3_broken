classdef CytonGamma300
    % CytonGamma300 Cyton Gamma 300 robot class.
    % For use with Peter Corke's Robotics, Vision and Control Toolbox.
    
    properties
        DHParams        % DH Parameters describing the robot geometry
        nJoints         % Number of joints in the robot
        robotModel      % SerialLink object describing the robot
        joints          % Joint Positions
        
    end
    
    methods
        
        function obj = set.joints(obj, joints)
            % Validation that the new joint position is within joint limits
            for i = 1:obj.nJoints
                % TODO: Validation here
            end
            obj.joints = joints;
        end
        
        function obj = CytonGamma300(name)
            % Creates a new Cyton 300 Robot Object. Default pose is 0
            if nargin < 1
                name = "Cyton Gamma 300";
            end
            obj.DHParams = [
                %   z       x       alpha       range       offset
                    0.120   0       90.00       300.0       0
                    0       0       -90.00      210.0       0
                    0.141   0       90.00       300.0       0
                    0       0.072   -90.00      210.0       90.00
                    0       0.072   90.00       210.0       0
                    0       0       90.00       210.0       90.00
                    0.130   0       180.0       300.0       180.0
                ];
            obj.nJoints = size(obj.DHParams,1);
            links = Link.empty(obj.nJoints, 0);
            for i = 1:size(obj.DHParams,1)
                links(i) = Link('d', obj.DHParams(i,1), ...
                    'a', obj.DHParams(i,2), 'alpha', deg2rad(obj.DHParams(i,3)), ...
                    'qlim', deg2rad([-obj.DHParams(i,4)/2, obj.DHParams(i,4)/2]), ...
                    'offset', deg2rad(obj.DHParams(i,5)));
            end
            obj.robotModel = SerialLink(links, 'name', name);
            obj.joints = zeros(1, obj.nJoints);
        end
        
        function teach(obj)
            % Opens a figure with the robot for teaching new poses
            obj.robotModel.teach(obj.joints);
        end
        
        function joints = getJoints(obj)
            % Gets the current joint positions of the robot
            joints = obj.robotModel.getpose();
        end
        
        function transform = getEndEffectorTransform(obj, joints)
            % Uses forward kinematics to get the position of the end
            % effector.
            if nargin < 2
                joints = obj.joints;
            end
            transform = obj.robotModel.fkine(joints);
        end
        
        function jacobian = getJacobian(obj, joints)
            % Gets the jacobian for a given robot joint position. 
            % If no position is supplied, the current pose is used.
            if nargin < 2
                joints = obj.joints;
            end
            jacobian = obj.robotModel.jacob0(joints);
        end
        
        function plot(obj)
            % Plots the robot
            obj.robotModel.plot(obj.joints);
        end
        
        function animate(obj)
            % Updates the plot of a robot
            obj.robotModel.animate(obj.joints);
        end
    end
end