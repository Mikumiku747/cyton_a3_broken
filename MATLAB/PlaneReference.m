classdef PlaneReference < handle
    % Class for computing points on a planar surface with thier own
    % coordinate scale
    
    properties
        baseTrans       % Stores the base transform for the plane
        scale           % Stores the conversion factor for plane coords
    end
    
    methods
        function obj = PlaneReference(baseT, xAxisT, yAxisT, scale)
            % Creates a new plane using three points as a reference. 
            % The first two points form the origin and X axis, and the
            % third is a point on the plane used to form the Y (and then Z)
            % axis.
            
            % Construct the base point
            basePos = baseT(1:3,4);
            % Construct the X Axis Vector
            xDiff = xAxisT(1:3,4) - baseT(1:3,4);
            xAxis = xDiff / norm(xDiff);
            % Construct a temporary Y axis (since the point provided may
            % not be fully perpendicular to the X axis)
            yDiff = yAxisT(1:3,4) - baseT(1:3,4);
            yAxisTmp = yDiff / norm(yDiff);
            % Construct the Z axis
            zAxis = cross(xAxis, yAxisTmp) / norm(cross(xAxis, yAxisTmp));
            % Re-Construct the Y Axis so it's definitely perpendicular
            yAxis = cross(xAxis, zAxis) / -norm(cross(xAxis, zAxis));
            % Craete the plane's base transform
            obj.baseTrans = ...
                [[xAxis yAxis zAxis basePos]' [0 0 0 1]']';
            % Save the scaling factor
            obj.scale = scale;
        end
        
        function worldCoords = convertFrom(obj, coord)
            % Converts from plane coordinates to real coordinates
            newTrans = obj.baseTrans * transl(...
                coord(1) / obj.scale(1), coord(2) / obj.scale(2),0);
            worldCoords = newTrans(1:3,4);
        end
        
        function planeCoords = convertTo(obj, trans)
            % Converts from real coordinates to plane coordinates.
            % This will project the given transform down to the plane if it
            % is not already in the plane.
            relativeTrans = obj.baseTrans\trans;
            relativeCoords = relativeTrans(1:2,4);
            planeCoords = relativeCoords' .* obj.scale;
        end
    end
end