function renderObjData(objects)
    % Renders objects identified by colorDetect to the plot
    % Ensure that hold is on or only the last data point will be shown
    
    for i = 1:size(objects,1)
        % Render the bounding box
        rectangle('Position',objects(i).BoundingBox, 'EdgeColor','r',...
            'LineWidth',2);
        % Render a crosshair on the centre
        plot(objects(i).Centroid(1), objects(i).Centroid(2),'-m+');
        text(objects(i).Centroid(1)+5, objects(i).Centroid(2)+5, string(i));
    end
end