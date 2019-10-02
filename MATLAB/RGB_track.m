clc; clear;
close all; 
objects = imaqfind; %find video input objects in memory
delete(objects); %delete a video input object from memory

% Capture the video frames using the videoinput function
% You have to replace the resolution & your installed adaptor name.
vid = videoinput('winvideo', 1, 'YUY2_640x480');

% Set the properties of the video object
set(vid, 'FramesPerTrigger', inf);
set(vid, 'ReturnedColorspace', 'rgb')
vid.FrameGrabInterval = 5;

%start the video aquisition here
start(vid)

% Set a loop that stop after 100 frames of aquisition
while(vid.FramesAcquired<=200)
    
    % Get the snapshot of the current frame
    data = getsnapshot(vid);   
    imshow(data)
    %1 for red, 2 for green, and 3 for blue.
    redChannel  = data(:, :, 1);
    greenChannel  = data(:,:,2);
    blueChannel = data(:,:,3);  
    %Now to track objects in real time
    %Look at the difference between the color you want and the average of the other two.
    redDifference = redChannel - (greenChannel + blueChannel)/2;
    greenDifference = greenChannel - (redChannel + blueChannel)/2;
    blueDifference = blueChannel - (redChannel + greenChannel)/2;   
    %Use a median filter to filter out noise
    snap_red = medfilt2(redDifference, [3 3]);
    snap_blue = medfilt2(greenDifference, [3 3]);
    snap_green = medfilt2(blueDifference, [3 3]);   
    % Convert the resulting grayscale image into a binary image.
    snap_red = imbinarize(snap_red,0.18);
    snap_blue = imbinarize(snap_blue,0.18);
    snap_green = imbinarize(snap_green,0.18);  
    % Remove all those pixels less than 300px
    snap_red = bwareaopen(snap_red, 300);
    snap_blue = bwareaopen(snap_blue, 300);
    snap_green = bwareaopen(snap_green, 300);   
    % Label all the connected components in the image. 
    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats_red = regionprops(snap_red, 'BoundingBox','Centroid');
    stats_blue = regionprops(snap_blue, 'BoundingBox','Centroid');
    stats_green = regionprops(snap_green, 'BoundingBox','Centroid');
    
    % Display the image
    hold on
%     imshow(snap_blue);
%     imshow(snap_green);
      
    % Puts red objects in rectangular box
    % for object_red = 1:length(stats_red)
        bb_red = stats_red(object_red).BoundingBox;
        bc_red = stats_red(object_red).Centroid;
        rectangle('Position',bb_red,'EdgeColor','r','LineWidth',1)
        plot(bc_red(1),bc_red(2), 'or')
    end
%     % Puts blue objects in rectangular box
    for object_blue = 1:length(stats_blue)
        bb_blue = stats_blue(object_blue).BoundingBox;
        bc_blue = stats_blue(object_blue).Centroid;
        rectangle('Position',bb_blue,'EdgeColor','b','LineWidth',1)
        plot(bc_blue(1),bc_blue(2), 'ob')
    end
%     % Puts blue objects in rectangular box
    for object_green = 1:length(stats_green)
        bb_green = stats_green(object_green).BoundingBox;
        bc_green = stats_green(object_green).Centroid;
        rectangle('Position',bb_green,'EdgeColor','g','LineWidth',1)
        plot(bc_green(1),bc_green(2), 'og')
    end
    hold off
    
end
% Both the loops end here.

% Stop the video aquisition.
stop(vid);

% Flush all the image data stored in the memory buffer.
flushdata(vid);

% Clear all variables
clear all
sprintf('%s','That was all about Image tracking, Guess that was pretty easy :) ')
