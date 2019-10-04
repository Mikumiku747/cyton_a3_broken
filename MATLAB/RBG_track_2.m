clc; clear;
close all; 
objects = imaqfind; %find video input objects in memory
delete(objects); %delete a video input object from memory

% Capture the video frames using the videoinput function
% You have to replace the resolution & your installed adaptor name.

redThresh = 0.255; % Threshold for red detection
greenThresh = 0.24; % Threshold for green detection
blueThresh = 0.04; % Threshold for blue detection

vid = videoinput('winvideo', 1, 'YUY2_640x480');

% Set the properties of the video object
set(vid, 'FramesPerTrigger', inf);
set(vid, 'ReturnedColorspace', 'rgb')
vid.FrameGrabInterval = 5;

%start the video aquisition here
start(vid)

% Set a loop that stop after 100 frames of aquisition
while(vid.FramesAcquired<=inf)
    
    % Get the snapshot of the current frame
    rgbFrame = getsnapshot(vid);   
    imshow(rgbFrame)
    %1 for red, 2 for green, and 3 for blue.
    redChannel  = rgbFrame(:, :, 1);
    greenChannel  = rgbFrame(:,:,2);
    blueChannel = rgbFrame(:,:,3);  
    %Now to track objects in real time
    %Look at the difference between the color you want and the average of the other two. 
    redDifference = imsubtract(redChannel, rgb2gray(rgbFrame));
    greenDifference = imsubtract(greenChannel, rgb2gray(rgbFrame));
    blueDifference = imsubtract(blueChannel, rgb2gray(rgbFrame)); 
    %Use a median filter to filter out noise
    snap_red = medfilt2(redDifference, [3 3]);
    snap_blue = medfilt2(greenDifference, [3 3]);
    snap_green = medfilt2(blueDifference, [3 3]);   
    % Convert the resulting grayscale image into a binary image.
    snap_red = imbinarize(snap_red,redThresh);
    snap_blue = imbinarize(snap_blue,blueThresh);
    snap_green = imbinarize(snap_green,greenThresh);  
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
      
    % Puts red objects in rectangular box
    for object_red = 1:length(stats_red)
        bb_red = stats_red(object_red).BoundingBox;
        bc_red = stats_red(object_red).Centroid;
        rectangle('Position',bb_red,'EdgeColor','r','LineWidth',2)
        plot(bc_red(1),bc_red(2), '-m+')
    end
%     % Puts blue objects in rectangular box
    for object_blue = 1:length(stats_blue)
        bb_blue = stats_blue(object_blue).BoundingBox;
        bc_blue = stats_blue(object_blue).Centroid;
        rectangle('Position',bb_blue,'EdgeColor','b','LineWidth',2)
        plot(bc_blue(1),bc_blue(2), '-m+')
    end
%     % Puts blue objects in rectangular box
    for object_green = 1:length(stats_green)
        bb_green = stats_green(object_green).BoundingBox;
        bc_green = stats_green(object_green).Centroid;
        rectangle('Position',bb_green,'EdgeColor','g','LineWidth',2)
        plot(bc_green(1),bc_green(2), '-m+')
    end
    hold off
    
end
