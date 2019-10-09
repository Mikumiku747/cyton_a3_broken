function [] = clearWebcam()

%find video input objects in memory
objects = imaqfind; 

%delete a video input object from memory
delete(objects); 

end

