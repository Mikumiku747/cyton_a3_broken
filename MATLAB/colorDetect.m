function objects = colorDetect(image, theshold, minSize)
    % Detects objects which aren't white
    
    if nargin < 2
        threshold = 0.15;
    end
    if nargin < 3
        minSize = 100;
    end
    
    % First we do a series of image filtering steps
    % Covnert to HSV color space
    im_hsv = rgb2hsv(image);
    % Filter based on saturation value
    im_sat = im_hsv(:,:,2) >= threshold;
    % Use a filter to de-noise the image
    im_noiseless = medfilt2(im_sat);
    % Remove small objects (which the noise filter can't remove)
    im_objs = bwareaopen(im_noiseless, minSize);
    
    % Next, we need to identify the objects in the image
    objects = regionprops(im_objs, 'BoundingBox', 'Centroid');
    
end