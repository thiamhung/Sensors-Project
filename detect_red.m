function [highlighted_img, centroids, r] = detect_red(img)
    % Convert the input image to HSV color space
    hsv_img = rgb2hsv(img);

    % Extract the Hue, Saturation, and Value channels
    h = hsv_img(:, :, 1);
    s = hsv_img(:, :, 2);
    v = hsv_img(:, :, 3);

    % Define the red color thresholds in HSV
    hue_lower_threshold = 0;    
    hue_upper_threshold = 0.1;   
    saturation_lower_threshold = 0.6; 
    value_lower_threshold = 0.5;   
    
    % Create masks for red color detection
    hue_mask = (h >= hue_lower_threshold & h <= hue_upper_threshold) | ...
               (h >= 0.9 | h <= 1);  % Add the upper range for red
    saturation_mask = (s >= saturation_lower_threshold);
    value_mask = (v >= value_lower_threshold);

    % Combine the masks to detect red regions
    red_mask = hue_mask & saturation_mask & value_mask;

    % Perform morphological operations on the red mask
    red_mask = imfill(red_mask, 'holes');
    red_mask = bwmorph(red_mask, 'erode', 2);
    red_mask = bwmorph(red_mask, 'dilate', 3);
    red_mask = imfill(red_mask, 'holes');

    % Find connected components and region properties
    stats = regionprops(red_mask, 'Centroid');
    
    if isempty(stats)
        r = 0;
        centroids = []; % No red objects found
    else
        r = 1;
        centroids = vertcat(stats.Centroid);
    end

    % Highlight the detected regions
    imgBoth = imoverlay(img, red_mask);

    % Highlight each centroid with a marker and show coordinates
    if r == 1
        for i = 1:size(centroids, 1)
            % Draw the marker
            imgBoth = insertMarker(imgBoth, centroids(i, :), 'x', 'color', 'blue', 'size', 10);
            
            % Annotate the centroid position
            positionText = sprintf('(%d, %d)', round(centroids(i, 1)), round(centroids(i, 2)));
            imgBoth = insertText(imgBoth, centroids(i, :), positionText, 'FontSize', 12, 'BoxColor', 'yellow', 'BoxOpacity', 0.8, 'TextColor', 'black');
        end
    end

    % Return the resulting image and the flag indicating red detection
    highlighted_img = imgBoth;
end
