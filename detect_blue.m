function [highlighted_img, centroids, b] = detect_blue(img)
    % Convert the input image to HSV color space
    hsi_img = rgb2hsv(img);

    % Extract the Hue, Saturation, and Value channels
    h = hsi_img(:, :, 1);
    s = hsi_img(:, :, 2);
    v = hsi_img(:, :, 3);

    % Define the blue color thresholds in HSV
    hue_lower_threshold = 0.5;    
    hue_upper_threshold = 0.72;   
    saturation_lower_threshold = 0.7; 
    value_lower_threshold = 0.5;   

    % Create masks for blue color detection
    hue_mask = (h >= hue_lower_threshold) & (h <= hue_upper_threshold);
    saturation_mask = (s >= saturation_lower_threshold);
    value_mask = (v >= value_lower_threshold);

    % Combine the masks to detect blue regions
    blue_mask = hue_mask & saturation_mask & value_mask;

    % Perform morphological operations on the blue mask if needed
    blue_mask = imfill(blue_mask, 'holes');
    blue_mask = bwmorph(blue_mask, 'erode', 2);
    blue_mask = bwmorph(blue_mask, 'dilate', 3);
    blue_mask = imfill(blue_mask, 'holes');

    % Find connected components and region properties
    stats = regionprops(blue_mask, 'Centroid');
    
    if isempty(stats)
        b = 0;
        centroids = []; % No blue objects found
    else
        b = 1;
        centroids = vertcat(stats.Centroid);
    end

    % Highlight the detected regions
    imgBoth = imoverlay(img, blue_mask);

    % Highlight each centroid with a blue color
    if b == 1
        for i = 1:size(centroids, 1)
            % Draw the marker
            imgBoth = insertMarker(imgBoth, centroids(i, :), 'x', 'color', 'blue', 'size', 10);
            
            % Annotate the centroid position
            positionText = sprintf('(%d, %d)', round(centroids(i, 1)), round(centroids(i, 2)));
            imgBoth = insertText(imgBoth, centroids(i, :), positionText, 'FontSize', 12, 'BoxColor', 'yellow', 'BoxOpacity', 0.8, 'TextColor', 'black');
        end
    end

    % Return the resulting image and the flag indicating blue detection
    highlighted_img = imgBoth;
end
