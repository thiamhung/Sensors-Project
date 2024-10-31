function [highlighted_img,centroids, y] = detect_yellow(img)
    % Convert the input image to HSV color space
    hsv_img = rgb2hsv(img);

    % Extract the Hue, Saturation, and Value channels
    h = hsv_img(:, :, 1);
    s = hsv_img(:, :, 2);
    v = hsv_img(:, :, 3);

    % Define the yellow color thresholds in HSV
    hue_lower_threshold = 0.1;    % Adjust as needed
    hue_upper_threshold = 0.38;    % Adjust as needed
    saturation_threshold = 0.8;    % Adjust as needed
    value_threshold = 0.2;         % Adjust as needed

    % Create masks for yellow color detection
    hue_mask = (h >= hue_lower_threshold) & (h <= hue_upper_threshold);
    saturation_mask = (s >= saturation_threshold);
    value_mask = (v >= value_threshold);

    % Combine the masks to detect yellow regions
    yellow_mask = hue_mask & saturation_mask & value_mask;

    % Check if yellow is detected
    y = any(yellow_mask(:));

    % Perform morphological operations on the yellow mask
    yellow_mask = imfill(yellow_mask, 'holes');
    yellow_mask = bwmorph(yellow_mask, 'erode', 2);
    yellow_mask = bwmorph(yellow_mask, 'dilate', 3);
    yellow_mask = imfill(yellow_mask, 'holes');

    % Find connected components and region properties
    stats = regionprops(yellow_mask, 'Centroid');
    
    if isempty(stats)
        y = 0;
        centroids = []; % No yellow objects found
    else
        y = 1;
        centroids = vertcat(stats.Centroid);
    end

    % Highlight the detected regions
    imgBoth = imoverlay(img, yellow_mask);

    % Highlight each centroid with a blue color
    if y == 1
        for i = 1:size(centroids, 1)
            % Draw the marker
            imgBoth = insertMarker(imgBoth, centroids(i, :), 'x', 'color', 'blue', 'size', 10);
            
            % Annotate the centroid position
            positionText = sprintf('(%d, %d)', round(centroids(i, 1)), round(centroids(i, 2)));
            imgBoth = insertText(imgBoth, centroids(i, :), positionText, 'FontSize', 12, 'BoxColor', 'yellow', 'BoxOpacity', 0.8, 'TextColor', 'black');
        end
    end

    % Return the resulting image and the flag indicating yellow detection
    highlighted_img = imgBoth;
end