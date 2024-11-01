function [color_detected, centroidsGreen, g] = detect_green(image)
    % DETECT_GREEN - Function to detect green objects in the input image
    %
    % Inputs:
    %   image - RGB image to be processed
    %
    % Outputs:
    %   color_detected - RGB image showing detected green areas
    %   centroidsGreen - Centroids of detected green objects
    %   g              - Flag indicating if any green objects were detected

    % Convert the image from RGB to HSV color space
    hsvImage = rgb2hsv(image);

    % Define thresholds for the green color in HSV space
    hueMin = 0.2;       % Lower end of green hue
    hueMax = 0.5;       % Upper end of green hue
    satMin = 0.1;       % Minimum saturation
    satMax = 1.0;       % Maximum saturation
    valMin = 0.2;       % Minimum value (brightness)
    valMax = 1.0;       % Maximum value (brightness)

    % Create binary mask based on the threshold values
    greenMask = (hsvImage(:,:,1) >= hueMin & hsvImage(:,:,1) <= hueMax) & ...
                (hsvImage(:,:,2) >= satMin & hsvImage(:,:,2) <= satMax) & ...
                (hsvImage(:,:,3) >= valMin & hsvImage(:,:,3) <= valMax);

    % Perform morphological operations to clean up the mask
    greenMask = imopen(greenMask, strel('disk', 5)); % Larger disk for more noise reduction
    greenMask = imclose(greenMask, strel('disk', 5));

    % Find connected components and calculate centroids
    stats = regionprops(greenMask, 'Centroid', 'Area');

    % Filter out small areas
    areaThreshold = 100; % Lower minimum area threshold
    stats = stats([stats.Area] > areaThreshold);

    % Extract centroids
    centroidsGreen = cat(1, stats.Centroid);

    % Highlight detected regions in the image
    color_detected = image;
    color_detected(repmat(~greenMask, [1, 1, 3])) = 0; % Set non-green areas to black

    % Set the detection flag
    g = ~isempty(centroidsGreen);

    % Optionally, annotate centroids on the image
    if g
        for i = 1:size(centroidsGreen, 1)
            color_detected = insertMarker(color_detected, centroidsGreen(i, :), 'x', 'color', 'green', 'size', 10);
            positionText = sprintf('(%d, %d)', round(centroidsGreen(i, 1)), round(centroidsGreen(i, 2)));
            color_detected = insertText(color_detected, centroidsGreen(i, :), positionText, 'FontSize', 12, 'BoxColor', 'yellow', 'BoxOpacity', 0.8, 'TextColor', 'black');
        end
    end
end
