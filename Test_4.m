clear all;
clc;
close all;
rosshutdown;

%% Start Dobot Magician Node
rosinit('192.168.27.1');

%% Start Webcam
try
    cam = webcam(2);
    preview(cam);
    disp('Camera initialized.');
catch
    error('No camera found. Exiting...');
end

% Initiate Dobot class for controlling the robot
dobot = DobotMagician();

while true
    % Reconfigure the joint into Initial Joint Position
    pause(1);
    q = [0 0 0 0];
    qw = [-0.5 0.05 0.5 0];
    dobot.PublishTargetJoint(q);
    pause(1);

    % Capture image
    img = snapshot(cam);

    % Detect colors: Red, Green, Blue
    [color_detected, centroidsRed, r] = detect_color(img, 'red');
    [color_detected, centroidsGreen, g] = detect_color(img, 'green');
    [color_detected, centroidsBlue, b] = detect_color(img, 'blue');

    % Overlay detected centroids on the image
    hold on;
    plot(centroidsRed(:,1), centroidsRed(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(centroidsGreen(:,1), centroidsGreen(:,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(centroidsBlue(:,1), centroidsBlue(:,2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    hold off;

    % Update the preview with detected points
    set(image_h, 'CData', color_detected);
    drawnow;

    % Add your existing logic here for processing centroids, moving the robot, etc.
    % ...

    % Ask user if they want to loop again
    if ~input('Loop again? (1 for yes, 0 for no): ')
        break;
    end
end

function [detectedImage, centroids, result] = detect_color(image, color)
    % Convert the image to HSV color space
    hsvImage = rgb2hsv(image);
    
    % Initialize result
    result = 0;
    centroids = [];
    
    switch color
        case 'red'
            % Define thresholds for red color
            lowerThreshold1 = [0, 0.5, 0.5]; % Lower bound
            upperThreshold1 = [0.05, 1, 1]; % Upper bound
            lowerThreshold2 = [0.95, 0.5, 0.5]; % Second lower bound for red
            upperThreshold2 = [1, 1, 1]; % Second upper bound for red
            mask1 = (hsvImage(:,:,1) >= lowerThreshold1(1)) & (hsvImage(:,:,1) <= upperThreshold1(1)) & ...
                     (hsvImage(:,:,2) >= lowerThreshold1(2)) & (hsvImage(:,:,2) <= upperThreshold1(2)) & ...
                     (hsvImage(:,:,3) >= lowerThreshold1(3)) & (hsvImage(:,:,3) <= upperThreshold1(3));
            mask2 = (hsvImage(:,:,1) >= lowerThreshold2(1)) & (hsvImage(:,:,1) <= upperThreshold2(1)) & ...
                     (hsvImage(:,:,2) >= lowerThreshold2(2)) & (hsvImage(:,:,2) <= upperThreshold2(2)) & ...
                     (hsvImage(:,:,3) >= lowerThreshold2(3)) & (hsvImage(:,:,3) <= upperThreshold2(3));
            mask = mask1 | mask2;
        
        case 'green'
            % Define thresholds for green color
            lowerThreshold = [0.25, 0.5, 0.5];
            upperThreshold = [0.45, 1, 1];
            mask = (hsvImage(:,:,1) >= lowerThreshold(1)) & (hsvImage(:,:,1) <= upperThreshold(1)) & ...
                   (hsvImage(:,:,2) >= lowerThreshold(2)) & (hsvImage(:,:,2) <= upperThreshold(2)) & ...
                   (hsvImage(:,:,3) >= lowerThreshold(3)) & (hsvImage(:,:,3) <= upperThreshold(3));
        
        case 'blue'
            % Define thresholds for blue color
            lowerThreshold = [0.55, 0.5, 0.5];
            upperThreshold = [0.75, 1, 1];
            mask = (hsvImage(:,:,1) >= lowerThreshold(1)) & (hsvImage(:,:,1) <= upperThreshold(1)) & ...
                   (hsvImage(:,:,2) >= lowerThreshold(2)) & (hsvImage(:,:,2) <= upperThreshold(2)) & ...
                   (hsvImage(:,:,3) >= lowerThreshold(3)) & (hsvImage(:,:,3) <= upperThreshold(3));
    end
    
    % Create detected image
    detectedImage = image;
    detectedImage(repmat(~mask, [1, 1, 3])) = 0; % Set non-detected areas to black
    
    % Calculate centroids
    stats = regionprops(mask, 'Centroid');
    centroids = cat(1, stats.Centroid);
    
    if ~isempty(centroids)
        result = 1; % At least one object detected
    end
end
