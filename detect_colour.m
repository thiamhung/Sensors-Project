clc;
clf;
clear all;

%% Initialize Webcam
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
dobot = DobotMagician;

while 1
    % Reconfigure the joint into Initial Joint Position
    pause(1)
    q = [0 0 0 0];
    qw = [-0.4763 0.0503 0.3455 0];
    dobot.PublishTargetJoint(qw);
    pause(1);
    
    % Getting the RGB Image from the Webcam
    for i = 1:10
        img = snapshot(cam); % Capture a frame from the webcam
        
        % Detect red, blue, and green objects
        [color_detected_red, centroidsRed, r] = detect_red(img);
        [color_detected_blue, centroidsBlue, b] = detect_blue(img);
        [color_detected_green, centroidsGreen, g] = detect_green(img);

        % Combine all detections into a single image
        combined_img = max(max(color_detected_red, color_detected_blue), color_detected_green);
        
        % Display the combined image
        imshow(combined_img);
        drawnow;
    end

    % Output centroid information if objects are found
    if r == 1
        disp(['RED Object Found, Centroids of object: ', mat2str(centroidsRed)]);
    else
        disp('No red object detected');
    end

    if b == 1
        disp(['BLUE Object Found, Centroids of object: ', mat2str(centroidsBlue)]);
    else
        disp('No blue object detected');
    end

    if g == 1
        disp(['GREEN Object Found, Centroids of object: ', mat2str(centroidsGreen)]);
    else
        disp('No green object detected');
    end

    % Additional robot actions or depth estimation can be placed here
    % as per the original commented-out code.

    % Loop control (optional prompt to repeat loop, if needed)
    % input('Loop again?...');
end
