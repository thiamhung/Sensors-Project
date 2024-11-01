% clc;
clf;
clear all
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
% cam = webcam; % Connect to the webcam on the Raspberry Pi
% Initiate Dobot class for controlling the robot
dobot = DobotMagician;
% while 1
    % Reconfigure the joint into Initial Joint Position
    pause(1)
    q = [0 0 0 0];
    % qw = [-0.4763 0.0503 0.3455 0];
    dobot.PublishTargetJoint(q);
    pause(1);
    % Getting the RGB Image from the Webcam
    for i = 1:10
        img = snapshot(cam); % Capture a frame from the webcam
        [color_detected, centroidsRed, r] = detect_red(img);
        imshow(color_detected);
        drawnow;
    end
    if r == 1
        disp(['Object Found, Centroids of object: ', mat2str(centroidsRed)])
    elseif r == 0
        error('Cannot find object');
    end
    % Depth Estimation Placeholder (Static Depth Value)
    % Assuming a fixed depth value since there's no depth camera
    z_coordinates_red = 0.3; % Example static depth value in meters

    % Intrinsic parameters of the camera (use calibration values if available)
    fx = 196.3766;
    fy = 188.9726;
    cx = 153.1704;
    cy = 125.1502;
    K = [fx,  0, cx; ...
        0, fy, cy; ...
        0,  0,  1];
    invK = inv(K);
    % Retrieve Centroid
    u_red = centroidsRed(:, 1);
    v_red = centroidsRed(:, 2);
    % Convert to 3D Coordinates
    object_3D_coordinates = convertTo3DCoordinates(u_red, v_red, z_coordinates_red, invK);
    % Translating the camera into the DoBot frame of reference
    cameraToDobot = transl(0.32, 0, 0.24) * trotz(-pi/2) * trotx(pi);

    % Create a transformation matrix from the X and Y value from object_3D_coordinates
    objectTr = transl(object_3D_coordinates(1), object_3D_coordinates(2), object_3D_coordinates(3));

    % Transforming the object coordinate into the DoBot's frame of reference
    objectInRobot = cameraToDobot * objectTr;

    % Extracting the 3D-coordinate of the object
    objectPose = objectInRobot(1:3, 4);

    % Assigning value to the X, Y, Z (There is some offset due to camera distortion)
    X = objectPose(1) - 0.105;
    Y = objectPose(2) + 0.043;
    Z = objectPose(3) - 0.005;
    % Manually check the coordinate to make sure the coordinate is inside DoBot's range
    disp([X, Y, Z]);
    % input('Proceed?');
    % Safety measure to limit the DoBot's range
    if X < 0 || X > 0.32 || Y < -0.5 || Y > 0.5 || Z < -0.07
        error("Bad input");
    end
    % Waypoint
    dobot.PublishTargetJoint(q);
    pause(1);
    % Assign the Coordinates and send the message using to the DoBot
    % Taking the Box
    end_effector_position = [X, Y, Z];
    end_effector_rotation = [0, 0, 0];
    dobot.PublishEndEffectorPose(end_effector_position, end_effector_rotation);
    pause(1);
    %% Turn on tool
    % Open tool
    onOff = 1;
    openClose = 1;
    dobot.PublishToolState(onOff, openClose);
    pause(3);
    % Go to Initial Joint as a Waypoint
    q1 = q;
    dobot.PublishTargetJoint(q1);
    pause(1);
    % Go to the target location to place the box
    qTarget = [0.5368, 1.0600, 0.4929, 0];
    dobot.PublishTargetJoint(qTarget);
    pause(3);
    %% Turn off tool
    onOff = 0;
    openClose = 0;
    dobot.PublishToolState(onOff, openClose);
    pause(1);
    %% Return to Initial Joint Position
    q1 = q;
    dobot.PublishTargetJoint(q1);
    pause(1);
    % input('Loop again?...');
% end
