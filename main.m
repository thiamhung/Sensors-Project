clc;
clf;
clear all
%% Initiate ROS
rosshutdown;
rosinit('192.168.27.1');

% Subscribe to relevant topic - RGB Image from RGB-D Camera
rgbSub = rossubscriber('camera/color/image_raw');
pause(1);
image_h = imshow(readImage(rgbSub.LatestMessage));
% Initiate Dobot class for controlign the robot
dobot = DobotMagician;
while 1
    % Reconfigure the joint into Initial Joint Position
    pause(1)
    q = [ 0 0 0 0];
    qw = [-0.4763    0.0503    0.3455         0];
    dobot.PublishTargetJoint(qw)
    pause(1)
    % Getting the RGB Image and pass it through color sorting - Red Color
    for i = 1:10
        image_h.CData = readImage(rgbSub.LatestMessage);
        [color_detected, centroidsRed,r] = detect_red(image_h.CData);
        set(image_h,'CData',color_detected);
    end
    if r == 1
        disp(['Object Found, Centroids of object: ', num2str(centroidsRed)])
    elseif r == 0
        error('Cannot find object');
    end
    % Subscribe to relevant topic - Depth Image from RGB-D Camera
    depthSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
    pause(1);
    % Read the depth image
    depthImage = readImage(depthSub.LatestMessage);
    depthImage = double(depthImage) / 1000.0; % Convert depth values from millimeters to meters
    % Retrive the Z coordinate from the depth value
    z_coordinates_red = calculateDepth(centroidsRed, depthImage);
    % intrinsic parameter of the camera
    
    % Parameter from camera ROS topic
    % fx = 606.8311157226562;
    % fy = 606.0000610351562;
    % cx = 333.31500244140625;
    % cy = 246.64346313476562;
    
    % Parameter from calibration
    fx = 602.68820;
    fy = 601.12176;
    cx = 334.29113;
    cy = 246.67656;
    K = [fx,  0, cx; ...
        0, fy, cy; ...
        0,  0,  1];
    invK = inv(K);
    % Retrived Centroid
    u_red = centroidsRed(:, 1);
    v_red = centroidsRed(:, 2);
    object_3D_coordinates = convertTo3DCoordinates(u_red, v_red, z_coordinates_red - 0.035, invK);
    % Camera distance to DoBot
    % cameraInDobot = [0.24, 0, 0.32];
    % Translating the camera into the DoBot frame or reference
    cameraToDobot = transl(0.32,0,0.24) * trotz(-pi/2) * trotx(pi);
    % Create a transformation matrix from the X and Y value from object_3D_coordinates
    objectTr = transl(object_3D_coordinates(1),object_3D_coordinates(2),object_3D_coordinates(3));
    % Transforming the object coordinate into the DoBot's frame of reference
    objectInRobot = cameraToDobot * objectTr;
    % Extracting the 3d-coordinate of the object
    objectPose = objectInRobot(1:3,4);
    % Assigning value to the X Y Z (There is some offset due to camera distorsion)
    X = objectPose(1) - 0.105;
    Y = objectPose(2) + 0.043;
    Z = objectPose(3) - 0.005;
    % Manualy check the coordinate to make sure the coordinate is inside DoBot's range
    disp([X , Y, Z])
    % input('Proceed?')
    % Safety measure to limit the DoBot's range
    if X < 0 || X > 0.32 || Y < -0.5 || Y > 0.5 || Z < -0.07
        error("Bad input")
    end
    
    % Waypoint
    dobot.PublishTargetJoint(q)
    pause(1)
    % Assign the Coordinates and send the message using ROS to the DoBot
    % Taking the Box
    end_effector_position = [X Y Z];
    end_effector_rotation = [0,0,0];
    dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
    pause(1)
    %% Turn on tool
    % Open tool
    onOff = 1;
    openClose = 1;
    dobot.PublishToolState(onOff,openClose);
    pause(3)
    % Go to Initial Joint as a Waypoint
    q1 = q;
    dobot.PublishTargetJoint(q1)
    pause(1)
    % Go to the target location to place the box
    qTarget = [0.6112    1.2365    0.2969         0];
    dobot.PublishTargetJoint(qTarget)
    pause(3)
    %% Turn off tool
    onOff = 0;
    openClose = 0;
    dobot.PublishToolState(onOff,openClose);
    pause(1)
    %% Configure the joint to initial joint position
    dobot.PublishTargetJoint(qw)
    pause(1)
    %% Sorting for Blue Color Object
    % asking input to proceed to next color - Blue
    % input('Next color?')
    
    disp('Finding Blue')
    % Getting the RGB Image and pass it through color sorting - Blue Color
    for i = 1:10
        image_h.CData = readImage(rgbSub.LatestMessage);
        [color_detected, centroidsBlue,b] = detect_blue(image_h.CData);
        set(image_h,'CData',color_detected);
        drawnow;
    end
    if b == 1
        disp(['Object Found, Centroids of object: ', num2str(centroidsRed)])
    elseif b == 0
        error('Cannot find object');
    end
    % Subscribe to relevant topic - Depth Image from RGB-D Camera
    depthSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
    pause(1);
    % Read the depth image
    depthImage = readImage(depthSub.LatestMessage);
    depthImage = double(depthImage) / 1000.0; % Convert depth values from millimeters to meters
    % Retrive the Z coordinate from the depth value
    z_coordinates_blue = calculateDepth(centroidsBlue, depthImage);
    % intrinsic parameter of the camera - callibrate each time we use
    u_blue = centroidsBlue(:, 1);
    v_blue = centroidsBlue(:, 2);
    object_3D_coordinates = convertTo3DCoordinates(u_blue, v_blue , z_coordinates_blue - 0.035, invK);
    % Camera distance to DoBot
    % cameraInDobot = [0.24, 0, 0.32];
    % Translating the camera into the DoBot frame or reference
    cameraToDobot = transl(0.32,0,0.24) * trotz(-pi/2) * trotx(pi);
    % Create a transformation matrix from the X and Y value from object_3D_coordinates
    objectTr = transl(object_3D_coordinates(1),object_3D_coordinates(2),object_3D_coordinates(3));
    % Transforming the object coordinate into the DoBot's frame of reference
    objectInRobot = cameraToDobot * objectTr;
    % Extracting the 3d-coordinate of the object
    objectPose = objectInRobot(1:3,4);
    % Assigning value to the X Y Z (There is some offset due to camera distorsion)
    X = objectPose(1) - 0.105;
    Y = objectPose(2) + 0.043;
    Z = objectPose(3) - 0.005;
    % Manualy check the coordinate to make sure the coordinate is inside DoBot's range
    disp([X , Y, Z])
    % input('Proceed?')
    % Safety measure to limit the DoBot's range
    if X < 0 || X > 0.32 || Y < -0.5 || Y > 0.5 || Z < -0.07
        error("Bad input")
    end
    % Waypoint
    dobot.PublishTargetJoint(q)
    pause(1)
    % Assign the Coordinates and send the message using ROS to the DoBot
    % Taking the Box
    end_effector_position = [X Y Z];
    end_effector_rotation = [0,0,0];
    dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
    pause(1)
    %% Turn on tool
    % Open tool
    onOff = 1;
    openClose = 1;
    dobot.PublishToolState(onOff,openClose);
    pause(3)
    % Go to Initial Joint as a Waypoint
    q1 = q;
    dobot.PublishTargetJoint(q1)
    pause(1)
    % Go to the target location to place the box
    qTarget = [0.7549    0.7956    0.7142         0];
    dobot.PublishTargetJoint(qTarget)
    pause(3)
    %% Turn off tool
    onOff = 0;
    openClose = 0;
    dobot.PublishToolState(onOff,openClose);
    pause(1)
    %% Return to Initial Joint Position
    dobot.PublishTargetJoint(qw)
    pause(1)
    %% Yellow
    % asking input to proceed to next color - Blue
    % input('Next color?')
    disp('Finding Yellow')
    % Getting the RGB Image and pass it through color sorting - Yellow Color
    for i = 1:10
        image_h.CData = readImage(rgbSub.LatestMessage);
        [color_detected, centroidsYellow,y] = detect_yellow(image_h.CData);
        set(image_h,'CData',color_detected);
    end
    if y == 1
        disp(['Object Found, Centroids of object: ', num2str(centroidsYellow)])
    elseif y == 0
        error('Cannot find object');
    end
    % Subscribe to relevant topic - Depth Image from RGB-D Camera
    depthSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
    pause(1);
    % Read the depth image
    depthImage = readImage(depthSub.LatestMessage);
    depthImage = double(depthImage) / 1000.0; % Convert depth values from millimeters to meters
    z_coordinates_yellow = calculateDepth(centroidsYellow, depthImage);
    % intrinsic parameter of the camera - callibrate each time we use
    u_yellow = centroidsYellow(:, 1);
    v_yellow = centroidsYellow(:, 2);
    object_3D_coordinates = convertTo3DCoordinates(u_yellow,v_yellow,z_coordinates_yellow - 0.035, invK);
    % Camera distance to DoBot
    % cameraInDobot = [0.24, 0, 0.32];
    % Translating the camera into the DoBot frame or reference
    cameraToDobot = transl(0.32,0,0.24) * trotz(-pi/2) * trotx(pi);
    % Create a transformation matrix from the X and Y value from object_3D_coordinates
    objectTr = transl(object_3D_coordinates(1),object_3D_coordinates(2),object_3D_coordinates(3));
    % Transforming the object coordinate into the DoBot's frame of reference
    objectInRobot = cameraToDobot * objectTr;
    % Extracting the 3d-coordinate of the object
    objectPose = objectInRobot(1:3,4);
    % Assigning value to the X Y Z (There is some offset due to camera distorsion)
    X = objectPose(1) - 0.105;
    Y = objectPose(2) + 0.043;
    Z = objectPose(3) - 0.005;
    % Manualy check the coordinate to make sure the coordinate is inside DoBot's range
    disp([X , Y, Z])
    % input('Proceed?')
    % Safety measure to limit the DoBot's range
    if X < 0 || X > 0.32 || Y < -0.5 || Y > 0.5 || Z < -0.07
        error("Bad input")
    end
    % Waypoint
    dobot.PublishTargetJoint(q)
    pause(1)
    % Assign the Coordinates and send the message using ROS to the DoBot
    % Taking the Box
    end_effector_position = [X Y Z];
    end_effector_rotation = [0,0,0];
    dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
    pause(1)
    %% Turn on tool
    % Open tool
    onOff = 1;
    openClose = 1;
    dobot.PublishToolState(onOff,openClose);
    pause(3)
    % Go to Initial Joint as a Waypoint
    q1 = q;
    dobot.PublishTargetJoint(q1)
    pause(1)
    % Go to the target location to place the box
    qTarget = [-0.5368    1.0600    0.4929         0];
    dobot.PublishTargetJoint(qTarget)
    pause(3)
    %% Turn off tool
    onOff = 0;
    openClose = 0;
    dobot.PublishToolState(onOff,openClose);
    pause(1)
    %% Return to Initial Joint Position
    q1 = q;
    dobot.PublishTargetJoint(q1)
    pause(1)
    
    input('Loop again?...')
end
 