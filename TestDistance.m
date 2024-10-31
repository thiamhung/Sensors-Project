% Create a webcam object
cam = webcam("Intel(R) RealSense(TM) Depth Camera 435 with RGB Module RGB");

% Capture a single frame
preview(cam)
frame = snapshot(cam);

% Display the captured image
imshow(frame);
title('Captured Image');

% Define the real-world width of the object (in mm)
realWorldWidth = 85.6; % For example, a credit card

% Convert the image to grayscale
grayImage = rgb2gray(frame);

i

% Use a simple threshold to create a binary image
binaryImage = imbinarize(grayImage);

% Find the object in the binary image
stats = regionprops(binaryImage, 'BoundingBox', 'Area');

% Assuming the largest object is the one of interest
[~, largestIdx] = max([stats.Area]);
boundingBox = stats(largestIdx).BoundingBox;

% Calculate the width of the object in pixels
objectWidthPixels = boundingBox(3); % Bounding box width

% Camera parameters (you may need to adjust these)
focalLength = 700; % Focal length in pixels (this is an example, use your camera's specifications)
imageWidth = size(frame, 2); % Width of the image in pixels

% Calculate the distance to the object (in mm)
distance = (realWorldWidth * focalLength) / objectWidthPixels; % Distance in mm

% Display results
fprintf('Detected Object Width: %.2f pixels\n', objectWidthPixels);
fprintf('Estimated Distance to Object: %.2f mm\n', distance);

% Clean up
clear cam;
