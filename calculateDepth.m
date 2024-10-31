function z_coordinate = calculateDepth(centroid, depthImage)
    % Validate input parameters
    if nargin < 2
        error('Missing required parameters: centroid and depthImage are required.');
    end

    % Check if the input is a non-empty matrix
    if isempty(centroid) || isempty(depthImage)
        error('Input matrices (centroid or depthImage) should not be empty.');
    end

    % Check if more than one centroid is provided
    if size(centroid, 1) > 1
        error('Multiple centroids detected.');
    end

    % Round the coordinates as pixel indices must be integers. Also, ensure they're within the valid range.
    x = min(max(round(centroid(1)), 1), size(depthImage, 2)); % X-coordinate
    y = min(max(round(centroid(2)), 1), size(depthImage, 1)); % Y-coordinate

    % Fetch the corresponding depth value
    z_coordinate = depthImage(y, x); % depthImage rows correspond to Y, columns to X

    % Handle any potential issue with depth data (e.g., missing or invalid depth)
    if isnan(z_coordinate) || z_coordinate == 0
        error('Invalid depth detected. The centroid does not correspond to a valid depth value.');
    end
end
