function object_3D_coordinates = convertTo3DCoordinates(u, v, Z, invK)
    % Create a column vector for the image point
    uv_point = [u; v; 1];  % Homogenous coordinates

    % Back-project to camera reference system
    cam_point = invK * uv_point;
    cam_point = cam_point / cam_point(3); % normalize

    % Use the depth value to scale the point in the camera's coordinate space
    X = cam_point(1) * Z;
    Y = cam_point(2) * Z;
    Z_world = Z; % Keeping the variable names consistent for clarity

    object_3D_coordinates = [X , Y, Z_world];
end