function T = euler_angles_translation_to_se3(euler_angles, translation)
    % Validate input
    if numel(euler_angles) ~= 3 || numel(translation) ~= 3
        error('Input must be 1x3 Euler angles and 1x3 translation vectors.');
    end

    % Extract yaw, pitch, and roll angles
    yaw = euler_angles(1);
    pitch = euler_angles(2);
    roll = euler_angles(3);

    % Rotation matrix for yaw (Z-axis rotation)
    cz = cos(yaw);
    sz = sin(yaw);
    Rz = [cz, -sz, 0;
          sz,  cz, 0;
          0,   0,  1];

    % Rotation matrix for pitch (Y-axis rotation)
    cy = cos(pitch);
    sy = sin(pitch);
    Ry = [cy,  0,  sy;
          0,   1,  0;
         -sy,  0,  cy];

    % Rotation matrix for roll (X-axis rotation)
    cx = cos(roll);
    sx = sin(roll);
    Rx = [1,  0,   0;
          0,  cx, -sx;
          0,  sx,  cx];

    % Combine the separate rotation matrices to obtain the final rotation matrix
    R = Rz * Ry * Rx;

    % Combine rotation matrix R and translation vector t into a 4x4 SE(3) transformation matrix T
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = translation;
end
