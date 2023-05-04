function T = euler_angles_translation_to_se3(euler_angles, translation)
    % Validate input
    if numel(euler_angles) ~= 3 || numel(translation) ~= 3
        error('Input must be 1x3 Euler angles and 1x3 translation vectors.');
    end

    % Extract yaw, pitch, and roll angles
    yaw = euler_angles(1);
    pitch = euler_angles(2);
    roll = euler_angles(3);

    % Calculate rotation matrix R from Euler angles (ZYX convention)
    cy = cos(yaw);
    sy = sin(yaw);
    cp = cos(pitch);
    sp = sin(pitch);
    cr = cos(roll);
    sr = sin(roll);

    R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
         -sp,    cp*sr,            cp*cr];

    % Combine rotation matrix R and translation vector t into a 4x4 SE(3) transformation matrix T
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = translation;
end
