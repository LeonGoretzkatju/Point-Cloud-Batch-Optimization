function [euler_angles, translation] = se3_to_euler_angles_translation(T)
    % Validate input
    if size(T,1) ~= 4 || size(T,2) ~= 4
        error('Input must be a 4x4 matrix.');
    end

    % Extract the rotation matrix R from the transformation matrix T
    R = T(1:3, 1:3);

    % Extract the translation vector t from the transformation matrix T
    translation = T(1:3, 4);

    % Calculate Euler angles (ZYX convention) from the rotation matrix R
    if R(3, 1) < 1
        if R(3, 1) > -1
            pitch = asin(-R(3, 1));
            yaw = atan2(R(2, 1), R(1, 1));
            roll = atan2(R(3, 2), R(3, 3));
        else
            pitch = pi / 2;
            yaw = -atan2(-R(2, 3), R(2, 2));
            roll = 0;
        end
    else
        pitch = -pi / 2;
        yaw = atan2(-R(2, 3), R(2, 2));
        roll = 0;
    end

    % Combine Euler angles into a single 1x3 vector
    euler_angles = [yaw, pitch, roll];
end