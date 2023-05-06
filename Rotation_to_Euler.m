function Euler_angle = Rotation_to_Euler(R)
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
    Euler_angle = [yaw; pitch; roll];
end