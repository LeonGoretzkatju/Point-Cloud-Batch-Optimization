function [dRyaw,dRpitch,dRroll] = DiffRotationMatrix(euler_angles)
    yaw = euler_angles(1);
    pitch = euler_angles(2);
    roll = euler_angles(3);
    cz = cos(yaw);
    sz = sin(yaw);
    dRyaw = [-sz, -cz, 0;
              cz, -sz, 0;
              0,  0,   1];
    cy = cos(pitch);
    sy = sin(pitch);
    dRpitch = [-sy, 0, cy;
                0,  1, 0;
               -cy, 0, -sy];
    cx = cos(roll);
    sx = sin(roll);
    dRroll = [1, 0, 0;
              0,-sx,-cx;
              0, cx,-sx];

end