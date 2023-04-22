function RotationMatrix = RMatrixRPY(Roll,Pitch,Yaw)
%RMatrixRPY - Description
%
% Syntax: RotationMatrix = RMatrixRPY(Roll,Pitch,Yaw)
%
% Long description
% Calculates the rotation matrix from roll, pitch and yaw angles
% Roll, Pitch and Yaw are in radians
% RotationMatrix is a 3x3 matrix
% The rotation matrix is calculated as follows:
Rz = [cos(Yaw) -sin(Yaw) 0; sin(Yaw) cos(Yaw) 0; 0 0 1];
Ry = [cos(Pitch) 0 sin(Pitch); 0 1 0; -sin(Pitch) 0 cos(Pitch)];
Rx = [1 0 0; 0 cos(Roll) -sin(Roll); 0 sin(Roll) cos(Roll)];

RotationMatrix = Rz*Ry*Rx;
    
end