%% Coding by Yingyu Wang
% Get the initial poses from odometry input
% Odom is the incremental form

function [Pose] = FuncGetInitialPose(Odom)

nO = size(Odom,1);
Pose = zeros(nO+1,3);

Phi1 = 0;
R1 = eye(2);
T1 = [0;0];

for i=1:nO
    dT = Odom(i,1:2)';
    T2 = R1'*dT+T1;
    Phi2 = Odom(i,3)+Phi1;
    
    while Phi2>pi || Phi2<-pi
            Phi2 = wrap(Phi2);
    end
    
    Pose(i+1,:) = [T2;Phi2]';
    Phi1 = Phi2;
    T1 = T2;
    R1 = FuncRMatrix2D(Phi1);
end

