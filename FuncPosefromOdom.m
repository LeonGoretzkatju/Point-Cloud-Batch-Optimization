function [Pose,Odom] = FuncPosefromOdom(Odom,Pose_GT,DATA_MODE)

%calculate poses from odom
if strcmp(DATA_MODE,'REAL')==1
    Pose_0 = Pose_GT(1,:);
else
    Pose_0 = Odom(1,:);
    Odom = Odom(2:end,:);
end

Pose = zeros(size(Odom,1)+1,3);

for i=0:size(Odom,1)
    if i==0
        Pose(1,1:2) = Pose_0(1,1:2);
        Pose(1,3) = Pose_0(1,3);
    else
        Pose (i+1,1:2) = (theta2R(Pose(i,3)) * Odom(i,1:2)')' + Pose(i,1:2);
        Pose(i+1,3) = wrap(Pose(i,3) + Odom(i,3));
    end
end

end