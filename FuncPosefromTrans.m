function Pose = FuncPosefromTrans(Trans, num_nodes)
    Pose = {};
    for i = 1:num_nodes
        if i == 1
            Pose_i = eye(4,4);
            Pose{end+1} = Pose_i;
        else
            Pose_i = Trans{i-1}*Pose{i-1};
            Pose{end+1} = Pose_i;
        end
    end
end