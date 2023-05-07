function Pose = FuncInverParamPose(Pose_Vector_new)
    Pose = {};
    for i = 1:size(Pose_Vector_new,1)
        euler_angles_i = Pose_Vector_new(i,1:3);
        translations_i = Pose_Vector_new(i,4:6)';
        Posei = euler_angles_translation_to_se3(euler_angles_i,translations_i);
        Pose{end+1} = Posei;
    end
end