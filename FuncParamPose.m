function Pose_Vector = FuncParamPose(Pose)
    Pose_Vector = zeros(length(Pose),6);
    for i = 1:length(Pose)
        [euler_anglesi,translationsi] = se3_to_euler_angles_translation(Pose{i});
        Pose_Vector(i,1:3) = euler_anglesi;
        Pose_Vector(i,4:6) = translationsi';
    end
end