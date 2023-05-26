function Pose = AddNoise(Pose)
    Pose_Length = length(Pose);
    sigma_R = 0.001;
    sigma_T = 0.01;
    for i = 1:(Pose_Length-1)
        Pose2 = Pose{i+1};
        [euler_angle, translation] = se3_to_euler_angles_translation(Pose2);
        euler_angle_noise = euler_angle + sigma_R*randn(1,3);
        translation_noise = translation + sigma_T*randn(3,1);
        Pose_Noise = euler_angles_translation_to_se3(euler_angle_noise,translation_noise);
        Pose{i+1} = Pose_Noise;
    end
end