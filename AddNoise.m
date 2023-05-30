function Pose_Noise_New = AddNoise(Pose)
    Pose_Noise = {};
    Pose_Length = length(Pose);
    sigma_R = 0.002;
    sigma_T = 0.0;
    Pose_Noise{end+1} = Pose{1};
    for i = 1:(Pose_Length-1)
        Pose2 = Pose{i+1};
        [euler_angle, translation] = se3_to_euler_angles_translation(Pose2);
        euler_angle_noise = euler_angle + sigma_R*rand(1,3);
        translation_noise = translation + sigma_T*rand(3,1);
        Pose_Noise_k = euler_angles_translation_to_se3(euler_angle_noise,translation_noise);
        Pose_Noise{end+1} = Pose_Noise_k;
    end
    Pose_Noise_New = Pose_Noise;
end