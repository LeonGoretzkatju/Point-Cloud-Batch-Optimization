function Trans = Trans_Preprocess(Trans_original)
    Trans = {};
    sigma_R = 0.005;
    sigma_T = 0.0;
    for i = 1:length(Trans_original)
        Transition = Trans_original{i};
        Translation = Transition(1:3,4);
        Translation_meter = Translation / 1000.0;
        Transition(1:3,4) = Translation_meter;
%         [euler_angles,translation_part] = se3_to_euler_angles_translation(Transition);
%         euler_angles_new = euler_angles + sigma_R*randn(1,3);
%         Transition_New = euler_angles_translation_to_se3(euler_angles_new,translation_part);
        Trans{end+1} = Transition;
    end
end