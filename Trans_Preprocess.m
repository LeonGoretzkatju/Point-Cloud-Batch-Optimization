function Trans = Trans_Preprocess(Trans_original)
    Trans = {};
    for i = 1:length(Trans_original)
        Transition = Trans_original{i};
        Translation = Transition(1:3,4);
        Translation_meter = Translation / 1000.0;
        Transition(1:3,4) = Translation_meter;
        Trans{end+1} = Transition;
    end
end