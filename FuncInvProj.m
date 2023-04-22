function local_index = FuncInvProj(Map,Pose,p_1,p_2,p_3,p_4)
Pose = Pose';
Scale = Map.Scale;
Origin = Map.Origin;

Si_1 = (p_1'-1) * Scale + Origin;
Si_2 = (p_2'-1) * Scale + Origin;
Si_3 = (p_3'-1) * Scale + Origin;
Si_4 = (p_4'-1) * Scale + Origin;

nn = length(Pose);

parfor i=1:nn
    posei = Pose(:,i);
    Ri = FuncRMatrix2D(posei(3));

    xy_1 = Ri * (Si_1 - posei(1:2));
    xy_2 = Ri * (Si_2 - posei(1:2));
    xy_3 = Ri * (Si_3 - posei(1:2));
    xy_4 = Ri * (Si_4 - posei(1:2));
    local_index{i} =  [xy_1;xy_2;xy_3;xy_4];
end
       
end