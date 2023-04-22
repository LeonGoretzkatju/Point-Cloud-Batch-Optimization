function [p_1,p_2,p_3,p_4,index_select] = FuncCalBound(Map,Rate,KERNEL_SIZE)
%%
% This function is used to find the corrosponding four cells around the
% projected points on the boundries, these points can be used to build the
% selected map. Others points are the fixed points which be used as
% building fix map.
%%
Size_i = Map.Size_i;
Size_j = Map.Size_j;
ab = exp(Map.Grid);
CMap = ab;
CMap(CMap<1)=0;
CMap(CMap>1)=1;

r = (KERNEL_SIZE-1)/2;
KERNEL = ones(KERNEL_SIZE,KERNEL_SIZE)./(KERNEL_SIZE*KERNEL_SIZE);
ConvMap = zeros(Size_i,Size_j);
for i = 1:Size_i
    for j = 1:Size_j
        if i-r>0 && j-r>0 && i+r<=Size_i && j+r<=Size_j
            Block = CMap(i-r:i+r,j-r:j+r);
        elseif i-r<=0 && j-r<=0 && i+r<=Size_i && j+r<=Size_j
            Block = CMap(i:i+2*r,j:j+2*r);
        elseif i-r>0 && j-r<=0 && i+r>Size_i && j+r<=Size_j
            Block = CMap(i-2*r:i,j:j+2*r);
        elseif i-r>0 && j-r>0 && i+r>Size_i && j+r>Size_j
            Block = CMap(i-2*r:i,j-2*r:j);
        elseif i-r<=0 && j-r>0 && i+r<=Size_i && j+r<=Size_j
            Block = CMap(i:i+2*r,j-r:j+r);
        elseif i-r>0 && j-r>0 && i+r>Size_i && j+r<=Size_j
            Block = CMap(i-2*r:i,j-r:j+r);
        elseif i-r>0 && j-r<=0 && i+r<=Size_i && j+r<=Size_j
            Block = CMap(i-r:i+r,j:j+2*r);
        elseif i-r>0 && j-r>0 && i+r<=Size_i && j+r>Size_j
            Block = CMap(i-r:i+r,j-2*r:j);
        elseif i-r<=0 && j-r>0 && i+r<=Size_i && j+r>Size_j
            Block = CMap(i:i+2*r,j-2*r:j);
        else
            fprintf('Missing some elements, the index is%d,%d\n\n',i,j);
        end
        ConvMap(i,j) = sum(sum(KERNEL.*Block));
    end
end
    
Map_edge = (0<ConvMap).*(ConvMap<1);
% end

[index_x,index_y] = find(Map_edge==1);

index_low = [index_x,index_y];
index_low_right = [index_x,index_y+1];
index_low_below = [index_x+1,index_y];
index_low_belowR = [index_x+1,index_y+1];

index_low = [index_low;index_low_right;index_low_below;index_low_belowR];
index_low = unique(index_low,'rows');


% link to high resolution map
index_select = zeros(Rate*length(index_low),2);

for i=1:length(index_low)
    tem = index_low(i,:);
    tem_x = ((tem(1)-1)*Rate+1):(tem(1)*Rate);
    tem_y = ((tem(2)-1)*Rate+1):(tem(2)*Rate);
    vec_x = ones(1,Rate*length(tem_x));
    for j=1:length(tem_x)
        vec_x((j-1)*Rate+1:j*Rate) = tem_x(j);
    end
    vec_y = repmat(tem_y,[1,Rate]);
    index_select((i-1)*length(vec_x)+1:i*length(vec_y),1:2) = [vec_x',vec_y'];
end

id_select_x = index_select(:,1);
id_select_y = index_select(:,2);

id_select_11 = [id_select_x-2,id_select_y-2];
id_select_12 = [id_select_x-2,id_select_y-1];
id_select_13 = [id_select_x-2,id_select_y];
id_select_14 = [id_select_x-2,id_select_y+1];
id_select_15 = [id_select_x-2,id_select_y+2];

id_select_21 = [id_select_x-1,id_select_y-2];
id_select_22 = [id_select_x-1,id_select_y-1];
id_select_23 = [id_select_x-1,id_select_y];
id_select_24 = [id_select_x-1,id_select_y+1];
id_select_25 = [id_select_x-1,id_select_y+2];

id_select_31 = [id_select_x,id_select_y-2];
id_select_32 = [id_select_x,id_select_y-1];
id_select_34 = [id_select_x,id_select_y+1];
id_select_35 = [id_select_x,id_select_y+2];

id_select_41 = [id_select_x+1,id_select_y-2];
id_select_42 = [id_select_x+1,id_select_y-1];
id_select_43 = [id_select_x+1,id_select_y];
id_select_44 = [id_select_x+1,id_select_y+1];
id_select_45 = [id_select_x+1,id_select_y+2];

id_select_51 = [id_select_x+2,id_select_y-2];
id_select_52 = [id_select_x+2,id_select_y-1];
id_select_53 = [id_select_x+2,id_select_y];
id_select_54 = [id_select_x+2,id_select_y+1];
id_select_55 = [id_select_x+2,id_select_y+2];


index_select = [index_select;id_select_11;id_select_12;id_select_13;id_select_14;id_select_15;...
    id_select_21;id_select_22;id_select_23;id_select_24;id_select_25;...
    id_select_31;id_select_32;id_select_34;id_select_35;...
    id_select_41;id_select_42;id_select_43;id_select_44;id_select_45;...
    id_select_51;id_select_52;id_select_53;id_select_54;id_select_55];

index_select = unique(index_select,'rows');



min_x = index_x;
min_y = index_y;
max_x = index_x + 1;
max_y = index_y + 1;


% % write the format of [u,v] of the map
% % p_1 ... p_4 is the cordinates of points, not for matrix but should be
% -1
p_1 = [min_y,min_x];
p_2 = [min_y,max_x];
p_3 = [max_y,min_x];
p_4 = [max_y,max_x];

end