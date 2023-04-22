function Select_Scan = FuncSelectScan(Scan,local_index)

nn = length(Scan);
mm = size(local_index{1},2);
Select_Scan = cell(1,nn);

for i=1:nn
    xy = Scan{i}.xy;
    x = xy(:,1);
    y = xy(:,2);
    odd = Scan{i}.Odd;
    local_xy = local_index{i};

    for j=1:mm
        tem_xy = local_xy(:,j);
        local_x(1) = tem_xy(1);
        local_x(2) = tem_xy(3);
        local_x(3) = tem_xy(7);
        local_x(4) = tem_xy(5);
        local_x(5) = tem_xy(1);

        local_y(1) = tem_xy(2);
        local_y(2) = tem_xy(4);
        local_y(3) = tem_xy(8);
        local_y(4) = tem_xy(6);
        local_y(5) = tem_xy(2);

        tem = inpolygon(x,y,local_x',local_y');
        if j==1
            tem_pre = tem;
        else
            tem_pre = tem_pre+tem;
            tem_pre(tem_pre~=0)=1;
        end
    end
    
    sel_x = x(tem_pre==1);
    sel_y = y(tem_pre==1);
    sel_odd = odd(tem_pre==1); 
    
    Select_Scan{i}.xy = [sel_x,sel_y];
    Select_Scan{i}.Odd = sel_odd;
end
end