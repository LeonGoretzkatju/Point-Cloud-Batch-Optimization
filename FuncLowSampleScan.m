function Low_Scan = FuncLowSampleScan(Scan,Rate)


if Rate ==1
    Low_Scan = Scan;
else
    nn = length(Scan);
    Low_Scan = cell(1,nn);
    for i=1:nn

        x = Scan{i}.xy(:,1);
        y = Scan{i}.xy(:,2);
        Odd = Scan{i}.Odd;

        index = find(Odd>0);

        mm = length(index);
        cell_label = cell(1,(mm-1));
%         label = [];
        for j=1:(mm-1)
            d = index(j+1) - index(j);
            low_d = d./Rate;
            if low_d <=1
                low_d = 1;
            end


            if low_d ==1
                value = [index(j),(index(j+1)-1)];

            else
                value = index(j):Rate:(index(j+1)-1);

            end
            cell_label{j} = value;
%             label = [label,value];

        end
        label = horzcat(cell_label{:});
        Low_Scan{i}.xy = [x(label),y(label)];
        Low_Scan{i}.Odd = Odd(label);

    end

end
end