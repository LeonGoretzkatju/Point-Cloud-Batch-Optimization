function Select_Map = FuncGradientSelectMap(y,x,Select_Map,out_index)
% [u,v] [x,y] is the direction of cordinate in the map, is not means the direction of matrix
% u/x means the column of the matrix and v/y means the row in of the marix
Size_i = Select_Map.Size_i;
Size_j = Select_Map.Size_j;
% rounding (x,y) to zero direction
% [u,v] is the discrete grid cordinate of the map, in matrix u means column, and v means row
y(out_index) = [];
x(out_index) = [];
u = fix(x);  
v = fix(y);
b_0 =  y - v;
b_1 = 1 - b_0;
a_0 = x - u;
a_1 = 1 - a_0;

C = [a_1.*b_1;a_0.*b_1;a_1.*b_0;a_0.*b_0];
% calculate gradient
Select_Map = FuncGradientMap(Select_Map);

m_00 = [u;v]';
m_00(m_00(:,1)>=Size_j)=Size_j-1;
m_00(m_00(:,2)>=Size_i)=Size_i-1;
    
m_10 = [m_00(:,1)+1,m_00(:,2)];
m_01 = [m_00(:,1),m_00(:,2)+1];
m_11 = m_00 +1;


% id of matrix is (num_column-1) * size(matrix,1) + num_row
id_m00 = (m_00(:,1)-1) * Size_i + m_00(:,2);
id_m10 = (m_10(:,1)-1) * Size_i + m_10(:,2);
id_m01 = (m_01(:,1)-1) * Size_i + m_01(:,2);
id_m11 = (m_11(:,1)-1) * Size_i + m_11(:,2);
Gdu = [Select_Map.DgridGu(id_m00),Select_Map.DgridGu(id_m10),Select_Map.DgridGu(id_m01),Select_Map.DgridGu(id_m11)];
Gdv = [Select_Map.DgridGv(id_m00),Select_Map.DgridGv(id_m10),Select_Map.DgridGv(id_m01),Select_Map.DgridGv(id_m11)];

% deal with the nan value
Gdu(isnan(Gdu)) = 0;
Gdv(isnan(Gdv)) = 0;


Gdugrid = sum(C'.*Gdu,2);
Gdvgrid = sum(C'.*Gdv,2);

Select_Map.DgridGu = Gdugrid';
Select_Map.DgridGv = Gdvgrid';
end

