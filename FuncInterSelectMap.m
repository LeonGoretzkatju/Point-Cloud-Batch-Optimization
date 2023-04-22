function [Md,out_index] = FuncInterSelectMap(y,x,Select_Map)

Grid = Select_Map.Grid;
size_v = Select_Map.Size_i;
size_u = Select_Map.Size_j;
yx = [fix(y)',fix(x)'];
out_member = ismember(yx,Select_Map.Select_Point,'rows');
out_index = find(out_member==0);
y(out_index) = [];
x(out_index) = [];
% rounding (x,y) to zero direction
% [u,v] is the discrete grid cordinate of the map, in matrix u means column and v means row
u = fix(x);  
v = fix(y);
     
b_0 =  y - v;
b_1 = 1 - b_0;
a_0 = x - u;
a_1 = 1 - a_0;

m_00 = [u;v]';
m_00(m_00(:,1)>=size_u)=size_u-1;
m_00(m_00(:,2)>=size_v)=size_v-1;
    
m_10 = [m_00(:,1)+1,m_00(:,2)];
m_01 = [m_00(:,1),m_00(:,2)+1];
m_11 = m_00 +1;

% id for matrix
id_m00 = (m_00(:,1)-1) * size_v + m_00(:,2);
id_m10 = (m_10(:,1)-1) * size_v + m_10(:,2);
id_m01 = (m_01(:,1)-1) * size_v + m_01(:,2);
id_m11 = (m_11(:,1)-1) * size_v + m_11(:,2);

grid = [Grid(id_m00), Grid(id_m10), Grid(id_m01), Grid(id_m11)];
Md = a_1'.*(b_1'.*grid(:,1) + b_0'.*grid(:,2)) + a_0'.*(b_1'.*grid(:,3) + b_0'.*grid(:,4));
Md = Md';

end