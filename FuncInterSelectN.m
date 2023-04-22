function MN = FuncInterSelectN(y,x,Select_Map,out_index)
N = Select_Map.N;

size_v = Select_Map.Size_i;
size_u = Select_Map.Size_j;
% rounding (x,y) to zero direction
% [u,v] is the discrete grid cordinate of the map, in matrix u means column and v means row

y(out_index) = [];
x(out_index) = [];
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

id_m00 = (m_00(:,1)-1) * size(N,1) + m_00(:,2);
id_m10 = (m_10(:,1)-1) * size(N,1) + m_10(:,2);
id_m01 = (m_01(:,1)-1) * size(N,1) + m_01(:,2);
id_m11 = (m_11(:,1)-1) * size(N,1) + m_11(:,2);

NN = [N(id_m00), N(id_m10), N(id_m01), N(id_m11)];
MN = a_1'.*(b_1'.*NN(:,1) + b_0'.*NN(:,2)) + a_0'.*(b_1'.*NN(:,3) + b_0'.*NN(:,4));
MN = MN';
end