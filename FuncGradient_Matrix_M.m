function Map = FuncGradient_Matrix_M(y,x,Map)

[size_v,size_u] = size(Map.Grid);
% rounding (x,y) to zero direction
u = fix(x);  % [u,v] is the discrete grid cordinate of the map, in matrix u means column and v means row
v = fix(y);
      
% u_GT = fix(x_GT);
% v_GT = fix(y_GT);

b_0 =  y - v;
b_0(b_0==0)=1;
b_1 = 1 - b_0;
a_0 = x - u;
a_0(a_0==0)=1;
a_1 = 1 - a_0;
B = [-b_1; b_1; -b_0; b_0]';
A = [-a_1; -a_0; a_1; a_0]';

% b_0_GT =  y_GT - v_GT;
% b_1_GT = 1 - b_0_GT;
% a_0_GT = x_GT - u_GT;
% a_1_GT = 1 - a_0_GT;
% B_GT = [-b_1_GT; b_1_GT; -b_0_GT; b_0_GT]';
% A_GT = [-a_1_GT; -a_0_GT; a_1_GT; a_0_GT]';


m_00 = [u;v]';
m_00(m_00(:,1)>=size_u)=size_u-1;
m_00(m_00(:,2)>=size_v)=size_v-1;
    
m_10 = [m_00(:,1)+1,m_00(:,2)];
m_01 = [m_00(:,1),m_00(:,2)+1];
m_11 = m_00 +1;

% m_00_GT = [u_GT;v_GT]';
% m_00_GT(m_00_GT(:,1)>=size_u)=size_u-1;
% m_00_GT(m_00_GT(:,2)>=size_v)=size_v-1;
%     
% m_10_GT = [m_00_GT(:,1)+1,m_00_GT(:,2)];
% m_01_GT = [m_00_GT(:,1),m_00_GT(:,2)+1];
% m_11_GT = m_00_GT +1;
   

% id of matrix is (num_column-1) * size(matrix,1) + num_row
id_m00 = (m_00(:,1)-1) * size(Map.Grid,1) + m_00(:,2);
id_m10 = (m_10(:,1)-1) * size(Map.Grid,1) + m_10(:,2);
id_m01 = (m_01(:,1)-1) * size(Map.Grid,1) + m_01(:,2);
id_m11 = (m_11(:,1)-1) * size(Map.Grid,1) + m_11(:,2);


% id_m00_GT = (m_00_GT(:,1)-1) * size(Map.Grid,1) + m_00_GT(:,2);
% id_m10_GT = (m_10_GT(:,1)-1) * size(Map.Grid,1) + m_10_GT(:,2);
% id_m01_GT = (m_01_GT(:,1)-1) * size(Map.Grid,1) + m_01_GT(:,2);
% id_m11_GT = (m_11_GT(:,1)-1) * size(Map.Grid,1) + m_11_GT(:,2);


grid = [Map.Grid(id_m00), Map.Grid(id_m10), Map.Grid(id_m01), Map.Grid(id_m11)];

% grid_GT = [Map.Grid(id_m00_GT), Map.Grid(id_m10_GT), Map.Grid(id_m01_GT), Map.Grid(id_m11_GT)];

Gdugrid = sum(B .* grid,2);
Gdvgrid = sum(A .* grid,2);
Map.DgridGu = Gdugrid';
Map.DgridGv = Gdvgrid';

% Gdugrid_GT = sum(B_GT .* grid_GT,2);
% Gdvgrid_GT = sum(A_GT .* grid_GT,2);
% Map.DgridGu = Gdugrid_GT';
% Map.DgridGv = Gdvgrid_GT';

end

