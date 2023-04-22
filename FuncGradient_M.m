function Map = FuncGradient_M(y,x,Map)

[size_v,size_u] = size(Map.Grid);
% rounding (x,y) to zero direction
u = fix(x);  % [u,v] is the discrete grid cordinate of the map, in matrix u means column and v means row
v = fix(y);

b_0 =  y - v;
b_1 = 1 - b_0;
a_0 = x - u;
a_1 = 1 - a_0;
B = [-b_1; b_1; -b_0; b_0]';
A = [-a_1; -a_0; a_1; a_0]';


size_x = size(x); 
size_j = size_x(2); % number of projected points
grid = [];



parfor i = 1:size_j   
    
%     u_1 = u(i)+1;
%     v_1 = v(i)+1;
    m_00 = [u(i), v(i)];
    m_00(m_00(:,1)>=size_u)=size_u-1;
    m_00(m_00(:,2)>=size_v)=size_v-1;
    
    m_10 = [m_00(:,1)+1,m_00(:,2)];
    m_01 = [m_00(:,1),m_00(:,2)+1];
    m_11 = m_00 +1;
   
    tem = [m_00;
        m_10;
        m_01;
        m_11];   % M(m00...11) round M(P_m)
    

    tem_grid = Map.Grid(tem(:,2), tem(:,1)); % Map.Grid(v,u)
    tem_grid = diag(tem_grid)';
    grid = [grid; tem_grid];    
end

Gdugrid = sum(B .* grid,2);
Gdvgrid = sum(A .* grid,2);
% Gdugrid_ = diag(B * grid');
% Gdvgrid_ = diag(A * grid');
Map.DgridGu = Gdugrid';
Map.DgridGv = Gdvgrid';

end

