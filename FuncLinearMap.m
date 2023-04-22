function Map = FuncLinearMap(Map,Pose,D,K)

%%
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Scale = Map.Scale;
Origin = Map.Origin;

nD = length(D);
[nv,nu] = size(D{1});

%%
fx = K(1,1);
fy = K(2,2);
u0 = K(1,3);
v0 = K(2,3);

u = repmat([1:nu],nv,1);
v = repmat([1:nv]',1,nu);
u = ((u-u0)/fx);
v = ((v-v0)/fy);
u = reshape(u,[],1);
v = reshape(v,[],1);
XY1 = [u,v,ones(nu*nv,1)]';

%%
Error = [];
JDID1 = [];
JDID2 = [];
JDVal = [];

%%
for k = 1:nD
    Z = reshape(D{k},[],1)';
    XYZ = XY1.*Z;
    
    IDk = nv*nu*(k-1)+1:nv*nu*k;
    
    RZ = FuncRZ(Pose(k,1));
    RY = FuncRY(Pose(k,2));
    RX = FuncRX(Pose(k,3));
    Rk = RX*RY*RZ;
    Tk = Pose(k,4:6)';
    XYZ2 = Rk'*XYZ+Tk;
    XY3 = (XYZ2(1:2,:)-Origin)/Scale+1;
    
    Ek = XYZ2(3,:);
    Error = [Error;Ek'];
    
    %%    
    u = XY3(1,:);
    v = XY3(2,:);
    u1 = fix(u);
    v1 = fix(v);
    
    dEdM = [(v1+1-v).*(u1+1-u);(v-v1).*(u1+1-u);(v1+1-v).*(u-u1);(v-v1).*(u-u1)];    
    dEdMID2 = [Size_j*(v1-1)+u1;Size_j*v1+u1;Size_j*(v1-1)+u1+1;Size_j*v1+u1+1];
    dEdMID1 = repmat(IDk,4,1);
    
    JDID1 = [JDID1;reshape(dEdMID1',[],1)];
    JDID2 = [JDID2;reshape(dEdMID2',[],1)];
    JDVal = [JDVal;reshape(dEdM',[],1)];
    
end

%%
JD = sparse(JDID1,JDID2,JDVal,nv*nu*nD,Size_i*Size_j);
HH = FuncMapConst(Map);
HH = HH*10;

I = JD'*JD+HH;
bI = JD'*Error;

Grid = I\bI;
Map.Grid = reshape(Grid,Size_j,Size_i)';

end
