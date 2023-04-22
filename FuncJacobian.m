
function [JP,JD]= FuncJacobian(Map,Pose,D,Dgrid,K)

%%
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Grid = Map.Grid;
Scale = Map.Scale;
Origin = Map.Origin;

nD = length(D);
[nv,nu] = size(D{1});

nMap = Size_i*Size_j;
nPose = nD*6;
nObs = nD*Size_i*Size_j;

JID1 = [];
JID2 = [];
JID3 = [];
JID4 = [];
ValP = [];
ValD = [];

nCnt1 = 0;
nCnt2 = 0;
%%
for i = 1:Size_i
    for j = 1:Size_j
        Flg = 0;
        P = [([j;i]-1)*Scale+Origin;Grid(i,j)];
        for k = 1:nD
            nCnt1 = nCnt1+1;
%             if Grid(i,j)~=0                
                RZ = FuncRZ(Pose(k,1));
                RY = FuncRY(Pose(k,2));
                RX = FuncRX(Pose(k,3));
                Rk = RX*RY*RZ;
%                 Rk = RMatrixYPR22(Pose(k,1),Pose(k,2),Pose(k,3));
                Tk = Pose(k,4:6)';
                Xj = P-Tk;
                Pk = K*Rk*Xj;
                u = Pk(1)/Pk(3);
                v = Pk(2)/Pk(3);
                d = Pk(3);
                if u>=1 && u<=nu && v>=1 && v<=nv
                    dRZdA = FuncdRZdA(Pose(k,1));
                    dRYdB = FuncdRYdB(Pose(k,2));
                    dRXdG = FuncdRXdG(Pose(k,3));
                    
                    dxdA = K*RX*RY*dRZdA*Xj;    
                    dxdB = K*RX*dRYdB*RZ*Xj;
                    dxdG = K*dRXdG*RY*RZ*Xj;
%                     dxdR = [dxdA,dxdB,dxdG];
                    
                    dxdP = K*Rk;%%
                    dxdT = -dxdP;

                    dxdRT = [dxdA,dxdB,dxdG,dxdT];

                    dudx = Funcdudx(Pk);
%                     u2 = u-K(1,3);
%                     v2 = v-K(2,3);
                    dDdu = [Dgrid{k}.Gu(v,u),Dgrid{k}.Gv(v,u)];
%                     if abs(Dgrid{k}.Gu(v,u))>=0.01 || abs(Dgrid{k}.Gv(v,u))>=0.01
%                         dDdu = [0,0];
%                     end
                    
                    dDkdRT = dDdu*dudx*dxdRT;
                    dDkdP = dDdu*dudx*dxdP;
                    dDkdM = dDkdP(3);
                    
                    dDdRT = dxdRT(3,:);
                    dDdM = dxdP(3,3);
                    
                    dEdRT = dDkdRT-dDdRT;
                    dEdM = dDkdM-dDdM;
%                     dEdM = -dDdM;
%                     dEdRT = -dDdRT;

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Test
%                     if abs(dEdM)<0.05
%                         dEdM = 0;
% %                         dEdRT = zeros(1,6);
% %                         dEdM = -dDdM;
% %                         dEdRT = -dDdRT;
%                     end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    JID1 = [JID1,nCnt1,nCnt1,nCnt1,nCnt1,nCnt1,nCnt1];
                    JID2 = [JID2,6*k-5:6*k];
                    JID3 = [JID3,nCnt1];
                    JID4 = [JID4,Size_j*(i-1)+j];
                    ValP = [ValP,dEdRT];
                    ValD = [ValD,dEdM];
                    
%                     dk = Dgrid{k}.G(v,u);
%                     E(nCnt1) = dk-d;
                    nCnt2 = nCnt2+1;
                    Flg = Flg+1;
                end                
%             end
        end
        
        if Flg==0 && Grid(i,j)~=0
%             Grid(i,j) = 0;
        end
    end
end

%%
JP = sparse(JID1,JID2,ValP,nObs,nPose);
JD = sparse(JID3,JID4,ValD,nObs,nMap);

end

