function [IS,IO] = FuncGetI_No_Odom(ErrorS)

% nO = size(Odom,1);
% % Sigma_O = [0.05,0.05,0.0003];
% Sigma_O = [0.05,0.05,0.02];
% 
% aa = repmat(1./Sigma_O.^2,nO,1);
% iO = reshape(aa',[],1);
% IO = sparse(1:3*nO,1:3*nO,iO);

Sigma_S = 1;%0.5;
nS = length(ErrorS);
IS = sparse(1:nS,1:nS,1./Sigma_S.^2);