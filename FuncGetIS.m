function IS = FuncGetIS(ErrorS)


Sigma_S = 1;%0.5;
nS = length(ErrorS);
IS = sparse(1:nS,1:nS,1./Sigma_S.^2);

end