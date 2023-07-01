function IS = GetInformationMfromS(ErrorS)
    Sigma_S = 2.0;%0.5;
    nS = length(ErrorS);
    IS = sparse(1:nS,1:nS,1./Sigma_S.^2);
end