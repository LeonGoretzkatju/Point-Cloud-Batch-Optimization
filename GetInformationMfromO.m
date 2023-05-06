function IO = GetInformationMfromO(Odom)
    nO = length(Odom);
    Sigma_O = [0.05,0.05,0.05,0.02,0.02,0.02];
    
    aa = repmat(1./Sigma_O.^2,nO,1);
    iO = reshape(aa',[],1);
    IO = sparse(1:6*nO,1:6*nO,iO);
end