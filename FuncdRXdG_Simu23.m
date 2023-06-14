function dRxdG = FuncdRXdG_Simu23(Gamma)
    dRxdG = [1 0 0;
        0 -sin(Gamma) cos(Gamma);
        0 -cos(Gamma) -sin(Gamma)];
end