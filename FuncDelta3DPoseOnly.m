function [DeltaP, Sum_Delta] = FuncDelta3DPoseOnly(JP,ErrorS,IS)
JP = JP(:,7:end);
U = JP'*IS*JP;
ErrorS = sparse(ErrorS);
EP = -JP'*IS*ErrorS;
II = U;
EE = EP;
DeltaP = II\EE;
Sum_Delta = DeltaP'*DeltaP;
end