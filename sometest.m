IDk = 4:6;
dEdPID1 = repmat(IDk,100,1)';
a = [1,2,3;2,3,4];
b = [1,3,2;3,2,4];
c = sum(a.*b);
JPID1 = 1:100;
JPID2 = 101:200;
JPVal = 1:100;
JP = sparse(JPID1,JPID2,JPVal);
c = 0.3;
fix(c)