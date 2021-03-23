%% for test

sigma = 1;
S1 = [0 sigma; -sigma 0];
D1 = [1 0;
    1 0];
ob1 = obsv(S1,D1); % observable

%% ===========
S2 = blkdiag(S1,S1);
D2 = [1 0 0 0;
    0 0 1 0];
ob2 = obsv(S2,D2); % observable

%% ===========
S3 = blkdiag(S1,S1,S1);
a = 1, b = 2;
D3 = [1 0 0 0, 0 0;
    0 0 1 0, a b];
ob3 = obsv(S3,D3); % not observable

%% ===========
S4 = [0 sigma; -sigma 0];
a = 0; b = 1;
D4 = D1 + [a b]'*[0 1];
ob4 = obsv(S4,D4)