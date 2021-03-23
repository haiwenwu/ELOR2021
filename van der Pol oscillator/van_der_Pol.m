function dx = van_der_Pol(t,x)
% This example shows how to implement the self-learning mechanism for the
% output regulation of van der Pol oscillator
% Copyright 2020 Haiwen Wu

%% ==================== System parameter
mu1 = 1; mu2 = 2;

%% ==================== States
dx = zeros(25,1);
v = x(1:10);
eta1_a = x(11:12); eta1_b = x(13); eta1_c = x(14); 
eta2_a = x(15:18); eta2_b = x(19:20); eta2_c = x(21); 
q = x(22);
dq = x(23);
est = x(24:25);


%% ==================== exosystem ====================
s1 = 1; s2 = 3;
s3 = 1.*(t>=0 & t<50) + (3.5-0.05.*t).*(t>=50 & t<60) + 0.5.*(t>=60 & t<90);
s4 = 1; s5 = 1;
S1 = [0 s1; -s1 0];
S2 = [0 s2; -s2 0];
S3 = [0 s3; -s3 0];
S4 = [0 s4; -s4 0];
S5 = [0 s5; -s5 0];
S = blkdiag(S1,S2,S3,S4,S5);


%% ==================== reference ====================
qr=(x(1)+x(3)).*(t>=0 & t<25) + x(5).*(t>=25 & t<90) + 0.*(t>=90 & t<=110);


%% ==================== Controller parameters ===================
%% IM_1
m = [1; 1.4142]; eta1B = [eta1_b; m(2)];
M = [0 1; -m(1) -m(2)]; N = [0; 1];
varrho1 = m-eta1B; 
Phia1 = [0 1; -varrho1(1) -varrho1(2)];
Gamma1 = (m-varrho1)'*(Phia1+eye(2));

%% IM_2
m2 = [1; 2.7; 3.4; 2.1]; eta2B = [eta2_b(1); m2(2); eta2_b(2); m2(4)];
M2 = [0 1 0 0; 0 0 1 0; 0 0 0 1; -m2']; N2 = [0; 0; 0; 1];
varrho2 = m2-eta2B; 
Phia2 = [0 1 0 0; 0 0 1 0; 0 0 0 1; -varrho2'];
Gamma2 = (m2-varrho2)'*(Phia2+eye(4));

%% Certainty equivalence property
pos1 = Gamma1*inv(Phia1)*eta1_a;
vel1 = Gamma1*eta1_a; 
acc1 = Gamma1*Phia1*eta1_a; 

pos2 = Gamma2*inv(Phia2)*eta2_a;
vel2 = Gamma2*eta2_a; 
acc2 = Gamma2*Phia2*eta2_a; 

pos = pos2.*(t>=0 & t<25) + pos1.*(t>=25 & t<90) + 0.*(t>=90 & t<=110);
vel = vel2.*(t>=0 & t<25) + vel1.*(t>=25 & t<90) + 0.*(t>=90 & t<=110);
acc = acc2.*(t>=0 & t<25) + acc1.*(t>=25 & t<90) + 0.*(t>=90 & t<=110);

rho1 = pos + vel^3 + acc;
rho2 = [-vel pos*vel^2];

%% Error
e = q - qr;
delta1 = 5*e + dq - Gamma1*eta1_a;
delta2 = 5*e + dq - Gamma2*eta2_a; 
delta5 = 5*e + dq; 

delta = delta2.*(t>=0 & t<25) + delta1.*(t>=25 & t<90) + delta5.*(t>=90 & t<=110);

%% Input
u = rho1 + rho2*est - 5*delta; 


%% ==================== ODE functions ====================
dx(1:10) = S*v;                    %% exosystem

dx(11:12) = M*eta1_a + N*eta1_c;
dx(13) = -10*eta1_a(1)*(eta1_a(1)*eta1_b + eta1_a(2)*m(2) - eta1_c);
dx(14) = -eta1_c + dq;             %% IM_1
  
dx(15:18) = M2*eta2_a + N2*eta2_c;
dx(19:20) = -500*[eta2_a(1); eta2_a(3)]*([eta2_a(1) eta2_a(3)]*eta2_b ...
    + [eta2_a(2) eta2_a(4)]*[m2(2); m2(4)] - eta2_c);
dx(21) = -eta2_c + dq;             %% IM_2

dx(22) = dq;
dx(23) = -q + mu1*dq - dq^3 - mu2*q*dq^2 + u;  %% plant

dx(24:25) = -100*rho2'*delta;      %% adaptive law

