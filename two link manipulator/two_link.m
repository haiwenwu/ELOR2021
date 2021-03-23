function dx = two_link(t,x)
% This example shows how to implement the self-learning mechanism for the
% output regulation of two-link manipulator

% Copyright 2020 Haiwen Wu

%% ==================== States ====================
dx = zeros(46,1);
q = x(1:2); % position
dq = x(3:4); % velocity
eta1_a = x(5:6); eta1_b = x(7); eta1_c = x(8);     %% IM1_1
eta2_a = x(9:10); eta2_b = x(11); eta2_c = x(12);  %% IM1_2
zeta_a = reshape(x(13:28),4,4); zeta_b = x(29:32); %% IM2
est = x(33:46); %% parameter estimation


%% ==================== System parameter ====================
theta = [3.9000; 0.7500; 1.1250; 0; 0];


%% ==================== Reference and disturbance ===============
s1 = pi/3; s2 = pi/6; %% frequencies
a1 = 1; a2 = 0.5;     %% amplitudes
qr1 = a1*sin(s1*t).*(t>=0 & t<30) + 2*a1*(1-cos(s1*t)).*(t>=30 & t<60) ...
    + 2*a1*(-1+cos(s2*t)).*(t>=60 & t<100);
qr2 = a2*sin(s1*t).*(t>=0 & t<30) + 2*a2*(1-cos(s1*t)).*(t>=30 & t<60) ...
    + 2*a2*(-1+cos(s2*t)).*(t>=60 & t<100);
qr = [qr1; qr2];

k = fix(t+1);
if k-1/k^2 <= t && t<=k
    d01 = 1; d02 = 1;
else
    d01 = 0; d02 = 0;
end
d0 = [d01; d02];           %% unmodeled disturbance
d = [sin(2*t); sin(2*t)];  %% modeled disturbance


%% ==================== Controller parameters ===================
%% IM1_1
m = [1; 1.4142]; eta1B = [eta1_b; m(2)];
M = [0 1; -m(1) -m(2)]; N = [0; 1];
varrho1 = m-eta1B; 
Phia1 = [0 1; -varrho1(1) -varrho1(2)];
Gamma1 = (m-varrho1)'*(Phia1+eye(2));  

%% IM1_2
eta2B = [eta2_b; m(2)];
varrho2=m-eta2B; 
Phia2 = [0 1; -varrho2(1) -varrho2(2)];
Gamma2 = (m-varrho2)'*(Phia2+eye(2));  

%% Error signal
e = q - qr;
delta = [10 0; 0 10]*(1+e.^2).*e + dq - [Gamma1*eta1_a; Gamma2*eta2_a];

%% Certainty equivalence property
pos = [Gamma1*inv(Phia1)*eta1_a; Gamma2*inv(Phia2)*eta2_a];
vel = [Gamma1*eta1_a; Gamma2*eta2_a];
acc = [Gamma1*Phia1*eta1_a; Gamma2*Phia2*eta2_a];
Ybar = Y(pos, vel, acc, vel);  %%  regressor matrix


%% IM2
Mbar = [M, zeros(2,2); zeros(2,2), M];
Nbar = [N, zeros(2,1); zeros(2,1), N];
Gamma_0 = [0 m(2) 0 0; 0 0 0 m(2)];
Gamma_1 = [1 0 0 0; 0 0 0 0];
Gamma_2 = [0 0 0 0; 0 0 1 0];
rho1 = Gamma_0*zeta_b;
rho2 = [Gamma_1*zeta_b, Gamma_2*zeta_b, Gamma_1*zeta_a, ...
    Gamma_2*zeta_a, Ybar - Gamma_0*zeta_a]; %% dimension: 2 + 2*4 + 4

%% input
u = rho1 + rho2*est - 10*(8+delta.^2).*delta;


%% ==================== ODE functions ====================
dx(1:4) = EL2(theta,q,dq,u - d -d0); %% EL equation

dx(5:6) = M*eta1_a + N*eta1_c;
dx(7) = -10*eta1_a(1)*(eta1_a(1)*eta1_b + eta1_a(2)*m(2) - eta1_c);
dx(8) = -eta1_c + dq(1);           %% IM1_1

dx(9:10) = M*eta2_a + N*eta2_c;
dx(11) = -10*eta2_a(1)*(eta2_a(1)*eta2_b + eta2_a(2)*m(2) - eta2_c);
dx(12) = -eta2_c + dq(2);          %% IM1_2

dx(13:28) = kron(eye(4),Mbar)*x(13:28) + kron(eye(4),Nbar)*reshape(Ybar,8,1);
dx(29:32) = Mbar*zeta_b + Nbar*u;  %% IM2

dx(33:46) = -30*rho2'*delta;       %% adaptive law
