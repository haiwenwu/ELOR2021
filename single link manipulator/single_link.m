function dx = single_link(t,x)
% This example shows how to implement the self-learning mechanism for the
% output regulation of single-link manipulator
% Copyright 2020 Haiwen Wu


%% ==================== System parameters ====================
a1 = 1/12; a2 = 9.8;
a = [a1; a2];

%% ==================== States ====================
dx = zeros(21,1);
q = x(1);
xi = x(2);
eta_a = x(3:4); eta_a1 = eta_a(1); eta_a2 = eta_a(2);
eta_b = x(5);
eta_c = x(6);
zeta2 = reshape(x(7:12),2,3);
zeta1 = x(13:14);
est = x(15:21);

%% ==================== Reference and disturbance ===============
qr = sin(pi/6*t).*(t>=0 & t<30) + (-pi/3 + cos(pi/6*t)).*(t>=30 & t<60) ...
    + (pi/2 + sin(pi/4*t)).*(t>=60 & t<=90);
d = cos(0.5*t);

%% ==================== Controller parameters ===================
m = [1; 1.4142]; 
M = [0 1; -m(1) -m(2)]; N = [0; 1]; 
% etaA = x(3:4);
etaB = [eta_b; m(2)];
varrho = m - etaB;
Phi0 = [0 1; -varrho(1) -varrho(2)];
Gamma = (m - varrho)'*(Phi0 + eye(2));
pos = Gamma*inv(Phi0)*eta_a;
vel = Gamma*eta_a;
acc = Gamma*Phi0*eta_a;
Y = [acc, cos(pos), -sin(pos)];

Gamma0 = [0 m(2)];
Gamma2 = [1 0];
rho1 = Gamma0 * zeta1;
rho2 = [Gamma2 * zeta1, Gamma2 * zeta2, Y - Gamma0*zeta2];

e = q - qr;
delta = 10*(1 + 0*e^2)*e + xi - Gamma*eta_a;
u = rho1 + rho2*est - 10*(1 + 0*delta^2)*delta;


%% ODE functions
dx(1) = x(2);
dx(2) = a1^(-1)*(u - d - a2*cos(q));   %% System

dx(3:4) = M*eta_a + N*eta_c;
dx(5) = - 1.0*eta_a1*(eta_a1*eta_b + eta_a2*m(2) - eta_c);
dx(6) = - eta_c + xi;  %% Internal model 1

dx(7:12) = kron(eye(3),M)*x(15:20) + kron(eye(3),N)*Y';
dx(13:14) = M*zeta1 + N*u;  %% Internal model 1
 
dx(15:21) = -1*rho2*delta;  %% Adaptive law
