clc; clear all; close all;

% options = odeset('AbsTol',1e-9,'RelTol',1e-5);
[t,x] = ode15s('van_der_Pol', [0 110], [1; 0; 1; 0; 1; 0; 1; 0; 1; 0; zeros(15,1)]);


%% ==================== Reference ====================
qr = (x(:,1)+x(:,3)).*(t>=0 & t<25) + (x(:,5)).*(t>=25 & t<90) + 0.*(t>=90 & t<=110);

e1 = x(:,22)-x(:,1)-x(:,3);
e2 = x(:,22)-x(:,5);
e5 = x(:,22)-0;


%% ==================== Plot ====================
figure(1)
subplot(2,1,1)
line([25,25],[-2.5,2.5],'Color',[0.5 0.5 0.5]);
hold on; grid on
line([50,50],[-2.5,2.5],'Color',[0.5 0.5 0.5]);
line([60,60],[-2.5,2.5],'Color',[0.5 0.5 0.5]);
line([90,90],[-2.5,2.5],'Color',[0.5 0.5 0.5]);
plot(t,qr,'k','linewidth',1.0);
ylabel('$q_{\rm ref}$','FontSize',14,'Interpreter','latex');
axis([0 110 -2.5 2.5]);

subplot(2,1,2)
line([25,25],[-1.5,1],'Color',[0.5 0.5 0.5]);
hold on; grid on;
line([50,50],[-1.5,1],'Color',[0.5 0.5 0.5]);
line([60,60],[-1.5,1],'Color',[0.5 0.5 0.5]);
line([90,90],[-1.5,1],'Color',[0.5 0.5 0.5]);
plot(t,(e1).*(t>=0 & t<25) + (e2).*(t>=25 & t<90) + e5.*(t>=90 & t<=110),'k','linewidth',1.0);
xlabel('Time (s)','FontSize',14,'Interpreter','latex');
ylabel('$e$','FontSize',14,'Interpreter','latex');
axis([0 110 -1.5 1]);
