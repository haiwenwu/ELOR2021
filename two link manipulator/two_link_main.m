clc; clear all; close all;

[t,x] = ode15s('two_link', [0 100], zeros(46,1));

%% ==================== Reference ====================
s1 = pi/3; s2 = pi/6;
a1 = 1; a2 = 0.5;
xr = zeros(length(t),2);
for i=1:length(t)
    if t(i)<=30
        xr(i,1:2) = [a1*sin(s1*t(i)) a2*sin(s1*t(i))];
    elseif t(i)<=60
        xr(i,1:2) = 2.*[a1*(1-cos(s1*t(i))) a2*(1-cos(s1*t(i)))];
    else
        xr(i,1:2) = 2.*[a1*(-1+cos(s2*t(i))) a2*(-1+cos(s2*t(i)))];
    end
end

%% ==================== Plot ====================
figure(2)
subplot(2,1,1)
line([30,30],[-4,4],'Color',[0.6 0.6 0.6]);
hold on; grid on; box off;
line([60,60],[-4,4],'Color',[0.6 0.6 0.6]);
L1 = plot(t,xr(:,1),'k','linewidth',1.0);
L2 = plot(t,xr(:,2),'m','linewidth',1.0);
axis([0 100 -4 4]);
ylabel('$q_{\rm ref}$ (rad)','FontSize',14,'Interpreter','latex');
legend1 = legend([L1,L2],'$q_{1,{\rm ref}}$','$q_{2,{\rm ref}}$');
set(legend1,...
    'Interpreter','latex',...
    'EdgeColor',[0.8 0.8 0.8],...
    'FontSize',14);
% axis([0 90 -2.5 3]);

subplot(2,1,2)
line([30,30],[-0.2,0.2],'Color',[0.6 0.6 0.6]);
hold on; grid on; box off;
line([60,60],[-0.2,0.2],'Color',[0.6 0.6 0.6]);
L1 = plot(t,x(:,1)-xr(:,1),'k','linewidth',1.0);
L2 = plot(t,x(:,2)-xr(:,2),'m','linewidth',1.0);
axis([0 100 -0.2 0.2]);
xlabel('Time (s)','FontSize',14,'Interpreter','latex');
ylabel('$e$ (rad)','FontSize',14,'Interpreter','latex');
legend2 = legend([L1,L2],'$e_{1}$','$e_{2}$');
set(legend2,...
    'Interpreter','latex',...
    'EdgeColor',[0.8 0.8 0.8],...
    'FontSize',14);
% axis([0 90 -0.15 0.1]);
