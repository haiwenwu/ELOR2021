clc; clear all; close all;

[t,x] = ode23('single_link', [0 90], zeros(21,1));

%% ==================== Reference ====================
qr = zeros(length(t),1);
for i=1:length(t)
    if t(i)<=30
        qr(i) = sin(pi/6*t(i));
    elseif t(i)<=60
        qr(i) = -pi/3 + cos(pi/6*t(i));
    else
        qr(i) = pi/2 + sin(pi/4*t(i));
    end
end

%% ==================== Plot ====================
figure(1)
subplot(2,1,1)
plot(t,qr,'k','linewidth',1.0);
hold on; grid on; box off;
line([30,30],[-3,3],'Color',[0.6 0.6 0.6]);
line([60,60],[-3,3],'Color',[0.6 0.6 0.6]);
axis([0 90 -3 3]);
ylabel('$q_{\rm ref}$ (rad)','FontSize',14,'Interpreter','latex');

subplot(2,1,2)
plot(t,x(:,1) - qr,'k','linewidth',1.0);
hold on; grid on; box off;
line([30,30],[-3 3],'Color',[0.6 0.6 0.6]);
line([60,60],[-3 3],'Color',[0.6 0.6 0.6]);
axis([0 90 -3 3]);
xlabel('Time (s)','FontSize',14,'Interpreter','latex');
ylabel('$e$ (rad)','FontSize',14,'Interpreter','latex');


