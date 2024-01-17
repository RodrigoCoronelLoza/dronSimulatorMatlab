for i=1:length(simulation.plots.theta)
tiempo_t(i)=i*5/9;
end

% figure
% plot(tiempo_t(8640:11205)/1000,simulation.plots.theta(8640:11205)*(180/pi))
% hold on 
% plot(tiempo_t(8640:11205)/1000,simulation.plots.pitchControl(8640:11205)*(180/pi))
% ylabel('\theta [°]');
% xlabel('time[s]');
% ylim ([-36,1])
% 
% grid on
% 
% legend ('simu','ref')

figure
subplot(2,1,1);
plot(tiempo_t(4500:23400)/1000,simulation.plots.theta(4500:23400)*(180/pi))
ylabel('\theta [°]');
xlabel('time[s]');
xlim ([2.5,13])
ylim ([-37,2])
grid on
% legend ('simu','ref')

subplot(2,1,2);
plot(tiempo_t(4500:23400)/1000,simulation.plots.vel_y(4500:23400))
ylabel('vitesse [m/s]');
xlabel('time[s]');
xlim ([2.5,13])
ylim ([-2,16])
% legend ('simu','ref')
grid on