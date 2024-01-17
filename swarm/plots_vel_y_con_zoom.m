% figure
% subplot(3,1,1);
% plot(1:length(simulation.plots.vel_y),simulation.plots.vel_y)
% hold on
% plot(1:length(simulation.plots.ref_v_y),simulation.plots.ref_v_y)
% xlabel('y_v_e_l');
% ylabel('value');
% 
% subplot(3,1,2);
% 
% plot((1:length(simulation.plots.vel_y_Control_prop)),simulation.plots.vel_y_Control_prop)
% hold on
% % plot((1:length(simulation.plots.vel_y_Control_der)),simulation.plots.vel_y_Control_der)
% % hold on
% plot((1:length(simulation.plots.vel_y_Control_int)),simulation.plots.vel_y_Control_int)
% 
% 
% legend ('prop','int')
% xlabel('command');
% ylabel('value');
% 
% subplot(3,1,3);
% 
% plot((1:length(simulation.plots.vel_y_Control_prop)),simulation.plots.vel_y_Control_prop...
%     +simulation.plots.vel_y_Control_int)

for i=1:length(simulation.plots.vel_y)
tiempo(i)=i*5/9;
end
figure
subplot(3,1,1);

 plot(tiempo/1000,simulation.plots.vel_y)
 hold on
 plot(tiempo/1000,simulation.plots.ref_v_y)
%  hold on
%  plot(tiempo/1000,simulation.plots.ref_v_y_soft)
legend ('velocity y','ref');
%  legend ('velocity y','ref','soft');
ylabel('y_v_e_l [m/s]');
xlabel('time [s]');
grid  on

axes('position',[.25 .85 .05 .05])
box on % put box around new pair of axes
indexOfInterest = (tiempo < 4000) & (tiempo > 0); % range of t near perturbation

plot(tiempo(indexOfInterest)/1000,simulation.plots.vel_y(indexOfInterest))
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.ref_v_y(indexOfInterest))

grid on
subplot(3,1,2);

plot(tiempo/1000,simulation.plots.vel_y_Control_prop)
% hold on
% plot(tiempo/1000,simulation.plots.vel_y_Control_der)
hold on
plot(tiempo/1000,simulation.plots.vel_y_Control_int)
ylim ([0 0.25])

% hold on
% plot(tiempo/1000,simulation.plots.vel_y_Control_ff)


legend ('prop','int')
% legend ('prop','int','feed fordward')
% legend ('prop','der','int')
ylabel('command [rad]');
xlabel('time[s]');
grid  on

subplot(3,1,3);

plot(tiempo/1000,simulation.plots.vel_y_Control_prop...
    +simulation.plots.vel_y_Control_int)
ylim ([0 0.25])
% plot(tiempo/1000,simulation.plots.vel_y_Control_prop...
%     +simulation.plots.vel_y_Control_int)

legend ('total command')
ylabel(' total command [rad]');
xlabel('time[s]');
grid  on

axes('position',[.25 .35 .05 .05])
box on % put box around new pair of axes
indexOfInterest = (tiempo < 4000) & (tiempo > 0); % range of t near perturbation

plot(tiempo(indexOfInterest)/1000,simulation.plots.vel_y_Control_prop(indexOfInterest)+simulation.plots.vel_y_Control_int(indexOfInterest))
xlim ([-0.1 4])
% ylim ([-16 10])
% axis tight
% legend ('total command')
% xlabel('time[s]');
% ylabel('total command');
grid  on


 