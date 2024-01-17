% figure
% 
% 
% subplot(3,1,1);
% plot(1:length(simulation.plots.vel_z),simulation.plots.vel_z)
% hold on
% plot(1:length(simulation.plots.ref_v_z),simulation.plots.ref_v_z)
% xlabel('z_v_e_l');
% ylabel('value');
% 
% subplot(3,1,2);
% plot(1:length(simulation.plots.z_pos),simulation.plots.z_pos)
% xlabel('z_p_o_s');
% ylabel('value');
% 
% subplot(3,1,3);
% 
% plot((1:length(simulation.plots.thrustControl_prop)),simulation.plots.thrustControl_prop)
% hold on
% plot((1:length(simulation.plots.thrustControl_der)),simulation.plots.thrustControl_der)
% hold on
% plot((1:length(simulation.plots.thrustControl_int)),simulation.plots.thrustControl_int)
% 
% % legend ('prop','der')
% legend ('prop','der','int')
% xlabel('command');
% ylabel('value');


for i=1:length(simulation.plots.vel_z)
tiempo(i)=i*5/9;
end

figure


subplot(4,1,1);
plot(tiempo/1000,simulation.plots.vel_z)
% hold on
% plot(tiempo/1000,simulation.plots.ref_v_z)
hold on
plot(tiempo/1000,simulation.plots.ref_v_z_soft)
xlabel('time [s]');
ylabel('z_v_e_l [m/s]');
legend ('velocity z','smooth ref');
grid  on

% axes('position',[.25 .85 .05 .05])
% box on % put box around new pair of axes
% indexOfInterest = (tiempo < 4000) & (tiempo > 0); % range of t near perturbation
% 
% plot(tiempo(indexOfInterest)/1000,simulation.plots.vel_z(indexOfInterest))
% hold on
% plot(tiempo(indexOfInterest)/1000,simulation.plots.ref_v_z(indexOfInterest))

% grid on
%  subplot(4,1,2);
% plot(tiempo/1000,simulation.plots.z_pos)
% xlabel('time [s]');
% ylabel('z_p_o_s');
% grid  on

subplot(4,1,2);

plot(tiempo/1000,simulation.plots.thrustControl_prop)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_der)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_int)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_double_integral)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_ff)

legend ('prop-pos','prop-vel','int','double int','feed forward')
xlabel('time[s]');
ylabel('command [N]');
grid  on

axes('position',[.25 .65 .35 .10])
box on % put box around new pair of axes
indexOfInterest = (tiempo < 20000) & (tiempo > 500); % range of t near perturbation
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_prop(indexOfInterest)) % plot on new axes
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_der(indexOfInterest)) % plot on new axes
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_int(indexOfInterest)) % plot on new axes
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_double_integral(indexOfInterest))
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_ff(indexOfInterest))

axis tight
% hold on
% plot(tiempo/1000,simulation.plots.thrustControl_double_integral)
% hold on
% plot(tiempo/1000,simulation.plots.thrustControl_ff)
% legend ('prop','der')
% legend ('prop','der','int','double_int','feed_forward')
legend ('prop-pos','prop-vel','int','double int','feed forward')
xlabel('time[s]');
ylabel('command [N]');
grid  on

axes('position',[.65 .65 .35 .10])
box on % put box around new pair of axes
indexOfInterest = (tiempo < 220000) & (tiempo > 200000); % range of t near perturbation
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_prop(indexOfInterest)) % plot on new axes
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_der(indexOfInterest)) % plot on new axes
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_int(indexOfInterest)) % plot on new axes
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_double_integral(indexOfInterest))
hold on
plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_ff(indexOfInterest))

ylim([-0.5 1])
% axis tight
% hold on
% plot(tiempo/1000,simulation.plots.thrustControl_double_integral)
% hold on
% plot(tiempo/1000,simulation.plots.thrustControl_ff)
% legend ('prop','der')
% legend ('prop','der','int','double_int','feed_forward')
legend ('prop-pos','prop-vel','int','double int','feed forward')
xlabel('time[s]');
ylabel('command [N]');
grid  on

subplot(4,1,3);
subplot(4,1,4);

plot(tiempo/1000,simulation.plots.thrustControl_prop+simulation.plots.thrustControl_der+...
    simulation.plots.thrustControl_int+simulation.plots.thrustControl_double_integral+simulation.plots.thrustControl_ff)

% plot(tiempo/1000,simulation.plots.thrustControl_prop+simulation.plots.thrustControl_der+simulation.plots.thrustControl_int)
% axis tight
legend ('total command')
xlabel('time[s]');
ylabel('total command [N]');
grid  on
% axes('position',[.25 .35 .05 .05])
% box on % put box around new pair of axes
% indexOfInterest = (tiempo < 4000) & (tiempo > 0); % range of t near perturbation
% 
% plot(tiempo(indexOfInterest)/1000,simulation.plots.thrustControl_prop(indexOfInterest)+simulation.plots.thrustControl_der(indexOfInterest)+simulation.plots.thrustControl_int(indexOfInterest))
% xlim ([-0.1 4])
% ylim ([-16 10])
% % axis tight
% % legend ('total command')
% % xlabel('time[s]');
% % ylabel('total command');
% grid  on
