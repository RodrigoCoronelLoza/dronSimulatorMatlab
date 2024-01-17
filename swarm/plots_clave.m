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


subplot(3,1,1);
plot(tiempo/1000,simulation.plots.vel_z)
hold on
% plot(tiempo/1000,simulation.plots.ref_v_z)
% hold on
plot(tiempo/1000,simulation.plots.ref_v_z_soft)
xlabel('time [s]');
ylabel('z_v_e_l [m/s]');
legend ('velocity z','smooth ref');
grid  on

%  subplot(4,1,2);
% plot(tiempo/1000,simulation.plots.z_pos)
% xlabel('time [s]');
% ylabel('z_p_o_s');
% grid  on

subplot(3,1,2);

plot(tiempo/1000,simulation.plots.thrustControl_prop)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_der)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_int)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_double_integral)
hold on
plot(tiempo/1000,simulation.plots.thrustControl_ff)
% legend ('prop','der')
legend ('prop','der','int','double_int','feed_forward')
% legend ('prop','der','int')
% legend ('prop','der','int','feed forward')
xlabel('time[s]');
ylabel('command [N]');
grid  on

subplot(3,1,3);

plot(tiempo/1000,simulation.plots.thrustControl_prop+simulation.plots.thrustControl_der+...
    simulation.plots.thrustControl_int+simulation.plots.thrustControl_double_integral+simulation.plots.thrustControl_ff)

% plot(tiempo/1000,simulation.plots.thrustControl_prop+simulation.plots.thrustControl_der+simulation.plots.thrustControl_int)

legend ('total command')
xlabel('time[s]');
ylabel('total command [N]');
grid  on
