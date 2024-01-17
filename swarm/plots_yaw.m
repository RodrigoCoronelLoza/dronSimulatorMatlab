for i=1:length(simulation.plots.front_graph)
tiempo_y(i)=i*5/9;
end

% figure
% 
% 
% subplot(2,1,1);
% plot(1:length(simulation.plots.psi),simulation.plots.psi)
% hold on
% plot(1:length(simulation.plots.yaw_ref),simulation.plots.yaw_ref)
% xlabel('z_v_e_l');
% ylabel('value');
% 
% 
% subplot(2,1,2);
% 
% plot((1:length(simulation.plots.yaw_prop_ctrl)),simulation.plots.yaw_prop_ctrl)
% hold on
% plot((1:length(simulation.plots.yaw_der_ctrl)),simulation.plots.yaw_der_ctrl)
% hold on
% plot((1:length(simulation.plots.yaw_int_ctrl)),simulation.plots.yaw_int_ctrl)
% 
% % legend ('prop','der')
% legend ('prop','der','int')
% xlabel('command');
% ylabel('value');

figure


subplot(3,1,1);
plot(tiempo_y/1000,simulation.plots.psi)
hold on
plot(tiempo_y/1000,simulation.plots.yaw_ref)
ylabel('yaw');
xlabel('time [s]');
legend('yaw','ref')
grid  on


subplot(3,1,2);

plot(tiempo_y/1000,simulation.plots.yaw_prop_ctrl)
hold on
plot(tiempo_y/1000,simulation.plots.yaw_der_ctrl)
hold on
plot(tiempo_y/1000,simulation.plots.yaw_int_ctrl)

% legend ('prop','der')
legend ('prop','der','int')
xlabel('time [s]');
ylabel('command');
grid on

subplot(3,1,3);

plot(tiempo_y/1000,simulation.plots.yaw_prop_ctrl+simulation.plots.yaw_der_ctrl+simulation.plots.yaw_int_ctrl)

legend ('TOTAL')
xlabel('time [s]');
ylabel('total command');
grid on