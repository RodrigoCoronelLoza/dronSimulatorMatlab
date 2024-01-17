figure


subplot(2,1,1);
plot(1:length(simulation.plots.psi),simulation.plots.psi)
% hold on
% plot(1:length(simulation.plots.ref_v_z),simulation.plots.ref_v_z)
xlabel('z_v_e_l');
ylabel('value');


subplot(2,1,2);

plot((1:length(simulation.distancelock3DController.distanceLock3DController.inner.Current_yaw_prop)),simulation.distancelock3DController.distanceLock3DController.inner.Current_yaw_prop)
hold on
plot((1:length(simulation.distancelock3DController.distanceLock3DController.inner.Current_yaw_der)),simulation.distancelock3DController.distanceLock3DController.inner.Current_yaw_der)
hold on
plot((1:length(simulation.distancelock3DController.distanceLock3DController.inner.Current_yaw_int)),simulation.distancelock3DController.distanceLock3DController.inner.Current_yaw_int)

% legend ('prop','der')
legend ('prop','der','int')
xlabel('command');
ylabel('value');

