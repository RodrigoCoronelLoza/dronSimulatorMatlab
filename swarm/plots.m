%%%%%%%%%%%%%%%%%%%%5
figure
subplot(2,2,1);
plot(1:length(simulation.plots.frontEffective),simulation.plots.frontEffective)
xlabel('Actual distance front');
ylabel('value');
subplot(2,2,2);
plot(1:length(simulation.plots.backEffective),simulation.plots.backEffective)
xlabel('Actual distance back');
ylabel('value');
subplot(2,2,3);
plot(1:length(simulation.plots.rightEffective),simulation.plots.rightEffective)
xlabel('Actual distance right');
ylabel('value');
subplot(2,2,4);
plot(1:length(simulation.plots.leftEffective),simulation.plots.leftEffective)
xlabel('Actual distance left');
ylabel('value');
%%%%%%%%%
figure
plot(1:length(simulation.plots.front_graph),simulation.plots.front_graph)
hold on
plot(1:length(simulation.plots.ref_dist_ortho),simulation.plots.ref_dist_ortho)
xlabel('front_graph');
ylabel('value');
%%%%%%%%%%%%%%%%%
figure
subplot(3,1,1);
plot(1:length(simulation.plots.vel_x),simulation.plots.vel_x)
xlabel('x_v_e_l');
ylabel('value');
subplot(3,1,2);
plot(1:length(simulation.plots.vel_y),simulation.plots.vel_y)
xlabel('y_v_e_l');
ylabel('value');
subplot(3,1,3);
plot(1:length(simulation.plots.vel_z),simulation.plots.vel_z)
xlabel('z_v_e_l');
ylabel('value');
%%%%%%%%%%%%%%%%%%%%%
figure
subplot(2,2,1);
plot(1:length(simulation.plots.frontViewed),simulation.plots.frontViewed)
xlabel('Viewed distance front');
ylabel('value');
subplot(2,2,2);
plot(1:length(simulation.plots.backViewed),simulation.plots.backViewed)
xlabel('Viewed distance back');
ylabel('value');
subplot(2,2,3);
plot(1:length(simulation.plots.rightViewed),simulation.plots.rightViewed)
xlabel('Viewed distance right');
ylabel('value');
subplot(2,2,4);
plot(1:length(simulation.plots.leftViewed),simulation.plots.leftViewed)
xlabel('Viewed distance left');
ylabel('value');
%%%%%%%%%%%%%%%%%%%%%
figure
subplot(3,1,1);
plot(1:length(simulation.plots.phi),simulation.plots.phi)
xlabel('\phi');
ylabel('value');
subplot(3,1,2);
plot(1:length(simulation.plots.psi),simulation.plots.psi)
xlabel('\psi');
ylabel('value');
subplot(3,1,3);
plot(1:length(simulation.plots.theta),simulation.plots.theta)
xlabel('\theta');
ylabel('value');
%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(4,1,1);
plot(1:length(simulation.plots.rollControl),simulation.plots.rollControl)
xlabel('angle roll Control');
ylabel('value');
subplot(4,1,2);
plot(1:length(simulation.plots.yawControl),simulation.plots.yawControl)
xlabel('angle yaw control');
ylabel('value');
subplot(4,1,3);
plot(1:length(simulation.plots.pitchControl),simulation.plots.pitchControl)
xlabel('angle pitch control');
ylabel('value');
subplot(4,1,4);
plot(1:length(simulation.plots.thrustControl),simulation.plots.thrustControl)
xlabel('thrust control');
ylabel('value');
%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(3,1,1);
plot(1:length(simulation.plots.x_pos),simulation.plots.x_pos)
xlabel('x_p_o_s');
ylabel('value');
subplot(3,1,2);
plot(1:length(simulation.plots.y_pos),simulation.plots.y_pos)
xlabel('y_p_o_s');
ylabel('value');
subplot(3,1,3);
plot(1:length(simulation.plots.z_pos),simulation.plots.z_pos)
xlabel('z_p_o_s');
ylabel('value');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(4,1,1);
plot(1:length(simulation.plots.vel_y),simulation.plots.vel_y)
xlabel('y_v_e_l');
ylabel('value');

subplot(4,1,2);
plot(1:length(simulation.plots.vel_z),simulation.plots.vel_z)
xlabel('z_v_e_l');
ylabel('value');

subplot(4,1,3);
plot(1:length(simulation.plots.z_pos),simulation.plots.z_pos)
xlabel('z_p_o_s');
ylabel('value');

subplot(4,1,4);

plot(1:length(simulation.plots.thrustControl_prop),simulation.plots.thrustControl_prop)
hold on
plot(1:length(simulation.plots.thrustControl_der),simulation.plots.thrustControl_der)
xlabel('command');
ylabel('value');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure

% plot(simulation.plots.x_pos,simulation.plots.y_pos)
% xlabel('x_p_o_s');
% ylabel('y_p_o_s');
plot3(simulation.plots.x_pos,simulation.plots.y_pos, simulation.plots.z_pos)
xlabel('x_p_o_s');
ylabel('y_p_o_s');
zlabel('z_p_o_s');
axis equal
figure
%simulation.obstacleWorld.sphere.graph_sphere
plot3(simulation.plots.x_pos,simulation.plots.y_pos, simulation.plots.z_pos,'-r')
xlabel('x_p_o_s');
ylabel('y_p_o_s');
zlabel('z_p_o_s');
hold on
%simulation.obstacleWorld.cilindre.graph_cylinder
%simulation.obstacleWorld.cube.graph_parale
simulation.obstacleWorld.plan_incl.graph_plan_incl;
%simulation.obstacleWorld.cilindre_y_z.graph_cylinder_y_z;
axis equal