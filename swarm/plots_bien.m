for i=1:length(simulation.plots.x_pos)
tiempo_t(i)=i*5/9;
end
%%%%%%%%%%%%%%%%%%%%%%%%% position
figure
subplot(3,1,1);
plot(tiempo_t/1000,simulation.plots.x_pos)
ylabel('x_p_o_s [m]');
xlabel('time [s]');
grid on
subplot(3,1,2);
plot(tiempo_t/1000,simulation.plots.y_pos)
ylabel('y_p_o_s [m]');
xlabel('time[s]');
grid on
subplot(3,1,3);
plot(tiempo_t/1000,simulation.plots.z_pos)
ylabel('z_p_o_s [m]');
xlabel('time[s]');
grid on
%%%%%%%%%%%%%%%%%%%%%% velocities
figure
subplot(3,1,1);
plot(tiempo_t/1000,simulation.plots.vel_x)
ylabel('x_v_e_l');
xlabel('time [s]');
grid on
subplot(3,1,2);
plot(tiempo_t/1000,simulation.plots.vel_y)
ylabel('y_v_e_l');
xlabel('time[s]');
grid on
subplot(3,1,3);
plot(tiempo_t/1000,simulation.plots.vel_z)
ylabel('z_v_e_l');
xlabel('time[s]');
grid on
%%%%%%%%%%%%%%%%%%%%%% angles
figure
subplot(3,1,1);
plot(tiempo_t/1000,simulation.plots.phi*(180/pi))
ylabel('\phi [°]');
xlabel('time[s]');
grid on
subplot(3,1,2);
plot(tiempo_t/1000,simulation.plots.psi*(180/pi))
ylabel('\psi [°]');
xlabel('time[s]');
grid on
subplot(3,1,3);
plot(tiempo_t/1000,simulation.plots.theta*(180/pi))
ylabel('\theta [°]');
xlabel('time[s]');
grid on

%%%%%%%%%%%%%%%%%%%%%% angles head
%%
figure
subplot(2,1,1);
plot(tiempo_t/1000,simulation.plots.theta_head)
ylabel('theta head [°]');
xlabel('time [s]');
grid on
subplot(2,1,2);
plot(tiempo_t/1000,simulation.plots.phi_head)
ylabel('phi head [°]');
xlabel('time[s]');
grid on
%%

%%%%%% trajectory

figure

plot3(simulation.plots.x_pos,simulation.plots.y_pos, simulation.plots.z_pos,'-r')
xlabel('x_p_o_s [m]');
ylabel('y_p_o_s [m]');
zlabel('z_p_o_s [m]');
hold on



%simulation.obstacleWorld.cilindre.graph_cylinder
%simulation.obstacleWorld.cube.graph_parale
simulation.obstacleWorld.plan_incl.graph_plan_incl;
% simulation.obstacleWorld.cilindre_y_z.graph_cylinder_y_z;
%  simulation.obstacleWorld.paraboloid_y_z.graph_paraboloide_y_z;
% axis equal
figure

plot3(simulation.plots.x_pos,simulation.plots.y_pos, simulation.plots.z_pos,'-r')
xlabel('x_p_o_s [m]');
ylabel('y_p_o_s [m]');
zlabel('z_p_o_s [m]');
hold on

% ylim ([0 1000])
% zlim ([0 1000])
% axis equal
%simulation.obstacleWorld.cilindre.graph_cylinder
% simulation.obstacleWorld.cube.graph_parale
simulation.obstacleWorld.plan_incl.graph_plan_incl;
% simulation.obstacleWorld.paraboloid_y_z.graph_paraboloide_y_z;
view (90,0)
