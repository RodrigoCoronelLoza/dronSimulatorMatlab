% figure
% subplot(2,1,1);
% plot(1:length(simulation.plots.front_graph),simulation.plots.front_graph)
% hold on
% 
% plot(1:length(simulation.plots.ref_dist_ortho),simulation.plots.ref_dist_ortho)
% % length(simulation.plots.ref_dist_ortho)
% % plot(1:length(simulation.plots.ref_dist_ortho),ones(1,length(simulation.plots.ref_dist_ortho)))
% grid on
% xlabel('front_graph');
% ylabel('value');

for i=1:length(simulation.plots.front_graph)
tiempo_f(i)=i*5/9;
end

% tiempo_f=transpose(tiempo_f);
% frente=simulation.plots.front_graph;
% frente=transpose(frente);
% tiempo_f(1,63:1:end)
figure
% plot(tiempo_f(1,63:1:end)/1000,simulation.plots.front_graph(1,63:1:end))
% hold on
plot(tiempo_f/1000,simulation.plots.front_graph)
% plot(tiempo_f(63,:)/1000,frente(63,:))
% frente(63,:)
% hold on
% plot(tiempo_f/1000,simulation.plots.ref_dist_ortho)
hold on
plot(tiempo_f/1000,simulation.plots.ref_dist_ortho_soft)
legend ('distance seen','smooth ref')
ylabel('orthogonal distance head US sensor [m]');
xlabel('time [s]');
grid on
