%! U.S. simulation script
clear all; close all; clc

figure
hold on

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
axis equal

%! Simulation parameters
% simulationSteps = 20000;            % expressed in milliseconds

%! Objects
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   drone = drone_model;                %!instantiation of a drone object
%   obstaculo=obstacle;                 %!instantiation of an obstacle object  
%   obstaculo.cil;
%   obstaculo.cylinder(1,3,0,2,1,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simulation = US_simulator;          %!instantiation of a simulation object

% %! Set the duration of the simulation
% tic
% for k=1:simulationSteps
%     updateSimulation(simulation)
% end
% toc

%! Illimited simulation
while true
    updateSimulation(simulation)
end

us_control(simulation)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BW = imread('drone.jpg');
[rows,cols] = find(BW==0);
scatter(cols,rows);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

