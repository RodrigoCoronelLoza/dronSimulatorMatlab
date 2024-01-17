clear all;
close all;
clc;

line_sfx1_obstacle_repulsion;

%% Config and set gains
addpath('../../nav/mexes'); % Path to the loadjson script
config = loadjson('mix.cfg'); % Load config parameters

mex_sfx1_obstacle_repulsion('set_gains',...
   config.repulsion.AngleLeft,config.repulsion.AngleRight,config.repulsion.AngleMin,....
   config.repulsion.AngleMax,config.repulsion.DistanceMinToObstacle,config.repulsion.DistanceMaxToObstacle);

%%initializations
Duration_simu_s = 200;
Frequency = 200;
state = zeros(Duration_simu_s*Frequency,12);
distanceMeasured = zeros(Duration_simu_s*Frequency,4);
qRef = zeros(Duration_simu_s*Frequency,4);
qRef(:,1) = ones(Duration_simu_s*Frequency,1);
%%wind
% windDirectionDeg = 0;
% norm_factor = 1;%% gain on wind norm
% mean_wind = 10;%% km per seconds
% wind_model;
windVelocity = zeros(Duration_simu_s*Frequency,2);
eXom = drone_model();
%%TODO build objects
%myObstacle = obstacle(); yes

%us = ultrasoundObject(myObstacle);

%%TODO model ultrasound
%us.setFrequency(myFrequency);
%us.setPattern(myPattern);
eXom.setPositionNed(2,2,0)
%% main loop
for k = 2:1:Duration_simu_s * Frequency
    t(k,1) = k / Frequency ;
    
    %%TODO insert control command law here
%     phiRef = 0;
%     thetaRef = -10*pi/180*0;
%     psiRef = -pi*0;
    zRef = 0;
    
    %%TODO Pasquale
    %   [phiRef,thetaRef] =  obstacle_avoidance(usObject);
    
    %%simulating model
    
    %%TODO step US
    %us.update(eXom.previousState.positionNed);
    distanceMeasured(k,1) = 2+eXom.previousState.positionNed.y;
    [qRef(k,:)] = ....
        mex_sfx1_obstacle_repulsion('step',distanceMeasured(k,1),distanceMeasured(k,2),distanceMeasured(k,3),distanceMeasured(k,4));
    
    [psiRef(k,1), thetaRef(k,1), phiRef(k,1)] =  quat2_euler_angle(qRef(k,:));
    eXom.updateModel(phiRef(k,1),thetaRef(k,1),psiRef(k,1),zRef,windVelocity);
    state(k,:) = eXom.getCurrentState();
end

outputs = mex_sfx1_obstacle_repulsion('get_traces');

%% Plotting
figure(1)
plot3(state(:,2),state(:,1),state(:,3));
grid on;

figure(2)
ax(1) = subplot(311);
plot(t,state(:,2))
grid on;
ax(2) = subplot(312);
plot(t,[state(:,10) phiRef]*180/pi)
grid on;
ax(3) = subplot(313);
grid on;