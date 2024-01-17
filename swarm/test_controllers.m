clear all
close all
clc
%%
%X Y axis
line_sfx1_quaternion;
line_sfx1_angular_rates;

%
line_sfx1_heading;
line_sfx1_yawRate;

% Z axis
line_sfx1_altitude;

% Mix
line_sfx1_mix;
%
%% Load dataset
load('~/Data/Mat_files/EX-00-07397_0400.mat'); % Load your dataset 397/400/(178)
bindEstimation = '';
t = (eval([bindEstimation 'controllers_timestamp'])-eval([bindEstimation 'controllers_timestamp(1)']))/1000000; % time vector [s]
%%indices
indices = 1:length(t);
%% Or simulating
% tMax = 30;
% t = 0:0.005:tMax;

%% Config and set gains
addpath('../../nav/mexes'); % Path to the loadjson script
config = loadjson('mix.cfg'); % Load config parameters

% Mix
D1 = config.mix.D1;
D2 = config.mix.D2;
D3 = config.mix.D3;
D4 = config.mix.D4;
K = config.mix.K;
Kt = config.mix.Kt;
rho0 = config.mix.rho0;
rho = config.mix.rho;
minRpm = config.mix.minRpm;
maxRpm = config.mix.maxRpm;
maxPwm = config.mix.maxPwm;
minimalTauZGuaranteed = config.mix.minimalTauZGuaranteed;

mex_sfx1_mix('set_gains',...
   D1, D2, D3, D4, K, Kt, rho0, rho, minRpm, maxRpm, maxPwm, minimalTauZGuaranteed);

% Quaternions
Kpx = config.quat.Kpx;
Kpy = config.quat.Kpy;
Kpz = config.quat.Kpz;
Kdx = config.quat.Kdx;
Kdy = config.quat.Kdy;
Kdz = config.quat.Kdz;
Kix = config.quat.Kix;
Kiy = config.quat.Kiy;
Kiz = config.quat.Kiz;
SatIx = config.quat.SatIx;
SatIy = config.quat.SatIy;
SatIz = config.quat.SatIz;

mex_sfx1_quaternion('set_gains', ...
   Kpx, Kpy, Kpz, Kix, Kiy, Kiz, Kdx, Kdy, Kdz, ...
   SatIx, SatIy, SatIz);

% yawRate
Kp = config.yawRate.Kp;
Ki = config.yawRate.Ki;
Kd = config.yawRate.Kd;
SatI = config.yawRate.SatI;
tauMax = config.yawRate.tauMax;

mex_sfx1_yawRate('set_gains', ...
    Kp, Ki, Kd, SatI, tauMax);

% angularRates
Kpx = config.angularRates.Kpx;
Kpy = config.angularRates.Kpy;
Kdx = config.angularRates.Kdx;
Kdy = config.angularRates.Kdy;
Kix = config.angularRates.Kix;
Kiy = config.angularRates.Kiy;
SatIx = config.angularRates.SatIx;
SatIy = config.angularRates.SatIy;
maxCmd = config.angularRates.maxCmd;

mex_sfx1_angular_rates('set_gains', ...
    Kpx, Kpy, Kix, Kiy, Kdx, Kdy, SatIx, SatIy, maxCmd);

% heading
Kp = config.heading.Kp;
Kd = config.heading.Kd;
Ki = config.heading.Ki;
SatI = config.heading.SatI;

mex_sfx1_heading('set_gains', ...
    Kp, Ki, Kd, SatI);

% altitude
Kp = config.alt.Kp;
Kd = config.alt.Kd;
defaultKi = config.alt.defaultKi;
defaultSatI = config.alt.defaultSatI;
satOutput = config.alt.satOutput;
altitudeDerivativeCommandFilterB = config.alt.altitudeDerivativeCommandFilterB;
altitudeDerivativeCommandFilterA = config.alt.altitudeDerivativeCommandFilterA;

mex_sfx1_altitude('set_gains', ...
    Kp, defaultKi, Kd, defaultSatI,satOutput,...
    altitudeDerivativeCommandFilterA(1), altitudeDerivativeCommandFilterA(2), ...
    altitudeDerivativeCommandFilterB(1), altitudeDerivativeCommandFilterB(2));

%%
%state
position = zeros(length(t),3);
velocity = zeros(length(t),3);
euleurAngles = zeros(length(t),3);

%quat
    %%in
quatRef = zeros(length(t),4);
quatEst = zeros(length(t),4);
quatRef(:,1) = ones(length(t),1);
quatEst(:,1) = ones(length(t),1);

%%Control
yaw = [0]; 
pitch = [10]*pi/180; 
roll = [0];
qRef = euler_angle2_quat(pitch, roll, yaw);
quatRef(:,1) = qRef(1)*ones(length(t),1);
quatRef(:,2) = qRef(2)*ones(length(t),1);
quatRef(:,3) = qRef(3)*ones(length(t),1);
quatRef(:,4) = qRef(4)*ones(length(t),1);
euleurAnglesRef = zeros(length(t),3);

    %%out
tauXq = zeros(length(t),1);
tauYq = zeros(length(t),1);
tauZq = zeros(length(t),1);

%heading
    %%in
headingEst = zeros(length(t),1);
headingRef = zeros(length(t),1);
    %%out
yawRateEst = zeros(length(t),1);

%yaw rate
    %%in
yawRateEst = zeros(length(t),1);
yawRateRef = zeros(length(t),1);
    %%out
tauZ = zeros(length(t),1);

%angular velocity
    %%in
wEst = zeros(length(t),3);
wRef = zeros(length(t),3);
    %%out
tauXY = zeros(length(t),2);

%%z
    %%in
zRef = -10*ones(length(t),1);

zEst = zeros(length(t),1);
vzEst = zeros(length(t),1); 
	%%out
thrust = zeros(length(t),1); 

%%mix
    %%in
% tauZ = control_mix_tauZ;
ff = -17.5*ones(length(t),1);
    %%out
rpms = zeros(length(t),4);
sat = zeros(length(t),1);
finalDetailed = zeros(length(t),6);
%%
quatRef(:,2:4) = control_quat_qRef(:,1:3);
quatRef(:,1) = control_quat_qRef(:,4);
headingRef = control_heading_ref;
zRef = -control_alt_ato_ref;

quatEst(:,2:4) = control_quat_qEst(:,1:3);
quatEst(:,1) = control_quat_qEst(:,4);
headingEst(1,1) = control_heading_est(1,1);

for k=2:length(t)
    %% quaternion
    [tauXq(k,:), tauYq(k,:), tauZq(k,:)] = ...
        mex_sfx1_quaternion('step', quatRef(k,:), quatEst(k-1,:));
    %% heading
    [yawRateRef(k,:)] = ...
        mex_sfx1_heading('step',headingEst(k-1,:),headingRef(k,:));
    %% yaw
    [tauZ(k,:)] = ...
        mex_sfx1_yawRate('step',yawRateRef(k,:),yawRateEst(k-1,:));
    %% angular
    wRef(k,:) = [tauXq(k,:), tauYq(k,:), tauZq(k,:)];
    [tauXY(k,:)] = ....
        mex_sfx1_angular_rates('step',wEst(k-1,:),wRef(k,:));
    %% z
    [thrust(k,:)] = ....
        mex_sfx1_altitude('step',position(k-1,3),zRef(k,:),velocity(k-1,3));
    %% mix
    [rpms(k,:),sat(k,:),finalDetailed(k,:)] = ....
        mex_sfx1_mix('step',tauXY(k,1),tauXY(k,2),tauZ(k,:),ff(k,:),thrust(k,:));
    
    %% calling dynamic model
    [position(k,:), velocity(k,:),....
        quatEst(k,:), wEst(k,:), rpms(k,:)] = .....
        dynamical_model_update(position(k-1,:)', velocity(k-1,:)',....
        quatEst(k-1,:)', wEst(k-1,:)', rpms(k,:)'*2*pi/60, 0.005);
    
    [yaw, pitch, roll] =  quat2_euler_angle(quatEst(k,:)); 
    euleurAngles(k,1) = roll;
    euleurAngles(k,2) = pitch;
    euleurAngles(k,3) = yaw;

    headingEst(k,1) = yaw;
    
    [yaw, pitch, roll] =  quat2_euler_angle(quatRef(k,:));
    euleurAnglesRef(k,1) = roll;
    euleurAnglesRef(k,2) = pitch;
    euleurAnglesRef(k,3) = yaw;
        
    yawRateEst(k,:) = wEst(k,3);
end

%%outputs = mex_sfx1_mix('get_traces');
%% Angle plot
figure(1)
ax(1) = subplot(311);
plot(t,[euleurAnglesRef(:,1) euleurAngles(:,1)]*180/pi);grid on
legend('roll ref','angles')
ax(2) = subplot(312);
plot(t,[tauXY(:,1) wEst(:,1)]*180/pi);grid on
legend('omega ref X','omega X')
ax(3) = subplot(313);
plot(t,finalDetailed(:,1));grid on
legend('roll command')
linkaxes(ax,'x')
%% Altitude plot
figure(2)
ax(1) = subplot(311);
plot(t,[euleurAnglesRef(:,2) euleurAngles(:,2)]*180/pi);grid on
legend('pitch ref','angles')
ax(2) = subplot(312);
plot(t,[tauXY(:,2) wEst(:,2)]*180/pi);grid on
legend('omega ref Y','omega Y')
ax(3) = subplot(313);
plot(t,finalDetailed(:,2));grid on
legend('pitch command')
linkaxes(ax,'x')

%% Altitude plot
figure(3)
ax(1) = subplot(311);
plot(t,[position(:,3) zRef]);grid on
ylim([-10 50]);
legend('z est','z ref')
ax(2) = subplot(312);
plot(t,velocity(:,3));grid on
ylim([-10 50]);
legend('vz est')
ax(3) = subplot(313);
plot(t,finalDetailed(:,5:6),t,thrust);grid on
legend('final command mex','final command mex','thrust altitude control')
linkaxes(ax,'x')

%%
figure(4)
ax(1) = subplot(311);
plot(t,[headingRef(:,1) headingEst(:,1)]*180/pi);grid on
legend('yaw ref','angles')
ax(2) = subplot(312);
plot(t,[yawRateRef yawRateEst]*180/pi);grid on
legend('omega ref Z','omega Z')
ax(3) = subplot(313);
plot(t,tauZ);
hold on
plot(t,finalDetailed(:,3),t,finalDetailed(:,4));grid on
legend('tauZ','yaw command')
linkaxes(ax,'x')

%%
% figure(6)
% plot(t,rpms)