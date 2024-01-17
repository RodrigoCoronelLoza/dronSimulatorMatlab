% clear all
% close all
% clc
%%
line_sfx1_heading;
%% Load dataset
load('~/Data/Mat_files/EX-00-07397_0397.mat'); % Load your dataset % 178
bindEstimation = '';
t = (eval([bindEstimation 'controllers_timestamp'])-eval([bindEstimation 'controllers_timestamp(1)']))/1000000; % time vector [s]
%indices
indices = 1:length(t);
%
%%inputs
headingEst = control_heading_est;
headingRef = control_heading_ref;

%%%outputs
yawRate = zeros(length(t),1);

for k=1:length(t)

     [yawRate(k,:)] = ....
         mex_sfx1_heading('step',headingEst(k,:),headingRef(k,:));
    
end

outputs = mex_sfx1_heading('get_traces');
%%
figure()
ax(1) = subplot(211);
plot(t, [headingRef headingEst]);
legend('heading ref','heading est');
ax(2) = subplot(212);
plot(t,yawRate,t,control_yawRate_ref,'-.');grid on
legend('yawRate mex','yawRate online')
linkaxes(ax,'x');
