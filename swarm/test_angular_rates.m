clear all
close all
clc
%%
line_sfx1_angular_rates;
%% Load dataset
load('~/Documents/MATLAB/windEstimatorTest/EX-00-07397_0178.mat'); % Load your dataset
bindEstimation = '';
t = (eval([bindEstimation 'controllers_timestamp'])-eval([bindEstimation 'controllers_timestamp(1)']))/1000000; % time vector [s]
%%indices
indices = 1:length(t);
%%
%%%inputs
wEst = zeros(length(t),4);
wRef = ones(length(t),4);

%%%outputs
tauXY = zeros(length(t),2);

for k=1:length(t)

     [tauXY(k,:)] = ....
         mex_sfx1_angular_rates('step',wEst(k,:),wRef(k,:));
    
end

outputs = mex_sfx1_angular_rates('get_traces');
%%
% figure(1)
% ax(1) = subplot(311);
% plot(t,[rpms],t,control_mix_rpm,'-.');grid on
% legend('rpm mex','rpm online')
% ax(2) = subplot(312);   
% plot(t,[sat],t,control_mix_saturated);grid on
% legend('rpm mex','rpm online')
% legend('P','I','D','sum')
% ax(3) = subplot(313);
% plot(t,[finalDetailed(:,5:6)],t,[control_mix_finalCommandDetailed(:,5:6)],'-.',t,finalDetailed(:,5)+finalDetailed(:,6));grid on
% legend('final command mex','final command online')
% linkaxes(ax,'x')
% 
% figure(2)
% plot(t,[control_alt_ato_ref control_alt_ato_est])
% 
% figure(3)
% plot(outputs.tx.timestamps/200,outputs.Tff0001.values)