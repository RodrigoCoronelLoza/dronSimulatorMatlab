clear all
close all
clc
%%
line_sfx1_mix;
%% Load dataset
load('~/Documents/MATLAB/windEstimatorTest/EX-00-07397_0178.mat'); % Load your dataset
bindEstimation = '';
t = (eval([bindEstimation 'controllers_timestamp'])-eval([bindEstimation 'controllers_timestamp(1)']))/1000000; % time vector [s]
%%indices
indices = 1:length(t);
%      if (nrhs != 6) { throw std::invalid_argument("Not the right arguments (tauX, tauY, tauZ, ff, altitude)"); }
%      if (nlhs != 3) { throw std::invalid_argument("Not the right returned variables (rpms, bool, finalDetailed)"); }
%%
%%%inputs
tauX = control_mix_tauX;
tauY = control_mix_tauY;
tauZ = control_mix_tauZ;
ff = control_mix_feedForward;
altitude = control_mix_alt;
%%%outputs
rpms = zeros(length(t),4);
sat = zeros(length(t),1);
finalDetailed = zeros(length(t),6);

for k=1:length(t)

     [rpms(k,:),sat(k,:),finalDetailed(k,:)] = ....
         mex_sfx1_mix('step',tauX(k,:),tauY(k,:),tauZ(k,:),ff(k,:),altitude(k,:));
    
end

outputs = mex_sfx1_mix('get_traces');
%%
figure(1)
ax(1) = subplot(311);
plot(t,[rpms],t,control_mix_rpm,'-.');grid on
legend('rpm mex','rpm online')
ax(2) = subplot(312);   
plot(t,[sat],t,control_mix_saturated);grid on
legend('rpm mex','rpm online')
legend('P','I','D','sum')
ax(3) = subplot(313);
plot(t,[finalDetailed(:,5:6)],t,[control_mix_finalCommandDetailed(:,5:6)],'-.',t,finalDetailed(:,5)+finalDetailed(:,6));grid on
legend('final command mex','final command online')
linkaxes(ax,'x')

figure(2)
plot(t,[control_alt_ato_ref control_alt_ato_est])

figure(3)
plot(outputs.Tff0001.timestamps/200,outputs.Tff0001.values)