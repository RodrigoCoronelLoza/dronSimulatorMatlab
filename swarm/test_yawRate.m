clear all
close all
clc
%%
line_sfx1_yawRate;
%% Load dataset
load('~/Data/Mat_files/EX-00-07397_0400.mat'); % Load your dataset
bindEstimation = '';
t = (eval([bindEstimation 'controllers_timestamp'])-eval([bindEstimation 'controllers_timestamp(1)']))/1000000; % time vector [s]
%%indices
indices = 1:length(t);
%%
%%%inputs
yawRateEst = control_yawRate_est;
yawRateRef = control_yawRate_ref;

%%%outputs
tauZ = zeros(length(t),1);

for k=1:length(t)

     [tauZ(k,:)] = ....
         mex_sfx1_yawRate('step',yawRateRef(k,:),yawRateEst(k,:));
    
end

outputs = mex_sfx1_yawRate('get_traces');
%%
figure(1)
plot(t,tauZ,t,control_mix_tauZ,'-.');grid on
legend('tauZ mex','tauZ online')

% figure(2)
% plot(t,[control_alt_ato_ref control_alt_ato_est])
