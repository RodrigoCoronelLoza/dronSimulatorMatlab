% comp_1=planeFitting_planeNormal(:,1);
% comp_2=planeFitting_planeNormal(:,2);
% comp_3=planeFitting_planeNormal(:,3);
% 
% mean_1=  mean(comp_1)
% mean_2=  mean(comp_2)
% mean_3=  mean(comp_3)
% 
% stad_dev_1= std(comp_1)
% stad_dev_2= std(comp_2)
% stad_dev_3= std(comp_3)
% 
% mad_1= mad(comp_1)
% mad_2= mad(comp_2)
% mad_3= mad(comp_3)
bindEstimation = '';
t_controllers = (eval([bindEstimation 'controllers_timestamp'])-eval([bindEstimation 'controllers_timestamp(1)']))/1000000; % time vector [s]
t_estimation = (eval([bindEstimation 'estimation_timestamp'])-eval([bindEstimation 'estimation_timestamp(1)']))/1000000; % time vector [s]

subplot (2,1,1)
plot (t_controllers,state_pitch)

subplot (2,1,2)
plot (t_controllers(2604:2984),state_pitch(2604:2984))

% figure
% 
% plot (t_controllers(2604:2984),state_pitch(2604:2984))
% 
% figure
% % subplot (2,1,1)
% plot (1:length(state_pitch),state_pitch)

% subplot (2,1,2)
% plot (1:length(state_pitch),state_pitch)
figure

% % plot (1:length(state_speed_body_x),state_speed_body_x)



% subplot (2,1,1)

ref=-25;
referencia=zeros(1,2985-2604);
unos=ref*ones(1,2985-2604);
for i=1:length(referencia)
    if(i<48)
        gran_referencia(i)=referencia(i);
    else
        gran_referencia(i)=unos(i);
    end
end




% plot (1:1:(1948-1710),state_pitch(1710:1:1947))
plot (t_controllers(2604:2984),state_pitch(2604:2984))
ylabel ('pitch value [°]')
xlabel ('time')
grid on
hold on
plot (t_controllers(2604:2984),gran_referencia)


ref=-25;
referencia=zeros(1,2686-2644);
unos=ref*ones(1,2686-2644);
for i=1:length(referencia)
    if(i<=8)
        gran_referencia_2(i)=referencia(i);
    else
        gran_referencia_2(i)=unos(i);
    end
end

figure
plot (t_controllers(2644:2685),state_pitch(2644:2685))
ylabel ('pitch value [°]')
xlabel ('time')
grid on
% figure
hold on
% plot (t_controllers(2644:2685),gran_referencia_2,'*')
syms x
fplot(@(x)(-25)*heaviside((x-133.4)), [133 135.5])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j=1:(2686-2644)
tiempo_clave(j)=t_controllers(j+2644)-t_controllers(2644);
end

figure
plot (tiempo_clave,state_pitch(2644:2685))
ylabel ('pitch value [°]')
xlabel ('time [s]')
grid on
hold on
% plot (tiempo_clave,gran_referencia_2)
syms z
fplot(@(z)(-25)*heaviside((z-0.45)), [0 2])

% % subplot (2,1,2)
% hold on
% plot (1:1:(1948-1710),state_pitch(1710:1:1947))
% ylabel ('vitesse u [m/s]')

% subplot (2,1,1)
% plot (1:1:(1948-1710),state_pitch(1710:1:1947))
% ylabel ('pitch value [°]')
% xlabel ('time')
% grid on
% subplot (2,1,2)
% plot (1:1:(1948-1710),state_speed_body_x(1710:1:1947))
% ylabel ('vitesse u [m/s]')
% 
% hold on
% plot (1:length(transpose(comp_3)),transpose(comp_3))

% xlabel ('time')
% grid on
% ylabel ('value of the component')
% legend ('comp_1','comp_2','comp_3')