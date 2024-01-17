Kp_z=6;
Kp_der_z=1;
Td_z=12;
g=9.81;
C_x=0.25;
m=1.5;
C=1.15;
N=10;


K_pos=1.2950;
K_vel=-4.2775;
K_int=1;
Ti=150;

Kproppos= tf ([(-(K_pos))],[1 0])
Kpropvel= tf ([K_vel],[1])
Kint= tf ([-K_int],[Ti 0 0])
G= tf (-[1/m],[1 C/m]);
% G=minreal(G)

K_intermedio=parallel(Kproppos,Kpropvel);
K=parallel(K_intermedio,Kint);

% ff= tf ([1 C/m],-[1/m])
% ff=minreal(ff);
% K=parallel (K,ff);
K=minreal (K)

Te=0.051
ret = tf([1],[1],'Inputdelay',Te)

H=series (K,G)
% H_int=series (ff,G)
% H=parallel (H_int,H)
H= minreal (H)
H= series (H,ret)

% T= H/(1+H)
% minreal (T)

[Gm,Pm,Wgm,Wpm] = margin(H)

% T=feedback (H,1)
nyquist(H)


axes('position',[.65 .70 .20 .20])
box on % put box around new pair of axes

nyquist(H)
% axis tight
ylim ([-20 20])
xlim ([-4 4])
figure
margin (H)