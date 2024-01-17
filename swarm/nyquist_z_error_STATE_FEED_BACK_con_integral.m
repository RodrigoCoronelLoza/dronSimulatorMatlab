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

Kproppos= tf ([K_pos],[1])
Kpropvel= tf ([K_vel],[1])
Kint= tf ([K_int],[Ti 0])
G_alpha= tf ([-1/m],[1 C/m 0]);
G_beta= tf ([-1/m],[1 C/m]);



K_beta= Kpropvel;
K_alpha=parallel(Kproppos,Kint);

K=parallel(K_intermedio,Kint);
K=minreal (K)

H=series (K,G)
H= minreal (H)
% T= H/(1+H)
% minreal (T)

[Gm,Pm,Wgm,Wpm] = margin(H)

% T=feedback (H,1)
nyquist(H)