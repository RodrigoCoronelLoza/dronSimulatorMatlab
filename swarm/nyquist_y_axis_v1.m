Kp_y=0.24;
Kp_int_y=1;
Ti_y=10;
g=9.81;
C_x=0.28;

K1= tf ([Kp_y],[1])
K2= tf ([Kp_int_y],[Ti_y 0])
G= tf ([g],[1 C_x])

K=parallel (K1,K2)
K=minreal(K)
H=series (K,G)
H= minreal (H)

% T= H/(1+H)
% minreal (T)

[Gm,Pm,Wgm,Wpm] = margin(H)
margin(H)
T=feedback (H,1);
nyquist(H);

figure
margin (H)