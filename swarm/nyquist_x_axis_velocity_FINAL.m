Kp_x_pos=-0.0862;
Kp_x_vel=0.2117;
Ti_y=10;
g=9.81;
C_x=0.35;

K1= tf ([-Kp_x_pos],[1 0])
K2= tf ([Kp_x_vel],[1])
G= tf ([g],[1 C_x])
Te=0.051
ret = tf([1],[1],'Inputdelay',Te)

K=parallel (K1,K2)
K=minreal(K)
H=series (K,G)
H= minreal (H)
H= series (ret,H)
H= minreal (H)

% T= H/(1+H)
% minreal (T)

[Gm,Pm,Wgm,Wpm] = margin(H)
margin(H)
T=feedback (H,1);
nyquist(H);

figure
margin (H)