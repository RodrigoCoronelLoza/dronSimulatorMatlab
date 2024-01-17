Kp_z=6;
Kp_der_z=1;
Td_z=12;
g=9.81;
C_x=0.25;
m=1.5;
C=1;
N=10;

K1= tf ([Kp_z],[1])
K2= tf ([Kp_der_z*Td_z 0 0],[Td_z/N 1])
G= tf ([1/m],[1 C])

K=parallel (K1,K2)
H=series (K,G)

% T= H/(1+H)
 minreal (H)
[Gm,Pm,Wgm,Wpm] = margin(H)

T=feedback (H,1)
nyquist(H)