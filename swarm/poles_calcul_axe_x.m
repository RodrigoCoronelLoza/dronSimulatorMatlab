% close all
% clear all
% clc
Ts=0.063; % [s] sampling period
%%%%%%%%%%%%%%%%%1 kpy=0.12 Ti=500
% Tset= 5.8546e5*1e-3;    % [s] settling time
% Ovshoot=0.139352;    % overshoot

%%%%%%%%%%%%%%%%%2 kpy=0.24 Ti=250

%  Tset= 3.3913e5*1e-3;   % [s] settling time
%  Ovshoot=0.219297;    % overshoot
 
%  Tset= 3.43;   % [s] settling time
%  Ovshoot=0.065382;    % overshoot
 
 Tset= 3;   % [s] settling time
 Ovshoot=0.102698; 

%%%%%%%%%%%%%%%%%%%%%%%%%%3 kpy=0.3 Ti=200

% Tset= 3.0070e5*1e-3;    % [s] settling time
% Ovshoot=0.230933;    % overshoot
% 



%Damping factor

Damp_f=-(log(Ovshoot))/(sqrt(pi^2+((log(Ovshoot))^2)));
Wn=4/(Damp_f*Tset);

Wn*Ts*sqrt(1-Damp_f^2);
p1=-2*exp(-Damp_f*Wn*Ts)*cos(Wn*Ts*sqrt(1-Damp_f^2));
p2=exp(-2*Damp_f*Wn*Ts);

a=1;
b=p1;
c=p2;

z1=-(b + (b^2 - 4*a*c)^(1/2))/(2*a);
z2=-(b - (b^2 - 4*a*c)^(1/2))/(2*a);

C=0.25;

% A= [ 0 1 0;0 -C/M 0;0 0 -C/M];
% B= [ 0;-1/M;-1/M];
% p= [z1 z2 -1];
% 
% Co= ctrb (A,B)
% 
% rank(Co)
A= [ 0 1 ;0 -C];
B= [ 0;9.81];
p= [z1 z2 ];

Co= ctrb (A,B);

rank(Co);

K=place(A,B,p)



