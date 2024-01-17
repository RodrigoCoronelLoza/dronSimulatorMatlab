function [x,v,q,w,W] = dynamical_model_update(x, v, q, w, W, timeStep)

% x = drone position
% v = drone velocity
% q = quaternions
% w = angular speed
% W = motor speed

%% Drone data
format long;
mass = 17.2/9.81; % kg

inertiaMatrix = [0.013076,  0,          0.00002; ...
                 0,         0.03561,    -0.000052; ...
                 0.00002,   -0.000052,  0.023767]; 

invInertiaMatrix = inv(inertiaMatrix);

Tr = 0.04;

% Centers of gravity of motors
d1 = 0.131;
d2 = 0.131;
d3 = 0.25029;
d4 = 0.18029;

k = 0.0000134;
kt = 0.00000022;

P = [k*d3,  k*d4, -k*d4, -k*d3 ; ...
     k*d1, -k*d2, -k*d2,  k*d1 ; ...
     -kt,   kt,   -kt,    kt] ;

rotorForceCoeff = k;

 % Motor Reference Speed
Wcons = [0;0;0;0];

%% Physical constants
physics_G = 9.81;

%% External parameters
wind = zeros(3,1);

% Motor saturation
Wsat = 8000*2*pi/60;
for i=1:4
    if W(i) > Wsat
        W(i) = Wsat;
    end
end

%% Runge Kutta calculation

x0 = x;
v0 = v;
q0 = q;
w0 = w;
W0 = W;

for i=1:4
    
    if (i==2 || i==3)
        x = x0 + 0.5*timeStep*kx(:,i-1);
        v = v0 + 0.5*timeStep*kv(:,i-1);
        q = q0 + 0.5*timeStep*kq(:,i-1);
        w = w0 + 0.5*timeStep*kw(:,i-1);
        W = W0 + 0.5*timeStep*kW(:,i-1);
    elseif i==4
        x = x0 + timeStep*kx(:,i-1);
        v = v0 + timeStep*kv(:,i-1);
        q = q0 + timeStep*kq(:,i-1);
        w = w0 + timeStep*kw(:,i-1);
        W = W0 + timeStep*kW(:,i-1);
    end
    
    % Normalise quaternion
    q = q/norm(q);
    %% Calculate the derivative
    
    vb = (quat2cosMat(q'))'*v;
    
    % dx/dt = v
    dxdt = v + wind;
    
    % dv/dt = g ez - T/m R ez
    ez = [0; 0; 0; 1];
    T = rotorForceCoeff * sum(abs(W.^2));
    
    thrust  = quat2cosMat(q')*[0;0;-T/mass];
    dragForceBody = [-0.35*vb(1) ; -0.28*vb(2) ; -vb(3)];
    dragForceNed = quat2cosMat(q')*dragForceBody;
    
    dvdt = dragForceNed(1:3,:) + thrust;
    dvdt(3) = dvdt(3) + physics_G;
    
    % dw/dt = If^(-1) [ -w ^ (If w) - Ga + Ta ]
    dwdt = invInertiaMatrix *( cross(-w,(inertiaMatrix * w)) + P * abs(W.^2));
    dwdt(3) = dwdt(3) - w(3);
    
    % dq/dt = 0.5 q w
    wq =[0 ; w];
    if norm(wq)>0
        wq = wq/norm(wq);
    end
    
    dqdt = 0.5 * quat_multiply(q,wq);
    % dW/dt = (1/Tr)*(W - Wc)
    dWdt = 0;% -(W - Wcons) / Tr;
    
    kx(:,i) = dxdt;
    kv(:,i) = dvdt;
    kq(:,i) = dqdt;
    kw(:,i) = dwdt;
    kW(:,i) = dWdt;

end
%% Update values

x = x0 + 1/6* (kx(:,1)+2*kx(:,2)+2*kx(:,3)+kx(:,4)) * timeStep;
v = v0 + 1/6* (kv(:,1)+2*kv(:,2)+2*kv(:,3)+kv(:,4)) * timeStep;
q = q0 + 1/6* (kq(:,1)+2*kq(:,2)+2*kq(:,3)+kq(:,4)) * timeStep;
w = w0 + 1/6* (kw(:,1)+2*kw(:,2)+2*kw(:,3)+kw(:,4)) * timeStep;
W = W0 + 1/6* (kW(:,1)+2*kW(:,2)+2*kW(:,3)+kW(:,4)) * timeStep;

q = q/norm(q);


