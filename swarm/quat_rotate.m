function [vRot] = quat_rotate(q,v)

% Quaternion components
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% Vector components
v1 = v(1);
v2 = v(2);
v3 = v(3);

rotationMatrix = [  1-2*q2^2-2*q3^2     2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2); ...
                    2*(q1*q2-q0*q3)     1-2*q1^2-2*q3^2     2*(q2*q3+q0*q1); ...
                    2*(q1*q3+q0*q2)     2*(q2*q3-q0*q1)     1-2*q1^2-2*q2^2 ];
                

vRot = rotationMatrix*v;
