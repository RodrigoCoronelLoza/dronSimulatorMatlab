function [yaw pitch roll] = quat2_euler_angle(q)
    
sqvx = q(2)*q(2);
sqvy = q(3)*q(3);
sqvz = q(4)*q(4);

roll = real(atan2(2*q(3)*q(4) + 2*q(1)*q(2), 1 - 2*sqvx - 2*sqvy));
pitch = real(asin(2*(q(1)*q(3) - q(2)*q(4))));
yaw = real(atan2(2*q(2)*q(3) + 2*q(1)*q(4), 1 - 2*sqvy - 2*sqvz));
