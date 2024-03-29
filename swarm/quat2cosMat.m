function [n] = quat2cosMat(q)

n = zeros(3,3);

q = q./norm(q);

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

n(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
n(1,2) = 2 * (q1*q2 + q0*q3);
n(1,3) = 2 * (q1*q3 - q0*q2);
n(2,1) = 2 * (q1*q2 - q0*q3);
n(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
n(2,3) = 2 * (q2*q3 + q0*q1);
n(3,1) = 2 * (q1*q3 + q0*q2);
n(3,2) = 2 * (q2*q3 - q0*q1);
n(3,3) = q0^2 - q1^2 - q2^2 + q3^2;