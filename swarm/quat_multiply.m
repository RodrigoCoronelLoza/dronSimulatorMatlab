function [q] = quat_multiply(q1,q2)

q(1,1) = q2(1)*q1(1)-q2(2)*q1(2)-q2(3)*q1(3)-q2(4)*q1(4);
q(2,1) = q2(1)*q1(2)+q2(2)*q1(1)-q2(3)*q1(4)+q2(4)*q1(3);
q(3,1) = q2(1)*q1(3)+q2(2)*q1(4)+q2(3)*q1(1)-q2(4)*q1(2);
q(4,1) = -q2(1)*q1(4)+q2(2)*q1(3)+q2(3)*q1(2)+q2(4)*q1(1);

if norm(q)>0
    q = q/norm(q);
end