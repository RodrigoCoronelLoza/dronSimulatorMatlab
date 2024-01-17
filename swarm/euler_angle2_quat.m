function [q] = euler_angle2_quat(pitch, roll, yaw)

spitch = sin(0.5*pitch);
cpitch = cos(0.5*pitch);
sroll = sin(0.5*roll);
croll = cos(0.5*roll);
syaw = sin(0.5*yaw);
cyaw = cos(0.5*yaw);

w = croll*cpitch*cyaw + sroll*spitch*syaw;
x = sroll*cpitch*cyaw - croll*spitch*syaw;
y = croll*spitch*cyaw + sroll*cpitch*syaw;
z = croll*cpitch*syaw - sroll*spitch*cyaw;

q = [w; x; y; z];