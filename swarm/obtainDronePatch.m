clear all close all clc
figure 
%Get the bmp file
uiopen('/home/plongobardi/StageDocuments/(2) OBSTACLE AVOIDANCE TASK/index.png',1)
%uiopen('/home/plongobardi/StageDocuments/(2) OBSTACLE AVOIDANCE TASK/drone.bmp',1)
%%
drone = cdata;
drone = drone(:,:,1);
[r,c]=find(drone<=150 );
drone_x = c-mean(c);
drone_y = -r+mean(r);
scalingFactor = 100;
drone_x = drone_x/scalingFactor;
drone_y = drone_y/scalingFactor;
scatter(drone_x,drone_y,'.','k')