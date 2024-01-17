
function [dist_1,dist_2,dist_3,dist_4,dist_5,dist_6] = distance_UWB(anclas,pos_drone_x,pos_drone_y,pos_drone_z)


% DEFINICION DE LAS POSICIONES DE LAS ANCLAS DE 
% % % % theta_drone = droneState.currentState.eulerAnglesRad.theta;
% % % % phi_drone = droneState.currentState.eulerAnglesRad.phi;

% pos1=anclas.a1
% pos2=anclas.a2
% pos3=anclas.a3
% pos4=anclas.a4
% pos5=anclas.a5
% pos6=anclas.a6
posicion_drone = [pos_drone_x; pos_drone_y; pos_drone_z];
error_lim_inf=-0.1;
error_lim_sup=0.1;
% % % % 
% % % % vect_decalage_front=[0  + constants.us1Y;...
% % % %                  0  + constants.us1X;...
% % % %                  -constants.us1Z];
% % % % 
% % % % vect_decalage_bottom=[constants.us2Y;constants.us2X;-constants.us2Z];
% % % % 
% % % % vect_decalage_left=[0 + constants.us3Y;...
% % % %                     0 + constants.us3X;...
% % % %                     -constants.us3Z];
% % % % vect_decalage_right=[0 + constants.us4Y;...
% % % %                      0 + constants.us4X;...
% % % %                      -constants.us4Z];
% % % %                 
% % % % vect_decalage_back=[0 + constants.us5Y;...
% % % %                     0 + constants.us5X;...
% % % %                     -constants.us5Z];
% % % %                    
% % % % direction_to_measure=[1,15,2];%change also in the 3D controller
% % % % direction_to_measure=direction_to_measure/norm(direction_to_measure);
% % % % 
dist_min_detected=0;
dist_max_detected=100;
% % % % 
% % % % ang=pi/4;
% % % % 
% % % % %! The directions are expressed in the drone frame
% % % % dir_front=[1;0;0];
% % % % dir_back=[-1;0;0];
% % % % dir_left= [cos(ang);-sin(ang);0];
% % % % dir_right=[cos(ang);sin(ang);0];
% % % % dir_bottom=[0;0;1];
% % % % dir_up=[0;0;-1];
% % % % dir_rside=[0;1;0];
% % % % 
% % % % R = zeros(3,3) ;
% % % %             cpsi = cos(psi);
% % % %             spsi = sin(psi);
% % % %             cth = cos(theta);
% % % %             sth = sin(theta);
% % % %             cph = cos(phi);
% % % %             sph = sin(phi);
% % % %            
% % % %             R(1,1) = cpsi*cth;
% % % %             R(1,2) = cpsi*sph*sth-spsi*cph;
% % % %             R(1,3) = cpsi*sth*cph+spsi*sph;
% % % %             R(2,1) = spsi*cth;
% % % %             R(2,2) = spsi*sth*sph+cpsi*cph;
% % % %             R(2,3) = spsi*sth*cph-cpsi*sph;
% % % %             R(3,1) = -sth ;
% % % %             R(3,2) = cth*sph ;
% % % %             R(3,3) = cth*cph ;

% % % % if (psi~=0 || theta~=0 ||phi~=0)
% % % % %     gol=1
% % % % dir_front= R*dir_front;
% % % % dir_back=R*dir_back;
% % % % dir_left=R*dir_left;
% % % % dir_right=R*dir_right;
% % % % dir_bottom=R*dir_bottom;
% % % % dir_up=R*dir_up;
% % % % dir_rside=R*dir_rside;
% % % % vect_decalage_front=R*vect_decalage_front;
% % % % vect_decalage_bottom=R*vect_decalage_bottom;
% % % % vect_decalage_left=R*vect_decalage_left;
% % % % vect_decalage_right=R*vect_decalage_right;
% % % % vect_decalage_back=R*vect_decalage_back;
% % % % end

% vect_decalage_front

% % % % dir_front= dir_front/norm(dir_front); %VECTORES EN EL REPERE DEL MUNDO
% % % % dir_back=dir_back/norm(dir_back);
% % % % dir_left=dir_left/norm(dir_left);
% % % % dir_right=dir_right/norm(dir_right);
% % % % dir_bottom=dir_bottom/norm(dir_bottom);
% % % % dir_up=dir_up/norm(dir_up);
% % % % dir_rside=dir_rside/norm(dir_rside);




% %%%%%%%%%%%%%%%%%%%%front direction
% clave=1
% dir_front

% en_x_solo_pos=pos_drone_x
% en_y_solo_pos=pos_drone_y
% en_z_solo_pos=pos_drone_z
% 
% en_x=pos_drone_x+vect_decalage_front(1)
% en_y=pos_drone_y+vect_decalage_front(2)
% en_z=pos_drone_z+vect_decalage_front(3)

%%%%% surface_following 1

% [dist_front,theta_head,phi_head,normal_surface_vector,yaw_value] = measurement_front(obstacul,i,j,k,l,m,n,theta_drone,phi_drone,...
%                                dir_front,dir_up,...
%                                dir_rside,pos_drone_x+vect_decalage_front(1),...
%                                pos_drone_y+vect_decalage_front(2),...
%                                pos_drone_z+vect_decalage_front(3));
%                         
vector_d1=posicion_drone-anclas.a1;
dist_1=norm(vector_d1);
% error_lim_inf+(error_lim_sup-error_lim_inf)*rand
dist_1=dist_1+(error_lim_inf+(error_lim_sup-error_lim_inf)*rand);
% theta_head=0;
% phi_head=0;
% normal_surface_vector= [1;0;0];
% yaw_value=0;
%%%%%distance_lock
% [dist_front,theta_head,phi_head,normal_surface_vector,yaw_value] = measurement_front_direction(obstacul,i,j,k,l,m,theta_drone,phi_drone,...
%                                dir_front,dir_up,...
%                                dir_rside,direction_to_measure,pos_drone_x+vect_decalage_front(1),...
%                                pos_drone_y+vect_decalage_front(2),...
%                                pos_drone_z+vect_decalage_front(3));                         
%                            
%%%%%%                  
% dist_front = measurement(obstacul,i,j,k,l,dir_front(1),dir_front(2),dir_front(3),...
%                          pos_drone_x+vect_decalage_front(1),pos_drone_y+vect_decalage_front(2),...
%                          pos_drone_z+vect_decalage_front(3));

if (dist_1 < dist_min_detected)
    dist_1=dist_min_detected;
end
if (dist_1 > dist_max_detected)
    dist_1=dist_max_detected;
end     

%%%%%%%%%%%%%%%%%%%% back direction
% dist_back = measurement(obstacul,i,j,k,l,dir_back(1),dir_back(2),dir_back(3),...
%                          pos_drone_x+vect_decalage_back(1),pos_drone_y+vect_decalage_back(2),...
%                          pos_drone_z+vect_decalage_back(3));
vector_d2=posicion_drone-anclas.a2;
dist_2=norm(vector_d2);
dist_2=dist_2+(error_lim_inf+(error_lim_sup-error_lim_inf)*rand);
if (dist_2 < dist_min_detected)
    dist_2=dist_min_detected;
end
if (dist_2 > dist_max_detected)
    dist_2=dist_max_detected;
end     

% %%%%%%%%%%%%%%%%%%%%left

% dist_left = measurement(obstacul,i,j,k,l,dir_left(1),dir_left(2),dir_left(3),...
%                          pos_drone_x+vect_decalage_left(1),pos_drone_y+vect_decalage_left(2),...
%                          pos_drone_z+vect_decalage_left(3));

vector_d3=posicion_drone-anclas.a3;
dist_3=norm(vector_d3);
dist_3=dist_3+(error_lim_inf+(error_lim_sup-error_lim_inf)*rand);

if (dist_3 < dist_min_detected)
    dist_3=dist_min_detected;
end
if (dist_3 > dist_max_detected)
    dist_3=dist_max_detected;
end

%%%%%%%%%%%%%%%%%%%%%%%% right

% dist_right = measurement(obstacul,i,j,k,l,dir_right(1),dir_right(2),dir_right(3),...
%                          pos_drone_x+vect_decalage_right(1),pos_drone_y+vect_decalage_right(2),...
%                          pos_drone_z+vect_decalage_right(3));
vector_d4=posicion_drone-anclas.a4;
dist_4=norm(vector_d4);
dist_4=dist_4+(error_lim_inf+(error_lim_sup-error_lim_inf)*rand);

if (dist_4 < dist_min_detected)
    dist_4=dist_min_detected;
end
if (dist_4 > dist_max_detected)
    dist_4=dist_max_detected;
end


%%%%%%%%%%%%%%%%%%%%%%%% bottom
%dir_bottom
% dist_bottom = measurement(obstacul,i,j,k,l,dir_bottom(1),dir_bottom(2),dir_bottom(3),...
%                          pos_drone_x+vect_decalage_bottom(1),pos_drone_y+vect_decalage_bottom(2),...
%                          pos_drone_z+vect_decalage_bottom(3));
vector_d5=posicion_drone-anclas.a5;
dist_5=norm(vector_d5);
dist_5=dist_5+(error_lim_inf+(error_lim_sup-error_lim_inf)*rand);

if (dist_5 < dist_min_detected)
    dist_5=dist_min_detected;
end
if (dist_5 > dist_max_detected)
    dist_5=dist_max_detected;
end

vector_d6=posicion_drone-anclas.a6;
dist_6=norm(vector_d6);
dist_6=dist_6+(error_lim_inf+(error_lim_sup-error_lim_inf)*rand);

if (dist_6 < dist_min_detected)
    dist_6=dist_min_detected;
end
if (dist_6 > dist_max_detected)
    dist_6=dist_max_detected;
end
end