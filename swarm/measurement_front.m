function[estimated_distance,theta_head,phi_head,normal_surface_vector,yaw_value]= measurement_front(obstacul,i,j,k,l,m,n,theta_drone,phi_drone,dir_front,dir_up,dir_rside,pos_drone_x,pos_drone_y,pos_drone_z)
% gol=1
yaw_value=0;
theta_head=100;
phi_head=100;
normal_surface_vector=[0;0;0];
angle_limit=27.5*(pi/180);
estimated_distance=100000;
%dist_back=100000;
%d_1=0;
%d_3=0;
d_1=100000;
%d_3=100000;
%prim_comp=dir_up(1)

%%%%cylindre
for u=1:i
    
    height=obstacul.cilindre(u).height;
    radius=obstacul.cilindre(u).radius;
    center_x=obstacul.cilindre(u).center_x;
    center_y=obstacul.cilindre(u).center_y;
    center_z=obstacul.cilindre(u).center_z;
    % if (pos_drone_z<center_z+height/2 && pos_drone_z >center_z-height/2)
    %tic
    % [t]= solve((x-center_x)^2+(y-center_y)^2==radius^2);
    %toc
    %tic
    %%%%%%%%%%%%%%%%%%%%%%%%%%5
    normal_surface=[center_x-pos_drone_x;center_y-pos_drone_y;center_z-pos_drone_z];%interior
    normal_surface=normal_surface/norm(normal_surface);
    
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if ((pos_drone_x-center_x)^2+(pos_drone_y-center_y)^2<=radius^2 ) %gauche
        dir_normal=-normal_surface;
        normal_surface=-normal_surface;
    else
        dir_normal=normal_surface;%interior
    end
    
    %     normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    %     normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal(1);dir_normal(2)];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    yaw_value=angle_1;
    
    
    %%%%%%%%%%%%%%%5555
    
    normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
    normal_plan_1=normal_plan_1/norm(normal_plan_1);
    normal_surface=normal_surface/norm(normal_surface);
    dot_product_1=dot(normal_plan_1,normal_surface);
    %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
    %             dot(normal_plan_1,normal_surface));
    if (dot_product_1<=0.02 && dot_product_1>=-0.02)
        
        %             GOL=1
        %solo calculo de theta
        %normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov= wrapToPi(atan2(norm(cross(dir_front,normal_surface)),...
            dot(dir_front,normal_surface)));
        phi_head_prov=0;
    else
        %calculo de los dos angulos
        phi_head_prov= wrapToPi(atan2(norm(cross(dir_up,-normal_surface)),...
            dot(dir_up,-normal_surface)));
        
        normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
            dot(dir_front,normal_plan_2)));
    end
    
    %%%%%%%%%%%%%%%%%%%%%
    alpha=pos_drone_x-center_x;
    beta=pos_drone_y-center_y;
    a=dir_normal(1)^2+dir_normal(2)^2;
    b=(2*(alpha)*dir_normal(1))+(2*(beta)*dir_normal(2));
    c=alpha^2+beta^2-radius^2;
    %toc
    % t_1=double(t(1))
    % t_2=double(t(2))
    t_1=((-b+sqrt(b*b-4*a*c))/2*a);
    t_2=((-b-sqrt(b*b-4*a*c))/2*a);
    
    if (isreal(t_1)&& isreal(t_2)&&((t_1>0)||(t_2>0)))
        if(t_1>=0 && t_2>=0)
            if(t_1<t_2)
                t_v=t_1;
            else
                t_v=t_2;
            end
            x_v=pos_drone_x+t_v*dir_normal(1);
            y_v=pos_drone_y+t_v*dir_normal(2);
            z_v=pos_drone_z+t_v*dir_normal(3);
            vect_2=-[dir_normal(1);dir_normal(2);dir_normal(3)];
        end
        if(t_1*t_2<0)
            if(t_1>0)
                t_v=t_1;
            end
            if(t_2>0)
                t_v=t_2;
            end
            x_v=pos_drone_x+t_v*dir_normal(1);
            y_v=pos_drone_y+t_v*dir_normal(2);
            z_v=pos_drone_z+t_v*dir_normal(3);
            vect_2=[dir_normal(1);dir_normal(2);dir_normal(3)];
        end
        
        vect_1=[2*(x_v-center_x);2*(y_v-center_y);0];
        vect_1=vect_1/norm(vect_1);
        
        % vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if (z_v<center_z+height/2 && z_v>center_z-height/2 &&(angle_bet<angle_limit)&&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
                theta_head=theta_head_prov;
                phi_head=phi_head_prov;
                normal_surface_vector=normal_surface;
            end
        end
    end
    
    
    
    % t_2=((-b-sqrt(b*b-4*a*c))/2*a);
    % x_2=pos_drone_x+t_2*dir_front(1);
    % y_2=pos_drone_y+t_2*dir_front(2);
    % z_2=pos_drone_z+t_2*dir_front(3);
    %
    % vect_1=[2*(x_2-center_x);2*(y_2-center_y);0];
    % vect_1=vect_1/norm(vect_1);
    % vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
    % angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
    %
    % if (z_2<center_z+height/2 && z_2>center_z-height/2 &&(angle_bet<angle_limit)...
    %         &&isreal(t_2))
    %
    %     if (t_2>=0 )
    %          d_1=t_2;
    %     end
    %     if (abs(d_1)<dist_front)
    %     dist_front=abs(d_1);
    %     end
    % else
    %     d_1=100000;
    % end
    
    
%     t_v=(center_z+height/2-pos_drone_z)/dir_front(3);
%     
%     if (length(t_v)>0 &&isreal(t_v))
%         x_v=pos_drone_x+t_v*dir_front(1);
%         y_v=pos_drone_y+t_v*dir_front(2);
%         
%         
%         vect_1=[0;0;1];
%         vect_1=vect_1/norm(vect_1);
%         if (pos_drone_z>=center_z+height/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
%         else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
%         end
%         angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
%         
%         if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2 &&(angle_bet<angle_limit))
%             
%             if (t_v>=0 )
%                 d_1=t_v;
%             end
%             if (abs(d_1)<estimated_distance)
%                 estimated_distance=abs(d_1);
%                 normal_surface_vector=-vect_1;
%             end
%         end
%     end
%     
%     t_v=(center_z-height/2-pos_drone_z)/dir_front(3);
%     
%     if (length(t_v)>0 &&isreal(t_v))
%         x_v=pos_drone_x+t_v*dir_front(1);
%         y_v=pos_drone_y+t_v*dir_front(2);
%         
%         vect_1=[0;0;-1];
%         vect_1=vect_1/norm(vect_1);
%         if(pos_drone_z<center_z-height/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
%         else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
%         end
%         angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
%         
%         if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2 &&(angle_bet<angle_limit))
%             
%             if (t_v>=0 )
%                 d_1=t_v;
%             end
%             if (abs(d_1)<estimated_distance)
%                 estimated_distance=abs(d_1);
%                 normal_surface_vector=-vect_1;
%             end
%         end
%         
%     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%plano z= - algo primera
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %     v_1=[dir_front(1);dir_front(2)];
    %     v_1=v_1/norm(v_1);
    
    if (pos_drone_z<=center_z-heigth/2) %gauche
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=1;
    else
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=-1;
    end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    %     v_2=[dir_normal_x;dir_normal_y];
    %     v_2=v_2/norm(v_2);
    %     angle_1 = acos(dot(v_1,v_2));
    
    normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
    dot_product_1=dot(normal_plan_1,normal_surface);
    %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
    %             dot(normal_plan_1,normal_surface));
    
    
    
    if (dot_product_1<=0.02 && dot_product_1>=-0.02)
        %             gol=22
        %solo calculo de theta
        %normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
            dot(dir_front,normal_surface));
        theta_head_prov=wrapToPi(theta_head_prov);
        phi_head_prov=0;
    else
        %calculo de los dos angulos
        phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
            dot(dir_up,-normal_surface));
        phi_head_prov=wrapToPi(phi_head_prov);
        
        normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
            dot(dir_front,normal_plan_2)));
    end
    
    dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
    module=norm(dir_normal);
    dir_normal_x=dir_normal_x/module;
    dir_normal_y=dir_normal_y/module;
    dir_normal_z=dir_normal_z/module;
    
    t_v=(center_z-heigth/2-pos_drone_z)/dir_normal_z;
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        %         x_val=pos_drone_x+t_v*dir_front(1);
        %         y_val=pos_drone_y+t_v*dir_front(2);
        x_val=pos_drone_x+t_v*dir_normal_x;
        y_val=pos_drone_y+t_v*dir_normal_y;
        
        vect_1=[0;0;-1];
        if (pos_drone_z<center_z-heigth/2)
            vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
        else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2&&...
                (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
                theta_head=theta_head_prov;
                phi_head=phi_head_prov;
                normal_surface_vector=normal_surface;
            end
        end
    end
    
    
    %%%%%plano z=+ a algo 2segunda
    
    
    %     v_1=[dir_front(1);dir_front(2)];
    %     v_1=v_1/norm(v_1);
    
    if (pos_drone_z<=center_z+height/2) %gauche
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=1;
    else
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=-1;
    end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    %     v_2=[dir_normal_x;dir_normal_y];
    %     v_2=v_2/norm(v_2);
    %     angle_1 = acos(dot(v_1,v_2));
    
    normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
    dot_product_1=dot(normal_plan_1,normal_surface);
    %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
    %             dot(normal_plan_1,normal_surface));
    if (dot_product_1<=0.02 && dot_product_1>=-0.02)
        %             gol=22
        %solo calculo de theta
        %normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
            dot(dir_front,normal_surface));
        theta_head_prov=wrapToPi(theta_head_prov);
        phi_head_prov=0;
    else
        %calculo de los dos angulos
        phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
            dot(dir_up,-normal_surface));
        phi_head_prov=wrapToPi(phi_head_prov);
        
        normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
            dot(dir_front,normal_plan_2)));
    end
    %length(t_v);
    
    dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
    module=norm(dir_normal);
    dir_normal_x=dir_normal_x/module;
    dir_normal_y=dir_normal_y/module;
    dir_normal_z=dir_normal_z/module;
    
    t_v=(center_z+heigth/2-pos_drone_z)/dir_normal_z;
    
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        x_val=pos_drone_x+t_v*dir_normal_x;
        y_val=pos_drone_y+t_v*dir_normal_y;
        
        
        vect_1=[0;0;1];
        if (pos_drone_z>=center_z+heigth/2)
            vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
        else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2&&...
                (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
                theta_head=theta_head_prov;
                phi_head=phi_head_prov;
                normal_surface_vector=normal_surface;
            end
        end
    end
end
%%%%%%sphere
for u=1:j
    
    radius=obstacul.sphere(u).radius;
    center_x=obstacul.sphere(u).center_x;
    center_y=obstacul.sphere(u).center_y;
    center_z=obstacul.sphere(u).center_z;
    
    alpha=pos_drone_x-center_x;
    beta=pos_drone_y-center_y;
    gamma= pos_drone_z-center_z;
    
    a=dir_front(1)^2+dir_front(2)^2+dir_front(3);
    b=(2*(alpha)*dir_front(1))+(2*(beta)*dir_front(2))+(2*(gamma)*dir_front(3));
    c=alpha^2+beta^2-radius^2+gamma^2;
    
    t_1=((-b+sqrt(b*b-4*a*c))/2*a);
    t_2=((-b-sqrt(b*b-4*a*c))/2*a);
    
    
    
    if(isreal(t_1) && isreal(t_2)&& ((t_1>0)||(t_2>0)))
        if (t_1>=0 && t_2>=0)
            if(t_2<t_1) t_v=t_2;
            else t_v=t_1;
            end
            x_v=pos_drone_x+t_v*dir_front(1);
            y_v=pos_drone_y+t_v*dir_front(2);
            z_v=pos_drone_z+t_v*dir_front(3);
            vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
            
        end
        
        if (t_1*t_2<0)
            if (t_1<0 && t_2>0)
                
                t_v=t_2;
            end
            
            if (t_1>0 && t_2<0)
                
                t_v=t_1;
            end
            x_v=pos_drone_x+t_v*dir_front(1);
            y_v=pos_drone_y+t_v*dir_front(2);
            z_v=pos_drone_z+t_v*dir_front(3);
            vect_2= [dir_front(1);dir_front(2);dir_front(3)];
        end
        
        vect_1=[2*(x_v-center_x);2*(y_v-center_y);2*(z_v-center_z)];
        vect_1=vect_1/norm(vect_1);
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if (angle_bet<angle_limit)
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
        
    end
end


% %%%%%%%cube

for u=1:k
    
    side_x=obstacul.cube(u).side_x;
    side_y=obstacul.cube(u).side_y;
    side_z=obstacul.cube(u).side_z;
    center_x=obstacul.cube(u).center_x;
    center_y=obstacul.cube(u).center_y;
    center_z=obstacul.cube(u).center_z;
    
    % if(center_x>0)
    
    
    %[t]= solve(x==center_x-side_x/2);
    %t_v=double(t);
    %%%plano x= a algo
    
    
    t_v=(center_x-side_x/2-pos_drone_x)/dir_front(1);
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        y_val=pos_drone_y+t_v*dir_front(2);
        z_val=pos_drone_z+t_v*dir_front(3);
        
        
        vect_1=[-1;0;0];
        if (pos_drone_x<center_x-side_x/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%%%estoy afuera
        else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((y_val<center_y+side_y/2) && (y_val>center_y-side_y/2)...
                &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                (angle_bet<angle_limit))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
    end
    %%%plano x= a algo
    
    t_v=(center_x+side_x/2-pos_drone_x)/dir_front(1);
    
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        y_val=pos_drone_y+t_v*dir_front(2);
        z_val=pos_drone_z+t_v*dir_front(3);
        
        
        vect_1=[1;0;0];
        if (pos_drone_x>=center_x+side_x/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%estoy afuera
        else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((y_val<center_y+side_y/2) && (y_val>center_y-side_y/2)...
                &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                (angle_bet<angle_limit))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
    end
    
    
    
    %%%%plano y= a algo
    t_v=(center_y-side_y/2-pos_drone_y)/dir_front(2);
    
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        x_val=pos_drone_x+t_v*dir_front(1);
        z_val=pos_drone_z+t_v*dir_front(3);
        
        
        vect_1=[0;-1;0];
        if (pos_drone_y<center_y-side_y/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%%%estoy afuera
        else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                (angle_bet<angle_limit))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
    end
    
    %%%%plano y= a algo
    t_v=(center_y+side_y/2-pos_drone_y)/dir_front(2);
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        x_val=pos_drone_x+t_v*dir_front(1);
        z_val=pos_drone_z+t_v*dir_front(3);
        
        
        vect_1=[0;1;0];
        if (pos_drone_y>=center_y+side_y/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%%%estoy afuera
        else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                (angle_bet<angle_limit))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
    end
    %%%%%plano z= a algo
    
    if (pos_drone_z>=center_z+side_z/2) %gauche
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=-1;
    else
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=1;
    end
    
    t_v=(center_z+side_z/2-pos_drone_z)/dir_normal_z
    
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        x_val=pos_drone_x+t_v*dir_normal_x;
        y_val=pos_drone_y+t_v*dir_normal_y;
        carajo=111111
        
        vect_1=[0;0;-1];
        if (pos_drone_z<center_z+side_z/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
        else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2))
        
        if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                &&(y_val<center_y+side_y/2)&&(y_val>center_y-side_y/2))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
    end
    %%%%%plano z= a algo
    
    t_v=(center_z-side_z/2-pos_drone_z)/dir_front(3);
    
    %length(t_v);
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        x_val=pos_drone_x+t_v*dir_front(1);
        y_val=pos_drone_y+t_v*dir_front(2);
        
        
        vect_1=[0;0;1];
        if (pos_drone_z>=center_z+side_z/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%%%estoy afuera
        else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                &&(y_val<center_y+side_y/2)&&(y_val>center_y-side_y/2)&&...
                (angle_bet<angle_limit))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
            end
        end
    end
    
end
% inclined plan
for u=1:l
    
%     pos_drone_x
%     pos_drone_y
%     pos_drone_z
    
    side_x=obstacul.plan_incl(u).side_x;
    side_y=obstacul.plan_incl(u).side_y;
    side_z=obstacul.plan_incl(u).side_z;
    center_x=obstacul.plan_incl(u).center_x;
    center_y=obstacul.plan_incl(u).center_y;
    center_z=obstacul.plan_incl(u).center_z;
    %y_0=obstacul.plan_incl(u).y_0;
    angle=obstacul.plan_incl(u).angle;
    
    %%%plano x= a algo 111111111111111111111
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if (pos_drone_x<=center_x-side_x/2) %gauche
        dir_normal_x=1;
        dir_normal_y=0;
        dir_normal_z=0;
    else
        dir_normal_x=-1;
        dir_normal_y=0;
        dir_normal_z=0;
    end
    
    %     if (pos_drone_z<=center_z-side_z/2) %gauche
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=1;
    %     else
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=-1;
    %     end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal_x;dir_normal_y];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    if (angle_1==0)
        
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        
        
        
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            %             gol=22
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface));
            theta_head_prov=wrapToPi(theta_head_prov)
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface));
            phi_head_prov=wrapToPi(phi_head_prov);
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)));
        end
        
        dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
        module=norm(dir_normal);
        dir_normal_x=dir_normal_x/module;
        dir_normal_y=dir_normal_y/module;
        dir_normal_z=dir_normal_z/module;
        
        t_v=(center_x-side_x/2-pos_drone_x)/dir_normal_x;
        
        if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
            y_val=pos_drone_y+t_v*dir_normal_y;
            z_val=pos_drone_z+t_v*dir_normal_z;
            
            
            vect_1=[-1;0;0];
            if (pos_drone_x<center_x-side_x/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
            else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
            end
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if ((y_val<center_y+side_y/2+tan(angle)*z_val) && (y_val>center_y-side_y/2+tan(angle)*z_val)...
                    &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
                d_1=t_v;
                if (abs(d_1)<estimated_distance)
                    estimated_distance=abs(d_1);
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    normal_surface_vector=normal_surface;
                    
                end
            end
        end
    end
    %%%plano x= a algo 2222222222222
    
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if (pos_drone_x<=center_x+side_x/2) %gauche
        dir_normal_x=1;
        dir_normal_y=0;
        dir_normal_z=0;
    else
        dir_normal_x=-1;
        dir_normal_y=0;
        dir_normal_z=0;
    end
    
    %     if (pos_drone_z<=center_z-side_z/2) %gauche
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=1;
    %     else
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=-1;
    %     end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal_x;dir_normal_y];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    if (angle_1==0)
        
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        
        
        
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            %             gol=22
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface));
            theta_head_prov=wrapToPi(theta_head_prov);
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface));
            phi_head_prov=wrapToPi(phi_head_prov);
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)))
        end
        
        dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
        module=norm(dir_normal);
        dir_normal_x=dir_normal_x/module;
        dir_normal_y=dir_normal_y/module;
        dir_normal_z=dir_normal_z/module;
        
        t_v=(center_x+side_x/2-pos_drone_x)/dir_normal_x;
        
        
        if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
            y_val=pos_drone_y+t_v*dir_normal_y;
            z_val=pos_drone_z+t_v*dir_normal_z;
            
            
            vect_1=[1;0;0];
            if (pos_drone_x>=center_x+side_x/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%estoy afuera
            else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
            end
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if ((y_val<center_y+side_y/2+tan(angle)*z_val) && (y_val>center_y-side_y/2+tan(angle)*z_val)...
                    &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
                d_1=t_v;
                if (abs(d_1)<estimated_distance)
                    estimated_distance=abs(d_1);
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    normal_surface_vector=normal_surface;
                    
                end
            end
        end
        
    end
    
    %%%%plano y= a algo 1era
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if (pos_drone_y<=center_y-side_y/2+tan(angle)*pos_drone_z) %gauche
        dir_normal_x=0;
        dir_normal_y=1;
        dir_normal_z=-tan(angle);
    else
        dir_normal_x=0;
        dir_normal_y=-1;
        dir_normal_z=tan(angle);
    end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal_x;dir_normal_y];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    
%     if ((angle_1<=0.01 && angle_1>=-0.01) || (angle_1<=pi+0.01 && angle_1>=pi-0.01))
    if(1)
%         dir_front
%         norm(dir_front)
%         escalar_prod=dot(dir_up,dir_front)
%         dir_up
%         normal_surface
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
%         normal_plan_1=dir_rside;
        normal_plan_1=normal_plan_1/norm(normal_plan_1);
        normal_surface=normal_surface/norm(normal_surface);
        dot_product_1=dot(normal_plan_1,normal_surface);
        
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
%         dot_product_1
%         angulito=acos(dot_product_1)
%         angulito=angulito*180/pi
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            %             GOL=1
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= wrapToPi(atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface)));
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= wrapToPi(atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface)));
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)));
        end
        %         dir_front
        %         dir_up
        %         dir_rside
        %     dir_normal_x=0;
        %     dir_normal_y=1;
        %     dir_normal_z=-tan(angle);
        dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
        module=norm(dir_normal);
        dir_normal_x=dir_normal_x/module;
        dir_normal_y=dir_normal_y/module;
        dir_normal_z=dir_normal_z/module;
        %     t_v=(center_y-side_y/2-pos_drone_y+pos_drone_z*tan(angle))...
        %         /(dir_front(2)-(dir_front(3)*tan(angle)));
        
        t_v=(center_y-side_y/2-pos_drone_y+pos_drone_z*tan(angle))...
            /(dir_normal_y-(dir_normal_z*tan(angle)));
        
        
        if (isreal(t_v)&& (length(t_v)>0)&&(t_v>=0))
            %         x_val=pos_drone_x+t_v*dir_front(1);
            %         z_val=pos_drone_z+t_v*dir_front(3);
            
            x_val=pos_drone_x+t_v*dir_normal_x;
            z_val=pos_drone_z+t_v*dir_normal_z;
            
            
            vect_1=[0;-1;tan(angle)];
            %y_val>center_y-side_y/2+tan(angle)*z_val;
            if (pos_drone_y<=center_y-side_y/2+tan(angle)*pos_drone_z)
                %             vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%%%estoy afuera
                vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];
            else
                %             vect_2=[dir_front(1);dir_front(2);dir_front(3)];
                vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
            end
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                    &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
                d_1=t_v;
                %                 gol=11111
                if (abs(d_1)<estimated_distance)
                    %                     gol=11111
                    estimated_distance=abs(d_1);
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    normal_surface_vector=normal_surface;
                   
                end
            else
                carajo=11111;
            end
        end
    else
        REMIERDA=111;
    end
    %%%%plano y= a algo 2da
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if (pos_drone_y<=center_y+side_y/2+tan(angle)*pos_drone_z) %gauche
        dir_normal_x=0;
        dir_normal_y=1;
        dir_normal_z=-tan(angle);
    else
        dir_normal_x=0;
        dir_normal_y=-1;
        dir_normal_z=tan(angle);
    end
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal_x;dir_normal_y];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    
    
    
    %     if (angle_1==0  || angle_1==pi)
%     if ((angle_1<=0.01 && angle_1>=-0.01) || (angle_1<=pi+0.01 && angle_1>=pi-0.01))
       if(1)
        %         GOL=2
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            %             gol=22
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= wrapToPi(atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface)));
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= wrapToPi(atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface)));
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)));
        end
        %     dir_normal_x=0;
        %     dir_normal_y=-1;
        %     dir_normal_z=tan(angle);
        dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
        module=norm(dir_normal);
        dir_normal_x=dir_normal_x/module;
        dir_normal_y=dir_normal_y/module;
        dir_normal_z=dir_normal_z/module;
        
        %     t_v=(center_y+side_y/2-pos_drone_y+pos_drone_z*tan(angle))...
        %         /(dir_front(2)-(dir_front(3)*tan(angle)));
        %     gol=1
        %     center_y
        %     side_y/2
        %     pos_drone_y
        %     tan(angle)
        %     dir_normal_y
        %     dir_normal_z
        
        t_v=(center_y+side_y/2-pos_drone_y+pos_drone_z*tan(angle))...
            /(dir_normal_y-(dir_normal_z*tan(angle)));
        
        
        if (isreal(t_v)&&(length(t_v)>0)&&(t_v>=0))
            %         x_val=pos_drone_x+t_v*dir_front(1);
            %         z_val=pos_drone_z+t_v*dir_front(3);
            
            x_val=pos_drone_x+t_v*dir_normal_x;
            z_val=pos_drone_z+t_v*dir_normal_z;
            
            
            vect_1=[0;1;-tan(angle)];
            %   y_val<center_y+side_y/2+tan(angle)*z_val
            if (pos_drone_y>=center_y+side_y/2+tan(angle)*pos_drone_z)
                %             vect_2=-[dir_front(1);dir_front(2);dir_front(3)];%%%estoy afuera
                vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];
            else
                vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
                %vect_2=[dir_front(1);dir_front(2);dir_front(3)];
            end
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                    &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
                d_1=t_v;
                %                 gol=22222;
                if (abs(d_1)<estimated_distance)
                    %                                         gol=22222
                    estimated_distance=abs(d_1);
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    normal_surface_vector=normal_surface;
                    
                end
            end
        end
    end
    
    %     theta_head
    %     phi_head
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%plano z= - algo primera
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %     v_1=[dir_front(1);dir_front(2)];
    %     v_1=v_1/norm(v_1);
    
    if (pos_drone_z<=center_z-side_z/2) %gauche
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=1;
    else
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=-1;
    end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    %     v_2=[dir_normal_x;dir_normal_y];
    %     v_2=v_2/norm(v_2);
    %     angle_1 = acos(dot(v_1,v_2));
    
    normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
    dot_product_1=dot(normal_plan_1,normal_surface);
    %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
    %             dot(normal_plan_1,normal_surface));
    
    
    
    if (dot_product_1<=0.02 && dot_product_1>=-0.02)
        %             gol=22
        %solo calculo de theta
        %normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
            dot(dir_front,normal_surface));
        theta_head_prov=wrapToPi(theta_head_prov);
        phi_head_prov=0;
    else
        %calculo de los dos angulos
        phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
            dot(dir_up,-normal_surface));
        phi_head_prov=wrapToPi(phi_head_prov);
        
        normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
            dot(dir_front,normal_plan_2)));
    end
    
    dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
    module=norm(dir_normal);
    dir_normal_x=dir_normal_x/module;
    dir_normal_y=dir_normal_y/module;
    dir_normal_z=dir_normal_z/module;
    
    t_v=(center_z-side_z/2-pos_drone_z)/dir_normal_z;
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        %         x_val=pos_drone_x+t_v*dir_front(1);
        %         y_val=pos_drone_y+t_v*dir_front(2);
        x_val=pos_drone_x+t_v*dir_normal_x;
        y_val=pos_drone_y+t_v*dir_normal_y;
        
        vect_1=[0;0;-1];
        if (pos_drone_z<center_z-side_z/2)
            vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
        else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                &&(y_val<center_y+side_y/2)&&(y_val>center_y-side_y/2)&&...
                (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
                theta_head=theta_head_prov;
                phi_head=phi_head_prov;
                normal_surface_vector=normal_surface;
                
            end
        end
    end
    
    
    %%%%%plano z=+ a algo 2segunda
    
    
    %     v_1=[dir_front(1);dir_front(2)];
    %     v_1=v_1/norm(v_1);
    
    if (pos_drone_z<=center_z+side_z/2) %gauche
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=1;
    else
        dir_normal_x=0;
        dir_normal_y=0;
        dir_normal_z=-1;
    end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    %     v_2=[dir_normal_x;dir_normal_y];
    %     v_2=v_2/norm(v_2);
    %     angle_1 = acos(dot(v_1,v_2));
    
    normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
    dot_product_1=dot(normal_plan_1,normal_surface);
    %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
    %             dot(normal_plan_1,normal_surface));
    if (dot_product_1<=0.02 && dot_product_1>=-0.02)
        %             gol=22
        %solo calculo de theta
        %normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
            dot(dir_front,normal_surface));
        theta_head_prov=wrapToPi(theta_head_prov);
        phi_head_prov=0;
    else
        %calculo de los dos angulos
        phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
            dot(dir_up,-normal_surface));
        phi_head_prov=wrapToPi(phi_head_prov);
        
        normal_plan_2=cross(dir_rside,normal_surface);
        theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
            dot(dir_front,normal_plan_2)));
    end
    %length(t_v);
    
    dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
    module=norm(dir_normal);
    dir_normal_x=dir_normal_x/module;
    dir_normal_y=dir_normal_y/module;
    dir_normal_z=dir_normal_z/module;
    
    t_v=(center_z+side_z/2-pos_drone_z)/dir_normal_z;
    
    
    if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
        x_val=pos_drone_x+t_v*dir_normal_x;
        y_val=pos_drone_y+t_v*dir_normal_y;
        
        
        vect_1=[0;0;1];
        if (pos_drone_z>=center_z+side_z/2)
            vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
        else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
        end
        angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
        
        if ((x_val<center_x+side_x/2) && (x_val>center_x-side_x/2)...
                &&(y_val<center_y+side_y/2)&&(y_val>center_y-side_y/2)&&...
                (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
            d_1=t_v;
            if (abs(d_1)<estimated_distance)
                estimated_distance=abs(d_1);
                theta_head=theta_head_prov;
                phi_head=phi_head_prov;
                normal_surface_vector=normal_surface;
               
            end
        end
    end
    
    
end

%%%%cylindre_y_z
for u=1:m
    
    
    height=obstacul.cilindre_y_z(u).height;
    radius=obstacul.cilindre_y_z(u).radius;
    center_x=obstacul.cilindre_y_z(u).center_x;
    center_y=obstacul.cilindre_y_z(u).center_y;
    center_z=obstacul.cilindre_y_z(u).center_z;
    

    
    normal_surface=[center_x-pos_drone_x;center_y-pos_drone_y;center_z-pos_drone_z];%interior
    normal_surface=normal_surface/norm(normal_surface);
    
    
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if ((pos_drone_y-center_y)^2+(pos_drone_z-center_z)^2<=radius^2 ) 
        dir_normal=-normal_surface;
        normal_surface=-normal_surface;
    else
        dir_normal=normal_surface;%vector  interior
    end
    
    %     normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    %     normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal(1);dir_normal(2)];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    if ((angle_1<=0.05 && angle_1>=-0.05) || (angle_1<=pi+0.05 && angle_1>=pi-0.05))
        
        
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        normal_plan_1=normal_plan_1/norm(normal_plan_1);
        normal_surface=normal_surface/norm(normal_surface);
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            
           
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= wrapToPi(atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface)));
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= wrapToPi(atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface)));
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)));
        end
        
        alpha=pos_drone_y-center_y;
        beta=pos_drone_z-center_z;
        a=dir_normal(2)^2+dir_normal(3)^2;
        b=(2*(alpha)*dir_normal(2))+(2*(beta)*dir_normal(3));
        c=alpha^2+beta^2-radius^2;
        
        t_1=((-b+sqrt(b*b-4*a*c))/2*a);
        t_2=((-b-sqrt(b*b-4*a*c))/2*a);
        
        if (isreal(t_1)&& isreal(t_2)&&((t_1>0)||(t_2>0)))
            if(t_1>=0 && t_2>=0)
                if(t_1<t_2)
                    t_v=t_1;
                else
                    t_v=t_2;
                end
                x_v=pos_drone_x+t_v*dir_normal(1);
                y_v=pos_drone_y+t_v*dir_normal(2);
                z_v=pos_drone_z+t_v*dir_normal(3);
                vect_2=-[dir_normal(1);dir_normal(2);dir_normal(3)];
            end
            if(t_1*t_2<0)
                if(t_1>0)
                    t_v=t_1;
                end
                if(t_2>0)
                    t_v=t_2;
                end
                x_v=pos_drone_x+t_v*dir_normal(1);
                y_v=pos_drone_y+t_v*dir_normal(2);
                z_v=pos_drone_z+t_v*dir_normal(3);
                vect_2=[dir_normal(1);dir_normal(2);dir_normal(3)];
            end
            
            vect_1=[0;2*(y_v-center_y);2*(z_v-center_z)];
            vect_1=vect_1/norm(vect_1);
            vect_2=vect_2/norm(vect_2);
            
            % vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if (x_v<center_x+height/2 && x_v>center_x-height/2 &&...
                    (angle_bet<angle_limit))
                d_1=t_v;
                if (abs(d_1)<estimated_distance)
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    estimated_distance=abs(d_1);
                    normal_surface_vector=normal_surface;
                end
            end
        end
        
    end
    
    % t_2=((-b-sqrt(b*b-4*a*c))/2*a);
    % x_2=pos_drone_x+t_2*dir_front(1);
    % y_2=pos_drone_y+t_2*dir_front(2);
    % z_2=pos_drone_z+t_2*dir_front(3);
    %
    % vect_1=[2*(x_2-center_x);2*(y_2-center_y);0];
    % vect_1=vect_1/norm(vect_1);
    % vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
    % angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
    %
    % if (z_2<center_z+height/2 && z_2>center_z-height/2 &&(angle_bet<angle_limit)...
    %         &&isreal(t_2))
    %
    %     if (t_2>=0 )
    %          d_1=t_2;
    %     end
    %     if (abs(d_1)<dist_front)
    %     dist_front=abs(d_1);
    %     end
    % else
    %     d_1=100000;
    % end
    
    
    %%%plano x= a algo 111111111111111111111
    
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if (pos_drone_x<=center_x-height/2) %gauche
        dir_normal_x=1;
        dir_normal_y=0;
        dir_normal_z=0;
    else
        dir_normal_x=-1;
        dir_normal_y=0;
        dir_normal_z=0;
    end
    
    %     if (pos_drone_z<=center_z-side_z/2) %gauche
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=1;
    %     else
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=-1;
    %     end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal_x;dir_normal_y];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    if (angle_1==0)
        
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        
        
        
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            %             gol=22
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface));
            theta_head_prov=wrapToPi(theta_head_prov);
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface));
            phi_head_prov=wrapToPi(phi_head_prov);
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)));
        end
        
        dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
        module=norm(dir_normal);
        dir_normal_x=dir_normal_x/module;
        dir_normal_y=dir_normal_y/module;
        dir_normal_z=dir_normal_z/module;
        
        t_v=(center_x-side_x/2-pos_drone_x)/dir_normal_x;
        
        if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
            y_val=pos_drone_y+t_v*dir_normal_y;
            z_val=pos_drone_z+t_v*dir_normal_z;
            
            
            vect_1=[-1;0;0];
            if (pos_drone_x<center_x-side_x/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
            else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
            end
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if ((y_v-center_y)^2+(z_v-center_z)^2<=radius^2 &&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
                d_1=t_v;
                if (abs(d_1)<estimated_distance)
                    estimated_distance=abs(d_1);
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    normal_surface_vector=normal_surface;
                end
            end
        end
    end
    %%%plano x= a algo 2222222222222
    
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if (pos_drone_x<=center_x+height/2) %gauche
        dir_normal_x=1;
        dir_normal_y=0;
        dir_normal_z=0;
    else
        dir_normal_x=-1;
        dir_normal_y=0;
        dir_normal_z=0;
    end
    
    %     if (pos_drone_z<=center_z-side_z/2) %gauche
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=1;
    %     else
    %         dir_normal_x=0;
    %         dir_normal_y=0;
    %         dir_normal_z=-1;
    %     end
    
    normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal_x;dir_normal_y];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    if (angle_1==0)
        
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        
        
        
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            %             gol=22
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface));
            theta_head_prov=wrapToPi(theta_head_prov);
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface));
            phi_head_prov=wrapToPi(phi_head_prov);
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)))
        end
        
        dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
        module=norm(dir_normal);
        dir_normal_x=dir_normal_x/module;
        dir_normal_y=dir_normal_y/module;
        dir_normal_z=dir_normal_z/module;
        
        t_v=(center_x+side_x/2-pos_drone_x)/dir_normal_x;
        
        
        if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
            y_val=pos_drone_y+t_v*dir_normal_y;
            z_val=pos_drone_z+t_v*dir_normal_z;
            
            
            vect_1=[1;0;0];
            if (pos_drone_x>=center_x+side_x/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%estoy afuera
            else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
            end
            angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
            
            if ((y_v-center_y)^2+(z_v-center_z)^2<=radius^2&&...
                    (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
                d_1=t_v;
                if (abs(d_1)<estimated_distance)
                    estimated_distance=abs(d_1);
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    normal_surface_vector=normal_surface;
                end
            end
        end
        
    end
    
    
    %     t_v=(center_z+height/2-pos_drone_z)/dir_front(3);
    %
    %     if (length(t_v)>0 &&isreal(t_v))
    %         x_v=pos_drone_x+t_v*dir_front(1);
    %         y_v=pos_drone_y+t_v*dir_front(2);
    %
    %
    %         vect_1=[0;0;1];
    %         vect_1=vect_1/norm(vect_1);
    %         if (pos_drone_z>=center_z+height/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
    %         else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
    %         end
    %         angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
    %
    %         if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2 &&(angle_bet<angle_limit))
    %
    %             if (t_v>=0 )
    %                 d_1=t_v;
    %             end
    %             if (abs(d_1)<estimated_distance)
    %                 estimated_distance=abs(d_1);
    %                 normal_surface_vector=-vect_1;
    %             end
    %         end
    %     end
    %
    %     t_v=(center_z-height/2-pos_drone_z)/dir_front(3);
    %
    %     if (length(t_v)>0 &&isreal(t_v))
    %         x_v=pos_drone_x+t_v*dir_front(1);
    %         y_v=pos_drone_y+t_v*dir_front(2);
    %
    %         vect_1=[0;0;-1];
    %         vect_1=vect_1/norm(vect_1);
    %         if(pos_drone_z<center_z-height/2) vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
    %         else vect_2=[dir_front(1);dir_front(2);dir_front(3)];
    %         end
    %         angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
    %
    %         if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2 &&(angle_bet<angle_limit))
    %
    %             if (t_v>=0 )
    %                 d_1=t_v;
    %             end
    %             if (abs(d_1)<estimated_distance)
    %                 estimated_distance=abs(d_1);
    %                 normal_surface_vector=-vect_1;
    %             end
    %         end
    %
    %     end
end
%%%%%%%%%%%%%%%%%%paraboloide

for u=1:n
    
    alpha_par=obstacul.paraboloid_y_z(u).alpha;
    height_y=obstacul.paraboloid_y_z(u).height;
    height_z=obstacul.paraboloid_y_z(u).altitude_z;
    center_x=obstacul.paraboloid_y_z(u).center_x;
    center_y=obstacul.paraboloid_y_z(u).center_y;
    center_z=obstacul.paraboloid_y_z(u).center_z;
    

    
    normal_surface=[0;-2*alpha_par*(pos_drone_y-center_y);1];%interior
    normal_surface=normal_surface/norm(normal_surface);
    
    
    v_1=[dir_front(1);dir_front(2)];
    v_1=v_1/norm(v_1);
    
    if ((pos_drone_z-center_z)>=(alpha_par*(pos_drone_y-center_y)^2) ) 
        dir_normal=-normal_surface;
        normal_surface=-normal_surface;
    else
%         aqui=1
        dir_normal=normal_surface;%vector  interior
    end
    
    %     normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
    %     normal_surface=normal_surface/norm(normal_surface);
    v_2=[dir_normal(1);dir_normal(2)];
    v_2=v_2/norm(v_2);
    angle_1 = acos(dot(v_1,v_2));
    
    if ((angle_1<=0.05 && angle_1>=-0.05) || (angle_1<=pi+0.05 && angle_1>=pi-0.05))
        
        
        normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
        normal_plan_1=normal_plan_1/norm(normal_plan_1);
        normal_surface=normal_surface/norm(normal_surface);
        dot_product_1=dot(normal_plan_1,normal_surface);
        %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
        %             dot(normal_plan_1,normal_surface));
        if (dot_product_1<=0.02 && dot_product_1>=-0.02)
            
           
            %solo calculo de theta
            %normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= wrapToPi(atan2(norm(cross(dir_front,normal_surface)),...
                dot(dir_front,normal_surface)));
            phi_head_prov=0;
        else
            %calculo de los dos angulos
            phi_head_prov= wrapToPi(atan2(norm(cross(dir_up,-normal_surface)),...
                dot(dir_up,-normal_surface)));
            
            normal_plan_2=cross(dir_rside,normal_surface);
            theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
                dot(dir_front,normal_plan_2)));
        end
        
%         dir_normal
%         center_y
%         center_z
%         alpha_par
%         pos_drone_y
%         pos_drone_z
        
        alpha=pos_drone_y-center_y;
        beta=pos_drone_z-center_z;
        
        a=alpha_par*dir_normal(2)*dir_normal(2);
        b=(alpha_par*2*alpha*dir_normal(2))-dir_normal(3);
        c=(alpha_par*alpha*alpha)-beta;
        
%         a 
%         b
%         c
        
        
        t_1=((-b+sqrt(b*b-4*a*c))/(2*a));
        t_2=((-b-sqrt(b*b-4*a*c))/(2*a));
        
        if (isreal(t_1)&& isreal(t_2)&&((t_1>0)||(t_2>0)))
            if(t_1>=0 && t_2>=0)
                if(t_1<t_2)
                    t_v=t_1;
                else
                    t_v=t_2;
                end
                x_v=pos_drone_x+t_v*dir_normal(1);
                y_v=pos_drone_y+t_v*dir_normal(2);
                z_v=pos_drone_z+t_v*dir_normal(3);
                vect_2=-[dir_normal(1);dir_normal(2);dir_normal(3)];
            end
            if(t_1*t_2<0)
                if(t_1>0)
                    t_v=t_1;
                end
                if(t_2>0)
                    t_v=t_2;
                end
                x_v=pos_drone_x+t_v*dir_normal(1);
                y_v=pos_drone_y+t_v*dir_normal(2);
                z_v=pos_drone_z+t_v*dir_normal(3);
                vect_2=[dir_normal(1);dir_normal(2);dir_normal(3)];
            end
            
            vect_1=[0;2*(y_v-center_y);2*(z_v-center_z)];
            vect_1=vect_1/norm(vect_1);
            vect_2=vect_2/norm(vect_2);
            
            % vect_2=-[dir_front(1);dir_front(2);dir_front(3)];
            angle_bet = 0;
%             leego=1
            if (x_v<center_x+height_y/2 && x_v>center_x-height_y/2 &&...
                   y_v<center_y+height_y && y_v>=0 &&... 
                   z_v<center_z+height_z && z_v>=0 &&... 
                   (angle_bet<angle_limit))
                d_1=t_v;
                if (abs(d_1)<estimated_distance)
                    theta_head=theta_head_prov;
                    phi_head=phi_head_prov;
                    estimated_distance=abs(d_1);
                    normal_surface_vector=normal_surface;
                end
            end
        end
        
    end
    
    
    
    
%     %%%plano x= a algo 111111111111111111111
%     
%     v_1=[dir_front(1);dir_front(2)];
%     v_1=v_1/norm(v_1);
%     
%     if (pos_drone_x<=center_x-height/2) %gauche
%         dir_normal_x=1;
%         dir_normal_y=0;
%         dir_normal_z=0;
%     else
%         dir_normal_x=-1;
%         dir_normal_y=0;
%         dir_normal_z=0;
%     end
%     
%     %     if (pos_drone_z<=center_z-side_z/2) %gauche
%     %         dir_normal_x=0;
%     %         dir_normal_y=0;
%     %         dir_normal_z=1;
%     %     else
%     %         dir_normal_x=0;
%     %         dir_normal_y=0;
%     %         dir_normal_z=-1;
%     %     end
%     
%     normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
%     normal_surface=normal_surface/norm(normal_surface);
%     v_2=[dir_normal_x;dir_normal_y];
%     v_2=v_2/norm(v_2);
%     angle_1 = acos(dot(v_1,v_2));
%     
%     if (angle_1==0)
%         
%         normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
%         dot_product_1=dot(normal_plan_1,normal_surface);
%         %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
%         %             dot(normal_plan_1,normal_surface));
%         
%         
%         
%         if (dot_product_1<=0.02 && dot_product_1>=-0.02)
%             %             gol=22
%             %solo calculo de theta
%             %normal_plan_2=cross(dir_rside,normal_surface);
%             theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
%                 dot(dir_front,normal_surface));
%             theta_head_prov=wrapToPi(theta_head_prov);
%             phi_head_prov=0;
%         else
%             %calculo de los dos angulos
%             phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
%                 dot(dir_up,-normal_surface));
%             phi_head_prov=wrapToPi(phi_head_prov);
%             
%             normal_plan_2=cross(dir_rside,normal_surface);
%             theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
%                 dot(dir_front,normal_plan_2)));
%         end
%         
%         dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
%         module=norm(dir_normal);
%         dir_normal_x=dir_normal_x/module;
%         dir_normal_y=dir_normal_y/module;
%         dir_normal_z=dir_normal_z/module;
%         
%         t_v=(center_x-side_x/2-pos_drone_x)/dir_normal_x;
%         
%         if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
%             y_val=pos_drone_y+t_v*dir_normal_y;
%             z_val=pos_drone_z+t_v*dir_normal_z;
%             
%             
%             vect_1=[-1;0;0];
%             if (pos_drone_x<center_x-side_x/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%%%estoy afuera
%             else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
%             end
%             angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
%             
%             if ((y_v-center_y)^2+(z_v-center_z)^2<=radius^2 &&...
%                     (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
%                 d_1=t_v;
%                 if (abs(d_1)<estimated_distance)
%                     estimated_distance=abs(d_1);
%                     theta_head=theta_head_prov;
%                     phi_head=phi_head_prov;
%                     normal_surface_vector=normal_surface;
%                 end
%             end
%         end
%     end
%     %%%plano x= a algo 2222222222222
%     
%     v_1=[dir_front(1);dir_front(2)];
%     v_1=v_1/norm(v_1);
%     
%     if (pos_drone_x<=center_x+height/2) %gauche
%         dir_normal_x=1;
%         dir_normal_y=0;
%         dir_normal_z=0;
%     else
%         dir_normal_x=-1;
%         dir_normal_y=0;
%         dir_normal_z=0;
%     end
%     
%     normal_surface=[dir_normal_x;dir_normal_y;dir_normal_z];
%     normal_surface=normal_surface/norm(normal_surface);
%     v_2=[dir_normal_x;dir_normal_y];
%     v_2=v_2/norm(v_2);
%     angle_1 = acos(dot(v_1,v_2));
%     
%     if (angle_1==0)
%         
%         normal_plan_1= cross (dir_up,dir_front);%puede ser reemplazado por der
%         dot_product_1=dot(normal_plan_1,normal_surface);
%         %         angle_2 = atan2(norm(cross(normal_plan_1,normal_surface)),...
%         %             dot(normal_plan_1,normal_surface));
%         
%         
%         
%         if (dot_product_1<=0.02 && dot_product_1>=-0.02)
%             %             gol=22
%             %solo calculo de theta
%             %normal_plan_2=cross(dir_rside,normal_surface);
%             theta_head_prov=atan2(norm(cross(dir_front,normal_surface)),...
%                 dot(dir_front,normal_surface));
%             theta_head_prov=wrapToPi(theta_head_prov);
%             phi_head_prov=0;
%         else
%             %calculo de los dos angulos
%             phi_head_prov= atan2(norm(cross(dir_up,-normal_surface)),...
%                 dot(dir_up,-normal_surface));
%             phi_head_prov=wrapToPi(phi_head_prov);
%             
%             normal_plan_2=cross(dir_rside,normal_surface);
%             theta_head_prov= pi/2- wrapToPi(atan2(norm(cross(dir_front,normal_plan_2)),...
%                 dot(dir_front,normal_plan_2)))
%         end
%         
%         dir_normal=[dir_normal_x;dir_normal_y;dir_normal_z];
%         module=norm(dir_normal);
%         dir_normal_x=dir_normal_x/module;
%         dir_normal_y=dir_normal_y/module;
%         dir_normal_z=dir_normal_z/module;
%         
%         t_v=(center_x+side_x/2-pos_drone_x)/dir_normal_x;
%         
%         
%         if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
%             y_val=pos_drone_y+t_v*dir_normal_y;
%             z_val=pos_drone_z+t_v*dir_normal_z;
%             
%             
%             vect_1=[1;0;0];
%             if (pos_drone_x>=center_x+side_x/2) vect_2=-[dir_normal_x;dir_normal_y;dir_normal_z];%estoy afuera
%             else vect_2=[dir_normal_x;dir_normal_y;dir_normal_z];
%             end
%             angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
%             
%             if ((y_v-center_y)^2+(z_v-center_z)^2<=radius^2&&...
%                     (angle_bet<angle_limit)&&(phi_head_prov<pi/4)&&(phi_head_prov>-pi/4))
%                 d_1=t_v;
%                 if (abs(d_1)<estimated_distance)
%                     estimated_distance=abs(d_1);
%                     theta_head=theta_head_prov;
%                     phi_head=phi_head_prov;
%                     normal_surface_vector=normal_surface;
%                 end
%             end
%         end
%         
%     end
    
   
end
end