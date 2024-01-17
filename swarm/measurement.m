function[estimated_distance]= measurement(obstacul,i,j,k,l,direction_x,direction_y,direction_z,pos_drone_x,pos_drone_y,pos_drone_z)
angle_limit=27.5*(pi/180);
estimated_distance=100000;
%dist_back=100000;
%d_1=0;
%d_3=0;
d_1=100000;
%d_3=100000;


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
alpha=pos_drone_x-center_x;
beta=pos_drone_y-center_y;
a=direction_x^2+direction_y^2;
b=(2*(alpha)*direction_x)+(2*(beta)*direction_y);
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
    x_v=pos_drone_x+t_v*direction_x;
    y_v=pos_drone_y+t_v*direction_y;
    z_v=pos_drone_z+t_v*direction_z;
    vect_2=-[direction_x;direction_y;direction_z];
    end
    if(t_1*t_2<0)
    if(t_1>0)
        t_v=t_1;
    end
    if(t_2>0)
        t_v=t_2;
    end
    x_v=pos_drone_x+t_v*direction_x;
    y_v=pos_drone_y+t_v*direction_y;
    z_v=pos_drone_z+t_v*direction_z;
    vect_2=[direction_x;direction_y;direction_z];
    end

vect_1=[2*(x_v-center_x);2*(y_v-center_y);0];
vect_1=vect_1/norm(vect_1);
% vect_2=-[direction_x;direction_y;direction_z];
angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));

if (z_v<center_z+height/2 && z_v>center_z-height/2 &&(angle_bet<angle_limit))
    d_1=t_v;  
    if (abs(d_1)<estimated_distance)
    estimated_distance=abs(d_1);
    end
end    
end



% t_2=((-b-sqrt(b*b-4*a*c))/2*a);
% x_2=pos_drone_x+t_2*direction_x;
% y_2=pos_drone_y+t_2*direction_y;
% z_2=pos_drone_z+t_2*direction_z;
% 
% vect_1=[2*(x_2-center_x);2*(y_2-center_y);0];
% vect_1=vect_1/norm(vect_1);
% vect_2=-[direction_x;direction_y;direction_z];
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


t_v=(center_z+height/2-pos_drone_z)/direction_z;

if (length(t_v)>0 &&isreal(t_v))
x_v=pos_drone_x+t_v*direction_x;
y_v=pos_drone_y+t_v*direction_y;


vect_1=[0;0;1];
vect_1=vect_1/norm(vect_1);
if (pos_drone_z>=center_z+height/2) vect_2=-[direction_x;direction_y;direction_z];
else vect_2=[direction_x;direction_y;direction_z];
end    
angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));

if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2 &&(angle_bet<angle_limit))
    
    if (t_v>=0 )
         d_1=t_v;
    end   
    if (abs(d_1)<estimated_distance)
    estimated_distance=abs(d_1);
    end
end
end

t_v=(center_z-height/2-pos_drone_z)/direction_z;

if (length(t_v)>0 &&isreal(t_v))
x_v=pos_drone_x+t_v*direction_x;
y_v=pos_drone_y+t_v*direction_y;

vect_1=[0;0;-1];
vect_1=vect_1/norm(vect_1);
if(pos_drone_z<center_z-height/2) vect_2=-[direction_x;direction_y;direction_z];
else vect_2=[direction_x;direction_y;direction_z];    
end
angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));

if ((x_v-center_x)^2+(y_v-center_y)^2<=radius^2 &&(angle_bet<angle_limit))
    
    if (t_v>=0 )
         d_1=t_v;
    end   
    if (abs(d_1)<estimated_distance)
    estimated_distance=abs(d_1);
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

a=direction_x^2+direction_y^2+direction_z;
b=(2*(alpha)*direction_x)+(2*(beta)*direction_y)+(2*(gamma)*direction_z);
c=alpha^2+beta^2-radius^2+gamma^2;

t_1=((-b+sqrt(b*b-4*a*c))/2*a);
t_2=((-b-sqrt(b*b-4*a*c))/2*a);



if(isreal(t_1) && isreal(t_2)&& ((t_1>0)||(t_2>0)))
    if (t_1>=0 && t_2>=0)
       if(t_2<t_1) t_v=t_2;
       else t_v=t_1;
       end
    x_v=pos_drone_x+t_v*direction_x;
    y_v=pos_drone_y+t_v*direction_y;
    z_v=pos_drone_z+t_v*direction_z;
    vect_2=-[direction_x;direction_y;direction_z]; 
       
    end
    
    if (t_1*t_2<0)
    if (t_1<0 && t_2>0)

    t_v=t_2;
    end
    
    if (t_1>0 && t_2<0)

    t_v=t_1;
    end
    x_v=pos_drone_x+t_v*direction_x;
    y_v=pos_drone_y+t_v*direction_y;
    z_v=pos_drone_z+t_v*direction_z;
    vect_2= [direction_x;direction_y;direction_z];
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


t_v=(center_x-side_x/2-pos_drone_x)/direction_x;

if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    y_val=pos_drone_y+t_v*direction_y;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[-1;0;0];
  if (pos_drone_x<center_x-side_x/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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

t_v=(center_x+side_x/2-pos_drone_x)/direction_x;


if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    y_val=pos_drone_y+t_v*direction_y;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[1;0;0];
  if (pos_drone_x>=center_x+side_x/2) vect_2=-[direction_x;direction_y;direction_z];%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
t_v=(center_y-side_y/2-pos_drone_y)/direction_y;


if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[0;-1;0];
  if (pos_drone_y<center_y-side_y/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
t_v=(center_y+side_y/2-pos_drone_y)/direction_y;

if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[0;1;0];
  if (pos_drone_y>=center_y+side_y/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
t_v=(center_z-side_z/2-pos_drone_z)/direction_z;


if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    y_val=pos_drone_y+t_v*direction_y;
    
  
  vect_1=[0;0;-1];
  if (pos_drone_z<center_z-side_z/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
%%%%%plano z= a algo

t_v=(center_z+side_z/2-pos_drone_z)/direction_z;

%length(t_v);

if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    y_val=pos_drone_y+t_v*direction_y;
    
  
  vect_1=[0;0;1];
  if (pos_drone_z>=center_z+side_z/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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

for u=1:l

side_x=obstacul.plan_incl(u).side_x;
side_y=obstacul.plan_incl(u).side_y;
side_z=obstacul.plan_incl(u).side_z;
center_x=obstacul.plan_incl(u).center_x;
center_y=obstacul.plan_incl(u).center_y;
center_z=obstacul.plan_incl(u).center_z;
%y_0=obstacul.plan_incl(u).y_0;
angle=obstacul.plan_incl(u).angle;

%%%plano x= a algo

t_v=(center_x-side_x/2-pos_drone_x)/direction_x;

if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    y_val=pos_drone_y+t_v*direction_y;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[-1;0;0];
  if (pos_drone_x<center_x-side_x/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
  end
  angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
  
  if ((y_val<center_y+side_y/2+tan(angle)*z_val) && (y_val>center_y-side_y/2+tan(angle)*z_val)...
          &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&... 
          (angle_bet<angle_limit)) 
    d_1=t_v;
  if (abs(d_1)<estimated_distance)
      estimated_distance=abs(d_1);
  end
  end
end
%%%plano x= a algo

t_v=(center_x+side_x/2-pos_drone_x)/direction_x;


if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    y_val=pos_drone_y+t_v*direction_y;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[1;0;0];
  if (pos_drone_x>=center_x+side_x/2) vect_2=-[direction_x;direction_y;direction_z];%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
  end
  angle_bet = atan2(norm(cross(vect_1,vect_2)),dot(vect_1,vect_2));
  
  if ((y_val<center_y+side_y/2+tan(angle)*z_val) && (y_val>center_y-side_y/2+tan(angle)*z_val)...
          &&(z_val<center_z+side_z/2)&&(z_val>center_z-side_z/2)&&... 
          (angle_bet<angle_limit)) 
    d_1=t_v;
  if (abs(d_1)<estimated_distance)
      estimated_distance=abs(d_1);
  end
  end
end



%%%%plano y= a algo
t_v=(center_y-side_y/2-pos_drone_y+pos_drone_z*tan(angle))...
    /(direction_y-(direction_z*tan(angle)));


if (isreal(t_v)&& (length(t_v)>0)&&(t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    z_val=pos_drone_z+t_v*direction_z;
    
  
  vect_1=[0;-1;tan(angle)];
  %y_val>center_y-side_y/2+tan(angle)*z_val;
  if (pos_drone_y<=center_y-side_y/2+tan(angle)*pos_drone_z) 
      vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
t_v=(center_y+side_y/2-pos_drone_y+pos_drone_z*tan(angle))...
    /(direction_y-(direction_z*tan(angle)));
%t_v=(center_y+side_y/2-pos_drone_y)/direction_y;

if (isreal(t_v)&&(length(t_v)>0)&&(t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    z_val=pos_drone_z+t_v*direction_z;
  
  vect_1=[0;1;-tan(angle)];
%   y_val<center_y+side_y/2+tan(angle)*z_val
  if (pos_drone_y>=center_y+side_y/2+tan(angle)*pos_drone_z) 
      vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
t_v=(center_z-side_z/2-pos_drone_z)/direction_z;


if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    y_val=pos_drone_y+t_v*direction_y;
    
  
  vect_1=[0;0;-1];
  if (pos_drone_z<center_z-side_z/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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
%%%%%plano z= a algo

t_v=(center_z+side_z/2-pos_drone_z)/direction_z;

%length(t_v);

if (isreal(t_v)&& (length(t_v)>0)&& (t_v>=0))
    x_val=pos_drone_x+t_v*direction_x;
    y_val=pos_drone_y+t_v*direction_y;
    
  
  vect_1=[0;0;1];
  if (pos_drone_z>=center_z+side_z/2) vect_2=-[direction_x;direction_y;direction_z];%%%estoy afuera
  else vect_2=[direction_x;direction_y;direction_z];
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

end