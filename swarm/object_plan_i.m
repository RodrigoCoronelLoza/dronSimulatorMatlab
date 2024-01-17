classdef object_plan_i < hgsetget & obstacle
  
    properties
        %y_0;%intersection with the a y axis
        angle; %angle between the plan and the z axis
        side_x;
        side_y;
        side_z;
        center_x;
        center_y;
        center_z;
        
    end
    
    methods
        function obj = object_plan_i (angle,center_x,center_y,center_z,side_x,side_y,side_z)
          %obj.y_0=y_0;
          obj.angle=angle;
          obj.side_x=side_x;
          obj.side_y=side_y;
          obj.side_z=side_z;
          obj.center_x=center_x;
          obj.center_y=center_y;
          obj.center_z=center_z;
         
        end
        
        
       function graph_plan_incl (obj)
            nb_cubes=size(obj);
            number= nb_cubes(1,2);
       
       for i=1:number
        
       clear X Y Z;     
       %d_x = linspace(-obj(i).side_x/2,obj(i).side_x/2,10);
       %d_x=[-obj(i).side_x/2 obj(i).side_x/2];
%        d_y=[-obj(i).side_y/2 obj(i).side_y/2];
       %d_z=[-obj(i).side_z/2 obj(i).side_z/2];
       %d_z= linspace(-obj(i).side_z/2,obj(i).side_z/2,10);
       %d_y=obj(i).y_0+(tan(obj(i).angle)).*(d_z);
%        [X,Y,Z]=meshgrid(d_x,d_y,d_z)
       X=[-obj(i).side_x/2;obj(i).side_x/2;-obj(i).side_x/2;obj(i).side_x/2];
       Z=[-obj(i).side_z/2;-obj(i).side_z/2;obj(i).side_z/2;obj(i).side_z/2];
       
       Y=obj(i).center_y+(tan(obj(i).angle)).*(Z);
       Y_1=Y-obj(i).side_y/2;
       Y_2=Y+obj(i).side_y/2;
%        if((obj(i).psi~=0)||(obj(i).tetha~=0)||(obj(i).phi~=0))
%        
%        ang_phi=obj(i).phi;
%        ang_tetha=obj(i).tetha;
%        ang_psi=obj(i).psi;
%        %psi=(5/180)*pi;
% 
%        eul=[ang_psi ang_tetha ang_phi];
%        rotmZYX=eul2rotm(eul);
% 
%        quat = rotm2quat(rotmZYX);
%        R=quat2rotm(quat);
%        
%        for u=1:2
%         for v=1:2
%          for w=1:2
%         vector=R*[X(u,v,w);Y(u,v,w);Z(u,v,w)];
%         X(u,v,w)=vector(1);
%         Y(u,v,w)=vector(2);
%         Z(u,v,w)=vector(3);
%         
%         end
%         end
%        end
%        
%        end
       
       X = X + obj(i).center_x;
       %Y_1 = Y_1 + obj(i).center_y;
       Z = Z + obj(i).center_z;
       
       %Y_2 = Y_2 + obj(i).center_y;
       
       %points=[X(:),Y(:),Z(:);obj(i).center_x,obj(i).center_y,obj(i).center_z]
       
       points=[X(:),Y_1(:),Z(:);X(:),Y_2(:),Z(:)];
       T1=delaunay(points);
       tetramesh(T1,points,'FaceColor',[0 0 0.6],...
           'EdgeColor',[0 0 1],'FaceAlpha',0.5);
       hold on
       end
            
            
            
        end
    end
    
end