classdef object_parale < hgsetget & obstacle

properties
        center_x ;
        center_y ;
        center_z ;
        side_x;
        side_y;
        side_z;
        psi;
        tetha;
        phi;        
end
methods
        function obj = object_parale (center_x,center_y,center_z,side_x,side_y,side_z)
            obj.center_x=center_x;
            obj.center_y=center_y;
            obj.center_z=center_z;
            obj.side_x=side_x;
            obj.side_y=side_y;
            obj.side_z=side_z;
            obj.psi=0;
            obj.tetha=0;
            obj.phi=0;
        end
        
        function graph_parale (obj)
            nb_cubes=size(obj);
            number= nb_cubes(1,2);
       
       for i=1:number
        
       clear X Y Z;     
       d_x=[-obj(i).side_x/2 obj(i).side_x/2];
       d_y=[-obj(i).side_y/2 obj(i).side_y/2];
       d_z=[-obj(i).side_z/2 obj(i).side_z/2];
       [X,Y,Z]=meshgrid(d_x,d_y,d_z);
       
       if((obj(i).psi~=0)||(obj(i).tetha~=0)||(obj(i).phi~=0))
       
       ang_phi=obj(i).phi;
       ang_tetha=obj(i).tetha;
       ang_psi=obj(i).psi;
       %psi=(5/180)*pi;

       eul=[ang_psi ang_tetha ang_phi];
       rotmZYX=eul2rotm(eul);

       quat = rotm2quat(rotmZYX);
       R=quat2rotm(quat);
       
       for u=1:2
        for v=1:2
         for w=1:2
        vector=R*[X(u,v,w);Y(u,v,w);Z(u,v,w)];
        X(u,v,w)=vector(1);
        Y(u,v,w)=vector(2);
        Z(u,v,w)=vector(3);
        
        end
        end
       end
       
       end
       
       X = X + obj(i).center_x;
       Y = Y + obj(i).center_y;
       Z = Z + obj(i).center_z;
       
       points=[X(:),Y(:),Z(:);obj(i).center_x,obj(i).center_y,obj(i).center_z];
       T1=delaunay(points);
       tetramesh(T1,points,'FaceColor',[0 0 0.5],...
           'EdgeColor',[0 0 1],'FaceAlpha',0.3);
       hold on
       end
            
            
            
        end
end
end