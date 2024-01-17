classdef object_cylinder_y_z < hgsetget & obstacle

properties
        center_x ;
        center_y ;
        center_z ;
        radius;
        height;
        psi;
        tetha;
        phi;
end
methods
        function obj = object_cylinder_y_z (center_x,center_y,center_z,radius,height)
            obj.center_x=center_x;
            obj.center_y=center_y;
            obj.center_z=center_z;
            obj.radius=radius;
            obj.height=height;
            obj.psi=0;
            obj.tetha=0;
            obj.phi=0;
        end
        
        function graph_cylinder_y_z (obj)
            nb_cubes=size(obj);
            number= nb_cubes(1,2);
       %figure  
       for i=1:number
        
       param = 25 ;
       r = obj(i).radius * ones(param, param);
       %[h] = meshgrid(linspace(-obj(i).height/2,obj(i).height/2,param));
       %[phi_rot] = meshgrid(linspace(0, 2*pi, param),);
       [phi_rot,h] = meshgrid(linspace(0, 2*pi, param),linspace(-obj(i).height/2,obj(i).height/2,param));
       y=r.*cos(phi_rot);
       %cos(2*pi)
       z=r.*sin(phi_rot);
       x=h;
       
       if((obj(i).psi~=0) || (obj(i).tetha~=0) || (obj(i).phi~=0))
       
       ang_phi=obj(i).phi;
       ang_tetha=obj(i).tetha;
       ang_psi=obj(i).psi;
       %psi=(5/180)*pi;

       eul=[ang_psi ang_tetha ang_phi];
       rotmZYX=eul2rotm(eul);

       quat = rotm2quat(rotmZYX);
       R=quat2rotm(quat);
       
       for u=1:param
        for v=1:param
        vector=R*[x(u,v);y(u,v);z(u,v)];
        x(u,v)=vector(1);
        y(u,v)=vector(2);
        z(u,v)=vector(3);
        
        end
       end
       
       end
       
       x = x + obj(i).center_x;  % center at 16 in x-direction
       y = y + obj(i).center_y;  % center at 40 in y-direction
       z = z + obj(i).center_z;   % center at 2 in z-direction
        % Now we use the surface command to add the sphere. We just need to set the FaceColor as desired.
       surface(x,y,z,'FaceColor', 'none')
       hold on
       end
            
            
        end
end
end