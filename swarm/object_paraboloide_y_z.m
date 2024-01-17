classdef object_paraboloide_y_z < hgsetget & obstacle

properties
        center_x ;
        center_y ;
        center_z ;
        alpha;%% y=alpha*x^2
        altitude_z;
        height;
        psi;
        tetha;
        phi;
end
methods
        function obj = object_paraboloide_y_z (center_x,center_y,center_z,alpha,height,altitude_z)
            obj.center_x=center_x;
            obj.center_y=center_y;
            obj.center_z=center_z;
            obj.alpha=alpha;
            obj.height=height;
            obj.altitude_z=altitude_z;
            obj.psi=0;
            obj.tetha=0;
            obj.phi=0;
        end
        
        function graph_paraboloide_y_z (obj)
            
            nb_cubes=size(obj);
            number= nb_cubes(1,2);
            
       %figure  
       for i=1:number
        
       param = 25 ;
       alpha_c = obj(i).alpha * ones(param, param);
%        [h] = meshgrid(linspace(-obj(i).height/2,obj(i).height/2,param));
%        [Y] = meshgrid(linspace(0,obj(i).height,param));
       %[phi_rot] = meshgrid(linspace(0, 2*pi, param),);
       %[phi_rot,h] = meshgrid(linspace(0, 2*pi, param),linspace(-obj(i).height/2,obj(i).height/2,param));
       [Y,h] = meshgrid(linspace(0, obj(i).height, param),linspace(-obj(i).height/2,obj(i).height/2,param));
       
%        talla=size(Y.*Y)
       y=Y;
       
       z=obj(i).alpha*(Y.*Y);
       
       x=h;
       
       
       
       x = x + obj(i).center_x;  % center at 16 in x-direction
       y = y + obj(i).center_y;  % center at 40 in y-direction
       z = z + obj(i).center_z;   % center at 2 in z-direction
        % Now we use the surface command to add the sphere. We just need to set the FaceColor as desired.
       surface(x,y,z,'FaceColor', 'none')
       zlim ([0 obj(i).altitude_z])
       hold on
       end
            
            
        end
end
end