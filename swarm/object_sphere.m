classdef object_sphere < hgsetget & obstacle

properties
        center_x ;
        center_y ;
        center_z ;
        radius;        
end
methods
        function obj = object_sphere (center_x,center_y,center_z,radius)
            obj.center_x=center_x;
            obj.center_y=center_y;
            obj.center_z=center_z;
            obj.radius=radius;
        end
        
        function graph_sphere (obj)
            nb_cubes=size(obj);
            number= nb_cubes(1,2);
       %figure  
       for i=1:number
        
       param = 25 ;
       r = obj(i).radius * ones(param, param); % radius is 5
       [th, phi] = meshgrid(linspace(0, 2*pi, param), linspace(-pi, pi, param));
       clear x y z 
       [x,y,z] = sph2cart(th, phi, r);

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