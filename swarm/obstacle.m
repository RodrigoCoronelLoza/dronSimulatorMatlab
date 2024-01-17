classdef obstacle < hgsetget
properties
        nb_cylinder
        nb_paralepipede
        nb_sphere
        cilindre
        sphere
        cube
        plan_incl
        cilindre_y_z
        paraboloid_y_z
        
end
methods
        %builder
        function obj = obstacle%(center_x,center_y,center_z,radius,height)
            obj.nb_cylinder=0;
            obj.nb_paralepipede=0;
            obj.nb_sphere=0;
            
            %obj.cilindre = object_cylinder(0,0,0,1,1);
        end
        function cil(obj)
            obj.cilindre = object_cylinder(0,0,0,1,1);
        end
        function cub(obj)
            obj.cube=object_parale(0,0,0,1,1,1);
        end
        function sph (obj)
            obj.sphere=object_sphere(0,0,0,1); 
        end
        function pl_inclin (obj)
            %(y_0,angle,side_x,side_y,side_z,center_x,center_y,center_z)
            obj.plan_incl=object_plan_i(0,1,1,1,1,1,1); 
        end
        
        function parabola(obj)
           obj.paraboloid_y_z=object_paraboloide_y_z(0,0,0,1,1,1);
        end
        
        function cil_y_z(obj)
        
            obj.cilindre_y_z=object_cylinder_y_z(0,0,0,1,1); 
        end
        
        
        function cylinder(obj,i,center_x,center_y,center_z,radius,height)
            
            %obj.cilindre(i)=cast(obj.cilindre(i),'object_cylinder');
           
            obj.cilindre(i)=object_cylinder (center_x,center_y,center_z,radius,height);
            %obj.cilindro=radius;
        end
        
        function spher(obj,i,center_x,center_y,center_z,radius)
            obj.sphere(i)=object_sphere(center_x,center_y,center_z,radius);
        end
        
        function parale(obj,i,center_x,center_y,center_z,side_x,side_y,side_z)
            obj.cube(i)=object_parale(center_x,center_y,center_z,side_x,side_y,side_z);
            
        end
        
        function inclined_plan (obj,i,angle,center_x,center_y,center_z,side_x,side_y,side_z)
            obj.plan_incl(i)=object_plan_i(angle,center_x,center_y,center_z,side_x,side_y,side_z);   
        end
        
        function cylinder_y_z(obj,i,center_x,center_y,center_z,radius,height)
           
            obj.cilindre_y_z(i)=object_cylinder_y_z (center_x,center_y,center_z,radius,height);
            
        end
        
        function paraboloide_y_z(obj,i,center_x,center_y,center_z,alpha,heigth,altitude_z)
            obj.paraboloid_y_z(i)=object_paraboloide_y_z (center_x,center_y,center_z,alpha,heigth,altitude_z);
            
        end
        
end

end