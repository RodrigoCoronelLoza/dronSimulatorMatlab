            obstaculo=obstacle;                 %!instantiation of an obstacle object

%                         obstaculo.cil;
%                         obstaculo.cylinder(1,9,0,0,3,10);
%                                                 obstaculo.cub;
%                                                 obstaculo.parale (1,-16.5,2.5,0,32,1,1);
%                                                 obstaculo.parale (1,0,0,-2,2,2,1);
            %             obstaculo.parale (3,-7,3,-1,4,6,6);
            %             obstaculo.parale (4,-7,-3,-1,4,6,6);
            %             obstaculo.parale (5,-7,6,-1,4,6,6);
            %
            %              obstaculo.sph
            %             obstaculo.spher (1,-4,6,0,4);
            %             obstaculo.spher (2,0,-5,0,1);
            %                         obstaculo.cilindre.graph_cylinder
            %                                      obstaculo.cube.graph_parale
            %               obstaculo.sphere.graph_sphere
            %               obstaculo.cil;
            %               obstaculo.cylinder(1,-10,15,0,13,6);

            %             obstaculo.cil_y_z;
            %             obstaculo.cylinder_y_z(1,0,11,-2,10,5);
                        obstaculo.parabola;
            obstaculo.paraboloide_y_z(1,0,0,0,1,20,50);  
%             obstaculo.pl_inclin;
%             obstaculo.inclined_plan(1,pi/3,0,3,0,50,20,100);
%                                       obstaculo.inclined_plan(1,pi/3,0,0,-3,10,2,5);
            %             obstaculo.cylinder(2,-2,2.5,0,0.5,6);
            %                           obstaculo.cub;

            %             %%%%%%%%%%%%%%Front Wall%%%%%%%%
            %                         obstaculo.parale (1,0,0.8,0,20,1,20);
            %                         obstaculo.parale (2,2,0,0,1,20,20);
            %             obstaculo.parale (3,3,6,0,3,1,6);
            %             obstaculo.parale (4,-6,6,0,3,1,6);
            %             obstaculo.parale (5,6,6,0,3,1,6);
            %             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %             %%%%%%%%%%%%%%Left Wall%%%%%%%%
            %             obstaculo.parale (6,-7,5,0,1,3,6);
            %             obstaculo.parale (7,-7,2,0,1,3,6);
            %             obstaculo.parale (8,-7,-1,0,1,3,6);
            %             obstaculo.parale (9,-7,-4,0,1,3,6);
            %             obstaculo.parale (10,-7,-6,0,1,2,6);
            %             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %             %%%%%%%%%%%%%%Right Wall%%%%%%%%
            %             obstaculo.parale (11,7,5,0,1,3,6);
            %             obstaculo.parale (12,7,2,0,1,3,6);
            %             obstaculo.parale (13,7,-1,0,1,3,6);
            %             obstaculo.parale (14,7,-4,0,1,3,6);
            %             obstaculo.parale (15,7,-6,0,1,2,6);
            %             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %             %%%%%%%%%%%%%%Back Wall%%%%%%%%
            %             obstaculo.parale (16,0,-7,0,3,1,6);
            %             obstaculo.parale (17,-3,-7,0,3,1,6);
            %             obstaculo.parale (18,3,-7,0,3,1,6);
            %             obstaculo.parale (19,-6,-7,0,3,1,6);
            %             obstaculo.parale (20,6,-7,0,3,1,6);
            %             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%                         obstaculo.sph
%                         obstaculo.spher (1,0,4,0,2);
            % % % % % %             obstaculo.spher (2,0,-2,0,1);
%             obstaculo.cilindre.graph_cylinder
%                                                    obstaculo.cube.graph_parale
%                         obstaculo.sphere.graph_sphere
%             obstaculo.plan_incl.graph_plan_incl;

obstaculo.paraboloid_y_z.graph_paraboloide_y_z;

            xlabel('X [m]');
            ylabel('Y [m]');
            zlabel('Z [m]');
            %             obstaculo.cilindre_y_z.graph_cylinder_y_z;
                        
            %             obj.obstacleWorld = obstaculo;