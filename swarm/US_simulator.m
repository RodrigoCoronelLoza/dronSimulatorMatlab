classdef US_simulator < handle
    % This class simulates the working principle of the US sensors
    % following a peculiar round robin mode to detect obstacles
    %! Available Schemas:
    %     DOWN_ONLY = 2             ---->        6 Hz
    %     BACK_ONLY = 3             ---->        6 Hz
    %     LEFT_ONLY = 4             ---->        6 Hz
    %     RIGHT_ONLY = 5            ---->        6 Hz
    %     FRONT_ONLY = 6            ---->        6 Hz
    %     ROUND_ROBIN_FRONT = 15 (...FD-LR-FD-LB-FD-LR-FD-RB...)
    %     ROUND_ROBIN_RIGHT = 16 (...LR-FD-RB-FD-LR-FD-RB-FD...)
    %     ROUND_ROBIN_LEFT = 17  (...LR-FD-LB-FD-LR-FD-LB-FD...)
    %     ROUND_ROBIN_BACK = 18  (...LR-RB-FD-LB-LR-RB-FD-LB...)
    %     ROUND_ROBIN_ALL1 = 20  (...F-R-B-L-D...)  ---> 1.5 Hz
    %     ROUND_ROBIN_FRONT1 = 21 (...L-F-R...)     ---> 2.5 Hz
    %     ROUND_ROBIN_FrontOnly = 22                ---> 16 Hz
    
    properties
        simulationTime
        Time
        counter_dist
        anchor_selector
        counter_1
        counter_2
        counter_3
        counter_4
        counter_5
        counter_6
        counter_7
        Delta_t
        RR_mode             % structure identifying which RR mode has to be used
        array_distances
        position_ancla
        kalman_filter
        obstacleWorld
        drone_state
        drone_state_matrix
        obstAvController
        distancelock3DController
        surffollowingController
        activeSensors
        affichage
        command
        ctrl
        plots
        measure
        numDrones
        drondron
        numDistances
        counter_UWB_dist
        posicion
        velocidad
%         arrays_UWB_distances
    end
    
    methods
        %! class constructor
        function obj = US_simulator()
            
%             line_sfx1_surface_following;
            obj.Delta_t = 0 ;
            obj.simulationTime = 0 ;
            obj.Time = 1 ;
            obj.counter_dist=1;
            obj.anchor_selector=1;
            obj.counter_1=1;
            obj.counter_2=1;
            obj.counter_3=1;
            obj.counter_4=1;
            obj.counter_5=1;
            obj.counter_6=1;
            obj.counter_7=1;
            obj.affichage.video.Time = 0;
            obj.affichage.video.videoLength = 1000;
            obj.affichage.video.video_started = 0;
            obj.numDrones=10;
            obj.numDistances=5;
            obj.counter_UWB_dist=0;
            
            obj.RR_mode.Schema = 22 ;
%             obj.RR_mode.Frequency = 25 ; % Frequency of the measurements [Hz]
            obj.RR_mode.Frequency = 16 ; 
            % ! The partition depends on how the U.S. sensors are branched
            % on SPI1 and SPI2
            obj.array_distances.partition1.d1 = [] ;     % Front-----
            obj.array_distances.partition1.d2 = [] ;     % Bottom----
            obj.array_distances.partition2.d3 = [] ;     % Left______
            obj.array_distances.partition2.d4 = [] ;     % Right_____
            obj.array_distances.partition2.d5 = [] ;     % Back______
            
            obj.array_distances.d1 = [] ;     % ancla 1-----
            obj.array_distances.d2 = [] ;     % ancla 2----
            obj.array_distances.d3 = [] ;     % ancla 3______
            obj.array_distances.d4 = [] ;     % ancla 4_____
            obj.array_distances.d5 = [] ;     % ancla 5______
            obj.array_distances.d6 = [] ;     % ancla 6
            
            obj.position_ancla.a1=[1;0;0];
            obj.position_ancla.a2=[0;1;0];
            obj.position_ancla.a3=[0;0;1];
            obj.position_ancla.a4=[1;1;0];
            obj.position_ancla.a5=[1;0;1];
            obj.position_ancla.a6=[1;1;1];
            obj.position_ancla.a7=[0;1;1];
            obj.position_ancla.a8=[2;1;2];
            
            obj.posicion.x=[];
            obj.posicion.y=[];
            obj.posicion.z=[];
            
            obj.velocidad.x=[];
            obj.velocidad.y=[];
            obj.velocidad.z=[];
                   
%             obj.kalman_filter.R=0.98;
%             obj.kalman_filter.acc_max=[5;5;5];% [m/s]
%             obj.kalman_filter.delta_tk=0.005; %[s] el delta que esta
%             abajo nomas
            
            worldCreator(obj) ;
            
            obj.drone_state = drone_model ;
            obj.drone_state_matrix = drone_model;
            
            for k=1:obj.numDrones
                obj.drone_state_matrix(1,k) = drone_model;
            end
%             [0;-1.59;2.39]
            set_initial_position (obj,obj.drone_state,0,-1.59,2.39,0,0,0,0,0,0);
            set_initial_position (obj,obj.drone_state_matrix(1,1),0,2,5,0,0,0,0,0,0);
            
%             size (obj.drone_state_matrix)
            obj.obstAvController = us_control;
            
            %inicializar el controlador
            obj.surffollowingController=surfFollCtrl;
            obj.distancelock3DController=distanceLock3DCtrl;
            obj.kalman_filter=kalmanFilter;
            
            obj.activeSensors.us1 = 0 ;                  % Front-----
            obj.activeSensors.us2 = 0 ;                  % Bottom----
            obj.activeSensors.us3 = 0 ;                  % Left_____
            obj.activeSensors.us4 = 0 ;                  % Right_____
            obj.activeSensors.us5 = 0 ;                  % Back______
            
            %! Simulation affichage
            obj.affichage.constants.droneRealLength = 0.562;        % [m]
            obj.affichage.constants.droneRealWidth = 0.801;         % [m]
            obj.affichage.constants.DroneOriginX = -0.005;          % [m]
            obj.affichage.constants.DroneOriginY = 0.013;           % [m]
            obj.affichage.drone.pixels = load('drone_pixelsREAL.mat');
            drone_x = obj.affichage.drone.pixels.drone_x;
            drone_y = obj.affichage.drone.pixels.drone_y;
            obj.affichage.constants.lengthScalingFactor =...
                (max(drone_y)-min(drone_y))/obj.affichage.constants.droneRealLength;
            obj.affichage.constants.widthScalingFactor =...
                (max(drone_x)-min(drone_x))/obj.affichage.constants.droneRealWidth;
            obj.affichage.drone.pixels.drone_x =...
                obj.affichage.drone.pixels.drone_x/obj.affichage.constants.widthScalingFactor...
                +obj.affichage.constants.DroneOriginX;
            obj.affichage.drone.pixels.drone_y =...
                obj.affichage.drone.pixels.drone_y/obj.affichage.constants.lengthScalingFactor...
                +obj.affichage.constants.DroneOriginY;
            obj.affichage.drone.model = ...
                scatter(obj.affichage.drone.pixels.drone_x,obj.affichage.drone.pixels.drone_y,'.','k');
            % hacer 10 models para llenar como matriz
            obj.affichage.drone.model_matrix = ...
                scatter(obj.affichage.drone.pixels.drone_x,obj.affichage.drone.pixels.drone_y,'.','k');
            for k=1:obj.numDrones
                obj.affichage.drone.model_matrix(1,k) = ...
                scatter(obj.affichage.drone.pixels.drone_x,obj.affichage.drone.pixels.drone_y,'.','k');
            end
            definePositionUSVerteces(obj);
            obj.affichage.us1 = fill3(obj.affichage.us1Verteces(1,:),...
                obj.affichage.us1Verteces(2,:),obj.affichage.us1Verteces(3,:),...
                'black','Visible','off','FaceAlpha',0.5);
            obj.affichage.us2 = fill3(obj.affichage.us2Verteces(1,:),...
                obj.affichage.us2Verteces(2,:),obj.affichage.us2Verteces(3,:),...
                'black','Visible','off','FaceAlpha',0.5);
            obj.affichage.us3 = fill3(obj.affichage.us3Verteces(1,:),...
                obj.affichage.us3Verteces(2,:),obj.affichage.us3Verteces(3,:),...
                'black','Visible','off','FaceAlpha',0.5);
            obj.affichage.us4 = fill3(obj.affichage.us4Verteces(1,:),...
                obj.affichage.us4Verteces(2,:),obj.affichage.us4Verteces(3,:),...
                'black','Visible','off','FaceAlpha',0.5);
            obj.affichage.us5 = fill3(obj.affichage.us5Verteces(1,:),...
                obj.affichage.us5Verteces(2,:),obj.affichage.us5Verteces(3,:),...
                'black','Visible','off','FaceAlpha',0.5);
            % % % % % % % % %             obj.affichage.drone.pixels = load('drone_pixelsREAL.mat');
            % % % % % % % % %             ax = axes('Position',[.41 .38 .1 .1],'XLim',[-2 2],'YLim',[-2 2]);
            %             obj.affichage.drone.model = ...
            %                 scatter3(obj.affichage.drone.pixels.drone_x,obj.affichage.drone.pixels.drone_y,...
            %                 zeros(size(obj.affichage.drone.pixels.drone_x)),'.','k');
            % % % % % % % % %             obj.affichage.drone.model = ...
            % % % % % % % % %                 scatter(obj.affichage.drone.pixels.drone_x,obj.affichage.drone.pixels.drone_y,'.','k');
            % % % % % % % % %
            % % % % % % % % %
            initializeGUI(obj);
            
            obj.command.phiRef = 0;
            obj.command.thetaRef = 0;
            obj.command.psiRef = 0;
            obj.command.zRef=0;
            obj.command.thrust=0;
            
            obj.ctrl.rollCtrl=0;
            obj.ctrl.pitchCtrl=0;
            obj.ctrl.yawCtrl=0;
            obj.ctrl.thrustCtrl=0;
            
            obj.ctrl.thrust_prop=0;
            obj.ctrl.thrust_der=0;
            obj.ctrl.thrust_int=0;
            obj.ctrl.thrust_ff=0;
            obj.ctrl.thrust_double_int=0;
            
            obj.ctrl.vel_y_prop=0;
            obj.ctrl.vel_y_der=0;
            obj.ctrl.vel_y_int=0;
            obj.ctrl.vel_y_ff=0;
            
            obj.ctrl.yaw_prop_ctrl=0;
            obj.ctrl.yaw_der_ctrl=0;
            obj.ctrl.yaw_int_ctrl=0;
            
            obj.command.maxVelocityNed.x = 0;
            obj.command.maxVelocityNed.y = 0;
            obj.command.maxVelocityNed.z = 0;
            
            obj.plots.yawCtrl = 0 ;
            obj.plots.pitchCtrl = 0 ;
            obj.plots.rollCtrl = 0 ;
            
            obj.measure.pente=0;
            obj.measure.normal_surface_vector=[0;0;0];
            obj.measure.front=0;
            obj.measure.phi_head=0;
            obj.measure.theta_head=0;
            
        end
        
        function worldCreator(obj)
            obstaculo=obstacle;                 %!instantiation of an obstacle object
            
            %             obstaculo.cil;
            %             obstaculo.cylinder(1,9,0,0,3,10);
%                                                 obstaculo.cub;
            %                                     obstaculo.parale (1,-16.5,2.5,0,32,1,1);
%                                                 obstaculo.parale (1,0,0,-2,1,300,1);
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
            
%                         obstaculo.cil_y_z;
%                         obstaculo.cylinder_y_z(1,0,51,-10,50,5);
            obstaculo.pl_inclin;
            obstaculo.inclined_plan(1,pi/3,0,3,0,400,1,400);
%             obstaculo.inclined_plan(1,pi/4,0,3,0,400,1,400);
            %                           obstaculo.inclined_plan(1,pi/12,0,0,-3,10,10,2);
            %             obstaculo.cylinder(2,-2,2.5,0,0.5,6);
%                           obstaculo.cub;
            
%            obstaculo.parabola;
%            obstaculo.paraboloide_y_z(1,0,0,0,0.1,100,1000);  
            
            % % % % % %             obstaculo.sph
            % % % % % %             obstaculo.spher (1,0,4,0,2);
            % % % % % %             obstaculo.spher (2,0,-2,0,1);
            %obstaculo.cilindre.graph_cylinder
%                                        obstaculo.cube.graph_parale
            %             obstaculo.sphere.graph_sphere
            obstaculo.plan_incl.graph_plan_incl;
%                         obstaculo.cilindre_y_z.graph_cylinder_y_z;
%             obstaculo.paraboloid_y_z.graph_paraboloide_y_z;
            obj.obstacleWorld = obstaculo;
        end
        
        function initializeGUI(obj)
            obj.affichage.GUI.bg = uibuttongroup('Visible','off',...
                'Position',[0 0 .15 1],...
                'SelectionChangedFcn',@bselection);
            
            % Create three radio buttons in the button group.
            obj.affichage.GUI.RR_F = uicontrol(obj.affichage.GUI.bg,'Style',...
                'radiobutton',...
                'String','FRONT RR',...
                'Position',[10 170 100 30],...
                'HandleVisibility','off','Value',0);
            
            obj.affichage.GUI.RR_R = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
                'String','RIGHT RR',...
                'Position',[10 150 100 30],...
                'HandleVisibility','off','Value',0);
            
            obj.affichage.GUI.RR_L = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
                'String','LEFT RR',...
                'Position',[10 130 100 30],...
                'HandleVisibility','off','Value',0);
            
            obj.affichage.GUI.RR_B = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
                'String','BACK RR',...
                'Position',[10 110 100 30],...
                'HandleVisibility','off','Value',0);
            
            obj.affichage.GUI.RR_All = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
                'String','ALL RR',...
                'Position',[10 90 100 30],...
                'HandleVisibility','off','Value',0);
            
            obj.affichage.GUI.RR_F1 = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
                'String','FRONT(1 sensor) RR',...
                'Position',[10 70 100 30],...
                'HandleVisibility','off','Value',0);
            
            obj.affichage.GUI.RR_Fonly = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
                'String','F_ONLY RR',...
                'Position',[10 50 100 30],...
                'HandleVisibility','off','Value',1);
            
            obj.affichage.GUI.bg.Visible = 'on';
            
            %! sliders to set the angles
            obj.affichage.GUI.sliders.phi = uicontrol('style','slider',...
                'units','normalized','Position',[0.01 0.33 0.02 0.5],...
                'Min',-pi/8,'Max',pi/8,'Value',0) ;
            uicontrol('string','<html>&phi</html>','Position',[10 650 30 30])
            uicontrol('string','<html>&pi/8</html>','Position',[10 580 30 30])
            uicontrol('string','<html>-&pi/8</html>','Position',[10 200 30 30])
            obj.affichage.editSlider.phi = uicontrol('Style','edit',...
                'Position',[10 620 40 30], 'String','0');
            
            obj.affichage.GUI.sliders.theta = uicontrol('style','slider',...
                'units','normalized','Position',[0.04 0.33 0.02 0.5],...
                'Min',-pi,'Max',pi,'Value',0) ;
            uicontrol('string','<html>&theta</html>','Position',[45 650 30 30])
            uicontrol('string','<html>&pi/8</html>','Position',[45 580 30 30])
            uicontrol('string','<html>-&pi/8</html>','Position',[45 200 30 30])
            obj.affichage.editSlider.theta = uicontrol('Style','edit',...
                'Position',[50 620 40 30], 'String','0');
            
            obj.affichage.GUI.sliders.psi = uicontrol('style','slider',...
                'units','normalized','Position',[0.07 0.33 0.02 0.5],...
                'Min',-pi/2,'Max',pi/2,'Value',0) ;
            uicontrol('string','<html>&psi</html>','Position',[80 650 30 30])
            uicontrol('string','<html>&pi/2</html>','Position',[80 580 30 30])
            uicontrol('string','<html>-&pi/2</html>','Position',[80 200 30 30 ])
            obj.affichage.editSlider.psi = uicontrol('Style','edit',...
                'Position',[90 620 40 30], 'String','0');
            
            obj.affichage.GUI.sliders.z = uicontrol('style','slider',...
                'units','normalized','Position',[0.10 0.33 0.02 0.5],...
                'Min',-10,'Max',10,'Value',0) ;
            uicontrol('string','<html>z</html>','Position',[115 650 30 30])
            uicontrol('string','<html>10</html>','Position',[115 580 30 30])
            uicontrol('string','<html>-10</html>','Position',[115 200 30 30 ])
            obj.affichage.editSlider.z = uicontrol('Style','edit',...
                'Position',[130 620 40 30], 'String','0');
            
            %! Set if you want to control the euler angles by using the
            %sliders or by setting the value in the textbox
            %(default:sliders)
            obj.affichage.SlidersORvalue = uicontrol('Style', 'checkbox', 'String', 'Value',...
                'units','normalized','Position',[0.1 0.95 0.05 0.05] );
            
            obj.affichage.NotInt = uicontrol('Style', 'checkbox', 'String', 'Int',...
                'units','normalized','Position',[0.95 0.95 0.05 0.05] );
            
            obj.affichage.Controller = uicontrol('Style', 'checkbox', 'String', 'Controller',...
                'units','normalized','Position',[0.85 0.95 0.05 0.05] );
            
            %! Slider for the simulation speed
            obj.affichage.GUI.sliders.simulationSpeed = uicontrol('style','slider',...
                'units','normalized','Position',[0.130 0.33 0.02 0.5],...
                'Min',30,'Max',200,'Value',100) ;
            obj.affichage.editAngleMax = uicontrol('Style','edit',...
                'Position',[150 580 30 30],...
                'String','SimulationSpeed');
            
             obj.affichage.zoom = uicontrol('Style', 'checkbox', 'String', 'ZOOM 2x',...
                'Position',[200 20 100 50] );
            
            
            % create the data
            d = [5; 5; 5; 5; 5; 5];
            pos= [0; 0; 0];
            ctrl_angle=[0;0;0];
            euler_angle=[0;0;0];
            head=[0;0];
            
            % Create the column and row names in cell arrays
            cnames = {'[m]'};
            rnames = {'FRONT','BOTTOM','LEFT','RIGHT','BACK'};
%             rnames = {'D_ANCLA1','D_ANCLA2','D_ANCLA3','D_ANCLA4','D_ANCLA5','D_ANCLA6'};
            
            c_pos_names={'[m]'};
            r_pos_names={'X','Y','Z'};
            
            c_vel_names={'[m/s]'};
            r_vel_names={'vel_X','vel_Y','vel_Z'};
            
            c_ctrl_names={'[rad]'};
            r_ctrl_names={'<html>&psi Ctrl</html>','<html>&theta Ctrl</html>','<html>&phi Ctrl</html>','thrust Ctrl'};
            
            c_head_names={'[°]'};
            r_head_names={'theta head','phi head'};
            
            c_eul_names={'[rad]'};
            r_eul_names={'<html>&psi</html>','<html>&theta</html>','<html>&phi</html>'};
            
            % Create the uitable
            obj.affichage.t = uitable(gcf,'Data',d,...
                'ColumnName',cnames,...
                'RowName',rnames);
            
            obj.affichage.t.Position(1) = 200;
            obj.affichage.t.Position(2) = 550;
            
            obj.affichage.table_2=uitable(gcf,'Data',pos,'ColumnName',...
                c_pos_names,'RowName',r_pos_names);
            
            obj.affichage.table_2.Position(1) = 200;
            obj.affichage.table_2.Position(2) = 450;
            
            obj.affichage.table_5=uitable(gcf,'Data',head,'ColumnName',...
                c_head_names,'RowName',r_head_names);
            
            obj.affichage.table_5.Position(1) = 200;
            obj.affichage.table_5.Position(2) = 350;
            
            obj.affichage.table_6=uitable(gcf,'Data',head,'ColumnName',...
                c_vel_names,'RowName',r_vel_names);
            
            obj.affichage.table_6.Position(1) = 200;
            obj.affichage.table_6.Position(2) = 250;
            
            obj.affichage.table_3=uitable(gcf,'Data',ctrl_angle,'ColumnName',...
                c_ctrl_names,'RowName',r_ctrl_names);
            
            obj.affichage.table_3.Position(1) = 950;
            obj.affichage.table_3.Position(2) = 150;
            
            obj.affichage.table_4=uitable(gcf,'Data',euler_angle,'ColumnName',...
                c_eul_names,'RowName',r_eul_names);
            
            obj.affichage.table_4.Position(1) = 1000;
            obj.affichage.table_4.Position(2) = 50;
            
            % Set width and height
            obj.affichage.t.Position(3) = obj.affichage.t.Extent(3);
            obj.affichage.t.Position(4) = obj.affichage.t.Extent(4);
            
            obj.affichage.table_2.Position(3) = obj.affichage.table_2.Extent(3);
            obj.affichage.table_2.Position(4) = obj.affichage.table_2.Extent(4);
            
            obj.affichage.table_3.Position(3) = obj.affichage.table_3.Extent(3);
            obj.affichage.table_3.Position(4) = obj.affichage.table_3.Extent(4);
            
            obj.affichage.table_4.Position(3) = obj.affichage.table_4.Extent(3);
            obj.affichage.table_4.Position(4) = obj.affichage.table_4.Extent(4);
            
            obj.affichage.table_5.Position(3) = obj.affichage.table_5.Extent(3);
            obj.affichage.table_5.Position(4) = obj.affichage.table_5.Extent(4);
            
            obj.affichage.table_6.Position(3) = obj.affichage.table_6.Extent(3);
            obj.affichage.table_6.Position(4) = obj.affichage.table_6.Extent(4);
            
            %             obj.affichage.editAngleLeft = uicontrol('Style','edit',...
            %                 'Units','normalized',...
            %                 'Position',[.8 .9 .08 .06],...
            %                 'String','-0.95');
            %             uicontrol('string','angleLeft','Units','normalized','Position',[.9 .9 .08 .06])
            %             obj.affichage.editAngleRight = uicontrol('Style','edit',...
            %                 'Units','normalized',...
            %                 'Position',[.8 .8 .08 .06],...
            %                 'String','0.95');
            %             uicontrol('string','angleRight','Units','normalized','Position',[.9 .8 .08 .06])
            %             obj.affichage.editAngleMin = uicontrol('Style','edit',...
            %                 'Units','normalized',...
            %                 'Position',[.8 .7 .08 .06],...
            %                 'String','0.08');
            %             uicontrol('string','angleMin','Units','normalized','Position',[.9 .7 .08 .06])
            %             obj.affichage.editAngleMax = uicontrol('Style','edit',...
            %                 'Units','normalized',...
            %                 'Position',[.8 .6 .08 .06],...
            %                 'String','0.32');
            %             uicontrol('string','angleMax','Units','normalized','Position',[.9 .6 .08 .06])
            obj.affichage.editdistanceMinToObstacle = uicontrol('Style','edit',...
                'Units','normalized',...
                'Position',[.8 .5 .08 .06],...
                'String','0');
            uicontrol('string','distanceMinToObstacle','Units','normalized','Position',[.9 .5 .08 .06])
            obj.affichage.editdistanceMaxToObstacle = uicontrol('Style','edit',...
                'Units','normalized',...
                'Position',[.8 .4 .08 .06],...
                'String','5');
            uicontrol('string','distanceMaxToObstacle','Units','normalized','Position',[.9 .4 .08 .06])
            
            %! Affichage of the growing plots for the Vx and Vy
            %             axes
            
            obj.affichage.MOVIE = uicontrol('Style', 'checkbox', 'String', 'MOVIE',...
                'units','normalized','Position',[0 0.01 0.05 0.05],'Value',0 );
            obj.affichage.MOVIEstring = uicontrol('Style','edit',...
                'Units','normalized',...
                'Position',[0.05 0.01 0.05 0.05],...
                'String','5000');
            
            %             obj.affichage.GUI.ChangeConfig = uicontrol(obj.affichage.GUI.bg,'Style','radiobutton',...
            %                 'String','Modify Config',...
            %                 'Position',[10 20 100 30],...
            %                 'Value',1);
        end
        
        function definePositionUSVerteces(obj)
            obj.affichage.constants.us1Y = 0.215;
            obj.affichage.constants.us1X = 0;
            obj.affichage.constants.us1Z = -0.01;
            obj.affichage.us1Verteces =...
                [0 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us1X...
                ,-0.3 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us1X...
                ,0.3 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us1X...
                ;0 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us1Y...
                ,0.5 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us1Y...
                ,0.5 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us1Y...
                ;obj.affichage.constants.us1Z,obj.affichage.constants.us1Z,obj.affichage.constants.us1Z];
            obj.affichage.constants.us2X = 0;
            obj.affichage.constants.us2Y = 0.06;
            obj.affichage.constants.us2Z = 0.04;
            obj.affichage.us2Verteces = [-0.15,0.15,0.15,-0.15;-0.1 + obj.affichage.constants.us2Y...
                ,-0.1 + obj.affichage.constants.us2Y,0.2 + obj.affichage.constants.us2Y...
                ,0.2 + obj.affichage.constants.us2Y;-0.03 + obj.affichage.constants.us2Z...
                ,-0.03 + obj.affichage.constants.us2Z,-0.03 + obj.affichage.constants.us2Z...
                ,-0.03 + obj.affichage.constants.us2Z];
            obj.affichage.constants.us3X = -0.04;
            obj.affichage.constants.us3Y = 0.06;
            obj.affichage.constants.us3Z = 0.05;
            obj.affichage.us3Verteces =...
                [0 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us3X...
                ,-0.6 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us3X...
                ,-0.3 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us3X...
                ;0 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us3Y...
                ,0.05 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us3Y...
                ,0.45 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us3Y...
                ;obj.affichage.constants.us3Z,obj.affichage.constants.us3Z,obj.affichage.constants.us3Z];
            obj.affichage.constants.us4X = 0.04;
            obj.affichage.constants.us4Y = 0.06;
            obj.affichage.constants.us4Z = 0.05;
            obj.affichage.us4Verteces =...
                [0 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us4X...
                ,0.6 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us4X...
                ,0.3 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us4X...
                ;0 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us4Y...
                ,0.05 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us4Y...
                ,0.45 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us4Y...
                ;obj.affichage.constants.us4Z,obj.affichage.constants.us4Z,obj.affichage.constants.us4Z];
            obj.affichage.constants.us5X = 0;
            obj.affichage.constants.us5Y = -0.15;
            obj.affichage.constants.us5Z = 0.05;
            obj.affichage.us5Verteces =...
                [0 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us5X...
                ,-0.3 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us5X...
                ,0.3 + obj.affichage.constants.DroneOriginX + obj.affichage.constants.us5X...
                ;0 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us5Y...
                ,-0.5 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us5Y...
                ,-0.5 + obj.affichage.constants.DroneOriginY + obj.affichage.constants.us5Y...
                ;obj.affichage.constants.us5Z,obj.affichage.constants.us5Z,obj.affichage.constants.us5Z];
        end
        
        function selectRrSchema(obj,Schema,Frequency)
            obj.RR_mode.Schema = Schema;
            obj.RR_mode.Frequency = Frequency;
        end
        
        function set_initial_position (obj,drone_model,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,ang_x,ang_y,ang_z)
            
            drone_model.currentState.positionNed.x = pos_y;
            drone_model.currentState.positionNed.y = pos_x;
            drone_model.currentState.positionNed.z = -pos_z;
            
            drone_model.currentState.velocityNed.x = vel_y;
            drone_model.currentState.velocityNed.y = vel_x;
            drone_model.currentState.velocityNed.z = -vel_z;
            
            drone_model.currentState.angularvelocityNed.x = ang_y;
            drone_model.currentState.angularvelocityNed.y = ang_x;
            drone_model.currentState.angularvelocityNed.z = -ang_z;
            
            drone_model.previousState.positionNed.x = pos_y;
            drone_model.previousState.positionNed.y = pos_x;
            drone_model.previousState.positionNed.z = -pos_z;
            
            drone_model.previousState.velocityNed.x = vel_y;
            drone_model.previousState.velocityNed.y = vel_x;
            drone_model.previousState.velocityNed.z = -vel_z;
            
            drone_model.previousState.angularvelocityNed.x = ang_y;
            drone_model.previousState.angularvelocityNed.y = ang_x;
            drone_model.previousState.angularvelocityNed.z = -ang_z;
        end
        
        function setMeasurements(obj,d1,d2,d3,d4,d5)
            obj.array_distances.partition1.d1 = d1;         % Front
            obj.array_distances.partition1.d2 = d2;        % Bottom
            obj.array_distances.partition2.d3 = d3;         % Left
            obj.array_distances.partition2.d4 = d4;         % Right
            obj.array_distances.partition2.d5 = d5;         % Back
        end
        
         function setMeasurements_ancla(obj,i,d1,d2,d3,d4,d5,d6)
            
%             i
%             d1
            obj.array_distances.d1(i) = d1 ;     % ancla 1-----
            obj.array_distances.d2(i) = d2 ;     % ancla 2----
            obj.array_distances.d3(i) = d3 ;     % ancla 3______
            obj.array_distances.d4(i) = d4 ;     % ancla 4_____
            obj.array_distances.d5(i) = d5 ;     % ancla 5______
            obj.array_distances.d6(i) = d6 ;     % ancla 6
         end
        
         function setMeasurements_ancla_4(obj,i,d1,d2,d3,d4)
            
%             i
%             d1
            obj.array_distances.d1(i) = d1 ;     % ancla 1-----
            obj.array_distances.d2(i) = d2 ;     % ancla 2----
            obj.array_distances.d3(i) = d3 ;     % ancla 3______
            obj.array_distances.d4(i) = d4 ;     % ancla 4_____
%             obj.array_distances.d5(i) = d5 ;     % ancla 5______
%             obj.array_distances.d6(i) = d6 ;     % ancla 6
         end
        
          function setMeasurements_ancla_5(obj,i,d1,d2,d3,d4,d5)
            
%             i
%             d1
            obj.array_distances.d1(i) = d1 ;     % ancla 1-----
            obj.array_distances.d2(i) = d2 ;     % ancla 2----
            obj.array_distances.d3(i) = d3 ;     % ancla 3______
            obj.array_distances.d4(i) = d4 ;     % ancla 4_____
            obj.array_distances.d5(i) = d5 ;     % ancla 5______
%             obj.array_distances.d6(i) = d6 ;     % ancla 6
        end
        
        function setActiveSensors(obj,us1,us2,us3,us4,us5)
            obj.activeSensors.us1 = us1;                    % Front
            obj.activeSensors.us2 = us2;                    % Bottom
            obj.activeSensors.us3 = us3;                    % Left
            obj.activeSensors.us4 = us4;                    % Right
            obj.activeSensors.us5 = us5;                    % Back
        end
        
        function checkActiveSensors(obj)
            switch obj.RR_mode.Schema
                case 2                                          % Bottom
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,1,0,0,0) ;
                        case 2
                            obj.simulationTime = 0;
                    end
                    
                case 3                                          % Back
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,0,0,1) ;
                        case 2
                            obj.simulationTime = 0;
                    end
                    
                case 4                                          % Left
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,1,0,0) ;
                        case 2
                            obj.simulationTime = 0;
                    end
                    
                case 5                                          % Right
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,0,1,0) ;
                        case 2
                            obj.simulationTime = 0;
                    end
                    
                case 6                                          % Front
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,1,0,0,0,0) ;
                        case 2
                            obj.simulationTime = 0;
                    end
                    
                case 15                                        % RoundRobin_Front
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 2
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 3
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 4
                            setActiveSensors(obj,0,0,1,0,1) ;
                        case 5
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 6
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 7
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 8
                            setActiveSensors(obj,0,0,0,1,1) ;
                        case 9
                            obj.simulationTime = 0;
                    end
                    
                case 16                                        % RoundRobin_Right
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 2
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 3
                            setActiveSensors(obj,0,0,0,1,1) ;
                        case 4
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 5
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 6
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 7
                            setActiveSensors(obj,0,0,0,1,1) ;
                        case 8
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 9
                            obj.simulationTime = 0;
                    end
                    
                case 17                                        % RoundRobin_Left
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 2
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 3
                            setActiveSensors(obj,0,0,1,0,1) ;
                        case 4
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 5
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 6
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 7
                            setActiveSensors(obj,0,0,1,0,1) ;
                        case 8
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 9
                            obj.simulationTime = 0;
                    end
                    
                case 18                                        % RoundRobin_Back
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 2
                            setActiveSensors(obj,0,0,0,1,1) ;
                        case 3
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 4
                            setActiveSensors(obj,0,0,1,0,1) ;
                        case 5
                            setActiveSensors(obj,0,0,1,1,0) ;
                        case 6
                            setActiveSensors(obj,0,0,0,1,1) ;
                        case 7
                            setActiveSensors(obj,1,1,0,0,0) ;
                        case 8
                            setActiveSensors(obj,0,0,1,0,1) ;
                        case 9
                            obj.simulationTime = 0;
                    end
                    
                case 20                                        % RoundRobin_All
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,1,0,0,0,0) ;
                        case 2
                            setActiveSensors(obj,0,0,0,1,0) ;
                        case 3
                            setActiveSensors(obj,0,0,0,0,1) ;
                        case 4
                            setActiveSensors(obj,0,0,1,0,0) ;
                        case 5
                            setActiveSensors(obj,0,1,0,0,0) ;
                        case 6
                            obj.simulationTime = 0;
                    end
                    
                case 21                                        % RoundRobin_Front1
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 1
                            setActiveSensors(obj,0,0,1,0,0) ;
                        case 2
                            setActiveSensors(obj,1,0,0,0,0) ;
                        case 3
                            setActiveSensors(obj,0,0,0,1,0) ;
                        case 4
                            obj.simulationTime = 0;
                    end
                    
                case 22                                        % RoundRobin_FrontOnly
%                     encasilla22=floor(obj.simulationTime/obj.Delta_t)
                    
                    switch floor(obj.simulationTime/obj.Delta_t)
                        case 0 
                            setActiveSensors(obj,1,0,0,0,0) ;
                        case 1
                            setActiveSensors(obj,1,0,0,0,0) ;
                        case 2
                            obj.simulationTime = 0;
                    end
                    
            end
        end
        
        function validateMeasurements(obj,d1,d2,d3,d4,d5)
            %! This method validates the distances only for the u.s.
            % sensors that according to the RoundRobin schema are active at
            % the moment
%            activo=obj.activeSensors.us1
            valid_d1 = obj.activeSensors.us1*d1 ;
            valid_d2 = obj.activeSensors.us2*d2 ;
            valid_d3 = obj.activeSensors.us3*d3 ;
            valid_d4 = obj.activeSensors.us4*d4 ;
            valid_d5 = obj.activeSensors.us5*d5 ;
           
            setMeasurements(obj,valid_d1,valid_d2,valid_d3,valid_d4,valid_d5) ;
            
        end
        
        function updateGUI(obj)
            %! GUI for simulation affichage
            if get(obj.affichage.GUI.RR_F,'value')
                obj.RR_mode.Schema = 15 ;
                obj.RR_mode.Frequency = 6 ;
            elseif get(obj.affichage.GUI.RR_R,'value')
                obj.RR_mode.Schema = 16 ;
                obj.RR_mode.Frequency = 6 ;
            elseif get(obj.affichage.GUI.RR_L,'value')
                obj.RR_mode.Schema = 17 ;
                obj.RR_mode.Frequency = 6 ;
            elseif get(obj.affichage.GUI.RR_B,'value')
                obj.RR_mode.Schema = 18 ;
                obj.RR_mode.Frequency = 6 ;
            elseif get(obj.affichage.GUI.RR_All,'value')
                obj.RR_mode.Schema = 20 ;
                obj.RR_mode.Frequency = 1.5 ;
            elseif get(obj.affichage.GUI.RR_F1,'value')
                obj.RR_mode.Schema = 21 ;
                obj.RR_mode.Frequency = 2.5 ;
            elseif get(obj.affichage.GUI.RR_Fonly,'value')
                obj.RR_mode.Schema = 22 ;
                obj.RR_mode.Frequency = 16 ;
            end
            
            if get(obj.affichage.SlidersORvalue,'Value')
                obj.command.phiRef = min(max(str2double(get(obj.affichage.editSlider.phi,'String')),-pi/8),pi/8);
                set(obj.affichage.editSlider.phi,'String',num2str(obj.command.phiRef));
                set(obj.affichage.GUI.sliders.phi,'value',obj.command.phiRef);
                obj.command.thetaRef = min(max(str2double(get(obj.affichage.editSlider.theta,'String')),-pi),pi);
                set(obj.affichage.GUI.sliders.theta,'value',obj.command.thetaRef);
                set(obj.affichage.editSlider.theta,'String',num2str(obj.command.thetaRef));
                obj.command.psiRef = min(max(str2double(get(obj.affichage.editSlider.psi,'String')),-pi/2),pi/2);
                set(obj.affichage.GUI.sliders.psi,'value',obj.command.psiRef);
                set(obj.affichage.editSlider.psi,'String',num2str(obj.command.psiRef));
                obj.command.zRef = min(max(str2double(get(obj.affichage.editSlider.z,'String')),-10),10);
                set(obj.affichage.GUI.sliders.z,'value',obj.command.zRef);
                set(obj.affichage.editSlider.z,'String',num2str(obj.command.zRef));
            else
                obj.command.phiRef = get(obj.affichage.GUI.sliders.phi,'value');
                set(obj.affichage.editSlider.phi,'String',num2str(obj.command.phiRef));
                obj.command.thetaRef = get(obj.affichage.GUI.sliders.theta,'value');
                set(obj.affichage.editSlider.theta,'String',num2str(obj.command.thetaRef));
                obj.command.psiRef = get(obj.affichage.GUI.sliders.psi,'value');
                set(obj.affichage.editSlider.psi,'String',num2str(obj.command.psiRef));
                obj.command.zRef = get(obj.affichage.GUI.sliders.z,'value');
                set(obj.affichage.editSlider.z,'String',num2str(obj.command.zRef));
            end
            
            %             if get(obj.affichage.GUI.ChangeConfig,'value')
            %             obj.obstAvController.obstacleRepulsion_controller.config.angleLeft = ...
            %                 str2double(char(get(obj.affichage.editAngleLeft,'String')));
            %             obj.obstAvController.obstacleRepulsion_controller.config.angleRight = ...
            %                 str2double(char(get(obj.affichage.editAngleRight,'String')));
            %             obj.obstAvController.obstacleRepulsion_controller.config.angleMin = ...
            %                 str2double(char(get(obj.affichage.editAngleMin,'String')));
            %             obj.obstAvController.obstacleRepulsion_controller.config.angleMax = ...
            %                 str2double(char(get(obj.affichage.editAngleMax,'String')));
            
            %             surffollowingController surfaceFollowing_controller
            obj.surffollowingController.surfaceFollowing_controller.config.distanceMinToObstacle = ...
                str2double(char(get(obj.affichage.editdistanceMinToObstacle,'String')));
            
            %             obj.obstAvController.obstacleRepulsion_controller.config.distanceMinToObstacle = ...
            %                 str2double(char(get(obj.affichage.editdistanceMinToObstacle,'String')));
            
            %             obj.obstAvController.obstacleRepulsion_controller.config.distanceMaxToObstacle = ...
            %                 str2double(char(get(obj.affichage.editdistanceMaxToObstacle,'String')));
            
            obj.surffollowingController.surfaceFollowing_controller.config.distanceMaxToObstacle = ...
                str2double(char(get(obj.affichage.editdistanceMaxToObstacle,'String')));
            %             end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VIDEO%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.affichage.video.videoLength =  str2double(char(get(obj.affichage.MOVIEstring,'String')));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function updateDronePositionInSimu(obj)
            % !EXPLANATION OF THE CONVENTION! : In our simulation in matlab
            % the drone's starting position has the drone's own x axis
            % matching the matlab's y axis whereas the drone's y axis
            % corresponds to matlab x axis. As a result, when the position
            % needs to be updated on the "affichage" displacement along x
            % of the actual drone have to be added on the plot y axis
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3D %%%%%%%%%%%%%%%%%%%%%%%%
            %! Rotation matrix for phi rotation (roll)
            R_theta = [ 1  0   0 ...
                ; 0  cos(obj.drone_state.currentState.eulerAnglesRad.theta) -sin(obj.drone_state.currentState.eulerAnglesRad.theta)...
                ; 0  sin(obj.drone_state.currentState.eulerAnglesRad.theta)  cos(obj.drone_state.currentState.eulerAnglesRad.theta)];
            
            R_theta_matrix = zeros(3,3);
            
            for k=1:obj.numDrones
                R_theta_matrix (:,:,k) = [ 1  0   0 ...
                ; 0  cos(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.theta) -sin(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.theta)...
                ; 0  sin(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.theta)  cos(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.theta)];
            end
            %! Rotation matrix for theta rotation (pitch)
            R_phi = [ cos(obj.drone_state.currentState.eulerAnglesRad.phi)  0   sin(obj.drone_state.currentState.eulerAnglesRad.phi)...
                ;            0   1  0 ...
                ; -sin(obj.drone_state.currentState.eulerAnglesRad.phi) 0 cos(obj.drone_state.currentState.eulerAnglesRad.phi)];
            
            %             R_phiUS = [ cos(obj.drone_state.currentState.eulerAnglesRad.phi)  0   -sin(obj.drone_state.currentState.eulerAnglesRad.phi)...
            %                 ;            0   1  0 ...
            %                 ; sin(obj.drone_state.currentState.eulerAnglesRad.phi) 0 cos(obj.drone_state.currentState.eulerAnglesRad.phi)];
            %
            %
            R_phi_matrix = zeros(3,3);
            
            for k=1:obj.numDrones
                R_phi_matrix (:,:,k) = [ cos(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.phi)  0   sin(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.phi)...
                ;            0   1  0 ...
                ; -sin(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.phi) 0 cos(obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.phi)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2D %%%%%%%%%%%%%%%%%%%%%%%%
            %! Rotation matrix for psi rotation (yaw)
            R_psi = [cos(-obj.drone_state.currentState.eulerAnglesRad.psi) -sin(-obj.drone_state.currentState.eulerAnglesRad.psi)  0 ...
                ;sin(-obj.drone_state.currentState.eulerAnglesRad.psi)  cos(-obj.drone_state.currentState.eulerAnglesRad.psi)        0 ...
                ;             0                                                   0                                                 1];
            R_psi_matrix = zeros(3,3);
            
            for k=1:obj.numDrones
                R_psi_matrix (:,:,k) = [cos(-obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.psi) -sin(-obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.psi)  0 ...
                ;sin(-obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.psi)  cos(-obj.drone_state_matrix(1,k).currentState.eulerAnglesRad.psi)        0 ...
                ;             0                                                   0                                                 1];
            end
            %
            %             R_psiUS = [cos(obj.drone_state.currentState.eulerAnglesRad.psi-pi/2) -sin(obj.drone_state.currentState.eulerAnglesRad.psi-pi/2)  0 ...
            %                 ;sin(obj.drone_state.currentState.eulerAnglesRad.psi-pi/2)  cos(obj.drone_state.currentState.eulerAnglesRad.psi-pi/2)        0 ...
            %                 ;             0                                                   0                                                 1];
            %
            %! Global rotation matrix
            R = R_psi*R_theta*R_phi;
            
            R_matrix = zeros(3,3);
            
            for k=1:obj.numDrones
                R_matrix (:,:,k) = R_psi_matrix(:,:,k)*R_theta_matrix(:,:,k)*R_phi_matrix(:,:,k); 
            end
            %             R_us = R_psiUS*R_thetaPt*R_phiUS;
            
            %! Pick previous pixel positions
            positionDroneOld(1,:)= obj.affichage.drone.pixels.drone_x;
            positionDroneOld(2,:)=obj.affichage.drone.pixels.drone_y;
            positionDroneOld(3,:)=zeros(size(obj.affichage.drone.pixels.drone_y));
            %! Rotation
            positionDroneNow = R*positionDroneOld;
            positionDroneNow_matrix = zeros(3,23966);
            
            for k=1:obj.numDrones
                positionDroneNow_matrix (:,:,k) = R_matrix(:,:,k)*positionDroneOld; 
            end
            %! Translation
            drone_x = positionDroneNow(1,:) + obj.drone_state.currentState.positionNed.y;
            drone_y = positionDroneNow(2,:) + obj.drone_state.currentState.positionNed.x;
            drone_z = positionDroneNow(3,:) - obj.drone_state.currentState.positionNed.z;
            
            drone_x_matrix= zeros(1,23966);
            drone_y_matrix= zeros(1,23966);
            drone_z_matrix= zeros(1,23966);
            for k=1:obj.numDrones
                drone_x_matrix(:,:,k)=positionDroneNow_matrix(1,:,k) + obj.drone_state_matrix(1,k).currentState.positionNed.y;
                drone_y_matrix(:,:,k)=positionDroneNow_matrix(2,:,k) + obj.drone_state_matrix(1,k).currentState.positionNed.x;
                drone_z_matrix(:,:,k)=positionDroneNow_matrix(3,:,k) - obj.drone_state_matrix(1,k).currentState.positionNed.z;
            end
            
            %! Affichage
            delete(obj.affichage.drone.model)
            obj.affichage.drone.model = scatter3(drone_x,drone_y,drone_z,'.','k');
            for k=1:obj.numDrones
                delete(obj.affichage.drone.model_matrix(1,k))
                obj.affichage.drone.model_matrix(1,k) = ...
                scatter3(drone_x_matrix(:,:,k),drone_y_matrix(:,:,k),drone_z_matrix(:,:,k),'.','k');
            end
            %! Rotation(US)
            positionUS1 = R*obj.affichage.us1Verteces;
            positionUS2 = R*obj.affichage.us2Verteces;
            positionUS3 = R*obj.affichage.us3Verteces;
            positionUS4 = R*obj.affichage.us4Verteces;
            positionUS5 = R*obj.affichage.us5Verteces;
            %! Translation(US):
            % Along x
            us_1(1,:) = positionUS1(1,:) + obj.drone_state.currentState.positionNed.y;
            us_2(1,:) = positionUS2(1,:) + obj.drone_state.currentState.positionNed.y;
            us_3(1,:) = positionUS3(1,:) + obj.drone_state.currentState.positionNed.y;
            us_4(1,:) = positionUS4(1,:) + obj.drone_state.currentState.positionNed.y;
            us_5(1,:) = positionUS5(1,:) + obj.drone_state.currentState.positionNed.y;
            % Along y
            us_1(2,:) = positionUS1(2,:) + obj.drone_state.currentState.positionNed.x;
            us_2(2,:) = positionUS2(2,:) + obj.drone_state.currentState.positionNed.x;
            us_3(2,:) = positionUS3(2,:) + obj.drone_state.currentState.positionNed.x;
            us_4(2,:) = positionUS4(2,:) + obj.drone_state.currentState.positionNed.x;
            us_5(2,:) = positionUS5(2,:) + obj.drone_state.currentState.positionNed.x;
            % Along z
            us_1(3,:) = positionUS1(3,:) - obj.drone_state.currentState.positionNed.z;
            us_2(3,:) = positionUS2(3,:) - obj.drone_state.currentState.positionNed.z;
            us_3(3,:) = positionUS3(3,:) - obj.drone_state.currentState.positionNed.z;
            us_4(3,:) = positionUS4(3,:) - obj.drone_state.currentState.positionNed.z;
            us_5(3,:) = positionUS5(3,:) - obj.drone_state.currentState.positionNed.z;
            %! Affichage
            set(obj.affichage.us1,'XData',us_1(1,:));
            set(obj.affichage.us2,'XData',us_2(1,:));
            set(obj.affichage.us3,'XData',us_3(1,:));
            set(obj.affichage.us4,'XData',us_4(1,:));
            set(obj.affichage.us5,'XData',us_5(1,:));
            
            set(obj.affichage.us1,'YData',us_1(2,:));
            set(obj.affichage.us2,'YData',us_2(2,:));
            set(obj.affichage.us3,'YData',us_3(2,:));
            set(obj.affichage.us4,'YData',us_4(2,:));
            set(obj.affichage.us5,'YData',us_5(2,:));
            
            set(obj.affichage.us1,'ZData',us_1(3,:));
            set(obj.affichage.us2,'ZData',us_2(3,:));
            set(obj.affichage.us3,'ZData',us_3(3,:));
            set(obj.affichage.us4,'ZData',us_4(3,:));
            set(obj.affichage.us5,'ZData',us_5(3,:));
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function simulationAffichage(obj)
            %%This function is called in updateSimulation and allows to
            % graphically visualize the simulation step-by-step
            updateGUI(obj);
            
            %! Distance measurements' affichage
            %Only return a value if the measurement is < 5 meters
            d1 = min([obj.array_distances.partition1.d1;5]);
            d2 = min([obj.array_distances.partition1.d2;5]);
            d3 = min([obj.array_distances.partition2.d3;5]);
            d4 = min([obj.array_distances.partition2.d4;5]);
            d5 = min([obj.array_distances.partition2.d5;5]);
            
%             d1=min([obj.array_distances.d1;100]);
%             d2=min([obj.array_distances.d2;100]);
%             d3=min([obj.array_distances.d3;100]);
%             d4=min([obj.array_distances.d4;100]);
%             d5=min([obj.array_distances.d5;100]);
%             d6=min([obj.array_distances.d6;100]);
            
            d = [d1;d2;d3;d4;d5];
            d(d==0) = 5;
            set(obj.affichage.t,'Data',d);
            
            obj.plots.frontViewed(obj.Time) = d(1) ;
            
            %             simulationSpeed = get(obj.affichage.GUI.sliders.simulationSpeed,'Value');
            %
            %             if ~mod(obj.Time,simulationSpeed)
            %             ind=obj.Time/simulationSpeed;
            % %             obj.measure.front(ind)=d(1);
            %             end
            
            %             if (ind>51)
            %             obj.measure.d_study=obj.measure.front(end-50:end);
            %
            %             u=1;
            %             obj.measure.d_study_vrai=zeros(1,51);
            %             for v=1:51;
            %             if (obj.measure.d_study(v)~=obj.obstAvController.obstacleRepulsion_controller.config.distanceMaxToObstacle &&...
            %                     obj.measure.d_study(v)~=obj.obstAvController.obstacleRepulsion_controller.config.distanceMinToObstacle &&...
            %                      obj.measure.d_study(v)~=0)
            %                 obj.measure.d_study_vrai(u)=obj.measure.d_study(v);
            %                 u=u+1;
            %             end
            %             end
            %
            %             obj.measure.d_study_vrai=obj.measure.d_study_vrai(1:u-1);
            %
            %             x=1:length(obj.measure.d_study_vrai);
            %             x=x/10;
            %             y=obj.measure.d_study_vrai;
            %             X = [ones(length(x),1) transpose(x)];
            %             b = X\transpose(y);
            %             obj.measure.pente=b(2);
            %
            %             end
            
            obj.plots.leftViewed(obj.Time) = d(3) ;
            obj.plots.rightViewed(obj.Time) = d(4) ;
            obj.plots.backViewed(obj.Time) = d(5) ;
            
            pos= [obj.drone_state.currentState.positionNed.y;...
                obj.drone_state.currentState.positionNed.x;...
                -obj.drone_state.currentState.positionNed.z];
            
            set(obj.affichage.table_2,'Data',pos);
            
            angle_ctrl = [obj.plots.yawCtrl;obj.plots.pitchCtrl;obj.plots.rollCtrl;obj.plots.thrustCtrl];
            
            set(obj.affichage.table_3,'Data',angle_ctrl);
            
            angle_euler = [obj.drone_state.currentState.eulerAnglesRad.psi;...
                obj.drone_state.currentState.eulerAnglesRad.theta;...
                obj.drone_state.currentState.eulerAnglesRad.phi];
            
            set(obj.affichage.table_4,'Data',angle_euler);
            
            head = [obj.drone_state.currentState.anglesCamera.thetaCamRad;...
                obj.drone_state.currentState.anglesCamera.phiCamRad];
            
            set(obj.affichage.table_5,'Data',head);
            
            velocity = [obj.drone_state.currentState.velocityNed.y;...
                obj.drone_state.currentState.velocityNed.x;...
                -obj.drone_state.currentState.velocityNed.z];
            
            set(obj.affichage.table_6,'Data',velocity);
            
            obj.plots.frontViewed(obj.Time) = d(1) ;
            obj.plots.leftViewed(obj.Time) = d(3) ;
            obj.plots.rightViewed(obj.Time) = d(4) ;
            obj.plots.backViewed(obj.Time) = d(5) ;
            
%             
%             
%             xlim([-8+obj.drone_state.currentState.positionNed.y 8+obj.drone_state.currentState.positionNed.y]);
%             ylim([-8+obj.drone_state.currentState.positionNed.x 8+obj.drone_state.currentState.positionNed.x]);
%             
            
            if get(obj.affichage.zoom,'Value')
                xlim([-2+obj.drone_state.currentState.positionNed.y 2+obj.drone_state.currentState.positionNed.y]);
                ylim([-2+obj.drone_state.currentState.positionNed.x 2+obj.drone_state.currentState.positionNed.x]);
                zlim([-2-obj.drone_state.currentState.positionNed.z 2-obj.drone_state.currentState.positionNed.z]);
            else
                xlim([-8+obj.drone_state.currentState.positionNed.y 8+obj.drone_state.currentState.positionNed.y]);
                ylim([-8+obj.drone_state.currentState.positionNed.x 8+obj.drone_state.currentState.positionNed.x]);
                zlim([-8-obj.drone_state.currentState.positionNed.z 8-obj.drone_state.currentState.positionNed.z]);
            end
            
            view(90,0);
        end
        
        function setModelTime(obj,inputTime)
            obj.drone_state.simulationParameters.Ts = inputTime;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VIDEO%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function startVideo(obj)
            %! Video settings: Create a new object video
            obj.affichage.video.new_file = 0;
            obj.affichage.video.file_name = 1;
            while ~obj.affichage.video.new_file
                if ~exist(strcat('simulationVideo',num2str(obj.affichage.video.file_name),'.avi'),'file')
                    obj.affichage.writerObj = VideoWriter(strcat('simulationVideo',num2str(obj.affichage.video.file_name)));
                    open(obj.affichage.writerObj)
                    obj.affichage.video.new_file = 1;
                    obj.affichage.video.video_started = 1;
                else
                    obj.affichage.video.file_name = obj.affichage.video.file_name+1;
                end
            end
            obj.affichage.video.Time = 0;
            disp('Video Started')
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function updateSimulation(obj)
            %%%%%%%%%%%%%%%%%%%%%%
            
            SORTIE= zeros (1,5);
            try
                
                dt = toc;
%                 if (dt > 0.005)
%                   dt=0.005;
%                 end
            catch
                dt = 0;
            end
            tic;
           
%             setModelTime(obj,dt);
            %%%%%%%%%%%%%%%%%%%%%%
            simulationSpeed = get(obj.affichage.GUI.sliders.simulationSpeed,'Value');
            
            %             simulationAffichage(obj);
            %             updateDronePositionInSimu(obj);
            
            if ~mod(obj.Time,simulationSpeed)
                %             if ~mod(obj.Time,obj.Delta_t)
                simulationAffichage(obj);
                updateDronePositionInSimu(obj);
                %%%%%%%%%%%%%%%%%%%%%%VIDEO%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if (get(obj.affichage.MOVIE,'Value') && obj.affichage.video.Time <  obj.affichage.video.videoLength)
                    if ~obj.affichage.video.video_started
                        startVideo(obj)
                    else
                        obj.affichage.frame = getframe(gcf) ;
                        writeVideo(obj.affichage.writerObj,obj.affichage.frame);
                        obj.affichage.video.Time = obj.affichage.video.Time+10;
                        disp(strcat('Remaining Video Time:',num2str(obj.affichage.video.videoLength-obj.affichage.video.Time)))
                    end
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
            %! Interval between two consecutive measurements [ms]
%             TIEMPO=obj.Time
            obj.Delta_t = round(1000/obj.RR_mode.Frequency) ; % habria que definir bien este delta_t
%             obj.Delta_t = 10;
            %             delta_t=
            %! Simulation time in milliseconds
            obj.simulationTime = obj.simulationTime + 1;
            %             display(obj.simulationTime)
            %             obj.Time = obj.Time + 1;
            
            us_vector = [obj.activeSensors.us1,obj.activeSensors.us2,...
                obj.activeSensors.us3,obj.activeSensors.us4,obj.activeSensors.us5] ;
            
            %                         if ~mod(obj.simulationTime,3)
            
%             if ~mod(obj.simulationTime,obj.Delta_t)
            if (~mod(obj.Time,obj.Delta_t)||(obj.Time==1))
                
% %             if (~mod(obj.Time,126)||(obj.Time==1))
                %! Compute distances according to drone and obstacle informations
            
%             tiempo_en_act1=obj.Time
%                 posit_antes_x=obj.drone_state.currentState.positionNed.y 
%                 posit_antes_y=obj.drone_state.currentState.positionNed.x 
%                 posit_antes_z=-obj.drone_state.currentState.positionNed.z 
%                 y_llega=obj.drone_state.currentState.positionNed.x
                [d1,d2,d3,d4,d5,theta_head,phi_head,normal_surface_vector,yaw_value] = distance_estimation_v_2 (obj.obstacleWorld,obj.affichage.constants,obj.drone_state,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z ...
                    ,-(obj.drone_state.currentState.eulerAnglesRad.psi-pi/2)...
                    ,obj.drone_state.currentState.eulerAnglesRad.theta...
                    ,(obj.drone_state.currentState.eulerAnglesRad.phi+pi)) ;
                
%               
%                 y_llega=obj.drone_state.currentState.positionNed.x
%                 d1
                
                %                 display(theta_head*(180/pi));
                %                 display(phi_head*(180/pi));
                
                %                 theta_head=theta_head*(180/pi);
                %                 phi_head=phi_head*(180/pi);
                
                obj.measure.normal_surface_vector=normal_surface_vector;
                obj.drone_state.currentState.anglesCamera.phiCamRad = phi_head*(180/pi);
                obj.drone_state.currentState.anglesCamera.thetaCamRad = theta_head*(180/pi);
                
%                 obj.measure.phi_head=phi_head*(180/pi);
%                 obj.measure.theta_head=theta_head*(180/pi);
                
                obj.measure.yaw_value=yaw_value;
                
                obj.measure.front=d1;
                obj.plots.frontEffective(obj.Time) = d1;
                front=obj.plots.frontEffective(obj.Time);
                
%                 if(front~=0)
%                     obj.plots.front_graph(obj.counter_5) = front;
%                     obj.plots.ref_dist_ortho(obj.counter_5) = ...
%                         obj.surffollowingController.surfaceFollowing_controller.ref.dist_ortho_ref;
%                     obj.counter_5=obj.counter_5+1;
%                 end
                obj.plots.leftEffective(obj.Time) = d3;
                obj.plots.rightEffective(obj.Time) = d4;
                obj.plots.backEffective(obj.Time) = d5;
                
                checkActiveSensors(obj) ;
                [obj.affichage.us1, obj.affichage.us2, obj.affichage.us3, obj.affichage.us4, obj.affichage.us5] ...
                    = simuAffichage(us_vector,obj.affichage.us1, obj.affichage.us2,...
                    obj.affichage.us3, obj.affichage.us4, obj.affichage.us5) ;
                
                validateMeasurements(obj,d1,d2,d3,d4,d5) ;
                
                %%%%%%%%%%%%%%%%%CONTROL LAW PART SURFACE FOLLOWING V_1%%%%%%%%%%%%%%%%%%%%%%%%
                
                
                obj.surffollowingController = updateDistances(obj.surffollowingController,obj.array_distances.partition1.d1...
                    ,obj.array_distances.partition1.d2,obj.array_distances.partition2.d3...
                    ,obj.array_distances.partition2.d4,obj.array_distances.partition2.d5);
                              
                obj.surffollowingController = updatedrone_state(obj.surffollowingController,obj.drone_state);

                
                obj.surffollowingController = updatenormal_vector(obj.surffollowingController,obj.measure.normal_surface_vector);
                
                
                if get(obj.affichage.Controller,'Value')
                 
                    obj.surffollowingController = updateController(obj.surffollowingController);
%                 
%                 [SORTIE]=mex_sfx1_surface_following('step',d3,d1,d4,d5,normal_surface_vector(1),...
%                     normal_surface_vector(2),normal_surface_vector(3),...
%                     obj.drone_state.currentState.velocityNed.x,obj.drone_state.currentState.velocityNed.y,...
%                     obj.drone_state.currentState.velocityNed.z);
                end
                
                
                if get(obj.affichage.NotInt,'Value')
                    thrustCtrl=obj.surffollowingController.surfaceFollowing_controller.output.thrust;
                  
                else
                    thrustCtrl=obj.surffollowingController.surfaceFollowing_controller.output.thrust_w_int;
                   
                end
%                 thrustCtrl
%                 thrustCtrl=SORTIE(5);
%                thrustCtrl=obj.surffollowingController.surfaceFollowing_controller.output.thrust;
                
                thrust_prop_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentThrust_1_z;
                thrust_der_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentThrust_2_z;
                thrust_int_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentThrust_3_z;
                thrust_ff_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentThrust_ff_z;
                thrust_double_int_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentThrust_second_integral_z;
                
                y_prop_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentOut_1_y;
                y_int_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentOut_2_y;
                y_ff_Ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.CurrentOut_ff;
                
                obj.ctrl.thrust_prop=thrust_prop_Ctrl;
                obj.ctrl.thrust_der=thrust_der_Ctrl;
                obj.ctrl.thrust_int=thrust_int_Ctrl;
                obj.ctrl.thrust_ff=thrust_ff_Ctrl;
                obj.ctrl.thrust_double_int=thrust_double_int_Ctrl;
                
                obj.ctrl.vel_y_prop=y_prop_Ctrl;
                obj.ctrl.vel_y_int=y_int_Ctrl;
                obj.ctrl.vel_y_ff=y_ff_Ctrl;
            

                
                yaw_prop_ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.Current_yaw_prop;
                yaw_der_ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.Current_yaw_der;
                yaw_int_ctrl=obj.surffollowingController.surfaceFollowing_controller.inner.Current_yaw_int;
                
                obj.ctrl.yaw_prop_ctrl=yaw_prop_ctrl;
                obj.ctrl.yaw_der_ctrl=yaw_der_ctrl;
                obj.ctrl.yaw_int_ctrl=yaw_int_ctrl;
%                 
                obj.ctrl.thrustCtrl=thrustCtrl;
     
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
         
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                [yawCtrl, pitchCtrl, rollCtrl] = quat2_euler_angle([...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef.w,...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef.x,...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef.y,...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef.z]);
% 
                   [yawCtrl_2, pitchCtrl_2, rollCtrl_2] = quat2_euler_angle([...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef_yaw.w,...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef_yaw.x,...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef_yaw.y,...
                    obj.surffollowingController.surfaceFollowing_controller.output.qRef_yaw.z]);
                
 
                yawCtrl=yawCtrl+yawCtrl_2;
                pitchCtrl=pitchCtrl+pitchCtrl_2;
                rollCtrl=rollCtrl+rollCtrl_2;

                obj.ctrl.yawCtrl=yawCtrl;
                obj.ctrl.pitchCtrl=pitchCtrl;
                obj.ctrl.rollCtrl=rollCtrl;
                
%                 
                obj.plots.yawCtrl = yawCtrl;
                obj.plots.pitchCtrl = pitchCtrl;
                obj.plots.rollCtrl = rollCtrl;
                obj.plots.thrustCtrl=thrustCtrl;
                
                %%%%KAL
                if(obj.numDistances==8)
                 if  (obj.counter_dist<=8)
                    %%%%%% una vez se tenga las primeras 6 medidas de
                    %%%%%% distancias se puede hacer la trilateracion
                    %%%%%% despues solo se usa kalman filter
                    
                    [d1,d2,d3,d4,d5,d6,d7,d8] = distance_UWB_8 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                    
                    %%%%%%%%%%%%%% TRILATERACION
                    
                else
                    %%%%%%%%%%%%% KALMAN FILTER
                    %%% COMO PRiMERA Y unica vez se debe usar el valor
                    %%% inicial para el kalman filter y despues solo se usa
                    %%% el filtro recursivamente
                [d1,d2,d3,d4,d5,d6,d7,d8] = distance_UWB_8(obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                
                obj.kalman_filter=updatecurrent_delta_tk(obj.kalman_filter,obj.Delta_t);
                obj.kalman_filter=updateprevious_delta_tk(obj.kalman_filter,obj.Delta_t);     
                
                switch obj.anchor_selector
                    case 1
%                     primer_caso=100000 
                    %%% si el counter esta en 7 agarrar la posicion de la trilateracion
                    %%% caso contrario con el anterior nomas
%                     obj.position_ancla.a1
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d1);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a1);
                    
                    if (obj.counter_dist==9)
                        obj.kalman_filter=initialize_state(obj.kalman_filter,[0;-1.59;2.39]); % poner aqui el resultado de la trilateracion
                    end
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 2
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d2);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a2);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 3
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d3);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a3);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 4
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d4);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a4);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 5
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d5);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a5);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);    
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 6
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d6);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a6);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;  
                    case 7
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d7);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a7);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;   
                    case 8
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d8);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a8);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=1;
                end
                end
                end
                
                if(obj.numDistances==7)
                 if  (obj.counter_dist<=7)
                    %%%%%% una vez se tenga las primeras 6 medidas de
                    %%%%%% distancias se puede hacer la trilateracion
                    %%%%%% despues solo se usa kalman filter
                    
                    [d1,d2,d3,d4,d5,d6,d7] = distance_UWB_7 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                    
                    %%%%%%%%%%%%%% TRILATERACION
                    
                else
                    %%%%%%%%%%%%% KALMAN FILTER
                    %%% COMO PRiMERA Y unica vez se debe usar el valor
                    %%% inicial para el kalman filter y despues solo se usa
                    %%% el filtro recursivamente
                [d1,d2,d3,d4,d5,d6,d7] = distance_UWB_7 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                
                obj.kalman_filter=updatecurrent_delta_tk(obj.kalman_filter,obj.Delta_t);
                obj.kalman_filter=updateprevious_delta_tk(obj.kalman_filter,obj.Delta_t);     
                
                switch obj.anchor_selector
                    case 1
%                     primer_caso=100000 
                    %%% si el counter esta en 7 agarrar la posicion de la trilateracion
                    %%% caso contrario con el anterior nomas
%                     obj.position_ancla.a1
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d1);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a1);
                    
                    if (obj.counter_dist==8)
                        obj.kalman_filter=initialize_state(obj.kalman_filter,[0;-1.59;2.39]); % poner aqui el resultado de la trilateracion
                    end
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 2
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d2);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a2);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 3
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d3);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a3);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 4
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d4);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a4);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 5
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d5);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a5);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);    
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 6
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d6);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a6);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;  
                    case 7
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d7);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a7);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=1;    
                end
                end
                end
                
                if(obj.numDistances==6)
                if  (obj.counter_dist<=6)
                    %%%%%% una vez se tenga las primeras 6 medidas de
                    %%%%%% distancias se puede hacer la trilateracion
                    %%%%%% despues solo se usa kalman filter
                    
                    [d1,d2,d3,d4,d5,d6] = distance_UWB (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                    
                    %%%%%%%%%%%%%% TRILATERACION
                    
                else
                    %%%%%%%%%%%%% KALMAN FILTER
                    %%% COMO PRiMERA Y unica vez se debe usar el valor
                    %%% inicial para el kalman filter y despues solo se usa
                    %%% el filtro recursivamente
                [d1,d2,d3,d4,d5,d6] = distance_UWB (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                
                obj.kalman_filter=updatecurrent_delta_tk(obj.kalman_filter,obj.Delta_t);
                obj.kalman_filter=updateprevious_delta_tk(obj.kalman_filter,obj.Delta_t);     
                
                switch obj.anchor_selector
                    case 1
%                     primer_caso=100000 
                    %%% si el counter esta en 7 agarrar la posicion de la trilateracion
                    %%% caso contrario con el anterior nomas
                    obj.position_ancla.a1
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d1);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a1);
                    
                    if (obj.counter_dist==7)
                        obj.kalman_filter=initialize_state(obj.kalman_filter,[0;-1.59;2.39]); % poner aqui el resultado de la trilateracion
                    end
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 2
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d2);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a2);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 3
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d3);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a3);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 4
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d4);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a4);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 5
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d5);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a5);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);    
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 6
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d6);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a6);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=1;                               
                end
                end

%                 [EKF_pos_x,EKF_pos_y,EKF_pos_z,EKF_vel_x,EKF_vel_y,EKF_vel_z]= kalman_filter (obj.position_ancla,d1,d2,d3,d4,d5,d6,obj.kalman_filter.R,obj.kalman_filter.acc_max,...
%                     obj.Delta_t);
%                 EKF_pos_x
%                 EKF_pos_y
%                     obj.Time
                    obj.setMeasurements_ancla (obj.counter_dist,d1,d2,d3,d4,d5,d6)
                    obj.posicion.x(obj.counter_dist) = obj.drone_state.currentState.positionNed.y;
                    obj.posicion.y(obj.counter_dist) = obj.drone_state.currentState.positionNed.x;
                    obj.posicion.z(obj.counter_dist) = - obj.drone_state.currentState.positionNed.z;
            
                    obj.velocidad.x(obj.counter_dist) = obj.drone_state.currentState.velocityNed.y;
                    obj.velocidad.y(obj.counter_dist) = obj.drone_state.currentState.velocityNed.x;
                    obj.velocidad.z(obj.counter_dist) = -obj.drone_state.currentState.velocityNed.z;
                else
                    %%%
                    if(obj.numDistances==5)
                if  (obj.counter_dist<=5)
                    %%%%%% una vez se tenga las primeras 6 medidas de
                    %%%%%% distancias se puede hacer la trilateracion
                    %%%%%% despues solo se usa kalman filter
                    
                    [d1,d2,d3,d4,d5] = distance_UWB_5 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                    
                    %%%%%%%%%%%%%% TRILATERACION
                    
                else
                    %%%%%%%%%%%%% KALMAN FILTER
                    %%% COMO PRiMERA Y unica vez se debe usar el valor
                    %%% inicial para el kalman filter y despues solo se usa
                    %%% el filtro recursivamente
                [d1,d2,d3,d4,d5] = distance_UWB_5 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                
                obj.kalman_filter=updatecurrent_delta_tk(obj.kalman_filter,obj.Delta_t);
                obj.kalman_filter=updateprevious_delta_tk(obj.kalman_filter,obj.Delta_t);     
                
                switch obj.anchor_selector
                    case 1
%                     primer_caso=100000 
                    %%% si el counter esta en 7 agarrar la posicion de la trilateracion
                    %%% caso contrario con el anterior nomas
%                     obj.position_ancla.a1
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d1);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a1);
                    
                    if (obj.counter_dist==6)
                        obj.kalman_filter=initialize_state(obj.kalman_filter,[0;-1.59;2.39]); % poner aqui el resultado de la trilateracion
                    end
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 2
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d2);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a2);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 3
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d3);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a3);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 4
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d4);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a4);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 5
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d5);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a5);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);    
                    obj.anchor_selector=1;    
%                     case 6
%                     obj.kalman_filter=updatedistance(obj.kalman_filter,d6);
%                     obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a6);
%                     obj.kalman_filter=updatefilter(obj.kalman_filter);
%                     obj.anchor_selector=1;                               
                end
                end
                obj.setMeasurements_ancla_5 (obj.counter_dist,d1,d2,d3,d4,d5)
                    obj.posicion.x(obj.counter_dist) = obj.drone_state.currentState.positionNed.y;
                    obj.posicion.y(obj.counter_dist) = obj.drone_state.currentState.positionNed.x;
                    obj.posicion.z(obj.counter_dist) = - obj.drone_state.currentState.positionNed.z;
            
                    obj.velocidad.x(obj.counter_dist) = obj.drone_state.currentState.velocityNed.y;
                    obj.velocidad.y(obj.counter_dist) = obj.drone_state.currentState.velocityNed.x;
                    obj.velocidad.z(obj.counter_dist) = -obj.drone_state.currentState.velocityNed.z;
              
               else
                    
                    if(obj.numDistances==4)
                if  (obj.counter_dist<=4)
                    %%%%%% una vez se tenga las primeras 6 medidas de
                    %%%%%% distancias se puede hacer la trilateracion
                    %%%%%% despues solo se usa kalman filter
                    
                    [d1,d2,d3,d4] = distance_UWB_4 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                    
                    %%%%%%%%%%%%%% TRILATERACION
                    
                else
                    %%%%%%%%%%%%% KALMAN FILTER
                    %%% COMO PRiMERA Y unica vez se debe usar el valor
                    %%% inicial para el kalman filter y despues solo se usa
                    %%% el filtro recursivamente
                [d1,d2,d3,d4] = distance_UWB_4 (obj.position_ancla,obj.drone_state.currentState.positionNed.y ...
                    ,obj.drone_state.currentState.positionNed.x,-obj.drone_state.currentState.positionNed.z) ;
                
                obj.kalman_filter=updatecurrent_delta_tk(obj.kalman_filter,obj.Delta_t);
                obj.kalman_filter=updateprevious_delta_tk(obj.kalman_filter,obj.Delta_t);   
                
                
                
                switch obj.anchor_selector
                    case 1
%                     primer_caso=100000 
                    %%% si el counter esta en 7 agarrar la posicion de la trilateracion
                    %%% caso contrario con el anterior nomas
%                     obj.position_ancla.a1
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d1);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a1);
                    
                    if (obj.counter_dist==5)
                        obj.kalman_filter=initialize_state(obj.kalman_filter,[0;-1.59;2.39]); % poner aqui el resultado de la trilateracion
                    end
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 2
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d2);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a2);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;
                    case 3
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d3);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a3);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=obj.anchor_selector+1;    
                    case 4
                    obj.kalman_filter=updatedistance(obj.kalman_filter,d4);
                    obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a4);
                    obj.kalman_filter=updatefilter(obj.kalman_filter);
                    obj.anchor_selector=1;     
%                     case 5
%                     obj.kalman_filter=updatedistance(obj.kalman_filter,d5);
%                     obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a5);
%                     obj.kalman_filter=updatefilter(obj.kalman_filter);    
%                     obj.anchor_selector=1;   
%                     case 6
%                     obj.kalman_filter=updatedistance(obj.kalman_filter,d6);
%                     obj.kalman_filter=updateanchor_position(obj.kalman_filter,obj.position_ancla.a6);
%                     obj.kalman_filter=updatefilter(obj.kalman_filter);
%                     obj.anchor_selector=1;                               
                  
                end
                end
                 obj.setMeasurements_ancla_4 (obj.counter_dist,d1,d2,d3,d4)
                    obj.posicion.x(obj.counter_dist) = obj.drone_state.currentState.positionNed.y;
                    obj.posicion.y(obj.counter_dist) = obj.drone_state.currentState.positionNed.x;
                    obj.posicion.z(obj.counter_dist) = - obj.drone_state.currentState.positionNed.z;
            
                    obj.velocidad.x(obj.counter_dist) = obj.drone_state.currentState.velocityNed.y;
                    obj.velocidad.y(obj.counter_dist) = obj.drone_state.currentState.velocityNed.x;
                    obj.velocidad.z(obj.counter_dist) = -obj.drone_state.currentState.velocityNed.z;
                    end
                    
                    end
                end
            
              %%%%%%%%%%%%%%%%%UPDATE MODEL%%%%%%%%%%%%%%%%%%%%%%%%
                
              
            
                obj.drone_state = updateModel(obj.drone_state , obj.command.phiRef+rollCtrl...
                    ,obj.command.thetaRef+pitchCtrl,obj.command.psiRef+yawCtrl,obj.command.zRef,...
                    obj.command.thrust+thrustCtrl,[0 0]);
            
                obj.counter_dist= obj.counter_dist+1;  
                
%             end
%                 obj.counter_UWB_dist=obj.counter_UWB_dist+1;
            else
                
            if ~mod(obj.Time,5)    
%             if ~mod(obj.simulationTime,9)
                 
%             tiempo_en_act2=obj.Time
                obj.drone_state = updateModel(obj.drone_state , obj.command.phiRef+obj.ctrl.rollCtrl,...
                    obj.command.thetaRef+obj.ctrl.pitchCtrl,obj.command.psiRef+obj.ctrl.yawCtrl,obj.command.zRef,...
                    obj.command.thrust+obj.ctrl.thrustCtrl+0,[0 0]);
                
%                 obj.drone_state = updateModel_2(obj.drone_state , obj.command.phiRef+obj.ctrl.rollCtrl,...
%                     obj.command.thetaRef+obj.ctrl.pitchCtrl,obj.command.psiRef+obj.ctrl.yawCtrl,obj.command.zRef,...
%                     obj.command.thrust+obj.ctrl.thrustCtrl+0,[0 0]);
            
            end
            end
            
            obj.plots.x_pos_kalman(obj.Time)= obj.kalman_filter.kalman_filter.inner.Current_aposteriori_state(1);
            obj.plots.y_pos_kalman(obj.Time)= obj.kalman_filter.kalman_filter.inner.Current_aposteriori_state(2);
            obj.plots.z_pos_kalman(obj.Time)= obj.kalman_filter.kalman_filter.inner.Current_aposteriori_state(3);
            obj.plots.x_vel_kalman(obj.Time)= obj.kalman_filter.kalman_filter.inner.Current_aposteriori_state(4);
            obj.plots.y_vel_kalman(obj.Time)= obj.kalman_filter.kalman_filter.inner.Current_aposteriori_state(5);
            obj.plots.z_vel_kalman(obj.Time)= obj.kalman_filter.kalman_filter.inner.Current_aposteriori_state(6);
            
            obj.plots.x_pos(obj.Time) = obj.drone_state.currentState.positionNed.y;
            obj.plots.y_pos(obj.Time) = obj.drone_state.currentState.positionNed.x;
            obj.plots.z_pos(obj.Time) = - obj.drone_state.currentState.positionNed.z;
            
            obj.plots.vel_x(obj.Time) = obj.drone_state.currentState.velocityNed.y;
            obj.plots.vel_y(obj.Time) = obj.drone_state.currentState.velocityNed.x;
            obj.plots.vel_z(obj.Time) = -obj.drone_state.currentState.velocityNed.z;
            
            obj.plots.phi(obj.Time) = obj.drone_state.currentState.eulerAnglesRad.phi;
            obj.plots.theta(obj.Time) = obj.drone_state.currentState.eulerAnglesRad.theta;
            obj.plots.psi(obj.Time) = obj.drone_state.currentState.eulerAnglesRad.psi;
            
            obj.plots.yawControl(obj.Time) = obj.ctrl.yawCtrl;
            obj.plots.pitchControl(obj.Time) = obj.ctrl.pitchCtrl;
            obj.plots.rollControl(obj.Time) = obj.ctrl.rollCtrl;
            obj.plots.thrustControl(obj.Time) = obj.ctrl.thrustCtrl;
            
            obj.plots.thrustControl_prop(obj.Time) = obj.ctrl.thrust_prop;
            obj.plots.thrustControl_der(obj.Time) = obj.ctrl.thrust_der;
            obj.plots.thrustControl_int(obj.Time) = obj.ctrl.thrust_int;
            obj.plots.thrustControl_ff(obj.Time) = obj.ctrl.thrust_ff;
            obj.plots.thrustControl_double_integral(obj.Time) =obj.ctrl.thrust_double_int;
            
            obj.plots.vel_y_Control_prop(obj.Time)=obj.ctrl.vel_y_prop;
            obj.plots.vel_y_Control_der(obj.Time)=obj.ctrl.vel_y_der;
            obj.plots.vel_y_Control_int(obj.Time)=obj.ctrl.vel_y_int;
            obj.plots.vel_y_Control_ff(obj.Time)=obj.ctrl.vel_y_ff;
            
            obj.plots.yaw_prop_ctrl(obj.Time)=obj.ctrl.yaw_prop_ctrl;
            obj.plots.yaw_der_ctrl(obj.Time) = obj.ctrl.yaw_der_ctrl;
            obj.plots.yaw_int_ctrl(obj.Time)=obj.ctrl.yaw_int_ctrl;

            obj.plots.ref_v_y(obj.Time) = ...
                obj.surffollowingController.surfaceFollowing_controller.ref.v_y_ref;
            
            obj.plots.ref_v_y_soft(obj.Time) = ...
                obj.surffollowingController.surfaceFollowing_controller.ref.v_y_ref_soft;
            
            obj.plots.ref_v_z(obj.Time) = ...
                obj.surffollowingController.surfaceFollowing_controller.ref.v_z_ref;
            
             obj.plots.ref_v_z_soft(obj.Time) = ...
                obj.surffollowingController.surfaceFollowing_controller.ref.v_z_ref_soft;
            
            obj.plots.yaw_ref(obj.Time) = ...
                obj.surffollowingController.surfaceFollowing_controller.ref.yaw_ref;
            
%             obj.plots.yaw_ref(obj.Time) = ...
%                 obj.distancelock3DController.distanceLock3DController.ref.yaw_ref;
           
            obj.plots.front_graph(obj.Time) = obj.measure.front;
            
            obj.plots.ref_dist_ortho(obj.Time) = ...
                        obj.surffollowingController.surfaceFollowing_controller.ref.dist_ortho_ref;
            
            obj.plots.ref_dist_ortho_soft(obj.Time)=...
                obj.surffollowingController.surfaceFollowing_controller.ref.dist_ortho_ref_soft;
%             obj.plots.ref_dist_ortho(obj.Time) = ...
%                         obj.distancelock3DController.distanceLock3DController.config.distance_Ref;
%                     
            obj.plots.theta_head(obj.Time)=obj.drone_state.currentState.anglesCamera.thetaCamRad;
            obj.plots.phi_head(obj.Time)=obj.drone_state.currentState.anglesCamera.phiCamRad;
            
            %%%%%%%%%%%%%%%%%%%%%%%VIDEO%%%%%%%%%%%%%%%%10%%%%%%%%%%%%%%%%%%%
            if (obj.affichage.video.Time ==  obj.affichage.video.videoLength)
                close(obj.affichage.writerObj);
                disp('Video Done')
                obj.affichage.video.Time = obj.affichage.video.Time+1;
                return
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.Time = obj.Time + 1;
            
        end
        
    end
    
end

