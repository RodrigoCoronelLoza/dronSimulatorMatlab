classdef surfFollCtrl < handle
    
    properties
        % drone_model
        genericController
        %obstacleRepulsion_controller
        surfaceFollowing_controller
    end
    
    methods
        %! class constructor
        function obj = surfFollCtrl()
            %            obj.drone_model = obj2.drone_state;
            
            %! Inputs
            obj.surfaceFollowing_controller.input.distanceFront = [] ;
            obj.surfaceFollowing_controller.input.distanceBottom = [] ;
            obj.surfaceFollowing_controller.input.distanceLeft = [] ;
            obj.surfaceFollowing_controller.input.distanceRight = [] ;
            obj.surfaceFollowing_controller.input.distanceBack = [] ;
            
            obj.surfaceFollowing_controller.input.drone_state=[];
            obj.surfaceFollowing_controller.input.direction_desired=[0,1,0];
            obj.surfaceFollowing_controller.input.normal_surface_vector=[];
            
            obj.surfaceFollowing_controller.input.yaw_value=[];%%NO
            
            %%%obtener obstaculos
            
            
            
            %%% Meter aqui las infelices constantes
            
            %! Configuration
           
           
            obj.surfaceFollowing_controller.config.Kp_yaw =3;%%%%%6 oficial
            obj.surfaceFollowing_controller.config.Kp_der_yaw =1 ;
            obj.surfaceFollowing_controller.config.Td_yaw =0.01 ;%%%%% 12 oficial 24 real
            obj.surfaceFollowing_controller.config.Kp_int_yaw=1;
            obj.surfaceFollowing_controller.config.Ti_yaw = 0.1 ;
            
%             obj.distanceLock3DCtrl.config.Kp_y = 0.24;%%%% 0.24 oficial
%             obj.distanceLock3DCtrl.config.Kp_int_y =1 ;
%             obj.distanceLock3DCtrl.config.Ti_y = 250 ;%%%% 250 oficial
            
            obj.surfaceFollowing_controller.config.N_yaw = 10 ;
%             obj.distanceLock3DCtrl.config.h = 0.063 ; %[s]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.surfaceFollowing_controller.config.Kp_z =6 ;%%%%%6 oficial%%%%nada
            obj.surfaceFollowing_controller.config.Kp_der_z =1 ;%%%%%%%%%nada
            obj.surfaceFollowing_controller.config.Td_z =12  ;%%%%% nada en 12 oficial 24 real
            
            obj.surfaceFollowing_controller.config.Kp_int_z=1;
            obj.surfaceFollowing_controller.config.Ti_z = 150 ;%%%%%%%%150 con ff 5 funca mejor
            
            obj.surfaceFollowing_controller.config.Kp_int_2_z=1;
            obj.surfaceFollowing_controller.config.Ti_2_z = 150;%%%%%%%%%%5,0.1 funca bien nomas 
            
            obj.surfaceFollowing_controller.config.Kp_der_ff_z =1 ;%%%%%%%%%nada
            obj.surfaceFollowing_controller.config.Td_ff_z =2  ;%%%%% nada en 12 oficial 24
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%YYYYYYYYYYYYYYYY
            obj.surfaceFollowing_controller.config.Kp_y = 0.24;%%%% 0.24 oficial
            obj.surfaceFollowing_controller.config.Kp_int_y =1 ;
            obj.surfaceFollowing_controller.config.Ti_y = 10 ;%%%% 250 oficial
            
            obj.surfaceFollowing_controller.config.Kp_der_ff_y =1 ;%%%%%%%%%nada
            obj.surfaceFollowing_controller.config.Td_ff_y =2  ;%%%%% nada en 12 oficial 24
            
            obj.surfaceFollowing_controller.config.N = 10 ;
            obj.surfaceFollowing_controller.config.h = 0.063 ; %[s]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.surfaceFollowing_controller.config.distanceMinToObstacle = 0.4 ;
            %!Distance from whitch we desire to start the repulsion, [m].
            obj.surfaceFollowing_controller.config.distanceMaxToObstacle = 4.5;
            obj.surfaceFollowing_controller.config.distance_ortho_Ref=1;
            obj.surfaceFollowing_controller.config.Currentdistance_ortho_Ref=0;
            obj.surfaceFollowing_controller.config.Previousdistance_ortho_Ref=0;
            obj.surfaceFollowing_controller.config.velocity_ref=0.5;
            obj.surfaceFollowing_controller.config.velocity_z_ref=[];
            obj.surfaceFollowing_controller.config.velocity_y_ref=[];
            
            %! innner variables
            
            obj.surfaceFollowing_controller.inner.Previous_yaw_prop=0;
            obj.surfaceFollowing_controller.inner.Current_yaw_prop=0;
            obj.surfaceFollowing_controller.inner.Previous_yaw_der=0;
            obj.surfaceFollowing_controller.inner.Current_yaw_der=0;
            obj.surfaceFollowing_controller.inner.Previous_yaw_int=0;
            obj.surfaceFollowing_controller.inner.Current_yaw_int=0;
            obj.surfaceFollowing_controller.inner.CurrentYawError=0;
            obj.surfaceFollowing_controller.inner.PreviousYawError=0;
            
            obj.surfaceFollowing_controller.inner.PreviousOut_yaw=0;
            obj.surfaceFollowing_controller.inner.CurrentOut_yaw=0;
            
            %%%%%%%%%%%%%%%%%%zzzzzzzzzzzzzz
            
            obj.surfaceFollowing_controller.inner.PreviousVelocityzRef=0;
            obj.surfaceFollowing_controller.inner.CurrentVelocityzRef=0;
            obj.surfaceFollowing_controller.inner.PreviousVelocityzRef_soft=0;
            obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft=0;
            
            obj.surfaceFollowing_controller.inner.PreviousPositionzRef_soft=0;
            obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft=0;
            obj.surfaceFollowing_controller.inner.PreviousThrust_1_z=0;
            obj.surfaceFollowing_controller.inner.CurrentThrust_1_z=0;
            obj.surfaceFollowing_controller.inner.PreviousThrust_2_z=0;
            obj.surfaceFollowing_controller.inner.CurrentThrust_2_z=0;
            obj.surfaceFollowing_controller.inner.PreviousThrust_3_z=0;
            obj.surfaceFollowing_controller.inner.CurrentThrust_3_z=0;
            obj.surfaceFollowing_controller.inner.PreviousThrust_4_z=0;
            obj.surfaceFollowing_controller.inner.CurrentThrust_4_z=0;
            obj.surfaceFollowing_controller.inner.Previousffder_z=0;
            obj.surfaceFollowing_controller.inner.Currentffder_z=0;
            obj.surfaceFollowing_controller.inner.CurrentThrust_second_integral_z=0;
            obj.surfaceFollowing_controller.inner.PreviousThrust_second_integral_z=0;
            obj.surfaceFollowing_controller.inner.CurrentThrust_ff_z=0;
            obj.surfaceFollowing_controller.inner.CurrentPositionError_z=0;
            obj.surfaceFollowing_controller.inner.PreviousPositionError_z=0;
            obj.surfaceFollowing_controller.inner.CurrentVelocityError_z=0;
            obj.surfaceFollowing_controller.inner.PreviousVelocityError_z=0;
            
            %%%%%%%%%%%%%%%%%%yyyyyyyyyyo xxxxxxxx
            obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y_soft=0;
            obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft=0;
            obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y=0;
            obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y=0;
            
            obj.surfaceFollowing_controller.inner.CurrentVelocityError_y=0;
            obj.surfaceFollowing_controller.inner.PreviousVelocityError_y=0;
            obj.surfaceFollowing_controller.inner.PreviousOut_1_y=0;
            obj.surfaceFollowing_controller.inner.CurrentOut_1_y=0;
            obj.surfaceFollowing_controller.inner.PreviousOut_2_y=0;
            obj.surfaceFollowing_controller.inner.CurrentOut_2_y=0;
                        
            obj.surfaceFollowing_controller.inner.Previousffder_y=0;
            obj.surfaceFollowing_controller.inner.Currentffder_y=0;
            
             obj.surfaceFollowing_controller.inner.CurrentOut_ff=0;
            
            obj.surfaceFollowing_controller.inner.PreviousOut_y=0;
            obj.surfaceFollowing_controller.inner.CurrentOut_y=0;
            
            %! Output
            
%             obj.surfaceFollowing_controller.output.qRef_yaw.w = [];
%             obj.surfaceFollowing_controller.output.qRef_yaw.x = [];
%             obj.surfaceFollowing_controller.output.qRef_yaw.y = [];
%             obj.surfaceFollowing_controller.output.qRef_yaw.z = [];
%             
%             obj.surfaceFollowing_controller.output.qRef.w = [];
%             obj.surfaceFollowing_controller.output.qRef.x = [];
%             obj.surfaceFollowing_controller.output.qRef.y = [];
%             obj.surfaceFollowing_controller.output.qRef.z = [];
%             obj.surfaceFollowing_controller.output.thrust=[];
%             obj.surfaceFollowing_controller.output.thrust_w_int=[];
            
            obj.surfaceFollowing_controller.output.qRef_yaw.w = 0;
            obj.surfaceFollowing_controller.output.qRef_yaw.x = 0;
            obj.surfaceFollowing_controller.output.qRef_yaw.y = 0;
            obj.surfaceFollowing_controller.output.qRef_yaw.z = 0;
            
            obj.surfaceFollowing_controller.output.qRef.w = 0;
            obj.surfaceFollowing_controller.output.qRef.x = 0;
            obj.surfaceFollowing_controller.output.qRef.y = 0;
            obj.surfaceFollowing_controller.output.qRef.z = 0;
            obj.surfaceFollowing_controller.output.thrust=0;
            obj.surfaceFollowing_controller.output.thrust_w_int=0;

            obj.surfaceFollowing_controller.output.yaw_ctrl=0;
            
            obj.surfaceFollowing_controller.ref.v_y_ref=0;
            obj.surfaceFollowing_controller.ref.v_y_ref_soft=0;
            obj.surfaceFollowing_controller.ref.v_z_ref=0;
            obj.surfaceFollowing_controller.ref.v_z_ref_soft=0;
            obj.surfaceFollowing_controller.ref.yaw_ref=0;
            obj.surfaceFollowing_controller.ref.dist_ortho_ref=0;
            obj.surfaceFollowing_controller.ref.dist_ortho_ref_soft=0;
            
            
            %! Initialize the generic controller
            initializeController(obj);
        end
        
        function initializeController(obj)
            obj.genericController.angles.angleBackLeft = 0 ;
            obj.genericController.angles.angleBackRight = 0 ;
            obj.genericController.angles.angleBackFront = 0 ;
            obj.genericController.angles.angleBackBack = 0 ;
            
            obj.genericController.ratios.ratioLeft = 0 ;
            obj.genericController.ratios.ratioRight = 0 ;
            obj.genericController.ratios.ratioFront = 0 ;
            obj.genericController.ratios.ratioBack = 0 ;
            
            obj.genericController.vectors.vectorBackRight = [0,0,1] ;
            obj.genericController.vectors.vectorBackLeft = [0,0,1] ;
            obj.genericController.vectors.vectorBackFront = [0,0,1] ;
            obj.genericController.vectors.vectorBackBack = [0,0,1] ;
            
            obj.genericController.totalVectorBack = [0,0,1] ;
            
            %%%%%Constants de la ley de control
            obj.genericController.lambda1 = 0 ;
            obj.genericController.lambda2 = 0 ;
            obj.genericController.alpha = 0 ;
            obj.genericController.beta = 0 ;
            
            obj.genericController.A_1 = 0 ;
            obj.genericController.A_2 = 0 ;
            obj.genericController.B_1 = 0 ;
            obj.genericController.B_2 = 0 ;
            
            
        end
        
  
        function obj = updateDistances(obj,d1,d2,d3,d4,d5)
            obj.surfaceFollowing_controller.input.distanceFront = d1 ;
            obj.surfaceFollowing_controller.input.distanceBottom = d2 ;
            obj.surfaceFollowing_controller.input.distanceLeft = d3 ;
            obj.surfaceFollowing_controller.input.distanceRight = d4 ;
            obj.surfaceFollowing_controller.input.distanceBack = d5 ;
            
        end
        
        function obj = updatedrone_state(obj,drone_state)
            obj.surfaceFollowing_controller.input.drone_state=drone_state;
        end
        
        function obj = updatenormal_vector(obj,normal_surface_vector)
            obj.surfaceFollowing_controller.input.normal_surface_vector=normal_surface_vector;
        end
        

       
        function obj = z_controller(obj)
            
           
            normal_down= obj.surfaceFollowing_controller.input.normal_surface_vector;
            z_world_down=[0;0;-1];
            angle_proj = atan2(norm(cross(normal_down,z_world_down)),dot(normal_down,z_world_down));
%             angle_proj*180/pi
            
            obj.surfaceFollowing_controller.config.Currentdistance_ortho_Ref=1;
            
            wc=2*pi*0.1;
            
            
            obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft=...
                exp(-wc*obj.surfaceFollowing_controller.config.h)*...
                obj.surfaceFollowing_controller.inner.PreviousPositionzRef_soft+...
                (1-exp(-wc*obj.surfaceFollowing_controller.config.h))*...
                obj.surfaceFollowing_controller.config.Previousdistance_ortho_Ref;
            
%             mierda=(1-exp(-wc*obj.surfaceFollowing_controller.config.h))*...
%                 obj.surfaceFollowing_controller.config.Previousdistance_ortho_Ref
%             
%             valor=obj.surfaceFollowing_controller.config.Previousdistance_ortho_Ref
%             sale_filtro= obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft
%             
            obj.surfaceFollowing_controller.ref.dist_ortho_ref_soft=...
                obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft;
            
            
%             if (angle_proj~=pi/2)
                %input_Front=obj.surfaceFollowing_controller.input.distanceFront
                
                z_dist_measured=obj.surfaceFollowing_controller.input.distanceFront/cos(angle_proj);
                %coseno=cos(angle_proj)
                
                %refrencia=obj.surfaceFollowing_controller.config.distance_ortho_Ref
%                 z_dist_ref=obj.surfaceFollowing_controller.config.distance_ortho_Ref/cos(angle_proj);
%                 z_dist_ref=obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft*cos(angle_proj);
                z_dist_ref=obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft/cos(angle_proj);
%             else
%                 %cagada=656
%                 z_dist_measured=obj.surfaceFollowing_controller.input.distanceFront;
%                 
%                 z_dist_ref=obj.surfaceFollowing_controller.config.distance_ortho_Ref;
% %                 z_dist_ref=obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft;
%             end
            
            %z_dist_measured
            %z_dist_ref
            
            obj.surfaceFollowing_controller.inner.CurrenPositionError_z=z_dist_ref-z_dist_measured;
            %error_z=obj.surfaceFollowing_controller.inner.CurrenPositionError_z
            
            obj.surfaceFollowing_controller.ref.dist_ortho_ref=...
                obj.surfaceFollowing_controller.config.distance_ortho_Ref;
            
%             obj.surfaceFollowing_controller.inner.CurrenPositionError_z=z_dist_ref-...
%                         (-obj.surfaceFollowing_controller.input.drone_state.currentState.positionNed.z);
            
                    
            obj.surfaceFollowing_controller.config.velocity_z_ref=...
                    obj.surfaceFollowing_controller.config.velocity_ref*sin(angle_proj);
                
            obj.surfaceFollowing_controller.inner.CurrentVelocityzRef=...
                obj.surfaceFollowing_controller.config.velocity_z_ref;
% %             obj.surfaceFollowing_controller.config.velocity_z_ref=1;

%             wc=2*pi*0.1;
            
            
            obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft=...
                exp(-wc*obj.surfaceFollowing_controller.config.h)*...
                obj.surfaceFollowing_controller.inner.PreviousVelocityzRef_soft+...
                (1-exp(-wc*obj.surfaceFollowing_controller.config.h))*...
                obj.surfaceFollowing_controller.inner.PreviousVelocityzRef;
            
            
            obj.surfaceFollowing_controller.ref.v_z_ref_soft=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft;
            
            
            obj.surfaceFollowing_controller.ref.v_z_ref=...
                obj.surfaceFollowing_controller.config.velocity_z_ref;
            
%             obj.surfaceFollowing_controller.inner.CurrentVelocityError_z=...
%                 obj.surfaceFollowing_controller.config.velocity_z_ref-...
%                 (-obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.z);
            
%             obj.surfaceFollowing_controller.inner.CurrentVelocityError_z=...
%                 obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft-...
%                 (-obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.z);

              obj.surfaceFollowing_controller.inner.CurrentVelocityError_z=...
                -obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft-...
                (obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.z);

            
            % proportionel
            
            
            obj.surfaceFollowing_controller.inner.CurrentThrust_1_z=obj.surfaceFollowing_controller.config.Kp_z*obj.surfaceFollowing_controller.inner.CurrenPositionError_z;
           
%             obj.surfaceFollowing_controller.inner.CurrentThrust_1_z=...
%                 (1.4987*3)*...
%                 obj.surfaceFollowing_controller.inner.CurrenPositionError_z;
            
%              obj.surfaceFollowing_controller.inner.CurrentThrust_1_z=...
%                 (1.4987)*...
%                 obj.surfaceFollowing_controller.inner.CurrenPositionError_z;
            obj.surfaceFollowing_controller.inner.CurrentThrust_1_z=...
                (1.2950)*...
                obj.surfaceFollowing_controller.inner.CurrenPositionError_z;
            ERROR_POSITION=obj.surfaceFollowing_controller.inner.CurrenPositionError_z;
            
            %%%% derivatif
            
            constant=obj.surfaceFollowing_controller.config.Kp_der_z*...
                (obj.surfaceFollowing_controller.config.Td_z/...
                (obj.surfaceFollowing_controller.config.Td_z+...
                (obj.surfaceFollowing_controller.config.N*...
                obj.surfaceFollowing_controller.config.h)));
            
            obj.surfaceFollowing_controller.inner.CurrentThrust_2_z=constant*...
                (obj.surfaceFollowing_controller.inner.PreviousThrust_2_z+...
                obj.surfaceFollowing_controller.config.N*...
                (obj.surfaceFollowing_controller.inner.CurrentVelocityError_z-...
                obj.surfaceFollowing_controller.inner.PreviousVelocityError_z));
           
%                 
%             obj.surfaceFollowing_controller.inner.CurrentThrust_2_z=(4.4987)*...
%                 obj.surfaceFollowing_controller.inner.CurrentVelocityError_z;
            obj.surfaceFollowing_controller.inner.CurrentThrust_2_z=(-4.2775)*...
                obj.surfaceFollowing_controller.inner.CurrentVelocityError_z;
            
            %%%% integral
           
            obj.surfaceFollowing_controller.inner.CurrentThrust_3_z=...
                obj.surfaceFollowing_controller.config.Kp_int_z*...
                (obj.surfaceFollowing_controller.inner.PreviousThrust_3_z+...
                ((obj.surfaceFollowing_controller.config.h/obj.surfaceFollowing_controller.config.Ti_z)*...
                obj.surfaceFollowing_controller.inner.CurrenPositionError_z));
%             obj.surfaceFollowing_controller.inner.CurrentThrust_3_z=0;
            
            %%%% feed fordward
            
            constant_feed_fordward=obj.surfaceFollowing_controller.config.Kp_der_ff_z*...
                (obj.surfaceFollowing_controller.config.Td_ff_z/...
                (obj.surfaceFollowing_controller.config.Td_ff_z+...
                (obj.surfaceFollowing_controller.config.N*...
                obj.surfaceFollowing_controller.config.h)));
            
            obj.surfaceFollowing_controller.inner.Currentffder_z=constant_feed_fordward*...
                (obj.surfaceFollowing_controller.inner.Previousffder_z+...
                (obj.surfaceFollowing_controller.config.N*...
                (obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft-...
                obj.surfaceFollowing_controller.inner.PreviousVelocityzRef_soft)));
            
            feed_forward=-1.5*obj.surfaceFollowing_controller.inner.Currentffder_z-...
                1.5*obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft;
            feed_forward=-feed_forward;
%             feed_forward=0;
%             feed_forward=1.5*obj.surfaceFollowing_controller.config.velocity_z_ref;
            
            
            obj.surfaceFollowing_controller.inner.CurrentThrust_ff_z=feed_forward;
            %%%%%%%% parcial
            obj.surfaceFollowing_controller.output.thrust_w_int=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_1_z...
                +obj.surfaceFollowing_controller.inner.CurrentThrust_2_z...
                +obj.surfaceFollowing_controller.inner.CurrentThrust_3_z+feed_forward; 
            
            
            
              obj.surfaceFollowing_controller.output.thrust=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_1_z...
                +obj.surfaceFollowing_controller.inner.CurrentThrust_2_z;
            
               %%%%%%%%%%%doble integral             
           obj.surfaceFollowing_controller.inner.CurrentThrust_4_z=...
                obj.surfaceFollowing_controller.config.Kp_int_2_z*...
                (obj.surfaceFollowing_controller.inner.PreviousThrust_4_z+...
                ((obj.surfaceFollowing_controller.config.h/obj.surfaceFollowing_controller.config.Ti_2_z)*...
                obj.surfaceFollowing_controller.inner.CurrenPositionError_z)); 
               
            obj.surfaceFollowing_controller.inner.CurrentThrust_second_integral_z=...
                obj.surfaceFollowing_controller.config.Kp_int_2_z*...
                (obj.surfaceFollowing_controller.inner.PreviousThrust_second_integral_z+...
                ((obj.surfaceFollowing_controller.config.h/obj.surfaceFollowing_controller.config.Ti_2_z)*...
                (obj.surfaceFollowing_controller.inner.CurrentThrust_4_z)));
            
            obj.surfaceFollowing_controller.inner.CurrentThrust_second_integral_z=0;
            
            obj.surfaceFollowing_controller.output.thrust_w_int=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_second_integral_z+...
                obj.surfaceFollowing_controller.output.thrust_w_int;
            
            
            %%%%% updates
            obj.surfaceFollowing_controller.inner.PreviousThrust_2_z=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_2_z;
            obj.surfaceFollowing_controller.inner.PreviousThrust_3_z=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_3_z;
            obj.surfaceFollowing_controller.inner.PreviousThrust_4_z=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_4_z;
            obj.surfaceFollowing_controller.inner.PreviousThrust_second_integral_z=...
                obj.surfaceFollowing_controller.inner.CurrentThrust_second_integral_z;
            obj.surfaceFollowing_controller.inner.PreviousPositionError_z=...
                obj.surfaceFollowing_controller.inner.CurrentPositionError_z;
            obj.surfaceFollowing_controller.inner.PreviousVelocityError_z=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityError_z;
            obj.surfaceFollowing_controller.inner.PreviousVelocityzRef=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityzRef;
            obj.surfaceFollowing_controller.inner.Previousffder_z=...
                obj.surfaceFollowing_controller.inner.Currentffder_z;
            obj.surfaceFollowing_controller.inner.PreviousPositionzRef_soft=...
                obj.surfaceFollowing_controller.inner.CurrentPositionzRef_soft;
            obj.surfaceFollowing_controller.config.Previousdistance_ortho_Ref=...
                obj.surfaceFollowing_controller.config.Currentdistance_ortho_Ref;
            obj.surfaceFollowing_controller.inner.PreviousVelocityzRef_soft=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityzRef_soft;
        end
        
        function obj = y_controller(obj)
            
            normal_down= obj.surfaceFollowing_controller.input.normal_surface_vector;
            z_world_down=[0;0;-1];
            angle_proj = atan2(norm(cross(normal_down,z_world_down)),dot(normal_down,z_world_down));
%             angle_proj*180/pi
            
            obj.surfaceFollowing_controller.config.velocity_y_ref=...
               obj.surfaceFollowing_controller.config.velocity_ref*cos(angle_proj);

%             obj.surfaceFollowing_controller.config.velocity_y_ref=1;
            
            obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y=...
                obj.surfaceFollowing_controller.config.velocity_y_ref;
            
            wc=2*pi*0.1;
            
            
            obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft=...
                exp(-wc*obj.surfaceFollowing_controller.config.h)*...
                obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y_soft+...
                (1-exp(-wc*obj.surfaceFollowing_controller.config.h))*...
                obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y;
            
            
            obj.surfaceFollowing_controller.ref.v_y_ref_soft=...
                 obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft;
            
            obj.surfaceFollowing_controller.ref.v_y_ref=...
                obj.surfaceFollowing_controller.config.velocity_y_ref;
            
%             obj.surfaceFollowing_controller.inner.CurrentVelocityError_y=...
%                 obj.surfaceFollowing_controller.config.velocity_y_ref-...
%                 obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.x;
           
            obj.surfaceFollowing_controller.inner.CurrentVelocityError_y=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft-...
                obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.x;

            %prop
            obj.surfaceFollowing_controller.inner.CurrentOut_1_y=...
                obj.surfaceFollowing_controller.config.Kp_y *...
                obj.surfaceFollowing_controller.inner.CurrentVelocityError_y;
            %int
            obj.surfaceFollowing_controller.inner.CurrentOut_2_y=...
                obj.surfaceFollowing_controller.config.Kp_int_y*...
                (obj.surfaceFollowing_controller.inner.PreviousOut_2_y+...
                ((obj.surfaceFollowing_controller.config.h/obj.surfaceFollowing_controller.config.Ti_y)*...
                obj.surfaceFollowing_controller.inner.CurrentVelocityError_y));
            
            
           %feed fordward
           
           
            constant_ff=obj.surfaceFollowing_controller.config.Kp_der_ff_y*...
                (obj.surfaceFollowing_controller.config.Td_ff_y/...
                (obj.surfaceFollowing_controller.config.Td_ff_y+...
                (obj.surfaceFollowing_controller.config.N*...
                obj.surfaceFollowing_controller.config.h)));
            
            
            
            obj.surfaceFollowing_controller.inner.Currentffder_y=constant_ff*...
                (obj.surfaceFollowing_controller.inner.Previousffder_y+...
                (obj.surfaceFollowing_controller.config.N*...
                (obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft-...
                obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y_soft)));
            
            ff=(-obj.surfaceFollowing_controller.inner.Currentffder_y-...
                (0.25*obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft))/9.81; 
           
           ff=-ff;
           
%             ff=obj.surfaceFollowing_controller.config.velocity_y_ref*(1.5/9.81);
%             ff=0;
            obj.surfaceFollowing_controller.inner.CurrentOut_ff=ff;
            obj.surfaceFollowing_controller.inner.CurrentOut_y=...
                obj.surfaceFollowing_controller.inner.CurrentOut_1_y+...
                obj.surfaceFollowing_controller.inner.CurrentOut_2_y+ff;
            
            angle_ctrl=obj.surfaceFollowing_controller.inner.CurrentOut_y;
            angle_ctrl= -angle_ctrl/1;
            angle_ctrl= max(-pi/12,min (angle_ctrl,pi/12));
            quatVector = [0,1,0];
            
            if norm(quatVector)
                quatVector = quatVector/norm(quatVector);
            end
            
            obj.surfaceFollowing_controller.output.qRef.w = cos(angle_ctrl/2);
            obj.surfaceFollowing_controller.output.qRef.x = sin(angle_ctrl/2)*quatVector(1);
            obj.surfaceFollowing_controller.output.qRef.y = sin(angle_ctrl/2)*quatVector(2);
            obj.surfaceFollowing_controller.output.qRef.z = sin(angle_ctrl/2)*quatVector(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            qRef = [obj.surfaceFollowing_controller.output.qRef.w,...
                obj.surfaceFollowing_controller.output.qRef.x,obj.surfaceFollowing_controller.output.qRef.y,...
                obj.surfaceFollowing_controller.output.qRef.z];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if norm(qRef)
                obj.surfaceFollowing_controller.output.qRef.w = ...
                    obj.surfaceFollowing_controller.output.qRef.w/norm(qRef);
                obj.surfaceFollowing_controller.output.qRef.x = ...
                    obj.surfaceFollowing_controller.output.qRef.x/norm(qRef);
                obj.surfaceFollowing_controller.output.qRef.y = ...
                    obj.surfaceFollowing_controller.output.qRef.y/norm(qRef);
                obj.surfaceFollowing_controller.output.qRef.z = ...
                    obj.surfaceFollowing_controller.output.qRef.z/norm(qRef);
            else
                obj.surfaceFollowing_controller.output.qRef.w = 1;
                obj.surfaceFollowing_controller.output.qRef.x = 0;
                obj.surfaceFollowing_controller.output.qRef.y = 0;
                obj.surfaceFollowing_controller.output.qRef.z = 0;
            end
            
            
            %%%%%%%%%%%%%%update%%%%%%%%%%%%
            obj.surfaceFollowing_controller.inner.PreviousVelocityError_y=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityError_y;
            obj.surfaceFollowing_controller.inner.PreviousOut_2_y=...
                obj.surfaceFollowing_controller.inner.CurrentOut_2_y;
            obj.surfaceFollowing_controller.inner.PreviousOut_y=...
                obj.surfaceFollowing_controller.inner.CurrentOut_y;
            obj.surfaceFollowing_controller.inner.Previousffder_y=...
                obj.surfaceFollowing_controller.inner.Currentffder_y;
            obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y;
            obj.surfaceFollowing_controller.inner.PreviousVelocityRef_y_soft=...
                obj.surfaceFollowing_controller.inner.CurrentVelocityRef_y_soft;
                 
        end
        
       function obj = yaw_controller(obj)
            
            
            direction=obj.surfaceFollowing_controller.input.direction_desired;
            if norm(direction)
            direction= direction/norm(direction);
            end
            
            if (direction(2)~=0)
                yaw_ref= atan(direction(1)/direction(2));
            else
                if(direction(1)~=0)
                    yaw_ref=pi/2;
                else
                    yaw_ref=0;
                end
            end
            
            obj.surfaceFollowing_controller.ref.yaw_ref=yaw_ref;
            obj.surfaceFollowing_controller.inner.CurrentYawError=yaw_ref-...
                obj.surfaceFollowing_controller.input.drone_state.currentState.eulerAnglesRad.psi;
            
            
            obj.surfaceFollowing_controller.inner.Current_yaw_prop=...
                obj.surfaceFollowing_controller.config.Kp_yaw*...
                obj.surfaceFollowing_controller.inner.CurrentYawError;
            
            constant=obj.surfaceFollowing_controller.config.Kp_der_yaw*...
                (obj.surfaceFollowing_controller.config.Td_yaw/...
                (obj.surfaceFollowing_controller.config.Td_yaw+...
                (obj.surfaceFollowing_controller.config.N_yaw*...
                obj.surfaceFollowing_controller.config.h)));
            
            obj.surfaceFollowing_controller.inner.Current_yaw_der=constant*...
                (obj.surfaceFollowing_controller.inner.Previous_yaw_der+...
                obj.surfaceFollowing_controller.config.N_yaw*...
                (obj.surfaceFollowing_controller.inner.CurrentYawError-...
                obj.surfaceFollowing_controller.inner.PreviousYawError));
            
            obj.surfaceFollowing_controller.inner.Current_yaw_int=...
                obj.surfaceFollowing_controller.config.Kp_int_yaw*...
                (obj.surfaceFollowing_controller.inner.Previous_yaw_int+...
                ((obj.surfaceFollowing_controller.config.h/obj.surfaceFollowing_controller.config.Ti_yaw)*...
                obj.surfaceFollowing_controller.inner.CurrentYawError));
            
            obj.surfaceFollowing_controller.inner.CurrentOut_yaw=...
                obj.surfaceFollowing_controller.inner.Current_yaw_prop+...
                obj.surfaceFollowing_controller.inner.Current_yaw_der+...
                obj.surfaceFollowing_controller.inner.Current_yaw_int;
            
          
            
            angle_ctrl=obj.surfaceFollowing_controller.inner.CurrentOut_yaw;
            angle_ctrl= angle_ctrl/10;
            quatVector = [0,0,1];
            if norm(quatVector)
                quatVector = quatVector/norm(quatVector);
            end
            
            obj.surfaceFollowing_controller.output.qRef_yaw.w = cos(angle_ctrl/2);
            obj.surfaceFollowing_controller.output.qRef_yaw.x = sin(angle_ctrl/2)*quatVector(1);
            obj.surfaceFollowing_controller.output.qRef_yaw.y = sin(angle_ctrl/2)*quatVector(2);
            obj.surfaceFollowing_controller.output.qRef_yaw.z = sin(angle_ctrl/2)*quatVector(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            qRef = [obj.surfaceFollowing_controller.output.qRef_yaw.w,...
                obj.surfaceFollowing_controller.output.qRef_yaw.x,obj.surfaceFollowing_controller.output.qRef_yaw.y,...
                obj.surfaceFollowing_controller.output.qRef_yaw.z];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if norm(qRef)
                obj.surfaceFollowing_controller.output.qRef_yaw.w = ...
                    obj.surfaceFollowing_controller.output.qRef_yaw.w/norm(qRef);
                obj.surfaceFollowing_controller.output.qRef_yaw.x = ...
                    obj.surfaceFollowing_controller.output.qRef_yaw.x/norm(qRef);
                obj.surfaceFollowing_controller.output.qRef_yaw.y = ...
                    obj.surfaceFollowing_controller.output.qRef_yaw.y/norm(qRef);
                obj.surfaceFollowing_controller.output.qRef_yaw.z = ...
                    obj.surfaceFollowing_controller.output.qRef_yaw.z/norm(qRef);
            else
                obj.surfaceFollowing_controller.output.qRef_yaw.w = 1;
                obj.surfaceFollowing_controller.output.qRef_yaw.x = 0;
                obj.surfaceFollowing_controller.output.qRef_yaw.y = 0;
                obj.surfaceFollowing_controller.output.qRef_yaw.z = 0;
            end
            
            %update

            obj.surfaceFollowing_controller.inner.PreviousYawError=...
                obj.surfaceFollowing_controller.inner.CurrentYawError;
            
            obj.surfaceFollowing_controller.inner.Previous_yaw_der=...
                obj.surfaceFollowing_controller.inner.Current_yaw_der;
            
            obj.surfaceFollowing_controller.inner.Previous_yaw_int=...
                obj.surfaceFollowing_controller.inner.Current_yaw_int;
            
            
        end
   
        
        %! Make the controller run
        function obj = updateController(obj) 
            yaw_controller(obj);
            z_controller(obj);
            y_controller(obj);
        end
        
        
    end
    
end
