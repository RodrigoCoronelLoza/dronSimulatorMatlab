classdef distanceLock3DCtrl < handle
    
    properties
        % drone_model
        genericController
        %obstacleRepulsion_controller
        distanceLock3DController
    end
    
    methods
        %! class constructor
        function obj = distanceLock3DCtrl()
            %            obj.drone_model = obj2.drone_state;
            
            %! Inputs
            obj.distanceLock3DController.input.distanceFront = [] ;
%             obj.surfaceFollowing_controller.input.distanceBottom = [] ;
%             obj.surfaceFollowing_controller.input.distanceLeft = [] ;
%             obj.surfaceFollowing_controller.input.distanceRight = [] ;
%             obj.surfaceFollowing_controller.input.distanceBack = [] ;
            
            obj.distanceLock3DController.input.drone_state=[];
            obj.distanceLock3DController.input.direction_desired=[1,15,2];
%             obj.surfaceFollowing_controller.input.normal_surface_vector=[];
            
%             obj.surfaceFollowing_controller.input.yaw_value=[];%%NO
            
            %%%obtener obstaculos
            
            
            
            %%% Meter aqui las infelices constantes
            
            %! Configuration
            %!Angle discribing the direction of left seen obstacles expressed in the body frame, [rad].
            %            obj.obstacleRepulsion_controller.config.angleLeft = -0.95 ;
            %            %!Angle discribing the direction of right seen obstacles expressed in the body frame, [rad].
            %            obj.obstacleRepulsion_controller.config.angleRight = 0.95 ;
            %            obj.obstacleRepulsion_controller.config.angleMin = 0.08 ;
            %            obj.obstacleRepulsion_controller.config.angleMax = 0.1 ;
            %!Distance from whitch we desire to saturate the repulsion, [m].
            
            %%%%%%%%%%%%%%%%%%%%%%YAW CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.distanceLock3DController.config.Kp_yaw =3;%%%%%6 oficial
            obj.distanceLock3DController.config.Kp_der_yaw =1 ;
            obj.distanceLock3DController.config.Td_yaw =0.01 ;%%%%% 12 oficial 24 real
            obj.distanceLock3DController.config.Kp_int_yaw=1;
            obj.distanceLock3DController.config.Ti_yaw = 0.1 ;
            
%             obj.distanceLock3DCtrl.config.Kp_y = 0.24;%%%% 0.24 oficial
%             obj.distanceLock3DCtrl.config.Kp_int_y =1 ;
%             obj.distanceLock3DCtrl.config.Ti_y = 250 ;%%%% 250 oficial
            
            obj.distanceLock3DController.config.N_yaw = 10 ;
%             obj.distanceLock3DCtrl.config.h = 0.063 ; %[s]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.distanceLock3DController.config.Kp_z =6 ;%%%%%6 oficial
            obj.distanceLock3DController.config.Kp_der_z =1 ;
            obj.distanceLock3DController.config.Td_z =12  ;%%%%% 12 oficial 24 real
            obj.distanceLock3DController.config.Kp_int_z=1;
            obj.distanceLock3DController.config.Ti_z = 150 ;
            
            obj.distanceLock3DController.config.Kp_y =1*0.24;%%%% 0.24 oficial
            obj.distanceLock3DController.config.Kp_der_y =1;
            obj.distanceLock3DController.config.Td_y =24  ;%%%%% 12 oficial 24 real
            obj.distanceLock3DController.config.Kp_int_y =1 ;
            obj.distanceLock3DController.config.Ti_y = 150 ;%%%% 250 oficial
            
            obj.distanceLock3DController.config.N = 10 ;
            obj.distanceLock3DController.config.h = 0.063 ; %[s]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.distanceLock3DController.config.distanceMinToObstacle = 0.4 ;
            %!Distance from whitch we desire to start the repulsion, [m].
            obj.distanceLock3DController.config.distanceMaxToObstacle = 4.5;
            
            obj.distanceLock3DController.config.distance_Ref=1;
%             obj.surfaceFollowing_controller.config.velocity_ref=1;
%             obj.surfaceFollowing_controller.config.velocity_z_ref=[];
%             obj.surfaceFollowing_controller.config.velocity_y_ref=[];
            
            %! innner variables
            
            %%%%%%%%%%%%%%%%%%zzzzzzzzzzzzzz
            obj.distanceLock3DController.inner.Previous_yaw_prop=0;
            obj.distanceLock3DController.inner.Current_yaw_prop=0;
            obj.distanceLock3DController.inner.Previous_yaw_der=0;
            obj.distanceLock3DController.inner.Current_yaw_der=0;
            obj.distanceLock3DController.inner.Previous_yaw_int=0;
            obj.distanceLock3DController.inner.Current_yaw_int=0;
            obj.distanceLock3DController.inner.CurrentYawError=0;
            obj.distanceLock3DController.inner.PreviousYawError=0;
            
            obj.distanceLock3DController.inner.PreviousOut_yaw=0;
            obj.distanceLock3DController.inner.CurrentOut_yaw=0;
%             obj.distanceLock3DController.inner.CurrentVelocityError_z=0;
%             obj.distanceLock3DController.inner.PreviousVelocityError_z=0;
%             
%             %%%%%%%%%%%%%%%%%%zzzzzzzzzzzzzz
            obj.distanceLock3DController.inner.PreviousThrust_prop_z=0;
            obj.distanceLock3DController.inner.CurrentThrust_prop_z=0;
            obj.distanceLock3DController.inner.PreviousThrust_der_z=0;
            obj.distanceLock3DController.inner.CurrentThrust_der_z=0;
            obj.distanceLock3DController.inner.PreviousThrust_int_z=0;
            obj.distanceLock3DController.inner.CurrentThrust_int_z=0;
            obj.distanceLock3DController.inner.CurrentPositionError_z=0;
            obj.distanceLock3DController.inner.PreviousPositionError_z=0;
            
%             obj.surfaceFollowing_controller.inner.CurrentVelocityError_z=0;
%             obj.surfaceFollowing_controller.inner.PreviousVelocityError_z=0;
%             
%             %%%%%%%%%%%%%%%%%%yyyyyyyyyyo xxxxxxxx
%             
            obj.distanceLock3DController.inner.CurrentPositionError_y=0;
            obj.distanceLock3DController.inner.PreviousPositionError_y=0;
            obj.distanceLock3DController.inner.CurrentVelocityError_y=0;
            obj.distanceLock3DController.inner.PreviousVelocityError_y=0;
            obj.distanceLock3DController.inner.Previous_prop_y=0;
            obj.distanceLock3DController.inner.Current_prop_y=0;
            obj.distanceLock3DController.inner.Previous_der_y=0;
            obj.distanceLock3DController.inner.Current_der_y=0;
            obj.distanceLock3DController.inner.Previous_int_y=0;
            obj.distanceLock3DController.inner.Current_int_y=0;
            
            obj.distanceLock3DController.inner.PreviousOut_y=0;
            obj.distanceLock3DController.inner.CurrentOut_y=0;
            
            %! Output
            
            
            obj.distanceLock3DController.output.qRef_yaw.w = [];
            obj.distanceLock3DController.output.qRef_yaw.x = [];
            obj.distanceLock3DController.output.qRef_yaw.y = [];
            obj.distanceLock3DController.output.qRef_yaw.z = [];
            
            obj.distanceLock3DController.output.qRef.w = [];
            obj.distanceLock3DController.output.qRef.x = [];
            obj.distanceLock3DController.output.qRef.y = [];
            obj.distanceLock3DController.output.qRef.z = [];
            
%             obj.distanceLock3DController.output.thrust=[];
            obj.distanceLock3DController.output.thrust_w_int=[];
% 
%         
%             
%             obj.surfaceFollowing_controller.ref.v_y_ref=0;
%             obj.surfaceFollowing_controller.ref.v_z_ref=0;
%             obj.surfaceFollowing_controller.ref.dist_ortho_ref=0;
            obj.distanceLock3DController.ref.yaw_ref=0;
            
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
        
        %        function obj = updateDistances(obj,d1,d2,d3,d4,d5)
        %            obj.obstacleRepulsion_controller.input.distanceFront = d1 ;
        %            obj.obstacleRepulsion_controller.input.distanceBottom = d2 ;
        %            obj.obstacleRepulsion_controller.input.distanceLeft = d3 ;
        %            obj.obstacleRepulsion_controller.input.distanceRight = d4 ;
        %            obj.obstacleRepulsion_controller.input.distanceBack = d5 ;
        %        end
        
        function obj = updateDistances(obj,d1,d2,d3,d4,d5)
            obj.distanceLock3DController.distanceFront = d1 ;
            obj.distanceLock3DController.input.distanceBottom = d2 ;
            obj.distanceLock3DController.input.distanceLeft = d3 ;
            obj.distanceLock3DController.input.distanceRight = d4 ;
            obj.distanceLock3DController.input.distanceBack = d5 ;
            
        end
        
        function obj = updatedrone_state(obj,drone_state)
            obj.distanceLock3DController.input.drone_state=drone_state;
        end
        
%         function obj = updatenormal_vector(obj,normal_surface_vector)
%             obj.surfaceFollowing_controller.input.normal_surface_vector=normal_surface_vector;
%         end
                
        function obj = z_controller(obj)
            
            %display(obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.x)
            %             display(obj.surfaceFollowing_controller.input.normal_surface_vector)
            direction= obj.distanceLock3DController.input.direction_desired;
            direction= direction/norm(direction);
            angle_proj = atan(direction(3)/sqrt((direction(1)*direction(1)+direction(2)*direction(2))));
            
            z_dist_ref=obj.distanceLock3DController.config.distance_Ref*sin(angle_proj);
            z_dist_measured=obj.distanceLock3DController.distanceFront*sin(angle_proj);
            
            obj.distanceLock3DController.inner.CurrenPositionError_z=z_dist_ref-z_dist_measured;
     
            obj.distanceLock3DController.inner.CurrentVelocityError_z=...
                0-...
                (-obj.distanceLock3DController.input.drone_state.currentState.velocityNed.z);
            error_z=obj.distanceLock3DController.inner.CurrentVelocityError_z
%             obj.surfaceFollowing_controller.ref.dist_ortho_ref=...
%                 obj.surfaceFollowing_controller.config.distance_ortho_Ref;
   
       
            % proportionel
           
%             obj.surfaceFollowing_controller.inner.CurrentThrust_1_z=obj.surfaceFollowing_controller.config.Kp_z*...
%                 obj.surfaceFollowing_controller.inner.CurrentPositionError_z;
            
            obj.distanceLock3DController.inner.CurrentThrust_prop_z=obj.distanceLock3DController.config.Kp_z*obj.distanceLock3DController.inner.CurrenPositionError_z;
            
            obj.distanceLock3DController.inner.CurrentThrust_prop_z=...
                (1.4987*3)*...
                obj.distanceLock3DController.inner.CurrenPositionError_z;
         
            
            %%%% derivatif
            
            constant=obj.distanceLock3DController.config.Kp_der_z*...
                (obj.distanceLock3DController.config.Td_z/...
                (obj.distanceLock3DController.config.Td_z+...
                (obj.distanceLock3DController.config.N*...
                obj.distanceLock3DController.config.h)));
            
            obj.distanceLock3DController.inner.CurrentThrust_der_z=constant*...
                (obj.distanceLock3DController.inner.PreviousThrust_der_z+...
                obj.distanceLock3DController.config.N*...
                (obj.distanceLock3DController.inner.CurrentPositionError_z-...
                obj.distanceLock3DController.inner.PreviousPositionError_z));
             
           
                
            obj.distanceLock3DController.inner.CurrentThrust_der_z=(4.4987*1)*...
                obj.distanceLock3DController.inner.CurrentVelocityError_z;
            
            obj.distanceLock3DController.inner.CurrentThrust_int_z=...
                obj.distanceLock3DController.config.Kp_int_z*...
                (obj.distanceLock3DController.inner.PreviousThrust_int_z+...
                ((obj.distanceLock3DController.config.h/obj.distanceLock3DController.config.Ti_z)*...
                obj.distanceLock3DController.inner.CurrenPositionError_z));
            
%             feed_forward=1.5*obj.surfaceFollowing_controller.config.velocity_z_ref;
            
            obj.distanceLock3DController.output.thrust_w_int=...
                obj.distanceLock3DController.inner.CurrentThrust_prop_z...
                +obj.distanceLock3DController.inner.CurrentThrust_der_z...
                +obj.distanceLock3DController.inner.CurrentThrust_int_z;
            %+feed_forward;
            
%               obj.surfaceFollowing_controller.output.thrust=...
%                 obj.surfaceFollowing_controller.inner.CurrentThrust_1_z...
%                 +obj.surfaceFollowing_controller.inner.CurrentThrust_2_z;
%           
%             sin_integral=obj.surfaceFollowing_controller.output.thrust;
            
%             thrust_output=obj.surfaceFollowing_controller.output.thrust;
            %             EMPUJE= obj.surfaceFollowing_controller.output.thrust
%             obj.plots.thrust_1(obj.Time)=obj.surfaceFollowing_controller.inner.CurrentThrust_1_z;           
            
            EMPUJE_1=obj.distanceLock3DController.inner.CurrentThrust_prop_z;
            EMPUJE_2=obj.distanceLock3DController.inner.CurrentThrust_der_z;
            EMPUJE_3=obj.distanceLock3DController.inner.CurrentThrust_int_z;
            
      
            
            %%%%% solo hacer al final actualizaciPositionones
            obj.distanceLock3DController.inner.PreviousThrust_der_z=...
                obj.distanceLock3DController.inner.CurrentThrust_der_z;
            obj.distanceLock3DController.inner.PreviousThrust_int_z=...
                obj.distanceLock3DController.inner.CurrentThrust_int_z;
            obj.distanceLock3DController.inner.PreviousPositionError_z=...
                obj.distanceLock3DController.inner.CurrentPositionError_z;
            obj.distanceLock3DController.inner.PreviousVelocityError_z=...
                obj.distanceLock3DController.inner.CurrentVelocityError_z;
           
        end
        
        function obj = y_controller(obj)
            
            direction= obj.distanceLock3DController.input.direction_desired;
            direction= direction/norm(direction);
            angle_proj = atan(direction(3)/sqrt((direction(1)*direction(1)+direction(2)*direction(2))));
            
            y_dist_ref=obj.distanceLock3DController.config.distance_Ref*cos(angle_proj);
            y_dist_measured=obj.distanceLock3DController.distanceFront*cos(angle_proj);
            
%             v_ref=0;
            
            
            obj.distanceLock3DController.inner.CurrentVelocityError_y=...
                0-...
                obj.distanceLock3DController.input.drone_state.currentState.velocityNed.x;
            
            error_y=obj.distanceLock3DController.inner.CurrentVelocityError_y
%               obj.distanceLock3DController.config.velocity_y_ref=...
%                  obj.surfaceFollowing_controller.config.velocity_ref*cos(angle_proj);
               
%             obj.surfaceFollowing_controller.ref.v_y_ref=...
%                 obj.surfaceFollowing_controller.config.velocity_y_ref;
            
            obj.distanceLock3DController.inner.CurrentPositionError_y=y_dist_ref-y_dist_measured;
   
            
            obj.distanceLock3DController.inner.Current_prop_y=...
                obj.distanceLock3DController.config.Kp_y *...
                obj.distanceLock3DController.inner.CurrentPositionError_y;
            
            constant=obj.distanceLock3DController.config.Kp_der_y*...
                (obj.distanceLock3DController.config.Td_y/...
                (obj.distanceLock3DController.config.Td_y+...
                (obj.distanceLock3DController.config.N*...
                obj.distanceLock3DController.config.h)));
            
            obj.distanceLock3DController.inner.Current_der_y=constant*...
                (obj.distanceLock3DController.inner.Previous_der_y+...
                obj.distanceLock3DController.config.N*...
                (obj.distanceLock3DController.inner.CurrentVelocityError_y-...
                obj.distanceLock3DController.inner.PreviousVelocityError_y));
            
            obj.distanceLock3DController.inner.Current_int_y=...
                obj.distanceLock3DController.config.Kp_int_y*...
                (obj.distanceLock3DController.inner.Previous_int_y+...
                ((obj.distanceLock3DController.config.h/obj.distanceLock3DController.config.Ti_y)*...
                obj.distanceLock3DController.inner.CurrentPositionError_y));
            
            prop_y= obj.distanceLock3DController.inner.Current_prop_y
            der_y=obj.distanceLock3DController.inner.Current_der_y
            int_y=obj.distanceLock3DController.inner.Current_int_y
            
   
%             ff=obj.surfaceFollowing_controller.config.velocity_y_ref*(1.5/9.81);
            
            obj.distanceLock3DController.inner.CurrentOut_y=...
                obj.distanceLock3DController.inner.Current_prop_y+...
                obj.distanceLock3DController.inner.Current_der_y+...
                obj.distanceLock3DController.inner.Current_int_y;
%             +ff;
            
            angle_ctrl=obj.distanceLock3DController.inner.CurrentOut_y;
%             angle_ctrl
            angle_ctrl= angle_ctrl/10;
%             angle_ctrl
            angle_ctrl= max(-pi/12,min (angle_ctrl,pi/12));
            quatVector = [0,1,0];
            if norm(quatVector)
                quatVector = quatVector/norm(quatVector);
            end
             %%% pensar en la sumada acaso dar la misma
             %%% cagada??????????????????
%             obj.distanceLock3DController.output.qRef.w = obj.distanceLock3DController.output.qRef_yaw.w + cos(angle_ctrl/2);
%             obj.distanceLock3DController.output.qRef.x = obj.distanceLock3DController.output.qRef_yaw.x + sin(angle_ctrl/2)*quatVector(1);
%             obj.distanceLock3DController.output.qRef.y = obj.distanceLock3DController.output.qRef_yaw.y + sin(angle_ctrl/2)*quatVector(2);
%             obj.distanceLock3DController.output.qRef.z = obj.distanceLock3DController.output.qRef_yaw.z + sin(angle_ctrl/2)*quatVector(3);
            
            obj.distanceLock3DController.output.qRef.w = cos(angle_ctrl/2);
            obj.distanceLock3DController.output.qRef.x = sin(angle_ctrl/2)*quatVector(1);
            obj.distanceLock3DController.output.qRef.y = sin(angle_ctrl/2)*quatVector(2);
            obj.distanceLock3DController.output.qRef.z = sin(angle_ctrl/2)*quatVector(3);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            qRef = [obj.distanceLock3DController.output.qRef.w,...
                obj.distanceLock3DController.output.qRef.x,obj.distanceLock3DController.output.qRef.y,...
                obj.distanceLock3DController.output.qRef.z];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if norm(qRef)
                obj.distanceLock3DController.output.qRef.w = ...
                    obj.distanceLock3DController.output.qRef.w/norm(qRef);
                obj.distanceLock3DController.output.qRef.x = ...
                    obj.distanceLock3DController.output.qRef.x/norm(qRef);
                obj.distanceLock3DController.output.qRef.y = ...
                    obj.distanceLock3DController.output.qRef.y/norm(qRef);
                obj.distanceLock3DController.output.qRef.z = ...
                    obj.distanceLock3DController.output.qRef.z/norm(qRef);
            else
                obj.distanceLock3DController.output.qRef.w = 1;
                obj.distanceLock3DController.output.qRef.x = 0;
                obj.distanceLock3DController.output.qRef.y = 0;
                obj.distanceLock3DController.output.qRef.z = 0;
            end
            
            
            
            %display(obj.surfaceFollowing_controller.input.drone_state.currentState.velocityNed.x)
            
            %%%%%%%%%%%%%%actualizar%%%%%%%%%%%%
            obj.distanceLock3DController.inner.PreviousVelocityError_y=...
                obj.distanceLock3DController.inner.CurrentVelocityError_y;
            obj.distanceLock3DController.inner.Previous_int_y=...
                obj.distanceLock3DController.inner.Current_int_y;
            obj.distanceLock3DController.inner.PreviousOut_y=...
                obj.distanceLock3DController.inner.CurrentOut_y;
                
            
            
        end
        
        function obj = yaw_controller(obj)
            
%             quatVector = [0,0,1];
            
            direction=obj.distanceLock3DController.input.direction_desired;
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
            
            obj.distanceLock3DController.ref.yaw_ref= yaw_ref;
            
            obj.distanceLock3DController.inner.CurrentYawError=yaw_ref-...
                obj.distanceLock3DController.input.drone_state.currentState.eulerAnglesRad.psi;
            
            
            obj.distanceLock3DController.inner.Current_yaw_prop=...
                obj.distanceLock3DController.config.Kp_yaw*...
                obj.distanceLock3DController.inner.CurrentYawError;
            
            constant=obj.distanceLock3DController.config.Kp_der_yaw*...
                (obj.distanceLock3DController.config.Td_yaw/...
                (obj.distanceLock3DController.config.Td_yaw+...
                (obj.distanceLock3DController.config.N_yaw*...
                obj.distanceLock3DController.config.h)));
            
            obj.distanceLock3DController.inner.Current_yaw_der=constant*...
                (obj.distanceLock3DController.inner.Previous_yaw_der+...
                obj.distanceLock3DController.config.N_yaw*...
                (obj.distanceLock3DController.inner.CurrentYawError-...
                obj.distanceLock3DController.inner.PreviousYawError));
            
            obj.distanceLock3DController.inner.Current_yaw_int=...
                obj.distanceLock3DController.config.Kp_int_yaw*...
                (obj.distanceLock3DController.inner.Previous_yaw_int+...
                ((obj.distanceLock3DController.config.h/obj.distanceLock3DController.config.Ti_yaw)*...
                obj.distanceLock3DController.inner.CurrentYawError));
            
            obj.distanceLock3DController.inner.CurrentOut_yaw=...
                obj.distanceLock3DController.inner.Current_yaw_prop+...
                obj.distanceLock3DController.inner.Current_yaw_der+...
                obj.distanceLock3DController.inner.Current_yaw_int;
            
          
            
            angle_ctrl=obj.distanceLock3DController.inner.CurrentOut_yaw;
%             angle_ctrl
            angle_ctrl= angle_ctrl/10;
%             angle_ctrl
%             angle_ctrl= max(-pi/12,min (angle_ctrl,pi/12));
            quatVector = [0,0,1];
            if norm(quatVector)
                quatVector = quatVector/norm(quatVector);
            end
            
            obj.distanceLock3DController.output.qRef_yaw.w = cos(angle_ctrl/2);
            obj.distanceLock3DController.output.qRef_yaw.x = sin(angle_ctrl/2)*quatVector(1);
            obj.distanceLock3DController.output.qRef_yaw.y = sin(angle_ctrl/2)*quatVector(2);
            obj.distanceLock3DController.output.qRef_yaw.z = sin(angle_ctrl/2)*quatVector(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            qRef = [obj.distanceLock3DController.output.qRef_yaw.w,...
                obj.distanceLock3DController.output.qRef_yaw.x,obj.distanceLock3DController.output.qRef_yaw.y,...
                obj.distanceLock3DController.output.qRef_yaw.z];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if norm(qRef)
                obj.distanceLock3DController.output.qRef_yaw.w = ...
                    obj.distanceLock3DController.output.qRef_yaw.w/norm(qRef);
                obj.distanceLock3DController.output.qRef_yaw.x = ...
                    obj.distanceLock3DController.output.qRef_yaw.x/norm(qRef);
                obj.distanceLock3DController.output.qRef_yaw.y = ...
                    obj.distanceLock3DController.output.qRef_yaw.y/norm(qRef);
                obj.distanceLock3DController.output.qRef_yaw.z = ...
                    obj.distanceLock3DController.output.qRef_yaw.z/norm(qRef);
            else
                obj.distanceLock3DController.output.qRef_yaw.w = 1;
                obj.distanceLock3DController.output.qRef_yaw.x = 0;
                obj.distanceLock3DController.output.qRef_yaw.y = 0;
                obj.distanceLock3DController.output.qRef_yaw.z = 0;
            end
            
            
%             obj.surfaceFollowing_controller.output.qRef_yaw.w = cos(angle_ctrl/2);
%             obj.surfaceFollowing_controller.output.qRef_yaw.x = sin(angle_ctrl/2)*quatVector(1);
%             obj.surfaceFollowing_controller.output.qRef_yaw.y = sin(angle_ctrl/2)*quatVector(2);
%             obj.surfaceFollowing_controller.output.qRef_yaw.z = sin(angle_ctrl/2)*quatVector(3);
        
%             normal_down= obj.surfaceFollowing_controller.input.normal_surface_vector
            obj.distanceLock3DController.inner.PreviousYawError=...
                obj.distanceLock3DController.inner.CurrentYawError;
            
            obj.distanceLock3DController.inner.Previous_yaw_der=...
                obj.distanceLock3DController.inner.Current_yaw_der;
            
            obj.distanceLock3DController.inner.Previous_yaw_int=...
                obj.distanceLock3DController.inner.Current_yaw_int;
            
            
        end
        
        
      
        %! Make the controller run
        function obj = updateController(obj)
            %             gol=1
            %             obj.surfaceFollowing_controller.output.qRef.w = 0;
            %             obj.surfaceFollowing_controller.output.qRef.x = 0;
            %             obj.surfaceFollowing_controller.output.qRef.y = 0;
            %             obj.surfaceFollowing_controller.output.qRef.z = 0;
            
%             
            
            yaw_controller(obj);
            
            z_controller(obj);
           
            y_controller(obj);
%             distance_lock(obj);
             %             display(obj.surfaceFollowing_controller.input.drone_state.currentState.positionNed.x)
            %            setCoefficients(obj);
            %            Angle2vectorBackLeft(obj);
            %            Angle2vectorBackFront(obj);
            %            Angle2vectorBackRight(obj);
            %            Angle2vectorBackBack(obj);
            %            finalRepulsion(obj);
        end
        
        
    end
    
end
