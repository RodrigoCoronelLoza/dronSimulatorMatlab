classdef us_control < handle
    
    properties
       % drone_model
        genericController
        obstacleRepulsion_controller   
    end
    
    methods
       %! class constructor
       function obj = us_control()
%            obj.drone_model = obj2.drone_state;
           
           %! Inputs
           obj.obstacleRepulsion_controller.input.distanceFront = [] ;    
           obj.obstacleRepulsion_controller.input.distanceBottom = [] ;
           obj.obstacleRepulsion_controller.input.distanceLeft = [] ;
           obj.obstacleRepulsion_controller.input.distanceRight = [] ;
           obj.obstacleRepulsion_controller.input.distanceBack = [] ;
           
           %! Configuration
           %!Angle discribing the direction of left seen obstacles expressed in the body frame, [rad].
           obj.obstacleRepulsion_controller.config.angleLeft = -0.95 ;
           %!Angle discribing the direction of right seen obstacles expressed in the body frame, [rad].
           obj.obstacleRepulsion_controller.config.angleRight = 0.95 ;
           obj.obstacleRepulsion_controller.config.angleMin = 0.08 ;
           obj.obstacleRepulsion_controller.config.angleMax = 0.1 ;
           %!Distance from whitch we desire to saturate the repulsion, [m].
           obj.obstacleRepulsion_controller.config.distanceMinToObstacle = 0.4 ;
           %!Distance from whitch we desire to start the repulsion, [m].
           obj.obstacleRepulsion_controller.config.distanceMaxToObstacle = 4.5;
           obj.obstacleRepulsion_controller.config.distanceRef=1;
           
           
           %! Output
           obj.obstacleRepulsion_controller.output.qRef.w = [];
           obj.obstacleRepulsion_controller.output.qRef.x = [];
           obj.obstacleRepulsion_controller.output.qRef.y = [];
           obj.obstacleRepulsion_controller.output.qRef.z = [];
           
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
           obj.obstacleRepulsion_controller.input.distanceFront = d1 ;    
           obj.obstacleRepulsion_controller.input.distanceBottom = d2 ;
           obj.obstacleRepulsion_controller.input.distanceLeft = d3 ;
           obj.obstacleRepulsion_controller.input.distanceRight = d4 ;
           obj.obstacleRepulsion_controller.input.distanceBack = d5 ;
       end
       
       function setCoefficients(obj)
           %! Computing the coefficients that determine the angle of
           %repulsion depending on the distance
           if obj.obstacleRepulsion_controller.config.distanceMinToObstacle && ...
                   obj.obstacleRepulsion_controller.config.distanceMaxToObstacle && ...
                   obj.obstacleRepulsion_controller.config.distanceMinToObstacle ...
                   ~= obj.obstacleRepulsion_controller.config.distanceMaxToObstacle
               
               obj.genericController.lambda2 = (obj.obstacleRepulsion_controller.config.angleMax ...
                   -  obj.obstacleRepulsion_controller.config.angleMin ...
                   * (obj.obstacleRepulsion_controller.config.distanceMaxToObstacle ...
                   / obj.obstacleRepulsion_controller.config.distanceMinToObstacle)) ...
                   / (1/(obj.obstacleRepulsion_controller.config.distanceMinToObstacle^2) ...
                   - 1/(obj.obstacleRepulsion_controller.config.distanceMinToObstacle ...
                   * obj.obstacleRepulsion_controller.config.distanceMaxToObstacle));
               
               obj.genericController.lambda1 = obj.obstacleRepulsion_controller.config.distanceMaxToObstacle ...
                   * obj.obstacleRepulsion_controller.config.angleMin ...
                   - obj.genericController.lambda2/obj.obstacleRepulsion_controller.config.distanceMaxToObstacle;                   
               
               obj.genericController.alpha = obj.obstacleRepulsion_controller.config.angleMax...
                   /(obj.obstacleRepulsion_controller.config.distanceMaxToObstacle-...
                   obj.obstacleRepulsion_controller.config.distanceRef)^2;
               
               obj.genericController.beta = obj.obstacleRepulsion_controller.config.angleMax...
                   /(-obj.obstacleRepulsion_controller.config.distanceRef)^2;
               
               obj.genericController.B_1 = ((obj.obstacleRepulsion_controller.config.distanceMinToObstacle*obj.obstacleRepulsion_controller.config.distanceRef)*...
                                           ((obj.obstacleRepulsion_controller.config.angleMax*obj.obstacleRepulsion_controller.config.distanceMinToObstacle)-...
                                           (obj.obstacleRepulsion_controller.config.angleMin*obj.obstacleRepulsion_controller.config.distanceRef)))/...
                                           (obj.obstacleRepulsion_controller.config.distanceRef-obj.obstacleRepulsion_controller.config.distanceMinToObstacle);
                                       
               obj.genericController.A_1 = (obj.obstacleRepulsion_controller.config.angleMax-...
                                           (obj.genericController.B_1/(obj.obstacleRepulsion_controller.config.distanceMinToObstacle)^2))*...
                                            obj.obstacleRepulsion_controller.config.distanceMinToObstacle;
           
               ct=obj.obstacleRepulsion_controller.config.distanceRef-...
                 (obj.obstacleRepulsion_controller.config.distanceMaxToObstacle+...
                 obj.obstacleRepulsion_controller.config.distanceMinToObstacle);
             
               obj.genericController.B_2 = ((-(obj.obstacleRepulsion_controller.config.angleMax*...
                                            obj.obstacleRepulsion_controller.config.distanceMinToObstacle)-...
                                            (ct*obj.obstacleRepulsion_controller.config.angleMin))*...
                                            obj.obstacleRepulsion_controller.config.distanceMinToObstacle*ct)/...
                                            (obj.obstacleRepulsion_controller.config.distanceRef-...
                                            obj.obstacleRepulsion_controller.config.distanceMaxToObstacle);
                                       
               obj.genericController.A_2 = -obj.obstacleRepulsion_controller.config.angleMin*ct-...
                                           obj.genericController.B_2/ct;
           end
       end
       
       function obj = computeAngleBack(obj,distance,source)
           ratio = 0;
%            if (distance > 0 & distance <= obj.obstacleRepulsion_controller.config.distanceMaxToObstacle)
%                %angleBack = obj.genericController.lambda1/distance + obj.genericController.lambda2/(distance^2);
%                  if (distance>=obj.obstacleRepulsion_controller.config.distanceRef)
%                      %gol=1;
%                      angleBack=-obj.genericController.alpha*(distance-obj.obstacleRepulsion_controller.config.distanceRef)^2;
%                  else
%                      angleBack=obj.genericController.beta*(distance-obj.obstacleRepulsion_controller.config.distanceRef)^2;   
%                  end
%                  %angleBack;
%                  angleBack = min(angleBack,obj.obstacleRepulsion_controller.config.angleMax);
% %                  
% %                angleBack = min(angleBack,obj.obstacleRepulsion_controller.config.angleMax);
%            else
%                angleBack = 0;
%            end
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
           if (distance > 0 & distance <= obj.obstacleRepulsion_controller.config.distanceMaxToObstacle)
               %angleBack = obj.genericController.lambda1/distance + obj.genericController.lambda2/(distance^2);
                 if (distance>=obj.obstacleRepulsion_controller.config.distanceRef)
                     d=distance-(obj.obstacleRepulsion_controller.config.distanceMaxToObstacle+...
                          obj.obstacleRepulsion_controller.config.distanceMinToObstacle);
                     angleBack= obj.obstacleRepulsion_controller.config.angleMin+...
                         (obj.genericController.B_2/d^2)+(obj.genericController.A_2/d);
%                      angleBack= (obj.genericController.B_2/d^2)+(obj.genericController.A_2/d); 
                     
                     %angleBack=-obj.genericController.alpha*(distance-obj.obstacleRepulsion_controller.config.distanceRef)^2;
            
                 else
                     %angleBack=obj.genericController.beta*(distance-obj.obstacleRepulsion_controller.config.distanceRef)^2;   
                     angleBack = -obj.obstacleRepulsion_controller.config.angleMin+...
                                (obj.genericController.B_1/distance^2)+...
                                (obj.genericController.A_1/distance);
                     
%                       angleBack =(obj.genericController.B_1/distance^2)+...
%                                  (obj.genericController.A_1/distance);

                 end
                 %angleBack;
                 angleBack = min(angleBack,obj.obstacleRepulsion_controller.config.angleMax);
%                  
%                angleBack = min(angleBack,obj.obstacleRepulsion_controller.config.angleMax);
           else
               angleBack = 0;
           end
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
           if obj.obstacleRepulsion_controller.config.angleMax > 0
               ratio = angleBack/obj.obstacleRepulsion_controller.config.angleMax;
           end
           
           switch source
               case 'LEFT'
                   obj.genericController.ratios.ratioLeft = ratio;
                   obj.genericController.angles.angleBackLeft = angleBack;
               case 'FRONT'
                   obj.genericController.ratios.ratioFront = ratio;
                   obj.genericController.angles.angleBackFront = angleBack;
               case 'RIGHT'
                   obj.genericController.ratios.ratioRight = ratio;
                   obj.genericController.angles.angleBackRight = angleBack;
               case 'BACK'
                   obj.genericController.ratios.ratioBack = ratio;
                   obj.genericController.angles.angleBackBack = angleBack;
           end
                   
       end
       
       %! Compute Left obstacle avoidance
       function Angle2vectorBackLeft(obj)
           computeAngleBack(obj,obj.obstacleRepulsion_controller.input.distanceLeft,'LEFT');
           if (obj.genericController.angles.angleBackLeft >= 0 ...
                   && obj.genericController.angles.angleBackLeft <= obj.obstacleRepulsion_controller.config.angleMax)
               
               obj.genericController.vectors.vectorBackLeft = [sin(obj.genericController.angles.angleBackLeft)...
                   * cos(obj.obstacleRepulsion_controller.config.angleLeft), ...
                   sin(obj.genericController.angles.angleBackLeft) ...
                   * sin(obj.obstacleRepulsion_controller.config.angleLeft), ...
                   cos(obj.genericController.angles.angleBackLeft)];
           else
               obj.genericController.vectors.vectorBackLeft = [0,0,1] ;
           end
       end
       
       %! Compute Front obstacle avoidance
       function Angle2vectorBackFront(obj)
           obj = computeAngleBack(obj,obj.obstacleRepulsion_controller.input.distanceFront,'FRONT');
           if (obj.genericController.angles.angleBackFront >= -obj.obstacleRepulsion_controller.config.angleMax ...
                   && obj.genericController.angles.angleBackFront <= obj.obstacleRepulsion_controller.config.angleMax)
               
               obj.genericController.vectors.vectorBackFront = [sin(obj.genericController.angles.angleBackFront), ...
                   0, cos(obj.genericController.angles.angleBackFront)];
           else
               obj.genericController.vectors.vectorBackFront = [0,0,1] ;
           end
       end
       
       %! Compute Right obstacle avoidance       
       function Angle2vectorBackRight(obj)
           computeAngleBack(obj,obj.obstacleRepulsion_controller.input.distanceRight,'RIGHT');
           if (obj.genericController.angles.angleBackRight >= 0 ...
                   && obj.genericController.angles.angleBackRight <= obj.obstacleRepulsion_controller.config.angleMax)
               
               obj.genericController.vectors.vectorBackRight = [sin(obj.genericController.angles.angleBackRight)...
                   * cos(obj.obstacleRepulsion_controller.config.angleRight), ...
                   sin(obj.genericController.angles.angleBackRight) ...
                   * sin(obj.obstacleRepulsion_controller.config.angleRight), ...
                   cos(obj.genericController.angles.angleBackRight)];
           else
               obj.genericController.vectors.vectorBackRight = [0,0,1] ;
           end
       end
       
       %! Compute Back obstacle avoidance        
       function Angle2vectorBackBack(obj)
           computeAngleBack(obj,obj.obstacleRepulsion_controller.input.distanceBack,'BACK');
           if (obj.genericController.angles.angleBackBack >= 0 ...
                   && obj.genericController.angles.angleBackBack <= obj.obstacleRepulsion_controller.config.angleMax)
               
               obj.genericController.vectors.vectorBackBack = [-sin(obj.genericController.angles.angleBackBack), ...
                   0, cos(obj.genericController.angles.angleBackBack)];
           else
               obj.genericController.vectors.vectorBackBack = [0,0,1] ;
           end
       end
       
       %! Compute the final repulsion
       function finalRepulsion(obj)
           obj.genericController.totalVectorBack = ...
               obj.genericController.ratios.ratioLeft * obj.genericController.vectors.vectorBackLeft ...
               + obj.genericController.ratios.ratioFront * obj.genericController.vectors.vectorBackFront ...
               + obj.genericController.ratios.ratioRight * obj.genericController.vectors.vectorBackRight ...
               + obj.genericController.ratios.ratioBack * obj.genericController.vectors.vectorBackBack; 
           
           if norm(obj.genericController.totalVectorBack)
               %! Normalize
               obj.genericController.totalVectorBack = ...
                   obj.genericController.totalVectorBack/norm(obj.genericController.totalVectorBack) ; 
           else
%                carajo=1
               obj.genericController.totalVectorBack = [0,0,1] ;
           end
%            a=norm(obj.genericController.totalVectorBack(1:2))
%            b=obj.genericController.totalVectorBack(3)
           %angle = atan2(norm(obj.genericController.totalVectorBack(1:2)),obj.genericController.totalVectorBack(3))
           angle = atan(norm(obj.genericController.totalVectorBack(1:2))/obj.genericController.totalVectorBack(3));
           angle = min(angle,obj.obstacleRepulsion_controller.config.angleMax) ;
           quatVector = [-obj.genericController.totalVectorBack(2),obj.genericController.totalVectorBack(1),0];
           if norm(quatVector)
               quatVector = quatVector/norm(quatVector);
           end
           
           obj.obstacleRepulsion_controller.output.qRef.w = cos(angle/2);
           obj.obstacleRepulsion_controller.output.qRef.x = sin(angle/2)*quatVector(1);
           obj.obstacleRepulsion_controller.output.qRef.y = sin(angle/2)*quatVector(2);
           obj.obstacleRepulsion_controller.output.qRef.z = quatVector(3);
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           qRef = [obj.obstacleRepulsion_controller.output.qRef.w,...
           obj.obstacleRepulsion_controller.output.qRef.x,obj.obstacleRepulsion_controller.output.qRef.y,...
           obj.obstacleRepulsion_controller.output.qRef.z ];
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           if norm(qRef)
               obj.obstacleRepulsion_controller.output.qRef.w = ...
                   obj.obstacleRepulsion_controller.output.qRef.w/norm(qRef);
               obj.obstacleRepulsion_controller.output.qRef.x = ...
                   obj.obstacleRepulsion_controller.output.qRef.x/norm(qRef);
               obj.obstacleRepulsion_controller.output.qRef.y = ...
                   obj.obstacleRepulsion_controller.output.qRef.y/norm(qRef);
               obj.obstacleRepulsion_controller.output.qRef.z = ...
                   obj.obstacleRepulsion_controller.output.qRef.z/norm(qRef);
           else
               obj.obstacleRepulsion_controller.output.qRef.w = 1;
               obj.obstacleRepulsion_controller.output.qRef.x = 0;
               obj.obstacleRepulsion_controller.output.qRef.y = 0;
               obj.obstacleRepulsion_controller.output.qRef.z = 0;
           end  
       end
       
       %! Make the controller run
       function obj = updateController(obj)
           setCoefficients(obj);
%            Angle2vectorBackLeft(obj);    
           Angle2vectorBackFront(obj);
%            Angle2vectorBackRight(obj);
%            Angle2vectorBackBack(obj);
           finalRepulsion(obj);
       end
       
              
    end
    
end
