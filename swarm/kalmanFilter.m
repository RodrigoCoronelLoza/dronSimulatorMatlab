classdef kalmanFilter < handle
    
    properties
        % drone_model
        %genericController
        %obstacleRepulsion_controller
%         surfaceFollowing_controller
        kalman_filter
    end
    
    methods
        %! class constructor
        function obj = kalmanFilter()
            %            obj.drone_model = obj2.drone_state;
            
            %! Inputs
            obj.kalman_filter.input.d = [];
            
            obj.kalman_filter.input.pos_ancla=[];
            
            
            obj.kalman_filter.input.drone_state=[];
%             obj.surfaceFollowing_controller.input.direction_desired=[0,1,0];
            obj.kalman_filter.input.input.delta_tk=[];
            obj.kalman_filter.input.Previous_delta_tk=[];
            obj.kalman_filter.input.Current_delta_tk=[];
%             obj.surfaceFollowing_controller.input.yaw_value=[];%%NO
            
            %%%obtener obstaculos
            
            
            
            %%% Meter aqui las infelices constantes
            
            %! Configuration
           
            obj.kalman_filter.config.acc_max_vect=[5,5,5];
            obj.kalman_filter.config.R=0.98;
           
            %! innner variables
            
            obj.kalman_filter.inner.Previous_apriori_state=zeros (6,1); %vector 6x1
            obj.kalman_filter.inner.Current_apriori_state=zeros (6,1); %vector 6x1
            obj.kalman_filter.inner.Previous_aposteriori_state=zeros (6,1); %vector 6x1
            obj.kalman_filter.inner.Current_aposteriori_state=zeros (6,1); %vector 6x1
            obj.kalman_filter.inner.Previous_apriori_Pmatrix=zeros(6);
            obj.kalman_filter.inner.Current_apriori_Pmatrix=zeros(6); %matrix 6x6+
%             obj.kalman_filter.inner.Previous_apriori_Pmatrix=10*ones(6);
%             obj.kalman_filter.inner.Current_apriori_Pmatrix=10*ones(6); %matrix 6x6
            obj.kalman_filter.inner.Previous_aposteriori_Pmatrix=zeros(6);
            obj.kalman_filter.inner.Current_aposteriori_Pmatrix=zeros(6); %matrix 6x6
            
            
            obj.kalman_filter.inner.Previous_Amatrix=zeros(6);
            obj.kalman_filter.inner.Current_Amatrix=zeros(6);
            obj.kalman_filter.inner.Previous_Qmatrix=zeros(6);
            obj.kalman_filter.inner.Current_Qmatrix=zeros(6);
            
            obj.kalman_filter.inner.Current_distance_estimation=0; %vector 6x1
            
            obj.kalman_filter.inner.Current_Hmatrix=0;
            
            obj.kalman_filter.inner.Current_Kmatrix=0;
            
            %! Output   
        end
        

        function obj = updatedistance(obj,d)
            obj.kalman_filter.input.d = d ;
        end
        
        function obj = updatedrone_state(obj,drone_state)
            obj.surfaceFollowing_controller.input.drone_state=drone_state;
        end
        
        function obj = updateanchor_position(obj,anchor_position)
            obj.kalman_filter.input.pos_ancla = anchor_position;
        end
        
        function obj = initialize_state(obj,trilateration_position)
            obj.kalman_filter.inner.Previous_aposteriori_state = [trilateration_position(1);trilateration_position(2);...
                trilateration_position(3);0;0;0];
        end
        
        function obj = updatecurrent_delta_tk(obj,delta_tk)
            obj.kalman_filter.input.Current_delta_tk=delta_tk*10^-3;
        end
        
        function obj = updateprevious_delta_tk(obj,delta_tk)
            obj.kalman_filter.input.Previous_delta_tk=delta_tk*10^-3;
        end
        
        function obj = filter(obj)
            
            %definicion de la matriz A
            
            obj.kalman_filter.inner.Previous_Amatrix =[eye(3),obj.kalman_filter.input.Previous_delta_tk*eye(3);...
                                                       zeros(3),eye(3)];
                                                   
            obj.kalman_filter.inner.Current_Amatrix =[eye(3),obj.kalman_filter.input.Current_delta_tk*eye(3);...
                                                       zeros(3),eye(3)];
            
            %definicion de la matriz Q
            
            PreviousQ1=[0.25*(obj.kalman_filter.input.Previous_delta_tk)^4,0.5*(obj.kalman_filter.input.Previous_delta_tk)^3;...
                0.5*(obj.kalman_filter.input.Previous_delta_tk)^3,(obj.kalman_filter.input.Previous_delta_tk)^2];                                       
            PreviousQ2=diag([obj.kalman_filter.config.acc_max_vect(1)^2 obj.kalman_filter.config.acc_max_vect(2)^2 obj.kalman_filter.config.acc_max_vect(3)^2]);
            
            obj.kalman_filter.inner.Previous_Qmatrix= kron(PreviousQ1,PreviousQ2);
            
            CurrentQ1=[0.25*(obj.kalman_filter.input.Current_delta_tk)^4,0.5*(obj.kalman_filter.input.Current_delta_tk)^3;...
                0.5*(obj.kalman_filter.input.Current_delta_tk)^3,(obj.kalman_filter.input.Current_delta_tk)^2];                                       
            CurrentQ2=diag([obj.kalman_filter.config.acc_max_vect(1)^2 obj.kalman_filter.config.acc_max_vect(2)^2 obj.kalman_filter.config.acc_max_vect(3)^2]);
            
            obj.kalman_filter.inner.Current_Qmatrix= kron(CurrentQ1,CurrentQ2);
            
            % Prediction (Process Update)
            obj.kalman_filter.inner.Current_apriori_state = obj.kalman_filter.inner.Previous_Amatrix*...
                obj.kalman_filter.inner.Previous_aposteriori_state;
            
            obj.kalman_filter.inner.Current_apriori_Pmatrix=...
                obj.kalman_filter.inner.Previous_Amatrix*obj.kalman_filter.inner.Previous_aposteriori_Pmatrix*...
                (obj.kalman_filter.inner.Previous_Amatrix)'+obj.kalman_filter.inner.Previous_Qmatrix;
            
            % Correction (Measurement update)
            
            obj.kalman_filter.inner.Current_distance_estimation = norm(obj.kalman_filter.inner.Current_apriori_state(1:3)-...
                obj.kalman_filter.input.pos_ancla);
            
            obj.kalman_filter.inner.Current_Hmatrix=[(obj.kalman_filter.inner.Current_apriori_state(1:3)-...
                obj.kalman_filter.input.pos_ancla)',0,0,0]/obj.kalman_filter.inner.Current_distance_estimation;
            
            obj.kalman_filter.inner.Current_Kmatrix = obj.kalman_filter.inner.Current_apriori_Pmatrix*...
                obj.kalman_filter.inner.Current_Hmatrix'*(obj.kalman_filter.inner.Current_Hmatrix*...
                obj.kalman_filter.inner.Current_apriori_Pmatrix*obj.kalman_filter.inner.Current_Hmatrix'+...
                obj.kalman_filter.config.R)^-1;
            
            obj.kalman_filter.inner.Current_aposteriori_state = obj.kalman_filter.inner.Current_apriori_state +...
                obj.kalman_filter.inner.Current_Kmatrix*(obj.kalman_filter.input.d-...
                obj.kalman_filter.inner.Current_distance_estimation);
            
            obj.kalman_filter.inner.Current_aposteriori_Pmatrix = (eye(6)-...
                obj.kalman_filter.inner.Current_Kmatrix*obj.kalman_filter.inner.Current_Hmatrix)*...
                obj.kalman_filter.inner.Current_apriori_Pmatrix;
            
            
            %%%%% updates
            obj.kalman_filter.inner.Previous_apriori_state=...
                obj.kalman_filter.inner.Current_apriori_state; %vector 6x1
            obj.kalman_filter.inner.Previous_aposteriori_state=...
                obj.kalman_filter.inner.Current_aposteriori_state; %vector 6x1
            obj.kalman_filter.inner.Previous_apriori_Pmatrix=...
                obj.kalman_filter.inner.Current_apriori_Pmatrix; %matrix 6x6
            obj.kalman_filter.inner.Previous_aposteriori_Pmatrix=...
                obj.kalman_filter.inner.Current_aposteriori_Pmatrix; %matrix 6x6
            
        end
        
        %! Make the filter run
        function obj = updatefilter(obj) 
            filter(obj); 
        end
        
        
    end
    
end
