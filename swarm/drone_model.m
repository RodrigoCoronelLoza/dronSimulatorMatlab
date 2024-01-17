classdef drone_model  < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetObservable)
        currentState;
        previousState;
        simulationParameters;
    end
    
    methods
        function obj = drone_model()
            
%             obj.init_real_model;
            %%! Initializing simulation parameters
%             obj.simulationParameters.Ts = 0.001 ;
            obj.simulationParameters.Ts = 0.005 ;
            %             obj.simulationParameters.Ts = 0.063 ;
            obj.simulationParameters.K = 0.1 ;
            obj.simulationParameters.gainAngle = 0.1;
            obj.simulationParameters.gainAltitude = 0.035;
            obj.simulationParameters.Cx = 0.28 ;
            obj.simulationParameters.Cy = 0.35;
            obj.simulationParameters.g = 9.81;
            
            
            %%! Initialzing current state
%             dist_ortho=4;
%             angle=pi/3;
%             a=0;
%             b=1;
%             c=-tan(angle);
%             d=-2.5;
%             
%             %n=[a;b;c];
%             n=-[a;b;c];
%             n=n/norm(n);
%             t=-d/(a*n(1)+b*n(2)+c*n(3));
%             
%             p_incl=[t*n(1);t*n(2);t*n(3)];
%             p_0=p_incl+dist_ortho*n;
%             
%             intix = p_0(2)-0.2150
%             intiy = p_0(1)+0
%             intiz = -p_0(3)-0.01
            
%             obj.currentState.positionNed.x = p_0(2)-0.2150;
%             obj.currentState.positionNed.y = p_0(1)+0;
%             obj.currentState.positionNed.z = -p_0(3)-0.01;
            
                        obj.currentState.positionNed.x = 0;
                        obj.currentState.positionNed.y = 0;
                        obj.currentState.positionNed.z = 0;
            
%             v=0;
%             
%             v_en_y=v*sin(angle);
%             v_en_z=v*cos(angle);
%             v_en_x=0;
            
                        obj.currentState.velocityNed.x = 0;
                        obj.currentState.velocityNed.y = 0;
                        obj.currentState.velocityNed.z = 0;
            
%             obj.currentState.velocityNed.x = v_en_y;
%             obj.currentState.velocityNed.y = v_en_x;
%             obj.currentState.velocityNed.z = -v_en_z;
            
            obj.currentState.angularvelocityNed.x = 0;
            obj.currentState.angularvelocityNed.y = 0;
            obj.currentState.angularvelocityNed.z = 0;
            
            obj.currentState.motorspeed = [0 0 0 0];
            %             obj.currentState.motorspeed.x = 0;
            %             obj.currentState.motorspeed.y = 0;
            %             obj.currentState.motorspeed.z = 0;
            
            obj.currentState.velocityBody.x = 0;
            obj.currentState.velocityBody.y = 0;
            obj.currentState.velocityBody.z = 0;
            
            obj.currentState.accNed.x = 0;
            obj.currentState.accNed.y = 0;
            obj.currentState.accNed.z = 0;
            
            obj.currentState.eulerAnglesRad.phi = 0;
            obj.currentState.eulerAnglesRad.theta = 0;
            obj.currentState.eulerAnglesRad.psi = 0;
            
            obj.currentState.quat.w = 1;
            obj.currentState.quat.x = 0;
            obj.currentState.quat.y = 0;
            obj.currentState.quat.z = 0;

            
%             obj.currentState.tauXq = 0;
%             obj.currentState.tauYq = 0;
%             obj.currentState.tauZq = 0;
            
            obj.currentState.headingEst = 0;
            
            obj.currentState.yawRateEst = 0;
%             obj.currentState.quat.z = 0;
            
            
            
            obj.currentState.anglesCamera.phiCamRad = 0;
            obj.currentState.anglesCamera.thetaCamRad = 0;
            
            %%! Initialzing previous state
                        obj.previousState.positionNed.x = 0;
                        obj.previousState.positionNed.y = 0;
                        obj.previousState.positionNed.z = 0;
%             
%             obj.previousState.positionNed.x = p_0(2)-0.2150;
%             obj.previousState.positionNed.y = p_0(1)+0;
%             obj.previousState.positionNed.z = -p_0(3)-0.01;
            
                        obj.previousState.velocityNed.x = 0;
                        obj.previousState.velocityNed.y = 0;
                        obj.previousState.velocityNed.z = 0;
%             
%             obj.previousState.velocityNed.x = v_en_y;
%             obj.previousState.velocityNed.y = v_en_x;
%             obj.previousState.velocityNed.z = -v_en_z;
            
            obj.previousState.angularvelocityNed.x = 0;
            obj.previousState.angularvelocityNed.y = 0;
            obj.previousState.angularvelocityNed.z = 0;
            
            obj.previousState.motorspeed = [0 0 0 0];
            %             obj.previousState.motorspeed.x = 0;
            %             obj.previousState.motorspeed.y = 0;
            %             obj.previousState.motorspeed.z = 0;
            
            obj.previousState.accNed.x = 0;
            obj.previousState.accNed.y = 0;
            obj.previousState.accNed.z = 0;
            
            obj.previousState.eulerAnglesRad.phi = 0;
            obj.previousState.eulerAnglesRad.theta = 0;
            obj.previousState.eulerAnglesRad.psi = 0;
            
            obj.previousState.quat.w = 1;
            obj.previousState.quat.x = 0;
            obj.previousState.quat.y = 0;
            obj.previousState.quat.z = 0;
            
            obj.previousState.headingEst = 0;
            
            obj.previousState.yawRateEst = 0;
            
            obj.previousState.anglesCamera.phiCamRad = 0;
            obj.previousState.anglesCamera.thetaCamRad = 0;
            
        end
        function obj = setPositionNed(obj,x,y,z)
            %%! Initialzing current state
            obj.currentState.positionNed.x = x;
            obj.currentState.positionNed.y = y;
            obj.currentState.positionNed.z = z;
            %%! Initialzing previous state
            obj.previousState.positionNed.x = 0;
            obj.previousState.positionNed.y = 0;
            obj.previousState.positionNed.z = 0;
            
        end
        function obj = setVelocityNed(obj,vx,vy,vz)
            %%! Initialzing current state
            obj.currentState.velocityNed.x = vx;
            obj.currentState.velocityNed.y = vy;
            obj.currentState.velocityNed.z = vz;
            %%! Initialzing previous state
            obj.previousState.velocityNed.x = vx;
            obj.previousState.velocityNed.y = vy;
            obj.previousState.velocityNed.z = vz;
        end
        %%! Udapting model, based on angle input
        
        function obj = updateModel(obj,phiRef,thetaRef,psiRef,zRef,Thrust,windVelocityNed)
            
            %             Thrust
            %%! Updating angular refs
            %             previo_psi=obj.previousState.eulerAnglesRad.psi
            %             previo_phi=obj.previousState.eulerAnglesRad.phi
            %             previo_theta=obj.previousState.eulerAnglesRad.theta
            
%             ts=obj.simulationParameters.Ts
            r = wrapToPi(obj.currentState.eulerAnglesRad.psi - obj.previousState.eulerAnglesRad.psi)*(1/obj.simulationParameters.Ts);
            rMax = 500*pi/180;
            max(min(r,rMax),-rMax);
            p = min((obj.currentState.eulerAnglesRad.phi - obj.previousState.eulerAnglesRad.phi)*(1/obj.simulationParameters.Ts),100*pi/180);
            q = min((obj.currentState.eulerAnglesRad.theta - obj.previousState.eulerAnglesRad.theta)*(1/obj.simulationParameters.Ts),100*pi/180);
            
            % current = previous
            
            
            
            obj.previousState.eulerAnglesRad = obj.currentState.eulerAnglesRad;
            %
            %             [obj.currentState.positionNed,obj.currentState.velocityNed,...
            %                 obj.currentState.quat,obj.currentState.angularvelocityNed,...
            %                 obj.currentState.motorspeed] = ...
            %                 dynamical_model_update(obj.previousState.positionNed,obj.previousState.velocityNed,...
            %                                        obj.previousState.quat,obj.previousState.angularvelocityNed,...
            %                                        obj.previousState.motorspeed, obj.simulationParameters.Ts);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
            %%%%%%%%%%%55
            
            
            
            
            %%! wind
            windNed.x = windVelocityNed(1,1);
            windNed.y = windVelocityNed(1,2);
            windNed.z = 0;
            
            windVelocityBody  = obj.fromWorldToBody(windNed,obj.previousState.eulerAnglesRad,1);
            
            psi = obj.previousState.eulerAnglesRad.psi;
            
            %%! angle model
            theta_phys = obj.previousState.eulerAnglesRad.theta ....
                + obj.simulationParameters.gainAngle *(thetaRef - obj.previousState.eulerAnglesRad.theta);
            
            phi_phys = obj.previousState.eulerAnglesRad.phi....
                + obj.simulationParameters.gainAngle *(phiRef - obj.previousState.eulerAnglesRad.phi);
            
            %             phi_phys = phiRef;
            %             theta_phys = thetaRef;
            %
            
            obj.previousState.velocityBody = obj.fromWorldToBody(obj.previousState.velocityNed,....
                obj.previousState.eulerAnglesRad,1);
            
            
            vPointBody.x = (r*obj.previousState.velocityBody.y)....
                - (q*obj.previousState.velocityNed.z).....
                -obj.simulationParameters.g*sin(theta_phys)  .... //thrust projected on body frame
                -obj.simulationParameters.Cx*obj.previousState.velocityBody.x.... // drag
                + obj.simulationParameters.Cx * windVelocityBody.x; %% wind
            
            vPointBody.y = p*obj.previousState.velocityNed.z -....
                (r*obj.previousState.velocityBody.x) ....
                + obj.simulationParameters.g*sin(phi_phys)*cos(theta_phys) ....
                -obj.simulationParameters.Cy*obj.previousState.velocityBody.y + ....
                obj.simulationParameters.Cy * windVelocityBody.y ;
            
            %             vPointBody.z = 0;
            M=1.5;
            C=1;
%             Thrust
%             velocity_z_previous= obj.previousState.velocityNed.z
            
%             vPointBody.z = -Thrust/M-C*obj.previousState.velocityNed.z + obj.simulationParameters.g;% neg?
           velocidadnedz= obj.previousState.velocityNed.z;
        vPointBody.z = -Thrust/M-C*obj.previousState.velocityNed.z; % neg?
%             v_point_z= vPointBody.z
            %             vPointBody
            
            aceleracionNedz =obj.currentState.accNed.z;
            obj.currentState.accNed = obj.fromWorldToBody(vPointBody,....
                obj.previousState.eulerAnglesRad,0);
            
%             acc_Ned_z=obj.currentState.accNed.z
            
            obj.currentState.velocityNed.x = obj.previousState.velocityNed.x + ....
                obj.currentState.accNed.x * obj.simulationParameters.Ts;
            obj.currentState.velocityNed.y = obj.previousState.velocityNed.y + ....
                obj.currentState.accNed.y * obj.simulationParameters.Ts;
            obj.currentState.velocityNed.z = obj.previousState.velocityNed.z + ....
                obj.currentState.accNed.z * obj.simulationParameters.Ts;
            
            %
            obj.currentState.eulerAnglesRad.theta = theta_phys;
            obj.currentState.eulerAnglesRad.phi = phi_phys;
            
            
            obj.currentState.positionNed.x = obj.previousState.positionNed.x + ....
                obj.currentState.velocityNed.x * obj.simulationParameters.Ts;
            obj.currentState.positionNed.y = obj.previousState.positionNed.y + .....
                obj.currentState.velocityNed.y * obj.simulationParameters.Ts;
            obj.currentState.positionNed.z = obj.previousState.positionNed.z + .....
                obj.currentState.velocityNed.z * obj.simulationParameters.Ts;
            
            
            
            %%z model first order
            %             obj.currentState.positionNed.z =  obj.previousState.positionNed.z....
            %                 + obj.simulationParameters.gainAltitude *(zRef - obj.previousState.positionNed.z);
            %                 %%velocity is the derivative of the first order
            %             obj.currentState.velocityNed.z = .....
            %                 (obj.currentState.positionNed.z-obj.previousState.positionNed.z)*(1/obj.simulationParameters.Ts);
            
            
            obj.currentState.eulerAnglesRad.psi = obj.previousState.eulerAnglesRad.psi ....
                + obj.simulationParameters.K * wrapToPi(psiRef - obj.previousState.eulerAnglesRad.psi) ;
            obj.currentState.eulerAnglesRad.psi = wrapToPi(obj.currentState.eulerAnglesRad.psi);
            
            %%%%%%%%%%%%%%%%%%5555
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
            
            %%! Previous = current, end of integration.
            obj.previousState.positionNed = obj.currentState.positionNed;
            obj.previousState.velocityNed = obj.currentState.velocityNed;
            obj.previousState.accNed = obj.currentState.accNed;
            
        end
        
        %         function obj = updateModel_2(obj,phiRef,thetaRef,psiRef,zRef,Thrust,windVelocityNed)
        function  init_real_model(obj)
            %%
            %X Y axis
            line_sfx1_quaternion;
            line_sfx1_angular_rates;
            
            %
            line_sfx1_heading;
            line_sfx1_yawRate;
            
            % Z axis
            line_sfx1_altitude;
            
            % Mix
            line_sfx1_mix;
            
            addpath('../../nav/mexes'); % Path to the loadjson script
            config = loadjson('mix.cfg'); % Load config parameters
            
            D1 = config.mix.D1;
            D2 = config.mix.D2;
            D3 = config.mix.D3;
            D4 = config.mix.D4;
            K = config.mix.K;
            Kt = config.mix.Kt;
            rho0 = config.mix.rho0;
            rho = config.mix.rho;
            minRpm = config.mix.minRpm;
            maxRpm = config.mix.maxRpm;
            maxPwm = config.mix.maxPwm;
            minimalTauZGuaranteed = config.mix.minimalTauZGuaranteed;
            
            mex_sfx1_mix('set_gains',...
                D1, D2, D3, D4, K, Kt, rho0, rho, minRpm, maxRpm, maxPwm, minimalTauZGuaranteed);
            
            % Quaternions
            Kpx = config.quat.Kpx;
            Kpy = config.quat.Kpy;
            Kpz = config.quat.Kpz;
            Kdx = config.quat.Kdx;
            Kdy = config.quat.Kdy;
            Kdz = config.quat.Kdz;
            Kix = config.quat.Kix;
            Kiy = config.quat.Kiy;
            Kiz = config.quat.Kiz;
            SatIx = config.quat.SatIx;
            SatIy = config.quat.SatIy;
            SatIz = config.quat.SatIz;
            
            mex_sfx1_quaternion('set_gains', ...
                Kpx, Kpy, Kpz, Kix, Kiy, Kiz, Kdx, Kdy, Kdz, ...
                SatIx, SatIy, SatIz);
            
            % yawRate
            Kp = config.yawRate.Kp;
            Ki = config.yawRate.Ki;
            Kd = config.yawRate.Kd;
            SatI = config.yawRate.SatI;
            tauMax = config.yawRate.tauMax;
            
            mex_sfx1_yawRate('set_gains', ...
                Kp, Ki, Kd, SatI, tauMax);
            
            % angularRates
            Kpx = config.angularRates.Kpx;
            Kpy = config.angularRates.Kpy;
            Kdx = config.angularRates.Kdx;
            Kdy = config.angularRates.Kdy;
            Kix = config.angularRates.Kix;
            Kiy = config.angularRates.Kiy;
            SatIx = config.angularRates.SatIx;
            SatIy = config.angularRates.SatIy;
            maxCmd = config.angularRates.maxCmd;
            
            mex_sfx1_angular_rates('set_gains', ...
                Kpx, Kpy, Kix, Kiy, Kdx, Kdy, SatIx, SatIy, maxCmd);
            
            % heading
            Kp = config.heading.Kp;
            Kd = config.heading.Kd;
            Ki = config.heading.Ki;
            SatI = config.heading.SatI;
            
            mex_sfx1_heading('set_gains', ...
                Kp, Ki, Kd, SatI);
            
            % altitude
            Kp = config.alt.Kp;
            Kd = config.alt.Kd;
            defaultKi = config.alt.defaultKi;
            defaultSatI = config.alt.defaultSatI;
            satOutput = config.alt.satOutput;
            altitudeDerivativeCommandFilterB = config.alt.altitudeDerivativeCommandFilterB;
            altitudeDerivativeCommandFilterA = config.alt.altitudeDerivativeCommandFilterA;
            
            
            mex_sfx1_altitude('set_gains', ...
                Kp, defaultKi, Kd, defaultSatI,satOutput,...
                altitudeDerivativeCommandFilterA(1), altitudeDerivativeCommandFilterA(2), ...
                altitudeDerivativeCommandFilterB(1), altitudeDerivativeCommandFilterB(2));
        end
        
        function obj = updateModel_2(obj,phiRef,thetaRef,psiRef,zRef,Thrust,windVelocityNed)
            %%
            %state
            
            position_prev=[obj.previousState.positionNed.x...
                obj.previousState.positionNed.y...
                obj.previousState.positionNed.z];
            
            position_now=[obj.currentState.positionNed.x...
                obj.currentState.positionNed.y...
                obj.currentState.positionNed.z];
            
            %             velocity = zeros(length(t),3);
            
            velocity_prev= [obj.previousState.velocityNed.x...
                obj.previousState.velocityNed.y...
                obj.previousState.velocityNed.z];
            
            velocity_now= [obj.currentState.velocityNed.x...
                obj.currentState.velocityNed.y...
                obj.currentState.velocityNed.z];
            
            
%             euleurAngles = zeros(length(t),3);
            
            eulerAngles_prev=[obj.previousState.eulerAnglesRad.phi...
                obj.previousState.eulerAnglesRad.theta ...
                obj.previousState.eulerAnglesRad.psi];
            
            eulerAngles_now=[obj.currentState.eulerAnglesRad.phi...
                obj.currentState.eulerAnglesRad.theta ...
                obj.currentState.eulerAnglesRad.psi];
            %
            %             position = zeros(length(t),3);
            %             velocity = zeros(length(t),3);
            %             euleurAngles = zeros(length(t),3);
            %
            %quat
            %%in
            
            % quat estado
            
            quatEst_prev =[obj.previousState.quat.w...
                obj.previousState.quat.x... 
                obj.previousState.quat.y...
                obj.previousState.quat.z];
            
            quatEst_now =[obj.currentState.quat.w...
                obj.currentState.quat.x... 
                obj.currentState.quat.y...
                obj.currentState.quat.z];
            
%             quatRef = zeros(1,4);
%             quatEst = zeros(length(t),4);
%             quatRef(:,1) = ones(1,1);
%             quatEst(:,1) = ones(length(t),1);
            
            %%Control
            yaw = psiRef;
            pitch = thetaRef;
            roll = phiRef;
            
            qRef = euler_angle2_quat(pitch, roll, yaw);
            quatRef(1) = qRef(1);
            quatRef(2) = qRef(2);
            quatRef(3) = qRef(3);
            quatRef(4) = qRef(4);
            
%             euleurAnglesRef = zeros(length(t),3);
            
            %%out
%             tauXq = zeros(length(t),1);
%             tauYq = zeros(length(t),1);
%             tauZq = zeros(length(t),1);
            
            tauXq = 0;
            tauYq = 0;
            tauZq = 0;
            
            %heading
            %%in
            headingEst_prev=obj.previousState.headingEst;
            headingEst_now=obj.currentState.headingEst;
            
%             headingEst = zeros(length(t),1);
            headingRef = psiRef;
            %%out
%             yawRateEst = zeros(length(t),1);
            
            %yaw rate
            %%in
%             yawRateEst = zeros(length(t),1);
            
            yawRateEst_prev=obj.previousState.yawRateEst;
            yawRateEst_now=obj.currentState.yawRateEst;
            
            yawRateRef = 0;
            %%out
            tauZ = 0;
            
            %angular velocity
            %%in
%             wEst = zeros(length(t),3);
            
            wEst_prev = [obj.previousState.angularvelocityNed.x...
                obj.previousState.angularvelocityNed.y... 
                obj.previousState.angularvelocityNed.z];
            
            wEst_now = [obj.currentState.angularvelocityNed.x...
                obj.currentState.angularvelocityNed.y... 
                obj.currentState.angularvelocityNed.z];
            
            wRef = zeros(1,3);
            %%out
            tauXY = zeros(1,2);
            
            %%z
            %%in
%             zRef = -10*ones(length(t),1);
            
            zRef_now = -zRef;
            
%             zEst = zeros(length(t),1);
%             vzEst = zeros(length(t),1);
            %%out
            thrust = 0;
            
            %%mix
            %%in
            % tauZ = control_mix_tauZ;
            ff = -17.5;
            %%out
            rpms = zeros(1,4);
            sat = 0;
            finalDetailed = zeros(1,6);
            %%
            %             quatRef(:,2:4) = control_quat_qRef(:,1:3);
            %             quatRef(:,1) = control_quat_qRef(:,4);
            %             headingRef = control_heading_ref;
            %             zRef = -control_alt_ato_ref;
            %
            %             quatEst(:,2:4) = control_quat_qEst(:,1:3);
            %             quatEst(:,1) = control_quat_qEst(:,4);
            %             headingEst(1,1) = control_heading_est(1,1);
            
            %             for k=2:length(t)
            quatEst_prev;
            %% quaternion
            [tauXq, tauYq, tauZq] = ...
                mex_sfx1_quaternion('step', quatRef, quatEst_prev);
            %% heading
            [yawRateRef] = ...
                mex_sfx1_heading('step',headingEst_prev,headingRef);
            %% yaw
            [tauZ] = ...
                mex_sfx1_yawRate('step',yawRateRef,yawRateEst_prev);
            %% angular
            wRef = [tauXq, tauYq, tauZq];
            [tauXY] = ....
                mex_sfx1_angular_rates('step',wEst_prev,wRef);
            %% z
            %                 [thrust(k,:)] = ....
            %                     mex_sfx1_altitude('step',position(k-1,3),zRef(k,:),velocity(k-1,3));
%             [thrust] = ....
%                 mex_sfx1_altitude('step',position_prev(3),zRef_now,velocity_prev(3));
            [thrust]=-Thrust;
            
            %% mix
            [rpms,sat,finalDetailed] = ....
                mex_sfx1_mix('step',tauXY(1),tauXY(2),tauZ,ff,thrust);
            
            %% calling dynamic model
%             te1=rpms
%             te2=rpms'
            quatEst_prev;
            position_prev;
            velocity_prev;
            
            wEst_prev;
            
            [position_now, velocity_now,....
                quatEst_now, wEst_now, rpms] = .....
                dynamical_model_update(position_prev', velocity_prev',....
                quatEst_prev', wEst_prev', rpms*2*pi/60, 0.005);
            
            position_now;
            quatEst_now;
            velocity_now;
            rpms;
            wEst_now;
            
            
            [yaw, pitch, roll] =  quat2_euler_angle(quatEst_now);
            euleurAngles_now(1) = roll;
            euleurAngles_now(2) = pitch;
            euleurAngles_now(3) = yaw;
            
            
            headingEst_now = yaw;
            
%             [yaw, pitch, roll] =  quat2_euler_angle(quatRef(k,:));
%             
%             euleurAnglesRef(k,1) = roll;
%             euleurAnglesRef(k,2) = pitch;
%             euleurAnglesRef(k,3) = yaw;
            
            yawRateEst_now = wEst_now(3);
            
            obj.currentState.positionNed.x = position_now(1);
            obj.currentState.positionNed.y = position_now(2);
            obj.currentState.positionNed.z = position_now(3);
            
            obj.currentState.velocityNed.x = velocity_now(1);
            obj.currentState.velocityNed.y = velocity_now(2);
            obj.currentState.velocityNed.z = velocity_now(3);
            
            obj.currentState.eulerAnglesRad.phi = euleurAngles_now(1);
            obj.currentState.eulerAnglesRad.theta = euleurAngles_now(2);
            obj.currentState.eulerAnglesRad.psi = euleurAngles_now(3);
            
            obj.currentState.quat.w = quatEst_now(1);
            obj.currentState.quat.x = quatEst_now(2);
            obj.currentState.quat.y = quatEst_now(3);
            obj.currentState.quat.z = quatEst_now(4);
            
            obj.currentState.tauXq=tauXq;
            obj.currentState.tauYq=tauYq;
            obj.currentState.tauZq=tauZq;
            
            obj.currentState.headingEst=headingEst_now;
            
            obj.currentState.yawRateEst=yawRateEst_now;
            
            obj.currentState.angularvelocityNed.x = wEst_now(1);
            obj.currentState.angularvelocityNed.y = wEst_now(2);
            obj.currentState.angularvelocityNed.z = wEst_now(3);
            
            
            
            obj.previousState.positionNed = obj.currentState.positionNed;
            obj.previousState.velocityNed = obj.currentState.velocityNed;
            %                 obj.previousState.accNed = obj.currentState.accNed;
            obj.previousState.eulerAnglesRad = obj.currentState.eulerAnglesRad;
            obj.previousState.quat=obj.currentState.quat;
            obj.previousState.headingEst = obj.currentState.headingEst;
            obj.previousState.yawRateEst = obj.currentState.yawRateEst;
            obj.previousState.angularvelocityNed = obj.currentState.angularvelocityNed;
            %             end
            
            
            
        end
        function state = getCurrentState(obj)
            % % directly to output
            state = [ ....
                obj.currentState.positionNed.x ....
                obj.currentState.positionNed.y ....
                obj.currentState.positionNed.z ....
                obj.currentState.velocityNed.x ....
                obj.currentState.velocityNed.y ....
                obj.currentState.velocityNed.z ....
                obj.currentState.accNed.x ....
                obj.currentState.accNed.y ....
                obj.currentState.accNed.z ....
                obj.currentState.eulerAnglesRad.phi ....
                obj.currentState.eulerAnglesRad.theta ....
                obj.currentState.eulerAnglesRad.psi ....
                ];
            
        end
        
        
        function [outputVector]=fromWorldToBody(obj,inputVector,eulerAngles,inverse)
            
            R = zeros(3,3) ;
            cpsi = cos(eulerAngles.psi);
            spsi = sin(eulerAngles.psi);
            cth = cos(eulerAngles.theta);
            sth = sin(eulerAngles.theta);
            cph = cos(eulerAngles.phi);
            sph = sin(eulerAngles.phi);
            
            R(1,1) = cpsi*cth;
            R(1,2) = cpsi*sph*sth-spsi*cph;
            R(1,3) = cpsi*sth*cph+spsi*sph;
            R(2,1) = spsi*cth;
            R(2,2) = spsi*sth*sph+cpsi*cph;
            R(2,3) = spsi*sth*cph-cpsi*sph;
            R(3,1) = -sth ;
            R(3,2) = cth*sph ;
            R(3,3) = cth*cph ;
            
            if(inverse == 1 )
                outputArray = R' * [inputVector.x;inputVector.y;inputVector.z];
            else
                outputArray = R * [inputVector.x;inputVector.y;inputVector.z];
            end
            
            outputVector.x = outputArray(1,1);
            outputVector.y = outputArray(2,1);
            outputVector.z = outputArray(3,1);
            
        end
    end
end
