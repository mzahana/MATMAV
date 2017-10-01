classdef MatMav < handle
    %MatMav 2.5 class : is an interface between MATLAB and multiple
    % Pixhawk/PX4 using Mavlink protocol.
    %   Copyright: This is developed by Mohamed Abdelkader, 2016
    %   Contact: matmav.toolbox@gmail.com
    %   Redistribution of this tool is not allowed without permission from
    %   the author!
    %
    %   This tool provides an easy interface with multiple Pixhawk/PX4
    %   flight controllers that are loaded with PX4 Frimware
    %
    %   This currently does not support Ardupilot Firmware.
    %   The communication works via serial telemetry modules or UDP.
    %
    %   This is a new major release (version 2.0) that supports
    %   multiple flight controllers, monitoring and control.
    %
    % To have a look at the available properties and methods type
    %   methods(MatMav_MA), and properties(MatMav_MA)
    %
    %To get help on look at the MATMAV guide on
    % https://mzahana.gitbooks.io/matmav-guide/content/
%% private, but visible, properties


    properties(SetAccess = private)
        
        % Number of vehciles to link
        number_of_target_systems=1;
        
        % for serial communication
        % COM port
        COM
        % Baud rate
        BAUD
        
        % for UDP communication
        % struct: holds the destination IP and port of each agent
        % example: for two agents
        % UDP_dest=
        % {'IP_address_1', port_number_1;
        %   'IP_address_2', port_number_2};
        UDP_dest
        UDP_localPort=10000;
        
        %2:position & yaw,  3:velocity and yaw
        setPointMask;
        
        % struct('x',0,'y',0,'z',0,'Yaw',0);
        setPosition;
        
        thrust_setpoint
            % target thrust setpoint: [0,1]
            
        angular_rates_sp
            % body rates setpoints: roll_rate, pitch_rate, yaw_rate
            
        quaternion_sp
            % attitude (in quaternion)
        
        % struct('vx',0,'vy',0,'vz',0,'Yaw',0);
        setVelocity;
        
        %struct('qw',1,'qx',0,'qy',0,'qz',0,'x',0,'y',0,'z',0);
        setMocap;
        
        % takeoff altitudes for each agent
        takeoffALT;
        
        % circle properties
%         circle_center;
%         circle_radius;
%         circle_direction;
%         circleFlags;
        
        % actuators commands
        actuators;
        
        % Flags that identifies which vehicle to send setpoints to
        setpointsFlags;
        
        % Flags that identifies which vehicle to send setMocap data to
        MocapFlags;
        
        % Flags that identifies which vehicle to actuator flags to
        actuatorsFlags;
        
        % flag for serial/udp connection status
        % 1: if anhy conneciton is ON, 0: if no connection
        IsConnect=false;
        
        % flag for serial connection status
        % 1: if connected to serial port, 0: not connected
        SerialConnected=false;
        
        % flag for UDP connection status
        % 1: if connected to UDP port, 0: not connected
        UDPConnected=false;
        
        % flag for Connection status with each Mav
        % row vector, 1: if connected with Mav, 0:not connected with Mav
        MavConncetionStatus;
        
        % Array to store available serial ports
        AvaiablePorts;
        
        % vector of component IDs of all connected Mavs
        Comp_ID=0;

        % the following variables to be initialized in the constructor
        
        % Received Attitude variable
        %=struct('roll',0,'pitch',0,'yaw',0);
        get_Attitude;
        
        get_Attitude_target;
            % target attitude
            % rollrate, pitchrate, yawrate, thrust,quaternion
        
        % Received Local NED variable
        %=struct('x',0,'y',0,'z',0);
        get_LocalNED;
        
        % Received GPS variables
        %=struct('lat',0,'lon',0,'Alt',0);
        get_GPS;
        
        % Received Battery vlotages
        % row vector [v1, v2, v3, ...];
        get_Battery_voltage;
        
        % Arm flag
        % 1: armed, 0: not armed
        IsArmed;
        
        % Mav mode
        % Manual, Position control, Altitude control, offboard
        get_vehicle_mode;
        
        % Received pressure info
        % struct
        get_pressure_info;
        
        % Received absolute pressure info
        % struct
        get_absolute_pressure;
        
        % Received differential pressure info
        % struct
        get_differential_pressure;
        
        % IMU data xacc,yacc,zacc,  xgyro,ygyro,zgyro
        get_IMUData;
        
        % Fence position limits, meters
        % [min, max]
        x_fence;
        y_fence;
        z_fence;
        
        % Fence velocity limits, m/s
        % [min, max]
        vx_fence;
        vy_fence;
        vz_fence;
        
        % Safe position for all Mavs x y z in NED, meters
        % Ser by setSafePostion() method
        SafePosition;
        
        % Fence mode flag
        % 1: go to safe positions (default)
        % 2: stay at limits
        FenceMode;
        
        IsnatnetConnected;
            % flag for natnet connection status
            % 1: connected, 0 : not connected
            
        natnet_MocapIP='127.0.0.1';
            % natnet mocap IP
        natnet_clientIP='127.0.0.1';
            % natnet client IP
        natnet_connectionType='Multicast';
            
        natnet_rotation;
        natnet_position; 
        natnet_positionNED; 
        natnet_rotationNED; 
        natnet_q_wxyz;
        
        server_port;
            % local udp port of server 
        nClients;
            % number of server clients
        ClientsAddr;
            % clients addresses & ports
            % {'IP1', port1;
            %  'IP2', port2}
        

    end

%% private, local, properties
    properties (Access=protected)
        
        %%%%%%% TIMERS
        %%%%(***UPDATE: REPLACED BY FLAGS, AND MERGED IN ONE MAIN TIMER)
        
        % OFFBOARD timer
        % periodically send offboard commands
        % (REMOVED PERMANENTLY. ONLY NEEDED TO BE TRIGGERD ONCE!)
        % use toggle_OFFB() function
        %offb_timer%=timer;
        
        % This is the MAIN timer for all periodic tasks
        % All peridoic tasks should be executed there
        % Time consuming tasks shoul NOT be executed there
        % as it included data trasmission and reception
        MAIN_timer%=timer;
        
        % transmission rate
        TX_rate = 0.1;
        current_tic = tic;
        
        % periodically receive data into buffer
        % (TO BE REMOVED AND MERGED IN A MAIN TIMER)
        %receive_timer;
        run_udp_rcv;
        
        % Mocap timer:
        % periodically sends motion capture data
        % (TO BE REMOVED AND MERGED IN A MAIN TIMER)
        %mocap_timer%=timer;
        run_mocap;
        %----> flag to indicate sending mocap data in main timer or not
        
        % setpoit  timer:
        % to periodically send setpoints
        % (TO BE REMOVED AND MERGED IN A MAIN TIMER)
        %setpoints_timer%=timer;
        run_setpoints;
        %----> flag to indicate sending setpoints in main timer or not
        
        % Connection timer
        % to monitor the connection status to Vehicles
        % (TO BE REMOVED AND MERGED IN A MAIN TIMER)
        %MavConnection_timer
        run_mavconn;
        %----> flag to indicate updating mav connection status in main timer or not
        mavconn_start_time;
        mavconn_dt=10;
        
        % Heartbeat timer:
        % sending heart beats, periodically
        % (TO BE REMOVED AND MERGED IN A MAIN TIMER)
        %heartbeat_timer
        run_heartbeat;
        %----> flag to indicate sending hearbeat in main timer or not
        hb_start_time;
        %----> time tic since last activation of heartbeat
        hb_dt=1;
        %----> heartbeat time period
        
        % Fence timer: to impose protection limits on 
        % position and velocity.
        % (TO BE REMOVED AND MERGED IN A MAIN TIMER)
        %fence_timer  
        run_fence;
        %----> flag to indicate activating fence function in main timer or not

        % Serial connection timer:
        %
        serialObject;
        

        udpSockets;
        %----> udp socket
        
        natnetObj;
        %----> natnet object
            
        sub_M;
            % cell array contains subscribtion matrices, for each client
            % each client has nVxnT martix. nV is number of vehicles
            % connected, nT is the total number of topics
            
        numTopics;
            % total number of topics
            % this is calculated from a function that has latest list of
            % topics
            
        IsServerON;
            % boolean variable for the server status (ON/OFF)
        
        server_buffIN;
            % input buffer
        server_buffOUT;
            % output buffer
            
        server_socket;
            % udp socket handle
            
        autoip;
            %  flag to allow(or not) auto IP/Port detection
            % of vehices connected via UDP
    end

    properties (Access = private)
        % Msgflag: to indicate if each message has placed new packets in
        % holdBuff.
        % Once a message (e.g. mocap) places a new packet in the assigned
        % slot in holdBuff, the correspoonding flag is set to 1
        % once the packet is copied to the sendBuff (to be sent), the
        % corresponding flag is set back to zero
        % ---- this is initialized in the constructor
        Msgflag
        
        MavData; % vehicles data

        MavStat % reflects arming state/maybe another stauts, too?!

        % offboard flag for each Mav
        offb_flag
        
        % for circle execution
%         circle_freq=1;
%         path_TS=0.2;
%         circle_counter;
%         pathIsIdle;

        % transmisson buffer: to be initialized in the constructor
        BuffLen;
        sendBuff;
        holdBuff;%=uint8(zeros(1,512));

    end
%% Constant properties,local access only
    properties (Constant,Access=private)
        
        stream_rate=0.02;% streaming through serial port, seconds, 50hz
        update_rate=0.01;% updating the Send buffer, seconds, 100Hz
        
        udpmaxsize=1024;
        
        MaxNofMavs=5; % maximum number of Mavs to consider
        % total number of possible messages to be sent
        % should be updates when new message is introduced in the class
        % currently: {Arm, mocap, setpoints, offboard, Manualmode, request_stream,set_param, set_actuator, set_gimbal, send_heartbeat, set_attitude_target_rc_channel_override}
        NumMessg=12;

        % packet lengthes, bytes
        armLen=41; mocapLen=44; setptLen=61; offboardLen=41; ManualLen=41;
        reqStreamLen=14; paramLen=31; actuatorsLen=51; gimbalLen=41;
        heartbeatLen=17; att_targetLen=47; rcChanLen=26;
        % what about..?
        % MsgLen=[armLen, mocapLen, setptLen, offboardLen, ManualLen] ?

        % order of each message
        arm_i=1; mocap_i=2; setpt_i=3; offboard_i=4; Manual_i=5;
        stream_i=6; param_i=7;actautors_i=8; gimbal_i=9;
        heartbeat_i=10;att_target_i=11; rcChan_i=12;

        % the position of each message in the holdBuff
        % should be done automatically !!
        armInd=1:41;
        mocapInd=41+1 : 41+44;
        setptInd=41+44+1 : 41+44+61;
        offboardInd=41+44+61+1: 41+44+61+41;
        ManualInd=41+44+61+41+1 : 41+44+61+41+41;
        ReqStreamInd=41+44+61+41+41+1 : 41+44+61+41+41+14;
        paramInd=41+44+61+41+41+14+1 : 41+44+61+41+41+14+31;
        actuatrosInd=41+44+61+41+41+14+31+1 : 41+44+61+41+41+14+31+51;
        gimbalInd=41+44+61+41+41+14+31+51+1 : 41+44+61+41+41+14+31+51+41;
        heartbeatInd=41+44+61+41+41+14+31+51+41+1 : 41+44+61+41+41+14+31+51+41+17;
        att_targetInd=41+44+61+41+41+14+31+51+41+17+1 : 41+44+61+41+41+14+31+51+41+17+47; 
        rcChanInd=41+44+61+41+41+14+31+51+41+17+47+1 : 41+44+61+41+41+14+31+51+41+17+47+26;

        MsgInd={1:41;...
                41+1 : 41+44;...
                41+44+1: 41+44+61;...
                41+44+61+1: 41+44+61+41;...
                41+44+61+41+1 : 41+44+61+41+41;...
                41+44+61+41+41+1 : 41+44+61+41+41+14;...
                41+44+61+41+41+14+1 : 41+44+61+41+41+14+31;...
                41+44+61+41+41+14+31+1 : 41+44+61+41+41+14+31+51;...
                41+44+61+41+41+14+31+51+1 : 41+44+61+41+41+14+31+51+41;...
                41+44+61+41+41+14+31+51+41+1 : 41+44+61+41+41+14+31+51+41+17;...
                41+44+61+41+41+14+31+51+41+17+1 : 41+44+61+41+41+14+31+51+41+17+47;...
                41+44+61+41+41+14+31+51+41+17+47+1 : 41+44+61+41+41+14+31+51+41+17+47+26};

        MsgsLen=[41, 44, 61, 41,41, 14,31, 51,41, 17, 47, 26];% should be used in a smarter way!!
    end
    
    properties(Constant)
        % MatMav version
        version='v2.5.2';
        
        position_yaw_flag=2;
        velocity_yaw_flag=3;
        acceleration_flag=4;
        thrust_flag=5;
        attitude_target_flag=6;
        angular_rates_flag=7;
    end
%% public methods, functions
    methods


         function obj=MatMav(nts)
            % This function is used to construct a MatMav object
            %
            %   Syntax: Obj=MatMav;
            %   Or
            %           Obj=MatMav(3);
            %   3 is the number of target systems to connect to
            %clc
            
            if nargin<1
                obj.set_number_of_target_systems(1);
            else
                if nts>obj.MaxNofMavs
                    error(['Maximum number of vehicles is ',...
                        num2str(obj.MaxNofMavs), ' for now.']);
                end
                obj.set_number_of_target_systems(nts);
            end
            
            % intialize timers here,
%             obj.offb_timer=timer;
%             obj.offb_timer.Period=obj.update_rate;
%             obj.offb_timer.TimerFcn={@(src,event)offb_ToSend(obj,src,event)};
%             obj.offb_timer.ExecutionMode='fixedSpacing';
%             
            obj.MAIN_timer=timer;
            obj.MAIN_timer.Period=obj.stream_rate;
            obj.MAIN_timer.ExecutionMode='fixedSpacing';
            obj.MAIN_timer.TimerFcn={@(src,event)MAIN_timer_callback(obj,src,event)};
            
%             obj.receive_timer=timer;
%             obj.receive_timer.Period=obj.rcv_rate;
%             obj.receive_timer.ExecutionMode='fixedSpacing';
%             obj.receive_timer.TimerFcn={@(src,event)udp_rcv_callback(obj,src,event)};
            obj.run_udp_rcv=false;
            
%             obj.mocap_timer=timer;
%             obj.mocap_timer.Period=obj.update_rate;
%             obj.mocap_timer.TimerFcn={@(src,event)mocap_ToSend(obj,src,event)};
%             obj.mocap_timer.ExecutionMode='fixedSpacing';
            obj.run_mocap=false;
            
%             obj.setpoints_timer=timer;
%             obj.setpoints_timer.Period=obj.update_rate;
%             obj.setpoints_timer.TimerFcn={@(src,event)setpoints_ToSend(obj,src,event)};
%             obj.setpoints_timer.ExecutionMode='fixedSpacing';
            obj.run_setpoints=false;
            
%             obj.MavConnection_timer=timer;
%             obj.MavConnection_timer.Period=10;
%             obj.MavConnection_timer.ExecutionMode='fixedSpacing';
%             obj.MavConnection_timer.TimerFcn={@(src,event)updateMavConncetion(obj,src,event)};
            obj.run_mavconn=false;
            
%             obj.heartbeat_timer=timer;
%             obj.heartbeat_timer.Period=1; % send hearbeat every one second
%             obj.heartbeat_timer.ExecutionMode='fixedSpacing';
%             obj.heartbeat_timer.TimerFcn={@(src,event)sendHeartbeat(obj,src,event)};
            obj.run_heartbeat=false;
            
%             obj.path_timer=timer;
%             obj.path_timer.Period=obj.path_TS;
%             obj.path_timer.ExecutionMode='fixedSpacing';
%             obj.path_timer.TimerFcn={@(src,event)updatePath(obj,src,event)};
            
%             obj.fence_timer=timer;
%             obj.fence_timer.Period=0.01;
%             obj.fence_timer.ExecutionMode='fixedSpacing';
%             obj.fence_timer.TimerFcn={@(src,event)fenceFunction(obj,src,event)};
            obj.run_fence=false;
            
            obj.ClientsAddr={'127.0.0.1','13000'};
            %--->initialize addresses of MATMAV clients
            
            obj.autoip=true;
            %----> set auto up detectioin to true, initially
            
            
          % create natnet object
          if ispc
              try
                %obj.natnetObj=natnet();
              catch
                  warning('natnet class is not on MATLAB PATH. Ignore this message if you are not using NatNet.');
              end
          end

            
        end % end of MatMav constructor

        function set_number_of_target_systems(obj,value)
            if ~obj.IsConnect
                if length(value)>1
                    error('Value can not be more than one number!');
                end
                if isempty(value)
                    error('Value can not be empty!');
                end

                obj.number_of_target_systems=value;

                obj.IsArmed=zeros(1,value);

                obj.get_Battery_voltage=zeros(1,value);

                obj.get_vehicle_mode=cell(1,value);
                
%                 obj.get_absolute_pressure=zeros(1,value);
%                 obj.get_differential_pressure=zeros(1,value);
                obj.get_pressure_info=struct('time_usec',cell(value,1),...
                                         'get_absolute_pressure',cell(value,1),...
                                         'get_differential_pressure',cell(value,1),...
                                         'scaled_pressure_time_ms',cell(value,1),...
                                         'scaled_abs_pressure', cell(value,1),...
                                         'scaled_diff_pressure',cell(value,1));
                obj.get_IMUData=struct('time_usec', cell(value,1),...
                                   'xacc',cell(value,1),...
                                   'yacc',cell(value,1),...
                                   'zacc',cell(value,1),...
                                   'xgyro',cell(value,1),...
                                   'ygyro',cell(value,1),...
                                   'zgyro',cell(value,1));

                obj.setPosition=struct('x', cell(value,1),...
                                       'y', cell(value,1),...
                                       'z', cell(value,1),...
                                       'Yaw', cell(value,1));
                                   
               obj.setVelocity=struct('vx', cell(value,1),...
                                       'vy', cell(value,1),...
                                       'vz', cell(value,1),...
                                       'Yaw', cell(value,1));

                obj.setMocap=struct('qw', cell(value,1),...
                                       'qx', cell(value,1),...
                                       'qy', cell(value,1),...
                                       'qz', cell(value,1),...
                                       'x', cell(value,1),...
                                       'y', cell(value,1),...
                                       'z', cell(value,1));

                obj.setpointsFlags=zeros(1,value);

                obj.MocapFlags=zeros(1,value);

                obj.get_Attitude=struct('time_msec',cell(value,1),...
                                       'roll', cell(value,1),...
                                       'pitch', cell(value,1),...
                                       'yaw',cell(value,1),...
                                       'rollspeed',cell(value,1),...
                                       'pitchspeed',cell(value,1),...
                                       'yawspeed',cell(value,1));
                                   
               obj.get_Attitude_target=struct('time_ms',cell(value,1),...
                                              'q0',cell(value,1),...
                                              'q1',cell(value,1),...
                                              'q2',cell(value,1),...
                                              'q3',cell(value,1),...
                                              'roll_rate',cell(value,1),...
                                              'pitch_rate',cell(value,1),...
                                              'yaw_rate',cell(value,1),...
                                              'thrust',cell(value,1));
               
               obj.get_LocalNED=struct('time_msec',cell(value,1),...
                                      'x', cell(value,1),...
                                      'y', cell(value,1),...
                                      'z', cell(value,1),...
                                      'vx', cell(value,1),...
                                      'vy', cell(value,1),...
                                      'vz', cell(value,1));

              obj.get_GPS=struct('time_usec',cell(value,1),...
                                'lat', cell(value,1),...
                                'lon', cell(value,1),...
                                'Alt', cell(value,1));
                            
              obj.UDP_dest=cell(value,2);

              obj.Msgflag=zeros(value,obj.NumMessg);

              obj.MavStat=zeros(1,value);

              obj.offb_flag=zeros(1,value);

              obj.holdBuff=uint8(zeros(value,sum(obj.MsgsLen)));

              obj.BuffLen=value*sum(obj.MsgsLen);

              obj.MavConncetionStatus=zeros(1,value);
              obj.takeoffALT=-1*ones(1,value);
              
%               obj.circle_center=zeros(value,2);
%               obj.circle_radius=ones(1,value);
%               obj.circleFlags=zeros(1,value);
%               obj.pathIsIdle=zeros(1,value);
%               obj.circle_counter=zeros(1,value);
%               obj.circle_direction=zeros(1,value);
              
              obj.Comp_ID=zeros(1,value);
              
              obj.setPointMask=obj.position_yaw_flag;
              
              obj.actuators=-1*ones(value,8);
              obj.actuatorsFlags=zeros(1,value);
              
              % in NED, meters
              obj.x_fence=ones(value,2);
              obj.x_fence(:,1)=-1;
              
              obj.y_fence=ones(value,2);
              obj.y_fence(:,1)=-1;
              
              obj.z_fence=ones(value,2);
              obj.z_fence(:,1)=0;
              
              % in NED, m/s
              obj.vx_fence=0.1*ones(value,2);
              obj.vx_fence(:,1)=-0.1;
              
              obj.vy_fence=0.1*ones(value,2);
              obj.vy_fence(:,1)=-0.1;
              
              obj.vz_fence=0.1*ones(value,2);
              obj.vz_fence(:,1)=-0.1;
              
              obj.SafePosition=zeros(value,3);
              obj.SafePosition(:,3)=-0.2;% stay at 20cm above ground
              
              obj.FenceMode=1;
              
              obj.thrust_setpoint=zeros(1,value);
              
              obj.angular_rates_sp=struct('roll_rate',cell(value,1),...
                                      'pitch_rate', cell(value,1),...
                                      'yaw_rate', cell(value,1));
                                  
              obj.quaternion_sp=zeros(value,4);
              
              obj.IsServerON=false; % server is intially OFF
              
            else
                error('Can not change number of target systems while connected!')
            end
        end

        % set function of COM port
        function set_COMPORT(obj,value)
            if ~ischar(value)
                error('Value must be string of the COM Port.');
            end
            obj.COM=value;
        end
        
        % set function of BAUD rate
        function set_BAUDRATE(obj,value)
            if ~isnumeric(value)
                error('Value mut be an integer e.g. (9600. 57600, 115200, 921600)');
            end
            obj.BAUD=value;
        end
        
        % set function of UDP_dest
        function set_UDPREMOTEADDR(obj,targets,values)
            if nargin <2
                error('Usage: set_UDPREMOTEADDR(target,{''addr'', port})');
            end
            if length(targets)> 1
                error('Please set one target at a time');
            end
            
            if targets > obj.number_of_target_systems
                error('There is a traget that is greater than total number of vehicles.');
            end
            if ~iscell(values)
                error('Value must be cell array e.g. {''IP'', Port}.');
            end
            if size(values,1) ~= length(targets)
                error('Number of addresses must match number of input targets.');
            end

                obj.UDP_dest(targets,:)=values;
        end
        
        % set function of UDP_localPort
        function set_UDPLOCALPRT(obj,value)
            if ~isnumeric(value)
                error('Value must be a number.');
            end
            if obj.IsConnect
                error('You can not change local port while connected.');
            end
            obj.UDP_localPort=value;
        end
        
        % set transmission rate, seconds
        function set_transmission_rate(obj, sec)
            if ~isnumeric(sec)
                error('Value must be numeric.');
            end
            obj.TX_rate = sec;
        end
        
        function v = get_transmission_rate(obj)
            v = obj.TX_rate;
        end
        
        % set function for setPointMask
        function set_setPointMask(obj, value)
            if ~isnumeric(value)
                error('Value must be numeric. Available Masks, 2(position) or 3(velocity)');
            end
            if value <2 || value >7
                error('Available Masks, 2(position), 3(velocity), 5(acceleration), 5(thrust), 6(attitude target), 7(body rates).');
            end
         obj.setPointMask=value;
        end
        
        % set function of setPosition
        function set_PositionSetPoints(obj,target,x,y,z,yaw)
            if ~isnumeric(x) || ~isnumeric(y) || ~isnumeric(z) || ~isnumeric(yaw)
                error('x y z yaw should be numeric.');
            end
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            obj.setPosition(target).x=x;
            obj.setPosition(target).y=y;
            obj.setPosition(target).z=z;
            obj.setPosition(target).Yaw=yaw;
        end
        % set function of setVelocity
        function set_VelocitySetPoints(obj,target,vx,vy,vz,yaw)
            if ~isnumeric(vx) || ~isnumeric(vy) || ~isnumeric(vz) || ~isnumeric(yaw)
                error('vx vy vz yaw should be numeric.');
            end
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            obj.setVelocity(target).vx=vx;
            obj.setVelocity(target).vy=vy;
            obj.setVelocity(target).vz=vz;
            obj.setVelocity(target).Yaw=yaw;
        end
        
        % set thrust setpoint
        function set_thrust_sp(obj,agent,value)
            if agent > obj.number_of_target_systems
                error('agent should be within the defined number of systems.');
            end
            
            if value <0 || value>1
                error('thrust value should be within [0,1]');
            end
            obj.thrust_setpoint(agent)=value;
        end
        
        % set angular rates setpoint
        function set_angular_rates_sp(obj, agent,r,p,y)
            % r,p,y should be rad/s
            if agent > obj.number_of_target_systems
                error('agent should be within the defined number of systems.');
            end
            obj.angular_rates_sp(agent).roll_rate=r;
            obj.angular_rates_sp(agent).pitch_rate=p;
            obj.angular_rates_sp(agent).yaw_rate=y;
        end
        
        % set quaternion attitude setpoint
        function set_attitude_sp(obj, agent, q0,q1,q2,q3)
            if agent > obj.number_of_target_systems
                error('agent should be within the defined number of systems.');
            end
            obj.quaternion_sp(agent,1)=q0;
            obj.quaternion_sp(agent,2)=q1;
            obj.quaternion_sp(agent,3)=q2;
            obj.quaternion_sp(agent,4)=q3;
        end
        
        % set function of setMocap
        function set_MocapData(obj, target,qw,qx,qy,qz,x,y,z)
            % sanity checks
            if ~isnumeric(qw) || ~isnumeric(qx) || ~isnumeric(qy) || ~isnumeric(qz) || ~isnumeric(x) || ~isnumeric(y) || ~isnumeric(z)
                error('qw qx qy qz x y z  must be numeric');
            end
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            obj.setMocap(target).qw=qw;
            obj.setMocap(target).qx=qx;
            obj.setMocap(target).qy=qy;
            obj.setMocap(target).qz=qz;
            obj.setMocap(target).x=x;
            obj.setMocap(target).y=y;
            obj.setMocap(target).z=z;
        end
        
        % set function of takeoffALT
        function set_takeoffALT(obj,target,value)
            % sanity checks
            if ~isnumeric(value)
                error('Value must be numeric');
            end
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            obj.takeoffALT(target)=value;
        end
        
        % set function of circle_center
%         function set_circle_center(obj, target, x,y)
%             % sanity checks
%             if ~isnumeric(x) || ~isnumeric(y)
%                 error('x y must be numeric.');
%             end
%             if length(target) > 1
%                 error('Set one agent at a time.');
%             end
%             if target > obj.number_of_target_systems
%                 error('Target is greater that available total targets.');
%             end
%             obj.circle_center(target,:)=[x y];
%         end
        
        % set function of circle_radius
%         function set_circle_radius(obj, target, value)
%             % sanity checks
%             if ~isnumeric(value)
%                 error('Value must be numeric');
%             end
%             if length(target) > 1
%                 error('Set one agent at a time.');
%             end
%             if target > obj.number_of_target_systems
%                 error('Target is greater that available total targets');
%             end
%             obj.circle_radius(target)=value;
%         end
        
        % set function of circle_direction
%         function set_circle_direction(obj,target, value)
%              % sanity checks
%             if ~isnumeric(value)
%                 error('Value must be numeric');
%             end
%             if length(target) > 1
%                 error('Set one agent at a time.');
%             end
%             if target > obj.number_of_target_systems
%                 error('Target is greater that available total targets');
%             end
%             obj.circle_direction(target)=value;
%         end
        
        % set function of actuators vector
        function set_actuatorsValues(obj,target,values)
            % sanity checks
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            if length(values) ~= 8
                error('Vlaues must be vector of 8 elements.');
            end
            obj.actuators=values;
        end
        
        % set function of setpointsFlags
        function set_setpointsFlags(obj, target,value)
            % sanity checks
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            if ~(value ==0 || value ==1)
                error('value must be 0 or 1.');
            end
            obj.setpointsFlags(target)=value;
        end
        
        % set function of actuatorsFlags
        function set_actuatorsFlags(obj, target,value)
            % sanity checks
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            if ~(value ==0 || value ==1)
                error('value must be 0 or 1.');
            end
            obj.actuatorsFlags(target)=value;
        end
        
        % set function of MocapFlags
        function set_MocapFlags(obj, target, value)
            % sanity checks
            if length(target) > 1
                error('Set one agent at a time.');
            end
            if target > obj.number_of_target_systems
                error('Target is greater that available total targets');
            end
            if ~(value ==0 || value ==1)
                error('value must be 0 or 1.');
            end
            obj.MocapFlags(target)=value;
            
        end
        
        % set function of circleFlags
%         function set_circleFlags(obj,target,value)
%             % sanity checks
%             if length(target) > 1
%                 error('Set one agent at a time.');
%             end
%             if target > obj.number_of_target_systems
%                 error('Target is greater that available total targets');
%             end
%             if ~(value ==0 || value ==1)
%                 error('value must be 0 or 1.');
%             end
%             obj.circleFlags(target)=value;
%         end
        
        % function to set x_fence
        function set_x_fence(obj,mavID,values)
            l=length(mavID);
            sz=size(values);
            if l>1
                error('Only set one Mav at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=2
                error('Values matrix has to be 1x2 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x2 matrix');
            end
            
            obj.x_fence(mavID,:)=values;
        end% End: set_x_fence
        
        % function to set y_fence
        function set_y_fence(obj,mavID,values)
            l=length(mavID);
            sz=size(values);
            if l>1
                error('Only set one Mav at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=2
                error('Values matrix has to be 1x2 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x2 matrix');
            end
            
            obj.y_fence(mavID,:)=values;
        end% End: set_y_fence
        
        % function to set z_fence
        function set_z_fence(obj,mavID,values)
            l=length(mavID);
            sz=size(values);
            if l>1
                error('Only set one Mav at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=2
                error('Values matrix has to be 1x2 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x2 matrix');
            end
            
            obj.z_fence(mavID,:)=values;
        end% End: set_z_fence
        
        % function to set vx_fence
        function set_vx_fence(obj,mavID,values)
            l=length(mavID);
            sz=size(values);
            if l>1
                error('Only set one Mav at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=2
                error('Values matrix has to be 1x2 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x2 matrix');
            end
            
            obj.vx_fence(mavID,:)=values;
        end% End: set_vx_fence
        
        % function to set vy_fence
        function set_vy_fence(obj,mavID,values)
            l=length(mavID);
            sz=size(values);
            if l>1
                error('Only set one Mav at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=2
                error('Values matrix has to be 1x2 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x2 matrix');
            end
            
            obj.vy_fence(mavID,:)=values;
        end% End: set_vy_fence
        
        % function to set vz_fence
        function set_vz_fence(obj,mavID,values)
            l=length(mavID);
            sz=size(values);
            if l>1
                error('Only set one Mav at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=2
                error('Values matrix has to be 1x2 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x2 matrix');
            end
            
            obj.vz_fence(mavID,:)=values;
        end% End: set_vz_fence
        
        % Set safe positions
        function setSafePosition(obj,mavID,xyz)
            l=length(mavID);
            sz=size(xyz);
            if l>1
                error('Only set one Mav''s Safe Position at a time.');
            end
            
            if mavID>obj.number_of_target_systems
                error('Mav ID is bigger than the max number of target systems.')
            end
            
            if sz(2) ~=3
                error('Values matrix has to be 1x3 matrix');
            end
            
            if sz(1) ~= 1
                error('Values matrix has to be 1x3 matrix');
            end
            
            obj.SafePosition(mavID,:)=xyz;
        end% End:setSafePosition
        
        % set Fence mode flag
        function setFenceMode(obj,mode)
            if mode==1
                obj.FenceMode=1;
            elseif mode==2
                obj.FenceMode=2;
            else
                error('Only mode=1,2 are supported.')
            end
        end%End:setFenceMode
        
        % Activate Fence
        function ActivateFence(obj)
            if ~obj.IsConnect
                error('No Serial/UDP connection is established.')
            end
            
            obj.run_fence=true;
            disp('Fence is activated.');
        end%End:ActivateFence
        
        function DeactivateFence(obj)
            obj.run_fence=false;
            disp('Fence is deactivated.');  
        end
        
        % find available ports
        function findPorts(obj)
            % This function retrieves the avialble 'Serial' ports that are
            % currently connected to this Machine.
            % This is not a self updating function! Run it whenever ports
            % are updated.
            %
            %   Syntax: Obj.findPorts

            p=instrhwinfo('serial');
            if isempty(p.AvailableSerialPorts)
                disp('No available serial ports. Connect one!')
            end
            obj.AvaiablePorts=p.AvailableSerialPorts;
            disp(obj.AvaiablePorts)
            clear p
        end % end of findPorts
        
        % connect to serial port
        function ConnectSerial(obj)
            % This function is used to connect to the serial port defined
            % by the properties COM and BAUD which have to be assigned
            % first.
            %
            %   Syntax: Obj.Connect
            
            if obj.IsConnect || obj.UDPConnected
                error('It seems there is already a running connection.')
            end

            if isempty(obj.COM)
                error('No Serial port is assigned. Use COM property to assign a port name')
            elseif isempty(obj.BAUD)
                error('No BAUD rate is assigned. Use BAUD property to assign one.')
%             elseif ~ismember(obj.COM, obj.AvaiablePorts)
%                 error([obj.COM, ' is not connected yet. Connect it first, or choose another one!'])
            else
                obj.serialObject = serial(obj.COM,'BaudRate',obj.BAUD);% create the serial object
                %Specify if the bytes-available event is generated after a specified number
                %   of bytes is available in the input buffer, or after a terminator is read
                % values: 'terminator', 'byte'
                obj.serialObject.BytesAvailableFcnMode='byte';

                obj.serialObject.OutputBufferSize=obj.BuffLen;
                obj.serialObject.InputBufferSize = 1024;

                %Specify the callback function to execute when a specified number of
                %   bytes is available in the input buffer, or a terminator is read
                obj.serialObject.BytesAvailableFcn={@(src,event)serialEventHandler(obj,src,event)};
                try
                    fopen(obj.serialObject );% open serial port, and start reading/writings
                    disp(['   Connected to ', obj.COM])
                    obj.IsConnect=true;
                    obj.SerialConnected=true;
                catch
                    fclose(obj.serialObject);
                    delete(obj.serialObject);
                    obj.IsConnect=false;
                    obj.SerialConnected=false;
                    error('Cannot open. Port is eaither being used, or not available.')
                end
            end

            % start monitoring heartbeat
            obj.mavconn_start_time=tic;
            obj.run_mavconn=true;
            
            % start transmition timer
            if ~strcmp(obj.MAIN_timer.Running,'on')
                start(obj.MAIN_timer)
            end

        end % end of Connect
        
        % connect via UDP
        function ConnectUDP(obj)
            %disp('Make sure the tcp_udp_ip folder is available in this folder')
            %addpath('tcp_udp_ip');
            if obj.IsConnect || obj.SerialConnected
                error('It seems there is already a running connection. Maybe Serial!')
            end
            
            if isempty(obj.UDP_localPort)
                error('Local port is not specified.');
            end
            
            % open udp socket
            try
                obj.udpSockets=pnet('udpsocket',obj.UDP_localPort,'noblock');
                sec=1e-2;
                pnet(obj.udpSockets,'setreadtimeout',sec);
                pnet(obj.udpSockets,'setwritetimeout',sec);
                disp(['Opened UDP socket on local port: ',num2str(obj.UDP_localPort)]);
                obj.UDPConnected=true;
                obj.IsConnect=true;
            catch
                obj.IsConnect=false;
                obj.UDPConnected=false;
                error('pnet is not available OR local port is not avilable.');
            end
            
            % start transmition timer
            if ~strcmp(obj.MAIN_timer.Running,'on')
                start(obj.MAIN_timer)
                obj.run_udp_rcv=true;
            end


            % start monitoring heartbeat
            obj.mavconn_start_time=tic;
            obj.run_mavconn=true;
            
                
        end % end: ConnectUDP
        
        function set_autoip(obj,flag)
            obj.autoip=flag;
        end
        
        % set NatNet connection type
        function set_natnet_ConnectionType(obj, connT)
            if strcmp(connT,'Multicast') || strcmp(connT, 'Unicast')
                obj.natnet_connectionType = connT;
                disp(['Connection type is set to: ', connT]);
            else
                disp('Connection type should be either: ''Multicast'' or ''Unicast'' ');
            end
        end
        
        % connect natnet
        function ConnectNatNet(obj)
            if ispc
                % disconnect any previous connection
                try
                    disp(['NatNet connection type is set to: ', obj.natnet_connectionType]);
                    obj.natnetObj.disable(0);
                    obj.natnetObj.disconnect();
                    obj.natnetObj=natnet();
                    obj.natnetObj.ClientIP = obj.natnet_MocapIP; % mocap IP
                    obj.natnetObj.HostIP = obj.natnet_clientIP; % laptop IP
                    obj.natnetObj.ConnectionType = obj.natnet_connectionType;
                    obj.natnetObj.connect();
                    obj.natnetObj.addlistener( 1 , 'getRigidBodiesfromMocap' );
                    obj.natnetObj.enable(0);% start listenning
                    
                    obj.IsnatnetConnected = obj.natnetObj.IsConnected;
                catch
                    disp('Caught some error');
                end
%                 if ~obj.natnetObj.IsConnected
%                     obj.natnetObj.ClientIP = obj.natnet_MocapIP; % mocap IP
%                     obj.natnetObj.HostIP = obj.natnet_clientIP; % laptop IP
%                     obj.natnetObj.ConnectionType = 'Multicast';
%                     obj.natnetObj.connect();
%                     obj.natnetObj.addlistener( 1 , 'getRigidBodiesfromMocap' );
%                     obj.natnetObj.enable(0);% start listenning
%                 else
%                     obj.natnetObj.enable(0);
%                 end
            else
                error('Only works in Windows.');
            end
        end % end ConnectNatNet
        
        function startServer(obj,prt,nclients)
            %%% this function starts MatMav server
            
            if (nargin<3 && nargin>1), nclients=1; end
            if (nargin<2), prt=25000; nclients=1; end
            obj.nClients=nclients;
            obj.server_port=prt;
            
            if isempty(obj.ClientsAddr{1})
                obj.ClientsAddr{1}='127.0.0.1';
                obj.ClientsAddr{2}=13000;
            end
            % start server only if it is off
            if ~obj.IsServerON
                %%% initialize  subscribtion matrix
                obj.sub_M=cell(nclients,1);
                % get total number of topics
                T=getTopics();
                obj.sub_M(:)={zeros(obj.number_of_target_systems,T.nT)};
                
                %%% start udp port
                try
                    obj.server_socket=pnet('udpsocket',obj.server_port,'noblock');
                    sec=1e-2;
                    pnet(obj.server_socket,'setreadtimeout',sec);
                    pnet(obj.server_socket,'setwritetimeout',sec)
                    obj.IsServerON=true;
                    disp(['Server is started on port ',num2str(obj.server_port)]);
                catch
                    pnet(obj.server_socket,'close');
                    obj.IsServerON=false;
                    error('pnet package is not available, or port is used.');
                end
            else
                warning('Server is already running');
            end
        end % end startServer()
        
        function stopServer(obj)
            %%% stop MatMav server
            obj.IsServerON=false;
            %%% re-initialize subscribtion matricies
            obj.sub_M=cell(obj.nClients,1);
            % get total number of topics
            T=getTopics();
            obj.sub_M(:)={zeros(obj.number_of_target_systems,T.nT)};
            
            % empty buffers
            obj.server_buffIN=[]; obj.server_buffOUT=[];
            
            % close udp socket
            if ~isempty(obj.server_socket)
                try
                    if pnet(obj.server_socket,'status')>0
                        pnet(obj.server_socket,'close');
                    end
                catch
                end
            end
            disp('Server stopped.');
        end % end stopServer
        
        function set_natnet_mocapIP(obj,value)
            if ~ischar(value)
                error('Should be string. e.g. ''127.0.0.1'' ');
            end
            obj.natnet_MocapIP=value;
        end % end set_natnet_mocapIP
        
        function set_natnet_clientIP(obj,value)
            if ~ischar(value)
                error('Should be string. e.g. ''127.0.0.1'' ');
            end
            obj.natnet_clientIP=value;
        end % end set_natnet_clientIP
        
        function sendMocap(obj,flag)
            % This function trigers the start/stop of streaming motion
            % capture data which are set by the property Obj.setMocap.
            %
            % Obj.setMocap is a structure of the form,
            % Obj.setMocap.qw, Obj.setMocap.qx, Obj.setMocap.qy,
            % Obj.setMocap.qz,
            % Obj.setMocap.x, Obj.setMocap.y, Obj.setMocap.z
            %
            %   Syntax: Obj.sendMocap(1) for start, Obj.sendMocap(1) for
            %   stop
            if obj.IsConnect
                if flag
                    obj.run_mocap=true;
                    disp('Sending Mocap data...');
                else
                    obj.run_mocap=false;
                    disp('Sending Mocap data is STOPPED.');
                end
            else
                error('Connect to Port first!')
            end

        end % end of sendMocap
        
        % set RC channels override
        function set_RC_Channels(obj, agent,chs)
            if agent>obj.number_of_target_systems
                error('Agent number is invalid');
            end
            if length(chs)~=8
                error('Channels data should be 1x8 vector');
            end
            [Buff,~]=mavlink('setChannelOverride', agent, 0,chs(1),...
                                                            chs(2),...
                                                            chs(3),...
                                                            chs(4),...
                                                            chs(5),...
                                                            chs(6),...
                                                            chs(7),...
                                                            chs(8));
            obj.holdBuff(agent, obj.rcChanInd)=Buff;
            obj.Msgflag(agent, obj.rcChan_i)=1;
        end % end set_RC_Channels

        function sendSetPoints(obj,flag)
            % This function trigers the start/stop of streaming position
            % setpoints which are set by the property Obj.setPosition.
            %
            % Obj.setPosition is a structure of the form,
            % Obj.setPosition.x, Obj.setPosition.y, Obj.setPosition.z
            % Obj.setPosition.Yaw
            %
            %   Syntax: Obj.sendSetPoints(1) for start, Obj.sendSetPoints(1) for
            %   stop
            if obj.IsConnect
                if flag
                    obj.run_setpoints=true;
                    disp('Sending setpoints...')
%                     if ~obj.run_setpoints
%                         obj.run_setpoints=true;
%                     else
%                         disp('setpoints streaming already runngin!')
%                     end
                else
                    obj.run_setpoints=false;
                    disp('Sending setpoints is STOPPED.')
%                     if obj.run_setpoints
%                         obj.run_setpoints=false;
%                     else
%                         disp('setpoints already stopped!')
%                     end
                end
            else
                error('Connect to Port first!')
            end
        end % end of sendSetPoints

%         function sendoffb(obj, flag)
%             % This function trigers the start/stop of streaming the
%             % OFFBOARD mode
%             %
%             %   Syntax: Obj.sendoffb(1) for start, Obj.sendoffb(1) for stop
%             if obj.IsConnect
%                 if flag
%                     if ~strcmp(obj.offb_timer.Running,'on')
% %                         obj.offb_timer.Period=obj.update_rate;
% %                         obj.offb_timer.TimerFcn={@(src,event)offb_ToSend(obj,src,event)};
% %                         obj.offb_timer.ExecutionMode='fixedSpacing';
%                         obj.offb_flag=1;
%                         start(obj.offb_timer)
%                     else
%                         disp('Offboard streaming already runngin!')
%                     end
%                 else
%                     if strcmp(obj.offb_timer.Running,'on')
%                         obj.offb_flag=0;
%                     for agent=1:obj.number_of_target_systems
%                             [Buff,~]=mavlink('offboard',agent,0,obj.offb_flag);
%                             obj.holdBuff(agent, obj.offboardInd)=Buff;
%                             obj.Msgflag(agent, obj.offboard_i)=1;
%                             obj.setpointsFlags(agent)=0;
%                     end
%                         % should add a puase here to let it sink?!!!
%                         stop(obj.offb_timer)
%                     else
%                         disp('Offboard already stopped!')
%                     end
%                 end
%             else
%                 error('Connect to Port first!')
%             end
%         end % end of sendoffb
        
        function toggle_OFFB(obj,target_sysID, flag)
            % This function trigers the start/stop of
            % OFFBOARD mode
            %
            %   Syntax: Obj.toggle_OFFB(sysID,1) for start, Obj.toggle_OFFB(sysID,0) for stop
            if obj.IsConnect
                if flag
                    [Buff,~]=mavlink('offboard',target_sysID,0,flag);
                    obj.holdBuff(target_sysID, obj.offboardInd)=Buff;
                    obj.Msgflag(target_sysID, obj.offboard_i)=1;
                else
                    [Buff,~]=mavlink('offboard',target_sysID,0,flag);
                    obj.holdBuff(target_sysID, obj.offboardInd)=Buff;
                    obj.Msgflag(target_sysID, obj.offboard_i)=1;
                    obj.setpointsFlags(target_sysID)=0;
                end
            else
                error('Connect to Port first!')
            end
        end % end toggle_OFFB

        %Arm Mav
        function Arm(obj,target_systems, flag)
            % This function sends an Arm/Disarm command
            %
            %   Syntax: Obj.Arm(target_systems,1), for Arming, Obj,Arm(target_systems,1) for Disarming
            if min(size(target_systems)>1)
                error('Input argument, target systems, should be 1-D array!')
            end
            if max(target_systems) > obj.number_of_target_systems
                error('target systems should be within the specified number of targets!');
            end

            if obj.IsConnect

                %timeout=10; % seconds
                %if (flag) % Arm
                    for agent=target_systems
                        [Buff,~]=mavlink('Arming', agent, flag);
                        for i=1:2
                            %if (~obj.IsArmed(agent))
                                obj.holdBuff(agent, obj.armInd)=Buff;
                                obj.Msgflag(agent, obj.arm_i)=1;
                                pause(0.01)
                            %else
                                %break;
                            %end
                        end

%                         if obj.IsArmed(agent)
%                             disp(['Mav ',num2str(agent),' is Armed!'])
%                         else
%                             error(['ERROR: Could not Arm Mav ',num2str(agent),' (timeout)',' check that Mav is ON, or try again!'])
%                         end
                    end
                %else % disarm
%                     for agent=target_systems
%                         [Buff,~]=mavlink('Arming', agent, flag);
%                         for i=1:2
%                             if (obj.IsArmed(agent))
%                                 obj.holdBuff(agent, obj.armInd)=Buff;
%                                 obj.Msgflag(agent, obj.arm_i)=1;
%                                 pause(0.1)
%                             else
%                                 break;
%                             end
%                         end
% 
% %                         if ~obj.IsArmed(agent)
% %                             disp(['Mav ',num2str(agent),' is DisArmed!'])
% %                         else
% %                             error(['ERROR: Could not DisArm Mav ',num2str(agent),' (timeout)',' check that Mav is ON, or try again!'])
% %                         end
%                     end

                %end
            else
                error('Port is Not connected')
            end
        end% end of Arm
        
%         function setGimbal(obj, target_sys,target_comp,roll,pitch,yaw)
%             % This function sends comands to control the gimbal
%             % Works for ONE target system at a time.
%             %
%             %   Syntax: Obj.setGimbal(target_systems, target_comp, roll,
%             %   pitch, yaw);
%             %   angles are in rad.
%             if length(target_sys) >1
%                 error('Input argument, target systems, should be only one at a time!');
%             end
%             if target_sys > obj.number_of_target_systems
%                 error('target system should be within the specified number of targets!');
%             end
%             if target_sys < 1
%                 error('target_system can not be less thatn 1');
%             end
%             
%             [Buff,~]=mavlink('gimbalCommand',target_sys,target_comp,roll,...
%                                 pitch,yaw);
%             obj.holdBuff(target_sys, obj.gimbalInd)=Buff;
%             obj.Msgflag(target_sys, obj.gimbal_i)=1;
%         end% End: setGimbal

        % set manual mode
        function setManual(obj, target_systems)
            % This function sends a command to the flight controller to
            % switch to the MANUAL mode.
            %
            %   Syntax: Obj.setManual(target_systems)
            if min(size(target_systems)>1)
                error('Input argument, target systems, should be 1-D array!')
            end
            if max(target_systems) > obj.number_of_target_systems
                error('target systems should be within the specified number of targets!');
            end
            if obj.IsConnect
                for agent=target_systems
                    [Buff,~]=mavlink('setManual',agent,0);
                    obj.holdBuff(agent, obj.ManualInd)=Buff;
                    obj.Msgflag(agent, obj.Manual_i)=1;
                end
            else
                error('Connect to Port first!')
            end
        end % end of Manual mode
        
        function takeoff(obj, target_systems)
            if min(size(target_systems)>1)
                error('Input argument, target systems, should be 1-D array!')
            end
            if max(target_systems) > obj.number_of_target_systems
                error('target systems should be within the specified number of targets!');
            end
            if obj.IsConnect
                for agent=target_systems
%                     if obj.IsArmed(agent)
                        if obj.get_LocalNED(agent).z < -1.0
                            disp(['Mav ', num2str(agent), ' is already in the air!'])
                        else
                            obj.setPosition(agent).x=obj.get_LocalNED(agent).x;
                            obj.setPosition(agent).y=obj.get_LocalNED(agent).y;
                            obj.setPosition(agent).z=obj.takeoffALT(agent);
                            obj.setPosition(agent).Yaw=obj.get_Attitude(agent).yaw;
                            obj.setpointsFlags(agent)=1;
                        end
%                     else % if Mav is not armed!
%                         disp(['Arm Mav ', num2str(agent), ' first!']);
%                     end
                end
                % activate setpoints
                obj.sendSetPoints(1);
                disp('Make sure to switch to OFFBOARD mode.')
            else
                error('Port is not connected!');
            end
        end % end of takeoff method
        
        function Land(obj, target_systems)
            if min(size(target_systems)>1)
                error('Input argument, target systems, should be 1-D array!')
            end
            if max(target_systems) > obj.number_of_target_systems
                error('target systems should be within the specified number of targets!');
            end
            
            if obj.IsConnect
                for agent=target_systems
                    %if obj.IsArmed(agent) % no need to land if disarmed!
                        if obj.get_LocalNED(agent).z > -0.2 % less 20 cm
                            obj.Arm(agent,0);
                        else
                            % TBD
%                             obj.executeCircle(0);
                            obj.setPosition(agent).x=obj.get_LocalNED(agent).x;
                            obj.setPosition(agent).y=obj.get_LocalNED(agent).y;
                            obj.setPosition(agent).z=-0.01;
                            obj.setPosition(agent).Yaw=obj.get_Attitude(agent).yaw;
                            
                        end
                    %end
                end
            end
            
        end % End of Land method
        
%         function executeCircle(obj,flag)
%             if obj.IsConnect
%                 if flag
%                     if strcmp(obj.path_timer.Running, 'on')
%                         disp('Circle execution is already running!')
%                     else
%                         for agent=1:obj.number_of_target_systems
%                             if obj.circleFlags(agent)
%                                 obj.setPosition(agent).x=obj.circle_center(agent,1);
%                                 obj.setPosition(agent).y=obj.circle_center(agent,2);
%                             end
%                         end
%                         start(obj.path_timer)
%                         disp('Circle execution is started.')
%                     end
%                 else
%                     if strcmp(obj.path_timer.Running, 'off')
%                         disp('Circle execution is already stopped!')
%                     else
%                         stop(obj.path_timer)
%                         disp('Circle execution is stopped.')
%                     end
%                     obj.circleFlags=zeros(1,obj.number_of_target_systems);
%                     obj.circle_counter=zeros(1,obj.number_of_target_systems);
%                 end
%             else
%                 error('Connect to Port first!')
%             end
%         end % end: executeCircle
        
        % method to request specific message stream at specific rate
        function RequestStream(obj,sys_id, stream_id, stream_rate, start_stop)
            if obj.IsConnect
                
                [Buff,~]=mavlink('RequestDataStream', sys_id,obj.Comp_ID(sys_id), stream_id, stream_rate,start_stop);
                obj.holdBuff(sys_id, obj.ReqStreamInd)=Buff;
                obj.Msgflag(sys_id, obj.stream_i)=1;
            else
                error('Start communicatoin first!')
            end
        end
        
        % method to set PWM on AUX outputs, on Pixhawk only
        % only aux1-aux4 working for now
        function setAUX(obj, sys_id, comp_id, aux1, aux2, aux3, aux4, aux5, aux6)
            lb=-1; ub=1;
            if aux1 <lb || aux1 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if aux2 <lb || aux2 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if aux3 <lb || aux3 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if aux4 <lb || aux4 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if aux5 <lb || aux5 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if aux6 <lb || aux6 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            
            control_group=3;% for aux outputs
            [Buff,~]=mavlink('setActuators',sys_id,comp_id,...
                                                    control_group,...
                                                    0,...
                                                    0,...
                                                    0,...
                                                    0,...
                                                    aux4,...
                                                    aux1,...
                                                    aux2,...
                                                    aux3);
            obj.holdBuff(sys_id,obj.actuatrosInd)=Buff;
            obj.Msgflag(sys_id, obj.actautors_i)=1;
        end
        
        function setActuators(obj, sys_id, comp_id,group_id, a1, a2, a3, a4, a5, a6,a7,a8)
            % set actuators group 0,1,2,3
            lb=-1; ub=1;
            if a1 <lb || a1 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a2 <lb || a2 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a3 <lb || a3 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a4 <lb || a4 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a5 <lb || a5 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a6 <lb || a6 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a7 <lb || a7 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end
            if a8 <lb || a8 >ub
                error(['Values are between [',num2str(lb),',',num2str(ub),']'])
            end

            [Buff,~]=mavlink('setActuators',sys_id,comp_id,...
                                                    group_id,...
                                                    a1,...
                                                    a2,...
                                                    a3,...
                                                    a4,...
                                                    a5,...
                                                    a6,...
                                                    a7,...
                                                    a8);
            obj.holdBuff(sys_id,obj.actuatrosInd)=Buff;
            obj.Msgflag(sys_id, obj.actautors_i)=1;
        end
        
        % set a paramter
        function setParamter(obj,sys_id,comp_id,param_name,param_value)
            if sys_id>obj.number_of_target_systems
                error('Target system is not within the defined maximum number of systems.')
            end
            if sys_id<1
                error('Target system cannot be less than 1.')
            end
            
            [Buff,~]=mavlink('setParameter',sys_id,comp_id,param_name,param_value);
            obj.holdBuff(sys_id, obj.paramInd)=Buff;
            obj.Msgflag(sys_id, obj.param_i)=1;
        end
        
        function enableHeartbeat(obj)
            % activates sending heartbeat
            if obj.IsConnect
                obj.hb_start_time=tic;
                obj.run_heartbeat=true;
                disp('Sending heartbeat...');
            else
                error('No established connection.');
            end
        end
        
        function disableHeartbeat(obj)
            % deactivates sending heartbeat
            obj.run_heartbeat=false;
            disp('Sending heartbeat is STOPPED.');
        end
        
        % get number of rigid bodies via natnet
        function n=getnatnet_nrb(~)
            global nrb;
            if ~isempty(nrb) && nrb>0
                n=nrb;
            else
                n=0;
            end
            % reset
            %nrb=0;
            
        end
        
        % get position in NED from natnet
        function pos=getnatnetPosNED(~,value)
            global positionNED;
            if ~isempty(positionNED)
                pos=positionNED(value);
            else
                pos=[];
            end
            
        end
        
        % disconnect natnet object
        function DisconnectNatNet(obj)
            global nrb;
            if ispc
                if obj.natnetObj.IsConnected
                    obj.natnetObj.disable(0);
                    obj.natnetObj.disconnect();
                    obj.IsnatnetConnected = obj.natnetObj.IsConnected;
                end
                
                nrb=0;
                
            else
                error('Only works on Windows.')
            end
        end
        % close/delete port
        function Disconnect(obj)
            % This function disconnects from the port defn=ined by the
            % properties COM and BAUD
            %
            %   Syntax: Obj.Disconnect
            if obj.IsConnect
                

%                 if strcmp(obj.mocap_timer.Running,'on')
%                     stop(obj.mocap_timer)
%                     disp('mocap streaming stopped!')
%                 end

%                 if strcmp(obj.setpoints_timer.Running,'on')
%                     stop(obj.setpoints_timer)
%                     disp('setpoints streaming stopped!')
%                 end

%                 if strcmp(obj.offb_timer.Running,'on')
%                     stop(obj.offb_timer);
%                     disp('setpoints streaming stopped!')
%                 end
%  
                if strcmp(obj.MAIN_timer.Running,'on')
                    stop(obj.MAIN_timer);
                    obj.run_udp_rcv=false;
                    obj.run_heartbeat=false;
                    obj.run_mavconn=false;
                    obj.run_mocap=false;
                    obj.run_setpoints=false;
                    obj.run_fence=false;
                end
                
                
%                 if strcmp(obj.receive_timer.Running,'on')
%                     stop(obj.receive_timer);
%                 end

%                 if strcmp(obj.MavConnection_timer.Running,'on')
%                     stop(obj.MavConnection_timer);
%                 end
                
%                 if strcmp(obj.path_timer.Running,'on')
%                     stop(obj.path_timer);
%                 end
                
%                 if strcmp(obj.heartbeat_timer.Running,'on')
%                     stop(obj.heartbeat_timer);
%                 end
%                 
%                 if strcmp(obj.fence_timer.Running,'on')
%                     stop(obj.fence_timer);
%                 end
                
                
                if isa(obj.serialObject,'serial')
                    if isvalid(obj.serialObject)
                        fclose(obj.serialObject);
                        delete(obj.serialObject)
                        obj.serialObject=[];
                        disp([obj.COM, ' is closed and deleted!'])
                    end
                end

                try
                stat=pnet(obj.udpSockets,'status');
                if stat>0
                    pnet(obj.udpSockets,'close')
                    pnet('closeall')
                    obj.udpSockets=[];
                    disp(['local UDP Port: ', num2str(obj.UDP_localPort), ' is closed!'])

                end
                catch
                    %warning('UDP socket is already closed.')
                end

                obj.IsConnect=false;
                obj.SerialConnected=false;
                obj.UDPConnected=false;

            else % not connected
%                 if strcmp(obj.mocap_timer.Running,'on')
%                     stop(obj.mocap_timer)
%                     disp('Mocap streaming is stopped')
%                 end
                
%                 if strcmp(obj.setpoints_timer.Running,'on')
%                     stop(obj.setpoints_timer)
%                     disp('Setpoint streaming is stopped')
%                 end

%                 if strcmp(obj.offb_timer.Running,'on')
%                     stop(obj.offb_timer)
%                     disp('Offboard streaming is stopped!')
%                 end

                if strcmp(obj.MAIN_timer.Running,'on')
                    stop(obj.MAIN_timer);
                    obj.run_udp_rcv=false;
                    obj.run_heartbeat=false;
                    obj.run_mavconn=false;
                    obj.run_mocap=false;
                    obj.run_setpoints=false;
                    obj.run_fence=false;
                end
                
%                 if strcmp(obj.receive_timer.Running,'on')
%                     stop(obj.receive_timer)
%                 end

%                 if strcmp(obj.MavConnection_timer.Running,'on')
%                     stop(obj.MavConnection_timer)
%                 end
                
%                 if strcmp(obj.path_timer.Running,'on')
%                     stop(obj.path_timer)
%                 end
                
%                 if strcmp(obj.heartbeat_timer.Running,'on')
%                     stop(obj.heartbeat_timer);
%                 end
                
%                 if strcmp(obj.fence_timer.Running,'on')
%                     stop(obj.fence_timer);
%                 end
                
                if isa(obj.serialObject,'serial')
                    if isvalid(obj.serialObject)
                        fclose(obj.serialObject);
                        delete(obj.serialObject)
                        obj.serialObject=[];
                        disp([obj.COM, ' is closed and deleted!'])
                    end
                end

                try
                stat=pnet(obj.udpSockets,'status');
                if stat>0
                    pnet(obj.udpSockets,'close')
                    obj.udpSockets=[];
                    disp(['local UDP Port: ', num2str(obj.UDP_localPort), ' is closed!'])

                end
                catch
                    %warning('UDP socket is already closed.')
                end

                obj.IsConnect=false;
                obj.SerialConnected=false;
                obj.UDPConnected=false;

                %error('Port is not connected!')

            end
            % reset variables
                obj.set_number_of_target_systems(obj.number_of_target_systems);
                disp('Variables are reset.')
        end % end of Disconnect

        function delete(obj)
            % This function deletes and cleans the object from memory
            %
            %   Syntax: Obj.delete
            %obj.Disconnect;
            
            if isa(obj.natnetObj,'natnet')
                try
                    obj.natnetObj.disconnect();
                    obj.natnetObj.delete();
                    obj.natnetObj=[];
                catch
                end
            end
            

%             if isvalid(obj.mocap_timer)
%                 if strcmp(obj.mocap_timer.Running,'on')
%                         stop(obj.mocap_timer)
%                         disp('Mocap streaming is stopped')
%                 end
% 
%                 delete(obj.mocap_timer)
% 
%             end

%             if isvalid(obj.setpoints_timer)
%                 if strcmp(obj.setpoints_timer.Running,'on')
%                         stop(obj.setpoints_timer)
%                         disp('Setpoints streaming is stopped')
%                 end
% 
%                 delete(obj.setpoints_timer)
% 
%             end

%             if isvalid(obj.offb_timer)
%                 if strcmp(obj.offb_timer.Running,'on')
%                         stop(obj.offb_timer)
%                         disp('Offboard streaming is stopped')
%                 end
% 
%                 delete(obj.offb_timer)
% 
%             end

            if isvalid(obj.MAIN_timer)
                if strcmp(obj.MAIN_timer.Running,'on')
                    stop(obj.MAIN_timer);
                    obj.run_udp_rcv=false;
                    obj.run_heartbeat=false;
                    obj.run_mavconn=false;
                    obj.run_mocap=false;
                    obj.run_setpoints=false;
                    obj.run_fence=false;
                end

                delete(obj.MAIN_timer)

            end
            
%             if isvalid(obj.receive_timer)
%                 if strcmp(obj.receive_timer.Running,'on')
%                         stop(obj.receive_timer)
%                 end
% 
%                 delete(obj.receive_timer)
% 
%             end

%             if isvalid(obj.MavConnection_timer)
%                 if strcmp(obj.MavConnection_timer.Running,'on')
%                         stop(obj.MavConnection_timer)
%                 end
% 
%                 delete(obj.MavConnection_timer)
% 
%             end
            
%             if isvalid(obj.path_timer)
%                 if strcmp(obj.path_timer.Running,'on')
%                         stop(obj.path_timer)
%                 end
% 
%                 delete(obj.path_timer)
% 
%             end
            
%             if isvalid(obj.heartbeat_timer)
%                 if strcmp(obj.heartbeat_timer.Running,'on')
%                         stop(obj.heartbeat_timer)
%                 end
% 
%                 delete(obj.heartbeat_timer)
% 
%             end
            
%             if isvalid(obj.fence_timer)
%                 if strcmp(obj.fence_timer.Running,'on')
%                         stop(obj.fence_timer)
%                 end
% 
%                 delete(obj.fence_timer)
% 
%             end
            
            % close serial
            if isa(obj.serialObject,'serial')
                if isvalid(obj.serialObject)
                    try
                        fclose(obj.serialObject);
                        delete(obj.serialObject)
                        obj.serialObject=[];
                        disp([obj.COM, ' is closed and deleted!'])
                    catch
                    end
                end
            end
            
            % close UDP
            try
                stat=pnet(obj.udpSockets,'status');
                if stat>0
                    pnet(obj.udpSockets,'close')
                    obj.udpSockets=[];
                    disp(['local UDP Port: ', num2str(obj.UDP_localPort), ' is closed!'])

                end
            catch
                    %warning('UDP socket is already closed.')
            end
            

            disp('MatMav object is deleted from memory!')

        end % end of delete

    end
%% private, local, functions
methods (Access = private)

    % callback function for serail object, called during reading
    %function serialEventHandler(obj,serialConnection,event)
    function serialEventHandler(obj,serialConnection,~)
%             %%% handle clients commands
%             if obj.IsServerON
%                 size=pnet(obj.server_socket,'readpacket',1024,'noblock');
%                 if(size > 0 )
%                     obj.server_buffIN=pnet(obj.server_socket,'read',obj.udpmaxsize,'double');
%                     % parse messsage, cmdData
%                     obj.parseCMD(obj.server_buffIN);
%                 else
%                     %return;
%                 end
%             end

            bytes = get(serialConnection, 'BytesAvailable');
            if(bytes > 0 ) % we may have alread read the data
                serialConnection.UserData = fread(serialConnection, bytes);
            else
                return;
            end

            obj.MavData=mavlink('readmessage',uint8(obj.serialObject.UserData'),length(obj.serialObject.UserData),obj.number_of_target_systems);
            for agent=1:obj.number_of_target_systems
                
                % get component IDs
                if  ~isempty(obj.MavData(agent).Component_ID) && obj.MavData(agent).Component_ID>0
                    obj.Comp_ID(agent)=obj.MavData(agent).Component_ID;
                end
                
                % get get_Attitude
                if ~isempty(obj.MavData(agent).roll) && obj.MavData(agent).roll>-1000
                    obj.get_Attitude(agent).roll=obj.MavData(agent).roll;
                    obj.get_Attitude(agent).pitch=obj.MavData(agent).pitch;
                    obj.get_Attitude(agent).yaw=obj.MavData(agent).yaw;
                    obj.get_Attitude(agent).rollspeed=obj.MavData(agent).rollspeed;
                    obj.get_Attitude(agent).pitchspeed=obj.MavData(agent).pitchspeed;
                    obj.get_Attitude(agent).yawspeed=obj.MavData(agent).yawspeed;
                    obj.get_Attitude(agent).time_msec=obj.MavData(agent).att_tstamp;
                end
                
                % get attitude target
                if obj.MavData(agent).att_t_ms > 0
                    obj.get_Attitude_target(agent).q0=obj.MavData(agent).q0_t;
                    obj.get_Attitude_target(agent).q1=obj.MavData(agent).q1_t;
                    obj.get_Attitude_target(agent).q2=obj.MavData(agent).q2_t;
                    obj.get_Attitude_target(agent).q3=obj.MavData(agent).q3_t;
                    obj.get_Attitude_target(agent).roll_rate=obj.MavData(agent).roll_rate_t;
                    obj.get_Attitude_target(agent).pitch_rate=obj.MavData(agent).pitch_rate_t;
                    obj.get_Attitude_target(agent).yaw_rate=obj.MavData(agent).yaw_rate_t;
                    obj.get_Attitude_target(agent).thrust=obj.MavData(agent).thrust_t;
                    obj.get_Attitude_target(agent).time_ms=obj.MavData(agent).att_t_ms;
                end

                % get get_LocalNED positions
                if ~isempty(obj.MavData(agent).localNEDx) && obj.MavData(agent).localNEDx>-123456
                obj.get_LocalNED(agent).x=obj.MavData(agent).localNEDx;
                obj.get_LocalNED(agent).y=obj.MavData(agent).localNEDy;
                obj.get_LocalNED(agent).z=obj.MavData(agent).localNEDz;
                obj.get_LocalNED(agent).vx=obj.MavData(agent).vx;
                obj.get_LocalNED(agent).vy=obj.MavData(agent).vy;
                obj.get_LocalNED(agent).vz=obj.MavData(agent).vz;
                obj.get_LocalNED(agent).time_msec=obj.MavData(agent).localned_tstamp;
                end
                
                % get GPS raw data
                if ~isempty(obj.MavData(agent).lat) && obj.MavData(agent).lat >0
                    obj.get_GPS(agent).lat=obj.MavData(agent).lat;
                    obj.get_GPS(agent).lon=obj.MavData(agent).lon;
                    obj.get_GPS(agent).Alt=obj.MavData(agent).alt;
                    obj.get_GPS(agent).time_usec=obj.MavData(agent).gps_tstamp;
                end

                % get if Mav is Armed or not
                if ~isempty(obj.MavData(agent).sys_status) && obj.MavData(agent).sys_status<255
                    obj.MavStat(agent)=obj.MavData(agent).sys_status;
                    if obj.MavStat(agent)==4
                        obj.IsArmed(agent)=true;
                    elseif obj.MavStat(agent)==3
                        obj.IsArmed(agent)=false;
                    end
                end

                % get Mav mode (e.g. MANUAL, ALTCTL, POSCTL, OFFBOARD)
                if ~isempty(obj.MavData(agent).custom_mode) && obj.MavData(agent).custom_mode>1
                    switch obj.MavData(agent).custom_mode
                        case 65536
                            obj.get_vehicle_mode{agent}='MANUAL';
                        case 131072
                            obj.get_vehicle_mode{agent}='ALTCTL';
                        case 393216
                            obj.get_vehicle_mode{agent}='OFFBOARD';
                        otherwise
                            obj.get_vehicle_mode{agent}='Other Mode';
                    end
                end
                
                % get battery status
                if ~isempty(obj.MavData(agent).voltage_battery) && obj.MavData(agent).voltage_battery>0
                    obj.get_Battery_voltage(agent)=obj.MavData(agent).voltage_battery;
                end
                
                if obj.MavData(agent).updated>0
                    obj.MavConncetionStatus(agent)=1;
                end
                
                % get pressure values
                if ~isempty(obj.MavData(agent).abs_pressure) && obj.MavData(agent).abs_pressure~=-123456
%                     obj.get_absolute_pressure(agent)=obj.MavData(agent).abs_pressure;
                    obj.get_pressure_info(agent).get_absolute_pressure=obj.MavData(agent).abs_pressure;
                    obj.get_pressure_info(agent).get_differential_pressure=obj.MavData(agent).diff_pressure;
                    obj.get_pressure_info(agent).time_usec=obj.MavData(agent).highers_tstamp;
                end
                
                % get scaled pressure values
                if ~isempty(obj.MavData(agent).scaled_abs_press) && obj.MavData(agent).scaled_abs_press~=-123456
                    obj.get_pressure_info(agent).scaled_abs_pressure=obj.MavData(agent).scaled_abs_press;
                    obj.get_pressure_info(agent).scaled_diff_pressure=obj.MavData(agent).scaled_diff_press;
                    obj.get_pressure_info(agent).scaled_pressure_time_ms=obj.MavData(agent).scaled_press_tms;
                end
                
                % get IMU data
                if ~isempty(obj.MavData(agent).xacc) && obj.MavData(agent).xacc~=-123456.0
                    obj.get_IMUData(agent).xacc=obj.MavData(agent).xacc;
                    obj.get_IMUData(agent).yacc=obj.MavData(agent).yacc;
                    obj.get_IMUData(agent).zacc=obj.MavData(agent).zacc;
                    obj.get_IMUData(agent).xgyro=obj.MavData(agent).xgyro;
                    obj.get_IMUData(agent).ygyro=obj.MavData(agent).ygyro;
                    obj.get_IMUData(agent).zgyro=obj.MavData(agent).zgyro;
                    obj.get_IMUData(agent).time_usec=obj.MavData(agent).highers_tstamp;
                end

            end
    end % end: serialEventHandler
    
    
    % UDP receive callback function
    %%%% TODO::: adapt to use new UDP function, pnet
    function udp_rcv_callback(obj)
%         %%% handle clients commands
%             if obj.IsServerON
%                 size=pnet(obj.server_socket,'readpacket',1024,'noblock');
%                 if(size > 0 )
%                     obj.server_buffIN=pnet(obj.server_socket,'read',obj.udpmaxsize,'double');
%                     % parse messsage, cmdData
%                     obj.parseCMD(obj.server_buffIN);
%                 else
%                     %return;
%                 end
%             end
            
            
            size=pnet(obj.udpSockets,'readpacket',obj.udpmaxsize,'noblock');
            if(size > 0 ) % we may have already read the data
                udpdata=pnet(obj.udpSockets,'read',obj.udpmaxsize,'uint8');
            else
                return;
            end
            
            obj.MavData=mavlink('readmessage',udpdata,size,obj.number_of_target_systems);
            
            for agent=1:obj.number_of_target_systems
                
                % match host address
                if obj.autoip
%                     if obj.MavData(agent).ID ==agent
%                         [ip,port]=pnet(obj.udpSockets,'gethost');
%                         obj.UDP_dest{agent,1}=[num2str(ip(1)),'.',num2str(ip(2)),'.',num2str(ip(3)),'.',num2str(ip(4))];
%                         obj.UDP_dest{agent,2}=port;
%                     end
                    [ip,port]=pnet(obj.udpSockets,'gethost');
                    ipstr=[num2str(ip(1)),'.',num2str(ip(2)),'.',num2str(ip(3)),'.',num2str(ip(4))];
                    if strcmp(obj.UDP_dest{agent,1},ipstr)==1
                        obj.UDP_dest{agent,2}=port;
                    end
                end
                
                % get component IDs
                if  ~isempty(obj.MavData(agent).Component_ID) && obj.MavData(agent).Component_ID>0
                    obj.Comp_ID(agent)=obj.MavData(agent).Component_ID;
                end
                
                % get get_Attitude
                if ~isempty(obj.MavData(agent).roll) && obj.MavData(agent).roll>-1000
                    obj.get_Attitude(agent).roll=obj.MavData(agent).roll;
                    obj.get_Attitude(agent).pitch=obj.MavData(agent).pitch;
                    obj.get_Attitude(agent).yaw=obj.MavData(agent).yaw;
                    obj.get_Attitude(agent).rollspeed=obj.MavData(agent).rollspeed;
                    obj.get_Attitude(agent).pitchspeed=obj.MavData(agent).pitchspeed;
                    obj.get_Attitude(agent).yawspeed=obj.MavData(agent).yawspeed;
                    obj.get_Attitude(agent).time_msec=obj.MavData(agent).att_tstamp;
                end
                
                % get attitude target
                if obj.MavData(agent).att_t_ms > 0
                    obj.get_Attitude_target(agent).q0=obj.MavData(agent).q0_t;
                    obj.get_Attitude_target(agent).q1=obj.MavData(agent).q1_t;
                    obj.get_Attitude_target(agent).q2=obj.MavData(agent).q2_t;
                    obj.get_Attitude_target(agent).q3=obj.MavData(agent).q3_t;
                    obj.get_Attitude_target(agent).roll_rate=obj.MavData(agent).roll_rate_t;
                    obj.get_Attitude_target(agent).pitch_rate=obj.MavData(agent).pitch_rate_t;
                    obj.get_Attitude_target(agent).yaw_rate=obj.MavData(agent).yaw_rate_t;
                    obj.get_Attitude_target(agent).thrust=obj.MavData(agent).thrust_t;
                    obj.get_Attitude_target(agent).time_ms=obj.MavData(agent).att_t_ms;
                end

                % get get_LocalNED positions
                if ~isempty(obj.MavData(agent).localNEDx) && obj.MavData(agent).localNEDx>-123456
                obj.get_LocalNED(agent).x=obj.MavData(agent).localNEDx;
                obj.get_LocalNED(agent).y=obj.MavData(agent).localNEDy;
                obj.get_LocalNED(agent).z=obj.MavData(agent).localNEDz;
                obj.get_LocalNED(agent).vx=obj.MavData(agent).vx;
                obj.get_LocalNED(agent).vy=obj.MavData(agent).vy;
                obj.get_LocalNED(agent).vz=obj.MavData(agent).vz;
                obj.get_LocalNED(agent).time_msec=obj.MavData(agent).localned_tstamp;
                end
                
                % get GPS raw data
                if ~isempty(obj.MavData(agent).lat) && obj.MavData(agent).lat >0
                    obj.get_GPS(agent).lat=obj.MavData(agent).lat;
                    obj.get_GPS(agent).lon=obj.MavData(agent).lon;
                    obj.get_GPS(agent).Alt=obj.MavData(agent).alt;
                    obj.get_GPS(agent).time_usec=obj.MavData(agent).gps_tstamp;
                end

                % get if Mav is Armed or not
                if ~isempty(obj.MavData(agent).sys_status) && obj.MavData(agent).sys_status<255
                    obj.MavStat(agent)=obj.MavData(agent).sys_status;
                    if obj.MavStat(agent)==4
                        obj.IsArmed(agent)=true;
                    elseif obj.MavStat(agent)==3
                        obj.IsArmed(agent)=false;
                    end
                end

                % get Mav mode (e.g. MANUAL, ALTCTL, POSCTL, OFFBOARD)
                if ~isempty(obj.MavData(agent).custom_mode) && obj.MavData(agent).custom_mode>1
                    switch obj.MavData(agent).custom_mode
                        case 65536
                            obj.get_vehicle_mode{agent}='MANUAL';
                        case 131072
                            obj.get_vehicle_mode{agent}='ALTCTL';
                        case 393216
                            obj.get_vehicle_mode{agent}='OFFBOARD';
                        otherwise
                            obj.get_vehicle_mode{agent}='MANUAL';
                    end
                end

                if ~isempty(obj.MavData(agent).voltage_battery) && obj.MavData(agent).voltage_battery>0
                    obj.get_Battery_voltage(agent)=obj.MavData(agent).voltage_battery;
                end
                
                if obj.MavData(agent).updated>0
                    obj.MavConncetionStatus(agent)=1;
                end
                
                % get pressure values
                if ~isempty(obj.MavData(agent).abs_pressure) && obj.MavData(agent).abs_pressure~=-123456
%                     obj.get_absolute_pressure(agent)=obj.MavData(agent).abs_pressure;
                    obj.get_pressure_info(agent).get_absolute_pressure=obj.MavData(agent).abs_pressure;
                    obj.get_pressure_info(agent).get_differential_pressure=obj.MavData(agent).diff_pressure;
                    obj.get_pressure_info(agent).time_usec=obj.MavData(agent).highers_tstamp;
                end
                
                % get scaled pressure values
                if ~isempty(obj.MavData(agent).scaled_abs_press) && obj.MavData(agent).scaled_abs_press~=-123456
                    obj.get_pressure_info(agent).scaled_abs_pressure=obj.MavData(agent).scaled_abs_press;
                    obj.get_pressure_info(agent).scaled_diff_pressure=obj.MavData(agent).scaled_diff_press;
                    obj.get_pressure_info(agent).scaled_pressure_time_ms=obj.MavData(agent).scaled_press_tms;
                end
                
                % get IMU data
                if ~isempty(obj.MavData(agent).xacc) && obj.MavData(agent).xacc>-123456.0
                    obj.get_IMUData(agent).xacc=obj.MavData(agent).xacc;
                    obj.get_IMUData(agent).yacc=obj.MavData(agent).yacc;
                    obj.get_IMUData(agent).zacc=obj.MavData(agent).zacc;
                    obj.get_IMUData(agent).xgyro=obj.MavData(agent).xgyro;
                    obj.get_IMUData(agent).ygyro=obj.MavData(agent).ygyro;
                    obj.get_IMUData(agent).zgyro=obj.MavData(agent).zgyro;
                    obj.get_IMUData(agent).time_usec=obj.MavData(agent).highers_tstamp;
                end

            end
            
            
    end % end of udp_rcv_callback()

    %function mocap_ToSend(obj,tmr,~)
    function mocap_ToSend(obj)
        for agent=1:obj.number_of_target_systems
            if obj.MocapFlags(agent)
                [Buff,~]=mavlink('mocap',agent,0, obj.setMocap(agent).qw,...
                                                  obj.setMocap(agent).qx,...
                                                  obj.setMocap(agent).qy,...
                                                  obj.setMocap(agent).qz,...
                                                  obj.setMocap(agent).x,...
                                                  obj.setMocap(agent).y,...
                                                  obj.setMocap(agent).z);
                obj.holdBuff(agent,obj.mocapInd)=Buff;
                obj.Msgflag(agent,obj.mocap_i)=1;
            end
        end
    end % end mocap_ToSend

    %function setpoints_ToSend(obj,tmr,~)
    function setpoints_ToSend(obj)
        for agent=1:obj.number_of_target_systems
            if obj.setpointsFlags(agent)
                % is it position/velocity set points
                if obj.setPointMask == obj.position_yaw_flag || obj.setPointMask == obj.velocity_yaw_flag || obj.setPointMask == obj.acceleration_flag
                    [Buff,~]=mavlink('setPoints',agent,0, obj.setPosition(agent).x,...
                                                          obj.setPosition(agent).y,...
                                                          obj.setPosition(agent).z,...
                                                          obj.setPosition(agent).Yaw,...
                                                          obj.setVelocity(agent).vx,...
                                                          obj.setVelocity(agent).vy,...
                                                          obj.setVelocity(agent).vz,...
                                                          obj.setPointMask);
                    obj.holdBuff(agent, obj.setptInd)=Buff;
                    obj.Msgflag(agent, obj.setpt_i)=1;
                % is it thrust set points
                elseif obj.setPointMask == obj.thrust_flag
                    [Buff,~]=mavlink('setAttitudeTarget_thrust',agent,0,obj.thrust_setpoint(agent));
                    obj.holdBuff(agent, obj.att_targetInd)=Buff;
                    obj.Msgflag(agent, obj.att_target_i)=1;
                % is it angular rates set points
                elseif obj.setPointMask == obj.angular_rates_flag
                    [Buff,~]=mavlink('setAttitudeTarget_rates',agent,0,obj.angular_rates_sp(agent).roll_rate,...
                                                                        obj.angular_rates_sp(agent).pitch_rate,...
                                                                        obj.angular_rates_sp(agent).yaw_rate);
                    obj.holdBuff(agent, obj.att_targetInd)=Buff;
                    obj.Msgflag(agent, obj.att_target_i)=1;
                % is it attitude set points
                elseif obj.setPointMask == obj.attitude_target_flag
                    [Buff,~]=mavlink('setAttitudeTarget_q',agent,0,obj.quaternion_sp(agent,1),...
                                                                    obj.quaternion_sp(agent,2),...
                                                                    obj.quaternion_sp(agent,3),...
                                                                    obj.quaternion_sp(agent,4));
                    obj.holdBuff(agent, obj.att_targetInd)=Buff;
                    obj.Msgflag(agent, obj.att_target_i)=1;
                end
            end
            % for actuators setpoints (to be removed)
            if obj.actuatorsFlags(agent)
                control_group=0;
                [Buff,~]=mavlink('setActuators',agent,obj.Comp_ID,...
                                                        control_group,...
                                                        obj.actuators(agent,1),...
                                                        obj.actuators(agent,2),...
                                                        obj.actuators(agent,3),...
                                                        obj.actuators(agent,4),...
                                                        obj.actuators(agent,5),...
                                                        obj.actuators(agent,6),...
                                                        obj.actuators(agent,7),...
                                                        obj.actuators(agent,8));
                obj.holdBuff(agent,obj.actuatrosInd)=Buff;
                obj.Msgflag(agent, obj.actautors_i)=1;
            end
        end
    end % end setpoints_ToSend

    %function offb_ToSend(obj,tmr,~)
    function offb_ToSend(obj,~,~)
        for agent=1:obj.number_of_target_systems
            if obj.setpointsFlags(agent)
                [Buff,~]=mavlink('offboard', agent, 0, obj.offb_flag);
                obj.holdBuff(agent, obj.offboardInd)=Buff;
                obj.Msgflag(agent, obj.offboard_i)=1;
            end
        end
    end

    %function updateMavConncetion(obj,tmr,~)
    function updateMavConncetion(obj)
        for agent=1:obj.number_of_target_systems
            if obj.MavConncetionStatus(agent)
                obj.MavConncetionStatus(agent)=0;
            end
        end
    end
    
    %function sendHeartbeat(obj,tmr,~)
    function sendHeartbeat(obj)
        [Buff,~]=mavlink('heartbeat');
        for a=1:obj.number_of_target_systems
            obj.holdBuff(a, obj.heartbeatInd)=Buff;
            obj.Msgflag(a, obj.heartbeat_i)=1;
        end
    end
    
    %function updatePath(obj,tmr,~)
%     function updatePath(obj,~,~)
%         for agent=1:obj.number_of_target_systems
%             if obj.circleFlags(agent)
%                 obj.pathIsIdle(agent)=1;
%                 %if norm([obj.setPosition(agent).x, obj.setPosition(agent).y]- [obj.get_LocalNED(agent).x, obj.get_LocalNED(agent).y]) <0.2
%                     obj.setPosition(agent).x=obj.circle_radius(agent)*sin(obj.circle_direction(agent)*obj.circle_freq*obj.path_TS*obj.circle_counter)+obj.circle_center(agent,1);
%                     obj.setPosition(agent).y=obj.circle_radius(agent)*cos(obj.circle_freq*obj.path_TS*obj.circle_counter)+obj.circle_center(agent,2);
%                     obj.circle_counter(agent)=obj.circle_counter(agent)+1;
%                 %end
%             else
%                 obj.pathIsIdle=0;
%             end
%         end
%     end
    
    %function fenceFunction(obj,tmr,~)
    function fenceFunction(obj)
        % implement fence algorithm
        if ~obj.IsConnect
            warning('Connect to serial/UDP port first. Fence will be deactivated.');
            obj.DeactivateFence();
        end
        
        if ~obj.run_setpoints
            warning('Setpoints streaming is not active. Fence will be deactivated.');
            obj.DeactivateFence();
        end
        
        for a=1:obj.number_of_target_systems
            
            if ~obj.MavConncetionStatus(a)
                warning(['Link with agent ', num2str(a),' is lost.', ' Fence will be skipped.']);
                continue;
            end
            
            if ~obj.IsArmed(a)
                warning(['Agent ', num2str(a),' is not armed.', ' Fence will be skipped.']);
                continue;
            end
            
            if abs(obj.get_LocalNED(a).z) < 0.1
                warning(['Agent ', num2str(a), ' has not taken off.', ' Fence will be skipped.']);
                continue;
            end
            
            % if we are in the FenceMode=1, go to safe position
            if obj.FenceMode==1
                if (obj.get_LocalNED(a).x > obj.x_fence(a,2)) || (obj.get_LocalNED(a).x < obj.x_fence(a,1))
                    obj.setPosition(a).x=obj.SafePosition(a,1);
                    obj.setPosition(a).y=obj.SafePosition(a,2);
                    obj.setPosition(a).z=obj.SafePosition(a,3);
                    obj.setPointMask=obj.position_yaw_flag;
                    break;
                end
                if (obj.get_LocalNED(a).y > obj.y_fence(a,2)) || (obj.get_LocalNED(a).y < obj.y_fence(a,1))
                    obj.setPosition(a).x=obj.SafePosition(a,1);
                    obj.setPosition(a).y=obj.SafePosition(a,2);
                    obj.setPosition(a).z=obj.SafePosition(a,3);
                    obj.setPointMask=obj.position_yaw_flag;
                    break;
                end
                if ((-1*obj.get_LocalNED(a).z) > obj.z_fence(a,2)) || ((-1*obj.get_LocalNED(a).z) < obj.z_fence(a,1))
                    obj.setPosition(a).x=obj.SafePosition(a,1);
                    obj.setPosition(a).y=obj.SafePosition(a,2);
                    obj.setPosition(a).z=obj.SafePosition(a,3);
                    obj.setPointMask=obj.position_yaw_flag;
                    break;
                end
                % if we are in mode 2, stay at limits
            elseif obj.FenceMode==2
                if (obj.get_LocalNED(a).x > obj.x_fence(a,2))
                    obj.setPosition(a).x=obj.x_fence(a,2);
                    obj.setPointMask=obj.position_yaw_flag;
                end
                if (obj.get_LocalNED(a).x < obj.x_fence(a,1))
                    obj.setPosition(a).x=obj.x_fence(a,1);
                    obj.setPointMask=obj.position_yaw_flag;
                end
                
                if (obj.get_LocalNED(a).y > obj.y_fence(a,2))
                    obj.setPosition(a).y=obj.y_fence(a,2);
                    obj.setPointMask=obj.position_yaw_flag;
                end
                if (obj.get_LocalNED(a).y < obj.y_fence(a,1))
                    obj.setPosition(a).y=obj.y_fence(a,1);
                    obj.setPointMask=obj.position_yaw_flag;
                end
                
                if ((-1*obj.get_LocalNED(a).z) > obj.z_fence(a,2))
                    obj.setPosition(a).z=-1*obj.z_fence(a,2);
                    obj.setPointMask=obj.position_yaw_flag;
                end
                if ((-1*obj.get_LocalNED(a).z) < obj.z_fence(a,1))
                    obj.setPosition(a).z=-1*obj.z_fence(a,1);
                    obj.setPointMask=obj.position_yaw_flag;
                end
            end
            
        end
        
    end

    %function MAIN_timer_callback(obj,src,~)
    function MAIN_timer_callback(obj,~,~)
        %%%%%%%%%%%% handle server/clients commands%%%%%%%%%
            if obj.IsServerON
                size=pnet(obj.server_socket,'readpacket',1024,'noblock');
                if(size > 0 )
                    obj.server_buffIN=pnet(obj.server_socket,'read',obj.udpmaxsize,'double');
                    % parse messsage, cmdData
                    obj.parseCMD(obj.server_buffIN);
                else
                    %return;
                end
            end
            
            %%% handle server transmission
            if obj.IsServerON
                % loop over clients
                for i=1:obj.nClients
                    % sanity check
                    if ~isempty(obj.ClientsAddr{i,1})
                        obj.server_buffOUT=obj.get_client_packet(i);
                        % sanity check
                        if ~isempty(obj.server_buffOUT)
                            try % Failsafe
                                pnet(obj.server_socket,'write',obj.server_buffOUT);              % Write to write buffer
                                pnet(obj.server_socket,'writepacket',obj.ClientsAddr{i,1},...
                                    obj.ClientsAddr{i,2});                                      % Send buffer as UDP packet
                            catch
                            end
                        end
                    end % finished sending to client i
                end % finshed sending to all clients
            end
            
            %%%%%%%%%% END of SERVER/CLIENT handeling %%%%%%%%%%%%%%%%%%
            
            obj.sendBuff=[];
            
            % check if setpoints are activated
            if obj.run_setpoints
                obj.setpoints_ToSend();
            end
            
            % check if mocap data are activated
            if obj.run_mocap
                obj.mocap_ToSend();
            end
            
            % check if we need to send heartbeat msg
            if obj.run_heartbeat
                if toc(obj.hb_start_time)>obj.hb_dt
                    obj.sendHeartbeat();
                    obj.hb_start_time=tic;
                end
            end
            
            % check if we need to update mav connection status
            if obj.run_mavconn
                if toc(obj.mavconn_start_time)>obj.mavconn_dt
                    obj.updateMavConncetion();
                    obj.mavconn_start_time=tic;
                end
            end
            
            % check if we need to activate fence
            if obj.run_fence
                obj.fenceFunction();
            end
            
            % if we are connected via Serial
            if obj.SerialConnected
                
                for agent=1:obj.number_of_target_systems
                    for i=1:obj.NumMessg
                        if obj.Msgflag(agent,i)
                            %obj.sendBuff(obj.MsgInd{i})=obj.holdBuff(obj.MsgInd{i});
                            obj.sendBuff=[obj.sendBuff ,obj.holdBuff(agent, obj.MsgInd{i})];
                            obj.Msgflag(agent,i)=0;
                        end

                    end
                end
                
                % debug -- disp(length(obj.sendBuff))
                % send if send buffer is not empty and passed stream rate
                t2=toc(obj.current_tic);
                if t2 > obj.TX_rate
                    obj.current_tic = tic;
                    if ~isempty(obj.sendBuff)
                        if strcmp(obj.serialObject.PinStatus.ClearToSend,'on')
                                while(obj.serialObject.BytesToOutput>0)
                                end
                                fwrite(obj.serialObject,obj.sendBuff,'uint8');
                        end
                    end
                end% stream rate passed
            
            end % end if we are connected via serial
            
            % If we are connected via UDP
            if obj.UDPConnected
                t2 = toc(obj.current_tic);
                if t2 > obj.TX_rate
                    obj.current_tic = tic;
                    for a=1:obj.number_of_target_systems
                        if ~isempty(obj.UDP_dest{a,1}) && obj.UDP_dest{a,2} >0
                            for i=1:obj.NumMessg
                                if obj.Msgflag(a,i)
                                    obj.Msgflag(a,i)=0;
                                    try % Failsafe
                                        t2=toc(obj.current_tic);
                                        if t2 > obj.TX_rate
                                            pnet(obj.udpSockets,'write',obj.holdBuff(a, obj.MsgInd{i}),'uint8');      % Write to write buffer
                                            pnet(obj.udpSockets,'writepacket',obj.UDP_dest{a,1},obj.UDP_dest{a,2});   % Send buffer as UDP packet
                                            obj.current_tic = tic;
                                        end

                                    catch
                                    end
                                    %pause(0.001);
                                end
                            end
                        end
                    end
                end% stream rate passed
            end % end if UDP connected
            
            
            
            
            %%%%%%%%%%%%% receive UDP packets %%%%%%%%%%
            if obj.run_udp_rcv
                obj.udp_rcv_callback();
            end
      end % end MAIN_timer_callback
      
      
      % natnet callback function
      function getnatnetinfo(obj,~,event)
           %properties TO ADD natnet_rotation natnet_position natnet_positionNED natnet_rotationNED natnet_q_wxyz
            %event.data.OtherMarkers(1)
            % number of available rigid bodies
            nrb=event.data.nRigidBodies;
            disp('debug')
            %frame = evnt.data.iFrame;

            obj.natnet_position=struct('x', cell(nrb,1),...
                               'y', cell(nrb,1),...
                               'z', cell(nrb,1));
            obj.natnet_positionNED=struct('x', cell(nrb,1),...
                               'y', cell(nrb,1),...
                               'z', cell(nrb,1));
            for i=1:nrb
                rb = event.data.RigidBodies( i );
                xyz=double([rb.x, rb.y, rb.z]); % in meters

                obj.natnet_position(i).x=xyz(1);
                obj.natnet_position(i).y=xyz(2);
                obj.natnet_position(i).z=xyz(3);


                rbqx = rb.qx;
                rbqy = rb.qy;
                rbqz = rb.qz ;
                rbqw = rb.qw;

                q = quaternion( rbqx, rbqy, rbqz, rbqw );
                qRot = quaternion( 0, 0, 0, 1);
                q = mtimes( q, qRot);
                a = EulerAngles( q , 'zyx' );
                rx = a( 1 ) * -180.0 / pi;
                %ry = a( 2 ) * -180.0 / pi;
                % should be
                ry = a( 2 ) * 180.0 / pi;

                %rz = a( 3 ) * 180.0 / pi;
                rz = a( 3 ) * -180.0 / pi;
                obj.natnet_rotation = [ rx , ry , rz ];

                % position in NED in meters
                obj.natnet_positionNED(i).x=double(rb.z); obj.natnet_positionNED(i).y=double(-rb.x); obj.natnet_positionNED(i).z=double(-rb.y);
                % Quaternion in NED in degrees
                obj.natnet_q_wxyz.w=rbqw; obj.natnet_q_wxyz.x=rbqz; q_wxyz.y=-rbqx; q_wxyz.z=-rbqy;

                % attitude angles in NED
                qNED = quaternion( rbqw, rbqz, -rbqx, -rbqy );
                aNED = EulerAngles( qNED , 'zyx' );
                obj.natnet_rotationNED=[aNED(1), aNED(2), aNED(3)]*180/pi;

            end
      end % end natnet callback function
      
      function setCommands(obj,cmdHandle)
          %TBD
          % This function activates commands set by remote clients
          % accroding to cmdHandle
          % cmdHandle.CMD, command number (-9997 for subscribtion, -9996 for set commands)
          % cmdHandle.cmdID, command ID in the list
          % cmdHandle.Data, data corresponds to cmdHandle.cmdID
          % cmdHandle.clientID, client ID
          % cmdHandle.vID, requested vehicle ID
          % cmdHandle.tID, requested topic ID
          
          %%% handle subscribtion commands
          if cmdHandle.CMD==-9997 % means subscribtion command
              % vehicle ID is in cmdHandle.Data(1)
              if cmdHandle.Data(2)>0
                obj.sub_M{cmdHandle.clientID,1}(cmdHandle.Data(1),cmdHandle.tID)=1; % subscrive
              elseif cmdHandle.Data(2)==0
                  obj.sub_M{cmdHandle.clientID,1}(cmdHandle.Data(1),cmdHandle.tID)=0; % unsubscribe
              else % negative
                  T=getTopics();
                  % unsbscribe from all topics
                  obj.sub_M{cmdHandle.clientID,1}=zeros(obj.number_of_target_systems,T.nT);
              end
          end
          
          %%% handle set commands
          if cmdHandle.CMD==-9996 % means requested set command
              switch cmdHandle.cmdID
                  case -1
                      % acknowlegement
                      
                      %prepare packet
                      pckt=zeros(1,24);
                      ackID=-9995;
                      pckt(1)=ackID;
                      pckt(2)=cmdHandle.clientID;
                      pckt(3)=-1; % ack code number, in the cmd list
                      pckt(4)=1; % acknowleged
                      pckt(24)=-9999; % end code number
                      % transmit packet
                      try % Failsafe
                        pnet(obj.server_socket,'write',pckt);              % Write to write buffer
                        pnet(obj.server_socket,'writepacket',...
                            obj.ClientsAddr{cmdHandle.clientID,1},...
                            obj.ClientsAddr{cmdHandle.clientID,2});        % Send buffer as UDP packet
                      catch
                          disp(['Could not send acknowlegment to client ',...
                              num2str(cmdHandle.clientID)]);
                      end
                  case 0
                      % set client IP/Port
                      ip=[num2str(cmdHandle.Data(1)),'.',...
                          num2str(cmdHandle.Data(2)),'.',...
                          num2str(cmdHandle.Data(3)),'.',...
                          num2str(cmdHandle.Data(4))];
                      prt=cmdHandle.Data(5);
                      obj.ClientsAddr{cmdHandle.clientID,1}=ip;
                      obj.ClientsAddr{cmdHandle.clientID,2}=prt;
                  case 1
                      % arm cmd: Data.vID, Data.flag
                      %obj.Arm(cmdHandle.Data.vID,cmdHandle.Data.flag);
                      % sanity check
                      if cmdHandle.Data(1)<=obj.number_of_target_systems
                        obj.Arm(cmdHandle.Data(1),cmdHandle.Data(2));
                      end
                  case 2
                      % takeoff: Data.vID
                      %obj.takeoff(cmdHandle.Data.vID);
                      if cmdHandle.Data(1)<=obj.number_of_target_systems
                        obj.takeoff(cmdHandle.Data(1));
                      end
                  case 3
                      % Land: Data.vID
                      %obj.Land(cmdHandle.Data.vID);
                      if cmdHandle.Data(1)<=obj.number_of_target_systems
                        obj.Land(cmdHandle.Data(1));
                      end
                  case 4
                      % setPointMask: Data.mask
                      %obj.set_setPointMask(cmdHandle.Data.mask);
                      obj.set_setPointMask(cmdHandle.Data(1));
                  case 5
                      % setpointsFlags: Data.vID, Data.flag
                      %obj.set_setpointsFlags(cmdHandle.Data.vID,...
                                             %cmdHandle.Data.flag);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_setpointsFlags(cmdHandle.Data(1),...
                                                 cmdHandle.Data(2));
                        end
                  case 6
                      % takeoffALT: Data.vID, Data.alt
%                       obj.set_takeoffALT(cmdHandle.Data.vID,...
%                                          cmdHandle.Data.alt);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_takeoffALT(cmdHandle.Data(1),...
                                             cmdHandle.Data(2));
                        end
                  case 7
                      % PositionSetPoints: Data.vID, Data.x, Data.y, Dat.z, Data.yaw
%                       obj.set_PositionSetPoints(cmdHandle.Data.vID,...
%                                                 cmdHandle.Data.x,...
%                                                 cmdHandle.Data.y,...
%                                                 cmdHandle.Data.z,...
%                                                 cmdHandle.Data.yaw);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_PositionSetPoints(cmdHandle.Data(1),...
                                                    cmdHandle.Data(2),...
                                                    cmdHandle.Data(3),...
                                                    cmdHandle.Data(4),...
                                                    cmdHandle.Data(5));
                        end
                  case 8
                      % VelocitySetPoints:
                      % Data.vID, Data.vx, Data.vy, Dat.vz, Data.yaw
%                       obj.set_VelocitySetPoints(cmdHandle.Data.vID,...
%                                                 cmdHandle.Data.vx,...
%                                                 cmdHandle.Data.vy,...
%                                                 cmdHandle.Data.vz,...
%                                                 cmdHandle.Data.yaw);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_VelocitySetPoints(cmdHandle.Data(1),...
                                                    cmdHandle.Data(2),...
                                                    cmdHandle.Data(3),...
                                                    cmdHandle.Data(4),...
                                                    cmdHandle.Data(5));
                        end
                  case 9
                      % attitude_sp
                      % Data.vID, Data.q0, Data.q1, Data.q2, Data.q3
%                       obj.set_attitude_sp(cmdHandle.Data.vID,...
%                                           cmdHandle.Data.q0,...
%                                           cmdHandle.Data.q1,...
%                                           cmdHandle.Data.q2,...
%                                           cmdHandle.Data.q3);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_attitude_sp(cmdHandle.Data(1),...
                                              cmdHandle.Data(2),...
                                              cmdHandle.Data(3),...
                                              cmdHandle.Data(4),...
                                              cmdHandle.Data(5));
                        end
                  case 10
                      % angular_rates_sp
                      % Data.vID, Data.r, Data.p, Data.y
%                       obj.set_angular_rates_sp(cmdHandle.Data.vID,...
%                                                cmdHandle.Data.r,...
%                                                cmdHandle.Data.p,...
%                                                cmdHandle.Data.y);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_angular_rates_sp(cmdHandle.Data(1),...
                                                   cmdHandle.Data(2),...
                                                   cmdHandle.Data(3),...
                                                   cmdHandle.Data(4));
                        end
                  case 11
                      % thrust_sp: Data.vID, Data.value
%                       obj.set_thrust_sp(cmdHandle.Data.vID,...
%                                         cmdHandle.Data.value);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.set_thrust_sp(cmdHandle.Data(1),...
                                            cmdHandle.Data(2));
                        end
                  case 12
                      % sendSetPoints: Data.flag
%                       obj.sendSetPoints(cmdHandle.Data.flag);
                        obj.sendSetPoints(cmdHandle.Data(1));
                  case 13
                      % toggle_OFFB: Data.vID, Data.flag
%                       obj.toggle_OFFB(cmdHandle.Data.vID,...
%                                       cmdHandle.Data.flag);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.toggle_OFFB(cmdHandle.Data(1),...
                                          cmdHandle.Data(2));
                        end
                  case 14
                      % setManual: Data.vID
%                       obj.setManual(cmdHandle.Data.vID);
                        if cmdHandle.Data(1)<=obj.number_of_target_systems
                            obj.setManual(cmdHandle.Data(1));
                        end
                  case 15
                      % setParamter: Data.vID, Data.cmpID, Data.pname,
                      % Data.pvalue
                      % NEED TO BE REVISITED
%                       obj.setParamter(cmdHandle.Data.vID,...
%                                       cmdHandle.Data.cmpID,...
%                                       cmdHandle.Data.pname,...
%                                       cmdHandle.pvalue);
              end
              
          end
      end % ednd setCommands
      
      function parseCMD(obj, packet)
          % This function parses the command packet coming from clients
          % Then, it sets the erquested commands accordingly
          
          % each packet is standardized as follows
          % total number of fields =24
          packetL=24; % packet length
          % | startID| clietnID | cmdID | field1 | ...| field20 | endID|
          %      1   |    1     |   1   |          20           |   1  |
          % end ID is always-9999
          % startID: -9997=subscribtion cmd ,-9996=set cmd
          sub=-9997; CMD=-9996;
          
          % endID
          endID=-9999;
          
          % find indices of startIDs
          s_ids=find(packet==sub | packet==CMD);
          
          % start pasrsing if and only if startIDs are found
          if numel(s_ids)>0
              for i=s_ids
                  % make sure the packet is valid by checking the endID
                  if packet(i+packetL-1)==endID
                      clientID=packet(i+1);
                      cmdID=packet(i+2);
                      % if it is for subscribtion=-9997
                      switch packet(i)
                          case sub
                              cmd_h.CMD=sub;
                              cmd_h.clientID=clientID;
                              cmd_h.tID=cmdID;
                              cmd_h.Data=packet(i+3:i+3+20-1);
                              % set subscribtion/commands
                              obj.setCommands(cmd_h);
                              
                          % if it is for set command=-9996
                          case CMD
                              cmd_h.CMD=CMD;
                              cmd_h.clientID=clientID;
                              cmd_h.cmdID=cmdID;
                              cmd_h.Data=packet(i+3:i+3+20-1); % payload
                              
                              % set subscribtion/commands
                              obj.setCommands(cmd_h);
                      end % end of parsing indvidual packet
                  end % end if the indvidual packet is valid
              end % end of parsing all packets
          end
      end % end parseCMD()
      
      function pckt=get_subs_packet(obj,tp_n,v_n)
          % this function prepares a packet for a specific topic, for a
          % specific vehicle; % vehicle number
          if v_n>obj.number_of_target_systems
              pckt=[];
              return;
          end
          pckt=zeros(1,24);
          pckt(1)=-9998;    % startID
          pckt(2)=tp_n;     % topic ID
          pckt(3)=v_n;
          pckt(24)=-9999;   % end ID
          switch tp_n
              case 1 % number of vehicles
                  pckt(4)=obj.number_of_target_systems;
              case 2
                  if ~isempty(obj.get_IMUData(v_n).time_usec)
                      pckt(4)=obj.get_IMUData(v_n).time_usec;
                      pckt(5)=obj.get_IMUData(v_n).xacc;
                      pckt(6)=obj.get_IMUData(v_n).yacc;
                      pckt(7)=obj.get_IMUData(v_n).zacc;
                      pckt(8)=obj.get_IMUData(v_n).xgyro;
                      pckt(9)=obj.get_IMUData(v_n).ygyro;
                      pckt(10)=obj.get_IMUData(v_n).zgyro;
                  end
              case 3 % Attitude
                  if ~isempty(obj.get_Attitude(v_n).time_msec)
                      pckt(4)=obj.get_Attitude(v_n).time_msec;
                      pckt(5)=obj.get_Attitude(v_n).roll;
                      pckt(6)=obj.get_Attitude(v_n).pitch;
                      pckt(7)=obj.get_Attitude(v_n).yaw;
                      pckt(8)=obj.get_Attitude(v_n).rollspeed;
                      pckt(9)=obj.get_Attitude(v_n).pitchspeed;
                      pckt(10)=obj.get_Attitude(v_n).yawspeed;
                  end
              case 4 % NED position
                  if ~isempty(obj.get_LocalNED(v_n).time_msec)
                      pckt(4)=obj.get_LocalNED(v_n).time_msec;
                      pckt(5)=obj.get_LocalNED(v_n).x;
                      pckt(6)=obj.get_LocalNED(v_n).y;
                      pckt(7)=obj.get_LocalNED(v_n).z;
                      pckt(8)=obj.get_LocalNED(v_n).vx;
                      pckt(9)=obj.get_LocalNED(v_n).vy;
                      pckt(10)=obj.get_LocalNED(v_n).vz;
                  end
              case 5 % mocap position
                  p=obj.getnatnetPosNED(v_n);
                  if ~isempty(p)
                      pckt(4)=p.x;
                      pckt(5)=p.y;
                      pckt(6)=p.z;
                  end
              case 6 % GPS
                  if ~isempty(obj.get_GPS(v_n).time_usec)
                      pckt(4)=obj.get_GPS(v_n).time_usec;
                      pckt(5)=obj.get_GPS(v_n).lat;
                      pckt(6)=obj.get_GPS(v_n).lon;
                      pckt(7)=obj.get_GPS(v_n).Alt;
                  end
              case 7 % battery
                  if ~isempty(obj.get_Battery_voltage(v_n))
                      pckt(4)=obj.get_Battery_voltage(v_n);
                  end
              case 8 % vehicle mode
                  if ~isemtpy(obj.get_vehicle_mode{v_n})
                      todouble=double(obj.get_vehicle_mode{v_n});
                      L=length(todouble);
                      pckt(4)=L;
                      pckt(5:4+L)=todouble;
                  end
          end
      end % end get_subs_packet
      
      function pckt=get_client_packet(obj,c)
          % this function prepares the final packet for a specific client
          % c is the client number
          
          % find requested topic indices 
          
          if isempty(find( (sum(obj.sub_M{c}))>0 ,1))
              pckt=[];
              return;
          end
          topic_i=find( (sum(obj.sub_M{c}))>0 );
          
          % total packet length
          pL=24*sum(sum(obj.sub_M{c}));
          pckt=zeros(1,pL);
          counter=1;
          for i=1:numel(topic_i)
              % get requested vehicle indicies for a specific topic
              v_i=find(obj.sub_M{c}(: , topic_i(i) ));
              if numel(v_i)>0 % sanity check
                  for j=1:numel(v_i)
                      pckt(counter*24-24+1:counter*24)=obj.get_subs_packet( topic_i(i), v_i(j) );
                      counter=counter+1;
                  end
              end
          end
      end
      
      

end
%% static methods
% obsolete methods
methods (Static)
    function Connect()
            disp('This function is not used anymore. Use ConnectSerial or ConnectUDP')
    end
    
end


      


%------------------------------DONE!---------------------------------------
end

