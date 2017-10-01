classdef MatMavClient<handle
%% info
%{
*This class creates a remote connection with MatMav server
* started on Aug 7th 2016
* by Mohamed Abdelkader
* mohamedashraf123@gmail.com
%}
%% private properties
properties (Access=protected)
    socket;
        % udp socket
    inbuff;
        % input buffer
    inbuff_ready;
        % flag if input buffer is ready to be used
    outbuff;
        % output buffer
    outbuff_ready;
        % flag if output buffer is ready to be used
        
    maxbuff=1024;
    
    timer_obj;
    
    commandList;
    
    TopicsList;
    
    ackflag;
        
end
%% Public properties
properties (SetAccess = private)
    clientID;
        % client ID. Should be unique among all clients.
    local_port;
        % client local port
    local_addr;
        % local ip addr
        
    server_addr;
        % {'IP', port};
        
    IsConneceted;
        % conneciton flag
        % true: connected to server, flase: not connected
        
    get_NumberOfVehicles;
    get_IMU;
    get_Attitude;
    get_LocalNED;
    get_MOCAP;
    get_Battery;
    get_Vehicle_Mode;
    get_GPS;
    
end
%% Public methods
methods
    function obj=MatMavClient(clid,lprt,srvip,srvprt)
        % class contructor
        if nargin~=4
            error('Insufficient inputs')
        end
        obj.clientID=clid;
        obj.local_port=lprt;
        obj.server_addr=cell(1,2);
        obj.server_addr{1,1}=srvip;
        obj.server_addr{1,2}=srvprt;
        
        obj.IsConneceted=false;
        
        obj.inbuff_ready=false;
        obj.outbuff_ready=false;
        
        obj.local_addr=[127 0 0 1];
        
        obj.commandList=getCommands();
        obj.TopicsList=getTopics();
        
        obj.get_NumberOfVehicles=0;
        
        obj.ackflag=false;
        
        % create communication timer
        obj.timer_obj=timer;
        obj.timer_obj.Period=0.01;
        obj.timer_obj.ExecutionMode='fixedSpacing';
        obj.timer_obj.TimerFcn={@(src,event)callbackFun(obj,src,event)};
    end
    
    function ConnectToServer(obj)
        % connects client to specified server in server_addr
        
        % sanity check
        if ~isempty(obj.server_addr{1,1})
            
            % create the udp socket
            try
                obj.socket=pnet('udpsocket',obj.local_port,'noblock');
                sec=1e-3;
                pnet(obj.socket,'setreadtimeout',sec);
                pnet(obj.socket,'setwritetimeout',sec)
            catch
                error('pnet is not available or port is not available.');
            end
            
            % run the communication timer
            try
                if ~strcmp(obj.timer_obj.Running,'on')
                    start(obj.timer_obj)
                end
                disp(['Opened UDP socket on local port: ',num2str(obj.local_port)]);
                disp('Requesting acknowlegment...');
                t=0;
                tmax=2; % max wait time [seconds]
                t1=tic;
                obj.sendMyAddr();
                pause(0.01)
                obj.Request_ACK();
                while (t<tmax)
                    if obj.ackflag
                        break;
                    end
                    % send ack message
                    t=toc(t1);
                end
                if obj.ackflag
                    disp('connected');
                    obj.ackflag=false;
                else
                    warning('Could not connect to server. Make sure address is correct, and server is running');
                    obj.ackflag=false;
                    stop(obj.timer_obj)
                    try
                        pnet(obj.socket,'close');
                    catch
                    end
                end
                
            catch
                error('Communication error.')
            end
        end
    end % end ConnectToServer
    
    function Disconnect(obj)
        % unsubscribe to all topics
        
        % stop timer
         if isvalid(obj.timer_obj)
            if strcmp(obj.timer_obj.Running,'on')
                    stop(obj.timer_obj)
            end
        end
        
        % close UDP
        try
            stat=pnet(obj.socket,'status');
            if stat>0
                pnet(obj.socket,'close')
                disp(['local UDP Port: ', num2str(obj.local_port), ' is closed!'])
            end
        catch
                %warning('UDP socket is already closed.')
        end
    end
    
    function set_local_addr(obj,addr)
        obj.local_addr=addr;
    end
    
    function set_local_port(obj,value)
        if obj.IsConneceted
            error('Connection is running. Stop it first.');
        else
            obj.local_port=value;
        end
    end
    
    function Request_ACK(obj)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'ACK'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %comand code number
        obj.outbuff_ready=true;
    end
    
    function sendMyAddr(obj)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'ADDR'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %comand code number
        obj.outbuff(4)=obj.local_addr(1);
        obj.outbuff(5)=obj.local_addr(2);
        obj.outbuff(6)=obj.local_addr(3);
        obj.outbuff(7)=obj.local_addr(4);
        obj.outbuff(8)=obj.local_port;
        
        obj.outbuff_ready=true;
    end
    
    function Arm(obj, vid, flag)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'ARM'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %comand code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=flag;
        obj.outbuff_ready=true;
    end
    
    function Takeoff(obj, vid)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'TAKEOFF'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %comand code number
        obj.outbuff(4)=vid;
        
        obj.outbuff_ready=true;
    end
    
    function Land(obj, vid)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'LAND'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %comand code number
        obj.outbuff(4)=vid;
        
        obj.outbuff_ready=true;
    end
    
    function set_setPointMask(obj, mask)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'setPointMask'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=mask;
        
        obj.outbuff_ready=true;
    end
    
    function set_setpointsFlags(obj,vid,flag)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'setpointsFlags'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=flag;
        
        obj.outbuff_ready=true;
    end
    
    function set_takeoffALT(obj,vid,alt)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'takeoffALT'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=alt;
        
        obj.outbuff_ready=true;
    end
    
    function set_PositionSetPoints(obj, vid, x,y,z,yaw)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'PositionSetPoints'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=x;
        obj.outbuff(6)=y;
        obj.outbuff(7)=z;
        obj.outbuff(8)=yaw;
        
        obj.outbuff_ready=true;
    end
    
    function set_VelocitySetPoints(obj, vid, vx,vy,vz,yaw)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'VelocitySetPoints'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=vx;
        obj.outbuff(6)=vy;
        obj.outbuff(7)=vz;
        obj.outbuff(8)=yaw;
        
        obj.outbuff_ready=true;
    end
    
    function set_attitude_sp(obj,vid,q0,q1,q2,q3)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'attitude_sp'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=q0;
        obj.outbuff(6)=q1;
        obj.outbuff(7)=q2;
        obj.outbuff(8)=q3;
        
        obj.outbuff_ready=true;
    end
    
    function set_angular_rates_sp(obj, vid,r,p,y)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'angular_rates_sp'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=r;
        obj.outbuff(6)=p;
        obj.outbuff(7)=y;
        
        obj.outbuff_ready=true;
    end
    
    function set_thrust_sp(obj, vid, value)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'thrust_sp'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=value;
        
        obj.outbuff_ready=true;
    end
    
    function set_sendSetPoints(obj, flag)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'sendSetPoints'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=flag;
        
        obj.outbuff_ready=true;
    end
    
    function toggle_OFFB(obj, vid, flag)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'toggle_OFFB'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        obj.outbuff(5)=flag;
        
        obj.outbuff_ready=true;
    end
    
    function setManual(obj,vid)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9996; % start code number
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.commandList.labels,'setManual'));
        obj.outbuff(3)=obj.commandList.CMDnumber(i); %command code number
        obj.outbuff(4)=vid;
        
        obj.outbuff_ready=true;
    end
    
    function subscribe(obj, tp,vid)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9997; % start code number (subscribtion)
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.TopicsList.labels,tp));
        if isempty(i)
            error('Topic not found')
        end
        obj.outbuff(3)=obj.TopicsList.Tnumber(i);
        obj.outbuff(4)=vid;
        obj.outbuff(5)=1;
        
        obj.outbuff_ready=true;
    end
    
    function unsubscribe(obj,tp,vid)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9997; % start code number (subscribtion)
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        i=find(strcmp(obj.TopicsList.labels,tp));
        if isempty(i)
            error('Topic not found')
        end
        obj.outbuff(3)=obj.TopicsList.Tnumber(i);
        obj.outbuff(4)=vid;
        obj.outbuff(5)=0;
        
        obj.outbuff_ready=true;
    end
    
    function unsubscribeAll(obj)
        obj.outbuff=zeros(1,24);
        obj.outbuff(1)=-9997; % start code number (subscribtion)
        obj.outbuff(2)=obj.clientID;
        obj.outbuff(24)=-9999;% end code number
        
        obj.outbuff(3)=0;
        obj.outbuff(4)=0;
        obj.outbuff(5)=-1; % negative ==> unsubscribe all
        
        obj.outbuff_ready=true;
    end
    
    
    
    function delete(obj)
        if isvalid(obj.timer_obj)
            if strcmp(obj.timer_obj.Running,'on')
                    stop(obj.timer_obj)
            end
            delete(obj.timer_obj)
        end
        
        % close UDP
        try
            stat=pnet(obj.socket,'status');
            if stat>0
                pnet(obj.socket,'close')
                obj.socket=[];
                disp(['local UDP Port: ', num2str(obj.local_port), ' is closed!'])

            end
        catch
                %warning('UDP socket is already closed.')
        end
        disp('Client is deleted.')
    end
end
%% private methods
methods(Access=private)
    % timer callback function
     function callbackFun(obj,~,event)
         %%% receive packets
         %parse packets
        size=pnet(obj.socket,'readpacket',obj.maxbuff,'noblock');
        if(size > 0 )
            obj.inbuff=pnet(obj.socket,'read',obj.maxbuff,'double');
            % parse messsage, cmdData
            obj.parseData(obj.inbuff);
            obj.inbuff=[];
        end
        
         %%% send packets
         % send only if buffer is ready
         if obj.outbuff_ready
            try % Failsafe
                pnet(obj.socket,'write',obj.outbuff);              % Write to write buffer
                pnet(obj.socket,'writepacket',obj.server_addr{1},...
                    obj.server_addr{2});   % Send buffer as UDP packet
                % reset buffer flag
                obj.outbuff_ready=false;
            catch
                % reset buffer flag
                obj.outbuff_ready=false;
                disp('could not send udp')
            end
         end
     end
     
    function parseOnePacket(obj,pack)
        sub=-9998; ack=-9995;
        switch pack(1)
            case ack
                clid=pack(2);
                ack=pack(4);
                if (clid==obj.clientID && ack==1)
                    obj.ackflag=true;
                end
            case sub
                tpn=pack(2); vn=pack(3);
                switch tpn
                    case 1 % number of vehicles
                        obj.get_NumberOfVehicles=pack(4);
                    case 2 % IMU
                        obj.get_IMU(vn).time_usec=pack(4);
                        obj.get_IMU(vn).xacc=pack(5);
                        obj.get_IMU(vn).yacc=pack(6);
                        obj.get_IMU(vn).zacc=pack(7);
                        obj.get_IMU(vn).xgyro=pack(8);
                        obj.get_IMU(vn).ygyro=pack(9);
                        obj.get_IMU(vn).zgyro=pack(10);
                    case 3 % Attiude
                        obj.get_Attitude(vn).time_msec=pack(4);
                        obj.get_Attitude(vn).roll=pack(5);
                        obj.get_Attitude(vn).pitch=pack(6);
                        obj.get_Attitude(vn).yaw=pack(7);
                        obj.get_Attitude(vn).rollspeed=pack(8);
                        obj.get_Attitude(vn).pitchspeed=pack(9);
                        obj.get_Attitude(vn).yawspeed=pack(10);
                    case 4 % NED position
                        obj.get_LocalNED(vn).time_msec=pack(4);
                        obj.get_LocalNED(vn).x=pack(5);
                        obj.get_LocalNED(vn).y=pack(6);
                        obj.get_LocalNED(vn).z=pack(7);
                        obj.get_LocalNED(vn).vx=pack(8);
                        obj.get_LocalNED(vn).vy=pack(9);
                        obj.get_LocalNED(vn).vz=pack(10);
                    case 5 % mocap position
                        obj.get_MOCAP(vn).x=pack(4);
                        obj.get_MOCAP(vn).y=pack(5);
                        obj.get_MOCAP(vn).z=pack(6);
                    case 6 % GPS
                        obj.get_GPS(vn).time_usec=pack(4);
                        obj.get_GPS(vn).lat=pack(5);
                        obj.get_GPS(vn).lon=pack(6);
                        obj.get_GPS(vn).Alt=pack(7);
                    case 7 % battery
                        obj.get_Battery(vn)=pack(4);
                    case 8
                        L=pack(4);
                        obj.get_Vehicle_Mode{vn}=char(pack(5:4+L));
                end
        end
    end
    
    function parseData(obj, packet)
        % this function parses coming data from server
         % each packet is standardized as follows
          % total number of fields =24
          packetL=24; % packet length
          % | startID| clietnID | cmdID | field1 | ...| field20 | endID|
          %      1   |    1     |   1   |          20           |   1  |
          % end ID is always-9999
          % startID: -9997=subscribtion cmd ,-9996=set cmd
          
          % find indices of startIDs
          sub=-9998; ack=-9995;
          endID=-9999;
          s_ids=find(packet==sub | packet==ack);
          % start pasrsing if and only if startIDs are found
          if numel(s_ids)>0
              for i=s_ids
                  % make sure the packet is valid by checking the endID
                  if packet(i+packetL-1)==endID
                      obj.parseOnePacket(packet(i:i+24-1));
                  end % end if the indvidual packet is valid
              end % end of parsing all packets
          end
    end
end
end