%%

clear classes;
clc;
%%
m=MatMav(2);
%%
m.findPorts();
%% sreial
m.set_COMPORT('COM3');
m.set_BAUDRATE(115200);
m.ConnectSerial();
%%
m.set_UDPLOCALPRT(14550);
m.set_UDPREMOTEADDR(1,{'192.168.1.110',12345})
m.ConnectUDP();
%%
m.get_LocalNED(1)
%%
m.Arm(1,0)
%%
prt=12000;
nclients=1;
m.startServer(prt,nclients);
%%
sys_id=2;
stream_id=105;% this ID is for IMU data
stream_delay=5;
m.RequestStream(sys_id, stream_id, stream_delay, 1)
%%
close all
target_sys_id=1;
T=1;
d=0.01;
logindx=1;
T=floor(T/d);
log=zeros(1,T);

for i=1:T
    log(i)=m.get_IMUData(target_sys_id).time_usec;
    pause(d)
end
clc
disp('Done')
% plot(log)
log;
length(unique(log))
%%
m.stopServer();

m.Disconnect
m.delete
