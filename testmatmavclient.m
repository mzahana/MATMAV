%%
clear classes;
clc;
%%
clid=1; lprt=14555; srvip='127.0.0.1'; srvprt=12000;
cl=MatMavClient(clid,lprt,srvip,srvprt);
%%
cl.ConnectToServer();
%%
cl.subscribe('ATT',1);
%%
cl.unsubscribeAll();
%%
clc
cl.get_Attitude(2)
%%
clc
cl.get_LocalNED(2)
%%
cl.get_IMU(1)
%%
cl.Arm(1,0)
%%
cl.get_NumberOfVehicles
%%
cl.set_setpointsFlags(1,1);
cl.set_PositionSetPoints(1,1,1,1,0);
%%
cl.set_sendSetPoints(0)
%%
close all
target_sys_id=1;
T=1;
d=0.01;
logindx=1;
T=floor(T/d);
log=zeros(1,T);

for i=1:T
    log(i)=cl.get_Attitude(target_sys_id).time_msec;
    pause(d)
end
clc
disp('Done')
%plot(log)
log;
length(unique(log))
%%
cl.Disconnect();

cl.delete();