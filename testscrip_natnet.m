%% info
% test script for natnet/matmav
%%
clear classes;
close all;
clc
mav=MatMav(3);
%% connect udp
mav.set_UDPLOCALPRT(14550);
mav.ConnectUDP();
%% connect natnet
mav.set_natnet_clientIP('169.254.29.66');
mav.set_natnet_mocapIP('169.254.151.169');
mav.ConnectNatNet();
%%
mav.getnatnetPosNED(1)
%%
xobj=mav.getnatnetPosNED(1).x;
yobj=mav.getnatnetPosNED(1).y;
zobj=mav.getnatnetPosNED(1).z;
%%
sys=3;
mav.get_LocalNED(sys)
%% set parameters
sys=2;
% XY
MPC_XY_FF=0.5;
MPC_XY_P=0.9;
MPC_XY_VEL_P=0.01;
MPC_XY_VEL_D=0.01;
MPC_XY_VEL_I=0.01;
MPC_XY_VEL_MAX=8;
% Z
MPC_Z_FF=0.5;
MPC_Z_P=1.0;
MPC_Z_VEL_P=0.2;
MPC_Z_VEL_D=0.0;
MPC_Z_VEL_I=0.02;
MPC_Z_VEL_MAX=3;
mav.setParamter(sys,0,'MPC_XY_FF',MPC_XY_FF);
pause(0.01)
mav.setParamter(sys,0,'MPC_XY_P',MPC_XY_P);
pause(0.01)
mav.setParamter(sys,0,'MPC_XY_VEL_P',MPC_XY_VEL_P);
pause(0.01)
mav.setParamter(sys,0,'MPC_XY_VEL_D',MPC_XY_VEL_D);
pause(0.01)
mav.setParamter(sys,0,'MPC_XY_VEL_I',MPC_XY_VEL_I);
pause(0.01)

% mav.setParamter(sys,0,'MPC_Z_FF',MPC_XY_FF);
% pause(0.01)
% mav.setParamter(sys,0,'MPC_Z_P',MPC_XY_P);
% pause(0.01)
% mav.setParamter(sys,0,'MPC_Z_VEL_P',MPC_XY_VEL_P);
% pause(0.01)
% mav.setParamter(sys,0,'MPC_Z_VEL_D',MPC_XY_VEL_D);
% pause(0.01)
% mav.setParamter(sys,0,'MPC_Z_VEL_I',MPC_XY_VEL_I);
% pause(0.01)
% mav.setParamter(sys,0,'MPC_Z_VEL_MAX',MPC_XY_VEL_MAX);
% pause(0.01)
%% arm
mav.Arm(2,1)
%% takeoff
sysID=2;
mav.set_takeoffALT(sysID,-2);
mav.takeoff(sysID);
mav.toggle_OFFB(sysID,1)
%%
x=1;y=-1;z=-1;yaw=0;
mav.set_setpointsFlags(sysID,1);
mav.set_PositionSetPoints(sysID,x,y,z,yaw);
%% land
sysID=2;
mav.Land(sysID);
%% off
mav.toggle_OFFB(sysID,0);
mav.setManual(sysID);
mav.Arm(sysID,0);
%%
mav.get_vehicle_mode
%% set servos
% sys1 
% sys2 -1 - 0.8
SYSID=2;
a=1*ones(1,8);
m.setActuators(SYSID, 0,3, a(1), a(2), a(3), a(4), a(5), a(6),a(7),a(8));
%%
mav.get_Battery_voltage
%% plot data
close all
T=10;
dt=0.01;
fig=figure(1);
axis([0,ceil(T/dt),-4,4])

xlabel('x-north'); ylabel('y-east');% zlabel('z-down');

h1 = animatedline;
h2 = animatedline;
for t=1:ceil(T/dt)
    x=mav.getnatnetPosNED(1).x;
    addpoints(h1,t,x);
    addpoints(h2,t,3);
    drawnow
    pause(dt)
end
%%
%mav.DisconnectNatNet();
mav.Disconnect
mav.delete();
