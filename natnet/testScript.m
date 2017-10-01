%% establish communication with Motive (motion capture software)
clear all
clc

c = natnet();
global rotation position positionNED rotationNED q_wxyz Mav1
% Mistake: in order to work, IPs are swapped !
%c.ClientIP = '169.254.141.12';
c.ClientIP = '169.254.151.169'; % mocap IP
%c.HostIP = '169.254.151.169';
c.HostIP = '169.254.29.66'; % laptop IP
c.ConnectionType = 'Multicast';
c.connect
% add 'listener' to listen to rigid bodies' information (position & orientation)
% for now, it receives data of first rigid body only!
c.addlistener( 1 , 'getRigidBodiesfromMocap' );
c.enable(0);% start listenning

%%
position
positionNED

%%
close all
T=30;
dt=0.01;
fig=figure(1);
axis([-4,4,-4,4,-4,4])

xlabel('x-north'); ylabel('y-east'); zlabel('z-down');

h = animatedline;
for t=1:ceil(T/dt)
    addpoints(h,positionNED(1).x,positionNED(1).y,-positionNED(1).z);
    drawnow
    pause(dt)
end
%%
% disconnect NatNet object
c.disable(0)
c.disconnect
