function c=getCommands()
% this function returns a structure with available set commands
% and there IDs.
% this is used to have a common list f set commands between server, and
% clients

c.labels={'ACK';            % -1 connection acknowledgement with Data.ack 
        'ADDR';             % 0 client IP/port, Data.IP(1)..Data.IP(4), Data.port
        'ARM';              %1 with Data.vID, Data.flag
        'TAKEOFF';          %2 with Data.vID
        'LAND';             %3 with Data.vID
        'setPointMask';     %4 with Data.mask
        'setpointsFlags';   %5 with Data.vID, Data.flag
        'takeoffALT';       %6 with Data.vID, Data.alt
        'PositionSetPoints';%7 with Data.vID, Data.x, Data.y, Dat.z, Data.yaw
        'VelocitySetPoints';%8 with Data.vID, Data.vx, Data.vy, Dat.vz, Data.yaw
        'attitude_sp';      %9 with Data.vID, Data.q0, Data.q1, Data.q2, Data.q3
        'angular_rates_sp'; %10 with Data.vID, Data.r, Data.p, Data.y
        'thrust_sp';        %11 with Data.vID, Data.value
        'sendSetPoints';    %12 with Data.flag
        'toggle_OFFB';      %13 with Data.vID, Data.flag
        'setManual';        %14 with Data.vID
        'setParamter';};    %15
    
c.nCMD=length(c.labels);    % total number of commands
c.CMDnumber=(1:c.nCMD)'-( 2*ones(c.nCMD,1) );      % topic numbers (IDs)
c.nFields=[1,5,2,1,1,1,2,2,5,5,5,4,2,1,2,1,4]';
end