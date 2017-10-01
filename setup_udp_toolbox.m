%% setup tcp_udp_ip toolbox
try
addpath('tcp_udp_ip');
catch
    error('tcp_udp_ip folder is not available!');
end

if ispc
    cd tcp_udp_ip
    mex -O pnet.c ws2_32.lib -DWIN32
    cd ..
end

if ismac
    cd tcp_udp_ip
    mex -O pnet.c
    cd ..
end