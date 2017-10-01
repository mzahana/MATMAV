clear;
close all;
clc;
%% preliminaries
%mac=0; pc=1;
os='pc';
% get current directory path
cdir=pwd;
%% compile mavlink
% compile mavlink code
mex mavlink.cpp

%% compile udp library
if ispc
    x=[pwd,'\tcp_udp_ip'];
    cd(x)
end
if ismac
    x=[pwd,'/tcp_udp_ip'];
    cd(x)
end
% compile mex files
if ispc
    mex -O pnet.c ws2_32.lib -DWIN32
end

if ismac
    mex -O pnet.c
end
% go back to folder
cd(cdir)
%% clear
clear;
disp('Done.')
