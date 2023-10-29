genDir = '~/catkin_ws/src/exploration_ws/src/';
addpath('/home/taxis/catkin_ws/src/exploration_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
clear classes
rehash toolboxcache

rosinit


% genDir = '~/catkin_ws/src/exploration_ws/src/'
% addpath('/home/taxis/catkin_ws/src/exploration_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
% savepath
% 
% clear classes
% rehash toolboxcache
% global f myMsg


node = ros.Node('/test_nod')

sub = ros.Subscriber(node,'boundary_info','boundary_compute/boundary_info', DataFormat='struct');

%make a topic that stops this
run = 1;
while(run)
    msg = sub.LatestMessage;
    [boundaries, isFree, pos] = parseBoundaries(msg);
    

end

clear node sub
rosshutdown




function [boundaries, isFree, pos] = parseBoundaries(msg)
    indxs = [0; msg.BoundaryIndex];
    n = length(msg.BoundaryIndex);
    boundaries = cell(n,1);
    isFree = cell(n,1);
    for i=1:n
        boundaries{i} = [msg.Xl(indxs(i)+1:indxs(i+1)), msg.Yl(indxs(i)+1:indxs(i+1))];
        boundaries{i} = boundaries{i} .*msg.MapResolution - [msg.MapX0, msg.MapY0];
        pos = [msg.PosX; msg.PosY] .*msg.MapResolution - [msg.MapX0; msg.MapY0];
        isFree{i} = msg.isfree(indxs(i)+1:indxs(i+1));
    end

end