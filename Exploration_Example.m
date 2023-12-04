
% make boundaries: roughly define boundary shape
% boundary will get upscaled before it's transformed
box = [1,-1;1,1;-1,1;-1,-1;1,-1;]';
L = [0,cumsum(sqrt(sum(diff(box,1,2).^2,1)))];
NofS = 100;
t = linspace(0,1,NofS);
l = L/L(end);
box = interp1(l,box',t,'linear');

%inner obstacle
small_box = 0.1*box;


%define frontiers
sz = [length(box),1];
box_isFree = false(sz);
box_isFree(20:40) = true; % first frontier
box_isFree(77:81) = true; % second frontier

sz = [length(box),1];
small_box_isFree = false(sz);


% 
% 

hm = HarmonicMap();
hm.setBoundaries({box,small_box}, {box_isFree, small_box_isFree})
hm.explore([0;-0.5],true);


