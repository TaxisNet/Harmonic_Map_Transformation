
%make boundaries
box = [1,-1;1,1;-1,1;-1,-1]';
L = [0,cumsum(sqrt(sum(diff(box,1,2).^2,1)))];
NofS = 1000;% Number of samples along the obstacle boundary
t = linspace(0,1,NofS);
l = L/L(end);
t = union(t,l);
t  = uniquetol(t,1e-12);
box = interp1(l,box',t,'linear');

wall = [-0.3, -1; 0.3, -1]';
Lo = [0,cumsum(sqrt(sum(diff(wall,1,2).^2,1)))];
NofSo = ceil(NofS*(Lo(end)/L(end)));
to = linspace(0,1,NofSo);
lo = Lo/Lo(end);
to = union(to,lo);
to = uniquetol(to,1e-12);
wall = interp1(lo,wall',to,'linear');

% 
% 

hm = HarmonicMap({box});
hm.plotMap();


%works
hm.explore([0.9 ; 0.9],1);


hm.setBoundaries({{box, wall}});
hm.explore([0;0],1);

