
%% Make Boundary Points 
%- Intermediate points can be easily extracted via linear interpolation
boundary_points = [0,0;0,6;2,6;2,7;0,7;0,12;6,12;6,10;4,10;4,9;7,9;7,12;12,12;12,7;9,7;9,6;12,6;12,3;9,3;9,2;12,2;12,0;6,0;6,6;7,6;7,7;5,7;5,1;3,1;3,0;0,0]'/12;
boundary_points = flip(boundary_points,2);
obstacle1_points = [0.1,0.2;0.1,0.3;0.3,0.3;0.3,0.2;0.1,0.2]';
obstacle1_points = flip(obstacle1_points,2);
obstacle2_points = [0.6,0.3;0.6,0.4;0.8,0.4;0.8,0.3;0.6,0.3]';
obstacle2_points = flip(obstacle2_points,2);


% === Harmonic Map ===
% === A diffeomorfic transformation f:W->D of the workspace onto a disk is
% calculated as follows - Any point in the interior of the workspace has a unique image in the disk.
NofS=2000; % Number of samples along the workspace boundary
t = linspace(0,1,NofS);
L = [0,cumsum(sqrt(sum(diff(boundary_points,1,2).^2,1)))];
l = L/L(end);
t = union(t,l);
t = uniquetol(t,1e-12);
p_boundary{1} = interp1(l,boundary_points',t,'linear');
% === Obstacle 1 ===
Lo = [0,cumsum(sqrt(sum(diff(obstacle1_points,1,2).^2,1)))];
NofSo=5*round(NofS*Lo(end)/L(end)); % Number of samples along the obstacle boundary
to = linspace(0,1,NofSo);
lo = Lo/Lo(end);
to = union(to,lo);
to = uniquetol(to,1e-12);
p_boundary{2} = interp1(lo,obstacle1_points',to,'linear');
% === Obstacle 2 ===
Lo = [0,cumsum(sqrt(sum(diff(obstacle2_points,1,2).^2,1)))];
NofSo=5*round(NofS*Lo( end)/L(end)); % Number of samples along the obstacle boundary
to = linspace(0,1,NofSo);
lo = Lo/Lo(end);
to = union(to,lo);
to = uniquetol(to,1e-12);
p_boundary{3} = interp1(lo,obstacle2_points',to,'linear');
% ==================

%%

%construct harmonic map object and view it
hm = HarmonicMap(p_boundary);
hm.fig = figure(1);
hm.plotMap()


%map point p to diskspace and find its jacobian
p = [0.8;0.8];
q = hm.map(p(1),p(2));
J = hm.jacobian(p(1),p(2));
%or use [q,J] = hm.compute(p(1),p(2))
hm.fig;
subplot(121)
plot(p(1),p(2),'ro')

subplot(122)
plot(q(1),q(2),'ro')

input('Press any key to continue');

%find the path from p_0 to p_d using the harmonic map
visualize = true;
p_0 = [0.2;0.8];
p_d= [0.9;0.1];
[t,p_path,q_path] = hm.navigate(p_0(1),p_0(2),p_d(1),p_d(2),visualize);

input('Press any key to continue');

%or visually pick you points
[t,p_path,q_path] = hm.navigate();
input('Press any key to continue');


%easily change the boundary
newBoundary = {p_boundary{1}};
hm.setBoundaries(newBoundary);
hm.navigate();
