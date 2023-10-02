classdef HarmonicMap < handle
    properties
        %states
        boundaries;
        centers;
        normals;
        control_points;
        outer_boundary;
        
        %no of elements per boundary
        total_elements;
        
        %augmented matrix
        A;
        %weights 
        Cx;
        Cy;
        
        %Outter boundary mapping angle
        theta;
        %the frontier indexes in the angle vec (aka theta)
        front_indx;
        
        %obstacles transform 2xN matrix
        inner_obj_q;
        
        %figure
        fig
    end
    
    methods (Access = private)
        function frontiers(obj)
        %3 cases 
        %1. connected boundary (boundaries{1} is n by 2 matrix)
        %2. single non connected boundary (boundaries{1} is n by 2 matrix)
        %3. multiple non connected boundaries (boundaries{1} is cell array of n by 2 matrix)
            outer_boundary = obj.boundaries{1};
            min_gap = 1e-5;
            if(iscell(outer_boundary))
                %assume that if b.p. is cell then 
                %is is not all coneected
                n = length(outer_boundary);
                new_boundary_points = [];
                new_len = [];
                obj.front_indx = zeros(n,1);

                for i = 1:n       
                    current_boundary = outer_boundary{i};
                    if (i == n), next_boundary = outer_boundary{1};
                    else, next_boundary = outer_boundary{i+1};
                    end

                    len = sqrt(sum(diff(current_boundary,1,1).^2, 2));
                    total_len = sum(len,1);
                    NofP = length(current_boundary);
                    avg_dist = total_len/NofP;

                    front_len = norm(current_boundary(end,:) - next_boundary(1,:));
                    front_NofS = ceil(front_len/avg_dist);

                    obj.front_indx(i) = length(new_len)+length(len)+1;
                    new_len = [new_len; 
                               len; 
                               avg_dist; 
                               zeros(front_NofS-3,1); 
                               avg_dist];


                    t = linspace(0,front_len,front_NofS);
                    front_boundary_points = interp1([0;front_len],...
                        [current_boundary(end,:); next_boundary(1,:)],...
                        t,...
                        'linear'); 
                    if(i==1)
                        new_boundary_points = [new_boundary_points;
                                                current_boundary; 
                                                front_boundary_points(2:end,:)];
                    else
                        new_boundary_points = [new_boundary_points;
                                                current_boundary(2:end,:); 
                                                front_boundary_points(2:end,:)];
                    end
                end
                obj.theta = cumsum(new_len,1);
                obj.theta = 2*pi*(obj.theta ./ obj.theta(end));
                obj.boundaries{1} = new_boundary_points;
                obj.outer_boundary = outer_boundary;
            elseif (norm(outer_boundary(end,:) - outer_boundary(1,:))> min_gap)
                %make this work with loop
                len = sqrt(sum(diff(outer_boundary,1,1).^2, 2));
                total_len =  sum(len,1);
                NofP = length(outer_boundary);
                avg_dist = total_len/NofP;

                front_len = norm(outer_boundary(end,:) - outer_boundary(1,:));
                front_NofS = ceil(front_len/avg_dist);

                new_len = [len; avg_dist; zeros(front_NofS-3,1); avg_dist];
                obj.front_indx = length(len)+1;

                t = linspace(0,front_len,front_NofS);
                front_boundary_points = interp1([0;front_len],...
                    [outer_boundary(end,:); outer_boundary(1,:)],...
                    t,...
                    'linear');

                new_boundary_points = [outer_boundary; front_boundary_points(2:end,:)];
                obj.boundaries{1} = new_boundary_points;
                obj.theta = cumsum(new_len,1);
                obj.theta = 2*pi*(obj.theta ./ obj.theta(end));
                obj.outer_boundary = {outer_boundary};
            else
                len = sqrt(sum(diff(outer_boundary,1,1).^2, 2));
                obj.theta = cumsum(len,1);
                obj.theta = 2*pi*(obj.theta ./ obj.theta(end));
                obj.front_indx = [];
            end
        end
           
        function set_states(obj)
            %set states: centers, normals, control_points, outer_lengths, total_elements
            dist = 1e-5;
        
            obj.centers = cell(1,length( obj.boundaries));
            obj.normals = cell(1,length( obj.boundaries));
            obj.control_points = cell(1,length( obj.boundaries));
            
            for i = 1:length( obj.boundaries)
                
                a =  obj.boundaries{i}(1:end-1,:);
                b =  obj.boundaries{i}(2:end,:);
        
               
                obj.centers{i} = 0.5*(a+b);
        
                if(i == 1)
                    %clockwise for outer
                    rot_matrix = [0 -1; 1 0];
                else
                    %counter-clockwise for inner
                    rot_matrix = [0 1; -1 0];
                end
        
        
                
                norm_vec = (b-a)*rot_matrix';
                %normalize
                norm_vec = norm_vec./sqrt(sum(norm_vec.^2,2));
                obj.normals{i} = norm_vec;

                %add adjacent normals and normalize again
                point_normals = norm_vec+circshift(norm_vec,1);
                point_normals = point_normals./sqrt(sum(point_normals.^2,2));
                
                %last & first point have the same normal
                point_normals = [point_normals; point_normals(1,:)];
                    
                obj.control_points{i} =  obj.boundaries{i}-dist*point_normals;
                
            end
            obj.total_elements = [0 cumsum(cellfun(@length,obj.centers))];
                
        end

        function geom_mtx = get_geom_mtx(obj)
            %calculate geometry matrix

            geom_mtx = zeros(obj.total_elements(end));

            for i = 1:length(obj.control_points)
                for j = 1:length(obj.control_points{i})-1
                    pa = obj.control_points{i}(j,:);
                    pb = obj.control_points{i}(j+1,:);
                    vec = pb - pa;
                    a = vec(1); b = vec(2);
                    a2b2 = a^2 + b^2;
                    len = sqrt(a2b2);
                    for k = 1:length(obj.centers)
                        points((obj.total_elements(k)+1):obj.total_elements(k+1),:)...
                            = obj.centers{k};
                    end
                        %points is 2 vectors of all points p* 
                        %so we calculate H ij(k) for all points
                        % and save them in the matrix geom_mtx

                        %G ij is assumend to be constant
                        x = points(:,1) - pa(1);
                        y = points(:,2) - pa(2);

                        axby = a*x + b*y; 
                        aybx = abs((a*y - b*x));
                        dist_sq = (x-a).^2 + (y-b).^2;

                        wc = log(dist_sq) + ...
                            axby./a2b2 .* log((x.^2+y.^2) ./ dist_sq) + ...
                            2.0 * aybx./a2b2 .* ... 
                            (atan2(axby, aybx) + atan2(a2b2-axby,aybx)) ...
                            -2.0;
                        wc = wc*len;

                        geom_mtx(:,obj.total_elements(i)+j) = wc;

                end
            end
        end

        function univ_mtx = get_univ_mtx(obj)
            %calculate universe matrix
            univ_mtx = zeros(length(obj.centers)-1,  obj.total_elements(end));

            if isempty(univ_mtx)
                return 
            end

            for i = 1:length(obj.control_points)
                for j = 1: length(obj.control_points{i})-1
                    xa = obj.control_points{i}(j,1);
                    ya = obj.control_points{i}(j,2);
                    xb = obj.control_points{i}(j+1,1);
                    yb = obj.control_points{i}(j+1,2);

                    a = xb - xa; b = yb - ya;

                    a2b2 = a^2+ b^2;
                    len = sqrt(a2b2);
                    R = [ xb-xa, -yb+ya; yb-ya, xb-xa];
                    R  = R./len;
                    for k = 2:length(obj.centers)
                        points = obj.centers{k};
                        po = (points - obj.centers{i}(j,:))*R;

                        xop = po(:,1) + (len/2);
                        xom = po(:,1) - (len/2);
                        yo = po(:,2);

                        gradl =  [
                            log((xom.^2+yo.^2)./(yo.^2+xop.^2))/2,...
                            atan(xom./yo)-atan(xop./yo)
                            ];

                        gradw = gradl*R';
                        delta = sum(obj.normals{k}.* gradw, 2);
                        if(i == k)
                                delta(j) = abs(delta(j));
                        end
                        univ_mtx(k-1, obj.total_elements(i)+j) = sum(delta);
                    end
                end
            end                   
        end
        
        function aug_mtx = get_aug_mtx(obj,geom_mtx,univ_mtx)
            %combines geometry and universe matric into augmented matrix
            ne = size(geom_mtx, 1);
            np = size(univ_mtx, 1);
            nt = ne + np;

            if np == 0
                aug_mtx = geom_mtx;
            else
                agm = max(abs(geom_mtx),[],'all'); 
                aum = max(abs(univ_mtx),[],'all');

                aug_mtx = zeros(nt, nt);
                aug_mtx(1:ne, 1:ne) = geom_mtx;
                aug_mtx(ne+1:end, 1:ne) = (agm/aum) * univ_mtx;


                for k =1:length(obj.total_elements)-2       %-1 in k th row
                    len = obj.total_elements(k+2) - obj.total_elements(k+1);
                    aug_mtx(obj.total_elements(k+1)+1:obj.total_elements(k+2), ne+k) =...
                        -ones(len, 1);
                end
            end
        end
        
        function solveMap(obj)
            %Solves the transfomation problem and finds
            %the weights Cx,Cy and the positions on inner obstacles
            obj.A = get_aug_mtx(obj, obj.get_geom_mtx(),obj.get_univ_mtx());
            bx = zeros(size(obj.A,1),1);
            by = zeros(size(obj.A,1),1);
            
            bx(1:obj.total_elements(1+1)) = cos(obj.theta);%actual centres
            by(1:obj.total_elements(1+1))= sin(obj.theta);

            X = linsolve(obj.A,bx);
            Y = linsolve(obj.A,by);

            obj.Cx = X(1:obj.total_elements(end));
            obj.Cy = Y(1:obj.total_elements(end));


            u = X(obj.total_elements(end)+1:end);
            v = Y(obj.total_elements(end)+1:end);
            obj.inner_obj_q = [u,v];
        end
    end
    
    
    
    methods
        function obj = HarmonicMap(boundaries)
            %constructor of map
            % boundaries must be  1 x N cell
            % each containing m x 2 points that define the boundaries
            % BOUNDARY POINTS MUST BE IN A COUNTER CLOCKWISE ORDER.
            obj.boundaries = boundaries;
            obj.frontiers();
            obj.set_states();
            obj.solveMap();

        end
        
        function setBoundaries(obj,boundaries)
            %recalculates the map
            % boundaries must be  1 x N cell
            % each containing m x 2 points that define the boundaries
            % BOUNDARY POINTS MUST BE IN A COUNTER CLOCKWISE ORDER.
            obj.boundaries = boundaries;
            obj.frontiers();
            obj.set_states();
            obj.solveMap();
            if ~isempty(obj.fig)
                clf(obj.fig)
            end

        end
        
        function q = map(obj,x,y)
            %the 2x1 vector q is the mapping of point (x,y) 
            %from workspace to disk space
            
            xo = x; yo = y; 
            wc_vec = zeros(obj.total_elements(end),1);

            for i = 1:length(obj.control_points)
                xa = obj.control_points{i}(1:end-1,1);
                ya = obj.control_points{i}(1:end-1,2);
                xb = obj.control_points{i}(2:end,1);
                yb = obj.control_points{i}(2:end,2);

                a = xb-xa;
                b = yb-ya;
                x = xo-xa;
                y = yo-ya;

                axby = a.*x+b.*y;
                aybx = a.*y-b.*x;
                x2y2 = x.^2+y.^2;
                a2b2 = a.^2+b.^2;

                %length of element
                len  = sqrt(a2b2);
                dist_sq = (x-a).^2 + (y-b).^2;
                wc = log(dist_sq) + ...
                    (axby./a2b2).*log(x2y2./(dist_sq)) + ...
                    2.0 * (aybx./a2b2).* ...
                    (atan(axby./aybx) + atan((a2b2-axby)./aybx))...
                    -2.0;

                    wc = len.*wc;
                    wc_vec(obj.total_elements(i)+1:obj.total_elements(i+1),:) = wc;
            end

            u = obj.Cx' * wc_vec;
            v = obj.Cy' * wc_vec;
            q = [u;v];
        end
        
        function jacob = jacobian(obj,x,y)
            %returns the jacobian point (x,y)
            gradu = [0,0];
            gradv = [0,0];
            for i=1:length(obj.control_points)
                             
                xa = obj.control_points{i}(1:end-1,1);
                ya = obj.control_points{i}(1:end-1,2);
                xb = obj.control_points{i}(2:end,1);
                yb = obj.control_points{i}(2:end,2);
                xv = xb-xa;
                yv = yb-ya;

                %Length of element.
                len = sqrt(xv.*xv+yv.*yv);
                xn = xv ./ len;
                yn = yv ./ len;

                %Rotation matrix from local to global coordinate frame.
                R = {xn -yn; yn xn};

                %Center of linear segment.
                xc = 0.5*(xa+xb);
                yc = 0.5*(ya+yb);

                %Point (x,y) w.r.t. to element's local coordinate frame.
                xl = x-xc;
                yl = y-yc;
                xo = R{1,1}.*xl + R{2,1}.*yl;
                yo = R{1,2}.*xl + R{2,2}.*yl;

                %Intermediate values.
                xop = xo+(len/2);
                xom = xo-(len/2);
                xop2 = xop.*xop;
                xom2 = xom.*xom;
                yo2 = yo.*yo;


                ku = obj.Cx(...
                obj.total_elements(i)+1 : obj.total_elements(i+1));

                kv = obj.Cy(...
                obj.total_elements(i)+1 : obj.total_elements(i+1));


                %Unweighted normal gradient expressed in local coordinate frame.
                gradl_x = -0.5*log((yo2 + xom2)./(yo2 + xop2));
                gradl_y = -atan(yo.*(xom-xop)./(yo2 + xom.*xop));
                %Append element's ij components to gradient of u w.r.t. (x, y).
                gradu(1) = gradu(1) +...
                    sum(ku .* ( R{1,1}.*gradl_x + R{1,2}.*gradl_y),1);
                gradu(2) = gradu(2) +... 
                    sum(ku .* ( R{2,1}.*gradl_x + R{2,2}.*gradl_y),1);
                %Append element's ij components to gradient of v w.r.t. (x, y).
                gradv(1) = gradv(1) +...
                    sum(kv .* ( R{1,1}.*gradl_x + R{1,2}.*gradl_y),1);
                gradv(2) = gradv(2) +...
                    sum(kv .* ( R{2,1}.*gradl_x + R{2,2}.*gradl_y),1);


            end
            jacob = [gradu; gradv];
        end
        
        function [q,J] = compute(obj,x,y)
            %returns both the transformed point q
            %and the jacobian of the point (x,y)
            q = obj.map(x,y);
            J = obj.jacobian(x,y);
        end
        
        function plotMap(obj)
            if isempty(obj.fig)
                obj.fig = figure();
            else
                clf(obj.fig)
            end
                
            obj.fig.Name = 'Harmonic Map Transformation';


            min_x = min(obj.boundaries{1}(:,1));
            max_x = max(obj.boundaries{1}(:,1));
            dx = max_x-min_x; 
            min_y = min(obj.boundaries{1}(:,2));
            max_y = max(obj.boundaries{1}(:,2));
            dy = max_y-min_y;
            D = 0.015;
            x_l = min_x-D*dx;
            x_u = max_x+D*dx;
            y_l = min_y-D*dy;
            y_u = max_y+D*dy;

            subplot(121)
            hold on
            if isempty(obj.outer_boundary)
                plot(obj.boundaries{1}(:,1),obj.boundaries{1}(:,2),...
                   'k-','Linewidth',3)
            else
                 plot(obj.boundaries{1}(:,1),obj.boundaries{1}(:,2),...
                   'r-','Linewidth',1)
               for i = 1:length(obj.outer_boundary)
                    plot(obj.outer_boundary{i}(:,1),obj.outer_boundary{i}(:,2),...
                   'k-','Linewidth',3)
               end
               
            end
            
            for i = 2:length(obj.boundaries)
               plot(obj.boundaries{i}(:,1),obj.boundaries{i}(:,2),...
                   'k-','Linewidth',2)
            end
            axis equal
            axis([x_l x_u y_l y_u])
            box on
            %hold off


            subplot(122)

            hold on 
            plot(cos(obj.theta),sin(obj.theta),'k-','LineWidth', 2)
            plot(cos(obj.theta(obj.front_indx)),sin(obj.theta(obj.front_indx)),'ro', 'MarkerSize', 6)
            plot(obj.inner_obj_q(:,1),obj.inner_obj_q(:,2), ...
                'r.','MarkerSize', 15);                
            axis equal
            axis([-1-D 1+D -1-D 1+D])
            box on
            %hold off                
        end
        
        function [t,p_path,q_path] = navigate(obj,x_0,y_0,x_d,y_d, vis)
            % A safe control scheme to navigate from any point [x_0; y_0]
            %towards a desired point [x_d;y_d] is simply calculated by 
            %[ux,uy]=-inv(J(p))*(f(p)-f(p_d)) for any p the belongs
            %to the interior of the workspace.
            %
            %Outputs:
            %t is time vector,
            %p_path is the nx2 matrix of points in the workspace path that 
            %correspond to t 
            %and q_path is the nx2 matrix of the transformed points 
            %in diskspace.
            %
            %Inputs: 
            %1. obj.navigation() makes you choose start and
            %destination points on the workspace plot (left).
            %2. obj.navigation(x_0,y_0,x_d,y_d) finds the paths without any
            %visual plots.
            %3. obj.navigate(obj,x_0,y_0,x_d,y_d, vis) does the same as 2
            %but also visualizes in the map plot if vis is true.
            
            if (nargin == 1)
                obj.plotMap()

                %Starting point
                [x_0,y_0] = ginput(1);
                obj.fig;
                subplot(121)
                plot(x_0,y_0,'ko','MarkerSize', 7)
                q_0 = obj.map(x_0,y_0);
                % q_d is the image of p_d=[x_d;y_d] and J_d is the Jacobian of the mapping
                subplot(122)
                plot(q_0(1),q_0(2),'ko','MarkerSize', 7)


                %Destination point
                [x_d,y_d] = ginput(1);
                obj.fig;
                subplot(121)
                plot(x_d,y_d,'kx','MarkerSize', 10)
                q_d = obj.map(x_d,y_d);
                % q_d is the image of p_d=[x_d;y_d]
                subplot(122)
                plot(q_d(1),q_d(2),'kx','MarkerSize', 10)

                options=odeset('abstol',1e-6,'reltol',1e-6,'events',@my_event);
                [t,p] = ode15s(@system_kin,[0,10],[x_0;y_0],options);
                disp(['Elapsed Time: ',num2str(t(end)),' sec']);
                x=p(:,1)';
                y=p(:,2)';
                len_x = length(x);
                q_points = zeros(2,len_x);
                for i=1:len_x
                    qtemp = obj.map(x(i), y(i));
                    q_points(:,i) = qtemp;
                end

                obj.fig;
                subplot(121)
                plot(x,y,'k','linewidth',1)
                subplot(122)
                plot(q_points(1,:),q_points(2,:),'r','linewidth',1)
            else
                q_0 = obj.map(x_0,y_0);
                q_d = obj.map(x_d,y_d);
                options=odeset('abstol',1e-6,'reltol',1e-6,'events',@my_event);
                [t,p] = ode15s(@system_kin,[0,10],[x_0;y_0],options);
                disp(['Elapsed Time: ',num2str(t(end)),' sec']);
                x=p(:,1)';
                y=p(:,2)';
                len_x = length(x);
                q_points = zeros(2,len_x);
                for i=1:len_x
                    qtemp = obj.map(x(i), y(i));
                    q_points(:,i) = qtemp;
                end
                
                if (nargin==6 && vis)
                    obj.plotMap()
                    obj.fig;
                    %plot p0 & q0
                    subplot(121)
                    plot(x_0,y_0,'ko','MarkerSize', 7)
                    subplot(122)
                    plot(q_0(1),q_0(2),'ko','MarkerSize', 7)
                    
                    %plot pd & qd
                    subplot(121)
                    plot(x_d,y_d,'kx','MarkerSize', 10)
                    subplot(122)
                    plot(q_d(1),q_d(2),'kx','MarkerSize', 10)
                    
                    %plot p path and q path
                    subplot(121)
                    plot(x,y,'k','linewidth',1)
                    subplot(122)
                    plot(q_points(1,:),q_points(2,:),'r','linewidth',1)
                end
            end
            p_path = p;
            q_path = q_points';
            function dx=system_kin(t,x) 
                % The robot kinematics
                q = obj.map(x(1),x(2));
                J = obj.jacobian(x(1),x(2));

                dx=-inv(J)*(q-q_d);
                dx = norm([x(1)-x_d,x(2)-y_d])*dx/(norm(dx)+0.001);             
            end
            
            function [value,isterminal,direction] = my_event(t,x) 
                % Event function to detect when we have arrived closed to the desired point 
                %and terminate earlier the simulation
                
                value = norm([x(1)-x_d;x(2)-y_d])-1e-2;
                isterminal = 1;
                direction = -1;
            end
        end
    end
end