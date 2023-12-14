classdef  HarmonicController < handle
    properties
        K_ang = 0.5;
        K_lin = 0.08;
        
        
        hm = HarmonicMap();
        
        namespace
        boundary_info_sub
        velocity_pub
        tftree
        
        rate
    end
    
    methods (Access = private)
        function callback(~,msg, test)
            test
            return
            if(~msg.CompFailed)
                [boundaries, isFree, ~] = parseBoundaries(msg);
                % check if order is counter clock wise. 
                % Looks like the outter is and the inner aren't
                % The order depends on the ouput of cv.findContours 
                % But documentation isn't very clear
                % Check out:  https://answers.opencv.org/question/170874/cvfindcontours-orientation/
                for i=1:length(boundaries)
                    try
                        if(~determinePointOrder(boundaries{i}))
                            boundaries{i} = flip(boundaries{i},1);
                            isFree{i} = flip(isFree{i},1);
                        end 
                    catch ME
                        disp(ME.message)
                    end
                end
        
                %calculate transform
                tic
                obj.hm.setBoundaries(boundaries,isFree);
                toc
                obj.hm.plotMap
               
                if(isempty(obj.hm.frontiers_q))
                    disp("Exporation Done!")
                    return
                end
        
                try
                    obj.q_front = obj.hm.getNearestFrontier(robotPos);
                catch
                    disp("Error finding nearest frontier")
                    obj.q_front = obj.hm.frontiers_q(1,:)';
                end
                
            end
        end
    end
    
    methods
        function obj = HarmonicController(ns)
            if (nargin==0)
                obj.namespace='';
            else
                obj.namespace = ns;
            end
            
            obj.boundary_info_sub = rossubscriber(strcat(obj.namespace,'/boundary_info'),'boundary_compute/boundary_info', @obj.callback , DataFormat='struct');
            obj.velocity_pub = rospublisher(strcat(obj.namespace,'/cmd_vel'),'geometry_msgs/Twist', DataFormat='struct');
            obj.tftree = rostf("DataFormat","struct");
            
        end
    end