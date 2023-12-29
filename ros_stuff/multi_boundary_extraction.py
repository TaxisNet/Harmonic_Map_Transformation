#!/usr/bin/env python3
from boundary_extraction import *


class MergingComputation(Computation):
    def __init__(self,compList):
        
        self.computation_list = compList

        self.merged_robots = []
    
        super(MergingComputation, self).__init__('/map_merge')
    #if is merged stop the other computations, publish this one and tell matlab to use the same one
        self.tf_map_frame = 'world'
        self.map_topic =  '/map_merge'+'/map'
        self.map_sub.unregister()
        self.map_sub = rospy.Subscriber(self.map_topic,OccupancyGrid,self.map_msg_callback)


    def map_msg_callback(self,msg):
        self.map_data_tmp = msg.data
        self.mapSize_py = [msg.info.width, msg.info.height]
        self.mapResolution = msg.info.resolution
        self.mapOrigin = [-int(msg.info.origin.position.x),-int(msg.info.origin.position.y)]
        self.robot_radius_in_cells = ceil(self.robot_radius/msg.info.resolution)
        self.robot_radius_in_cells+=1

    def updateToMerged(self, comp):
        comp.tf_map_frame = 'world'
        comp.map_topic =  '/map_merge'+'/map'
        #kills computation
        comp.map_sub.unregister()
        comp.map_data_tmp = None
        
        #now make it use the merged map

    def boundaryExtraction(self, map_data):
        self.isMerged = False
        
        map_data[map_data>0] = 1 #occupied 
        map_data[map_data<0] = -1 # unoccupied
        #zero is unknown
        
        # DILATE IMAGE 
        bound = np.array(map_data>0, dtype=np.uint8)
        kernel = np.ones((2,2), np.uint8)
        img_dilate = cv2.dilate(bound, kernel, iterations=2)
        map_data[img_dilate==1]=1

        # ERODE IMAGE
        kernel = np.zeros((2,2), np.uint8)
        img_dilate = cv2.erode(np.uint8(map_data.copy()), kernel, iterations=1)

         # DILATE FREE
        img_dilatefree = cv2.dilate(np.uint8(map_data.copy()), 255*np.ones((2,2), np.uint8), iterations=1) #(2,2)
        map_data[np.logical_and(img_dilatefree==255,map_data == 0)]= -1


        # DILATE UNK
        # kernel = np.ones((5,5), np.uint8)
        bound = np.array(map_data ==0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound.copy(), kernel, iterations=1)
        map_data[np.logical_and(img_dilate==1,map_data<0)] = 0  

        #DILATE BOUNDARIES BY ROBOT RADIUS
        kernel = np.ones((self.robot_radius_in_cells, self.robot_radius_in_cells), dtype=np.uint8)
        bound = np.array(map_data>0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound, kernel)
        map_data[img_dilate==1]=1



        free = map_data.copy() < 0
        convMat = np.array([[0,1,0],[1,-4,1],[0,1,0]]) #
        bound = self.conv2(free,convMat,mode='same')
        
        obsBound = bound.copy() >0
        obsBound = np.logical_and(obsBound,(map_data>0))

        unknown = map_data.copy()==0
        bound = self.conv2(unknown,convMat,mode='same')
        freeBound = bound.copy() > 0
        freeBound = np.logical_and(freeBound.copy(),(map_data<0))

        boundary = np.zeros(map_data.shape)
        boundary[freeBound] = -1
        boundary[obsBound] = 1

        # Find the connected boundary components 
        bb = (boundary.copy() != 0 )
        bb = np.array(bb, dtype=np.uint8)

        #FINALLY FIND THE CONTOURS
        contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
        #contour_hierarchy_list = self.get_contours_by_hierarchy(contours, hierarchy )

        robot_positions = []
        #GET ROBOT POSITION
        for comp in self.computation_list:
            try:
                (trans,rot) = listener.lookupTransform(self.tf_map_frame, comp.tf_robot_frame, rospy.Time(0))
                #pos in image pixels
                robot_pos = np.double([trans[0]/self.mapResolution,trans[1]/self.mapResolution]) + np.double(self.mapOrigin)/self.mapResolution
                robot_pos = np.round(robot_pos).astype(int)
                #do not remove because it cv2 pointPolygonTest produces an error.
                #robot_pos = (int(robot_pos[0]), int(robot_pos[1]))

                robot_positions.append(robot_pos)
            except:
                print("Error getting pos of {}".format(comp.namespace))
                pass

        ##########
        robot_position= np.double([1/self.mapResolution,2/self.mapResolution]) + np.double(self.mapOrigin)/self.mapResolution
        robot_position = np.round(robot_position).astype(int)
        robot_positions[1]=robot_position
        bou = boundary.copy()
        x = robot_positions[1][0]
        y = robot_positions[1][1]
        print("{} {}".format(x,y))
        bou[:,y] = 2
        plt.imshow(bou, cmap='gray')
        plt.show()
        ###############

        #find outer side of outer contour 
        for i, hierarchy_vec in enumerate(hierarchy[0]):
            if(hierarchy_vec[3]==-1):
                #check if 1st  robot is inside:
                
                if(cv2.pointPolygonTest(contours[i], robot_positions[0], measureDist=False) >= 0):
                    outer_outer_bound_indx = i
                    break

        #now try to find inner side outer of outer contour
        if(hierarchy[0][outer_outer_bound_indx][2]== -1):
            #if no children outer outer will become outer
            outer_bound_indx = outer_outer_bound_indx
            
        else:
            cur_bound_indx = hierarchy[0][outer_outer_bound_indx][2]
            outer_bound_indx = cur_bound_indx
            
            #if mutiple inner outer level boundaries
            while(True):
                if(cv2.pointPolygonTest(contours[cur_bound_indx], robot_positions[0], measureDist=False) >= 0):
                    outer_bound_indx = cur_bound_indx
                    break
                if(hierarchy[0][cur_bound_indx][0]== -1): break
                else: cur_bound_indx = hierarchy[0][cur_bound_indx][0]
            
            #now check if the rest are in the same boundary
            robot_positions.pop(0)
            self.merged_robots.append(self.computation_list[0].namespace)

            for i,rob_pos in enumerate(robot_positions):
                if(cv2.pointPolygonTest(contours[outer_bound_indx], rob_pos, measureDist=False) >= 0):
                    #i+1 to compernsate for the first element that is poped
                    self.merged_robots.append(self.computation_list[i+1].namespace)                    
                    self.isMerged = True
                    print("Merged")
            if(not self.isMerged):
                return None, None, None, None, None
        print(self.merged_robots)
        #append outer boundary to msg
        tmpout = np.array(contours[outer_bound_indx].copy()).reshape(contours[outer_bound_indx].shape[0],contours[outer_bound_indx].shape[2])
        xl = np.ascontiguousarray(np.flipud(tmpout[:,0]))
        yl = np.ascontiguousarray(np.flipud(tmpout[:,1]))
        nl = np.ascontiguousarray(np.size(xl))  
        nb = 1

        in_l = list()
        #get inner obstacles
        inner_bou_indx = hierarchy[0][outer_bound_indx][2]
        while True:
            if(inner_bou_indx!=-1):
                #append inner boundary to msg
                tmp = np.array(contours[inner_bou_indx].copy().reshape(contours[inner_bou_indx].shape[0],contours[inner_bou_indx].shape[2]))
                xl = np.append(xl,tmp[:,0])
                yl = np.append(yl,tmp[:,1])
                nl = np.append(nl,np.size(xl))
                nb += 1

                in_l.append(inner_bou_indx)

                inner_bou_indx = hierarchy[0][inner_bou_indx][0]
            else: break

        # debug
        # print([outer_outer_bound_indx, outer_bound_indx, in_l])
        # print(hierarchy)
        # print("--------------\n")

                

        boundary_out = boundary
        bbcopy = np.array(255*np.ones((np.shape(map_data.copy())[0],np.shape(map_data.copy())[1],3)),dtype=np.uint8)
        bbcopy[freeBound,0] = 0
        bbcopy[freeBound,2] = 0
        bbcopy[obsBound,0] = 0
        bbcopy[obsBound,1] = 0
        bbcopy[int(robot_positions[0][1])-1:int(robot_positions[0][1])+1,int(robot_positions[0][0])-1:int(robot_positions[0][0])+1,:] = 0
        
        #pub map image
        #used for debug only!
       
        self.image_msg =  self.br.cv2_to_imgmsg(bbcopy,"bgr8")
        #self.image_pub.publish(self.image_msg)
        
        # # plots for debugging
        # plt.subplot(231,title="freeBound")
        # plt.imshow(freeBound)
        # plt.subplot(232, title="obsBound")
        # plt.imshow(obsBound)
        # plt.subplot(233,title="booundary")
        # plt.imshow(boundary)
        # plt.subplot(212)
        # plt.plot(xl,yl, 'b-')
        # plt.plot(xl[boundary_out[yl,xl] < 0],yl[boundary_out[yl,xl] < 0],'g*') 
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()

        return xl,yl,nb,nl,boundary_out


if __name__=='__main__':

    rospy.init_node('boundary_comp_node', anonymous=True)
    listener = tf.TransformListener()

    computation_tb0 = Computation(ns='tb3_0')
    computation_tb1 = Computation(ns='tb3_1')

    mc = MergingComputation([computation_tb0, computation_tb1])
    rate = rospy.Rate(0.25)
    
    while not rospy.is_shutdown():
        
        mc.publish_data()
        rate.sleep()