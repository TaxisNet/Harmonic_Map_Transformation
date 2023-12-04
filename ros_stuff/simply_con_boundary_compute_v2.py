#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

import cv2
from scipy.ndimage import convolve
from tf import TransformListener
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Twist
import numpy as np
import tf

from boundary_compute.msg import boundary_info
from geometry_msgs.msg import PoseStamped


class Computation():
    def __init__(self):
          # MAP SUBSCRIBER
        rospy.Subscriber("/map",OccupancyGrid,self.map_msg_callback)
        self.map_size_x = None
        self.map_size_y = None
        self.map_data = None
        self.position = None

        self.mapSize_py = None
        self.mapResolution = None
        self.mapOrigin = None
        self.map_data_tmp = None

        ## OCG PARAMETERS
        self.lo_occ = 50
        self.lo_free = 0
        self.lo_max = 5000
        self.lo_min = -5000

        self.boundary_info_pub = rospy.Publisher('/boundary_info', boundary_info, queue_size = 1)
        self.boundary_info_msg = boundary_info()

        self.image = None
        self.br = CvBridge()

        self.image_pub = rospy.Publisher('/image_bou', Image,queue_size=1)
        self.image_msg = Image()

        ## SUBSCRIBE TO ROBOT POSITION
        self.sub_cur_pos = rospy.Subscriber("/robot_pose", PoseStamped, self.PoseCallback)

    def PoseCallback(self,msg):
        self.tf_robot = msg

    def map_msg_callback(self,msg):
        self.map_data_tmp = msg.data
        self.mapSize_py = [msg.info.width, msg.info.height]
        self.mapResolution = msg.info.resolution
        self.mapOrigin = [-int(msg.info.origin.position.x),-int(msg.info.origin.position.y)]


    def conv2(self,x,y,mode='same'):

        if not(mode == 'same'):
            raise Exception("Mode not supported")

        # Add singleton dimensions
        if (len(x.shape) < len(y.shape)):
            dim = x.shape
            for i in range(len(x.shape),len(y.shape)):
                dim = (1,) + dim
            x = x.reshape(dim)
        elif (len(y.shape) < len(x.shape)):
            dim = y.shape
            for i in range(len(y.shape),len(x.shape)):
                dim = (1,) + dim
            y = y.reshape(dim)

        origin = ()

        # Apparently, the origin must be set in a special way to reproduce
        # the results of scipy.signal.convolve and Matlab
        for i in range(len(x.shape)):
            if ( (x.shape[i] - y.shape[i]) % 2 == 0 and
                x.shape[i] > 1 and
                y.shape[i] > 1):
                origin = origin + (-1,)
            else:
                origin = origin + (0,)

        z = convolve(x,y, mode='constant', origin=origin)

        return z    
    
    def boundaryExtraction(self, map_data, position):
        # xl,yl,nb,nl,boundary_out = [None, None, None, None, None]
        
        map_data[map_data>0] = 1
        map_data[map_data<0] = -1
        
        # DILATE IMAGE 
        bound = np.array(map_data>0, dtype=np.uint8)
        kernel = np.ones((2,2), np.uint8)
        # kernel[0,0] = 0
        img_dilate = cv2.dilate(bound, kernel, iterations=2)

        # ERODE IMAGE
        kernel = np.zeros((2,2), np.uint8)
        map_data[img_dilate==1]=1
        img_dilate = cv2.erode(np.uint8(map_data.copy()), kernel, iterations=1)

         # DILATE FREE
        img_dilatefree = cv2.dilate(np.uint8(map_data.copy()), 255*np.ones((2,2), np.uint8), iterations=1) #(2,2)
        map_data[np.logical_and(img_dilatefree==255,map_data == 0)]= -1


         # DILATE UNK
        # kernel = np.ones((5,5), np.uint8)
        bound = np.array(map_data ==0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound.copy(), kernel, iterations=1)
        map_data[np.logical_and(img_dilate==1,map_data<0)] = 0  

        # FREE CONNECTED
        freeDis = np.ones(map_data.shape, dtype=np.uint8)
        freeDis[map_data<0] = 0

        # IMFILL
        # FUNCTION 
        im_th = freeDis.copy()
        
        h, w = im_th.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)

        cv2.floodFill(freeDis, mask, (int(position[0]),int(position[1])),1,flags=4)

        im_floodfill_inv = freeDis
        freeDis = np.logical_or(im_th,im_floodfill_inv)

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # !!!!!!!!!!!!!!!!!!!!!! THIS FREEDIS CODE IS DISREGARDED AS IT PRODUCES ERRORS
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #map_data[np.logical_not(freeDis.copy())] = 0


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
        
        
        contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
        countour_hierarchy_list = self.get_contours_by_hierarchy(contours, hierarchy )
        
        #img can be published for visual debugging
        
        # img = cv2.cvtColor(bb.copy(),cv2.COLOR_GRAY2RGB)
        # thicc = 1        
        # child_colors = {0:(255,255,255), 1:(255,0,0), 2: (0,0,255)}
        # for i in range(0,min(len(myList),3)):
        #     if i==1: continue
        #     for ci in myList[i]:
                

        #         cv2.drawContours(img, contours, ci, child_colors[i], thicc) 
        #         thicc = 1 if thicc == 3 else 3
        # debug    
        # if(len(countour_hierarchy_list)<1):
        #     print(hierarchy)
        
        outter_b = countour_hierarchy_list[0]
        b_no = outter_b[0]
        if(len(outter_b)>1): 
            print("Possibly Faulty Boundary: Multiple high level boundaries, picking largest")
            max_b = 0
            for b in outter_b:
                if(contours[b].shape[0]>max_b):
                    max_b = contours[b].shape[0]
                    b_no = b
                
        
        tmpout = np.array(contours[b_no].copy()).reshape(contours[b_no].shape[0],contours[b_no].shape[2])
        xl = np.ascontiguousarray(np.flipud(tmpout[:,0]))
        yl = np.ascontiguousarray(np.flipud(tmpout[:,1]))
        nl = np.ascontiguousarray(np.size(xl))  
        nb = 1

        if (len(countour_hierarchy_list)>2):
            inner_b = countour_hierarchy_list[2]
            for boundary_inx in inner_b:
                tmp = np.array(contours[boundary_inx].copy().reshape(contours[boundary_inx].shape[0],contours[boundary_inx].shape[2]))
                xl = np.append(xl,tmp[:,0])
                yl = np.append(yl,tmp[:,1])
                nl = np.append(nl,np.size(xl))
                nb += 1

                

        boundary_out = boundary
        bbcopy = np.array(255*np.ones((np.shape(map_data.copy())[0],np.shape(map_data.copy())[1],3)),dtype=np.uint8)
        bbcopy[freeBound,0] = 0
        bbcopy[freeBound,2] = 0
        bbcopy[obsBound,0] = 0
        bbcopy[obsBound,1] = 0
        bbcopy[int(position[1])-1:int(position[1])+1,int(position[0])-1:int(position[0])+1,:] = 0
        
        #pub map image
        #used for debug only!
        #self.image_msg =  self.br.cv2_to_imgmsg(img ,"bgr8")
        self.image_msg =  self.br.cv2_to_imgmsg(bbcopy,"bgr8")
        self.image_pub.publish(self.image_msg)
        
        
        return xl,yl,nb,nl,boundary_out
    
    def get_contours_by_hierarchy(self, contours, hierarchy ):
        # Organize contours by hierarchy level
        organized_contours = []

        # Create a dictionary to hold the hierarchy levels and corresponding contours
        hierarchy_dict = {}

        for i, contour in enumerate(contours):
            level = hierarchy[0][i][3]  # Get the hierarchy level of the contour
            if level not in hierarchy_dict:
                hierarchy_dict[level] = []
            hierarchy_dict[level].append(i) 

        # Sort contours within each hierarchy level
        for level in sorted(hierarchy_dict.keys()):
            organized_contours.append(hierarchy_dict[level])

        return organized_contours
        




if __name__=='__main__':

    rospy.init_node('boundary_comp_node', anonymous=True)
    listener = tf.TransformListener()

    computation = Computation()
   
    
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():

        failed_comp = False
        # CHECK IF MAP HAS BEEN RECEIVED
        if (not computation.map_data_tmp == None):
            # print("MAP RECEIVED")
            # IMPLEMENTATION OF THE occGridMapping_py FUNCTION
            map_output = np.transpose(np.reshape(computation.map_data_tmp,(computation.mapSize_py[1],computation.mapSize_py[0]),order='F'))
            map_data = map_output.copy()

           
            map_data[map_output.copy() == -1] = 0
            map_data[ np.logical_and(map_output.copy() < 50,map_output.copy() != -1)] = computation.lo_min
            map_data[map_output.copy() >= 50] =computation.lo_max
            map_data_uint8 = np.uint8(map_data.copy())
            
        
            # INFLATE THE BOUNDARY
            kernel = np.ones((1,1), np.uint8) #kernel = np.ones((3,3), np.uint8)
            map_uint8_dilate = cv2.dilate(map_data_uint8, kernel, iterations=1)
            map_dilate = map_data.copy()
            map_dilate[map_uint8_dilate == np.max(map_uint8_dilate)] = computation.lo_max

            # GET ROBOT POSITION
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                computation.position= np.double([trans[0]/computation.mapResolution,trans[1]/computation.mapResolution]) + np.double(computation.mapOrigin)/computation.mapResolution
                computation.position = computation.position.astype(int)
                haspos = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("pos exception")
                failed_comp = True
                continue

            haspos = True
            # TRY COMPUTATION
            if haspos:
                # xl_py, yl_py,nb_py,nl_py,boundary_py = computation.boundaryExtraction( map_dilate, computation.position)
                try:
                    xl_py, yl_py,nb_py,nl_py,boundary_py = computation.boundaryExtraction( map_dilate, computation.position)    
                    failed_comp = False
                except:
                    print("boundary extraction exception")
                    failed_comp = True

            # PARTITION THE BOUNDARY TO FREE AND OCC
            if not failed_comp:
                is_bou_free = np.full(np.shape(xl_py),False)
                is_bou_free[boundary_py[yl_py,xl_py] < 0] = True
                computation.boundary_info_msg.xl = xl_py
                computation.boundary_info_msg.yl = yl_py
                computation.boundary_info_msg.boundary_index = nl_py
                computation.boundary_info_msg.isfree = is_bou_free
                computation.boundary_info_msg.pos_x = computation.position[0]
                computation.boundary_info_msg.pos_y = computation.position[1]
                computation.boundary_info_msg.map_x0 = computation.mapOrigin[0]
                computation.boundary_info_msg.map_y0 = computation.mapOrigin[1]
                computation.boundary_info_msg.map_width = computation.mapSize_py[0]
                computation.boundary_info_msg.map_height = computation.mapSize_py[1]
                computation.boundary_info_msg.map_resolution = computation.mapResolution
                computation.boundary_info_msg.comp_failed = failed_comp
                computation.boundary_info_pub.publish(computation.boundary_info_msg)
                # print("Boundary Info published")
            else: 
                print('Computation Failed')
                computation.boundary_info_msg.comp_failed = failed_comp
                computation.boundary_info_msg.map_resolution = computation.mapResolution
                computation.boundary_info_msg.map_x0 = computation.mapOrigin[0]
                computation.boundary_info_msg.map_y0 = computation.mapOrigin[1]
                computation.boundary_info_msg.map_width = computation.mapSize_py[0]
                computation.boundary_info_msg.map_height = computation.mapSize_py[1]
                computation.boundary_info_pub.publish(computation.boundary_info_msg)
                
            
        else:
            # print('MAP NOT RECEIVED')
            pass
            

        rate.sleep()


    rospy.spin()
