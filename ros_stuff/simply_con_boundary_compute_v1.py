#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

import cv2
from scipy.ndimage.filters import convolve
from std_msgs.msg import Float32MultiArray
from tf import TransformListener
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rospy.numpy_msg import numpy_msg
import numpy as np
import tf
import math 
import traceback
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

        self.image_pub = rospy.Publisher('/image_bou', Image,queue_size=10)
        self.image_msg = Image()

        ## SUBSCRIBE TO DRONE POSITION
        self.sub_cur_pos = rospy.Subscriber("/robot_pose", PoseStamped, self.PoseCallback)

    def PoseCallback(self,msg):
        self.tf_drone = msg


    def map_msg_callback(self,msg):
        self.map_data_tmp = msg.data
        if self.mapResolution==None:
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

    def boundaryExtraction2(self, map_data, position):
        map_data = map_data.copy()
        # plt.imshow(map_data,cmap='hot', interpolation='nearest')
        # plt.colorbar()
        # plt.show()
        # PREPARE IMAGE 
        map_data[map_data>0] = 1
        map_data[map_data<0] = -1
        # map_data_uint8 = np.uint8(map_data.copy())


        # plt.imshow(map_data,cmap='hot', interpolation='nearest')
        # plt.colorbar()  
        # plt.show()
        
        # DILATE IMAGE 
        bound = np.array(map_data>0, dtype=np.uint8)
        kernel = np.ones((2,2), np.uint8)
        # kernel[0,0] = 0
        img_dilate = cv2.dilate(bound, kernel, iterations=2)

        
        # plt.imshow(img_dilate,cmap='hot', interpolation='nearest')
        # plt.colorbar()  
        # plt.show()
        

        # ERODE IMAGE
        kernel = np.zeros((2,2), np.uint8)
        map_data[img_dilate==1]=1
        img_dilate = cv2.erode(np.uint8(map_data.copy()), kernel, iterations=1)

        

        # fig, axs = plt.subplots(3)
        # axs[0].imshow(map_data,cmap='hot', interpolation='nearest')
        
        # DILATE FREE
        img_dilatefree = cv2.dilate(np.uint8(map_data.copy()), 255*np.ones((2,2), np.uint8), iterations=1) #(2,2)
        map_data[np.logical_and(img_dilatefree==255,map_data == 0)]= -1

        # DILATE UNK
        # kernel = np.ones((5,5), np.uint8)
        bound = np.array(map_data ==0, dtype=np.uint8)
        img_dilate = cv2.dilate(bound.copy(), kernel, iterations=1)
        map_data[np.logical_and(img_dilate==1,map_data<0)] = 0      

        # plt.imshow(map_data,cmap='hot', interpolation='nearest')
        # plt.colorbar()  
        # plt.show()

        
        # FREE CONNECTED
        freeDis = np.ones(map_data.shape, dtype=np.uint8)
        freeDis[map_data<0] = 0

        # IMFILL
        # FUNCTION 
        im_th = freeDis.copy()
        freeDis_copy = im_th.copy()
        h, w = im_th.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        
        cv2.floodFill(freeDis, mask, (int(position[0]),int(position[1])),1,flags=4)
        # cv2.floodFill(freeDis, mask, (int(position[0]),int(position[1])),255,flags=4)
        im_floodfill_inv = freeDis
        freeDis = np.logical_or(im_th,im_floodfill_inv)
        # IMPLEMENTATION
        # plt.imshow(map_data,cmap='hot', interpolation='nearest')
        # plt.colorbar()
        # plt.plot(position[0],position[1],'r*')
        # plt.show()

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # !!!!!!!!!!!!!!!!!!!!!! THIS FREEDIS CODE IS DISREGARDED AS IT PRODUCES ERRORS
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        map_data[np.logical_not(freeDis.copy())] = 0
        # plt.imshow(map_data,cmap='hot', interpolation='nearest')
        # plt.colorbar()
        # plt.plot(position[0],position[1],'r*')
        # plt.show()



        # FINDBOUND FUNCTION 
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        free = map_data.copy() < 0
        convMat = np.array([[0,1,0],[1,-4,1],[0,1,0]]) #
        bound = self.conv2(free,convMat,mode='same')
        obsBound = bound.copy() >0
        obsBound = np.logical_and(obsBound,(map_data>0))
        unknown = map_data.copy()==0

        # plt.imshow(map_data,cmap='hot', interpolation='nearest')
        # plt.colorbar()
        # plt.plot(position[0],position[1],'r*')
        # plt.show()

        bound = self.conv2(unknown,convMat,mode='same')
        freeBound = bound.copy() > 0

        freeBound = np.logical_and(freeBound.copy(),(map_data<0))

        boundary = np.zeros(map_data.shape)

        # plt.imshow(unknown,cmap='hot', interpolation='nearest')
        # plt.show()
        # plt.imshow(bound,cmap='hot', interpolation='nearest')
        # plt.show()
        # plt.imshow(freeBound,cmap='hot', interpolation='nearest')
        # plt.show()


        # HERE MISTALK
        boundary[freeBound] = -1
        boundary[obsBound] = 1
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # plt.imshow(boundary,cmap='hot', interpolation='nearest')
        # plt.show()

        
        # Find the connected boundary components
        bb = (boundary.copy() != 0 )
        bb = np.array(bb, dtype=np.uint8)

        plt.imshow(bb,cmap='hot', interpolation='nearest')
        plt.show()
        
        contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        # cv2.drawContours(bb,contours,-1,(255,0,0),1) 
        # plt.imshow(bb)
        # plt.show() 
        # print(len(contours))
        # plt.imshow(im2,cmap='hot', interpolation='nearest')
        # plt.show()
        



        # TRANSFORM HIERARCHY TO MATLAB "A" MATRIX 
        tmpout = np.array(contours[0].copy()).reshape(contours[0].shape[0],contours[0].shape[2])
        # n = 1
        n = len(contours)
        A = np.zeros((n,n),dtype=np.uint8)
        
        for k in range(0,n):
            if (hierarchy[0,k,3] != (-1)) and (hierarchy[0,k,3]!=k):
                A[k,int(hierarchy[0,k,3])] = 1
        
        encl_by_outer = np.where(A[:,0])
        # plt.imshow(A,cmap='hot', interpolation='nearest')
        # plt.show()
        print("------")
        print(encl_by_outer)
        #tmp2 = A[:,encl_by_outer].copy()
        tmp2 = encl_by_outer 
        tmp2 = np.sum(tmp2,axis=0)
        print(tmp2)
        innerind = np.reshape(np.where(tmp2),np.size(np.where(tmp2)))

        
        # PREPARE DATA FOR BEM 
        tmpout = np.array(contours[0].copy()).reshape(contours[0].shape[0],contours[0].shape[2])
        xl = np.ascontiguousarray(np.flipud(tmpout[:,0]))
        yl = np.ascontiguousarray(np.flipud(tmpout[:,1]))
        nl = np.ascontiguousarray(np.array(xl.shape))[0]

    
        print("innerind: ", end='')
        print(innerind)
        if np.size(np.array(innerind))!=0:
            for i in range (0,np.size(np.array(innerind))):
                
                j = int(innerind[i].copy())
                
                tmpin = np.array(contours[j].copy().reshape(contours[j].shape[0],contours[j].shape[2]))
                xl = np.append(xl,tmpin[:,0])
                yl = np.append(yl,tmpin[:,1])
                nl = np.append(nl,np.size(xl))
    #             nl[i+1] = np.size(xl)
        
        nl = np.reshape(nl.copy(),np.size(nl))
        
        numel = np.size(xl)
        nb = np.size(innerind)+1
        bound = np.ones((int(numel),1))
        bound[-1] = 0
        
        if ( np.size(encl_by_outer) > 1 ):
            for i in range(0,int(numel)):
                bb[int(xl[i]),int(yl[i])] = 0

            # IMCLOSE AND IMOPEN OPERATIONS
            se =  np.ones((5,5), np.uint8)
            # se[0,0] = 0
            # se[-1,0] = 0
            # se[0,-1] = 0
            # se[-1,-1] = 0
            bb = cv2.morphologyEx(bb, cv2.MORPH_CLOSE, se)
            bb = cv2.morphologyEx(bb, cv2.MORPH_OPEN, se)

            # bwareaopen
            # im = cv2.blur(bb,(4,4))
            # im = cv2.threshold(bb, 175 , 250, cv2.THRESH_BINARY)      
            
            contours, hierarchy  = cv2.findContours(bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            # cv2.drawContours(bb, contours, -1, (0,255,0), 3)
            
            # i=0
            # cl = ['r','g', 'b']
            # for contour in contours:

            #     plt.plot(contour[:,0,0],contour[:,0,1], cl[i%3])
            #     i+=1
            
            # plt.show()
            if(np.all(hierarchy != None)):
                fatherless = np.where(hierarchy[0,:,3]==-1)
                fatherless = np.array(fatherless)
                fatherless = fatherless[0].copy()
            
                for i in range(1,np.size(fatherless)):
                    
                    j = int(fatherless[i])
                    tmpin = contours[j]
                    
                    tmpin = tmpin[:,0]
                    xl = np.append(xl,np.reshape(tmpin[0:-1,0].copy(),np.size(tmpin[0:-1,0])))
                    yl = np.append(yl,np.reshape(tmpin[0:-1,1].copy(),np.size(tmpin[0:-1,1])))
                    nb = nb + 1
                    numel = np.size(xl)
                    nl = np.append(nl,np.size(xl))

                    print('Faulty Boundary')


        boundary_out = boundary.copy()
        # plt.imshow(boundary_out,cmap='hot', interpolation='nearest')
        # plt.colorbar()
        # plt.show()
        bbcopy = np.array(255*np.ones((np.shape(map_data.copy())[0],np.shape(map_data.copy())[1],3)),dtype=np.uint8)
        bbcopy[freeBound,0] = 0
        bbcopy[freeBound,2] = 0
        bbcopy[obsBound,0] = 0
        bbcopy[obsBound,1] = 0
        bbcopy[int(position[1])-1:int(position[1])+1,int(position[0])-1:int(position[0])+1,:] = 0
        self.image_msg =  self.br.cv2_to_imgmsg(bbcopy,"bgr8")
        self.image_pub.publish(self.image_msg)
        
        
        return xl,yl,numel,nb,nl,boundary_out

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
#                               MAIN FUNCTION 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

if __name__=='__main__':

    rospy.init_node('boundary_comp_node', anonymous=True)
    listener = tf.TransformListener()

    computation = Computation()
   

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

       
        
        
        failed_comp = True
        computation.boundary_info_msg.comp_failed = True
        # CHECK IF MAP HAS BEEN RECEIVED
        if (not computation.map_data_tmp == None):
            # print("MAP RECEIVED")
            # IMPLEMENTATION OF THE occGridMapping_py FUNCTION
            map_output = np.transpose(np.reshape(computation.map_data_tmp,(computation.mapSize_py[1],computation.mapSize_py[0]),order='F'))
            map_data = map_output.copy()

            # plt.imshow(map_data,cmap='hot', interpolation='nearest')
            # plt.colorbar()
            # plt.show()
           
            map_data[map_output.copy() == -1] = 0
            map_data[ np.logical_and(map_output.copy() < 50,map_output.copy() != -1)] = computation.lo_min
            map_data[map_output.copy() >= 50] =computation.lo_max
            map_data_uint8 = np.uint8(map_data.copy())
            
            #plt.imshow(map_data,cmap='hot', interpolation='nearest')
            #plt.colorbar()
            #plt.show()

        
            # INFLATE THE BOUNDARY
            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            kernel = np.ones((1,1), np.uint8) #kernel = np.ones((3,3), np.uint8)
            map_uint8_dilate = cv2.dilate(map_data_uint8, kernel, iterations=1)
            map_dilate = map_data.copy()
            map_dilate[map_uint8_dilate == np.max(map_uint8_dilate)] = computation.lo_max

            
            # plt.imshow(map_dilate,cmap='hot', interpolation='nearest')
            # plt.colorbar()
            # plt.show()

            # GET ROBOT POSITION
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                # trans = [computation.tf_drone.pose.position.x,computation.tf_drone.pose.position.y,computation.tf_drone.pose.position.z]
                computation.position= np.double([trans[0]/computation.mapResolution,trans[1]/computation.mapResolution]) + np.double(computation.mapOrigin)/computation.mapResolution 
                
                computation.position = computation.position.astype(int)
                haspos = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                failed_comp = True
                continue

            # TRY COMPUTATION
            # !!!!!! DEL THIS!!!!!!!! 
            #xl_py, yl_py, numel_py,nb_py,nl_py,boundary_py = computation.boundaryExtraction2( map_dilate, computation.position)
            if haspos:
                try:
                    # xl_py, yl_py, numel_py,nb_py,nl_py,boundary_py = computation.boundaryExtraction2( map_dilate, computation.position)
                    xl_py, yl_py, numel_py,nb_py,nl_py,boundary_py = computation.boundaryExtraction2( map_dilate, computation.position)    
                    failed_comp = False
                except:
                    failed_comp = True
            # xl_py, yl_py, numel_py,nb_py,nl_py,boundary_py = computation.boundaryExtraction2( map_dilate, computation.position)

            # plt.plot(xl_py,yl_py,'r-')
            # plt.show()
            # try:
            #     i=0
            #     cl = ['r','g', 'b']
            #     nl_py = np.insert(nl_py,0,0)
            #     for i in range(nb_py):

            #         plt.plot(xl_py[nl_py[i]:nl_py[i+1]],yl_py[nl_py[i]:nl_py[i+1]], cl[i%3])
            #         i+=1
                
            #     plt.show()
            # except:
            #     print('error')

            # PARTITION THE BOUNDARY TO FREE AND OCC
            if not failed_comp:
                is_bou_free = np.full(np.shape(xl_py),False)
                is_bou_free[boundary_py[yl_py,xl_py] < 0] = True
                computation.boundary_info_msg.xl = xl_py
                computation.boundary_info_msg.yl = yl_py
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
