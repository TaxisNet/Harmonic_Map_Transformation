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
        self.map_data_tmp = None
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.map_msg_callback)
        self.boundary_info_pub = rospy.Publisher('/boundary_info', boundary_info, queue_size=1)
        

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
    
    def boundaryExtraction(self):
        # map_data = np.array(self.map_data_tmp)
        map_data = np.reshape(self.map_data_tmp, self.mapSize_py)
        #now unoccupied is 0, occupied is 1 and unknown is -1.
        map_data[map_data>0] = 1
        map_data[map_data<0] = -1
        
        bound = np.array(map_data>0, dtype=np.uint8)
        kernel = np.ones((2,2), np.uint8)
        
        img_closed = cv2.morphologyEx(bound, cv2.MORPH_CLOSE, kernel)
        map_data[img_closed>0] = 1  

        
        bbcopy = np.array(255*np.ones((np.shape(map_data.copy())[0],np.shape(map_data.copy())[1],3)),dtype=np.uint8)
        bbcopy[map_data==0,0] = 0
        bbcopy[map_data==0,2] = 0
        bbcopy[map_data==1,0] = 0
        bbcopy[map_data==1,1] = 0
       

if __name__=='__main__':

    rospy.init_node('boundary_comp_node', anonymous=True)
    listener = tf.TransformListener()

    computation = Computation()
   
    
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if(computation.map_data_tmp):
            computation.boundaryExtraction()
        rate.sleep()


    rospy.spin()