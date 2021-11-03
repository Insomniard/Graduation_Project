#!/usr/bin/env python3
from logging import fatal
from numpy.core.fromnumeric import swapaxes
import rospy, cv2, sys
import numpy as np
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from cardrv_use_example_jeong import cardrv_control_example
from TargetCahch import targetcatch
from std_msgs.msg import Int8 , Bool

class my_script_run:
    
    def __init__(self):
        self.steer_degree_plma = 50
        rospy.init_node('letsgetit', anonymous=True)
        self.carDrv         = cardrv_control_example("gilbot_client", False)
        self.img_bgr_0 = None
        rospy.Subscriber('ridar_ctr_left',Bool,self.callback_lidar_left)
        rospy.Subscriber('ridar_ctr_right',Bool,self.callback_lidar_right)
        rospy.Subscriber('ridar_data_check',Bool,self.callback_lidar_stop)

        rospy.Subscriber('/handler_point',Int8,self.cb)
        rospy.Subscriber('/throtle_val',Int8,self.throttle_callback)
        self.pub_steer = rospy.Publisher('/value/steer_degree', String, queue_size=3)
        self.left_point = Bool()
        self.right_point = Bool()
        self.stop_point = Bool()
        self.hander_point = Int8()
        self.throttle_value = Int8()
        self.is_cb = False
        
        self.rate = rospy.Rate(20)
        self.Cset_aimode = 2
        while not rospy.is_shutdown():
            if self.left_point.data == False and self.right_point.data == False and self.stop_point.data == False:
                self.is_cb = True
            else : self.is_cb = False

            if self.is_cb:
                self.carDrv.Cset_drvmode = 1
                self.carDrv.Cset_throttle = self.throttle_value
                if self.hander_point.data ==1:
                    self.carDrv.Cset_steer_degree = 145
                    self.Cset_steer_raw=9999
                elif self.hander_point.data == 2:
                    self.carDrv.Cset_steer_degree =  1
                
                elif self.hander_point.data ==0:
                    self.carDrv.Cset_steer_degree=65
                else:
                    self.is_cb = False
                    self.carDrv.Cset_throttle = 0
                    self.carDrv.Cset_drvmode=3
                self.pub_steer.publish(str(self.carDrv.Cset_steer_degree))
                self.carDrv.Cset_active()

            else:

                if self.left_point.data == True:

                    self.carDrv.Cset_drvmode = 1
                    self.carDrv.Cset_throttle = self.throttle_value
                    self.carDrv.Cset_steer_degree =  1
                elif self.right_point.data == True:

                    self.carDrv.Cset_drvmode = 1
                    self.carDrv.Cset_throttle = self.throttle_value
                    self.carDrv.Cset_steer_degree = 145
                elif self.left_point.data == True and self.right_point.data == True and self.stop_point.data == True:
                    self.is_cb = False
                    self.carDrv.Cset_throttle = self.throttle_value
                    self.carDrv.Cset_drvmode=3
                else : self.carDrv.Cset_steer_degree= 65 # 75degree
            
            if self.is_cb == False and self.stop_point.data == True:
                    self.carDrv.Cset_throttle = 0
                    self.carDrv.Cset_drvmode=3

            self.carDrv.Cset_active()
            self.rate.sleep()

                   
    def throttle_callback(self, data):
        self.throttle_value = data.data
        print(self.throttle_value)
    def cb(self, data):
        self.hander_point.data = data.data
    def callback_lidar_left(self,data):
        self.left_point.data = data.data
    def callback_lidar_right(self,data):
        self.right_point.data = data.data
    def callback_lidar_stop(self,data):
        self.stop_point.data = data.data


    def callback_cam0(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.img_bgr_0 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow('IOELECTRON subscriber image test', self.img_bgr_0)
        cv2.waitKey(1)
    
    def callback_utm(self, msg):

        print("STAMP: %s"%(str(msg.header.stamp)))
        print("LATI: %s"%(str(msg.latitude)))
        print("LONG: %s"%(str(msg.longitude)))

    def callback_time(self, msg):
        print("TIME: %s"%(str(msg.data)))
    
    def callback_speed(self, msg):
        print("SPD: %skm/h"%(str(msg.data)))

    def callback_laser(self, msg):
        # print("LASER: %s"%(str(msg.data)))
        pass

if __name__=='__main__':
    control_main = None
    try:
        control_main = my_script_run()
    except rospy.ROSInterruptException:
        control_main.carDrv.demo_thread_run = False
        pass
