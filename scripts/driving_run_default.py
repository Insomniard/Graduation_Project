#!/usr/bin/env python3
from logging import fatal
from numpy.core.fromnumeric import swapaxes
import rospy, cv2, sys
import numpy as np
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from cardrv_use_example import cardrv_control_example
from TargetCatch import targetcatch
from std_msgs.msg import Int8 , Bool



#  self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
#         self.Cset_drvmode = 0 # 0:Normal, 1:Forward, 2:Reverse, 3:break
#         self.Cset_throttle = 0 # 0~99 throttle level
#         self.Cset_steer_degree = 90 # 0~180 steer degree mode set (180 upper is off)
#         self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
#         self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
#         self.Cset_horn = 0 # 0:horn off, 1:horn on

class my_script_run:
    def __init__(self):
        self.steer_degree_plma = 50
        rospy.init_node('control_programer', anonymous=True)
        self.carDrv         = cardrv_control_example("gilbot_control_cli") #driver code confusion.. import for simple coding.
       # self.cam_sub_0      = rospy.Subscriber("/camera0/compressed", CompressedImage, self.callback_cam0) #read for cam0 image.
        # self.gps_sub_utm    = rospy.Subscriber("/gps_node/get_UTM", NavSatFix, self.callback_utm)
        # self.gps_sub_time   = rospy.Subscriber("/gps_node/get_time", String, self.callback_time)
        # self.gps_sub_speed  = rospy.Subscriber("/gps_node/get_speed", String, self.callback_speed)
        # self.lidar_sub      = rospy.Subscriber("/scan", LaserScan, self.callback_laser)
        self.img_bgr_0 = None
        rospy.Subscriber('ridar_ctr_left',Bool,self.callback_lidar_left)
        rospy.Subscriber('ridar_ctr_right',Bool,self.callback_lidar_right)
        rospy.Subscriber('ridar_data_check',Bool,self.callback_lidar_stop)

        rospy.Subscriber('/handler_point',Int8,self.cb)
        self.left_point = Bool()
        self.right_point = Bool()
        self.stop_point = Bool()
        self.hander_point = Int8()
        self.is_cb = False

        
        self.rate = rospy.Rate(20) #car drv control rate hz (0.05s)
        self.Cset_aimode = 2
        ## 1 : 왼쪽헨들 2: 우짝헨들 3(A):빵빵   4(B): 우짝깜빡이 5(X): 좌짝깜빡이 6(Y): 램프 7(LT): 속도2배
        while not rospy.is_shutdown():
            # print(self.left_point.data,self.right_point.data,self.stop_point.data)
            if self.left_point.data == False and self.right_point.data == False and self.stop_point.data == False:
                self.is_cb = True
            else : self.is_cb = False
            
            if self.is_cb:
                self.carDrv.Cset_drvmode = 1
                self.carDrv.Cset_throttle = 10
                print("Tq",self.hander_point.data)
                if self.hander_point.data ==1:
                    self.carDrv.Cset_steer_degree = 145
                    # self.Cset_steer_raw=9999
                elif self.hander_point.data == 2:
                    self.carDrv.Cset_steer_degree =  1
                    # self.Cset_steer_raw=9999
                
                elif self.hander_point.data ==0:
                    self.carDrv.Cset_steer_degree=65
                else:
                    self.is_cb = False
                    self.carDrv.Cset_throttle = 0
                    self.carDrv.Cset_drvmode=3
                print(self.carDrv.Cset_steer_degree)

            else:

                if self.left_point.data == True:

                    self.carDrv.Cset_drvmode = 1
                    self.carDrv.Cset_throttle = 10
                    self.carDrv.Cset_steer_degree =  1
                elif self.right_point.data == True:

                    self.carDrv.Cset_drvmode = 1
                    self.carDrv.Cset_throttle = 10
                    self.carDrv.Cset_steer_degree = 145
                elif self.left_point.data == True and self.right_point.data == True and self.stop_point.data == True:
                    self.is_cb = False
                    self.carDrv.Cset_throttle = 0
                    self.carDrv.Cset_drvmode=3
                else : self.carDrv.Cset_steer_degree= 65 # 75degree
            
            if self.is_cb == False and self.stop_point.data == True:
                    self.carDrv.Cset_throttle = 0
                    self.carDrv.Cset_drvmode=3

                
            #print("handle h : ",self.hander_point.data)
                                # if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>1000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<5000:
                #     self.carDrv.Cset_steer_degree = 105
                #     self.Cset_steer_raw = 9999
                # if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>5000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<10000:
                #     self.carDrv.Cset_steer_degree = 120
                #     self.Cset_steer_raw = 9999
                # if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>10000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<15000:
                #     self.carDrv.Cset_steer_degree = 135
                #     self.Cset_steer_raw = 9999
                # if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>15000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<20000:
                #     self.carDrv.Cset_steer_degree = 150
                #     self.Cset_steer_raw = 9999
                # if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>20000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<25000:
                #     self.carDrv.Cset_steer_degree = 175
                #     self.Cset_steer_raw = 9999
                # if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>25000:
                #     self.carDrv.Cset_steer_degree = 180
                #     self.Cset_steer_raw = 9999


            # if self.joystick.getKey_RIGHT_STICK_LEFTRIGHT>0:
            #     if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>1000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<5000:
            #         self.carDrv.Cset_steer_degree = 75
            #         self.Cset_steer_raw = 9999
            #     if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>5000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<10000:
            #         self.carDrv.Cset_steer_degree = 60
            #         self.Cset_steer_raw = 9999
            #     if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>10000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<15000:
            #         self.carDrv.Cset_steer_degree = 45
            #         self.Cset_steer_raw = 9999
            #     if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>15000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<20000:
            #         self.carDrv.Cset_steer_degree = 30
            #         self.Cset_steer_raw = 9999
            #     if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>20000 and abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT/180))<25000:
            #         self.carDrv.Cset_steer_degree = 15
            #         self.Cset_steer_raw = 9999
            #     if abs(int(self.joystick.getKey_RIGHT_STICK_LEFTRIGHT))>25000:
            #         self.carDrv.Cset_steer_degree = 0
            #         self.Cset_steer_raw = 9999
            
            self.carDrv.Cset_active()
            self.rate.sleep()
    
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