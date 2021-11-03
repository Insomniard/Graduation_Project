#!/usr/bin/env python3

# from _typeshed import Self
from logging import fatal
import rospy
import time, threading
from rplidar import RPLidar
from std_msgs.msg import Float64, Int8, Bool

class ioe_lidar:
    def __init__(self, comport):
        rospy.init_node('lidar_scan', anonymous=True)
        self.comm = comport
        self.lidar = RPLidar("/dev/" + self.comm)
        self.info = self.lidar.get_info()
        self.health = self.lidar.get_health()

        print(str(self.info) + str(self.health))

        self.degree_buf = [0.0 for i in range(361)]

        self.thread_run = True
        self.degree_cal_thread = threading.Timer(1, self.degree_threading)
        self.degree_cal_thread.start()

    def degree_threading(self):
        print("Lidar receive thread start. PORT:" + self.comm)
        for i, scan in enumerate(self.lidar.iter_scans(max_buf_meas=1000)):
            try:
                for scan_value in scan: 
                    self.degree_buf[round(scan_value[1])] = scan_value[2] # 1 angle , 22 distance
            except:
                print("Error occurred.")
                self.thread_run = False

            if not self.thread_run:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                break
        print("Lidar receive threading terminated.")

    def lenth_ret(self, target_degree=0):
        if target_degree < 0 and target_degree > 360: target_degree = 0
        return self.degree_buf[target_degree]


if __name__ == '__main__':
    
    sen_Lidar = ioe_lidar("ttyUSB0")
    temp_list = []
    temp_list_right = []
    start_degree_left = 331
    end_degree_left = 360
    start_degree_right = 31
    end_degree_right = 60
    left_distance = 0.0
    right_distance = 0.0
    front_start_left = 300
    front_end_left = 330
    front_start_right = 1
    front_end_right = 30
    ctr_ridar_left = rospy.Publisher('ridar_ctr_left',Bool, queue_size=1)
    pub_ctr_pub_left = Bool()
    ctr_ridar_right = rospy.Publisher('ridar_ctr_right',Bool, queue_size=1)
    pub_ctr_pub_right = Bool()
    ctr_move = rospy.Publisher('ridar_data_check',Bool, queue_size=1)
    pub_ctr_move = Bool()
    ctr_throttle = rospy.Publisher('throtle_val', Int8, queue_size=2)
    pub_ctr_throtle = Int8()
    while rospy.is_shutdown:
        try:
            left_min = 10000.0
            right_min = 10000.0
            front_min = 10000.0
            left_max = 0.0
            right_max = 0.0
            front_max = 0.0
            degree_list = []
            for i in range (start_degree_left, end_degree_left): # Left Catch
                if (sen_Lidar.degree_buf[i] > 100.0 and left_min > sen_Lidar.degree_buf[i]):
                    left_min = sen_Lidar.degree_buf[i]
                    sen_Lidar.degree_buf[i] = 10000.0
                if(left_max < sen_Lidar.degree_buf[i]): left_max = sen_Lidar.degree_buf[i]

            for i in range (start_degree_right, end_degree_right): # Right Catch
                if (sen_Lidar.degree_buf[i] > 100.0 and right_min > sen_Lidar.degree_buf[i]):
                    right_min = sen_Lidar.degree_buf[i]
                    sen_Lidar.degree_buf[i] = 10000.0
                if(right_max < sen_Lidar.degree_buf[i]): right_max = sen_Lidar.degree_buf[i]

            for i in range (front_start_left, front_end_left): # Right Catch
                if (sen_Lidar.degree_buf[i] > 100.0 and front_min > sen_Lidar.degree_buf[i]):
                    front_min = sen_Lidar.degree_buf[i]
                    sen_Lidar.degree_buf[i] = 10000.0
                if(front_max < sen_Lidar.degree_buf[i]): front_max = sen_Lidar.degree_buf[i]
                
            for i in range (front_start_right, front_end_right): # Right Catch
                if (sen_Lidar.degree_buf[i] > 100.0 and front_min > sen_Lidar.degree_buf[i]):
                    front_min = sen_Lidar.degree_buf[i]
                    sen_Lidar.degree_buf[i] = 10000.0
                if(front_max < sen_Lidar.degree_buf[i]): front_max = sen_Lidar.degree_buf[i]


            if (left_min < 1500):
                pub_ctr_pub_left.data = True
                # pub_ctr_move = True
            else : pub_ctr_pub_left.data = False
            if (right_min < 1500):
                pub_ctr_pub_right.data = True
                # pub_ctr_move = True
            else : pub_ctr_pub_right.data = False
            if (pub_ctr_pub_left.data == True and pub_ctr_pub_right.data == True):
                pub_ctr_move.data = True
            else : pub_ctr_move.data = False
            
            if (front_min < 1000.0):
                throtle = 0.0
                isbreak = True
            elif (front_min < 2000.0):
                throtle = 10.0
            elif (front_min < 3000.0):
                throtle = 20.0
            elif (front_min < 4000.0):
                throtle = 30.0
            elif (front_min < 5000.0):
                throtle = 40.0
            else:
                throtle = 50.0
            
            pub_ctr_throtle.data = int(throtle)
            ctr_throttle.publish(pub_ctr_throtle)
            ctr_ridar_left.publish(pub_ctr_pub_left)
            ctr_ridar_right.publish(pub_ctr_pub_right)
            ctr_move.publish(pub_ctr_move)

            time.sleep(0.2)
        except KeyboardInterrupt:
            sen_Lidar.thread_run = False
            break
        except Exception as e:
            print("Error occurred. Exiting Program: " + str(e))
            sen_Lidar.thread_run = False
            break
    time.sleep(1)
    print("Terminate Lidar Process.")

