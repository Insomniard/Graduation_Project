#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import String
#This file is user control lib, copy to coding workspace.
class cardrv_control_example:
    def __init__(self, nodeName="gilbot_client", demo_mode=False):
        self.nodeName = nodeName
        
        self.state_view = True
        self.view_cycle = 0
        ###### class useget driver control (Powered by IOELECTRON)
        # -> [class = ros_car_control(NodeName, publisherName)] class init
        # -> [class.Cset_ref]               data set (control data variable set, 20Hz loop)
        # -> [class.Cset_active()]          call method (data set refresh timer refresh, 2 second limit watch dog.)
        # -> [class.Cget_ref]               data read right (car driver sensor data read.)
        
        # Example 
        # -> class.Cset_aimode = 2          Car pair to PC mode
        # -> class.Cset_drvmode = 1         drive forwarding
        # -> class.Cset_throttle = 20       forwarding throttle 20
        # -> class.Cset_steer_degree = 90   steer 90 degree for center
        # -> class.Cset_active()            active run

        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
        self.Cset_drvmode = 0 # 0:Normal, 1:Forward, 2:Reverse, 3:break
        self.Cset_throttle = 0 # 0~99 throttle level
        self.Cset_steer_degree = 90 # 0~180 steer degree mode set (180 upper is off)
        self.Cset_steer_raw = 9999 # 0~1023 steer law level mode set (1023 upper is off)
        self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
        self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
        self.Cset_horn = 0 # 0:horn off, 1:horn on
        self.Cset_custom_a = 0 #custom module data A transmit 0~9
        self.Cset_custom_b = 0 #custom module data A transmit 0~9

        self.Cget_connection = 0 #0:car connection error, 1:connected
        self.Cget_steer_degree = 250 #get last servo degree
        self.Cget_steer_rawlevel = 9999 #get steer raw level
        self.Cget_rotor_hall = 0 #get left+right hall data (1 hall = xxx mm)
        self.Cget_steer_limit_l = 111 #get steer left limit raw level
        self.Cget_steer_limit_c = 555 #get steer center raw level
        self.Cget_steer_limit_r = 999 #get steer right limit raw level
        self.Cget_voltage = 18.00 #get car power voltage (LiFePO4)
        self.Cget_rotor_hall_l = 0 #get left bldc rotor hall
        self.Cget_rotor_hall_r = 0 #get right bldc rotor hall
        self.Cget_current_char = 0 #get charging current
        self.Cget_current_dischar = 0 #get discharging current
        self.Cget_drv_temperature = 0 #get driver breaking res temperature (normal is air temperature.)
        self.Cget_drvmode = 0 #get car driving mode (1: forward, 2:reverse)
        self.Cget_lamp = 0 #get lamp state
        self.Cget_signal = 0 #get signal state (0:OFF, 1:left, 2:right, 3:both)
        self.Cget_brk = 0 #get solid break error state check
        self.Cget_report_a = 0 #get custom module data A
        self.Cget_report_b = 0 #get custom module data B
        
        #ROS init start
        self.classed_mode = False
        try: rospy.init_node(self.nodeName, anonymous=True)
        except:
            print("Error node init car client node import mode detected.")
            self.classed_mode = True
        
        self.pub_aimode =           rospy.Publisher("/command/aimode", String, queue_size=1)
        self.pub_drive_vector =     rospy.Publisher("/command/drive_vector", String, queue_size=1)
        self.pub_drive_throttle =   rospy.Publisher("/command/drive_throttle", String, queue_size=1)
        self.pub_steer_degree =     rospy.Publisher("/command/steer_degree", String, queue_size=1)
        self.pub_steer_raw =        rospy.Publisher("/command/steer_raw", String, queue_size=1)
        self.pub_signal =           rospy.Publisher("/command/signal", String, queue_size=1)
        self.pub_lamp =             rospy.Publisher("/command/lamp", String, queue_size=1)
        self.pub_horn =             rospy.Publisher("/command/horn", String, queue_size=1)
        self.pub_embed_custom_a =   rospy.Publisher("/command/embed_custom_a", String, queue_size=1)
        self.pub_embed_custom_b =   rospy.Publisher("/command/embed_custom_b", String, queue_size=1)
        
        self.sub_car_connected =        rospy.Subscriber("/value/car_connected", String, self.sub_callback_car_connected)
        self.sub_steer_degree =         rospy.Subscriber("/value/steer_degree", String, self.sub_callback_steer_degree)
        self.sub_steer_raw =            rospy.Subscriber("/value/steer_raw", String, self.sub_callback_steer_raw)
        self.sub_steer_limit_left =     rospy.Subscriber("/value/steer_limit_left", String, self.sub_callback_steer_limit_left)
        self.sub_steer_limit_center =   rospy.Subscriber("/value/steer_limit_center", String, self.sub_callback_steer_limit_center)
        self.sub_steer_limit_right =    rospy.Subscriber("/value/steer_limit_right", String, self.sub_callback_steer_limit_right)
        self.sub_rotor_hall =           rospy.Subscriber("/value/rotor_hall", String, self.sub_callback_rotor_hall)
        self.sub_rotor_hall_l =         rospy.Subscriber("/value/rotor_hall_l", String, self.sub_callback_rotor_hall_l)
        self.sub_rotor_hall_r =         rospy.Subscriber("/value/rotor_hall_r", String, self.sub_callback_rotor_hall_r)
        self.sub_voltage =              rospy.Subscriber("/value/voltage", String, self.sub_callback_voltage)
        self.sub_current_charging =     rospy.Subscriber("/value/current_charging", String, self.sub_callback_current_charging)
        self.sub_current_discharging =  rospy.Subscriber("/value/current_discharging", String, self.sub_callback_current_discharging)
        self.sub_drv_temperature =      rospy.Subscriber("/value/drv_temperature", String, self.sub_callback_drv_temperature)
        self.sub_car_vector =           rospy.Subscriber("/value/car_vector", String, self.sub_callback_car_vector)
        self.sub_car_lamp =             rospy.Subscriber("/value/car_lamp", String, self.sub_callback_car_lamp)
        self.sub_car_signal =           rospy.Subscriber("/value/car_signal", String, self.sub_callback_car_signal)
        self.sub_car_solidbrk =         rospy.Subscriber("/value/car_solidbrk", String, self.sub_callback_car_solidbrk)
        self.sub_embed_custom_a =       rospy.Subscriber("/value/embed_custom_a", String, self.sub_callback_embed_custom_a)
        self.sub_embed_custom_b =       rospy.Subscriber("/value/embed_custom_b", String, self.sub_callback_embed_custom_b)

        rospy.loginfo("Start Car driver / Node: %s"%(self.nodeName))
        self.rate = rospy.Rate(10)

        if not demo_mode:
            if not self.classed_mode:
                while not rospy.is_shutdown(): #for this coding
                    #get user code here.
                    self.rate.sleep()
        else:
            ######################### demo code ###########################
            test_cycle = 10
            while not rospy.is_shutdown():
                for a in range(test_cycle):
                    self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                    self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                    self.Cset_throttle = 30 # 0~99 throttle level
                    self.Cset_steer_degree = 90 # 0~180 steer degree mode set (180 upper is off)
                    self.Cset_steer_raw = 9999 # 0~1023 steer law level mode set (1023 upper is off)
                    self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
                    self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
                    self.Cset_horn = 0 # 0:horn off, 1:horn on
                    self.Cset_custom_a = 0 #custom module data A transmit 0~9
                    self.Cset_custom_b = 0 #custom module data B transmit 0~9
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                    self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                    self.Cset_throttle = 20 # 0~99 throttle level
                    self.Cset_steer_degree = 0 # 0~180 steer degree mode set (180 upper is off)
                    self.Cset_steer_raw = 9999 # 0~1023 steer law level mode set (1023 upper is off)
                    self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
                    self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
                    self.Cset_horn = 0 # 0:horn off, 1:horn on
                    self.Cset_custom_a = 0 #custom module data A transmit 0~9
                    self.Cset_custom_b = 0 #custom module data B transmit 0~9
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                    self.Cset_drvmode = 2 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                    self.Cset_throttle = 30 # 0~99 throttle level
                    self.Cset_steer_degree = 90 # 0~180 steer degree mode set (180 upper is off)
                    self.Cset_steer_raw = 9999 # 0~1023 steer law level mode set (1023 upper is off)
                    self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
                    self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
                    self.Cset_horn = 0 # 0:horn off, 1:horn on
                    self.Cset_custom_a = 0 #custom module data A transmit 0~9
                    self.Cset_custom_b = 0 #custom module data B transmit 0~9
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                    self.Cset_drvmode = 2 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                    self.Cset_throttle = 30 # 0~99 throttle level
                    self.Cset_steer_degree = 180 # 0~180 steer degree mode set (180 upper is off)
                    self.Cset_steer_raw = 9999 # 0~1023 steer law level mode set (1023 upper is off)
                    self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
                    self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
                    self.Cset_horn = 0 # 0:horn off, 1:horn on
                    self.Cset_custom_a = 0 #custom module data A transmit 0~9
                    self.Cset_custom_b = 0 #custom module data B transmit 0~9
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                    self.Cset_drvmode = 2 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                    self.Cset_throttle = 30 # 0~99 throttle level
                    self.Cset_steer_degree = 90 # 0~180 steer degree mode set (180 upper is off)
                    self.Cset_steer_raw = 9999 # 0~1023 steer law level mode set (1023 upper is off)
                    self.Cset_signal = 3 # 0:signal OFF, 1:left signal lamp on, 2:right signal lamp on, 3:both signal lamp on
                    self.Cset_lamp = 0 # 0:lamp off, 1:lamp on
                    self.Cset_horn = 0 # 0:horn off, 1:horn on
                    self.Cset_custom_a = 0 #custom module data A transmit 0~9
                    self.Cset_custom_b = 0 #custom module data B transmit 0~9
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()
                ######################### demo code ###########################
        
        if not self.classed_mode: print("Terminate. car driving control.")  
    
    def sub_callback_car_connected(self, msg): self.Cget_connection = int(msg.data)
    def sub_callback_steer_degree(self, msg): self.Cget_steer_degree = int(msg.data)
    def sub_callback_steer_raw(self, msg): self.Cget_steer_rawlevel = int(msg.data)
    def sub_callback_steer_limit_left(self, msg): self.Cget_steer_limit_l = int(msg.data)
    def sub_callback_steer_limit_center(self, msg): self.Cget_steer_limit_c = int(msg.data)
    def sub_callback_steer_limit_right(self, msg): self.Cget_steer_limit_r = int(msg.data)
    def sub_callback_rotor_hall(self, msg): self.Cget_rotor_hall = int(msg.data)
    def sub_callback_rotor_hall_l(self, msg): self.Cget_rotor_hall_l = int(msg.data)
    def sub_callback_rotor_hall_r(self, msg): self.Cget_rotor_hall_r = int(msg.data)
    def sub_callback_rotor_hall_r(self, msg): self.Cget_rotor_hall_r = int(msg.data)
    def sub_callback_voltage(self, msg): self.Cget_voltage = float(msg.data)
    def sub_callback_current_charging(self, msg): self.Cget_current_char = int(msg.data)
    def sub_callback_current_discharging(self, msg): self.Cget_current_dischar = int(msg.data)
    def sub_callback_drv_temperature(self, msg): self.Cget_drv_temperature = int(msg.data)
    def sub_callback_car_vector(self, msg): self.Cget_drvmode = int(msg.data)
    def sub_callback_car_lamp(self, msg): self.Cget_lamp = int(msg.data)
    def sub_callback_car_signal(self, msg): self.Cget_signal = int(msg.data)
    def sub_callback_car_solidbrk(self, msg): self.Cget_brk = int(msg.data)
    def sub_callback_embed_custom_a(self, msg): self.Cget_report_a = int(msg.data)
    def sub_callback_embed_custom_b(self, msg): self.Cget_report_b = int(msg.data)
    
    def get_state_view(self):
        if self.state_view:
            print("---------------client side--------------")
            print("DRVCONN(%d) STEERE_DEGREE(%03d)(%04d, %04d/%04d/%04d)"%(self.Cget_connection, self.Cget_steer_degree, self.Cget_steer_rawlevel, self.Cget_steer_limit_l, self.Cget_steer_limit_c, self.Cget_steer_limit_r))
            print("ROTOR_HA(%04d)(L:%04d R:%04d) VECMODE(%d) SBRK(%d)"%(self.Cget_rotor_hall, self.Cget_rotor_hall_l, self.Cget_rotor_hall_r, self.Cget_drvmode, self.Cget_brk))
            print("VOLTAGE(%.2f) CHARGING(%dA) DISCHARGING(%dA) TEMP(%d)"%(self.Cget_voltage, self.Cget_current_char, self.Cget_current_dischar, self.Cget_drv_temperature))
            print("LAMP(%d) SIGNAL(%d) CUSTOM_A(%d) CUSTOM_B(%d)"%(self.Cget_lamp, self.Cget_signal, self.Cget_report_a, self.Cget_report_b))

    def Cset_active(self):
        self.pub_aimode.publish(str(self.Cset_aimode))
        self.pub_drive_vector.publish(str(self.Cset_drvmode))
        self.pub_drive_throttle.publish(str(self.Cset_throttle))
        self.pub_steer_degree.publish(str(self.Cset_steer_degree))
        self.pub_steer_raw.publish(str(self.Cset_steer_raw))
        self.pub_signal.publish(str(self.Cset_signal))
        self.pub_lamp.publish(str(self.Cset_lamp))
        self.pub_horn.publish(str(self.Cset_horn))
        self.pub_embed_custom_a.publish(str(self.Cset_custom_a))
        self.pub_embed_custom_b.publish(str(self.Cset_custom_b))

if __name__=='__main__':
    car_control = None
    try:
        car_control = cardrv_control_example("gilbot_client", demo_mode=True)
    except rospy.ROSInterruptException:
        pass
    car_control.demo_thread_run = False