#!/usr/bin/env python3
import rospy, time
from cardriver.msg import control_rx, control_tx ###########################################################

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
        
        self.pub_ctrl_msg = control_tx()###########################################################
        self.pub_ctrl =             rospy.Publisher("/command/", control_tx, queue_size=1)###########################################################
        self.sub_values =           rospy.Subscriber("/value/", control_rx, self.sub_callback_car_status)###########################################################
        
        self.pub_ctrl_msg.steer_raw = 9999###########################################################
        self.pub_ctrl_msg.signal = 0###########################################################
        self.pub_ctrl_msg.lamp = 0###########################################################
        self.pub_ctrl_msg.horn = 0###########################################################
        self.pub_ctrl_msg.embed_custom_a = 0###########################################################
        self.pub_ctrl_msg.embed_custom_b = 0###########################################################
        
        rospy.loginfo("Start Car driver / Node: %s"%(self.nodeName))
        self.rate = rospy.Rate(10)

        if not self.classed_mode:
            test_cycle = 0
            while not rospy.is_shutdown(): #for this coding
                try:
                    test_cycle += 1
                    if test_cycle > 60:
                        test_cycle = 0
                    elif test_cycle > 50:
                        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                        self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                        self.Cset_throttle = 80 # 0~99 throttle level
                        self.Cset_steer_degree = 180
                    elif test_cycle > 40:
                        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                        self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                        self.Cset_throttle = 80 # 0~99 throttle level
                        self.Cset_steer_degree = 180
                    elif test_cycle > 30:
                        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                        self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                        self.Cset_throttle = 80 # 0~99 throttle level
                        self.Cset_steer_degree = 0
                    elif test_cycle > 20:
                        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                        self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                        self.Cset_throttle = 80 # 0~99 throttle level
                        self.Cset_steer_degree = 0
                    elif test_cycle > 10:
                        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                        self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                        self.Cset_throttle = 20 # 0~99 throttle level
                        self.Cset_steer_degree = 90
                    else:
                        self.Cset_aimode = 2 # 0:OFF, 1:JustSteer, 2:CarPairToPCReady, 3:ForceControl(Warnning)
                        self.Cset_drvmode = 1 # 0:Normal, 1:Forward, 2:Reverse, 3:break
                        self.Cset_throttle = 20 # 0~99 throttle level
                        self.Cset_steer_degree = 90


                    self.pub_ctrl_msg.aimode = self.Cset_aimode###########################################################
                    self.pub_ctrl_msg.drive_vector = self.Cset_drvmode###########################################################
                    self.pub_ctrl_msg.drive_throttle = self.Cset_throttle###########################################################
                    self.pub_ctrl_msg.steer_degree = self.Cset_steer_degree###########################################################
                    self.pub_ctrl_msg.steer_raw = self.Cset_steer_raw
                    self.pub_ctrl_msg.signal = self.Cset_signal
                    self.pub_ctrl_msg.lamp = self.Cset_lamp
                    self.pub_ctrl_msg.horn = self.Cset_horn
                    self.pub_ctrl_msg.embed_custom_a = self.Cset_custom_a
                    self.pub_ctrl_msg.embed_custom_b = self.Cset_custom_b

                    self.pub_ctrl.publish(self.pub_ctrl_msg)###########################################################
                    self.rate.sleep()
                except KeyboardInterrupt: break
                except: break

                
        if not self.classed_mode: print("Terminate. car driving control.")  
    
    def sub_callback_car_status(self, msg):
        self.Cget_connection = msg.car_connected
        self.Cget_steer_degree = msg.steer_degree 
        self.Cget_steer_rawlevel = msg.steer_raw 
        self.Cget_steer_limit_l = msg.steer_limit_left 
        self.Cget_steer_limit_c = msg.steer_limit_center
        self.Cget_steer_limit_r = msg.steer_limit_right 
        self.Cget_rotor_hall = msg.rotor_hall 
        self.Cget_rotor_hall_l = msg.rotor_hall_l
        self.Cget_rotor_hall_r = msg.rotor_hall_r
        self.Cget_voltage = msg.voltage
        self.Cget_current_char = msg.current_charging
        self.Cget_current_dischar = msg.current_discharging
        self.Cget_drv_temperature = msg.drv_temperature
        self.Cget_drvmode = msg.car_vector
        self.Cget_lamp = msg.car_lamp
        self.Cget_signal = msg.car_signal
        self.Cget_brk = msg.car_solidbrk
        self.Cget_report_a = msg.embed_custom_a
        self.Cget_report_b = msg.embed_custom_b
        self.get_state_view()
        
    def get_state_view(self):
        if self.state_view:
            print("---------------client side--------------")
            print("DRVCONN(%d) STEERE_DEGREE(%03d)(%04d, %04d/%04d/%04d)"%(self.Cget_connection, self.Cget_steer_degree, self.Cget_steer_rawlevel, self.Cget_steer_limit_l, self.Cget_steer_limit_c, self.Cget_steer_limit_r))
            print("ROTOR_HA(%04d)(L:%04d R:%04d) VECMODE(%d) SBRK(%d)"%(self.Cget_rotor_hall, self.Cget_rotor_hall_l, self.Cget_rotor_hall_r, self.Cget_drvmode, self.Cget_brk))
            print("VOLTAGE(%.2f) CHARGING(%dA) DISCHARGING(%dA) TEMP(%d)"%(self.Cget_voltage, self.Cget_current_char, self.Cget_current_dischar, self.Cget_drv_temperature))
            print("LAMP(%d) SIGNAL(%d) CUSTOM_A(%d) CUSTOM_B(%d)"%(self.Cget_lamp, self.Cget_signal, self.Cget_report_a, self.Cget_report_b))

if __name__=='__main__':
    car_control = None
    try:
        car_control = cardrv_control_example("gilbot_client", demo_mode=True)
    except rospy.ROSInterruptException:
        pass
    car_control.demo_thread_run = False
    