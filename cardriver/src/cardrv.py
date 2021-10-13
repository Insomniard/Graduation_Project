#!/usr/bin/env python3
import sys, rospy, time, serial, threading
from std_msgs import msg
from std_msgs.msg import String
from cardriver.msg import control_rx, control_tx

class gilbot_pubsub:
    def __init__(self, comport="ttyTHS2", nodeName="gilbot_r1"):
        self.nodeName = nodeName
        self.comm = comport
        self.serial_port = serial.Serial(
            port="/dev/" + self.comm,
            baudrate=38400,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0)
        
        self.rx_frames = []
        self.thread_run = True
        self.system_shutdown = False
        self.subscriber_timeout = 300

        self.old_rotor_hall_l = 0
        self.old_rotor_hall_r = 0
        
        #rospy.init_node(self.nodeName, anonymous=True)
        self.sub_car_control =      rospy.Subscriber("/command/", control_tx, self.sub_callback_active)
        self.pub_car_status =       rospy.Publisher("/value/", control_rx, queue_size=2)
        self.car_status_msg = control_rx()

        self.rate = rospy.Rate(10)
        rospy.loginfo("Start / Port:%s / Baudrate:%d / Node: %s"%(self.comm, self.serial_port.baudrate, self.nodeName))
        
        #driver control variables
        self.Cset_aimode = 0
        self.Cset_drvmode = 0
        self.Cset_throttle = 0
        self.Cset_steer_degree = 250
        self.Cset_steer_raw = 1500
        self.Cset_signal = 0
        self.Cset_lamp = 0
        self.Cset_horn = 0
        self.Cset_custom_a = 0
        self.Cset_custom_b = 0

        #driver reading variables
        self.Cget_connection = 0
        self.Cget_steer_degree = 250
        self.Cget_steer_rawlevel = 9999
        self.Cget_rotor_hall = 0
        self.return_cmd = 0
        self.Cget_steer_limit_l = 111
        self.Cget_steer_limit_c = 555
        self.Cget_steer_limit_r = 999
        self.Cget_voltage = 1800
        self.Cget_rotor_hall_l = 0
        self.Cget_rotor_hall_r = 0
        self.Cget_current_char = 0
        self.Cget_current_dischar = 0
        self.Cget_drv_temperature = 0
        self.Cget_drvmode = 0
        self.Cget_lamp = 0
        self.Cget_signal = 0
        self.Cget_brk = 0
        self.Cget_report_a = 0
        self.Cget_report_b = 0

         #timeout timer rospyRate 40Hz(25ms), 40cycle is 1 second.
        self.subscriber_timeout_cycle = 10
        
        if not rospy.is_shutdown():
            self.receive_thread = threading.Timer(0, self.serial_threading_rx)
            self.receive_thread.start()
            self.transmit_thread = threading.Timer(0.1, self.serial_threading_tx)
            self.transmit_thread.start()
            time.sleep(0.5) #port connetion check global deley.

        self.pub_sender_loop()

        self.sys_stop()
        self.print_log("Keeping main thread for other multiple thread join.....", end="")
        self.receive_thread.join()
        self.transmit_thread.join()
        self.print_log("Complite.")

    def speed_calculate(self, read_hall_l, read_hall_r):
        speed_poleSecond = 0
        speed_rpm_state_l = 0
        speed_rpm_state_r = 0
        speed_mms_state = 0
        speed_kmh_state = 0
        speed_kmh_state_f = 0

        read_poll_all = 0
        
        temp_int_hall = 0
        if(self.old_rotor_hall_l < read_hall_l):
            temp_int_hall += read_hall_l - self.old_rotor_hall_l
        elif(self.old_rotor_hall_l > read_hall_l):
            temp_int_hall += 4095-self.old_rotor_hall_l+read_hall_l
        read_poll_all += temp_int_hall
        rpm_state_l = temp_int_hall
        
        temp_int_hall = 0
        if(self.old_rotor_hall_r < read_hall_r):
            temp_int_hall += read_hall_r - self.old_rotor_hall_r
        elif(self.old_rotor_hall_r > read_hall_r):
            temp_int_hall += 4095-self.old_rotor_hall_r+read_hall_r
        read_poll_all += temp_int_hall
        rpm_state_r = temp_int_hall

        self.old_rotor_hall_l = read_hall_l
        self.old_rotor_hall_r = read_hall_r
        
        if(read_poll_all != 0):
            # 1초당 실행된다는 가정하에 RPM이 계산됨, 홀수는 3개중 1개만 사용하므로 24를 나누지 않고 8폴을 나눔.
            # (1초동안 읽어진 홀수)/(모터폴수)/(모터기어비)*(분단위60초)
            speed_poleSecond = (read_poll_all/2)
            speed_rpm_state_l = rpm_state_l/8/14.5*60
            speed_rpm_state_r = rpm_state_r/8/14.5*60

            #(인치단위 * 11인치) / (기어비14.5*모터폴수8)
            speed_cal_step_mm = (25.4 * 3.141592 * 11) / (14.5*8)

            speed_mms_state = speed_cal_step_mm * (read_poll_all/2)
            speed_kmh_state = speed_mms_state / 277
            speed_kmh_state_f = speed_mms_state / 27.7

        #반환 타입을 선택 한다.
        # speed_poleSecond : 초당 읽어진 폴수 (양 단 바퀴 폴수로 인식)
        # speed_rpm_state_l : 좌측 바퀴 RPM
        # speed_rpm_state_r : 우측 바퀴 RPM
        # speed_mms_state : 속도 (mm 단위 / 초)
        # speed_kmh_state : 속도 (km 단위 / 시간)
        # speed_kmh_state_f: 속도 (km 단위 소수점 포함 / 시간)
        return int(speed_mms_state)

    def pub_sender_loop(self):
        second_loop_cnt = 0
        while self.thread_run and not rospy.is_shutdown():
            try:
                second_loop_cnt += 1
                if(second_loop_cnt >= 10):
                    second_loop_cnt = 0
                    self.Cget_rotor_hall = self.speed_calculate(self.Cget_rotor_hall_l, self.Cget_rotor_hall_r)
                
                if self.subscriber_timeout > 0: self.subscriber_timeout -= 1 #connection check variable timer cycle
                if self.subscriber_timeout == 0: self.driver_ctrl_reset()
                
                self.car_status_msg.car_connected = self.Cget_connection
                self.car_status_msg.steer_degree = self.Cget_steer_degree
                self.car_status_msg.steer_raw = self.Cget_steer_rawlevel
                self.car_status_msg.steer_limit_left = self.Cget_steer_limit_l
                self.car_status_msg.steer_limit_center = self.Cget_steer_limit_c
                self.car_status_msg.steer_limit_right = self.Cget_steer_limit_r
                self.car_status_msg.rotor_hall = self.Cget_rotor_hall
                self.car_status_msg.rotor_hall_l = self.Cget_rotor_hall_l
                self.car_status_msg.rotor_hall_r = self.Cget_rotor_hall_r
                self.car_status_msg.voltage = self.Cget_voltage
                self.car_status_msg.current_charging = self.Cget_current_char
                self.car_status_msg.current_discharging = self.Cget_current_dischar
                self.car_status_msg.drv_temperature = self.Cget_drv_temperature
                self.car_status_msg.car_vector = self.Cget_drvmode
                self.car_status_msg.car_lamp = self.Cget_lamp
                self.car_status_msg.car_signal = self.Cget_signal
                self.car_status_msg.car_solidbrk = self.Cget_brk
                self.car_status_msg.embed_custom_a = self.Cget_report_a
                self.car_status_msg.embed_custom_b = self.Cget_report_b
                
                self.pub_car_status.publish(self.car_status_msg)
                
                self.rate.sleep()
            except KeyboardInterrupt:
                self.print_log("Keyboard exception !")
                self.sys_stop()
            except:
                self.sys_stop()
    
    def sub_callback_active(self, msg):
        self.subscriber_timeout = self.subscriber_timeout_cycle

        chk_value = msg.aimode
        if chk_value <= 3: self.Cset_aimode = chk_value
        chk_value = msg.drive_vector
        if chk_value <= 3: self.Cset_drvmode = chk_value
        chk_value = msg.drive_throttle
        if chk_value <= 99: self.Cset_throttle = chk_value
        chk_value = msg.steer_degree
        if chk_value <= 180: self.Cset_steer_degree = chk_value
        chk_value = msg.steer_raw
        if chk_value <= 1023: self.Cset_steer_raw = chk_value
        chk_value = msg.signal
        if chk_value <= 3: self.Cset_signal = chk_value
        chk_value = msg.lamp
        if chk_value <= 1: self.Cset_lamp = chk_value
        chk_value = msg.horn
        if chk_value <= 1: self.Cset_horn = chk_value
        chk_value = msg.embed_custom_a
        if chk_value <= 9: self.Cset_custom_a = chk_value
        chk_value = msg.embed_custom_b
        if chk_value <= 9: self.Cset_custom_b = chk_value
        
    def driver_ctrl_reset(self):
        self.Cset_aimode = 0
        self.Cset_drvmode = 0
        self.Cset_throttle = 0
        self.Cset_steer_degree = 250
        self.Cset_steer_raw = 9999
        self.Cset_signal = 0
        self.Cset_lamp = 0
        self.Cset_horn = 0
        
    def print_log(self, str_data, end="\r\n"):
        print(str_data, end=end)
        #rospy.loginfo(str_data)
        
    def sys_stop(self):
        self.thread_run = False
        if not self.system_shutdown:
            self.system_shutdown = True
            self.serial_port.cancel_read()
            self.serial_port.close()
            rospy.signal_shutdown("Node Close.")

    def serial_threading_rx(self):
        while self.thread_run and not rospy.is_shutdown():
            try:
                for c in self.serial_port.read():
                    self.rx_frames.append(chr(c))
                    if c == 13:
                        self.serial_command_receive(''.join(self.rx_frames))
                        del self.rx_frames[:]
                    elif len(self.rx_frames) > 255:
                        del self.rx_frames[:]
            except KeyboardInterrupt:
                self.print_log("Keyboard exception !")
                self.sys_stop()
            except Exception as e:
                self.print_log("Error occurred. Exiting Program: " + str(e))
                self.thread_run = False
        self.print_log("Close car driver rx thread.")
        self.sys_stop()
    
    def serial_command_receive(self, rx_frame):
        #RX frame EX: O,999,0418,0003,4,2912,027[13]
        # O: car to steer driver connection state ('O':connected, 'X':disconnected)
        # 999: get steer degree
        # 0418: get steer raw level
        # 0003: get left, right rotor hall average. (left+right)
        # 4: return data command
        #   ['1':left steer limit, '2':center steer level, '3':right steer limit, 
        #       '4':get voltage(2920=29.20v), '5':left rotor hall(0~4095), '6':right rotor hall(0~4095), 
        #       '7':current state('C'=charging, 'D'=discharging, 'xxy'=xx.yA)
        #       '8':breaking driver temperature('Ready...')
        #       '9':get mode ('abcd': a=drvmodestate, b=lampstate, c=signalstate, d=brkstate)
        #       '10':report_data_a, '11':report_data_b, else: none data]
        # 2912: return data 4 is get voltage (2912=29.12v)
        # 027: xor frame check data ([O,999,0418,0003,4,2912,] frame decimal xor. (non ascii))
        # [13]: last word [line feed, 13]
        split_data = None
        try: split_data = rx_frame.split(',')
        except: return
        
        if len(split_data) != 7: return
        
        cal_check_data = ord(rx_frame[0])
        for i in range(1, len(rx_frame)-4):
            cal_check_data ^= ord(rx_frame[i])
        
        if cal_check_data != int(split_data[6]): return
        
        if split_data[0] == 'O': self.Cget_connection = 1
        else: self.Cget_connection = 0
        self.Cget_steer_degree = int(split_data[1])
        self.Cget_steer_rawlevel = int(split_data[2])
        #self.Cget_rotor_hall = int(split_data[3]) #메인 홀데이터는 오버플로우 현상 처리 문제로 읽지 않겠음.
        self.return_cmd = int(split_data[4])
        if self.return_cmd == 1: self.Cget_steer_limit_l = int(split_data[5])
        elif self.return_cmd == 2: self.Cget_steer_limit_c = int(split_data[5])
        elif self.return_cmd == 3: self.Cget_steer_limit_r = int(split_data[5])
        elif self.return_cmd == 4: self.Cget_voltage = int(split_data[5])
        elif self.return_cmd == 5: self.Cget_rotor_hall_l = int(split_data[5])
        elif self.return_cmd == 6: self.Cget_rotor_hall_r = int(split_data[5])
        elif self.return_cmd == 7:
            if split_data[5][0] == '1':
                self.Cget_current_char = 0
                self.Cget_current_dischar = int(split_data[5][1:3])
            else:
                self.Cget_current_char = int(split_data[5][1:3])
                self.Cget_current_dischar = 0
        elif self.return_cmd == 8: self.Cget_drv_temperature = int(split_data[5])
        elif self.return_cmd == 9: 
            self.Cget_drvmode = int(split_data[5][0:1])
            self.Cget_lamp = int(split_data[5][1:2])
            self.Cget_signal = int(split_data[5][2:3])
            self.Cget_brk = int(split_data[5][3:4])
        elif self.return_cmd == 10: self.Cget_report_a = int(split_data[5])
        elif self.return_cmd == 11: self.Cget_report_b = int(split_data[5])

    def serial_threading_tx(self):
        #frame address0123456789.......
        #TX frame EX: F000901500EXX0004N010[13]
        # 0:Drive Mode ('F':Forward, 'R':Reverse, 'B':Break, else:Normal)
        # 1,2:Throttle level (00~99)
        # 3,4,5:Steer degree set (0~180, 181 upper is off)
        # 6,7,8,9:Steer level for ad steer level (0~1023, 1024 upper is off)
        # 10:signal lamp control ('L':left on, 'R':right on, 'E':both on, else:off)
        # 11:lamp control ('O':enable, else:disable)
        # 12:buzzer(horn) control ('O':enable, else:disable)
        # 13:set_custom_a
        # 14:set_custom_b
        # 15:return data cmd(10x)
        # 16:return data cmd(1x)
        # 17:ai action mode (N:OFF, S:JustSteer, P:CarReadyandPCSide, F:ForceControl)
        # 18,19,20: xor frame check data ([F000901500EXX0004N] frame decimal xor (non ascii))
        # 21: last word [line feed, 13]

        tx_frame = ""
        fail_count = 0
        loopchecker = 1

        while self.thread_run and not rospy.is_shutdown():
            try:
                time.sleep(0.05)
                if fail_count >= 5:
                    self.print_log("send fail. limited.")
                    self.thread_run = False
                
                if self.Cset_drvmode == 1: tx_frame = "F"
                elif self.Cset_drvmode == 2: tx_frame = "R"
                elif self.Cset_drvmode == 3: tx_frame = "B"
                else: tx_frame = "N"

                tx_frame += "%02d"%(self.Cset_throttle)
                tx_frame += "%03d"%(self.Cset_steer_degree)
                tx_frame += "%04d"%(self.Cset_steer_raw)

                if self.Cset_signal == 1: tx_frame += "L"
                elif self.Cset_signal == 2: tx_frame += "R"
                elif self.Cset_signal == 3: tx_frame += "E"
                else: tx_frame += "X"

                if self.Cset_lamp == 1: tx_frame += "O"
                else: tx_frame += "X"

                if self.Cset_horn == 1: tx_frame += "O"
                else: tx_frame += "X"

                if self.Cset_custom_a > 9: tx_frame += "9"
                else: tx_frame += "%d"%(self.Cset_custom_a)

                if self.Cset_custom_b > 9: tx_frame += "9"
                else: tx_frame += "%d"%(self.Cset_custom_b)

                tx_frame += "%02d"%(loopchecker)

                if self.Cset_aimode == 3: tx_frame += "F" #Force control all
                elif self.Cset_aimode == 2: tx_frame += "P" #Pair control all
                elif self.Cset_aimode == 1: tx_frame += "S" #steer only control
                else: tx_frame += "N" #pilot mode off

                cal_check_data = ord(tx_frame[0])
                for i in range(1, len(tx_frame)):
                    cal_check_data ^= ord(tx_frame[i])
                tx_frame += "%03d\r"%(cal_check_data)
                #self.print_log("OUT: " + tx_frame + "=>")
                
                loopchecker += 1
                if loopchecker > 11: loopchecker = 1
                
                self.serial_port.write(tx_frame.encode())
                fail_count = 0
            except KeyboardInterrupt:
                self.print_log("Keyboard exception !")
                self.sys_stop()
            except Exception as e:
                self.print_log("send fail. " + str(e))
                if fail_count < 5: fail_count += 1

        self.print_log("Close car driver tx thread.")
        self.sys_stop()

if __name__=='__main__':
    car_active = None
    start_port = "ttyTHS2"
    node_name = "gilbot_r1"

    try: start_port = str(sys.argv[1])
    except: start_port = "ttyTHS2" #default serial port

    try: node_name = str(sys.argv[2])
    except: node_name = "gilbot_r1" #default node name

    try:
        rospy.init_node(node_name, anonymous = True)
        car_active = gilbot_pubsub(start_port, node_name)
    except rospy.ROSInterruptException:
        car_active.sys_stop()
        pass
