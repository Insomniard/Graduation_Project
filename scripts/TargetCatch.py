#!/usr/bin/python3
import rospy
import cv2 as cv
import numpy as np
import time, threading
from std_msgs.msg import Int8
class targetcatch:
    def __init__(self):
        rospy.init_node('targetcatcher', anonymous=True)

        self.pub = rospy.Publisher('/handler_point',Int8, queue_size=1)
        self.pub_msg = Int8()
        self.hander_point = 0
        self.hsv = 0
        self.lower_blue1 = 0
        self.upper_blue1 = 0
        self.lower_blue2 = 0
        self.upper_blue2 = 0
        self.lower_blue3 = 0
        self.upper_blue3 = 0
        self.tracking = 60
        cv.namedWindow('img_color')
        cv.setMouseCallback('img_color', self.mouse_callback)
        self.cap = cv.VideoCapture(0)
        self.check_point = False
        self.horn_point = False
        fuckU = threading.Thread(target=self.Controlltarget)
        fuckU.start()


    def get_hander_point(self):
        return self.hander_point

    def mouse_callback(self, event, x, y, flags, param):
        
        ret, img_color=self.cap.read()
        self.hsv, self.lower_blue1, self.upper_blue1, self.lower_blue2, self.upper_blue2, self.lower_blue3, self.upper_blue3
        # 마우스 왼쪽 버튼 누를시 위치에 있는 픽셀값을 읽어와서 HSV로 변환합니다.
    
        if event == cv.EVENT_LBUTTONDOWN:
            print(img_color[y, x])
            color = img_color[y, x]
            one_pixel = np.uint8([[color]])
            self.hsv = cv.cvtColor(one_pixel, cv.COLOR_BGR2HSV)
            self.hsv = self.hsv[0][0]

            # HSV 색공간에서 마우스 클릭으로 얻은 픽셀값과 유사한 필셀값의 범위를 정합니다.
            if self.hsv[0] < 10:
                print("case1")
                self.lower_blue1 = np.array([self.hsv[0] - 10 + 180, self.tracking, self.tracking])
                self.upper_blue1 = np.array([180, 255, 255])
                self.lower_blue2 = np.array([0, self.tracking, self.tracking])
                self.upper_blue2 = np.array([self.hsv[0], 255, 255])
                self.lower_blue3 = np.array([self.hsv[0], self.tracking, self.tracking])
                self.upper_blue3 = np.array([self.hsv[0] + 10, 255, 255])
                #     print(i-10+180, 180, 0, i)
                #     print(i, i+10)
            elif self.hsv[0] > 170:
                print("case2")
                self.lower_blue1 = np.array([self.hsv[0], self.tracking, self.tracking])
                self.upper_blue1 = np.array([180, 255, 255])
                self.lower_blue2 = np.array([0, self.tracking, self.tracking])
                self.upper_blue2 = np.array([self.hsv[0] + 10 - 180, 255, 255])
                self.lower_blue3 = np.array([self.hsv[0] - 10, self.tracking, self.tracking])
                self.upper_blue3 = np.array([self.hsv[0], 255, 255])
                #     print(i, 180, 0, i+10-180)
                #     print(i-10, i)
            else:
                print("case3")
                self.lower_blue1 = np.array([self.hsv[0], self.tracking, self.tracking])
                self.upper_blue1 = np.array([self.hsv[0] + 10, 255, 255])
                self.lower_blue2 = np.array([self.hsv[0] - 10, self.tracking, self.tracking])
                self.upper_blue2 = np.array([self.hsv[0], 255, 255])
                self.lower_blue3 = np.array([self.hsv[0] - 10, self.tracking, self.tracking])
                self.upper_blue3 = np.array([self.hsv[0], 255, 255])
                #     print(i, i+10)
                #     print(i-10, i)

            print(self.hsv[0])
            print("@1", self.lower_blue1, "~", self.upper_blue1)
            print("@2", self.lower_blue2, "~", self.upper_blue2)
            print("@3", self.lower_blue3, "~", self.upper_blue3)


    def Controlltarget(self):
        
        while True:
            ret, img_color = self.cap.read()
            # ret, frame = cap.read()
            if ret:
                        
                cv.rectangle(img_color, (140, 40), (485, 450), (0, 0, 255), 5)
                # img_color = cv.imread('2.jpg')
                height, width = img_color.shape[:2]
                img_color = cv.resize(img_color, (width, height), interpolation=cv.INTER_AREA)

                # 원본 영상을 HSV 영상으로 변환합니다.
                img_hsv = cv.cvtColor(img_color, cv.COLOR_BGR2HSV)

                # 범위 값으로 HSV 이미지에서 마스크를 생성합니다.
                img_mask1 = cv.inRange(img_hsv, self.lower_blue1, self.upper_blue1)
                img_mask2 = cv.inRange(img_hsv, self.lower_blue2, self.upper_blue2)
                img_mask3 = cv.inRange(img_hsv, self.lower_blue3, self. upper_blue3)
                img_mask = img_mask1 | img_mask2 | img_mask3

                kerner = np.ones((11, 11), np.uint8)
                img_mask = cv.morphologyEx(img_mask, cv.MORPH_OPEN, kerner)
                img_mask = cv.morphologyEx(img_mask, cv.MORPH_CLOSE, kerner)

                numofLabels, img_label, stats, centroids = cv.connectedComponentsWithStats(img_mask)
                temp = []
                # stats[:2] -> 좌표값
                for idx, centroids in enumerate(centroids):
                    if stats[idx][0] == 0 and stats[idx][1] == 0:
                        continue
                    if np.any(np.isnan(centroids)):
                        continue
                    x, y, width, height, area = stats[idx]
                    centerX, centerY = int(centroids[0]), int(centroids[1])
                    # stats의 맨 마지막값이 면적 (area) -> 이 값 활용하여 특정객체 인식
                    # temp = list(int(area))
                    # for i in range(0, 100):
                    #     temp.append(area)
                    # areamax = max(temp)
                    # if areamax:
                    
                    if centerX > 140 and centerX < 485 and centerY > 40 and centerY < 450:
                        check_point = True
                        self.hander_point = 0
                        horn_point = False
                        # print("catch target")
                    elif centerX < 140 and centerX>20:
                        # print("Left Missing")
                        self.hander_point = 1
                    elif centerX > 485 and centerX < 620:
                        self.hander_point = 2
                        # print("Right Missing")
                    else:
                        check_point = False
                        # print("Missing target")
                        horn_point = True
                        self.hander_point = 4
                    
                    self.pub_msg.data = self.hander_point
                    self.pub.publish(self.pub_msg)
                    # print(area)
                    # print("=========")
                    # print(centerX)
                    # print(centerY)
                    # print("==========")
                    # if temp(max)==area:
                   # print("camera h : ",self.hander_point)
                # 마스크 이미지로 원본 이미지에서 범위값에 해당되는 영상 부분을 획득합니다.
                img_result = cv.bitwise_and(img_color, img_color, mask=img_mask)
                cv.imshow('img_color', img_color)
                # cv.imshow('img_mask', img_mask)
                cv.imshow('img_result', img_result)
                

            # ESC 키누르면 종료
                if cv.waitKey(1) & 0xFF == 27:
                    
                    break
                
        cv.destroyAllWindows()

    
    

if __name__=='__main__':
    targetcatch()
