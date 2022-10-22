#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from cv_bridge import CvBridge
# import numpy as np


class data_acquisition():

    def __init__(self):
        
        rospy.init_node(f'data_acquisition', anonymous=False)
        self.bridge = CvBridge()
        self.normalized = [[0.4546875, 0.08125], [0.5953125, 0.14375], [0.4546875, 0.153125], [0.5890625, 0.21875], [0.4515625, 0.225], [0.5953125, 0.2875], [0.459375, 0.284375], [0.5921875, 0.35625], [0.4578125, 0.35625], [0.590625, 0.4296875], [0.45625, 0.4296875], [0.59375, 0.4921875], [0.4578125, 0.4953125], [0.59375, 0.565625], [0.4578125, 0.5625], [0.5921875, 0.6328125], [0.4546875, 0.6328125], [0.590625, 0.7015625], [0.4578125, 0.703125], [0.59375, 0.7765625], [0.453125, 0.7765625], [0.59375, 0.8359375], [0.4515625, 0.8375], [0.5890625, 0.903125], [0.771875, 0.0828125], [0.903125, 0.1375], [0.771875, 0.1515625], [0.90625, 0.2109375], [0.7734375, 0.2234375], [0.9015625, 0.2828125], [0.7703125, 0.290625], [0.896875, 0.35], [0.7703125, 0.3578125], [0.9015625, 0.41875], [0.771875, 0.4265625], [0.9046875, 0.49375], [0.771875, 0.4921875], [0.903125, 0.5578125], [0.771875, 0.5625], [0.9015625, 0.6265625], [0.7765625, 0.6296875], [0.9, 0.6953125], [0.7765625, 0.696875], [0.903125, 0.7625], [0.778125, 0.7625], [0.903125, 0.8328125], [0.771875, 0.834375], [0.90625, 0.9015625]]
        self.slots = []

        rospy.Subscriber("/gazebo/overhead_cam/overhead_image", Image, self.callback)

        self.data_publisher = rospy.Publisher("/parking_slots", Int8MultiArray,queue_size=0)

        self.data = Int8MultiArray()



    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.resized = cv2.resize(cv_image, None, fx = 0.5, fy = 0.5)

    def process(self):
        gray = cv2.cvtColor(self.resized, cv2.COLOR_BGR2GRAY)
        # guass = cv2.GaussianBlur(gray, (5,5),0)
        # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 39, -2)
        ret, self.thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)
        # erode = cv2.erode(thresh, kernel, iterations=2)


    def update(self, max_area=0.1):

        self.img = self.resized.copy()
        self.slots = []
        h, w = self.img.shape[0:2]
        n = len(self.normalized)
        i = 0
        while i < n:
            y1, y2, x1, x2 = int(self.normalized[i][1]*h),int(self.normalized[i+1][1]*h), int(self.normalized[i][0]*w),int(self.normalized[i+1][0]*w)
            cv2.putText(self.img, f"{24 - i//2}", (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0), 2)
            slot = self.thresh[y1:y2,x1:x2]
            contours, hierarchy = cv2.findContours(slot, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                area = cv2.contourArea(max(contours, key=cv2.contourArea))
                # rospy.loginfo(f"{i}  {area/(slot.shape[0]*slot.shape[1])}")
                if area/(slot.shape[0]*slot.shape[1]) <= max_area:
                    cv2.rectangle(self.img, (x1,y1), (x2,y2), (0, 255, 0), 2)
                    self.slots.append(24 - i//2)
                else:
                    cv2.rectangle(self.img, (x1,y1), (x2,y2), (0, 0, 255), 2)
            else:
                cv2.rectangle(self.img, (x1,y1), (x2,y2), (0, 255, 0), 2)
                self.slots.append(24 - i//2)
            i += 2
        
        self.data.data = self.slots
        self.data_publisher.publish(self.data)


    def main(self):
        if hasattr(self, "resized"):
            self.process()
            self.update()
            cv2.imshow("output", self.img)
            cv2.waitKey(1)
        else:
            pass

  
if __name__ == '__main__':

    parking_lane = data_acquisition()

    r = rospy.Rate(29)
    while not rospy.is_shutdown():
        parking_lane.main()
        r.sleep()

  