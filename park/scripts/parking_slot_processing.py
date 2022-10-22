#!/usr/bin/python3
import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class slot_management():

    def __init__(self):
        
        rospy.init_node("data_processing", anonymous=False)

        rospy.Subscriber("/parking_slots", Int8MultiArray, self.callback)
        rospy.Subscriber("/gazebo/overhead_cam/overhead_image", Image, self.imagecallback)

        self.normalized = [[0.4546875, 0.08125], [0.5953125, 0.14375], [0.4546875, 0.153125], [0.5890625, 0.21875], [0.4515625, 0.225], [0.5953125, 0.2875], [0.459375, 0.284375], [0.5921875, 0.35625], [0.4578125, 0.35625], [0.590625, 0.4296875], [0.45625, 0.4296875], [0.59375, 0.4921875], [0.4578125, 0.4953125], [0.59375, 0.565625], [0.4578125, 0.5625], [0.5921875, 0.6328125], [0.4546875, 0.6328125], [0.590625, 0.7015625], [0.4578125, 0.703125], [0.59375, 0.7765625], [0.453125, 0.7765625], [0.59375, 0.8359375], [0.4515625, 0.8375], [0.5890625, 0.903125], [0.771875, 0.0828125], [0.903125, 0.1375], [0.771875, 0.1515625], [0.90625, 0.2109375], [0.7734375, 0.2234375], [0.9015625, 0.2828125], [0.7703125, 0.290625], [0.896875, 0.35], [0.7703125, 0.3578125], [0.9015625, 0.41875], [0.771875, 0.4265625], [0.9046875, 0.49375], [0.771875, 0.4921875], [0.903125, 0.5578125], [0.771875, 0.5625], [0.9015625, 0.6265625], [0.7765625, 0.6296875], [0.9, 0.6953125], [0.7765625, 0.696875], [0.903125, 0.7625], [0.778125, 0.7625], [0.903125, 0.8328125], [0.771875, 0.834375], [0.90625, 0.9015625]]

        self.bridge = CvBridge()

        self.slot_id = -1

        self.available_slots = []


    def imagecallback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.resized = cv2.resize(cv_image, None, fx = 0.5, fy = 0.5)
        self.h, self.w = self.resized.shape[:2]
        if self.slot_id != -1:

            no = 2*(24 - self.slot_id)
            cv2.rectangle(self.resized, (int(self.normalized[no][0]*self.w),int(self.normalized[no][1]*self.h)), (int(self.normalized[no+1][0]*self.w),int(self.normalized[no+1][1]*self.h)), (0, 255, 0), 2)
            cv2.imshow("nearest", self.resized)
            cv2.waitKey(1)

    def callback(self, data):

        self.available_slots = data.data


    def process(self):

        if len(self.available_slots) > 0:
            self.slot_id = min(self.available_slots, key=lambda x: x-12 if x>12 else x)

    def main(self):

        while not rospy.is_shutdown():
            self.process()
            rospy.loginfo(f"{self.slot_id}")

if __name__ == "__main__":
    
    parking_slot_manager = slot_management()

    parking_slot_manager.main()

    rospy.spin()
