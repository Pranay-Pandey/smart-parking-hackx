#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from cv_bridge import CvBridge
# import numpy as np

bridge = CvBridge()
normalized = [[0.4546875, 0.08125], [0.5953125, 0.14375], [0.4546875, 0.153125], [0.5890625, 0.21875], [0.4515625, 0.225], [0.5953125, 0.2875], [0.459375, 0.284375], [0.5921875, 0.35625], [0.4578125, 0.35625], [0.590625, 0.4296875], [0.45625, 0.4296875], [0.59375, 0.4921875], [0.4578125, 0.4953125], [0.59375, 0.565625], [0.4578125, 0.5625], [0.5921875, 0.6328125], [0.4546875, 0.6328125], [0.590625, 0.7015625], [0.4578125, 0.703125], [0.59375, 0.7765625], [0.453125, 0.7765625], [0.59375, 0.8359375], [0.4515625, 0.8375], [0.5890625, 0.903125], [0.771875, 0.0828125], [0.903125, 0.1375], [0.771875, 0.1515625], [0.90625, 0.2109375], [0.7734375, 0.2234375], [0.9015625, 0.2828125], [0.7703125, 0.290625], [0.896875, 0.35], [0.7703125, 0.3578125], [0.9015625, 0.41875], [0.771875, 0.4265625], [0.9046875, 0.49375], [0.771875, 0.4921875], [0.903125, 0.5578125], [0.771875, 0.5625], [0.9015625, 0.6265625], [0.7765625, 0.6296875], [0.9, 0.6953125], [0.7765625, 0.696875], [0.903125, 0.7625], [0.778125, 0.7625], [0.903125, 0.8328125], [0.771875, 0.834375], [0.90625, 0.9015625]]
# kernel = np.ones((5,5), np.uint8)
slots = []
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    resized = cv2.resize(cv_image, None, fx = 0.5, fy = 0.5)
    thresh = process(resized)
    output, slots = update(resized, thresh)
    # cv2.imshow("thresh", thresh)
    cv2.imshow("output", output)
    # rospy.loginfo(f"{slots}")
    cv2.waitKey(1)

    

def process(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # guass = cv2.GaussianBlur(gray, (5,5),0)
    # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 39, -2)
    ret, thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)
    # erode = cv2.erode(thresh, kernel, iterations=2) 
    return thresh

def update(img, thresh, max_area=0.1):
    h, w = img.shape[0:2]
    n = len(normalized)
    i = 0
    while i < n:
        y1, y2, x1, x2 = int(normalized[i][1]*h),int(normalized[i+1][1]*h), int(normalized[i][0]*w),int(normalized[i+1][0]*w)
        cv2.putText(img, f"{24 - i//2}", (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0), 2)
        slot = thresh[y1:y2,x1:x2]
        
        contours, hierarchy = cv2.findContours(slot, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            area = cv2.contourArea(max(contours, key=cv2.contourArea))
            # rospy.loginfo(f"{i}  {area/(slot.shape[0]*slot.shape[1])}")
            if area/(slot.shape[0]*slot.shape[1]) <= max_area:
                cv2.rectangle(img, (x1,y1), (x2,y2), (0, 255, 0), 2)
                slots.append(24 - i//2)
            else:
                cv2.rectangle(img, (x1,y1), (x2,y2), (0, 0, 255), 2)
        else:
            cv2.rectangle(img, (x1,y1), (x2,y2), (0, 255, 0), 2)
            slots.append(24 - i//2)
        i += 2
    data = Int8MultiArray()
    data.data = slots
    rospy.Publisher("/parking_slots", Int8MultiArray,queue_size=10).publish(data)
    return img, slots
def data_acquisition():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('data_acquisition', anonymous=False)

    rospy.Subscriber("/gazebo/overhead_cam/overhead_image", Image, callback)
    # rospy.Publisher("/parking_slots", Int8MultiArray,queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
  data_acquisition()