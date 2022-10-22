#!/usr/bin/python3
import rospy
from std_msgs.msg import Int8MultiArray



def callback(data):

    available_slots = data.data
    slot_id = process(available_slots)
    rospy.loginfo(f"{slot_id}")


def process(slots_data):

    slot_id = min(slots_data, key=lambda x: x-12 if x>12 else x)

    return slot_id

def decision():

    rospy.init_node("data_processing", anonymous=False)
    rospy.Subscriber("/parking_slots", Int8MultiArray, callback)

    rospy.spin()

if __name__ == "__main__":
    decision()