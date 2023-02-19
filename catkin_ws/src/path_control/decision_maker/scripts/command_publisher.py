#!/usr/bin/env python3
import rospy

from std_msgs import String

def command_publisher(command):
    command = str(command)
    rospy.init_node('command_publisher', anonymous=True)
    command_pub = rospy.Publisher('/automobile/command', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(command)
        command_pub.publish(command)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        command = {"action": 1, "speed": 0.0}
        command_publisher(command)
    except rospy.ROSInterruptException:
        pass
