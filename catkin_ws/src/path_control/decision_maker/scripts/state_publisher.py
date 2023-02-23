#!/usr/bin/env python3
import rospy
import math

from nav_msgs import Odometry

L = 26.0 # Wheel_base

def state_publisher(s, phi, theta):
    """
    s     : current_speed
    phi   : steering_command
    theta : curvature
    """
    x_dot = s * math.cos(math.radians(theta))
    y_dot = s * math.sin(math.radians(theta))
    theta_dot = s / L * phi
    pose_data = Odometry()
    pose_data.pose.pose.position.x = x_dot
    pose_data.pose.pose.position.y = y_dot
    pose_data.twist.twist.linear.z = theta_dot

    rospy.init_node('state_publisher', anonymous=True)
    state_pub = rospy.Publisher('/automobile/state_pose', Odometry, queue_size=50)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #rospy.loginfo(command)
        state_pub.publish(pose_data)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        s, phi, theta = 0, 0, 0
        state_publisher(s, phi, theta)
    except rospy.ROSInterruptException:
        pass
