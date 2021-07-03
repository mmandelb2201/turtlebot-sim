#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic.publisher')
bot_one_pub = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)
bot_two_pub = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = 0.5

while not rospy.is_shutdown():
    bot_one_pub.publish(move)
    bot_two_pub.publish(move)
    rate.sleep()
