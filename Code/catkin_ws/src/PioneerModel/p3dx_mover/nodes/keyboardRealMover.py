#!/usr/bin/env python
import getch
import roslib; roslib.load_manifest('p3dx_mover')
import rospy


from geometry_msgs.msg import Twist



KEY_UP = 65
KEY_DOWN = 66
KEY_RIGHT = 67
KEY_LEFT = 68
USER_QUIT = 100

MAX_FORWARD = 1.2
MAX_LEFT = 0.4
MIN_FORWARD = -1.2
MIN_LEFT = -0.4

forward = 0.0
left = 0.0
keyPress = 0

while(keyPress != USER_QUIT):
	pub = rospy.Publisher('/RosAria/cmd_vel', Twist)
	rospy.init_node('userToRosAria')

	twist = Twist()

	keyPress = getch.getArrow()

	if((keyPress == KEY_UP) and (forward <= MAX_FORWARD)):
		forward += 0.05
	elif((keyPress == KEY_DOWN) and (forward >= MIN_FORWARD)):
		forward -= 0.05
	elif((keyPress == KEY_LEFT) and (left <= MAX_LEFT)):
		left += 0.05
	elif((keyPress == KEY_RIGHT) and (left >= MIN_LEFT)):
		left -= 0.05

	# max backward/forward speed is 1.2 m/s
	if(forward > 1.2):
		forward = 1.2
	elif(forward < -1.2):
		forward = -1.2

	if(left > 0.4):
		left = 0.4
	elif(left < -0.4):
		left = -0.4

	twist.linear.x = forward
	twist.angular.z = left
	pub.publish(twist)
	rospy.sleep(0.01)


pub = rospy.Publisher('/RosAria/cmd_vel', Twist)
rospy.init_node('userToRosAria')
twist = Twist()
pub.publish(twist)
exit()
	
