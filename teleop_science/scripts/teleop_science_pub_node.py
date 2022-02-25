#!/usr/bin/env python

from science import Science
import sys
import rospy
from std_msgs.msg import Empty

def run():

	rospy.init_node("teleop_science_pub")

	arm_up = rospy.Publisher("teleop_science/arm_up", Empty, queue_size = 10)
	arm_down = rospy.Publisher("teleop_science/arm_down", Empty, queue_size = 10)
	arm_off = rospy.Publisher("teleop_science/arm_off", Empty, queue_size = 10)
	drill_up = rospy.Publisher("teleop_science/drill_up", Empty, queue_size = 10)
	drill_down = rospy.Publisher("teleop_science/drill_down", Empty, queue_size = 10)
	drill_off = rospy.Publisher("teleop_science/drill_off", Empty, queue_size = 10)
	drill_toggle = rospy.Publisher("teleop_science/drill_toggle", Empty, queue_size = 10)
	container_open = rospy.Publisher("teleop_science/container_open", Empty, queue_size = 10)
	container_close = rospy.Publisher("teleop_science/container_close", Empty, queue_size = 10)
	start = rospy.Publisher("teleop_science/start", Empty, queue_size = 10)
	stop = rospy.Publisher("teleop_science/stop", Empty, queue_size = 10)
	dist_calib = rospy.Publisher("teleop_science/distance_sensor_calibration", Empty, queue_size = 10)

	print("Enter commands: ")

	cmd = 'a'
	while not rospy.is_shutdown():
		cmd = sys.stdin.readline()

		for c in cmd:
			if c == 'w':
				arm_up.publish(Empty())
			elif c == 's':
				arm_down.publish(Empty())
			elif c == 'e':
				arm_off.publish(Empty())
			elif c == 'r':
				drill_up.publish(Empty())
			elif c == 'f':
				drill_down.publish(Empty())
			elif c == 't':
				drill_off.publish(Empty())
			elif c == 'd':
				drill_toggle.publish(Empty())
			elif c == 'o':
				container_open.publish(Empty())
			elif c == 'c':
				container_close.publish(Empty())
			elif c == 'p':
				start.publish(Empty())
			elif c == 'q':
				stop.publish(Empty())
			elif c == 'l':
				dist_calib.publish(Empty())
			elif c == 'x':
				print("Exit")
				exit()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
