#!/usr/bin/env python

from science import Science
import sys
import rospy
from std_msgs.msg import Empty

def run():
	rospy.init_node("teleop_science_pub")

	distance_calibration = rospy.Publisher("teleop_science/distance_sensor_calibration", Empty, queue_size = 10)

	start = rospy.Publisher("teleop_science/start", Empty, queue_size = 10)

	container_up = rospy.Publisher("teleop_science/container_up", Empty, queue_size = 10)
	container_down = rospy.Publisher("teleop_science/container_down", Empty, queue_size = 10)
	container_stop = rospy.Publisher("teleop_science/container_stop", Empty, queue_size = 10)
	
	drill_up = rospy.Publisher("teleop_science/drill_up", Empty, queue_size = 10)
	drill_down = rospy.Publisher("teleop_science/drill_down", Empty, queue_size = 10)
	drill_stop = rospy.Publisher("teleop_science/drill_stop", Empty, queue_size = 10)
	
	drill_cw = rospy.Publisher("teleop_science/drill_cw", Empty, queue_size = 10)
	drill_ccw = rospy.Publisher("teleop_science/drill_ccw", Empty, queue_size = 10)
	drill_off = rospy.Publisher("teleop_science/drill_off", Empty, queue_size = 10)
	
	container_open = rospy.Publisher("teleop_science/container_open", Empty, queue_size = 10)
	container_close = rospy.Publisher("teleop_science/container_close", Empty, queue_size = 10)
	
	probe_up = rospy.Publisher("teleop_science/probe_up", Empty, queue_size = 10)
	probe_down = rospy.Publisher("teleop_science/probe_down", Empty, queue_size = 10)
	
	stop_motors = rospy.Publisher("teleop_science/stop_motors", Empty, queue_size = 10)
	kill = rospy.Publisher("teleop_science/kill", Empty, queue_size = 10)
	

	print("Enter commands: ")

	while not rospy.is_shutdown():
		cmd = sys.stdin.readline()
		cmd = cmd[0]

		if cmd == 'l':
			distance_calibration.publish(Empty())

		elif cmd == 'z':
			start.publish(Empty())

		elif cmd == 'q':
			container_up.publish(Empty())
		elif cmd == 'a':
			container_down.publish(Empty())
		elif cmd == 'w':
			container_stop.publish(Empty())

		elif cmd == 'e':
			drill_up.publish(Empty())
		elif cmd == 'd':
			drill_down.publish(Empty())
		elif cmd == 'r':
			drill_stop.publish(Empty())

		elif cmd == 't':
			drill_cw.publish(Empty())
		elif cmd == 'g':
			drill_ccw.publish(Empty())
		elif cmd == 'y':
			drill_off.publish(Empty())

		elif cmd == 'u':
			container_open.publish(Empty())
		elif cmd == 'j':
			container_close.publish(Empty())

		elif cmd == 'i':
			probe_up.publish(Empty())
		elif cmd == 'k':
			probe_down.publish(Empty())
		
		elif cmd == 'x':
			stop_motors.publish(Empty())
		elif cmd == 'c':
			kill.publish(Empty())
			exit()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
