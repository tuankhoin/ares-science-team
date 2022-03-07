#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from science import Science


sci = Science()
sci.connect('/dev/ttyACM0')

if not sci.ready():
	print("Science payload connection not established")	
	exit()

print("Science payload connection established.")


def distance_calibration(data):
    sci.distance_calibration()


def start(data):
    sci.start()


def container_up(data):
    sci.container_up()

def container_down(data):
    sci.container_down()

def container_stop(data):
    sci.container_stop()


def drill_up(data):
    sci.drill_up()

def drill_down(data):
    sci.drill_down()

def drill_stop(data):
    sci.drill_stop()


def drill_cw(data):
    sci.drill_cw()

def drill_ccw(data):
    sci.drill_ccw()

def drill_off(data):
    sci.drill_off()


def container_open(data):
    sci.container_open()

def container_close(data):
    sci.container_close()


def probe_up(data):
    sci.container_open()

def probe_down(data):
    sci.container_close()




def stop_motors(data):
    sci.stop_motors()

def kill(data):
    sci.kill()


def teleop_science():
    rospy.init_node('teleop_science_sub', anonymous=True)

    rospy.Subscriber("teleop_science/distance_sensor_calibration", Empty, distance_calibration)

    rospy.Subscriber("teleop_science/start", Empty, start)

    rospy.Subscriber("teleop_science/container_up", Empty,  container_up)
    rospy.Subscriber("teleop_science/container_down", Empty,  container_down)
    rospy.Subscriber("teleop_science/container_stop", Empty,  container_stop)

    rospy.Subscriber("teleop_science/drill_up", Empty,  drill_up)
    rospy.Subscriber("teleop_science/drill_down", Empty,  drill_down)
    rospy.Subscriber("teleop_science/drill_stop", Empty,  drill_stop)

    rospy.Subscriber("teleop_science/drill_cw", Empty,  drill_cw)
    rospy.Subscriber("teleop_science/drill_ccw", Empty,  drill_ccw)
    rospy.Subscriber("teleop_science/drill_off", Empty,  drill_off)

    rospy.Subscriber("teleop_science/container_open", Empty,  container_open)
    rospy.Subscriber("teleop_science/container_close", Empty,  container_close)

    rospy.Subscriber("teleop_science/probe_up", Empty,  probe_up)
    rospy.Subscriber("teleop_science/probe_down", Empty,  probe_down)

    rospy.Subscriber("teleop_science/stop_motors", Empty, stop_motors)
    rospy.Subscriber("teleop_science/kill", Empty,  kill)

    print("ROS ready.")
    rospy.spin()


if __name__ == '__main__':
    try:
        teleop_science()
    except rospy.ROSInterruptException:
        pass
