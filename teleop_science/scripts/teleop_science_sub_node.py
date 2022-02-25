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


def arm_up(data):
    sci.arm_up()

def arm_down(data):
    sci.arm_down()

def arm_off(data):
    sci.arm_off()

def drill_up(data):
    sci.drill_up()

def drill_down(data):
    sci.drill_down()

def drill_off(data):
    sci.drill_off()

def drill_toggle(data):
    sci.drill_toggle()

def container_open(data):
    sci.container_open()

def container_close(data):
    sci.container_close()

def start(data):
    sci.start()

def stop(data):
    sci.stop()

def dist_calib(data):
    sci.dist_calib()




def teleop_science():
    rospy.init_node('teleop_science_sub', anonymous=True)


    rospy.Subscriber("teleop_science/arm_up", Empty, arm_up)
    rospy.Subscriber("teleop_science/arm_down", Empty, arm_down)
    rospy.Subscriber("teleop_science/arm_off", Empty, arm_off)
    
    rospy.Subscriber("teleop_science/drill_up", Empty, drill_up)
    rospy.Subscriber("teleop_science/drill_down", Empty, drill_down)
    rospy.Subscriber("teleop_science/drill_off", Empty, drill_off)

    rospy.Subscriber("teleop_science/drill_toggle", Empty, drill_toggle)

    rospy.Subscriber("teleop_science/container_open", Empty, container_open)
    rospy.Subscriber("teleop_science/container_close", Empty, container_close)

    rospy.Subscriber("teleop_science/start", Empty, start)
    rospy.Subscriber("teleop_science/stop", Empty, stop)

    rospy.Subscriber("teleop_science/distance_sensor_calibration", Empty, dist_calib)

    print("ROS ready.")
    rospy.spin()


if __name__ == '__main__':
    try:
        teleop_science()
    except rospy.ROSInterruptException:
        pass
