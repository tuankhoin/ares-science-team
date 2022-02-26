**Note: this is intended as a live document and may contain errors**

# Introduction
26/02/22 - The files and directories within this package (teleop_science) are aimed towards testing and operation of the science payload for the Lunar Resources Task. The science payload is controlled by an Arduino which itself receives commands over ROS from a User Interface at the Control Station.

# Terminology
"Science payload" - all electrical and mechanical components directly aimed at collecting regolith for processing. Includes drill + arm that raises/lowers it, container + arm that raises/lowers it, Arduino controller, wiring, distance sensors, moisture probes, and supporting metal frame.

"Regolith" - wet sand to be drilled, stored and processed to extract water.

"I2C" - serial communication bus used for attaching lower speed peripheral integrated circuits to processors/microcontrollers in short-distance, intra-board communication. In the case of this package, I2C is used to connect the distance sensor chips to the Arduino.

# Arduino Files

## serial_echo.ino

Purpose:

*26/02/22 - simply reads input bytes and echos them back as output. Useful as a starting point for those not completely familiar with Arduinos.*

Path:

src\teleop_science\arduino\serial_echo\serial_echo.ino

## Example1_ReadDistance.ino
Purpose:

 *26/02/22 - initial, isolated testing of distance sensors. The S2 distance sensor must start disconnected and then be reconnected so that I2C addresses can be assigned to each sensor separately (otherwise there is no way for the Arduino to distinguish between the 2 sensors in I2C address assignment).*

Path:

src\teleop_science\arduino\Example1_ReadDistance\Example1_ReadDistance.ino

## moisture_probe.ino
Purpose: 

*26/02/22 - Initial testing of code for reading in data from moisture probes. Requires further investigation to incorporate into sci_payload.ino*

Path:

src\teleop_science\arduino\moisture_probe\moisture_probe.ino

## **sci_payload.ino**
Purpose:

*25/02/22 - This is the main Arduino code controlling the operation of sensors and motors on the Science payload. As of time of writing, this code receives commands through the Arduino's serial port as single characters and calls the appropriate function to write to one or more pins, thus turning on or off different motors.*

*Note: the stepper motors controlling the drill and arm's up/down motion will not run when certain limit switches or distance values are reached. This is to avoid the arms detaching from the screw that lifts/lowers it and to stop the arms colliding with each other or the ground.*


Path:

src/teleop_science\arduino\sci_payload\sci_payload.ino

## DCmotor_2steppers_4limitswitches_2servos_test.ino
Purpose:

*25/02/22 - This file can be considered a legacy version of sci_payload.ino. The primary difference between it and sci_payload.ino is that it does not make full use of functional programming or a switch statement to handle various commands.*

Path: 

src/teleop_science\arduino\DCmotor_2steppers_4limitswitches_2servos_test\DCmotor_2steppers_4limitswitches_2servos_test.ino

# Python Files/ROS Nodes

## science.py

Purpose:

*26/02/22 - defines Python class "Science" which encapsulates the specifics of communication with the Arduino including initiating connnection, sending commands, and disconnecting.*

Path:

src\teleop_science\scripts\science.py

## test.py

Purpose:

*26/02/22 - confirms that commands can be sent directly through the serial bus to the Arduino using the Science class.*

Path:

src\teleop_science\scripts\test.py


## teleop_science_sub_node.py

Purpose:

*26/02/22 - subscribes to the ROS topics corresponding to commands for the Arduino, i.e. other nodes can publish to these topics and teleop_science_sub.py will relay them to the Arduino using the Science class*

Path:

src\teleop_science\scripts\teleop_science_sub_node.py

## teleop_science_pub_node.py

Purpose:

*26/02/22 - used for testing. Takes commands as input from the command line and publishes to the corresponding ROS topics.*

Path:

src\teleop_science\scripts\teleop_science_pub_node.py

# Setup

## Installation

26/02/22 - Make sure you have:
* [Oracle VM VirtualBox](https://www.virtualbox.org/wiki/Downloads)
* [Ubuntu 18.04 (bionic) virtual machine](https://linuxhint.com/install_ubuntu_18-04_virtualbox/)
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [Arduino IDE for Linux (in the VM)](https://docs.arduino.cc/software/ide-v1/tutorials/Linux)
* The entire joystick-motor-control catkin workspace (folder which contains teleop_science package)

## Running

26/02/22 - Make sure to:
1. <code> ls /dev </code> to see all the devices attached to the local system. The Arduino serial should look like <code> ttyACMx </code> where x = the rest of the serial's name
2. <code> chmod 666 /dev/ttyACMx </code> to enable communication with the Arduino
3. Upload the sketch via the Arduino IDE
4. <code> roscore </code> to begin the ROS Master
5. <code> catkin_make </code> to build the packagew
6. <code> source devel/setup.sh </code> (in every terminal, from the root of the catkin workspace)
7. <code> sudo chmod +x \<name of node> </code> for teleop_science_sub.py and teleop_science_pub.py
8. <code> rosrun teleop_science \<name of node> </code> for teleop_science_sub.py and teleop_science_pub.py
9. Follow the instructions in sci_payload.ino for setting up the distance sensors (requires sending a command acknowledgin when S2 has been reconnected).

## 

# To Do

*25/02/22 -*

* *Troubleshoot the sensors. Sensor 2 appears to be malfunctioning*
* *Finalise the ROS topics required for the UI*
* *Ensure comms is working including the Jetson + Rocket M5 setup which allows operation of ROS nodes to send commands and receive sensor data*
* *Incorporate the moisture probes into sci_payload.ino*


# Contributors

Joshua Newland (Software) - jtnewland@student.unimelb.edu.au

Ernest Cheong (Science)

Zac Tu (Software)

Khoi Tuan Nguyen (Science)