from serial import Serial
import time

class Science:
    def __init__(self):
        self.serial = None

    def connect(self, com_port, baud=9600, timeout=1):
        self.serial = Serial(com_port, baudrate=baud, timeout=timeout)
    
    def disconnect(self):
        self.serial.close()
        self.serial = None

    # send a command
    def send(self, command_byte):
        self.serial.write(bytearray([command_byte]))
    
    # receive a command
    def recv(self, len):
        return self.serial.read(len)

    # waiting for arduino to be ready
    def ready(self, max_retry=10):
        retry_count = 0
        response = None
        
        while not response and retry_count < max_retry:
            self.send('p')
            response = self.recv(1)
            retry_count += 1
        return response == 'p'
        
    # Send a byte to register that the switch has been flicked
    def distance_calibration(self):
        print("calibrating")
        # Can be any byte (doesn't interfere with other commands regardless)
        # Chose 'l' at random
        self.send('l')


    # start accepting inputs
    def start(self):
        print("start")
        self.send('z')


    # move container arm up
    def container_up(self):
        print("container arm up")
        self.send('q')

    # move container arm down
    def container_down(self):
        print("container arm down")
        self.send('a')

    # stop container arm
    def container_stop(self):
        print("stop container arm")
        self.send('w')
    

    # move drill arm up
    def drill_up(self):
        print("drill arm up")
        self.send('e')

    # move drill arm down
    def drill_down(self):
        print("drill arm down")
        self.send('d')

    # stop drill arm (translating)
    def drill_stop(self):
        print("stop drill arm")
        self.send('r')


    # start drilling clockwise
    def drill_cw(self):
        print("drill clockwise")
        self.send('t')

    # start drilling counter-clockwise
    def drill_ccw(self):
        print("drill counter-clockwise")
        self.send('g')

    # stop drilling
    def drill_off(self):
        print("drill off")
        self.send('y')


    # open container
    def container_open(self):
        print("open container")
        self.send('u')

    # close container
    def container_close(self):
        print("close container")
        self.send('j')


    # angle moisture probe up
    def probe_up(self):
        print("moisture probe up")
        self.send('i')

    # angle moisture probe down
    def probe_down(self):
        print("moisture probe down")
        self.send('k')


    # stop all motors
    def stop_motors(self):
        print("stop motors")
        self.send('x')
    
    # kill switch
    def kill(self):
        print("kill")
        self.send('c')

        # echo ensures motors are stopped before disconnecting from Arduino
        if(self.recv(1) == 'c'):
            self.disconnect()
        else:
            print("kill command not echoed!")

    