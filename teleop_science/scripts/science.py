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
            self.send('a')
            response = self.recv(1)
            retry_count += 1
        return response == 'a'

    # move arm up
    def arm_up(self):
        print("arm up")
        self.send('w')

    # move arm down
    def arm_down(self):
        print("arm down")
        self.send('s')

    def arm_off(self):
        print("arm off")
        self.send('e')
    
    # move drill up
    def drill_up(self):
        print("drill up")
        self.send('r')

    # move drill down
    def drill_down(self):
        print("drill down")
        self.send('f')

    # stop drill translating
    def drill_off(self):
        print("drill off (translation)")
        self.send('t')

    # start drilling
    def drill_toggle(self):
        print("toggle drill")
        self.send('d')

    # open container
    def container_open(self):
        print("open container")
        self.send('o')

    # close container
    def container_close(self):
        print("close container")
        self.send('c')

    # start everything
    def start(self):
        print("start")
        self.send('p')

    # stop everything
    def stop(self):
        print("stop")
        self.send('q')

    def dist_calib(self):
        print("calibrating")
        self.send('l')