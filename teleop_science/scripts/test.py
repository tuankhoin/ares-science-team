from science import Science
import time

s = Science()
s.connect('COM10')

print("Establishing connection with Arduino.")
resp = s.ready()

if resp:
    print("Device ready.")
else:
    print("Device timed out.")
    exit()

print("Move arm up")
s.arm_up()
print(s.recv(1))
time.sleep(1)

print("Move arm down")
s.arm_down()
print(s.recv(1))
time.sleep(1)

print("Stop everything")
s.stop()
print(s.recv(1))
time.sleep(1)

print("Start drilling")
s.drill()
print(s.recv(1))

print("Disconnecting from Arduino")
s.disconnect()

print("Exit")