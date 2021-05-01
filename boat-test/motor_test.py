import pigpio
from time import time

pi = pigpio.pi()

start = time()
pi.set_servo_pulsewidth(12, 1500)

while time() - start < 5:
    pass


start = time()
pi.set_servo_pulsewidth(12, 1600)

while time() - start < 5:
    pass

pi.set_servo_pulsewidth(12, 0)
