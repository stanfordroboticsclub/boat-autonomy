import pigpio
from time import time

pi = pigpio.pi()

try:
    start = time()
    pi.set_servo_pulsewidth(12, 1000)

    while time() - start < 5:
        pass

    start = time()
    pi.set_servo_pulsewidth(12, 1150)

    while time() - start < 15:
        pass

finally:
    pi.set_servo_pulsewidth(12, 0)
