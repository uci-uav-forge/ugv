import RPi.GPIO as GPIO
import time
import peripherals
from src.mavros.mavros.px4.mavros_test_common import MavrosTestCommon
from pymavlink import mavutil

# pin setup
GPIO.setmode(GPIO.BCM)

ussTrigPin = 18
ussEchoPin = 24 
servoPin = 17

uss = peripherals.uss(ussTrigPin, ussEchoPin)
servo = peripherals.servo(servoPin)

# setup up connection to pixhawk
ugv = MavrosTestCommon()
ugv.setUp() # setups mavros services and subscirbers 
ugv.set_arm(True, 5)
ugv.set_mode("OFFBOARD", 5)

# wait till aircraft says it is dropping ugv
time.sleep(5)
print('UGV NOW DROPPING')

# loop till uss sense that it is close to ground
# distance in centimeters, needs to be test with vechile to determine value while on floor
while uss.distance() > 4:
    time.sleep(0.5)
    pass

# move servo to cut wire
servo.moveTo(180)
print('UGV DISCONNECTED FROM UAV')

# tell uav or ground station that we disconnected and they can continue mission
time.sleep(2.5)
print('UGV WILL NOW DO MISSION')



ugv.set_arm(False, 5)

GPIO.cleanup()