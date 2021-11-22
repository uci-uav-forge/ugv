import RPi.GPIO as GPIO
import time
import peripherals
import ugv_functions as ugv
from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from pymavlink import mavutil

# pin setup
GPIO.setmode(GPIO.BCM)

ussTrigPin = 18
ussEchoPin = 24 
servoPin = 17

uss = peripherals.uss(ussTrigPin, ussEchoPin)
servo = peripherals.servo(servoPin)

# setup up connection to pixhawk
vehicle = ugv.connectMyCopter()
ugv.paramSetup(vehicle)
ugv.arm(vehicle)


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



GPIO.cleanup()