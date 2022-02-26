### this is to test the functionality of the ugv to make sure it can move, will not use ros or mavros
### uses l298n motor driver


import RPi.GPIO as GPIO
import time
import peripherals


# pin setup
GPIO.setmode(GPIO.BCM)

ussTrigPin = 18
ussEchoPin = 24 
servoPin = 17
uss = peripherals.uss(ussTrigPin, ussEchoPin)
servo = peripherals.servo(servoPin)


enA = 12
in1 = 22
in2 = 10
in3 = 5
in4 = 6
enB = 13

lmotor = peripherals.motor(enA, in1, in2)
rmotor = peripherals.motor(enB, in3, in4)

# wait till aircraft says it is dropping ugv
time.sleep(5)
print('UGV NOW DROPPING')

# loop till uss sense that it is close to ground
# distance in centimeters, needs to be test with vechile to determine value while on floor
while True:
    x= uss.distance()
    print('UGV is {} centimeters from the ground\n'.format(x))
    time.sleep(0.5)
    if x < 2:
        break

# move servo to cut wire
servo.moveTo(180)
time.sleep(0.5)
servo.moveTo(0)
print('UGV DISCONNECTED FROM UAV')

# tell uav or ground station that we disconnected and they can continue mission
time.sleep(2.5)
print('UGV WILL NOW DO MISSION')


lmotor.speed(50)
rmotor.speed(50)

lmotor.forward()
rmotor.forward()
time.sleep(3)
lmotor.backward()
rmotor.backward()
time.sleep(3)
lmotor.forward()
rmotor.backward()
time.sleep(3)

GPIO.cleanup()