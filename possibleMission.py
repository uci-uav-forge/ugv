import os
import RPi.GPIO as GPIO
import time
import peripherals

import rospy
from std_msgs.msg import Int8


class mission():
    def __init__(self) -> None:
        GPIO.setmode(GPIO.BCM)
        ussTrigPin = 18
        ussEchoPin = 24 
        servoPin = 17
        self.uss = peripherals.uss(ussTrigPin, ussEchoPin)
        self.servo = peripherals.servo(servoPin)

        self.landing = -1 #  -1: in air  0: landing   1: landed

        self.landingPub = rospy.Publisher('landing', Int8, queuesize=5)


    def landedSubCallback(self, data):
        self.landing = data.data

    def run(self):
        landingSub = rospy.Subscriber('landing', Int8, landedSubCallback)
        
        while self.landing == -1:
            pass


        distances = []
        while True:
            x = self.uss.distance()
            print('UGV is {} centimeters from the ground\n'.format(x))
            distances.append(x)
            print(distances)
            if len(distances) > 5: distances.pop(0)
            time.sleep(0.5)
            aveOfDistances = sum(distances) / len(distances)
            if len(distances) == 5 and aveOfDistances < 4:
                self.landingPub.publish(int(-1))
                break


            os.system('rosrun mavros mavsafety arm')
            time.sleep(5)
            os.system('rosrun mavros mavsys mode auto')

if __name__ == '__main__':

    rospy.init_node('ugv_mission_node')
    rate = rospy.Rate(10)
    mission()
    mission.run()