import RPi.GPIO as GPIO
import time

#ultrasonic sensor
class uss:
    def __init__(self, trigpin, echopin):
        self.trig = trigpin
        self.echo = echopin

        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.output(self.trig, 0)
        GPIO.setup(self.echo, GPIO.IN)

    def distance(self):
        GPIO.output(self.trig, 1)
        time.sleep(0.00001)
        GPIO.output(self.trig, 0)

        while GPIO.input(self.echo) == 0:
            self._start = time.time()
        # save time of arrival
        while GPIO.input(self.echo) == 1:
            self._stop = time.time()

        x = (self._stop - self._start) * 17150
        return x

class servo:
    def __init__(self, pin):
        self._pin = pin
        GPIO.setup(self._pin, GPIO.OUT)
        self._servo = GPIO.PWM(self._pin,50)
        self._servo.start(0)
        self.moveTo(0)

    def moveTo(self, angle):
        # angle in degrees, duty cycle 2-12 (0-180)
        self._servo.ChangeDutyCycle(2+angle/18)
        time.sleep(0.5)
        self._servo.ChangeDutyCycle(0)

    def stop(self):
        self._servo.stop()

class motor:
    ### use with l298n motor driver
    def __init__(self, en, in1, in2) -> None:
        self.en = en
        self.in1 = in1
        self.in2 = in2

        GPIO.setup(self.en,GPIO.OUT)
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)

        self.p = GPIO.PWM(en,1000)
        self.p.start(0)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
    
    def forward(self):
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)
    def backward(self):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)
    def stop(self):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
    def speed(self, s):
        ### enter 25/50/75
        self.p.ChangeDutyCycle(s)



if __name__ == '__main__':
    
    GPIO.setmode(GPIO.BCM)

    testuss = uss(18,24)
    print('uhh')
    for i in range(10):
        print(testuss.distance())
        time.sleep(1)
    print('poggers')

    testservo = servo(17)
    for i in range(0, 181, 30):
        testservo.moveTo(i)
        time.sleep(1)
    testservo.stop()

    GPIO.cleanup()
