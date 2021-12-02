import RPi.GPIO as GPIO
import time

#ultrasonic sensor
class uss:
    def __init__(self, trigpin, echopin):
        self.trig = trigpin
        self.echo = echopin

        print(self.trig)
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

        x = (self._start - self._stop) * 17150
        print('UGV is {} centimeters from the ground'.format(x))
        return x

class servo:
    def __init__(self, pin):
        self._pin = pin
        GPIO.setup(self._pin, GPIO.OUT)
        self._servo = GPIO.PWM(self._pin,50)
        self._servo.start(0)

    def moveTo(self, angle):
        # angle in degrees, duty cycle 2-12 (0-180)
        self._servo.ChangeDutyCycle(2+angle/18)
        time.sleep(0.5)
        self._servo.ChangeDutyCycle(0)

    def stop(self):
        self._servo.stop()


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)

    testuss = uss(18,24)
    for i in range(10):
        print(testuss.distance())
        time.sleep(1)

    testservo = servo(17)
    for i in range(0, 181, 30):
        testservo.moveTo(i)
        time.sleep(1)
    testservo.stop()

    GPIO.cleanup()
