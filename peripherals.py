import RPi.GPIO as GPIO
import time

#ultrasonic sensor
class uss:
    def __init__(self, trigport, echoport) -> None:
        self.trig = trigport
        self.echo = echoport

        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.output(self.trig, 0)
        GPIO.setup(self.echo, GPIO.IN)

    def distance(self):
        GPIO.output(self.trig, 1)
        time.sleep(0.00001)
        GPIO.output(self.trig, 0)

        while GPIO.input(self.echo) == 0:
            self.start = time.time()
        # save time of arrival
        while GPIO.input(self.echo) == 1:
            self.stop = time.time()

        x = (self.start - self.stop) * 17150
        print(x)
        return x

class servo:
    def __init__(self, pin) -> None:
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(11,50)
        self.servo.start(0)

    def moveTo(self, angle):
        # angle in degrees, duty cycle 2-12 (0-180)
        self.servo.ChangeDutyCycle(2+angle/18)
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0)


if __name__ == '__main__':
    test = uss(18,24)
    for i in range(10):
        print(test.distance())
        time.sleep(1)
            