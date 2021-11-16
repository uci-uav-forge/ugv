import RPi.GPIO as GPIO
import time

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

if __name__ == '__main__':
    test = uss(18,24)
    for i in range(10):
        print(test.distance())
        time.sleep(1)
            