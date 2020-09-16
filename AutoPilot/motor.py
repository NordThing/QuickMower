import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

Motor1A = 16
Motor1B = 18
Motor1E = 32

GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

pwmMotor1 = GPIO.PWM(Motor1E, 150)

pwmMotor1.start(0)

print "Turning motor on"
GPIO.output(Motor1A,GPIO.HIGH)
GPIO.output(Motor1B,GPIO.LOW)
GPIO.output(Motor1E,GPIO.HIGH)

for dc in range(50, 100, 5):
          pwmMotor1.ChangeDutyCycle(dc)
          sleep(2.50)

sleep(2)

print "Stopping motor"
GPIO.output(Motor1E,GPIO.LOW)

pwmMotor1.stop()

GPIO.cleanup()
