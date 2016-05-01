from MPU6050 import MPU6050
import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,GPIO.HIGH)
time.sleep(1)
GPIO.output(18,GPIO.LOW)
time.sleep(1)
GPIO.output(18,GPIO.HIGH)
time.sleep(1)
GPIO.output(18,GPIO.LOW)
time.sleep(1)
GPIO.output(18,GPIO.HIGH)
GPIO.output(18,GPIO.LOW)
print "done"
