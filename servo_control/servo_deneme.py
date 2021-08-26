import servo_control as sc
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) #GPIO 17 pini 50Hz ile pwm olarak ayarlandı
p.start(2.5) # Initialization 20ms

sc.servo_go(p, 180)
