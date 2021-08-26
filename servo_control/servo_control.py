#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|-||o||x|~~~#
#  DEHA SERVO CONTROL SCRIPT                                       #
#                                                                  #
#  SG90 tipi (mikro servo) servolar için yazılmıştır               #
#  Başka model Servo motorlarda pwm değerleri farklı olacaktır     #
#                                                                  #
#  servo_go Fonksiyonu servoyu istenen açı konumuna götürür.       #
#  servo_go(servo , angle)                                         #
#      servo : pwm alınan GPIO pini                                #
#      angle : servodan gidilmesi istenen açı değeri               #
#                                                                  #
#                                                 version : 1.1    #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
import RPi.GPIO as GPIO
import time

def get_pwm(angle):
    return (angle / 18) + 2.5

def servo_go(servo , angle):
    servo.ChangeDutyCycle(get_pwm(angle))

GPIO.setwarnings(False)

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) #GPIO 17 pini 50Hz ile pwm olarak ayarlandı
p.start(2.5) # Initialization 20ms

#Kramerli tasarım:
# Doldurma Süresi : 4sn
# Boşaltma Süresi : 8.3sn

try:
    while True:
        servo_go(p , 45)#açık
        time.sleep(1.5)
        #servo_go(p , 100)#kapalı
        #time.sleep(1.5)
        #servo_go(p , 180)
        break
        
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
        
