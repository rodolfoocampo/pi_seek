import RPi.GPIO as GPIO
import time

def abre():
	GPIO.setmode(GPIO.BOARD)

	GPIO.setup(3,GPIO.OUT)

	p = GPIO.PWM(3,50)
	p.start(2.5)

	#p.ChangeDutyCycle(4)
	#time.sleep(5
	 # turn towards 90 degree
	time.sleep(1) # sleep 1 second
	 # turn towards 0 degree
	time.sleep(1) # sleep 1 second
	p.ChangeDutyCycle(12.5) # turn towards 180 degree
        time.sleep(1) # sleep 1 second


	p.stop()
	GPIO.cleanup()
