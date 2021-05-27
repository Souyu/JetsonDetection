#Prototyping MQ2-Gas Module.
#  and pin 7 for signal(board numbering!). 
 
import time, sys
import RPi.GPIO as GPIO
 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.IN)
 
def action(pin):
    print('Sensor detected gas!')
    return
 
# if detected gas, it should provide a rising edge and print that it detected gas
GPIO.add_event_detect(7, GPIO.RISING)
GPIO.add_event_callback(7, action)
 
try:
    while True:
        print('alive')
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
    sys.exit()

