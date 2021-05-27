import time


from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
print("Servo Kit has finished Initializing!")

#Servo 1 Test
kit.servo[0].angle = 180
time.sleep(5)
print("Turning Servo 0 back!")
kit.servo[0].angle = 0

#Servo 2 Test
kit.servo[1].angle = 180
time.sleep(5)
print("Turning Servo 1 back!")
kit.servo[1].angle = 0