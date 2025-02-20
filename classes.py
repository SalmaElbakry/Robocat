import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
import RPi.GPIO as GPIO
class Robot:# Initialise the PCA9685 using the default address (0x40).
	def __init__(self):
		self.move_speed = 0x7FFF  # half of Max pulse length out of 0xFFFF
		self.LEFT_BACK = 23  #Left motor direction pin
		self.LEFT_FRONT = 24  #Left motor direction pin
		self.RIGHT_BACK = 27  #Right motor direction pin
		self.RIGHT_FRONT = 22  #Right motor direction pin
		self.i2c_bus = busio.I2C(SCL, SDA)
		self.pwm =  PCA9685(self.i2c_bus)
		self.pwm.frequency = 60

		GPIO.setup(self.LEFT_BACK, GPIO.OUT)   
		GPIO.setup(self.LEFT_FRONT, GPIO.OUT) 
		GPIO.setup(self.RIGHT_BACK, GPIO.OUT)   
		GPIO.setup(self.RIGHT_FRONT, GPIO.OUT) 

		self.ENA = 0  #Left motor speed PCA9685 port 0
		self.ENB = 1  #Right motor speed PCA9685 port 1

	# Set frequency to 60hz, good for servos.
	
	GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
	GPIO.setwarnings(False)
	#define L298N(Model-Pi motor drive board) GPIO pins

	def changespeed(self, leftSpeed, rightSpeed):
		self.pwm.channels[self.ENA].duty_cycle = leftSpeed
		self.pwm.channels[self.ENB].duty_cycle = rightSpeed
		# self.pwm.channels[self.ENA].duty_cycle = rightSpeed
		# self.pwm.channels[self.ENB].duty_cycle = leftSpeed

	def stopcar(self):
		GPIO.output(self.LEFT_BACK, GPIO.LOW)
		GPIO.output(self.LEFT_FRONT, GPIO.LOW)
		GPIO.output(self.RIGHT_BACK, GPIO.LOW)
		GPIO.output(self.RIGHT_FRONT, GPIO.LOW)
		self.changespeed(0, 0)

	def backward(self):
		GPIO.output(IN1, GPIO.HIGH)
		GPIO.output(IN2, GPIO.LOW)
		GPIO.output(IN3, GPIO.HIGH)
		GPIO.output(IN4, GPIO.LOW)
		#changespeed(move_speed)
		
	def forward(self):
		GPIO.output(self.LEFT_BACK, GPIO.LOW)
		GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
		GPIO.output(self.RIGHT_BACK, GPIO.LOW)
		GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)
		#self.changespeed(self.move_speed)
		#time.sleep(0.75)

	def turnRight(self):
		GPIO.output(self.LEFT_BACK, GPIO.LOW)
		GPIO.output(self.LEFT_FRONT, GPIO.HIGH)
		GPIO.output(self.RIGHT_BACK, GPIO.HIGH)
		GPIO.output(self.RIGHT_FRONT, GPIO.LOW)
		#self.changespeed(self.move_speed)
		#time.sleep(0.35)
		
	def turnLeft(self):
		GPIO.output(self.LEFT_BACK, GPIO.HIGH)
		GPIO.output(self.LEFT_FRONT, GPIO.LOW)
		GPIO.output(self.RIGHT_BACK, GPIO.LOW)
		GPIO.output(self.RIGHT_FRONT, GPIO.HIGH)
		#self.changespeed(self.move_speed)
		#time.sleep(0.35)

# PID Controller class
class PIDController:
	def __init__(self, Kp, Ki, Kd):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.prev_error = 0
		self.integral = 0

	def update(self, error, dt=0.033):
		# Update the integral term with anti-windup
		self.integral += error * dt
		max_integral = 100  # Adjust this based on your system's needs
		self.integral = max(min(self.integral, max_integral), -max_integral)
		print("Integral: ", self.integral)
		# Calculate the derivative term
		derivative = (error - self.prev_error) / dt

		# Compute the PID output
		output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

		# Update the previous error
		self.prev_error = error

		# Optional: Debugging output
		# print(f"Error: {error}, Integral: {self.integral}, Derivative: {derivative}, Output: {output}")

		return output

	