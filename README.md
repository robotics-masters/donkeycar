# donkeycar: a python self driving library
### for RoboHAT MM1

[![Build Status](https://travis-ci.org/autorope/donkeycar.svg?branch=dev)](https://travis-ci.org/autorope/donkeycar)
[![CodeCov](https://codecov.io/gh/autoropoe/donkeycar/branch/dev/graph/badge.svg)](https://codecov.io/gh/autorope/donkeycar/branch/dev)
[![PyPI version](https://badge.fury.io/py/donkeycar.svg)](https://badge.fury.io/py/donkeycar)
[![Py versions](https://img.shields.io/pypi/pyversions/donkeycar.svg)](https://img.shields.io/pypi/pyversions/donkeycar.svg)

Donkeycar is minimalist and modular self driving library for Python. It is
developed for hobbyists and students with a focus on allowing fast experimentation and easy
community contributions.

#### Quick Links
* [Donkeycar Updates & Examples](http://donkeycar.com)
* [Build instructions and Software documentation](http://docs.donkeycar.com)
* [Slack / Chat](https://donkey-slackin.herokuapp.com/)

![donkeycar](./docs/assets/build_hardware/donkey2.PNG)

#### Getting going for RoboHAT MM1
1. Follow the [documentation for Donkey Car](http://docs.donkeycar.com/guide/robot_sbc/setup_raspberry_pi/) but use this respository on the "peter" branch at step 11.

##### Step 11
```
git clone https://github.com/robotics-masters/donkeycar
cd donkeycar
git checkout peter
pip install -e .[pi]
pip install tensorflow==1.14.0
```

2. You have to install the software onto the Robo HAT MM1 as well (CircuitPython).  This can be done by attaching the Robo HAT MM1 to your computer's USB port and adding the below code to the code.py file.

```
# Donkey Car Driver for Robotics Masters Robo HAT MM1
#
# Notes:
#   This is to be run using CircuitPython 5.0
#   Date: 15/05/2019
#   Updated: 02/12/2019
#
#

import time
import board
import busio

from digitalio import DigitalInOut, Direction
from pulseio import PWMOut, PulseIn, PulseOut

## Customisation these variables
SMOOTHING_INTERVAL_IN_S = 0.025
DEBUG = False
ACCEL_RATE = 10

## functions
def servo_duty_cycle(pulse_ms, frequency = 60):
	period_ms = 1.0 / frequency * 1000.0
	duty_cycle = int(pulse_ms / 1000 / (period_ms / 65535.0))
	return duty_cycle

def state_changed(control):
        prev = control.value
	control.channel.pause()
	for i in range(0, len(control.channel)):
		val = control.channel[i]
                # prevent ranges outside of control space
		if(val < 1000 or val > 2000):
			continue
		# set new value
		control.value = (control.value + val) / 2

	if DEBUG:
		print("%f\t%s (%i): %i (%i)" % (time.monotonic(), control.name, len(control.channel), control.value, servo_duty_cycle(control.value)))
	control.channel.clear()
	control.channel.resume()

def state_changed_throttle(control):
        prev = control.value
	control.channel.pause()
	for i in range(0, len(control.channel)):
		#val = control.channel[i]
                # prevent ranges outside of control space
		if(val < 1000 or val > 2000):
			continue
		# cap maximum acceleration to prevent stall
		#if (val - prev) > ACCEL_RATE:
                #        val = (control.value + ACCEL_RATE)
                # set new value
		control.value = (control.value + val) / 2

	if DEBUG:
		print("%f\t%s (%i): %i (%i)" % (time.monotonic(), control.name, len(control.channel), control.value, servo_duty_cycle(control.value)))
	control.channel.clear()
	control.channel.resume()

class Control:
     def __init__(self, name, servo, channel, value):
	self.name = name
	self.servo = servo
	self.channel = channel
	self.value = value
	self.servo.duty_cycle = servo_duty_cycle(value)

## set up on-board LED
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT

## set up serial UART
# note UART(TX, RX, baudrate)
uart = busio.UART(board.TX1, board.RX1, baudrate = 115200, timeout = 0.001)

## set up servos and radio control channels
steering_pwm = PWMOut(board.SERVO2, duty_cycle = 2 ** 15, frequency = 60)
throttle_pwm = PWMOut(board.SERVO1, duty_cycle = 2 ** 15, frequency = 60)

steering_channel = PulseIn(board.RCC4, maxlen=64, idle_state=0)
throttle_channel = PulseIn(board.RCC3, maxlen=64, idle_state=0)

steering = Control("Steering", steering_pwm, steering_channel, 1500)
throttle = Control("Throttle", throttle_pwm, throttle_channel, 1500)

## Hardware Notification: starting
print("preparing to start...")
for i in range(0, 2):
	led.value = True
	time.sleep(0.5)
	led.value = False
	time.sleep(0.5)

last_update = time.monotonic()

# GOTO: main()

def main():
	global last_update
	
	data = bytearray('')
	datastr = ''
	last_input = 0
	steering_val = steering.value
	throttle_val = throttle.value

	while True:
		if(last_update + SMOOTHING_INTERVAL_IN_S > time.monotonic()):
			continue
		last_update = time.monotonic()

		if(len(throttle.channel) != 0):
			#state_changed_throttle(throttle)
			state_changed(throttle)

		if(len(steering.channel) != 0):
			state_changed(steering)

		if(DEBUG):
			print("Get: %i, %i" % (int(steering.value), int(throttle.value)))
		uart.write(b"%i, %i\r\n" % (int(steering.value), int(throttle.value)))
		while True:
			byte = uart.read(1)
			if(byte == None):
				break
			last_input = time.monotonic()
			if(DEBUG):
				print("Read from UART: %s" % (byte))
			if(byte == b'\r'):
				data = bytearray('')
				datastr = ''
				break
			data[len(data):len(data)] = byte
			datastr = ''.join([chr(c) for c in data]).strip() # convert bytearray to string
		if(len(datastr) >= 10):
			steering_val = steering.value
			throttle_val = throttle.value
			try:
				steering_val = int(datastr[:4])
				throttle_val = int(datastr[-4:])
			except ValueError:
				None
				
			data=bytearray('')
			datastr = ''
			last_input = time.monotonic()
			if(DEBUG):
				print("Set: %i, %i" % (steering_val, throttle_val))

		if(last_input + 10 < time.monotonic()):
			steering.servo.duty_cycle = servo_duty_cycle(steering.value)
			throttle.servo.duty_cycle = servo_duty_cycle(throttle.value)
		else:
			steering.servo.duty_cycle = servo_duty_cycle(steering_val)
			throttle.servo.duty_cycle = servo_duty_cycle(throttle_val)


## Run
print("Run!")
main()

```

3.  To drive the Donkey Car (after completing all the instructions):
```
python manage.py drive --js
```

#### Use Donkey if you want to:
* Make an RC car drive its self.
* Compete in self driving races like [DIY Robocars](http://diyrobocars.com)
* Experiment with autopilots, mapping computer vision and neural networks.
* Log sensor data. (images, user inputs, sensor readings)
* Drive your car via a web or game controller.
* Leverage community contributed driving data.
* Use existing CAD models for design upgrades.

### Get driving.
After building a Donkey2 you can turn on your car and go to http://localhost:8887 to drive.

### Modify your cars behavior.
The donkey car is controlled by running a sequence of events

```python
#Define a vehicle to take and record pictures 10 times per second.

import time
from donkeycar import Vehicle
from donkeycar.parts.cv import CvCam
from donkeycar.parts.datastore import TubWriter
V = Vehicle()

IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3

#Add a camera part
cam = CvCam(image_w=IMAGE_W, image_h=IMAGE_H, image_d=IMAGE_DEPTH)
V.add(cam, outputs=['image'], threaded=True)

#warmup camera
while cam.run() is None:
    time.sleep(1)

#add tub part to record images
tub = TubWriter(path='./dat',
          inputs=['image'],
          types=['image_array'])
V.add(tub, inputs=['image'], outputs=['num_records'])

#start the drive loop at 10 Hz
V.start(rate_hz=10)
```

See [home page](http://donkeycar.com), [docs](http://docs.donkeycar.com)
or join the [Slack channel](http://www.donkeycar.com/community.html) to learn more.
