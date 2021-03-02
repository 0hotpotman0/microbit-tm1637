"""
MicroPython for micro:bit TM1637 quad 7-segment LED display driver
https://github.com/mcauser/microbit-tm1637

MIT License
Copyright (c) 2017 Mike Causer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from microbit import sleep

_SEG = bytearray(b'\x3F\x06\x5B\x4F\x66\x6D\x7D\x07\x7F\x6F\x77\x7C\x39\x5E\x79\x71\x3D\x76\x06\x1E\x76\x38\x55\x54\x3F\x73\x67\x50\x6D\x78\x3E\x1C\x2A\x76\x6E\x5B\x00\x40\x63')

class TM1637(object):
	def __init__(self, clk, dio, brightness=7):
		self._c = clk
		self._d = dio
		self._b = max(0, min(brightness, 7))
		self._data_cmd()
		self._dsp_ctrl()

	def _start(self):
		self._d.write_digital(0)
		self._c.write_digital(0)

	def _stop(self):
		self._d.write_digital(0)
		self._c.write_digital(1)
		self._d.write_digital(1)

	def _data_cmd(self):
		self._start()
		self._write_byte(0x40)
		self._stop()

	def _dsp_ctrl(self):
		self._start()
		self._write_byte(0x88 | self._b)
		self._stop()

	def _write_byte(self, b):
		for i in range(8):
			self._d.write_digital((b >> i) & 1)
			self._c.write_digital(1)
			self._c.write_digital(0)
		self._c.write_digital(0)
		self._c.write_digital(1)
		self._c.write_digital(0)

	def brightness(self, val=None):
		if val is None:
			return self._b
		self._b = max(0, min(val, 7))
		self._data_cmd()
		self._dsp_ctrl()

	def write(self, segments, pos=0):
		if not 0 <= pos <= 3:
			raise ValueError("Position out of range")
		self._data_cmd()
		self._start()
		self._write_byte(0xC0 | pos)
		for seg in segments:
			self._write_byte(seg)
		self._stop()
		self._dsp_ctrl()

	def encode_string(self, string):
		segments = bytearray(len(string))
		for i in range(len(string)):
			segments[i] = self.encode_char(string[i])
		return segments

	def encode_char(self, char):
		o = ord(char)
		if o == 32:
			return _SEG[36] # space
		if o == 42:
			return _SEG[38] # star/degrees
		if o == 45:
			return _SEG[37] # dash
		if o >= 65 and o <= 90:
			return _SEG[o-55] # uppercase A-Z
		if o >= 97 and o <= 122:
			return _SEG[o-87] # lowercase a-z
		if o >= 48 and o <= 57:
			return _SEG[o-48] # 0-9
		raise ValueError("Character out of range: {:d} '{:s}'".format(o, chr(o)))

	def hex(self, val):
		string = '{:04x}'.format(val & 0xffff)
		self.write(self.encode_string(string))

	def number(self, num):
		num = max(-999, min(num, 9999))
		string = '{0: >4d}'.format(num)
		self.write(self.encode_string(string))

	def number_0(self, num):
		num = max(0, min(num, 9))
		string = '{0: >4d}'.format(num)
		#self.write([0,0,0,0])
		self.write(self.encode_string(string))
		
	def number_1(self, num):
		num = max(0, min(num, 9))
		string = '{0: >3d}'.format(num)
		#self.write([0,0,0,0])
		self.write(self.encode_string(string))

	def number_2(self, num):
		num = max(0, min(num, 9))
		string = '{0: >2d}'.format(num)
		#self.write([0,0,0,0])
		self.write(self.encode_string(string))

	def number_3(self, num):
		num = max(0, min(num, 9))
		string = '{0: >1d}'.format(num)
		#self.write([0,0,0,0])
		self.write(self.encode_string(string))
    

	def numbers_T(self, num1, num2, colon=True):
		num1 = max(-9, min(num1, 99))
		num2 = max(-9, min(num2, 99))
		segments = self.encode_string('{0:0>2d}{1:0>2d}'.format(num1, num2))
		if colon:
			segments[1] |= 0x80 # colon on
		self.write(segments)

	def numbers_F(self, num1, num2, colon=False):
		num1 = max(-9, min(num1, 99))
		num2 = max(-9, min(num2, 99))
		segments = self.encode_string('{0:0>2d}{1:0>2d}'.format(num1, num2))
		if colon:
			segments[1] |= 0x80 # colon off
		self.write(segments)


	
	def temperature(self, num):
		if num < -9:
			self.write([0x38, 0x3F]) # LO
		elif num > 99:
			self.write([0x76, 0x06]) # HI
		else:
			string = '{0: >2d}'.format(num)
			self.write(self.encode_string(string))
		self.write([_SEG[38], _SEG[12]], 2) # degrees C

	def show(self, string, colon=False):
		segments = self.encode_string(string)
		if len(segments) > 1 and colon:
			segments[1] |= 128
		self.write(segments[:4])

	def scroll(self, string, delay=250):
		segments = string if isinstance(string, list) else self.encode_string(string)
		data = [0] * 8
		data[4:0] = list(segments)
		for i in range(len(segments) + 5):
			self.write(data[0+i:4+i])
			sleep(delay)


#############################################################
#  From GroveOne.py
#############################################################


"""
I2C init  freq scl sda
"""
def i2c_init(freq):
    i2c.init(100000, pin19, pin20)

"""
Read one number to a 7-bit address
read n bytes from device with addr; repeat=True means a stop bit won't
be sent.
"""
def read_i2c(addr, NumberFormat, bool):
    return i2c.read(addr, NumberFormat, bool)


"""
# write buf to device with addr; repeat=True means a stop bit won't be sent.
"""
def wirte_i2c(addr, value, NumberFormat, bool):
    i2c.write(addr, value, NumberFormat, bool)
    
def measureInCentimeters(groveport, Unit):
    duration = 0
    distance = 0
    distanceBackup = 0
    pin = getattr(microbit, 'pin{}'.format(groveport))
    pin.write_digital(0)
    utime.sleep_us(2)
    pin.write_digital(1)
    utime.sleep_us(10)
    pin.write_digital(0)
    pin.read_digital()
    duration = machine.time_pulse_us(pin, 1, 10500)
    if (Unit == 1):
        distance = duration * 153 / 58 / 100
    else:
        distance = duration * 153 / 148 / 100
    if (distance > 0):
        distanceBackup = distance
    else:
        distance = distanceBackup
    sleep(50)

    return round(distance, 1)


def minifan(analogport, speed):
    pin = getattr(microbit, 'pin{}'.format(analogport))
    pin.write_analog((speed/(100-0)*(1023-0) + 0))


class Servo:
    def __init__(self, pin, freq=50, min_us=600, max_us=2400, angle=180):
        self.min_us = min_us
        self.max_us = max_us
        self.us = 0
        self.freq = freq
        self.angle = angle
        self.analog_period = 0
        pin = getattr(microbit, 'pin{}'.format(pin))
        self.pin = pin
        analog_period = round((1/self.freq) * 1000)  # hertz to miliseconds
        self.pin.set_analog_period(analog_period)

    def write_us(self, us):
        us = min(self.max_us, max(self.min_us, us))
        duty = round(us * 1024 * self.freq // 1000000)
        self.pin.write_analog(duty)
        # self.pin.write_digital(0)  # turn the pin off

    def write_angle(self, degrees=None):
        degrees = degrees % 360
        total_range = self.max_us - self.min_us
        degre = round(total_range * (degrees/(180-0)*(180-10)+0))
        
        us = self.min_us + degre // self.angle
        self.write_us(us)

def servo(analogport, angle):
    Servo(analogport).write_angle(angle)