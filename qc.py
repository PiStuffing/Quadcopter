#!/usr/bin/env python
from __future__ import division
import signal
import socket
import time
import string
import sys
import getopt
import time
import math
import threading
from array import *
import smbus
import select
import os
import struct
import logging
from RPIO import PWM
import RPIO
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library

############################################################################################
#
#  Adafruit i2c interface plus performance / error handling changes
#
############################################################################################
class I2C:

	def __init__(self, address, bus=smbus.SMBus(1)):
		self.address = address
		self.bus = bus

	def reverseByteOrder(self, data):
		"Reverses the byte order of an int (16-bit) or long (32-bit) value"
		# Courtesy Vishal Sapre
		dstr = hex(data)[2:].replace('L','')
		byteCount = len(dstr[::2])
		val = 0
		for i, n in enumerate(range(byteCount)):
			d = data & 0xFF
			val |= (d << (8 * (byteCount - i - 1)))
			data >>= 8
		return val

	def write8(self, reg, value):
		"Writes an 8-bit value to the specified register/address"
		while True:
			try:
				self.bus.write_byte_data(self.address, reg, value)
				logger.debug('I2C: Wrote 0x%02X to register 0x%02X', value, reg)
				break
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)

	def writeList(self, reg, list):
		"Writes an array of bytes using I2C format"
		while True:
			try:
				self.bus.write_i2c_block_data(self.address, reg, list)
				break
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)

	def readU8(self, reg):
		"Read an unsigned byte from the I2C device"
		while True:
			try:
				result = self.bus.read_byte_data(self.address, reg)
				logger.debug('I2C: Device 0x%02X returned 0x%02X from reg 0x%02X', self.address, result & 0xFF, reg)
				return result
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)

	def readS8(self, reg):
		"Reads a signed byte from the I2C device"
		while True:
			try:
				result = self.bus.read_byte_data(self.address, reg)
				logger.debug('I2C: Device 0x%02X returned 0x%02X from reg 0x%02X', self.address, result & 0xFF, reg)
				if (result > 127):
					return result - 256
				else:
					return result
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)

	def readU16(self, reg):
		"Reads an unsigned 16-bit value from the I2C device"
		while True:
			try:
				hibyte = self.bus.read_byte_data(self.address, reg)
				result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
				logger.debug('I2C: Device 0x%02X returned 0x%04X from reg 0x%02X', self.address, result & 0xFFFF, reg)
				if result == 0x7FFF or result == 0x8000:
					logger.critical('I2C read max value')
					time.sleep(0.001)
				else:
					return result
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)

	def readS16(self, reg):
		"Reads a signed 16-bit value from the I2C device"
		while True:
			try:
				hibyte = self.bus.read_byte_data(self.address, reg)
				if (hibyte > 127):
					hibyte -= 256
				result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
				logger.debug('I2C: Device 0x%02X returned 0x%04X from reg 0x%02X', self.address, result & 0xFFFF, reg)
				if result == 0x7FFF or result == 0x8000:
					logger.critical('I2C read max value')
					time.sleep(0.001)
				else:
					return result
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)
				
	def readList(self, reg, length):
		"Reads a a byte array value from the I2C device"
		while True:
			try:
				result = self.bus.read_i2c_block_data(self.address, reg, length)
				logger.debug('I2C: Device 0x%02X from reg 0x%02X', self.address, reg)
				return result
			except IOError, err:
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.001)


############################################################################################
#
#  Gyroscope / Accelerometer class for reading position / movement
#
############################################################################################
class MPU6050 :
	i2c = None

	# Registers/etc.
	__MPU6050_RA_XG_OFFS_TC= 0x00       # [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
	__MPU6050_RA_YG_OFFS_TC= 0x01       # [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
	__MPU6050_RA_ZG_OFFS_TC= 0x02       # [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
	__MPU6050_RA_X_FINE_GAIN= 0x03      # [7:0] X_FINE_GAIN
	__MPU6050_RA_Y_FINE_GAIN= 0x04      # [7:0] Y_FINE_GAIN
	__MPU6050_RA_Z_FINE_GAIN= 0x05      # [7:0] Z_FINE_GAIN
	__MPU6050_RA_XA_OFFS_H= 0x06	# [15:0] XA_OFFS
	__MPU6050_RA_XA_OFFS_L_TC= 0x07
	__MPU6050_RA_YA_OFFS_H= 0x08	# [15:0] YA_OFFS
	__MPU6050_RA_YA_OFFS_L_TC= 0x09
	__MPU6050_RA_ZA_OFFS_H= 0x0A	# [15:0] ZA_OFFS
	__MPU6050_RA_ZA_OFFS_L_TC= 0x0B
	__MPU6050_RA_XG_OFFS_USRH= 0x13     # [15:0] XG_OFFS_USR
	__MPU6050_RA_XG_OFFS_USRL= 0x14
	__MPU6050_RA_YG_OFFS_USRH= 0x15     # [15:0] YG_OFFS_USR
	__MPU6050_RA_YG_OFFS_USRL= 0x16
	__MPU6050_RA_ZG_OFFS_USRH= 0x17     # [15:0] ZG_OFFS_USR
	__MPU6050_RA_ZG_OFFS_USRL= 0x18
	__MPU6050_RA_SMPLRT_DIV= 0x19
	__MPU6050_RA_CONFIG= 0x1A
	__MPU6050_RA_GYRO_CONFIG= 0x1B
	__MPU6050_RA_ACCEL_CONFIG= 0x1C
	__MPU6050_RA_FF_THR= 0x1D
	__MPU6050_RA_FF_DUR= 0x1E
	__MPU6050_RA_MOT_THR= 0x1F
	__MPU6050_RA_MOT_DUR= 0x20
	__MPU6050_RA_ZRMOT_THR= 0x21
	__MPU6050_RA_ZRMOT_DUR= 0x22
	__MPU6050_RA_FIFO_EN= 0x23
	__MPU6050_RA_I2C_MST_CTRL= 0x24
	__MPU6050_RA_I2C_SLV0_ADDR= 0x25
	__MPU6050_RA_I2C_SLV0_REG= 0x26
	__MPU6050_RA_I2C_SLV0_CTRL= 0x27
	__MPU6050_RA_I2C_SLV1_ADDR= 0x28
	__MPU6050_RA_I2C_SLV1_REG= 0x29
	__MPU6050_RA_I2C_SLV1_CTRL= 0x2A
	__MPU6050_RA_I2C_SLV2_ADDR= 0x2B
	__MPU6050_RA_I2C_SLV2_REG= 0x2C
	__MPU6050_RA_I2C_SLV2_CTRL= 0x2D
	__MPU6050_RA_I2C_SLV3_ADDR= 0x2E
	__MPU6050_RA_I2C_SLV3_REG= 0x2F
	__MPU6050_RA_I2C_SLV3_CTRL= 0x30
	__MPU6050_RA_I2C_SLV4_ADDR= 0x31
	__MPU6050_RA_I2C_SLV4_REG= 0x32
	__MPU6050_RA_I2C_SLV4_DO= 0x33
	__MPU6050_RA_I2C_SLV4_CTRL= 0x34
	__MPU6050_RA_I2C_SLV4_DI= 0x35
	__MPU6050_RA_I2C_MST_STATUS= 0x36
	__MPU6050_RA_INT_PIN_CFG= 0x37
	__MPU6050_RA_INT_ENABLE= 0x38
	__MPU6050_RA_DMP_INT_STATUS= 0x39
	__MPU6050_RA_INT_STATUS= 0x3A
	__MPU6050_RA_ACCEL_XOUT_H= 0x3B
	__MPU6050_RA_ACCEL_XOUT_L= 0x3C
	__MPU6050_RA_ACCEL_YOUT_H= 0x3D
	__MPU6050_RA_ACCEL_YOUT_L= 0x3E
	__MPU6050_RA_ACCEL_ZOUT_H= 0x3F
	__MPU6050_RA_ACCEL_ZOUT_L= 0x40
	__MPU6050_RA_TEMP_OUT_H= 0x41
	__MPU6050_RA_TEMP_OUT_L= 0x42
	__MPU6050_RA_GYRO_XOUT_H= 0x43
	__MPU6050_RA_GYRO_XOUT_L= 0x44
	__MPU6050_RA_GYRO_YOUT_H= 0x45
	__MPU6050_RA_GYRO_YOUT_L= 0x46
	__MPU6050_RA_GYRO_ZOUT_H= 0x47
	__MPU6050_RA_GYRO_ZOUT_L= 0x48
	__MPU6050_RA_EXT_SENS_DATA_00= 0x49
	__MPU6050_RA_EXT_SENS_DATA_01= 0x4A
	__MPU6050_RA_EXT_SENS_DATA_02= 0x4B
	__MPU6050_RA_EXT_SENS_DATA_03= 0x4C
	__MPU6050_RA_EXT_SENS_DATA_04= 0x4D
	__MPU6050_RA_EXT_SENS_DATA_05= 0x4E
	__MPU6050_RA_EXT_SENS_DATA_06= 0x4F
	__MPU6050_RA_EXT_SENS_DATA_07= 0x50
	__MPU6050_RA_EXT_SENS_DATA_08= 0x51
	__MPU6050_RA_EXT_SENS_DATA_09= 0x52
	__MPU6050_RA_EXT_SENS_DATA_10= 0x53
	__MPU6050_RA_EXT_SENS_DATA_11= 0x54
	__MPU6050_RA_EXT_SENS_DATA_12= 0x55
	__MPU6050_RA_EXT_SENS_DATA_13= 0x56
	__MPU6050_RA_EXT_SENS_DATA_14= 0x57
	__MPU6050_RA_EXT_SENS_DATA_15= 0x58
	__MPU6050_RA_EXT_SENS_DATA_16= 0x59
	__MPU6050_RA_EXT_SENS_DATA_17= 0x5A
	__MPU6050_RA_EXT_SENS_DATA_18= 0x5B
	__MPU6050_RA_EXT_SENS_DATA_19= 0x5C
	__MPU6050_RA_EXT_SENS_DATA_20= 0x5D
	__MPU6050_RA_EXT_SENS_DATA_21= 0x5E
	__MPU6050_RA_EXT_SENS_DATA_22= 0x5F
	__MPU6050_RA_EXT_SENS_DATA_23= 0x60
	__MPU6050_RA_MOT_DETECT_STATUS= 0x61
	__MPU6050_RA_I2C_SLV0_DO= 0x63
	__MPU6050_RA_I2C_SLV1_DO= 0x64
	__MPU6050_RA_I2C_SLV2_DO= 0x65
	__MPU6050_RA_I2C_SLV3_DO= 0x66
	__MPU6050_RA_I2C_MST_DELAY_CTRL= 0x67
	__MPU6050_RA_SIGNAL_PATH_RESET= 0x68
	__MPU6050_RA_MOT_DETECT_CTRL= 0x69
	__MPU6050_RA_USER_CTRL= 0x6A
	__MPU6050_RA_PWR_MGMT_1= 0x6B
	__MPU6050_RA_PWR_MGMT_2= 0x6C
	__MPU6050_RA_BANK_SEL= 0x6D
	__MPU6050_RA_MEM_START_ADDR= 0x6E
	__MPU6050_RA_MEM_R_W= 0x6F
	__MPU6050_RA_DMP_CFG_1= 0x70
	__MPU6050_RA_DMP_CFG_2= 0x71
	__MPU6050_RA_FIFO_COUNTH= 0x72
	__MPU6050_RA_FIFO_COUNTL= 0x73
	__MPU6050_RA_FIFO_R_W= 0x74
	__MPU6050_RA_WHO_AM_I= 0x75

	__CALIBRATION_ITERATIONS = 100

	def __init__(self, address=0x68):
		self.i2c = I2C(address)
		self.address = address
		self.grav_x_offset = 0.0
		self.grav_y_offset = 0.0
		self.grav_z_offset = 0.0
		self.gyro_x_offset = 0.0
		self.gyro_y_offset = 0.0
		self.gyro_z_offset = 0.0
		self.sensor_data = array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		self.result_array = array('h', [0, 0, 0, 0, 0, 0, 0])

		logger.info('Reseting MPU-6050')

		#---------------------------------------------------------------------------
		# Reset all registers
		#---------------------------------------------------------------------------
		logger.debug('Reset all registers')
		self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
		time.sleep(5.0)
	
		#---------------------------------------------------------------------------
		# Sets sample rate to 1kHz/1+3 = 250Hz
		####### Code currently loops at 170Hz, so 250Hz guarantees fresh data ######
		#---------------------------------------------------------------------------
		logger.debug('Sample rate 250Hz')
		self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, 0x03)
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Sets clock source to gyro reference w/ PLL
		#---------------------------------------------------------------------------
		logger.debug('Clock gyro PLL')
		self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x02)
		time.sleep(0.1)

		#---------------------------------------------------------------------------
		# Disable FSync, Use of DLPF => 1kHz sample frequency used above divided by the
		# sample divide factor.
		# 0x01 = 180Hz
		# 0x02 =  100Hz
		# 0x03 =  45Hz
		# 0x04 =  20Hz
		# 0x05 =  10Hz
		# 0x06 =   5Hz
		#---------------------------------------------------------------------------
		logger.debug('5Hz DLPF')
		self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x06)
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Disable gyro self tests, scale of
		# 0x00 =  +/- 250 degrees/s
		# 0x08 =  +/- 500 degrees/s
		# 0x10 = +/- 1000 degrees/s
		# 0x18 = +/- 2000 degrees/s
		#---------------------------------------------------------------------------
		logger.debug('Gyro +/-500 degrees/s')
		self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, 0x08)
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Disable accel self tests, scale of +/-2g
		# 0x00 =  +/- 2g
		# 0x08 =  +/- 4g
		# 0x10 =  +/- 8g
		# 0x18 = +/- 16g
		#---------------------------------------------------------------------------
		logger.debug('Accel +/- 2g')
		self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x00)
		time.sleep(0.1)

		#---------------------------------------------------------------------------
		# Setup INT pin to latch and AUX I2C pass through
		#---------------------------------------------------------------------------
		logger.debug('Enable interrupt')
		self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x20)
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Enable data ready interrupt
		#---------------------------------------------------------------------------
		logger.debug('Interrupt data ready')
		self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x01)
		time.sleep(0.1)


	def readSensorsRaw(self):
		#---------------------------------------------------------------------------
		# Hard loop on the data ready interrupt until it gets set high.  This clears
		# the interrupt also
		#---------------------------------------------------------------------------
		while not (self.i2c.readU8(self.__MPU6050_RA_INT_STATUS) == 0x01):
			time.sleep(0.001)

		#---------------------------------------------------------------------------
		# For speed of reading, read all the sensors and parse to SHORTs after
		#---------------------------------------------------------------------------
		sensor_data = self.i2c.readList(self.__MPU6050_RA_ACCEL_XOUT_H, 14)

		for index in range(0, 14, 2):
			if (sensor_data[index] > 127):
				sensor_data[index] -= 256
			self.result_array[int(index / 2)] = (sensor_data[index] << 8) + sensor_data[index + 1]

		return self.result_array


	def readSensors(self):
		#---------------------------------------------------------------------------
		# +/- 2g 2 * 16 bit range for the accelerometer
		# +/- 500 degrees per second * 16 bit range for the gyroscope - converted to radians
		#---------------------------------------------------------------------------
		[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()

		fax = ax * 4.0 / 65536 - self.grav_x_offset
		fay = ay * 4.0 / 65536 - self.grav_y_offset
		faz = az * 4.0 / 65536 - self.grav_z_offset

		fgx = gx * 1000.0 * math.pi / (65536 * 180) - self.gyro_x_offset
		fgy = gy * 1000.0 * math.pi / (65536 * 180) - self.gyro_y_offset
		fgz = gz * 1000.0 * math.pi / (65536 * 180) - self.gyro_z_offset

		return fax, fay, faz, fgx, fgy, fgz
	
	def calibrateGyros(self):
		for loop_count in range(0, self.__CALIBRATION_ITERATIONS):
			[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()
			self.gyro_x_offset += gx
			self.gyro_y_offset += gy
			self.gyro_z_offset += gz

			time.sleep(0.05)

		self.gyro_x_offset *= 1000.0 * math.pi / (65536 * 180 * self.__CALIBRATION_ITERATIONS)
		self.gyro_y_offset *= 1000.0 * math.pi / (65536 * 180 * self.__CALIBRATION_ITERATIONS)
		self.gyro_z_offset *= 1000.0 * math.pi / (65536 * 180 * self.__CALIBRATION_ITERATIONS)


	def calibrateGravity(self, file_name):
		grav_x_offset = 0
		grav_y_offset = 0
		grav_z_offset = 0

		for loop_count in range(0, self.__CALIBRATION_ITERATIONS):
			[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()
			grav_x_offset += ax
			grav_y_offset += ay
			grav_z_offset += az

			time.sleep(0.05)

		grav_x_offset *= (4.0 / (65536 * self.__CALIBRATION_ITERATIONS))
		grav_y_offset *= (4.0 / (65536 * self.__CALIBRATION_ITERATIONS))
		grav_z_offset *= (4.0 / (65536 * self.__CALIBRATION_ITERATIONS))

		#---------------------------------------------------------------------------
		# Open the offset config file
		#---------------------------------------------------------------------------
		cfg_rc = True
		try:
			with open(file_name, 'w+') as cfg_file:
				cfg_file.write('%f\n' % grav_x_offset)
				cfg_file.write('%f\n' % grav_y_offset)
				cfg_file.write('%f\n' % grav_z_offset)
				cfg_file.flush()

		except IOError, err:
			logger.critical('Could not open offset config file: %s for writing', file_name)
			cfg_rc = False

		return cfg_rc


	def readGravity(self, file_name):
		#---------------------------------------------------------------------------
		# Open the Offsets config file, and read the contents
		#---------------------------------------------------------------------------
		cfg_rc = True
		try:
			with open(file_name, 'r') as cfg_file:
				str_grav_x_offset = cfg_file.readline()
				str_grav_y_offset = cfg_file.readline()
				str_grav_z_offset = cfg_file.readline()

			self.grav_x_offset = float(str_grav_x_offset)
			self.grav_y_offset = float(str_grav_y_offset)
			self.grav_z_offset = float(str_grav_z_offset)

		except IOError, err:
			logger.critical('Could not open offset config file: %s for reading', file_name)
			cfg_rc = False

		return cfg_rc

	def getEulerAngles(self, fax, fay, faz):
		#---------------------------------------------------------------------------
		# What's the angle in the x and y plane from horizontal in radians?
                # Note fax, fay, fax are all the calibrated outputs reading 0, 0, 0 on
                # horizontal ground as a measure of speed in a given direction.  For Euler we
                # need to re-add gravity of 1g so the sensors read 0, 0, 1 for a horizontal setting
		#---------------------------------------------------------------------------
		pitch = -math.atan2(fax, faz + 1.0)
		roll = math.atan2(fay,  faz + 1.0)
		tilt = math.atan2(faz + 1.0, math.pow(math.pow(fax, 2) + math.pow(fay, 2), 0.5))

		return pitch, roll, tilt

	def readTemp(self):
		temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
		temp = (float(temp) / 340) + 36.53
		logger.debug('temp = %s oC', temp)
		return temp


############################################################################################
# PID algorithm to take input accelerometer readings, and target accelermeter requirements, and
# as a result feedback new rotor speeds.
############################################################################################
class PID:

	def __init__(self, p_gain, i_gain, d_gain):
		self.last_error = 0.0
		self.last_time = time.time()
		self.p_gain = p_gain
		self.i_gain = i_gain
		self.d_gain = d_gain

		self.i_error = 0.0
		self.i_err_min = 0.0
		self.i_err_max = 0.0
		if i_gain != 0.0:
			self.i_err_min = -250.0 / i_gain
			self.i_err_max = +250.0 / i_gain


	def Compute(self, input, target):

		now = time.time()
		dt = (now - self.last_time)

		#---------------------------------------------------------------------------
		# Error is what the PID alogithm acts upon to derive the output
		#---------------------------------------------------------------------------
		error = target - input

		#---------------------------------------------------------------------------
		# The proportional term takes the distance between current input and target
		# and uses this proportially (based on Kp) to control the ESC pulse width
		#---------------------------------------------------------------------------
		p_error = error

		#---------------------------------------------------------------------------
		# The integral term sums the errors across many compute calls to allow for
		# external factors like wind speed and friction
		#---------------------------------------------------------------------------
		self.i_error += (error + self.last_error) * dt
		if self.i_gain != 0.0 and self.i_error > self.i_err_max:
			self.i_error = self.i_err_max
		elif self.i_gain != 0.0 and self.i_error < self.i_err_min:
			self.i_error = self.i_err_min
		i_error = self.i_error

		#---------------------------------------------------------------------------
		# The differential term accounts for the fact that as error approaches 0,
		# the output needs to be reduced proportionally to ensure factors such as
		# momentum do not cause overshoot.
		#---------------------------------------------------------------------------
		d_error = (error - self.last_error) / dt

		#---------------------------------------------------------------------------
		# The overall output is the sum of the (P)roportional, (I)ntegral and (D)iffertial terms
		#---------------------------------------------------------------------------
		p_output = self.p_gain * p_error
		i_output = self.i_gain * i_error
		d_output = self.d_gain * d_error

		#---------------------------------------------------------------------------
		# Store off last input (for the next differential calulation) and time for calculating the integral value
		#---------------------------------------------------------------------------
		self.last_error = error
		self.last_time = now

		#---------------------------------------------------------------------------
		# Return the output, which has been tuned to be the increment / decrement in ESC PWM
		#---------------------------------------------------------------------------
		return p_output, i_output, d_output

############################################################################################
#
#  Class for managing each blade + motor configuration via its ESC
#
############################################################################################
class ESC:
	pwm = None

	def __init__(self, pin, location, rotation, name):

		#---------------------------------------------------------------------------
		# The GPIO BCM numbered pin providing PWM signal for this ESC
		#---------------------------------------------------------------------------
		self.bcm_pin = pin

		#---------------------------------------------------------------------------
		# The location on the quad, and the direction of the motor controlled by this ESC
		#---------------------------------------------------------------------------
		self.motor_location = location
		self.motor_rotation = rotation

		#---------------------------------------------------------------------------
		# The PWM pulse width range required by this ESC in microseconds
		#---------------------------------------------------------------------------
		self.min_pulse_width = 1000
		self.max_pulse_width = 2000

		#---------------------------------------------------------------------------
		# The PWM pulse range required by this ESC
		#---------------------------------------------------------------------------
		self.current_pulse_width = self.min_pulse_width
		self.name = name

		#---------------------------------------------------------------------------
		# Initialize the RPIO DMA PWM
		#---------------------------------------------------------------------------
		if not PWM.is_setup():
			PWM.setup(1)    # 1us increment
			PWM.init_channel(RPIO_DMA_CHANNEL, 3000) # 3ms carrier period
		PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.current_pulse_width)


	def update(self, spin_rate):
		self.current_pulse_width = int(self.min_pulse_width + spin_rate)

		if self.current_pulse_width < self.min_pulse_width:
			self.current_pulse_width = self.min_pulse_width
		if self.current_pulse_width > self.max_pulse_width:
			self.current_pulse_width = self.max_pulse_width

		PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.current_pulse_width)

		
############################################################################################
#
# GPIO pins initialization for /OE on the PWM device and the status LED
#
############################################################################################
def RpioSetup():
	RPIO.setmode(RPIO.BCM)

	#-----------------------------------------------------------------------------------
	# Set the beeper output LOW
	#-----------------------------------------------------------------------------------
	logger.info('Set status sounder pin %s as out', RPIO_STATUS_SOUNDER)
	RPIO.setup(RPIO_STATUS_SOUNDER, RPIO.OUT, RPIO.LOW)

	#-----------------------------------------------------------------------------------
	# Set the MPU6050 interrupt input
	#-----------------------------------------------------------------------------------
	logger.info('Setup MPU6050 interrupt input %s', RPIO_SENSOR_DATA_RDY)
	RPIO.setup(RPIO_SENSOR_DATA_RDY, RPIO.IN, RPIO.PUD_DOWN)


############################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
############################################################################################
def CheckCLI(argv):
	cli_fly = False
	cli_calibrate_gravity = False
	cli_video = False
	cli_takeoff_speed = 590
	cli_vsp_gain = 150.0
	cli_vsi_gain = 50.0
	cli_vsd_gain = 0.0
	cli_hsp_gain = 1.0
	cli_hsi_gain = 0.0
	cli_hsd_gain = 0.0


	try:
		opts, args = getopt.getopt(argv,'fgvat:', ['vsp=', 'vsi=', 'vsd=', 'hsp=', 'hsi=', 'hsd='])
	except getopt.GetoptError:
		logger.critical('qcpi.py [-f][-t take_off_speed][-g][-v]')
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-f':
			cli_fly = True

		elif opt in '-t':
			cli_takeoff_speed = int(arg)

		elif opt in '-v':
			cli_video = True

		elif opt in '-g':
			cli_calibrate_gravity = True

		elif opt in '--vsp':
			cli_vsp_gain = float(arg)

		elif opt in '--vsi':
			cli_vsi_gain = float(arg)

		elif opt in '--vsd':
			cli_vsd_gain = float(arg)

		elif opt in '--hsp':
			cli_hsp_gain = float(arg)

		elif opt in '--hsi':
			cli_hsi_gain = float(arg)

		elif opt in '--hsd':
			cli_hsd_gain = float(arg)


	if not cli_calibrate_gravity and not cli_fly:
		logger.critical('Must specify -f or -g')
		logger.critical('  qcpi.py [-f] [-t speed] [-c] [-v]')
		logger.critical('  -f set whether to fly')
		logger.critical('  -t set the takeoff speed for manual takeoff')
		logger.critical('  -g calibrate and save the gravity offsets')
		logger.critical('  -v video the flight')
		logger.critical('  --vsp set vertical speed PID P gain')
		logger.critical('  --vsi set vertical speed PID P gain')
		logger.critical('  --vsd set vertical speed PID P gain')
		logger.critical('  --hsp set horizontal speed PID P gain')
		logger.critical('  --hsi set horizontal speed PID I gain')
		logger.critical('  --hsd set horizontal speed PID D gain')
		sys.exit(2)

	if cli_fly and (cli_takeoff_speed < 0 or cli_takeoff_speed > 1000):
		logger.critical('Test speed must lie in the following range')
		logger.critical('0 <= test speed <= 1000')
		sys.exit(2)

#	if (math.copysign(1.0, cli_hsp_gain) != math.copysign(1.0, cli_hsi_gain)) and (math.copysign(1.0, cli_hsp_gain) != math.copysign(1.0, cli_hsd_gain)):
#		logger.critical("All horizonatal gains must have the same sign")
#		sys.exit(2)
#
#	if (math.copysign(1.0, cli_vsp_gain) != math.copysign(1.0, cli_vsi_gain)) and (math.copysign(1.0, cli_vsp_gain) != math.copysign(1.0, cli_vsd_gain)):
#		logger.critical("All vertical gains must have the same sign")
#		sys.exit(2)

	return cli_calibrate_gravity, cli_fly, cli_takeoff_speed, cli_video, cli_vsp_gain, cli_vsi_gain, cli_vsd_gain, cli_hsp_gain, cli_hsi_gain, cli_hsd_gain

############################################################################################
#
# Count down beeper
#
############################################################################################
def CountdownBeep(num_beeps):
	for beep in range(0, num_beeps):
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)
		time.sleep(0.25)
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)
		time.sleep(0.25)
	time.sleep(2.0)

############################################################################################
#
# Shutdown triggered by early Ctrl-C or end of script
#
############################################################################################
def CleanShutdown():
	global esc_list
	global shoot_video
	global video

	#-----------------------------------------------------------------------------------
	# Time for teddy bye byes
	#-----------------------------------------------------------------------------------
	for esc in esc_list:
		logger.info('Stop blade %d spinning', esc_index)
		esc.update(0)

	#-----------------------------------------------------------------------------------
	# Stop the video if it's running
	#-----------------------------------------------------------------------------------
	if shoot_video:
		video.send_signal(signal.SIGINT)

	#-----------------------------------------------------------------------------------
	# Copy logs from /dev/shm (shared / virtual memory) to the Logs directory.
	#-----------------------------------------------------------------------------------
	now = datetime.now()
	now_string = now.strftime("%y%m%d-%H:%M:%S")
	log_file_name = "qcstats" + now_string + ".csv"
	shutil.move("/dev/shm/qclogs", log_file_name)

	#-----------------------------------------------------------------------------------
	# Clean up PWM / GPIO
	#-----------------------------------------------------------------------------------
	PWM.cleanup()
	RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)
	RPIO.cleanup()

	#-----------------------------------------------------------------------------------
	# Unlock memory we've used from RAM
	#-----------------------------------------------------------------------------------
	munlockall()

	sys.exit(0)

############################################################################################
#
# Signal handler for Ctrl-C => next FSM transition if running else stop
#
############################################################################################
def SignalHandler(signal, frame):
	global loop_count
	global fsm_input
	global INPUT_SIGNAL

	if loop_count > 0:
		fsm_input = INPUT_SIGNAL
	else:
		CleanShutdown()

############################################################################################
#
# Main
#
############################################################################################

#-------------------------------------------------------------------------------------------
# Lock code permanently in memory - no swapping to disk
#-------------------------------------------------------------------------------------------
MCL_CURRENT = 1
MCL_FUTURE  = 2
def mlockall(flags = MCL_CURRENT| MCL_FUTURE):
	result = libc.mlockall(flags)
	if result != 0:
		raise Exception("cannot lock memmory, errno=%s" % ctypes.get_errno())

def munlockall():
	result = libc.munlockall()
	if result != 0:
		raise Exception("cannot lock memmory, errno=%s" % ctypes.get_errno())


libc_name = ctypes.util.find_library("c")
libc = ctypes.CDLL(libc_name, use_errno=True)
mlockall()

#-------------------------------------------------------------------------------------------
# Set up the global constants
#-------------------------------------------------------------------------------------------
G_FORCE = 9.80665

RPIO_DMA_CHANNEL = 1

use_sockets = False

ESC_BCM_BL = 22
ESC_BCM_FL = 17
ESC_BCM_FR = 18
ESC_BCM_BR = 23

MOTOR_LOCATION_FRONT = 0b00000000
MOTOR_LOCATION_BACK = 0b00000010
MOTOR_LOCATION_LEFT = 0b00000000
MOTOR_LOCATION_RIGHT = 0b00000001

MOTOR_ROTATION_CW = 1
MOTOR_ROTATION_ACW = 2

NUM_SOCK = 5
RC_SILENCE_LIMIT = 10

#-------------------------------------------------------------------------------------------
# Through testing, take-off happens @ 587
#-------------------------------------------------------------------------------------------
TAKEOFF_READY_RATE = 590

#-------------------------------------------------------------------------------------------
# Set the BCM outputs assigned to LED and sensor interrupt
#-------------------------------------------------------------------------------------------
RPIO_STATUS_SOUNDER = 27
RPIO_SENSOR_DATA_RDY = 25

silent_scan_count = 0

#-------------------------------------------------------------------------------------------
# Set up the base logging
#-------------------------------------------------------------------------------------------
logger = logging.getLogger('QC logger')
logger.setLevel(logging.INFO)

#-------------------------------------------------------------------------------------------
# Create file and console logger handlers
#-------------------------------------------------------------------------------------------
file_handler = logging.FileHandler("/dev/shm/qclogs", 'w')
file_handler.setLevel(logging.WARNING)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.CRITICAL)

#-------------------------------------------------------------------------------------------
# Create a formatter and add it to both handlers
#-------------------------------------------------------------------------------------------
console_formatter = logging.Formatter('%(message)s')
console_handler.setFormatter(console_formatter)

file_formatter = logging.Formatter('[%(levelname)s] (%(threadName)-10s) %(funcName)s %(lineno)d %(message)s')
file_handler.setFormatter(file_formatter)

#-------------------------------------------------------------------------------------------
# Add both handlers to the logger
#-------------------------------------------------------------------------------------------
logger.addHandler(console_handler)
logger.addHandler(file_handler)

#-------------------------------------------------------------------------------------------
# Check the command line to see if we are calibrating or flying - if neither are set, CheckCLI sys.exit(0)s
#-------------------------------------------------------------------------------------------
calibrate_gravity, flying, takeoff_speed, shoot_video, vsp_gain, vsi_gain, vsd_gain, hsp_gain, hsi_gain, hsd_gain = CheckCLI(sys.argv[1:])
logger.critical("calibrate_gravity = %s, fly = %s, takeoff_speed = %d, shoot_video = %s, vsp_gain = %f, vsi_gain = %f, vsd_gain= %f, hsp_gain = %f, hsi_gain = %f, hsd_gain = %f", calibrate_gravity, flying, takeoff_speed, shoot_video, vsp_gain, vsi_gain, vsd_gain, hsp_gain, hsi_gain, hsd_gain)

#-------------------------------------------------------------------------------------------
# Initialize the gyroscope / accelerometer I2C object
#-------------------------------------------------------------------------------------------
mpu6050 = MPU6050(0x68)

#-------------------------------------------------------------------------------------------
# Calibrate to accelometer for exact gravity
#-------------------------------------------------------------------------------------------
if calibrate_gravity:
	if not mpu6050.calibrateGravity('./qcgravity.cfg'):
		print 'Gravity normalization error'
		sys.exit(1)
	sys.exit(0)

else:
	if not mpu6050.readGravity('./qcgravity.cfg'):
		print 'Gravity config error'
		print '- try running qcpi -c on a flat, horizontal surface first'
		sys.exit(1)


#-------------------------------------------------------------------------------------------
# From hereonin we're in flight mode.  First task is to shut the ESCs up.
#-------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------
# Assign motor properties to each ESC
#-------------------------------------------------------------------------------------------
pin_list = [ESC_BCM_FL, ESC_BCM_FR, ESC_BCM_BL, ESC_BCM_BR]
location_list = [MOTOR_LOCATION_FRONT | MOTOR_LOCATION_LEFT, MOTOR_LOCATION_FRONT | MOTOR_LOCATION_RIGHT, MOTOR_LOCATION_BACK | MOTOR_LOCATION_LEFT, MOTOR_LOCATION_BACK | MOTOR_LOCATION_RIGHT]
rotation_list = [MOTOR_ROTATION_ACW, MOTOR_ROTATION_CW, MOTOR_ROTATION_CW, MOTOR_ROTATION_ACW]
name_list = ['front left', 'front right', 'back left', 'back right']

#-------------------------------------------------------------------------------------------
# Prime the ESCs with the default 0 spin rotors
#-------------------------------------------------------------------------------------------
esc_list = []
for esc_index in range(0, 4):
	esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
	esc_list.append(esc)

#-------------------------------------------------------------------------------------------
# Now with some peace and quiet, enable RPIO for beeper and MPU 6050 interrupts
#-------------------------------------------------------------------------------------------
RpioSetup()

#-------------------------------------------------------------------------------------------
# Countdown: 5 beeps prior to gyro calibration
#-------------------------------------------------------------------------------------------
CountdownBeep(5)

#-------------------------------------------------------------------------------------------
# Calibrate the gyros
#-------------------------------------------------------------------------------------------
mpu6050.calibrateGyros()

#-------------------------------------------------------------------------------------------
# Countdown: 4 beeps prior to waiting for RC connection (code deleted as untested)
#-------------------------------------------------------------------------------------------
CountdownBeep(4)

#-------------------------------------------------------------------------------------------
# Countdown: 3 beeps for successful RC connection
#-------------------------------------------------------------------------------------------
CountdownBeep(3)

#---------------------------------------------------------------------------
# Set the signal handler here so the spin can be cancelled when loop_count = 0
# or moved on when > 0. Prior to this point Ctrl-C does what you'd expect
#---------------------------------------------------------------------------
loop_count = 0
signal.signal(signal.SIGINT, SignalHandler)

#-------------------------------------------------------------------------------------------
# Countdown: Start the video
#-------------------------------------------------------------------------------------------
CountdownBeep(2)

#-------------------------------------------------------------------------------------------
# Start up the video camera if required - this runs from take-off through to shutdown automatically.
# Run it in its own process group so that Ctrl-C for QC doesn't get through and stop the video
#-------------------------------------------------------------------------------------------
def Daemonize():
	os.setpgrp()

if shoot_video:
	now = datetime.now()
	now_string = now.strftime("%y%m%d-%H:%M:%S")
	video = subprocess.Popen(["raspivid", "-rot", "180", "-w", "1280", "-h", "720", "-o", "/home/pi/Videos/qcvid_" + now_string + ".h264", "-n", "-t", "0", "-fps", "30", "-b", "5000000"], preexec_fn =  Daemonize)

#-------------------------------------------------------------------------------------------
# Countdown: Get those blades spinning
#-------------------------------------------------------------------------------------------
CountdownBeep(1)

#------------------------------------------------------------------------------------------
# Set up the bits of state setup before takeoff
#-------------------------------------------------------------------------------------------
keep_looping = True
delta_time = 0.0

i_pitch = 0.0
i_roll = 0.0
i_yaw = 0.0

prev_c_pitch = 0.0
prev_c_roll = 0.0

eax_speed = 0.0
eay_speed = 0.0
eaz_speed = 0.0

eax_speed_target = 0.0
eay_speed_target = 0.0
eaz_speed_target = 0.0

yaw_angle_target = 0.0

INPUT_NONE = 0
INPUT_TAKEOFF = 1
INPUT_HOVER = 2
INPUT_LAND = 3
INPUT_STOP = 4
INPUT_SIGNAL = 4
fsm_input = INPUT_NONE

STATE_OFF = 0
STATE_ASCENDING = 1
STATE_HOVERING = 2
STATE_DESCENDING = 3
fsm_state = STATE_OFF


#-------------------------------------------------------------------------------------------
# Bring the ESCs up to just under takeoff speed
#-------------------------------------------------------------------------------------------
for beep_count in range(0, TAKEOFF_READY_RATE, 10):
	for esc in esc_list:
		#---------------------------------------------------------------------------
		# Spin up to just under take-off / hover speeds
		#---------------------------------------------------------------------------
		esc.update(beep_count);
	
	RPIO.output(RPIO_STATUS_SOUNDER, not RPIO.input(RPIO_STATUS_SOUNDER))
	time.sleep(0.01)

#-------------------------------------------------------------------------------------------
# Tuning: Set up the PID gains - some are hard coded, some come from the CLI parameters
#-------------------------------------------------------------------------------------------

#-------------------------------------------------------------------------------------------
# The earth X axis speed controls forward / backward speed
#-------------------------------------------------------------------------------------------
PID_EAX_SPEED_P_GAIN = hsp_gain
PID_EAX_SPEED_I_GAIN = hsi_gain
PID_EAX_SPEED_D_GAIN = hsd_gain	

#-------------------------------------------------------------------------------------------
# The earth Y axis speed controls left / right speed
#-------------------------------------------------------------------------------------------
PID_EAY_SPEED_P_GAIN = hsp_gain
PID_EAY_SPEED_I_GAIN = hsi_gain
PID_EAY_SPEED_D_GAIN = hsd_gain	

#-------------------------------------------------------------------------------------------
# The earth Z axis speed controls rise / fall speed
#-------------------------------------------------------------------------------------------
PID_EAZ_SPEED_P_GAIN = vsp_gain
PID_EAZ_SPEED_I_GAIN = vsi_gain
PID_EAZ_SPEED_D_GAIN = vsd_gain

#-------------------------------------------------------------------------------------------
# The PITCH ANGLE PID controls stable rotation speed about the Y-axis
#-------------------------------------------------------------------------------------------
PID_PITCH_ANGLE_P_GAIN = 2.5
PID_PITCH_ANGLE_I_GAIN = 0.0
PID_PITCH_ANGLE_D_GAIN = 0.1

#-------------------------------------------------------------------------------------------
# The ROLL ANGLE PID controls stable rotation speed about the X-axis
#-------------------------------------------------------------------------------------------
PID_ROLL_ANGLE_P_GAIN = 2.5
PID_ROLL_ANGLE_I_GAIN = 0.0
PID_ROLL_ANGLE_D_GAIN = 0.1

#-------------------------------------------------------------------------------------------
# The YAW ANGLE PID controls stable rotation speed about the Z-axis
#-------------------------------------------------------------------------------------------
PID_YAW_ANGLE_P_GAIN = 2.5
PID_YAW_ANGLE_I_GAIN = 5.0
PID_YAW_ANGLE_D_GAIN = 0.1

#-------------------------------------------------------------------------------------------
# The PITCH RATE PID controls stable rotation speed about the Y-axis
#-------------------------------------------------------------------------------------------
PID_PITCH_RATE_P_GAIN = 150      # Was 2.5 when using degrees rather than radians
PID_PITCH_RATE_I_GAIN = 300      # Was 5.0 when using degrees rather than radians
PID_PITCH_RATE_D_GAIN = 5.75     # Was 0.1 when using degrees rather than radians

#-------------------------------------------------------------------------------------------
# The ROLL RATE PID controls stable rotation speed about the X-axis
#-------------------------------------------------------------------------------------------
PID_ROLL_RATE_P_GAIN = 150       # Was 2.5 when using degrees rather than radians
PID_ROLL_RATE_I_GAIN = 300       # Was 5.0 when using degrees rather than radians
PID_ROLL_RATE_D_GAIN = 5.75      # Was 0.1 when using degrees rather than radians

#-------------------------------------------------------------------------------------------
# The YAW RATE PID controls stable rotation speed about the Z-axis
#-------------------------------------------------------------------------------------------
PID_YAW_RATE_P_GAIN = 150        # Was 2.5 when using degrees rather than radians
PID_YAW_RATE_I_GAIN = 300        # Was 5.0 when using degrees rather than radians
PID_YAW_RATE_D_GAIN = 5.75	 # Was 0.1 when using degrees rather than radians

#-------------------------------------------------------------------------------------------
# Enable time dependent factors PIDs - everything beyond here and "while keep_looping:" is time
# critical and should be kept to an absolute minimum.
#-------------------------------------------------------------------------------------------
pitch_angle_pid = PID(PID_PITCH_ANGLE_P_GAIN, PID_PITCH_ANGLE_I_GAIN, PID_PITCH_ANGLE_D_GAIN)
roll_angle_pid = PID(PID_ROLL_ANGLE_P_GAIN, PID_ROLL_ANGLE_I_GAIN, PID_ROLL_ANGLE_D_GAIN)
yaw_angle_pid = PID(PID_YAW_ANGLE_P_GAIN, PID_YAW_ANGLE_I_GAIN, PID_YAW_ANGLE_D_GAIN)

pitch_rate_pid = PID(PID_PITCH_RATE_P_GAIN, PID_PITCH_RATE_I_GAIN, PID_PITCH_RATE_D_GAIN)
roll_rate_pid = PID(PID_ROLL_RATE_P_GAIN, PID_ROLL_RATE_I_GAIN, PID_ROLL_RATE_D_GAIN)
yaw_rate_pid = PID(PID_YAW_RATE_P_GAIN, PID_YAW_RATE_I_GAIN, PID_YAW_RATE_D_GAIN)

eax_speed_pid = PID(PID_EAX_SPEED_P_GAIN, PID_EAX_SPEED_I_GAIN, PID_EAX_SPEED_D_GAIN)
eay_speed_pid = PID(PID_EAY_SPEED_P_GAIN, PID_EAY_SPEED_I_GAIN, PID_EAY_SPEED_D_GAIN)
eaz_speed_pid = PID(PID_EAZ_SPEED_P_GAIN, PID_EAZ_SPEED_I_GAIN, PID_EAZ_SPEED_D_GAIN)

#-------------------------------------------------------------------------------------------
# Diagnostic statistics log header
#-------------------------------------------------------------------------------------------
logger.warning(', Time, DT, Loop, fgx, fgy, fgz, fax, fay, faz, c pitch, c roll, i yaw, e tilt, evz, prp, pri, prd, pap, pai, pad, rrp, rri, rrd, rap, rai, rad, yrp, yri, yrd, yap, yai, yap, exp, exi, exd, eyp, eyi, eyd, ezp, ezi, ezd, pitch target, roll target, yaw target, pitch out, roll_out, yaw out, vert speed out, FL spin, FR spin, BL spin, BR spin')

time_handling_fsm = 0.0
time_handling_sensors = 0.0
time_handling_eangles = 0.0
time_handling_iangles = 0.0
time_handling_angles_filter = 0.0
time_handling_axes_shift = 0.0
time_handling_speed_pids = 0.0
time_handling_angle_pids = 0.0
time_handling_pid_outputs = 0.0
time_handling_diagnostics = 0.0

elapsed_time = 0.0
start_time = time.time()
last_log_time = start_time
current_time = start_time
prev_sample_time = current_time

while keep_looping:
	#-----------------------------------------------------------------------------------
	# Update the elapsed time since start, the time for the last iteration, and
	# set the next sleep time to compensate for any overrun in scheduling.
	#-----------------------------------------------------------------------------------
	current_time = time.time()
	delta_time = current_time - start_time - elapsed_time
	elapsed_time = current_time - start_time
	loop_count += 1

	#===================================================================================
	# Interpreter: FSM inputs are mostly generated on a timer for testing; the exceptions are
	# - SIGNAL generated by a Ctrl-C.  These produce the targets for the PIDs.  In this
	# case, only the vertical speed target is modified - the horizontal X and Y speed targets
	# are configured higher up to be 0.0 to create a stable hover regardless of take-off conditions
	# of any external factors such as wind or weight balance.
	#===================================================================================
	if fsm_input != INPUT_SIGNAL:
		if elapsed_time >= 0.0:
			fsm_input = INPUT_TAKEOFF

		if elapsed_time >= 3.0:
			fsm_input = INPUT_HOVER

		if elapsed_time >=  6.0:
			fsm_input = INPUT_LAND

		if elapsed_time >= 10.0:
			fsm_input = INPUT_STOP

	if fsm_state == STATE_OFF and fsm_input == INPUT_TAKEOFF:
		logger.critical('#AB: ASCENDING')
		fsm_state = STATE_ASCENDING
		fsm_input = INPUT_NONE
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)

		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
		eaz_speed_target = 0.5
		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------



	elif fsm_state == STATE_ASCENDING and (fsm_input == INPUT_HOVER or fsm_input == INPUT_SIGNAL):
		logger.critical('#AB: HOVERING')
		fsm_state = STATE_HOVERING
		fsm_input = INPUT_NONE
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)

		#-----------------------AUTONOMOUS VERTICAL HOVER SPEED---------------------
		eaz_speed_target = 0.0
		#-----------------------AUTONOMOUS VERTICAL HOVER SPEED---------------------



	elif fsm_state == STATE_HOVERING and (fsm_input == INPUT_LAND or fsm_input == INPUT_SIGNAL):
		logger.critical('#AB: DESCENDING')
		fsm_state = STATE_DESCENDING
		fsm_input = INPUT_NONE
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)

		#----------------------AUTONOMOUS VERTICAL LANDING SPEED--------------------
		eaz_speed_target = -0.35
		#----------------------AUTONOMOUS VERTICAL LANDING SPEED--------------------



	elif fsm_state == STATE_DESCENDING and (fsm_input == INPUT_STOP or fsm_input == INPUT_SIGNAL):
		logger.critical('#AB: LANDED')
		fsm_state = STATE_OFF
		fsm_input = INPUT_NONE
		keep_looping = False
		takeoff_speed = 0
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)

		#---------------------AUTONOMOUS VERTICAL PIN-DOWN SPEED--------------------
		eaz_speed_target = -0.10
		#---------------------AUTONOMOUS VERTICAL PIN_DOWN SPEED--------------------

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling FSM
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_fsm += sample_time - prev_sample_time
	prev_sample_time = sample_time


	#===================================================================================
	# Inputs: Read the data from the accelerometer and gyro
	#===================================================================================
	[fax, fay, faz, fgx, fgy, fgz] = mpu6050.readSensors()

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling sensors
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_sensors += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Angles: Get the Euler angles in radians
	#===================================================================================
	e_pitch, e_roll, e_tilt  = mpu6050.getEulerAngles(fax, fay, faz)

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling euler angles
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_eangles += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Integrate the gyros angular velocity to determine absolute angle of tilt in radians
	#-----------------------------------------------------------------------------------
	i_pitch += fgy * delta_time
	i_roll += fgx * delta_time
	i_yaw += fgz * delta_time

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling integrated angles
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_iangles += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Filter: Apply complementary filter to ensure long-term accuracy of pitch / roll angles
	# tau is the handover period of 0.1s (1 / frequency) that the integrated gyro high pass
	# filter is taken over by the accelerometer Euler low-pass filter.  The combination of
	# tau plus the time increment (delta_time) then provides a fraction to mix the two angles sources.
	#===================================================================================
	tau = 0.1
	tau_fraction = tau / (tau + delta_time)

	c_pitch = tau_fraction * (prev_c_pitch + fgy * delta_time) + (1 - tau_fraction) * e_pitch
	prev_c_pitch = c_pitch

	c_roll = tau_fraction * (prev_c_roll + fgx * delta_time) + (1 - tau_fraction) * e_roll
	prev_c_roll = c_roll

	#-----------------------------------------------------------------------------------
	# Choose the best measure of the angles
	#-----------------------------------------------------------------------------------
	pitch_angle = c_pitch
	roll_angle = c_roll
	yaw_angle = i_yaw

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling angle filter
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_angles_filter += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Axes: Convert the accelerometers' g force to earth coordinates, then integrate to
	# convert to speeds in earth's X and Y axes
	#===================================================================================
	eax = fax * math.cos(pitch_angle)
	eay = fay * math.cos(roll_angle)
	eaz = faz * math.cos(pitch_angle) * math.cos(roll_angle)

	eax_speed += eax * delta_time * G_FORCE
	eay_speed += eay * delta_time * G_FORCE
	eaz_speed += eaz * delta_time * G_FORCE

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling sensor angles
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_axes_shift += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# PIDs: Run the horizontal speed PIDs each rotation axis to determine targets for angle PID
	# and the verical speed PID to control height.
	#===================================================================================
	[p_out, i_out, d_out] = eax_speed_pid.Compute(eax_speed, eax_speed_target)
	eax_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	pitch_angle_target = + p_out + i_out + d_out

	[p_out, i_out, d_out] = eay_speed_pid.Compute(eay_speed, eay_speed_target)
	eay_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	roll_angle_target = - p_out - i_out - d_out

	[p_out, i_out, d_out] = eaz_speed_pid.Compute(eaz_speed, eaz_speed_target)
	eaz_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	eaz_speed_out = p_out + i_out + d_out

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling speed PIDs
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_speed_pids += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Run the absolute angle PIDs each rotation axis.
	#-----------------------------------------------------------------------------------
	[p_out, i_out, d_out] = pitch_angle_pid.Compute(pitch_angle, pitch_angle_target)
	pa_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	pitch_rate_target = p_out + i_out + d_out
	[p_out, i_out, d_out] = roll_angle_pid.Compute(roll_angle, roll_angle_target)
	ra_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	roll_rate_target = p_out + i_out + d_out
	[p_out, i_out, d_out] = yaw_angle_pid.Compute(yaw_angle, yaw_angle_target)
	ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	yaw_rate_target = p_out + i_out + d_out

	#-----------------------------------------------------------------------------------
	# Run the angular rate PIDs each rotation axis.
	#-----------------------------------------------------------------------------------
	[p_out, i_out, d_out] = pitch_rate_pid.Compute(fgy, pitch_rate_target)
	pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	pitch_out = p_out + i_out + d_out
	[p_out, i_out, d_out] = roll_rate_pid.Compute(fgx, roll_rate_target)
	rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	roll_out = p_out + i_out + d_out
	[p_out, i_out, d_out] = yaw_rate_pid.Compute(fgz, yaw_rate_target)
	yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	yaw_out = p_out + i_out + d_out

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling angle PIDs
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_angle_pids += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Mixer: Walk through the ESCs, and depending on their location, apply the output accordingly
	#===================================================================================
	pitch_out = int(round(pitch_out / 2))
	roll_out = int(round(roll_out / 2))
	yaw_out = int(round(yaw_out / 2))
	vert_out = takeoff_speed + eaz_speed_out

	for esc in esc_list:
		#---------------------------------------------------------------------------
		# Update all blades' power in accordance with the z error
		#---------------------------------------------------------------------------
		delta_spin = vert_out

		#---------------------------------------------------------------------------
		# For a left downwards roll, the x gyro goes negative, so the PID error is positive,
		# meaning PID output is positive, meaning this needs to be added to the left blades
		# and subtracted from the right.
		#---------------------------------------------------------------------------
		if esc.motor_location & MOTOR_LOCATION_RIGHT:
			delta_spin -= roll_out
		else:
			delta_spin += roll_out

		#---------------------------------------------------------------------------
		# For a forward downwards pitch, the y gyro goes positive, so the PID error is
		# negative, meaning PID output is negative, meaning this needs to be subtracted
		# from the front blades and added to the back.
		#---------------------------------------------------------------------------
		if esc.motor_location & MOTOR_LOCATION_BACK:
			delta_spin += pitch_out
		else:
			delta_spin -= pitch_out

		#---------------------------------------------------------------------------
		# An excess CW rotating of the front-right and back-left (FR & BL) blades
		# results in an ACW rotation of the quadcopter body. The z gyro produces
		# a positive output as a result. This then leads to the PID error
		# (target - gyro) being negative, meaning PID  output is negative
		# (assuming positive gains). Since the PID output needs to reduce the
		# over-enthusiastic CW rotation of the FR & BL blades, the negative PID
		# output needs to be added to those blades (thus slowing their rotation)
		# and subtracted from the ACW FL & BR blades (thus speeding them up) to
		# compensate for the yaw.
		#---------------------------------------------------------------------------
		if esc.motor_rotation == MOTOR_ROTATION_CW:
			delta_spin += yaw_out
		else:
			delta_spin -= yaw_out

		#---------------------------------------------------------------------------
		# Apply the blended outputs to the esc PWM signal
		#---------------------------------------------------------------------------
		esc.update(delta_spin)

	#-----------------------------------------------------------------------------------
	# Track proportion of time applying PID outputs
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_pid_outputs += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Diagnostic statistics log - every 0.1s
	#-----------------------------------------------------------------------------------
	if current_time - last_log_time > 0.1:
		logger.warning(', %f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s, %s, %s, %s, %s, %s, %s, %f, %f, %f, %d, %d, %d, %d, %d, %d, %d, %d', elapsed_time, delta_time, loop_count, fgx, fgy, fgz, fax, fay, faz, math.degrees(c_pitch), math.degrees(c_roll), math.degrees(i_yaw), math.degrees(e_tilt), eaz_speed, pr_diags, pa_diags, rr_diags, ra_diags, yr_diags, ya_diags, eax_diags, eay_diags, eaz_diags, pitch_rate_target, roll_rate_target, yaw_rate_target, pitch_out, roll_out, yaw_out, eaz_speed_out, esc_list[0].current_pulse_width, esc_list[1].current_pulse_width, esc_list[2].current_pulse_width, esc_list[3].current_pulse_width)
		last_log_time = current_time

	#-----------------------------------------------------------------------------------
	# Track proportion of time applying PID outputs
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_diagnostics += sample_time - prev_sample_time
	prev_sample_time = sample_time

#-------------------------------------------------------------------------------------------
# Stop the blades during shutdown analysis
#-------------------------------------------------------------------------------------------
for esc in esc_list:
	esc.update(0)

#-------------------------------------------------------------------------------------------
# Dump the loops per second
#-------------------------------------------------------------------------------------------
logger.critical("loop speed %f loops per second", loop_count / elapsed_time)

#-------------------------------------------------------------------------------------------
# Dump the percentage time handling each step
#-------------------------------------------------------------------------------------------
logger.critical("%% fsm:              %f", time_handling_fsm / elapsed_time * 100.0)
logger.critical("%% sensors:          %f", time_handling_sensors / elapsed_time * 100.0)
logger.critical("%% eangles:          %f", time_handling_eangles / elapsed_time * 100.0)
logger.critical("%% iangles:          %f", time_handling_iangles / elapsed_time * 100.0)
logger.critical("%% angles_filter:    %f", time_handling_angles_filter / elapsed_time * 100.0)
logger.critical("%% axes_shift:       %f", time_handling_axes_shift / elapsed_time * 100.0)
logger.critical("%% speed_pids:       %f", time_handling_speed_pids / elapsed_time * 100.0)
logger.critical("%% angle_pids:       %f", time_handling_angle_pids / elapsed_time * 100.0)
logger.critical("%% pid_outputs:      %f", time_handling_pid_outputs / elapsed_time * 100.0)
logger.critical("%% pid_diagnosticss: %f", time_handling_diagnostics / elapsed_time * 100.0)


#-------------------------------------------------------------------------------------------
# Time for telly bye byes
#-------------------------------------------------------------------------------------------
CleanShutdown()
