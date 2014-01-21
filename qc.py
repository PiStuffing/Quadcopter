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
		# Sets sample rate to 1kHz/1+3 = 250Hz or 4ms
		####### Code currently loops at 170Hz, so 250Hz guarantees fresh data ######
		####### while allowing sufficient time to read it                     ######
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
		logger.debug('5Hz DLPF to filter out non-gravitational acceleration for Euler')
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
		# the interrupt also - sleep is just 0.5ms as data is updates every 4ms - need
		# to allow time for the data to be read.  The alternative would be to have a
		# thread waking on the interrupt, but CPU efficiency here isn't a driving force.
		#---------------------------------------------------------------------------
		while not (self.i2c.readU8(self.__MPU6050_RA_INT_STATUS) == 0x01):
			time.sleep(0.0005)

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

		return fax, fay, faz, fgx, -fgy, fgz
	
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
		pitch = math.atan2(fax, math.pow(math.pow(faz + 1.0, 2) + math.pow(fay, 2), 0.5))
		roll = math.atan2(fay,  math.pow(math.pow(faz + 1.0, 2) + math.pow(fax, 2), 0.5))
		tilt = math.atan2(faz + 1.0, math.pow(math.pow(fax, 2) + math.pow(fay, 2), 0.5))
		return pitch, roll, tilt

	def readTemp(self):
		temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
		temp = (float(temp) / 340) + 36.53
		logger.debug('temp = %s oC', temp)
		return temp


############################################################################################
# PID algorithm to take input sensor readings, and target requirements, and
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
		# Store off last input for the next differential calculation and time for next integral calculation
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
# GPIO pins initialization for MPU6050 interrupt and the sounder
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
	cli_hover_speed = 590   # derived through holding the quad in the air at various speed, feeling for hover
	cli_vvp_gain = 150.0    # derived through testing - could be increased at risk of noise
	cli_vvi_gain = 50.0     # derived through testing - by including integrate get slow, more stable height
	cli_vvd_gain = 0.0
	cli_hvp_gain = 0.2      # 1m/s target => 0.2g => atan2(0.2g/1.0g) = 0.2 radians = 11 degrees
	cli_hvi_gain = 0.0
	cli_hvd_gain = 0.0
	cli_aap_gain = 2.5      # 0.2 radians (11 degrees) => 0.5 rad / second
	cli_aai_gain = 0.0
	cli_aad_gain = 0.1
	cli_arp_gain = 110      # 0.5 rad / second => +/- 16 PWM => seems plausible and safe
	cli_ari_gain = 130
	cli_ard_gain = 2.5
	cli_test_case = 0
	hover_speed_defaulted = True
	arp_set = False
	ari_set = False
	ard_set = False

	#-----------------------------------------------------------------------------------
	# Right, let's get on with reading the command line and checking consistency
	#-----------------------------------------------------------------------------------
	try:
		opts, args = getopt.getopt(argv,'fgvah:', ['tc=', 'vvp=', 'vvi=', 'vvd=', 'hvp=', 'hvi=', 'hvd=', 'aap=', 'aai=', 'aad=', 'arp=', 'ari=', 'ard='])
	except getopt.GetoptError:
		logger.critical('qcpi.py [-f][-t hover_speed][-g][-v]')
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-f':
			cli_fly = True

		elif opt in '-h':
			cli_hover_speed = int(arg)
			hover_speed_defaulted = False

		elif opt in '-v':
			cli_video = True

		elif opt in '-g':
			cli_calibrate_gravity = True

		elif opt in '--vvp':
			cli_vvp_gain = float(arg)

		elif opt in '--vvi':
			cli_vvi_gain = float(arg)

		elif opt in '--vvd':
			cli_vvd_gain = float(arg)

		elif opt in '--hvp':
			cli_hvp_gain = float(arg)

		elif opt in '--hvi':
			cli_hvi_gain = float(arg)

		elif opt in '--hvd':
			cli_hvd_gain = float(arg)

		elif opt in '--aap':
			cli_aap_gain = float(arg)

		elif opt in '--aai':
			cli_aai_gain = float(arg)

		elif opt in '--aad':
			cli_aad_gain = float(arg)

		elif opt in '--arp':
			cli_arp_gain = float(arg)
			arp_set = True

		elif opt in '--ari':
			cli_ari_gain = float(arg)
			ari_set = True

		elif opt in '--ard':
			cli_ard_gain = float(arg)
			ard_set = True

		elif opt in '--tc':
			cli_test_case = int(arg)


	if not cli_calibrate_gravity and not cli_fly and cli_test_case == 0:
		logger.critical('Must specify one of -f or -g or --tc')
		logger.critical('  qcpi.py [-f] [-t speed] [-c] [-v]')
		logger.critical('  -f set whether to fly')
		logger.critical('  -h set the hover speed for manual testing')
		logger.critical('  -g calibrate and save the gravity offsets')
		logger.critical('  -v video the flight')
		logger.critical('  --vvp set vertical speed PID P gain')
		logger.critical('  --vvi set vertical speed PID P gain')
		logger.critical('  --vvd set vertical speed PID P gain')
		logger.critical('  --hvp set horizontal speed PID P gain')
		logger.critical('  --hvi set horizontal speed PID I gain')
		logger.critical('  --hvd set horizontal speed PID D gain')
		logger.critical('  --hap set horizontal angle PID P gain')
		logger.critical('  --hai set horizontal angle PID I gain')
		logger.critical('  --had set horizontal angle PID D gain')
		sys.exit(2)

	elif cli_fly and (cli_hover_speed < 0 or cli_hover_speed > 1000):
		logger.critical('Test speed must lie in the following range')
		logger.critical('0 <= test speed <= 1000')
		sys.exit(2)

	elif cli_test_case != 0 and (cli_fly or cli_calibrate_gravity):
		logger.critical('Choose specific test case (--tc) or fly (-f) or calibrate gravity (-g)')
		sys.exit(2)

	elif cli_test_case != 0 and cli_test_case != 3 and hover_speed_defaulted:
		logger.critical('You are running testcase 1 or 2 (--tc) so you need to specify a hover speed (-h).')
		sys.exit(2)

	elif cli_test_case == 0 and cli_fly:
		logger.critical('Pre-flight checks passes, enjoy your flight, sir!')

	elif cli_test_case == 0 and cli_calibrate_gravity:
		logger.critical('Calibrate gravity is it, sir!')

	elif cli_test_case != 3 and hover_speed_defaulted:
		logger.critical('You are running testcase 1 or 2 (--tc) so you need to specify a hover speed (-h).')
		sys.exit(2)

	elif cli_test_case < 1 or cli_test_case > 3:
		logger.critical('Choose test case 1, 2 or 3')
		sys.exit(2)

	elif cli_test_case == 3 and not arp_set and not ari_set and not ard_set:
		logger.critical('You must choose a starting point for the angular rate PID P, I and D gains')
		logger.critical('Try sudo python ./qc.py --tc=3 -h 450 --arp=50 --ari=0.0 --ard=0.0 and work up from there')
		sys.exit(2)

	elif cli_test_case == 3 and ard_set and ari_set and ard_set:
		cli_vvp_gain = 0.0
		cli_vvi_gain = 0.0
		cli_vvd_gain = 0.0
		cli_hvp_gain = 0.0
		cli_hvi_gain = 0.0
		cli_hvd_gain = 0.0
		cli_aap_gain = 0.0
		cli_aai_gain = 0.0
		cli_aad_gain = 0.0

	elif cli_test_case == 3:
		logger.critical('For testcase 3, you must set all of --arp, --ari and --ard parameters')
		sys.exit(2)


	return cli_calibrate_gravity, cli_fly, cli_hover_speed, cli_video, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_aap_gain, cli_aai_gain, cli_aad_gain, cli_arp_gain, cli_ari_gain, cli_ard_gain, cli_test_case

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
calibrate_gravity, flying, hover_speed, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, aap_gain, aai_gain, aad_gain, arp_gain, ari_gain, ard_gain, test_case = CheckCLI(sys.argv[1:])
logger.critical("calibrate_gravity = %s, fly = %s, hover_speed = %d, shoot_video = %s, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, aap_gain = %f, aai_gain = %f, aad_gain = %f, arp_gain = %f, ari_gain = %f, ard_gain = %f, test_case = %d", calibrate_gravity, flying, hover_speed, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, aap_gain, aai_gain, aad_gain, arp_gain, ari_gain, ard_gain, test_case)

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
		print '- try running qcpi -g on a flat, horizontal surface first'
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
# Countdown: 4 beeps prior to waiting for RC connection
#-------------------------------------------------------------------------------------------
CountdownBeep(4)

#-------------------------------------------------------------------------------------------
# Wait pending a sockets connection with the RC if required
#-------------------------------------------------------------------------------------------
inputs = []
outputs = []

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

evx = 0.0
evy = 0.0
evz = 0.0

evx_target = 0.0
evy_target = 0.0
evz_target = 0.0

ya_target = 0.0

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
# START TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the right way
#-------------------------------------------------------------------------------------------
if test_case == 1:
	for esc in esc_list:
		for beep_count in range(0, hover_speed, 10):
			#---------------------------------------------------------------------------
			# Spin up to just under take-off / hover speeds#
			#---------------------------------------------------------------------------
			esc.update(beep_count);
			time.sleep(0.01)
		time.sleep(10.0)
		esc.update(0);

	time.sleep(10.0)
	CleanShutdown()
#-------------------------------------------------------------------------------------------
# END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the right way
#-------------------------------------------------------------------------------------------


#-------------------------------------------------------------------------------------------
# START TESTCASE 2 CODE: Spin all the motors together for 10s judging how much lift is provided
#-------------------------------------------------------------------------------------------
elif test_case == 2:
	for beep_count in range(0, hover_speed, 10):
		for esc in esc_list:
			#---------------------------------------------------------------------------
			# Spin up to just under take-off / hover speeds
			#---------------------------------------------------------------------------
			esc.update(beep_count);
		
		RPIO.output(RPIO_STATUS_SOUNDER, not RPIO.input(RPIO_STATUS_SOUNDER))
		time.sleep(0.01)

	time.sleep(10.0)
	CleanShutdown()
#-------------------------------------------------------------------------------------------
# END TESTCASE 2 CODE: Spin all the motors together for 10s judging how much lift is provided
#-------------------------------------------------------------------------------------------


#-------------------------------------------------------------------------------------------
# Bring the ESCs up to just under takeoff speed
#-------------------------------------------------------------------------------------------
else:
	for beep_count in range(0, hover_speed, 10):
		for esc in esc_list:
			if test_case == 3 and (esc.motor_location == (MOTOR_LOCATION_FRONT + MOTOR_LOCATION_RIGHT) or esc.motor_location == (MOTOR_LOCATION_BACK + MOTOR_LOCATION_LEFT)):
				#---------------------------------------------------------------------------
				# Spin up to just under take-off / hover speeds
				#---------------------------------------------------------------------------
				esc.update(0)
			else:
				esc.update(beep_count)
		
		RPIO.output(RPIO_STATUS_SOUNDER, not RPIO.input(RPIO_STATUS_SOUNDER))
		time.sleep(0.01)

#===========================================================================================
# Tuning: Set up the PID gains - some are hard coded mathematical approximations, some come
# from the CLI parameters to allow for tuning
#===========================================================================================
# A PID is a simple algorithm which takes a desired state of a system (the "target", perhaps from a remote control)
# and the current state (the "input" or "feedback", perhaps from a sensor), subtracts them to determine the "error" in
# the system, and applies some simple math(s) to come up with a corrective "output". It does this repeatedly with the aim
# that the "error" reduces and is maintained at near zero due to the "feedback".
#
# This mechanism allows complex systems to be broken down and corrected even when a complete mathematical model of the
# system is too complicated or external factors mean the system cannot be modelled directly.
#
# In a quadcopter, the external targets (those a user control for flight) are horizontal and vertical speed plus tilt.
#
# - Horizontal speed error is corrected proportionally by providing the corrective output as the horizontal acceleration target
# - Horizontal acceleration can be converted directly to an angle of tilt using trigonometry, so the horizontal acceleration target is
#   converted to the pitch / roll angle target.
# - Tilt angle error is corrected proportionally by providing the corrective output to the angular speed PID target
# - Angular speed error is corrected proportionally by providing the corrective output to the motors' ESCs
#
# - Horizontal speed feedback is provided by integrated accelerometer (or GPS in future)
# - Angular feedback comes from accelerometer Euler and integrated gyro sensors passed through a complementary filter to track theta
# - Angular speed feedback comes directly from the gyros
#
# That's 3 PIDs each for horizontal X & Y axes movement - 6 in total so far.
#
# - Vertical speed error is corrected proportionally by providing the corrective output to the motors' ESCs (cf. angular speed above)
#
# - Vertical speed feedback comes from the Z-axis accelerometer integrated over time, and compensated for any tilt (cos(theta)cos(phi))
#
# That's 2 PID for vertical Z axis speed control.
#
# - Yaw speed error is corrected proportionally by providing the corrective output to the motors' ESCs
#
# - Yaw speed feedback comes directly from the Z-axis gyro
#
# That's 2 PID for Z axis yaw control.
#
# So 9 PIDs in total
#===========================================================================================

#-------------------------------------------------------------------------------------------
# The earth X axis speed controls forward / backward speed
#-------------------------------------------------------------------------------------------
PID_EVX_P_GAIN = hvp_gain
PID_EVX_I_GAIN = hvi_gain
PID_EVX_D_GAIN = hvd_gain	

#-------------------------------------------------------------------------------------------
# The earth Y axis speed controls left / right speed
#-------------------------------------------------------------------------------------------
PID_EVY_P_GAIN = hvp_gain
PID_EVY_I_GAIN = hvi_gain
PID_EVY_D_GAIN = hvd_gain	

#-------------------------------------------------------------------------------------------
# The earth Z axis speed controls rise / fall speed
#-------------------------------------------------------------------------------------------
PID_EVZ_P_GAIN = vvp_gain
PID_EVZ_I_GAIN = vvi_gain
PID_EVZ_D_GAIN = vvd_gain

#-------------------------------------------------------------------------------------------
# The PITCH ANGLE PID controls stable rotation speed about the Y-axis
#-------------------------------------------------------------------------------------------
PID_PA_P_GAIN = aap_gain
PID_PA_I_GAIN = aai_gain
PID_PA_D_GAIN = aad_gain

#-------------------------------------------------------------------------------------------
# The ROLL ANGLE PID controls stable rotation speed about the X-axis
#-------------------------------------------------------------------------------------------
PID_RA_P_GAIN = aap_gain
PID_RA_I_GAIN = aai_gain
PID_RA_D_GAIN = aad_gain

#-------------------------------------------------------------------------------------------
# The YAW ANGLE PID controls stable rotation speed about the Z-axis
#-------------------------------------------------------------------------------------------
PID_YA_P_GAIN = 0.0 # 2.5
PID_YA_I_GAIN = 0.0 # 5.0
PID_YA_D_GAIN = 0.0

#-------------------------------------------------------------------------------------------
# The PITCH RATE PID controls stable rotation speed about the Y-axis
#-------------------------------------------------------------------------------------------
PID_PR_P_GAIN = arp_gain
PID_PR_I_GAIN = ari_gain
PID_PR_D_GAIN = ard_gain

#-------------------------------------------------------------------------------------------
# The ROLL RATE PID controls stable rotation speed about the X-axis
#-------------------------------------------------------------------------------------------
PID_RR_P_GAIN = arp_gain
PID_RR_I_GAIN = ari_gain
PID_RR_D_GAIN = ard_gain

#-------------------------------------------------------------------------------------------
# The YAW RATE PID controls stable rotation speed about the Z-axis
#-------------------------------------------------------------------------------------------
PID_YR_P_GAIN = arp_gain / 5
PID_YR_I_GAIN = ari_gain / 5
PID_YR_D_GAIN = ard_gain / 5

#-------------------------------------------------------------------------------------------
# Enable time dependent factors PIDs - everything beyond here and "while keep_looping:" is time
# critical and should be kept to an absolute minimum.
#-------------------------------------------------------------------------------------------

#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw angle PIDs
#-------------------------------------------------------------------------------------------
pa_pid = PID(PID_PA_P_GAIN, PID_PA_I_GAIN, PID_PA_D_GAIN)
ra_pid = PID(PID_RA_P_GAIN, PID_RA_I_GAIN, PID_RA_D_GAIN)
ya_pid = PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN)

#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw rate PIDs
#-------------------------------------------------------------------------------------------
pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN)
rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN)
yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN)

#-------------------------------------------------------------------------------------------
# Start the X, Y (horizontal) and Z (vertical) velocity PIDs
#-------------------------------------------------------------------------------------------
evx_pid = PID(PID_EVX_P_GAIN, PID_EVX_I_GAIN, PID_EVX_D_GAIN)
evy_pid = PID(PID_EVY_P_GAIN, PID_EVY_I_GAIN, PID_EVY_D_GAIN)
evz_pid = PID(PID_EVZ_P_GAIN, PID_EVZ_I_GAIN, PID_EVZ_D_GAIN)

#-------------------------------------------------------------------------------------------
# Diagnostic statistics log header
#-------------------------------------------------------------------------------------------
logger.warning(', Time, DT, Loop, fgx, fgy, fgz, fax, fay, faz, i pitch, i roll, e pitch, e roll, c pitch, c roll, i yaw, e tilt, evx, exp, exi, exd, pap, pai, pad, prp, pri, prd, pf_out, eay, eyp, eyi, eyd, rap, rai, rad, rrp, rri, rrd, rf_out, evz, ezp, ezi, ezd, efz_out, yap, yai, yap, yrp, yri, yrd, yf_out, FL spin, FR spin, BL spin, BR spin')

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

		if elapsed_time >= 8.0:
			fsm_input = INPUT_LAND

		if elapsed_time >= 11.0:
			fsm_input = INPUT_STOP

	if fsm_state == STATE_OFF and fsm_input == INPUT_TAKEOFF:
		logger.critical('#AB: ASCENDING')
		fsm_state = STATE_ASCENDING
		fsm_input = INPUT_NONE
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)

		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
		evz_target = 0.33
		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------



	elif fsm_state == STATE_ASCENDING and (fsm_input == INPUT_HOVER or fsm_input == INPUT_SIGNAL):
		logger.critical('#AB: HOVERING')
		fsm_state = STATE_HOVERING
		fsm_input = INPUT_NONE
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)

		#-----------------------AUTONOMOUS VERTICAL HOVER SPEED---------------------
		evz_target = 0.0
		#-----------------------AUTONOMOUS VERTICAL HOVER SPEED---------------------



	elif fsm_state == STATE_HOVERING and (fsm_input == INPUT_LAND or fsm_input == INPUT_SIGNAL):
		logger.critical('#AB: DESCENDING')
		fsm_state = STATE_DESCENDING
		fsm_input = INPUT_NONE
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)

		#----------------------AUTONOMOUS VERTICAL LANDING SPEED--------------------
		evz_target = -0.33
		#----------------------AUTONOMOUS VERTICAL LANDING SPEED--------------------



	elif fsm_state == STATE_DESCENDING and (fsm_input == INPUT_STOP or fsm_input == INPUT_SIGNAL):
		logger.critical('#AB: LANDED')
		fsm_state = STATE_OFF
		fsm_input = INPUT_NONE
		keep_looping = False
		hover_speed = 0
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)

		#---------------------AUTONOMOUS VERTICAL PIN-DOWN SPEED--------------------
		evz_target = -0.0
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
	tau = 0.05
	tau_fraction = tau / (tau + delta_time)

	c_pitch = tau_fraction * (prev_c_pitch + fgy * delta_time) + (1 - tau_fraction) * e_pitch
	prev_c_pitch = c_pitch

	c_roll = tau_fraction * (prev_c_roll + fgx * delta_time) + (1 - tau_fraction) * e_roll
	prev_c_roll = c_roll

	#-----------------------------------------------------------------------------------
	# Choose the best measure of the angles
	#-----------------------------------------------------------------------------------
	pa = c_pitch
	ra = c_roll
	ya = i_yaw

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
	eax = fax * math.cos(pa)
	eay = fay * math.cos(ra)
	eaz = faz * math.cos(pa) * math.cos(ra)

	evx += eax * delta_time * G_FORCE
	evy += eay * delta_time * G_FORCE
	evz += eaz * delta_time * G_FORCE

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
	[p_out, i_out, d_out] = evx_pid.Compute(evx, evx_target)
	evx_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	evx_out = p_out + i_out + d_out

	[p_out, i_out, d_out] = evy_pid.Compute(evy, evy_target)
	evy_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	evy_out =  p_out + i_out + d_out

	[p_out, i_out, d_out] = evz_pid.Compute(evz, evz_target)
	evz_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	efz_out = p_out + i_out + d_out

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling speed PIDs
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_speed_pids += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Convert the horizontal velocity PID output i.e. the horizontal acceleration target in g's
	# into the pitch and roll angle PID targets in readians
	#-----------------------------------------------------------------------------------
	pa_target = -math.atan2(evx_out, 1.0)
	ra_target = -math.atan2(evy_out, 1.0)

	#-----------------------------------------------------------------------------------
	# Run the absolute angle PIDs each rotation axis.
	#-----------------------------------------------------------------------------------
	[p_out, i_out, d_out] = pa_pid.Compute(pa, pa_target)
	pa_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	pr_target = p_out + i_out + d_out
	[p_out, i_out, d_out] = ra_pid.Compute(ra, ra_target)
	ra_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	rr_target = p_out + i_out + d_out
	[p_out, i_out, d_out] = ya_pid.Compute(ya, ya_target)
	ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	yr_target = p_out + i_out + d_out

	#-----------------------------------------------------------------------------------
	# Run the angular rate PIDs each rotation axis.
	#-----------------------------------------------------------------------------------
	[p_out, i_out, d_out] = pr_pid.Compute(fgy, pr_target)
	pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	pf_out = p_out + i_out + d_out
	[p_out, i_out, d_out] = rr_pid.Compute(fgx, rr_target)
	rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	rf_out = p_out + i_out + d_out
	[p_out, i_out, d_out] = yr_pid.Compute(fgz, yr_target)
	yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
	yf_out = p_out + i_out + d_out

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling angle PIDs
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_angle_pids += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Mixer: Walk through the ESCs, and depending on their location, apply the output accordingly
	#===================================================================================
	pf_out = int(round(pf_out / 2))
	rf_out = int(round(rf_out / 2))
	yf_out = int(round(yf_out / 2))
	vert_out = hover_speed + int(round(efz_out))

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
			delta_spin -= rf_out
		else:
			delta_spin += rf_out

		#---------------------------------------------------------------------------
		# For a forward downwards pitch, the y gyro goes negative, so the PID error is
		# postive, meaning PID output is positive, meaning this needs to be added to the
		# front blades and subreacted from the back.
		#---------------------------------------------------------------------------
		if esc.motor_location & MOTOR_LOCATION_BACK:
			delta_spin -= pf_out
		else:
			delta_spin += pf_out

		#---------------------------------------------------------------------------
		# START TESTCASE 3 CODE: Disable front-right and back-left blades; disable yaw PID
		#---------------------------------------------------------------------------
		if test_case == 3:
			yf_out = 0
			if (esc.motor_location == (MOTOR_LOCATION_FRONT | MOTOR_LOCATION_RIGHT) or esc.motor_location == (MOTOR_LOCATION_BACK | MOTOR_LOCATION_LEFT)):
				delta_spin = 0

		#---------------------------------------------------------------------------
		# END TESTCASE 3 CODE: Disable front-left and back right blades
		#---------------------------------------------------------------------------

		#---------------------------------------------------------------------------
		# An excess CW rotating of the front-right and back-left (FR & BL) blades
		# results in an CW rotation of the quadcopter body. The z gyro produces
		# a negative output as a result. This then leads to the PID error
		# being postive, meaning PID  output is positive. Since the PID output needs to reduce the
		# over-enthusiastic CW rotation of the FR & BL blades, the positive PID
		# output needs to be subtracted from those blades (thus slowing their rotation)
		# and added to the ACW FL & BR blades (thus speeding them up) to
		# compensate for the yaw.
		#---------------------------------------------------------------------------
		if esc.motor_rotation == MOTOR_ROTATION_ACW:
			delta_spin -= yf_out
		else:
			delta_spin += yf_out

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
		logger.warning(', %f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s, %f, %f, %s, %s, %s, %f, %f, %s, %f, %s, %s, %f, %d, %d, %d, %d', elapsed_time, delta_time, loop_count, fgx, fgy, fgz, fax, fay, faz, math.degrees(i_pitch), math.degrees(i_roll), math.degrees(e_pitch), math.degrees(e_roll), math.degrees(c_pitch), math.degrees(c_roll), math.degrees(i_yaw), math.degrees(e_tilt), evx, evx_diags, pa_diags, pr_diags, pf_out, evy, evy_diags, ra_diags, rr_diags, rf_out, evz, evz_diags, efz_out, ya_diags, yr_diags, yf_out, esc_list[0].current_pulse_width, esc_list[1].current_pulse_width, esc_list[2].current_pulse_width, esc_list[3].current_pulse_width)
		last_log_time = current_time

	#-----------------------------------------------------------------------------------
	# Track proportion of time logging diagnostics
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_diagnostics += sample_time - prev_sample_time
	prev_sample_time = sample_time

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
