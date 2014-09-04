#!/usr/bin/env python

###############################################################################################
###############################################################################################
##                                                                                           ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub            ##
## PiStuffing/Quadcopterunder GPL for non-commercial application.  Any code derived from     ##
## this should retain this copyright comment.                                                ##
##                                                                                           ##
## Copyright 2014 Andy Baker (Hove) - andy@pistuffing.co.uk                                  ##
##                                                                                           ##
###############################################################################################
###############################################################################################

from __future__ import division
import signal
import socket
import time
import string
import sys
import getopt
import math
import threading
from array import *
import smbus
import select
import os
import struct
import logging

#-------------------------------------------------------------------------------------------
# Only required when using GPIO.wait_for_event() to block for hardware interrupts
#-------------------------------------------------------------------------------------------
import RPi.GPIO as RPIO
# import RPIO

from RPIO import PWM
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library
import random

############################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
############################################################################################
class I2C:

	def __init__(self, address, bus=smbus.SMBus(1)):
		self.address = address
		self.bus = bus
		self.misses = 0

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
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def writeList(self, reg, list):
		"Writes an array of bytes using I2C format"
		while True:
			try:
				self.bus.write_i2c_block_data(self.address, reg, list)
				break
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def readU8(self, reg):
		"Read an unsigned byte from the I2C device"
		while True:
			try:
				result = self.bus.read_byte_data(self.address, reg)
				logger.debug('I2C: Device 0x%02X returned 0x%02X from reg 0x%02X', self.address, result & 0xFF, reg)
				return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

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
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def readU16(self, reg):
		"Reads an unsigned 16-bit value from the I2C device"
		while True:
			try:
				hibyte = self.bus.read_byte_data(self.address, reg)
				result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
				logger.debug('I2C: Device 0x%02X returned 0x%04X from reg 0x%02X', self.address, result & 0xFFFF, reg)
				if result == 0x7FFF or result == 0x8000:
					logger.critical('I2C read max value')
					time.sleep(0.0005)
				else:
					return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

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
					time.sleep(0.0005)
				else:
					return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)
				
	def readList(self, reg, length):
		"Reads a a byte array value from the I2C device"
		while True:
			try:
				result = self.bus.read_i2c_block_data(self.address, reg, length)
				logger.debug('I2C: Device 0x%02X from reg 0x%02X', self.address, reg)
				return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def getMisses(self):
		return self.misses


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

	def __init__(self, address=0x68, dlpf=6):
		self.i2c = I2C(address)
		self.address = address
		self.sensor_data = array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		self.result_array = array('h', [0, 0, 0, 0, 0, 0, 0])
		self.misses = 0

		self.gx_offset = 0.0
		self.gy_offset = 0.0
		self.gz_offset = 0.0

		self.ax_offset = 0.0
		self.ay_offset = 0.0
		self.az_offset = 0.0
		self.ax_gain = 1.0
		self.ay_gain = 1.0
		self.az_gain = 1.0

		self.ax_offset = -52.83
		self.ay_offset = 146.79
		self.az_offset = 845.01
		self.ax_gain = 0.994675113
		self.ay_gain = 0.992126089
		self.az_gain = 1.000029908

		logger.info('Reseting MPU-6050')
		#---------------------------------------------------------------------------
		# Ensure chip has completed boot
		#---------------------------------------------------------------------------
		time.sleep(0.5)

		#---------------------------------------------------------------------------
		# Reset all registers
		#---------------------------------------------------------------------------
		logger.debug('Reset all registers')
		self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
		time.sleep(5.0)
	
		#---------------------------------------------------------------------------
		# Sets sample rate to 1kHz/1+0 = 1kHz or 1ms
		#---------------------------------------------------------------------------
		logger.debug('Sample rate 1kHz')
		self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, 0x0)
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
		logger.debug('configurable DLPF to filter out non-gravitational acceleration for Euler')
		self.i2c.write8(self.__MPU6050_RA_CONFIG, dlpf)
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Disable gyro self tests, scale of
		# 0x00 =  +/- 250 degrees/s
		# 0x08 =  +/- 500 degrees/s
		# 0x10 = +/- 1000 degrees/s
		# 0x18 = +/- 2000 degrees/s
		# See SCALE_GYRO for converstion from raw data to units of radians per second
		#---------------------------------------------------------------------------
		# int(math.log(degrees / 250, 2)) << 3
		logger.debug('Gyro +/-250 degrees/s')
		self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, 0x00)
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Disable accel self tests, scale of +/-2g
		# 0x00 =  +/- 2g
		# 0x08 =  +/- 4g
		# 0x10 =  +/- 8g
		# 0x18 = +/- 16g
		# See SCALE_ACCEL for convertion from raw data to units of meters per second squared
		#---------------------------------------------------------------------------
		# int(math.log(g / 2, 2)) << 3

		logger.debug('Accel +/- 2g')
		self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x00)
		time.sleep(0.1)

		#---------------------------------------------------------------------------
		# Setup INT pin to latch and AUX I2C pass through
		#---------------------------------------------------------------------------
		logger.debug('Enable interrupt')
		self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x10) # 0x10 for edge detection, 0x20 for polling
		time.sleep(0.1)
	
		#---------------------------------------------------------------------------
		# Enable data ready interrupt
		#---------------------------------------------------------------------------
		logger.debug('Interrupt data ready')
		self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x01)
		time.sleep(0.1)

		#---------------------------------------------------------------------------
		# Check DLPF programming has worked.
		#---------------------------------------------------------------------------
		check_dlpf = self.i2c.readU8(self.__MPU6050_RA_CONFIG)
		if check_dlpf != dlpf:
			logger.critical("dlpf_check = %d, dlpf_config = %d", dlpf_check, dlpf_config);
			CleanShutdown()

		#---------------------------------------------------------------------------
		# Setup the interrupt for data ready
		#---------------------------------------------------------------------------
		RPIO.edge_detect_init(RPIO_DATA_READY_INTERRUPT, RPIO.RISING)


	def readSensorsRaw(self):
		global time_now

		#---------------------------------------------------------------------------
		# Clear the data ready interrupt, optionally make sure it clears, then hard loop
		# on the data ready interrupt until it gets set high again
		# to ensure we get the freshest set of valid data.  Sleep just 0.5ms as data is
		# updated every 4ms - need to allow time for the data to be read.
		#
		# The alternative is to wait for a rising edge n the data interrupt pin.
		#---------------------------------------------------------------------------

		#---------------------------------------------------------------------------
		# Clear any pending interrupt and wait for fresh data
		#---------------------------------------------------------------------------
#AB:		self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)

#-------------------------------------------------------------------------------------------
# Redundant code checking that the reset above has worked prior to the next step below
#-------------------------------------------------------------------------------------------
#AB:		while not (self.i2c.readU8(self.__MPU6050_RA_INT_STATUS) == 0x00):
#AB:                       time.sleep(0.0001)

#-------------------------------------------------------------------------------------------
# Current working and fast example using polling of the interupt status register
#-------------------------------------------------------------------------------------------
#AB:		while not (self.i2c.readU8(self.__MPU6050_RA_INT_STATUS) == 0x01):
#AB:			self.misses += 1

#-------------------------------------------------------------------------------------------
# Working but slower approach to use the hardware interrupt directly to detect data ready
#-------------------------------------------------------------------------------------------
		RPIO.edge_detect_wait(RPIO_DATA_READY_INTERRUPT)

		#---------------------------------------------------------------------------
		# For speed of reading, read all the sensors and parse to SHORTs after.  This
		# also ensures a self consistent set of sensor data compared to reading each
		# individually where the sensor data registers could be updated between reads.
		#---------------------------------------------------------------------------
		sensor_data = self.i2c.readList(self.__MPU6050_RA_ACCEL_XOUT_H, 14)

		#---------------------------------------------------------------------------
		# Time stamp the data for the best integration possible in the main
		# processing loop
		#---------------------------------------------------------------------------
		time_now = time.time()

		for index in range(0, 14, 2):
			if (sensor_data[index] > 127):
				sensor_data[index] -= 256
			self.result_array[int(index / 2)] = (sensor_data[index] << 8) + sensor_data[index + 1]

		return self.result_array

	def readSensors(self):
		#---------------------------------------------------------------------------
		# +/- 2g 2 * 16 bit range for the accelerometer
		# +/- 250 degrees per second * 16 bit range for the gyroscope - converted to radians
		#---------------------------------------------------------------------------
		[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()

		qax = (ax + self.ax_offset) * self.ax_gain
		qay = (ay + self.ay_offset) * self.ay_gain
		qaz = (az + self.az_offset) * self.az_gain

		qgx = gx - self.gx_offset
		qgy = gy - self.gy_offset
		qgz = gz - self.gz_offset

		return qax, qay, qaz, qgx, -qgy, qgz
	

	def calibrateGyros(self):
		self.gx_offset = 0.0
		self.gy_offset = 0.0
		self.gz_offset = 0.0

		for loop_count in range(0, self.__CALIBRATION_ITERATIONS):
			[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()
			self.gx_offset += gx
			self.gy_offset += gy
			self.gz_offset += gz

		self.gx_offset /= self.__CALIBRATION_ITERATIONS
		self.gy_offset /= self.__CALIBRATION_ITERATIONS
		self.gz_offset /= self.__CALIBRATION_ITERATIONS


	def calibrateGravity(self, file_name):
		gravity_x = 0.0
		gravity_y = 0.0
		gravity_z = 0.0
		temp_raw = 0


		for loop_count in range(0, self.__CALIBRATION_ITERATIONS):
			[ax, ay, az, temp_raw, gx, gy, gz] = self.readSensorsRaw()
			gravity_x += ax
			gravity_y += ay
			gravity_z += az

			time.sleep(0.05)

		gravity_x /= self.__CALIBRATION_ITERATIONS
		gravity_y /= self.__CALIBRATION_ITERATIONS
		gravity_z /= self.__CALIBRATION_ITERATIONS

		temp = (float(temp_raw) / 340) + 36.53

		#---------------------------------------------------------------------------
		# Open the offset config file
		#---------------------------------------------------------------------------
		cfg_rc = True
		try:
			with open(file_name, 'a') as cfg_file:
				cfg_file.write('%d, ' % temp_raw)
				cfg_file.write('%f, ' % temp)
				cfg_file.write('%f, ' % gravity_x)
				cfg_file.write('%f, ' % gravity_y)
				cfg_file.write('%f\n' % gravity_z)
				cfg_file.flush()

		except IOError, err:
			logger.critical('Could not open offset config file: %s for writing', file_name)
			cfg_rc = False

		return cfg_rc


	def readTemp(self):
		temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
		temp = (float(temp) / 340) + 36.53
		logger.debug('temp = %s oC', temp)
		return temp


	def getMisses(self):
		i2c_misses = self.i2c.getMisses()
		return self.misses, i2c_misses
		


############################################################################################
#
# PID algorithm to take input sensor readings, and target requirements, and
# as a result feedback new rotor speeds.
#
############################################################################################
class PID:

	def __init__(self, p_gain, i_gain, d_gain, now):
		self.last_error = 0.0
		self.last_time = now
		self.p_gain = p_gain
		self.i_gain = i_gain
		self.d_gain = d_gain

		self.i_error = 0.0
		self.i_err_min = 0.0
		self.i_err_max = 0.0
		if i_gain != 0.0:
			self.i_err_min = -250.0 / i_gain
			self.i_err_max = +250.0 / i_gain


	def Compute(self, input, target, now):
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
		# Initialize the RPIO DMA PWM for this ESC.
		#---------------------------------------------------------------------------
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
# Convert a vector to quadcopter-frame coordinates from earth-frame coordinates
#
############################################################################################
def GetEulerAngles(ax, ay, az):
	#---------------------------------------------------------------------------
	# What's the angle in the x and y plane from horizontal in radians?
	#---------------------------------------------------------------------------
	pitch = math.atan2(ax, math.pow(math.pow(ay, 2) + math.pow(az, 2), 0.5))
	roll = math.atan2(ay, math.pow(math.pow(ax, 2) + math.pow( az, 2), 0.5))
	tilt = math.atan2(math.pow(math.pow(ax, 2) + math.pow(ay, 2), 0.5), az)
	return pitch, roll, tilt

############################################################################################
#
# Convert a vector to quadcopter-frame coordinates from earth-frame coordinates
#
############################################################################################
def E2QFrame(evx, evy, evz, pa, ra, ya, ta):
	global use_tilt

	#===================================================================================
	# Axes: Convert a vector from earth- to quadcopter frame
	#
	# Matrix
	# ---------
	# |qvx|   | cos(pa) * cos(ya),                                 cos(pa) * sin(ya),                               -sin(pa)          | |evx|
	# |qvy| = | sin(ra) * sin(pa) * cos(ya) - cos(ra) * sin(ya),   sin(ra) * sin(pa) * sin(ya) + cos(ra) * cos(ya),  sin(ra) * cos(pa)| |evy|
	# |qvz|   | cos(ra) * sin(pa) * cos(ya) + sin(ra) * sin(ya),   cos(ra) * sin(pa) * sin(ya) - sin(ra) * cos(ya),  cos(pa) * cos(ra)| |evz|
	#
	#===================================================================================
	c_pa = math.cos(pa)
	s_pa = -math.sin(pa)
	c_ra = math.cos(ra)
	s_ra = math.sin(ra)
	c_ya = math.cos(ya)
	s_ya = math.sin(ya)
	c_ta = math.cos(ta)
	s_ta = math.sin(ta)

	qvx = evx * c_pa * c_ya                        + evy * c_pa * s_ya                        - evz * s_pa
	qvy = evx * (s_ra * s_pa * c_ya - c_ra * s_ya) + evy * (s_ra * s_pa * s_ya + c_ra * c_ya) + evz * s_ra * c_pa
	qvz = evx * (c_ra * s_pa * c_ya + s_ra * s_ya) + evy * (c_ra * s_pa * s_ya - s_ra * c_ya) + evz * c_pa * c_ra

	return qvx, qvy, qvz


############################################################################################
#
# Convert a vector to earth-frame coordingates from quadcopter-frame coordinates.
#
############################################################################################
def Q2EFrame(qvx, qvy, qvz, pa, ra, ya, ta):
	global use_tilt

	#==================================================================================
	# We need an inverse of the Earth- to quadcopter-frame matrix above.  It is only
	# needed for accurate gravity calculation at take-off.  For that reason, yaw can be
	# omitted making it simpler. However the method used could equally well be used in
	# a context requiring yaw to be included.
	#
	# Earth to quadcopter rotation matrix
	# -----------------------------------
	# | c_pa * c_ya,                                 c_pa * s_ya,                -s_pa    |
	# | s_ra * s_pa * c_ya - c_ra * s_ya,   s_ra * s_pa * s_ya + c_ra * c_ya,  s_ra * c_pa|
	# | c_ra * s_pa * c_ya + s_ra * s_ya,   c_ra * s_pa * s_ya - s_ra * c_ya,  c_pa * c_ra|
	#
	# Remove yaw
	# ----------
	# | c_pa,            0,     -s_pa    |
	# | s_ra * s_pa,   c_ra,  s_ra * c_pa|
	# | c_ra * s_pa,  -s_ra,  c_pa * c_ra|
	#
	# Transpose
	# ---------
	# |  c_pa, s_ra * s_pa,  c_ra * s_pa |
	# |   0,       c_ra,        -s_ra    |
	# | -s_pa, s_ra * c_pa,  c_pa * c_ra |
	#
	# Check by multiplying
	# --------------------
	# | c_pa,            0,     -s_pa    ||  c_pa, s_ra * s_pa,  c_ra * s_pa |
	# | s_ra * s_pa,   c_ra,  s_ra * c_pa||   0,       c_ra,        -s_ra    |
	# | c_ra * s_pa,  -s_ra,  c_pa * c_ra|| -s_pa, s_ra * c_pa,  c_pa * c_ra |
	#
	# Row 1, Column 1
	# ---------------
	# c_pa * c_pa + s_pa * s_pa = 1
	#
	# Row 1, Column 2
	# ---------------
	# c_pa * s_pa * s_ra -s_pa * s_ra * c_pa  = 0
	#
	# Row 1, Column 3
	# ---------------
	# c_pa * c_ra * s_pa - s_pa * c_pa * c_ra = 0
	#
	# Row 2, Column 1
	# ---------------
	# s_ra * s_pa * c_pa - s_ra * c_pa * s_pa = 0
	#
	# Row 2, Column 2
	# ---------------
	# s_ra * s_pa * s_ra * s_pa + c_ra * c_ra + s_ra * c_pa * s_ra * c_pa =
	# s_ra^2 * s_pa^2 + c_ra^2 + s_ra^2 * c_pa^2 =
	# s_ra^2 * (s_pa^2 + c_pa^2) + c_ra^2 =
	# s_ra^2 + c_ra^2 = 1
	#
	# Row 2, Column 3
	# ---------------
	# s_ra * s_pa * c_ra * s_pa - c_ra * s_ra + s_ra * c_pa * c_pa * c_ra =
	# (s_ra * c_ra * (s_pa^2 - 1 + c_pa^2) = 0
	#
	# Row 3, Column 1
	# ---------------
	# c_ra * s_pa * c_pa - c_pa * c_ra * s_pa = 0
	#
	# Row 3, Column 2
	# ---------------
	# c_ra * s_pa * s_ra * s_pa -s_ra * c_ra + c_pa * c_ra * s_ra * c_pa =
	# (c_ra * s_ra) * (s_pa^2 - 1 + c_pa^2) = 0
	#
	# Row 3, Column 3
	# ---------------
	# c_ra * s_pa * c_ra * s_pa + s_ra * s_ra + c_pa * c_ra * c_pa * c_ra =
	# c_ra^2 * s_pa^2 + s_ra^2 + c_pa^2 * c_ra^2 =
	# c_ra^2 * (s_pa^2 + c_pa^2) + s_ra^2 =
	# c_ra^2 + s_ra^2 = 1
	#===================================================================================
	c_pa = math.cos(pa)
	s_pa = -math.sin(pa)
	c_ra = math.cos(ra)
	s_ra = math.sin(ra)
	c_ya = math.cos(ya)
	s_ya = math.sin(ya)
	c_ta = math.cos(ta)
	s_ta = math.sin(ta)

	evx = qvx * c_pa * c_ya + qvy * (s_ra * s_pa * c_ya - c_ra * s_ya) + qvz * (c_ra * s_pa * c_ya + s_ra * s_ya)
	evy = qvx * c_pa * s_ya + qvy * (s_ra * s_pa * s_ya + c_ra * c_ya) + qvz * (c_ra * s_pa * s_ya - s_ra * c_ya)
	evz = -qvx * s_pa       + qvy *  s_ra * c_pa                       + qvz * c_pa * c_ra

	return evx, evy, evz


############################################################################################
#
# GPIO pins initialization for MPU6050 interrupt, sounder and hardware PWM
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
	logger.info('Setup MPU6050 interrupt input %s', RPIO_DATA_READY_INTERRUPT)
	RPIO.setup(RPIO_DATA_READY_INTERRUPT, RPIO.IN) # , RPIO.PUD_DOWN)

	#-----------------------------------------------------------------------------------
	# Set up the globally shared single PWM channel
	#-----------------------------------------------------------------------------------
	PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
	PWM.setup(1)                                    # 1us increment
	PWM.init_channel(RPIO_DMA_CHANNEL, 3000)        # 3ms carrier period

############################################################################################
#
# GPIO pins cleanup for MPU6050 interrupt, sounder and hardware PWM
#
############################################################################################
def RpioCleanup():
	PWM.cleanup()
	RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)
	RPIO.edge_detect_term(RPIO_DATA_READY_INTERRUPT)
	RPIO.cleanup()


############################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
############################################################################################
def CheckCLI(argv):
	cli_fly = False
	cli_calibrate_sensors = False
	cli_video = False

	cli_hover_target = 550

	#-----------------------------------------------------------------------------------
	# Defaults for vertical velocity PIDs
	#-----------------------------------------------------------------------------------
	cli_vvp_gain = 300.0
	cli_vvi_gain = 150.0
	cli_vvd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Defaults for horizontal velocity PIDs
	#-----------------------------------------------------------------------------------
	cli_hvp_gain = 0.5
	cli_hvi_gain = 0.25
	cli_hvd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Defaults for pitch rate PIDs
	#-----------------------------------------------------------------------------------
	cli_prp_gain = 120.0
	cli_pri_gain = 0.0
	cli_prd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Defaults for roll rate PIDs
	#-----------------------------------------------------------------------------------
	cli_rrp_gain = 100.0
	cli_rri_gain = 0.0
	cli_rrd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Other configuration defaults
	#-----------------------------------------------------------------------------------
	cli_test_case = 0
	cli_tau = 0.5
	cli_dlpf = 4
	cli_jitter = 0
	cli_diagnostics = False
	cli_motion_frequency = 25
	cli_rtf_period = 1.0

	hover_target_defaulted = True
	no_drift_control = False
	prp_set = False
	pri_set = False
	prd_set = False
	rrp_set = False
	rri_set = False
	rrd_set = False

	#-----------------------------------------------------------------------------------
	# Right, let's get on with reading the command line and checking consistency
	#-----------------------------------------------------------------------------------
	try:
		opts, args = getopt.getopt(argv,'dfcvh:j:m:r:', ['tc=', 'vvp=', 'vvi=', 'vvd=', 'hvp=', 'hvi=', 'hvd=', 'prp=', 'pri=', 'prd=', 'rrp=', 'rri=', 'rrd=', 'tau=', 'dlpf='])
	except getopt.GetoptError:
		logger.critical('Must specify one of -f or -c or --tc')
		logger.critical('  qcpi.py [-f] [-t speed] [-c] [-v]')
		logger.critical('  -f set whether to fly')
		logger.critical('  -h set the hover speed for manual testing')
		logger.critical('  -c calibrate sensors against temperature and save')
		logger.critical('  -d enable diagnostics')
		logger.critical('  -v video the flight')
		logger.critical('  -m ??  set motion processing update frequency')
		logger.critical('  -j ??  set motion processing frequency jitter')
		logger.critical('  -r ??  set the ready-to-fly period')
		logger.critical('  --vvp  set vertical speed PID P gain')
		logger.critical('  --vvi  set vertical speed PID P gain')
		logger.critical('  --vvd  set vertical speed PID P gain')
		logger.critical('  --hvp  set horizontal speed PID P gain')
		logger.critical('  --hvi  set horizontal speed PID I gain')
		logger.critical('  --hvd  set horizontal speed PID D gain')
		logger.critical('  --prp  set pitch rotation PID P gain')
		logger.critical('  --pri  set pitch rotation PID I gain')
		logger.critical('  --prd  set pitch rotation PID D gain')
		logger.critical('  --rrp  set roll rotation PID P gain')
		logger.critical('  --rri  set roll rotation PID I gain')
		logger.critical('  --rrd  set roll rotation PID D gain')
		logger.critical('  --tc   select which testcase to run')
		logger.critical('  --tau  set the complementary filter period')
		logger.critical('  --dlpf set the digital low pass filter')
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-f':
			cli_fly = True

		elif opt in '-h':
			cli_hover_target = int(arg)
			hover_target_defaulted = False

		elif opt in '-v':
			cli_video = True

		elif opt in '-c':
			cli_calibrate_sensors = True

		elif opt in '-j':
			cli_jitter = int(arg)

		elif opt in '-m':
			cli_motion_frequency = int(arg)
	
		elif opt in '-d':
			cli_diagnostics = True

		elif opt in '-r':
			cli_rtf_period = float(arg)
	
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

		elif opt in '--prp':
			cli_prp_gain = float(arg)
			prp_set = True

		elif opt in '--pri':
			cli_pri_gain = float(arg)
			pri_set = True

		elif opt in '--prd':
			cli_prd_gain = float(arg)
			prd_set = True

		elif opt in '--rrp':
			cli_rrp_gain = float(arg)
			rrp_set = True

		elif opt in '--rri':
			cli_rri_gain = float(arg)
			rri_set = True

		elif opt in '--rrd':
			cli_rrd_gain = float(arg)
			rrd_set = True

		elif opt in '--tc':
			cli_test_case = int(arg)

		elif opt in '--tau':
			cli_tau = float(arg)

		elif opt in '--dlpf':
			cli_dlpf = int(arg)

	if not cli_calibrate_sensors and not cli_fly and cli_test_case == 0:
		logger.critical('Must specify one of -f, -c or --tc')
		sys.exit(2)

	elif not cli_calibrate_sensors and (cli_hover_target < 0 or cli_hover_target > 1000):
		logger.critical('Hover speed must lie in the following range')
		logger.critical('0 <= test speed <= 1000')
		sys.exit(2)

	elif cli_test_case == 0 and cli_fly:
		logger.critical('Pre-flight checks passed, enjoy your flight, sir!')

	elif cli_test_case == 0 and cli_calibrate_sensors:
		logger.critical('Calibrate sensors is it, sir!')

	elif cli_test_case == 0:
		logger.critical('You must specify flight (-f) or gravity calibration (-c)')
		sys.exit(2)

	elif cli_fly or cli_calibrate_sensors:
		logger.critical('Choose a specific test case (--tc) or fly (-f) or calibrate gravity (-g)')
		sys.exit(2)

	#---------------------------------------------------------------------------------------
	# Test case 1: Check all the blades work and spin in the right direction
	#---------------------------------------------------------------------------------------
	elif cli_test_case != 1:
		logger.critical('Only testcase 1 is valid')
		sys.exit(2)

	elif hover_target_defaulted:
		logger.critical('You must choose a specific hover speed (-h) for all test cases.')
		sys.exit(2)


	return cli_calibrate_sensors, cli_fly, cli_hover_target, cli_video, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_prp_gain, cli_pri_gain, cli_prd_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_test_case, cli_tau, cli_dlpf, cli_jitter, cli_motion_frequency, cli_rtf_period, cli_diagnostics

############################################################################################
#
# Count down beeper
#
############################################################################################
def CountdownBeep(num_beeps):
	logger.critical('%d...', num_beeps)
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
	# Stop the signal handler
	#-----------------------------------------------------------------------------------
	signal.signal(signal.SIGINT, signal.SIG_IGN)

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
	RpioCleanup()

	#-----------------------------------------------------------------------------------
	# Unlock memory we've used from RAM
	#-----------------------------------------------------------------------------------
	munlockall()

	#-----------------------------------------------------------------------------------
	# Reset the signal handler to default
	#-----------------------------------------------------------------------------------
	signal.signal(signal.SIGINT, signal.SIG_DFL)

	sys.exit(0)

############################################################################################
#
# Signal handler for Ctrl-C => next FSM update if running else stop
#
############################################################################################
def SignalHandler(signal, frame):
	global keep_looping

	if loop_count > 0:
		keep_looping = False
	else:
		CleanShutdown()

############################################################################################
#
# Flight plan management
#
############################################################################################
class FlightPlan:

	#-----------------------------------------------------------------------------------
	# The flight plan - move to file at some point
	#-----------------------------------------------------------------------------------
	fp_evx_target  = [0.0,       0.0,       0.0,       0.0,       0.0]
	fp_evy_target  = [0.0,       0.0,       0.0,       0.0,       0.0]
	fp_evz_target  = [0.0,       0.3,       0.0,      -0.3,       0.0]
	fp_time        = [0.0,       3.0,       3.0,       3.0,       0.0]
	fp_name        = ["RTF",  "ASCENT",   "HOVER", "DESCENT",    "STOP"]
	_FP_STEPS = 4

	#-----------------------------------------------------------------------------------
	# Flight plan step transition FSM constants
	#-----------------------------------------------------------------------------------
	_FSM_INPUT_NONE = 0
	_FSM_INPUT_UPDATE = 1
	_FSM_INPUT_STOP = 2

	_FSM_STATE_OFF = 0
	_FSM_STATE_UPDATING = 1
	_FSM_STATE_STABLE = 2

	def __init__(self, time_now):

		self.fsm_input = self._FSM_INPUT_UPDATE
		self.fsm_state = self._FSM_STATE_STABLE

		self.update_start = 0.0

		self.fp_total_time = 0.0
		self.update_start = 0.0
		self.update_total_time = 0.0

		self.prev_evx_target = 0.0
		self.prev_evy_target = 0.0
		self.prev_evz_target = 0.0
		self.evx_target = 0.0
		self.evy_target = 0.0
		self.evz_target = 0.0
		self.next_evx_target = 0.0
		self.next_evy_target = 0.0
		self.next_evz_target = 0.0

		self.fp_index = 0
		self.fp_total_time = 0.0
		self.start_time = time_now


	def getTargets(self, time_now):
		global keep_looping

		elapsed_time = time_now - self.start_time

		#---------------------------------------------------------------------------
		# Work out the next input to the FSM - this mostly just depends on time
		#---------------------------------------------------------------------------
		if self.fsm_state == self._FSM_STATE_STABLE and (elapsed_time >= self.fp_total_time + self.update_total_time):
				self.fp_index += 1

				logger.critical('-> %s', self.fp_name[self.fp_index])
				self.fsm_input = self._FSM_INPUT_UPDATE
				self.next_evx_target = self.fp_evx_target[self.fp_index]
				self.next_evy_target = self.fp_evy_target[self.fp_index]
				self.next_evz_target = self.fp_evz_target[self.fp_index]

				self.prev_evx_target = self.fp_evx_target[self.fp_index - 1]
				self.prev_evy_target = self.fp_evy_target[self.fp_index - 1]
				self.prev_evz_target = self.fp_evz_target[self.fp_index - 1]

				self.fp_total_time += self.fp_time[self.fp_index]

				if self.fp_index == self._FP_STEPS:
					self.fsm_input = self._FSM_INPUT_STOP

		#---------------------------------------------------------------------------
		# Now we've decided the next input, apply it
		#---------------------------------------------------------------------------

		if self.fsm_state == self._FSM_STATE_STABLE and self.fsm_input == self._FSM_INPUT_UPDATE:
			self.fsm_state = self._FSM_STATE_UPDATING
			self.next_fsm_state = self._FSM_STATE_STABLE
			self.fsm_input = self._FSM_INPUT_NONE


		if self.fsm_state == self._FSM_STATE_UPDATING and self.fsm_input == self._FSM_INPUT_NONE:
			if self.update_start == 0.0:
				self.update_start = time_now
				self.fsm_update_period = 2.0 * math.copysign(self.next_evz_target - self.prev_evz_target, 1.0) + 0.2

			update_fraction = (time_now - self.update_start) / self.fsm_update_period

#        		#-----------------------AUTONOMOUS BINARY TRANSITION------------------------
#        		self.evx_target =  self.next_evx_target
#        		self.evy_target =  self.next_evy_target
#        		self.evz_target =  self.next_evz_target
#        		#-----------------------AUTONOMOUS BINARY TRANSITION------------------------


#        		#-----------------------AUTONOMOUS LINEAR TRANSITION------------------------
#        		self.evx_target = self.prev_evx_target + update_fraction * (self.next_evx_target - self.prev_evx_target)
#        		self.evy_target = self.prev_evy_target + update_fraction * (self.next_evy_target - self.prev_evy_target)
#        		self.evz_target = self.prev_evz_target + update_fraction * (self.next_evz_target - self.prev_evz_target)
#        		#-----------------------AUTONOMOUS LINEAR TRANSITION------------------------

			#-----------------------AUTONOMOUS SINUSOIDAL TRANSITION--------------------
			signed_one = math.copysign(1.0, self.next_evx_target - self.prev_evx_target)
			self.evx_target =  self.prev_evx_target + signed_one * 0.5 * (self.next_evx_target - self.prev_evx_target) * (signed_one + math.sin(signed_one * (2 * update_fraction - 1) * math.pi / 2))
			signed_one = math.copysign(1.0, self.next_evy_target - self.prev_evy_target)
			self.evy_target =  self.prev_evy_target + signed_one * 0.5 * (self.next_evy_target - self.prev_evy_target) * (signed_one + math.sin(signed_one * (2 * update_fraction - 1) * math.pi / 2))
			signed_one = math.copysign(1.0, self.next_evz_target - self.prev_evz_target)
			self.evz_target =  self.prev_evz_target + signed_one * 0.5 * (self.next_evz_target - self.prev_evz_target) * (signed_one + math.sin(signed_one * (2 * update_fraction - 1) * math.pi / 2))
			#-----------------------AUTONOMOUS SINUSOIDAL TRANSITION--------------------

			if update_fraction >= 1.00:
				#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
				self.evx_target = self.next_evx_target
				self.evy_target = self.next_evy_target
				self.evz_target = self.next_evz_target
				#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------

				logger.critical('@ %s', self.fp_name[self.fp_index])
				self.fsm_state = self._FSM_STATE_STABLE
				self.fsm_input = self._FSM_INPUT_NONE
				self.update_total_time += time_now - self.update_start
				self.update_start = 0.0

		if self.fsm_state == self._FSM_STATE_UPDATING and self.fsm_input == self._FSM_INPUT_UPDATE:

			#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
			self.evx_target = self.next_evx_target
			self.evy_target = self.next_evy_target
			self.evz_target = self.next_evz_target
			#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------

			logger.critical('@ %s', self.fp_name[fp_index])
			self.fsm_state = self._FSM_STATE_STABLE
			self.fsm_input = self._FSM_INPUT_NONE
			self.update_total_time += time_now - self.update_start
			self.update_start = 0.0

		if self.fsm_input == self._FSM_INPUT_STOP:
			keep_looping = False


		return self.evx_target, self.evy_target, self.evz_target

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
# Set the BCM output / intput assigned to LED and sensor interrupt respectively
#-------------------------------------------------------------------------------------------
RPIO_DMA_CHANNEL = 1
RPIO_STATUS_SOUNDER = 27
RPIO_DATA_READY_INTERRUPT = 25

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

file_formatter = logging.Formatter('[%(levelname)s] (%(threadName)-10s) %(funcName)s %(lineno)d, %(message)s')
file_handler.setFormatter(file_formatter)

#-------------------------------------------------------------------------------------------
# Add both handlers to the logger
#-------------------------------------------------------------------------------------------
logger.addHandler(console_handler)
logger.addHandler(file_handler)

#-------------------------------------------------------------------------------------------
# Check the command line for calibration or flight parameters
#-------------------------------------------------------------------------------------------
calibrate_sensors, flying, hover_target, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, test_case, tau, dlpf, jitter, motion_frequency, rtf_period, diagnostics = CheckCLI(sys.argv[1:])
logger.critical("calibrate_sensors = %s, fly = %s, hover_target = %d, shoot_video = %s, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, prp_gain = %f, pri_gain = %f, prd_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, test_case = %d, tau = %f, dlpf = %d, jitter = %d, motion_frequency = %f, rtf_period = %f, diagnostics = %s", calibrate_sensors, flying, hover_target, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, test_case, tau, dlpf, jitter, motion_frequency, rtf_period, diagnostics)

#-------------------------------------------------------------------------------------------
# Initialize the motion processing period and jitter
#-------------------------------------------------------------------------------------------
motion_period = 1 / motion_frequency
if jitter != 0:
	motion_period *= (1 + random.uniform(-jitter, +jitter) / 100)

#-------------------------------------------------------------------------------------------
# Set up the global constants
# - gravity in meters per second squared
# - accelerometer in g's
# - gyroscope in radians per second
#-------------------------------------------------------------------------------------------
GRAV_ACCEL = 9.80665
SCALE_GYRO = 500.0 * math.pi / (65536 * 180)
SCALE_ACCEL = 4.0 / 65536


#-------------------------------------------------------------------------------------------
# Enable RPIO for beeper, MPU 6050 interrupts and PWM
#-------------------------------------------------------------------------------------------
RpioSetup()

#-------------------------------------------------------------------------------------------
# Set up the ESC to GPIO pin and location mappings and assign to each ESC
#-------------------------------------------------------------------------------------------
ESC_BCM_BL = 22
ESC_BCM_FL = 17
ESC_BCM_FR = 18
ESC_BCM_BR = 23

MOTOR_LOCATION_FRONT = 0b00000001
MOTOR_LOCATION_BACK =  0b00000010
MOTOR_LOCATION_LEFT =  0b00000100
MOTOR_LOCATION_RIGHT = 0b00001000

MOTOR_ROTATION_CW = 1
MOTOR_ROTATION_ACW = 2

pin_list = [ESC_BCM_FL, ESC_BCM_FR, ESC_BCM_BL, ESC_BCM_BR]
location_list = [MOTOR_LOCATION_FRONT | MOTOR_LOCATION_LEFT, MOTOR_LOCATION_FRONT | MOTOR_LOCATION_RIGHT, MOTOR_LOCATION_BACK | MOTOR_LOCATION_LEFT, MOTOR_LOCATION_BACK | MOTOR_LOCATION_RIGHT]
rotation_list = [MOTOR_ROTATION_ACW, MOTOR_ROTATION_CW, MOTOR_ROTATION_CW, MOTOR_ROTATION_ACW]
name_list = ['front left', 'front right', 'back left', 'back right']

#-------------------------------------------------------------------------------------------
# Prime the ESCs with the default 0 spin rotors to shut them up.
#-------------------------------------------------------------------------------------------
esc_list = []
for esc_index in range(0, 4):
	esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
	esc_list.append(esc)

#-------------------------------------------------------------------------------------------
# Initialize the gyroscope / accelerometer I2C object
#-------------------------------------------------------------------------------------------
mpu6050 = MPU6050(0x68, dlpf)

#-------------------------------------------------------------------------------------------
# Countdown: 5 beeps prior to gyro calibration
#-------------------------------------------------------------------------------------------
CountdownBeep(5)

#-------------------------------------------------------------------------------------------
# Calibrate the sensors to build a trend line for sensor offsets against temperature
#-------------------------------------------------------------------------------------------
if calibrate_sensors:
	mpu6050.calibrateGravity("./qcoffsets.csv")
	sys.exit(0)

#-------------------------------------------------------------------------------------------
# Calibrate gyros - this is a one-off
#-------------------------------------------------------------------------------------------
mpu6050.calibrateGyros()

#-------------------------------------------------------------------------------------------
# Countdown: 4 beeps prior calculating take-off platform tilt
#-------------------------------------------------------------------------------------------
CountdownBeep(4)

#-------------------------------------------------------------------------------------------
# Measure average gravity distribution across the quadframe
#-------------------------------------------------------------------------------------------
qax_integrated = 0.0
qay_integrated = 0.0
qaz_integrated = 0.0

loop_count = 0
while loop_count != 1000:
	qax, qay, qaz, qgx, qgy, qgz = mpu6050.readSensors()

	loop_count += 1

	qax_integrated += qax
	qay_integrated += qay
	qaz_integrated += qaz

#-------------------------------------------------------------------------------------------
# Work out the average acceleration due to gravity
# Save off the quad-frame raw gravity vector
#-------------------------------------------------------------------------------------------
qfrgv_x = qax_integrated * SCALE_ACCEL / loop_count
qfrgv_y = qay_integrated * SCALE_ACCEL / loop_count
qfrgv_z = qaz_integrated * SCALE_ACCEL / loop_count

qfrev_x = 0.0
qfrev_y = 0.0
qfrev_z = 0.0

#-------------------------------------------------------------------------------------------
# Iteratively refine measurements for offsets and angles
#-------------------------------------------------------------------------------------------
num_iterations = 1
logger.critical("qfrgv_x, qfrgv_y, qfrgv_z, qfrgv_pitch, qfrgv_roll, qfrgv_tilt, efrgv_x, efrgv_y, efrgv_z, etrev_x, efrev_y, efrev_z, qfrev_x, qfrev_y, qfrev_z")
for iteration in range(0, num_iterations):
	#-------------------------------------------------------------------------------------------
	# Get the take-off platform slope
	#-------------------------------------------------------------------------------------------
	qfrgv_pitch, qfrgv_roll, qfrgv_tilt = GetEulerAngles(qfrgv_x - qfrev_x, qfrgv_y - qfrev_y, qfrgv_z - qfrev_z)

	#-------------------------------------------------------------------------------------------
	# Find better values for the qfrev_* from the revised angles
	#-------------------------------------------------------------------------------------------
	efrgv_x, efrgv_y, efrgv_z = Q2EFrame(qfrgv_x, qfrgv_y, qfrgv_z, qfrgv_pitch, qfrgv_roll, 0.0, qfrgv_tilt)
	efrev_x = efrgv_x - 0.0
	efrev_y = efrgv_y - 0.0
	efrev_z = 0.0
#AB:	efrev_z = efrgv_z - GRAV_ACCEL

	qfrev_x, qfrev_y, qfrev_z = E2QFrame(efrev_x, efrev_y, efrev_z, qfrgv_pitch, qfrgv_roll, 0.0, qfrgv_tilt)
	logger.critical("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", qfrgv_x, qfrgv_y, qfrgv_z, qfrgv_pitch, qfrgv_roll, qfrgv_tilt, efrgv_x, efrgv_y, efrgv_z, efrev_x, efrev_y, efrev_z, qfrev_x, qfrev_y, qfrev_z)

#-------------------------------------------------------------------------------------------
# Dump the variety of sensor misses
#-------------------------------------------------------------------------------------------
mpu6050_misses, i2c_misses = mpu6050.getMisses()
logger.critical("mpu6050 %d misses, i2c %d misses", mpu6050_misses, i2c_misses)

#-------------------------------------------------------------------------------------------
# Initialize complementary filter angles
#-------------------------------------------------------------------------------------------
prev_c_pitch = qfrgv_pitch
prev_c_roll = qfrgv_roll
i_pitch = qfrgv_pitch
i_roll = qfrgv_roll
i_yaw = 0.0

#-------------------------------------------------------------------------------------------
# Cleanup integration variables
#-------------------------------------------------------------------------------------------
qax_integrated = 0.0
qay_integrated = 0.0
qaz_integrated = 0.0

qgx_integrated = 0.0
qgy_integrated = 0.0
qgz_integrated = 0.0

start_integrating = True

#-------------------------------------------------------------------------------------------
# Countdown: 3 beeps prior to setting up the interrupt handler
#-------------------------------------------------------------------------------------------
CountdownBeep(3)

#-------------------------------------------------------------------------------------------
# Set the signal handler here so the core processing loop can be stopped (or not started) by
# Ctrl-C.
#---------------------------------------------------------------------------
loop_count = 0
signal.signal(signal.SIGINT, SignalHandler)

#-------------------------------------------------------------------------------------------
# Countdown: 2 beeps prior to starting the video
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
# Countdown: 1 beep to get those blades spinning
#-------------------------------------------------------------------------------------------
CountdownBeep(1)

#------------------------------------------------------------------------------------------
# Set up the bits of state setup before takeoff
#-------------------------------------------------------------------------------------------
keep_looping = True

evx_target = 0.0
evy_target = 0.0
evz_target = 0.0

qvx_input = 0.0
qvy_input = 0.0
qvz_input = 0.0

pr_target = 0.0
rr_target = 0.0
yr_target = 0.0

ya_target = 0.0

qvx_diags = "0.0, 0.0, 0.0"
qvy_diags = "0.0, 0.0, 0.0"
qvz_diags = "0.0, 0.0, 0.0"
pr_diags = "0.0, 0.0, 0.0"
rr_diags = "0.0, 0.0, 0.0"
yr_diags = "0.0, 0.0, 0.0"

hover_speed = 0
ready_to_fly = False

#-------------------------------------------------------------------------------------------
# START TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the right way
#-------------------------------------------------------------------------------------------
if test_case == 1:
	for esc in esc_list:
		for count in range(0, hover_target, 10):
			#-------------------------------------------------------------------
			# Spin up to user determined (-h) hover speeds ~200
			#-------------------------------------------------------------------
			esc.update(count)
			time.sleep(0.01)
		time.sleep(10.0)
		esc.update(0)
	CleanShutdown()
#-------------------------------------------------------------------------------------------
# END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the right way
#-------------------------------------------------------------------------------------------

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
# - Horizontal speed error is corrected by providing the as the horizontal acceleration target
# - The horizontal acceleration target is converted to a required rotation rate
# - Rotation rate error is corrected proportionally by providing the corrective output to the motors' ESCs
#
# - Horizontal speed feedback is provided by integrated accelerometer (or GPS in future)
# - Angular speed feedback comes directly from the gyros
#
# That's 2 PIDs each for horizontal X & Y axes movement - 4 in total so far.
#
# - Vertical speed error is corrected proportionally by providing the corrective output to the motors' ESCs (cf. angular speed above)
#
# - Vertical speed feedback comes from the Z-axis accelerometer integrated over time, and compensated for any tilt (cos(theta)cos(phi))
#
# That's 1 PID for vertical Z axis speed control.
#
# - Yaw speed error is corrected proportionally by providing the corrective output to the motors' ESCs
#
# - Yaw speed feedback comes directly from the Z-axis gyro
#
# That's 2 PID for Z axis yaw control.
#
# So 7 PIDs in total
#===========================================================================================

#-------------------------------------------------------------------------------------------
# The earth X axis speed controls forward / backward speed
#-------------------------------------------------------------------------------------------
PID_QVX_P_GAIN = hvp_gain
PID_QVX_I_GAIN = hvi_gain
PID_QVX_D_GAIN = hvd_gain	

#-------------------------------------------------------------------------------------------
# The earth Y axis speed controls left / right speed
#-------------------------------------------------------------------------------------------
PID_QVY_P_GAIN = hvp_gain
PID_QVY_I_GAIN = hvi_gain
PID_QVY_D_GAIN = hvd_gain	

#-------------------------------------------------------------------------------------------
# The earth Z axis speed controls rise / fall speed
#-------------------------------------------------------------------------------------------
PID_QVZ_P_GAIN = vvp_gain
PID_QVZ_I_GAIN = vvi_gain
PID_QVZ_D_GAIN = vvd_gain

#-------------------------------------------------------------------------------------------
# The YAW ANGLE PID maintains a stable rotation angle about the Z-axis
#-------------------------------------------------------------------------------------------
PID_YA_P_GAIN = 6.0
PID_YA_I_GAIN = 3.0
PID_YA_D_GAIN = 1.0

#-------------------------------------------------------------------------------------------
# The PITCH RATE PID controls stable rotation rate around the Y-axis
#-------------------------------------------------------------------------------------------
PID_PR_P_GAIN = prp_gain
PID_PR_I_GAIN = pri_gain
PID_PR_D_GAIN = prd_gain

#-------------------------------------------------------------------------------------------
# The ROLL RATE PID controls stable rotation rate around the X-axis
#-------------------------------------------------------------------------------------------
PID_RR_P_GAIN = rrp_gain
PID_RR_I_GAIN = rri_gain
PID_RR_D_GAIN = rrd_gain

#-------------------------------------------------------------------------------------------
# The YAW RATE PID controls stable rotation speed around the Z-axis
#-------------------------------------------------------------------------------------------
PID_YR_P_GAIN = rrp_gain / 2.0
PID_YR_I_GAIN = rri_gain / 2.0
PID_YR_D_GAIN = rrd_gain / 2.0

logger.critical('Thunderbirds are go!')

#-------------------------------------------------------------------------------------------
# Diagnostic log header
#-------------------------------------------------------------------------------------------
if diagnostics:
	logger.warning('time, dt, loop, qgx, qgy, qgz, efrgv_x, efrgv_y, efrgv_z, qfrev_x, qfrev_y, qfrev_z, qax, qay, qaz, qfrgv_x, qfrgv_y, qfrgv_z, qvx_input, qvy_input, qvz_input, i pitch, i roll, e pitch, e roll, c pitch, c roll, i yaw, e tilt, evx_target, qvx_target, qxp, qxi, qxd, pr_target, prp, pri, prd, pr_out, evy_yarget, qvy_target, qyp, qyi, qyd, rr_target, rrp, rri, rrd, rr_out, evz_target, qvz_target, qzp, qzi, qzd, qvz_out, yr_target, yrp, yri, yrd, yr_out, FL spin, FR spin, BL spin, BR spin')

#==========================================================================================
# Initialize critical timing immediately before starting the PIDs
#==========================================================================================
time_now = time.time()

#-------------------------------------------------------------------------------------------
# Start the yaw absolute angle PID
#-------------------------------------------------------------------------------------------
ya_pid = PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN, time_now)

#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw rate PIDs
#-------------------------------------------------------------------------------------------
pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN, time_now)
rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN, time_now)
yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN, time_now)

#-------------------------------------------------------------------------------------------
# Start the X, Y (horizontal) and Z (vertical) velocity PIDs
#-------------------------------------------------------------------------------------------
qvx_pid = PID(PID_QVX_P_GAIN, PID_QVX_I_GAIN, PID_QVX_D_GAIN, time_now)
qvy_pid = PID(PID_QVY_P_GAIN, PID_QVY_I_GAIN, PID_QVY_D_GAIN, time_now)
qvz_pid = PID(PID_QVZ_P_GAIN, PID_QVZ_I_GAIN, PID_QVZ_D_GAIN, time_now)

start_time = time_now
last_log_time = time_now
prev_sample_time = time_now
last_motion_update = time_now
elapsed_time = 0.0

while keep_looping:
	#===================================================================================
	# Sensors: Read the sensor values; note that this also sets the time_now to be as
	# accurate a time stamp for the sensor data as possible.
	#===================================================================================
	qax, qay, qaz, qgx, qgy, qgz = mpu6050.readSensors()

	#-----------------------------------------------------------------------------------
	# Now we have the sensor snapshot, tidy up the rest of the variable so that processing
	# takes zero time.
	#-----------------------------------------------------------------------------------
	delta_time = time_now - start_time - elapsed_time
	elapsed_time = time_now - start_time
	loop_count += 1

	#===================================================================================
	# Integration: Sensor data is integrated over time, and later averaged to produce
	# smoother yet still accurate acceleration and rotation since the last PID updates.
	#===================================================================================
	if start_integrating:
		start_integrating = False
		integration_start = time_now

		prev_qgx = qgx
		prev_qgy = qgy
		prev_qgz = qgz
		prev_qax = qax
		prev_qay = qay
		prev_qaz = qaz
	else:
		#---------------------------------------------------------------------------
		# Integrate the gyros readings.
		#---------------------------------------------------------------------------
		qgx_integrated += (prev_qgx + qgx) * delta_time
		qgy_integrated += (prev_qgy + qgy) * delta_time
		qgz_integrated += (prev_qgz + qgz) * delta_time

		prev_qgx = qgx
		prev_qgy = qgy
		prev_qgz = qgz

		#---------------------------------------------------------------------------
		# Integrate the accelerometer readings.
		#---------------------------------------------------------------------------
		qax_integrated += (prev_qax + qax) * delta_time
		qay_integrated += (prev_qay + qay) * delta_time
		qaz_integrated += (prev_qaz + qaz) * delta_time

		prev_qax = qax
		prev_qay = qay
		prev_qaz = qaz

	#===================================================================================
	# Motion Processing:  Use the recorded data to produce motion data and feed in the motion PIDs
	#===================================================================================
	if time_now - last_motion_update >= motion_period:
		last_motion_update += motion_period

		#----------------------------------------------------------------------------------
		# Work out the average acceleration and rotation rate
		#----------------------------------------------------------------------------------
		integration_period = time_now - integration_start
		start_integrating = True

		#----------------------------------------------------------------------------------
		# Sort out units and the double accounting in integration.
		#----------------------------------------------------------------------------------
		qgx_integrated *= SCALE_GYRO / 2
		qgy_integrated *= SCALE_GYRO / 2
		qgz_integrated *= SCALE_GYRO / 2

		qax_integrated *= SCALE_ACCEL / 2
		qay_integrated *= SCALE_ACCEL / 2
		qaz_integrated *= SCALE_ACCEL / 2

		#----------------------------------------------------------------------------------
		# Convert the integrated gyroscope reading back to an averaged gyroscope reading
		#----------------------------------------------------------------------------------
		qgx = qgx_integrated / integration_period
		qgy = qgy_integrated / integration_period
		qgz = qgz_integrated / integration_period

		#----------------------------------------------------------------------------------
		# Convert the integrate accelerometer reading back to an averaged accelerometer reading
		#----------------------------------------------------------------------------------
		qfrav_x = qax_integrated / integration_period
		qfrav_y = qay_integrated / integration_period
		qfrav_z = qaz_integrated / integration_period

		#---------------------------------------------------------------------------
		# Correct the accelerometer readings by subtracing the quad frame error vector
		#---------------------------------------------------------------------------
		qax = qfrav_x - qfrev_x
		qay = qfrav_y - qfrev_y
		qaz = qfrav_z - qfrev_z

		#===================================================================================
		# Angles: Get angles in radians
		#===================================================================================
		e_pitch, e_roll, e_tilt = GetEulerAngles(qax, qay, qaz)

		i_pitch += qgy_integrated
		i_roll += qgx_integrated
		i_yaw += qgz_integrated

		#-----------------------------------------------------------------------------------
		# Apply complementary filter to ensure long-term accuracy of pitch / roll angles
		# 1/tau is the handover frequency that the integrated gyro high pass filter is taken over
		# by the accelerometer Euler low-pass filter providing fast reaction to change from the
		# gyro yet with low noise accurate Euler angles from the acclerometer.
		#
		# The combination of tau plus the time increment provides a fraction to mix
		# the two angles sources.
		#-----------------------------------------------------------------------------------
		tau_fraction = tau / (tau + integration_period)

		c_pitch = tau_fraction * (prev_c_pitch + qgy_integrated) + (1 - tau_fraction) * e_pitch
		prev_c_pitch = c_pitch

		c_roll = tau_fraction * (prev_c_roll + qgx_integrated) + (1 - tau_fraction) * e_roll
		prev_c_roll = c_roll

		#-----------------------------------------------------------------------------------
		# Choose the best measure of the angles
		#-----------------------------------------------------------------------------------
		pa = c_pitch
		ra = c_roll
		ta = e_tilt
		ya = i_yaw

		#----------------------------------------------------------------------------------
		# Reset the averaged values for the next time round
		#----------------------------------------------------------------------------------
		qax_integrated = 0.0
		qay_integrated = 0.0
		qaz_integrated = 0.0

		qgx_integrated = 0.0
		qgy_integrated = 0.0
		qgz_integrated = 0.0

		#-----------------------------------------------------------------------------------
		# Get the curent flight plan targets
		#-----------------------------------------------------------------------------------
		if not ready_to_fly:
			if hover_speed >= hover_target:
				hover_speed = hover_target
				ready_to_fly = True	

				#-------------------------------------------------------------------
				# Register the flight plan with the authorities
				#-------------------------------------------------------------------
				fp = FlightPlan(time_now)

			else:
				hover_speed += int(hover_target * motion_period / rtf_period)

		else:
			evx_target, evy_target, evz_target = fp.getTargets(time_now)

		#-----------------------------------------------------------------------------------
		# Update the time for the next processing loop
		#-----------------------------------------------------------------------------------
		motion_period = 1 / motion_frequency
		if jitter != 0:
			motion_period *= (1 + random.uniform(-jitter, +jitter) / 100)

		#-----------------------------------------------------------------------------------
		# Convert earth-frame velocity targets to quadcopter frame.  This isn't a rotation
		# matrix conversion, simply accounting for the angle from horizontal / vertical.
		#-----------------------------------------------------------------------------------
		qvx_target, qvy_target, qvz_target = E2QFrame(evx_target, evy_target, evz_target, pa, ra, ya, ta)
#AB:		qvx_target = evx_target / math.cos(pa)
#AB:		qvy_target = evy_target / math.cos(ra)
#AB:		qvz_target /= math.cos(ta)

		#-----------------------------------------------------------------------------------
		# Redistribute gravity around the new orientation of the quad
		#-----------------------------------------------------------------------------------
		qfrgv_x, qfrgv_y, qfrgv_z = E2QFrame(efrgv_x, efrgv_y, efrgv_z, pa, ra, ya, ta)
#AB:		qfrgv_x, qfrgv_y, qfrgv_z = E2QFrame(0, 0, 1, pa, ra, ya, ta)

		#-----------------------------------------------------------------------------------
		# Delete reorientated gravity from raw accelerometer readings and sum to make net velocity
		#-----------------------------------------------------------------------------------
		qvx_input += (qfrav_x - qfrgv_x) * integration_period * GRAV_ACCEL
		qvy_input += (qfrav_y - qfrgv_y) * integration_period * GRAV_ACCEL
		qvz_input += (qfrav_z - qfrgv_z) * integration_period * GRAV_ACCEL

		#===========================================================================
		# Motion PIDs: Run the horizontal speed PIDs each rotation axis to determine
		# targets for absolute angle PIDs and the verical speed PID to control height.
		#===========================================================================
		[p_out, i_out, d_out] = qvx_pid.Compute(qvx_input, qvx_target, time_now)
		qvx_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		qvx_out = p_out + i_out + d_out

		[p_out, i_out, d_out] = qvy_pid.Compute(qvy_input, qvy_target, time_now)
		qvy_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		qvy_out =  p_out + i_out + d_out

		[p_out, i_out, d_out] = qvz_pid.Compute(qvz_input, qvz_target, time_now)
		qvz_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		qvz_out = p_out + i_out + d_out

		#---------------------------------------------------------------------------
		# Convert the horizontal velocity PID output i.e. the horizontal acceleration
		# target in q's into the pitch and roll angle PID targets in radians
		#---------------------------------------------------------------------------
#AB:		pr_target = -math.atan2(qvx_out, math.pow(math.pow(qay, 2) + math.pow(qaz, 2), 0.5))
#AB:		rr_target = -math.atan2(qvy_out, math.pow(math.pow(qax, 2) + math.pow(qaz, 2), 0.5))
		pr_target = -qvx_out
		rr_target = -qvy_out

		#---------------------------------------------------------------------------
		# Convert the vertical velocity PID output direct to PWM pulse width.
		#---------------------------------------------------------------------------
		vert_out = hover_speed + int(round(qvz_out))

		#===========================================================================
		# Attitude PIDs: Run the rotation rate PIDs each rotation axis to determine
		# overall PWM output.
		#===========================================================================
		[p_out, i_out, d_out] = ya_pid.Compute(ya, ya_target, time_now)
		ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		yr_target = p_out + i_out + d_out

		[p_out, i_out, d_out] = pr_pid.Compute(qgy, pr_target, time_now)
		pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		pr_out = p_out + i_out + d_out
		[p_out, i_out, d_out] = rr_pid.Compute(qgx, rr_target, time_now)
		rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		rr_out = p_out + i_out + d_out
		[p_out, i_out, d_out] = yr_pid.Compute(qgz, yr_target, time_now)
		yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		yr_out = p_out + i_out + d_out

		#---------------------------------------------------------------------------
		# Convert the rotation rate PID outputs direct to PWM pulse width
		#---------------------------------------------------------------------------
		pr_out = int(round(pr_out / 2))
		rr_out = int(round(rr_out / 2))
		yr_out = int(round(yr_out / 2))

		#===========================================================================
		# PID output distribution: Walk through the ESCs, and apply the PID outputs
		# i.e. the updates PWM pulse widths according to where the ESC is sited on the
		# frame
		#===========================================================================
		for esc in esc_list:
			#-------------------------------------------------------------------
			# Update all blades' power in accordance with the z error
			#-------------------------------------------------------------------
			delta_spin = vert_out

			#-------------------------------------------------------------------
			# For a left downwards roll, the x gyro goes negative, so the PID error is positive,
			# meaning PID output is positive, meaning this needs to be added to the left blades
			# and subtracted from the right.
			#-------------------------------------------------------------------
			if esc.motor_location & MOTOR_LOCATION_RIGHT:
				delta_spin -= rr_out
			else:
				delta_spin += rr_out

			#-------------------------------------------------------------------
			# For a forward downwards pitch, the y gyro goes positive, but is negated
			# in mpu6050.readSensors so it is consistent with the accelerometer +
			# Euler angle calculations.  The PID error is postive as a result,
			# meaning PID output is positive, meaning this needs to be added to the
			# front blades and subtracted from the back.
			#-------------------------------------------------------------------
			if esc.motor_location & MOTOR_LOCATION_BACK:
				delta_spin -= pr_out
			else:
				delta_spin += pr_out

			#-------------------------------------------------------------------
			# For CW yaw, the z gyro goes negative, so the PID error is postitive,
			# meaning PID output is positive, meaning this need to be added to the
			# ACW (FL and BR) blades and subtracted from the CW (FR & BL) blades.
			#-------------------------------------------------------------------
			if esc.motor_rotation == MOTOR_ROTATION_CW:
				delta_spin += yr_out
			else:
				delta_spin -= yr_out

			#-------------------------------------------------------------------
			# Apply the blended outputs to the esc PWM signal
			#-------------------------------------------------------------------
			esc.update(delta_spin)

		#-----------------------------------------------------------------------------------
		# Diagnostic log - every motion loop
		#-----------------------------------------------------------------------------------
		if diagnostics:
			logger.warning('%f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %f, %s, %d, %f, %f, %s, %f, %s, %d, %f, %f, %s, %d, %f, %s, %d, %d, %d, %d, %d', elapsed_time, integration_period, loop_count, qgx, qgy, qgz, efrgv_x, efrgv_y, efrgv_z, qfrev_x, qfrev_y, qfrev_z, qax, qay, qaz, qfrgv_x, qfrgv_y, qfrgv_z, qvx_input, qvy_input, qvz_input, math.degrees(i_pitch), math.degrees(i_roll), math.degrees(e_pitch), math.degrees(e_roll), math.degrees(c_pitch), math.degrees(c_roll), math.degrees(i_yaw), math.degrees(e_tilt), evx_target, qvx_target, qvx_diags, math.degrees(pr_target), pr_diags, pr_out, evy_target, qvy_target, qvy_diags, math.degrees(rr_target), rr_diags, rr_out, evz_target, qvz_target, qvz_diags, qvz_out, yr_target, yr_diags, yr_out, esc_list[0].current_pulse_width, esc_list[1].current_pulse_width, esc_list[2].current_pulse_width, esc_list[3].current_pulse_width)


#-------------------------------------------------------------------------------------------
# Dump the loops per second
#-------------------------------------------------------------------------------------------
logger.critical("loop speed %f loops per second", loop_count / elapsed_time)

#-------------------------------------------------------------------------------------------
# Dump the variety of sensor misses
#-------------------------------------------------------------------------------------------
mpu6050_misses, i2c_misses = mpu6050.getMisses()
logger.critical("mpu6050 %d misses, i2c %d misses", mpu6050_misses, i2c_misses)

#-------------------------------------------------------------------------------------------
# Time for telly bye byes
#-------------------------------------------------------------------------------------------
CleanShutdown()
