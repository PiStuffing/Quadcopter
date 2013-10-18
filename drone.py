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

############################################################################################
#
#  Adafruit i2c interface plus bug fix
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
#  PCA9685 class for setting blade PWM frequencies
#
############################################################################################
class PCA9685 :
	i2c = None

	# Registers/etc.
	__SUBADR1	    = 0x02
	__SUBADR2	    = 0x03
	__SUBADR3	    = 0x04
	__MODE1	      = 0x00
	__PRESCALE	   = 0xFE
	__LED0_ON_L	  = 0x06
	__LED0_ON_H	  = 0x07
	__LED0_OFF_L	 = 0x08
	__LED0_OFF_H	 = 0x09
	__ALLLED_ON_L	= 0xFA
	__ALLLED_ON_H	= 0xFB
	__ALLLED_OFF_L       = 0xFC
	__ALLLED_OFF_H       = 0xFD

	def __init__(self, address=0x40):
		self.i2c = I2C(address)
		self.address = address
		logger.info('Reseting PCA9685')
		self.i2c.write8(self.__MODE1, 0x00)

	def setPCA9685Freq(self, freq):
		"Sets the PWM frequency"
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
					
		logger.info('Setting PWM frequency to %d Hz', freq)
		logger.debug('Estimated pre-scale: %d', prescaleval)

		prescale = math.floor(prescaleval + 0.5)

		logger.info('Final pre-scale: %d', prescale)

		oldmode = self.i2c.readU8(self.__MODE1)
		newmode = (oldmode & 0x7F) | 0x10	     # sleep
		self.i2c.write8(self.__MODE1, newmode)	# go to sleep
		self.i2c.write8(self.__PRESCALE, int(math.floor(prescale)))
		self.i2c.write8(self.__MODE1, oldmode)
		time.sleep(0.005)
		self.i2c.write8(self.__MODE1, oldmode | 0x80)

	def setPCA9685(self, channel, on, off):
		"Sets a single PWM channel"
		self.i2c.write8(self.__LED0_ON_L + 4 * channel, on & 0xFF)
		self.i2c.write8(self.__LED0_ON_H + 4 * channel, on >> 8)
		self.i2c.write8(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
		self.i2c.write8(self.__LED0_OFF_H + 4 * channel, off >> 8)

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
		self.ax_offset = 0
		self.ay_offset = 0
		self.az_offset = 0
		self.gx_offset = 0
		self.gy_offset = 0
		self.gz_offset = 0
		self.sensor_data = array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		self.result_array = array('h', [0, 0, 0, 0, 0, 0, 0])

		logger.info('Reseting MPU-6050')

		#---------------------------------------------------------------------------
		# Reset all registers
		#---------------------------------------------------------------------------
		logger.debug('Reset all registers')
		self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
		time.sleep(0.5)
	
		#---------------------------------------------------------------------------
		# Sets sample rate to 1000/1+4 = 200Hz
		#---------------------------------------------------------------------------
		logger.debug('Sample rate 200Hz')
		self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, 0x04)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Sets clock source to gyro reference w/ PLL
		#---------------------------------------------------------------------------
		logger.debug('Clock gyro PLL')
		self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x02)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Controls frequency of wakeups in accel low power mode plus the sensor standby modes
		#---------------------------------------------------------------------------
		logger.debug('Disable low-power')
		self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_2, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Disable gyro self tests, scale of +/- 500 degrees/s
		#---------------------------------------------------------------------------
		logger.debug('Gyro +/-500 degrees/s')
		self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, 0x08)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Disable accel self tests, scale of +/-2g
		#---------------------------------------------------------------------------
		logger.debug('Accel +/- 2g')
		self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x00)
		time.sleep(0.005)

		#---------------------------------------------------------------------------
		# Setup INT pin to latch and AUX I2C pass through
		#---------------------------------------------------------------------------
		logger.debug('Enable interrupt')
		self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x20)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Enable data ready interrupt
		#---------------------------------------------------------------------------
		logger.debug('Interrupt data ready')
		self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x01)
		time.sleep(0.005)

		#---------------------------------------------------------------------------
		# Disable FSync, 5Hz DLPF => 1kHz sample frequency used above divided by the
		# sample divide factor.
		#---------------------------------------------------------------------------
		logger.debug('5Hz DLPF')
		self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x06)   # 0x05 => 10Hz DLPF
		time.sleep(0.005)
	
		logger.debug('Gumph hereafter...')
	
		#---------------------------------------------------------------------------
		# Freefall threshold of |0mg|
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_FF_THR, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Freefall duration limit of 0
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_FF_DUR, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Motion threshold of 0mg
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_MOT_THR, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Motion duration of 0s
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_MOT_DUR, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Zero motion threshold
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_ZRMOT_THR, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Zero motion duration threshold
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_ZRMOT_DUR, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Disable sensor output to FIFO buffer
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# AUX I2C setup
		# Sets AUX I2C to single master control, plus other config
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_I2C_MST_CTRL, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Setup AUX I2C slaves
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_ADDR, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_REG, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_CTRL, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV1_ADDR, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV1_REG, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV1_CTRL, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV2_ADDR, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV2_REG, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV2_CTRL, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV3_ADDR, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV3_REG, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV3_CTRL, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV4_ADDR, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV4_REG, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV4_DO, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV4_CTRL, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV4_DI, 0x00)
	
		#---------------------------------------------------------------------------
		# Slave out, dont care
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV0_DO, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV1_DO, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV2_DO, 0x00)
		self.i2c.write8(self.__MPU6050_RA_I2C_SLV3_DO, 0x00)
	
		#---------------------------------------------------------------------------
		# More slave config
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Reset sensor signal paths
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_SIGNAL_PATH_RESET, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Motion detection control
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_MOT_DETECT_CTRL, 0x00)
		time.sleep(0.005)
	
		#--------------------------------------------------------------------------
		# Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x00)
		time.sleep(0.005)
	
		#---------------------------------------------------------------------------
		# Data transfer to and from the FIFO buffer
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_FIFO_R_W, 0x00)
		time.sleep(0.005)


	def readSensorsRaw(self):
		#---------------------------------------------------------------------------
		# Clear the interrupt by reading the interrupt status register,
		#---------------------------------------------------------------------------
		self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)

		#---------------------------------------------------------------------------
		# Hard loop on the data ready interrupt until it gets set high
		#---------------------------------------------------------------------------
		while not (self.i2c.readU8(self.__MPU6050_RA_INT_STATUS) == 0x01):
			time.sleep(0.001)
			continue

		#---------------------------------------------------------------------------
		# Disable the interrupt while we read the data
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)

#       	#---------------------------------------------------------------------------
#       	# Read the sensor data, keeping the interrupt latched
#       	#---------------------------------------------------------------------------
#       	ax = self.i2c.readS16(self.__MPU6050_RA_ACCEL_XOUT_H)
#       	ay = self.i2c.readS16(self.__MPU6050_RA_ACCEL_YOUT_H)
#       	az = self.i2c.readS16(self.__MPU6050_RA_ACCEL_ZOUT_H)
#       	gx = self.i2c.readS16(self.__MPU6050_RA_GYRO_XOUT_H)
#       	gy = self.i2c.readS16(self.__MPU6050_RA_GYRO_YOUT_H)
#       	gz = self.i2c.readS16(self.__MPU6050_RA_GYRO_ZOUT_H)
#
#       	self.result_array = [ax, ay, az, 0, gx, gy, gz]

		#---------------------------------------------------------------------------
		# For speed of reading, read all the sensors and parse to USHORTs after
		#---------------------------------------------------------------------------
		sensor_data = self.i2c.readList(self.__MPU6050_RA_ACCEL_XOUT_H, 14)

		for index in range(0, 14, 2):
			if (sensor_data[index] > 127):
				sensor_data[index] -= 256
			self.result_array[int(index / 2)] = (sensor_data[index] << 8) + sensor_data[index + 1]

		#---------------------------------------------------------------------------
		# Reenable the interrupt
		#---------------------------------------------------------------------------
		self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x01)

		return self.result_array


	def readSensors(self):
		#---------------------------------------------------------------------------
		# +/- 2g 2 * 16 bit range for the accelerometer
		# +/- 500 degrees * 16 bit range for the gyroscope
		#---------------------------------------------------------------------------
		[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()
		fax = float(ax * self.__CALIBRATION_ITERATIONS - self.ax_offset) * 4.0 / float(65536 * self.__CALIBRATION_ITERATIONS)
		fay = float(ay * self.__CALIBRATION_ITERATIONS - self.ay_offset) * 4.0 / float(65536 * self.__CALIBRATION_ITERATIONS)
		faz = float(az * self.__CALIBRATION_ITERATIONS - self.az_offset) * 4.0 / float(65536 * self.__CALIBRATION_ITERATIONS)
		fgx = float(gx * self.__CALIBRATION_ITERATIONS - self.gx_offset) * 1000.0 / float(65536 * self.__CALIBRATION_ITERATIONS)
		fgy = float(gy * self.__CALIBRATION_ITERATIONS - self.gy_offset) * 1000.0 / float(65536 * self.__CALIBRATION_ITERATIONS)
		fgz = float(gz * self.__CALIBRATION_ITERATIONS - self.gz_offset) * 1000.0 / float(65536 * self.__CALIBRATION_ITERATIONS)
		return fax, fay, faz, fgx, fgy, fgz
	
	def updateOffsets(self, file_name):
		ax_offset = 0
		ay_offset = 0
		az_offset = 0
		gx_offset = 0
		gy_offset = 0
		gz_offset = 0

		for loop_count in range(0, self.__CALIBRATION_ITERATIONS):
			[ax, ay, az, temp, gx, gy, gz] = self.readSensorsRaw()
			ax_offset += ax
			ay_offset += ay
			az_offset += az
			gx_offset += gx
			gy_offset += gy
			gz_offset += gz

			time.sleep(0.05)

		self.ax_offset = ax_offset
		self.ay_offset = ay_offset
		self.az_offset = az_offset
		self.gx_offset = gx_offset
		self.gy_offset = gy_offset
		self.gz_offset = gz_offset

		#---------------------------------------------------------------------------
		# Open the offset config file
		#---------------------------------------------------------------------------
		cfg_rc = True
		try:
			with open(file_name, 'w+') as cfg_file:
				cfg_file.write('%d\n' % ax_offset)
				cfg_file.write('%d\n' % ay_offset)
				cfg_file.write('%d\n' % az_offset)
				cfg_file.write('%d\n' % gx_offset)
				cfg_file.write('%d\n' % gy_offset)
				cfg_file.write('%d\n' % gz_offset)
				cfg_file.flush()

		except IOError, err:
			logger.critical('Could not open offset config file: %s for writing', file_name)
			cfg_rc = False

		return cfg_rc


	def readOffsets(self, file_name):
		#---------------------------------------------------------------------------
		# Open the Offsets config file, and read the contents
		#---------------------------------------------------------------------------
		cfg_rc = True
		try:
			with open(file_name, 'r') as cfg_file:
				str_ax_offset = cfg_file.readline()
				str_ay_offset = cfg_file.readline()
				str_az_offset = cfg_file.readline()
				str_gx_offset = cfg_file.readline()
				str_gy_offset = cfg_file.readline()
				str_gz_offset = cfg_file.readline()

			self.ax_offset = int(str_ax_offset)
			self.ay_offset = int(str_ay_offset)
			self.az_offset = int(str_az_offset)
			self.gx_offset = int(str_gx_offset)
			self.gy_offset = int(str_gy_offset)
			self.gz_offset = int(str_gz_offset)

		except IOError, err:
			logger.critical('Could not open offset config file: %s for reading', file_name)
			cfg_rc = False

		return cfg_rc

	def getEulerAngles(self, fax, fay, faz):
		#---------------------------------------------------------------------------
		# What's the angle in the x and y plane from horizonal?
		#---------------------------------------------------------------------------
		theta = math.atan2(fax, math.pow(math.pow(fay, 2) + math.pow(faz, 2), 0.5))
		psi = math.atan2(fay, math.pow(math.pow(fax, 2) + math.pow(faz, 2), 0.5))
		phi = math.atan2(math.pow(math.pow(fax, 2) + math.pow(fay, 2), 0.5), faz)

		theta *=  (180 / math.pi)
		psi *=  (180 / math.pi)
		phi *=  (180 / math.pi)

		return theta, psi, phi

	def readTemp(self):
		temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
		temp = (float(temp) / 340) + 36.53
		logger.debug('temp = %s oC', temp)
		return temp


#############################################################################################
# PID algorithm to take input accelerometer readings, and target accelermeter requirements, and
# as a result feedback new rotor speeds.
#############################################################################################
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

		#--------------------------------------------------------------------
		# Error is what the PID alogithm acts upon to derive the output
		#--------------------------------------------------------------------
		error = target - input

		#--------------------------------------------------------------------
		# The proportional term takes the distance between current input and target
		# and uses this proportially (based on Kp) to control the blade speeds
		#--------------------------------------------------------------------
		p_error = error

		#--------------------------------------------------------------------
		# The integral term sums the errors across many compute calls to allow for
		# external factors like wind speed and friction
		#--------------------------------------------------------------------
		self.i_error += (error + self.last_error) * dt
		if self.i_gain != 0.0 and self.i_error > self.i_err_max:
			self.i_error = self.i_err_max
			logger.warning('Cropped to max integral')
		elif self.i_gain != 0.0 and self.i_error < self.i_err_min:
			self.i_error = self.i_err_min
			logger.warning('Cropped to min integral')
		i_error = self.i_error

		#--------------------------------------------------------------------
		# The differential term accounts for the fact that as error approaches 0,
		# the output needs to be reduced proportionally to ensure factors such as
		# momentum do not cause overshoot.
		#--------------------------------------------------------------------
		d_error = (error - self.last_error) / dt

		#--------------------------------------------------------------------
		# The overall output is the sum of the (P)roportional, (I)ntegral and (D)iffertial terms
		#--------------------------------------------------------------------
		p_output = self.p_gain * p_error
		i_output = self.i_gain * i_error
		d_output = self.d_gain * d_error

		#--------------------------------------------------------------------
		# Store off last input (for the next differential calulation) and time for calculating the integral value
		#--------------------------------------------------------------------
		self.last_error = error
		self.last_time = now

		#--------------------------------------------------------------------
		# Return the output, which has been tuned to be the increment / decrement in blade PWM
		#--------------------------------------------------------------------
		return p_output, i_output, d_output

############################################################################################
#
#  Class for managing each blade configuration
#
############################################################################################
class QUADBLADE:
	pwm = None

	def __init__(self, pin, location, rotation, name):
		self.pin = pin
		self.location = location
		self.rotation = rotation
		if use_pca9685:
			self.min_spin_pulse = 2048
			self.max_spin_pulse = 4096
		else:
			self.min_spin_pulse = 1000
			self.max_spin_pulse = 2000


		self.spin_hover = self.min_spin_pulse
		self.current_spin = self.min_spin_pulse
		self.name = name

		#---------------------------------------------------------------------------
		# Initialize the PCA9685 I2C object or RPIO DMA PWM
		#---------------------------------------------------------------------------
		logger.info('blade %s on pin %d at location %d now spinning at %d', self.name, self.pin, self.location, self.current_spin)
		if use_pca9685:
			self.pwm = PCA9685(0x40)
			self.pwm.setPCA9685Freq(400)
			self.pwm.setPCA9685(self.pin, 0, self.current_spin)
		else:
			if not PWM.is_setup():
				PWM.setup(1)    # 1us increment
				PWM.init_channel(RPIO_DMA_CHANNEL, 3000)
			PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.pin, 0, self.current_spin)


	def spinStart(self):
		self.current_spin = self.min_spin_pulse
		logger.info('blade %s now at %d', self.name, self.current_spin)
		if use_pca9685:
			self.pwm.setPCA9685(self.pin, 0, self.current_spin)
		else:
			PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.pin, 0, self.current_spin)


	def spinStop(self):
		self.current_spin = self.min_spin_pulse
		logger.info('blade %s now at %d', self.name, self.current_spin)
		if use_pca9685:
			self.pwm.setPCA9685(self.channel, 0, self.current_spin)
		else:
			PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.pin, 0, self.current_spin)


	def spinHandover(self, spin_hover):
		self.spin_hover = self.min_spin_pulse + spin_hover

	def spinUpdate(self, zero_offset_spin):
		self.current_spin = int(self.spin_hover + zero_offset_spin)

		if self.current_spin < self.min_spin_pulse:
			self.current_spin = self.min_spin_pulse
		if self.current_spin > self.max_spin_pulse:
			self.current_spin = self.max_spin_pulse

		logger.debug('blade %s now at %d', self.name, self.current_spin)
		if use_pca9685:
			self.pwm.setPCA9685(self.channel, 0, self.current_spin)
		else:
			PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.pin, 0, self.current_spin)

		
############################################################################################
#
# GPIO pins initialization for /OE on the PWM device and the status LED
#
############################################################################################
def RpioSetup():
	RPIO.setmode(RPIO.BCM)

	#-----------------------------------------------------------------------------------
	# Set the LED output LOW
	#-----------------------------------------------------------------------------------
	logger.info('Set status sounder pin %s as out', RPIO_STATUS_SOUNDER)
	RPIO.setup(RPIO_STATUS_SOUNDER, RPIO.OUT, RPIO.LOW)

	#-----------------------------------------------------------------------------------
	# Set the /OE output for PCA9685 output HIGH
	#-----------------------------------------------------------------------------------
	if use_pca9685:
		logger.info('Set PCA9685 /OE pin %s as out', RPIO_PWM_OE_BAR)
		RPIO.setup(RPIO_PWM_OE_BAR, RPIO.OUT, RPIO.HIGH)

	#-----------------------------------------------------------------------------------
	# Set the MPU6050 interrupt input
	#-----------------------------------------------------------------------------------
	logger.info('Setup MPU6050 interrupt input %s', RPIO_SENSOR_DATA_RDY)
	RPIO.setup(RPIO_SENSOR_DATA_RDY, RPIO.IN, RPIO.PUD_DOWN)

############################################################################################
#
# Parse the received commmand to convert it to the equivalent directional / operational command
# The message format is a basic TLV with header and footer:
# Type: 1 byte
# Len: 1 byte - currently always 4 - includes this header
# Data: 2 bytes
#
############################################################################################
class TLVSTREAM():

	__CTRL_CMD_ABORT =      0
	__CTRL_CMD_TAKEOFF =    1
	__CTRL_CMD_LANDING =    2
	__CTRL_CMD_HOVER =      3
	__CTRL_CMD_UP_DOWN =    4
	__CTRL_CMD_FWD_RVS =    5
	__CTRL_CMD_LEFT_RIGHT = 6
	__CTRL_CMD_KEEPALIVE =  7
	__CTRL_CMD_DATA_ACK =   8

	def __init__(self):
		self.cache = ""

	def Parser(recv_buff):
		self.cache += resv_buff
		send_buff = ""

		fax_target = 0.0
		fay_target = 0.0
		faz_target = 0.0

		#---------------------------------------------------------------------------
		# Parse the message TLVs - assume no exception
		#---------------------------------------------------------------------------
		if len(self.cache) >= 4:
			type, length, msg_id = struct.unpack_from('!BBH', self.cache, 0)

			#-----------------------------------------------------------
			# If we have a complete TLV, parse it
			#-----------------------------------------------------------
			if len(self.cache) >= 4 + length:

				#---------------------------------------------------------------------------
				# If we've received a valid message type, then respond with an ACK
				#---------------------------------------------------------------------------
				send_buff = struct.pack('!BBH', __CTRL_CMD_DATA_ACK, 4, msg_id)
				drc_sck.send(send_buff)
				send_buff = ""

				#---------------------------------------------------------------------------
				# If the message content length > 0, unpack that too
				#---------------------------------------------------------------------------
				if length > 0:
					value = struct.unpack_from('!I', self.cache, 4)
					logger.info('type %d, length 0, msg_id %d', type, length, msg_id)
				else:
					logger.info('type %d, length %d, msg_id %d, value %d', type, length, msg_id, value)


				#---------------------------------------------------------------------------
				# Enact the command - decide the targets for the PID algorithm
				#---------------------------------------------------------------------------
				if type == __CTRL_CMD_ABORT:
					#----------------------------------------------------------------------------
					# Hard shutdown
					#----------------------------------------------------------------------------
					logger.warning('ABORT')
					os.kill(os.getpid(), signal.SIGINT)

				elif type == __CTRL_CMD_TAKEOFF:
					#----------------------------------------------------------------------------
					# Spin each blade up to the calibrated 0g point and then increment slight for a while beofre setting back to 0g
					#----------------------------------------------------------------------------
					logger.info('TAKEOFF')
					fax_target = 0.0
					fay_target = 0.0

					#AB: I think this shoud be a manual incremental increase in blade speeds to achieve takeoff before handover to hover
					faz_target = 1.01

				elif type == __CTRL_CMD_LANDING:
					#----------------------------------------------------------------------------
					# Spin each blade down to the calibrated 0g point and them decrement slightly for a controlled landing
					#----------------------------------------------------------------------------
					logger.info('LANDING')
					fax_target = 0.0
					fay_target = 0.0
					#AB: Less sure about this one though
					faz_target = 0.99

				elif type == __CTRL_CMD_HOVER:
					#----------------------------------------------------------------------------
					# Spin each blade down to the calibrated 0g point
					#----------------------------------------------------------------------------
					logger.info('HOVER')
					fax_target = 0.0
					fay_target = 0.0
					faz_target = 1.0

				elif type == __CTRL_CMD_UP_DOWN:
					#----------------------------------------------------------------------------
					# Increment the speed of all blades proportially to the command data
					#----------------------------------------------------------------------------
					logger.info('UP/DOWN %d', int(value))
					fax_target = 0.0
					fay_target = 0.0
					faz_target = 1.0 + (float(value * 0.05) / 128)

				elif type == __CTRL_CMD_FWD_RVS:
					#----------------------------------------------------------------------------
					# ????????????????
					#----------------------------------------------------------------------------
					logger.info('FWD/RVS %d', int(value))

				elif type == __CTRL_CMD_LEFT_RIGHT:
					#----------------------------------------------------------------------------
					# ????????????????
					#----------------------------------------------------------------------------
					logger.info('LEFT/RIGHT %d', int(value))

				elif type == __CTRL_CMD_KEEPALIVE:
					#----------------------------------------------------------------------------
					# No change to blade power, keep stable at the current setting
					#----------------------------------------------------------------------------
					logger.debug('KEEPALIVE')
				else:
					#----------------------------------------------------------------------------
					# Unrecognised command - treat as an abort
					#----------------------------------------------------------------------------
					logger.warning('UNRECOGNISED COMMAND: ABORT')
					os.kill(os.getpid(), signal.SIGINT)

				#---------------------------------------------------------------------------
				# TLV is wholly parsed, decrease the cache size
				#----------------------------------------------------------------------------
				self.cache = self.cache[4 + length : len(self.cache)]
			else:
				#---------------------------------------------------------------------------
				# Insufficient data to form a whole TLV, get out
				#---------------------------------------------------------------------------
				parsed_tlv = False


		else:
			#---------------------------------------------------------------------------
			# Insufficient data to parse, leave target unchanged !!!!!!!!!!!!!!!!!!!!!!!!
			#---------------------------------------------------------------------------
			parsed_tlv = False

		#---------------------------------------------------------------------------
		# No more complete TLVs to part, get out, returning how much data we have dealt with
		#---------------------------------------------------------------------------
		return fax_target, fay_target, faz_target


############################################################################################
#
# Check CLI validity, set calibrate_sensors / fly_drone or sys.exit(1)
#
############################################################################################
def CheckCLI(argv):
	cli_calibrate = False
	cli_fly = False
	cli_test = False
	cli_video = False
	cli_test_speed = -1
	cli_vsp_gain = 0.0
	cli_vsi_gain = 0.0
	cli_vsd_gain = 0.0


	try:
		opts, args = getopt.getopt(argv,"fcvt:p:i:d:")
	except getopt.GetoptError:
		logger.critical('dronepi.py [-f|-t speed] [-c] [-v] [-p proportional] [-i integral] [-ddifferential]')
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-f':
			cli_fly = True

		if opt == '-c':
			cli_calibrate = True

		if opt in '-t':
			cli_test = True
			cli_test_speed = int(arg)

		if opt in '-v':
			cli_video = True

		if opt in '-p':
			cli_vsp_gain = float(arg)

		if opt in '-i':
			cli_vsi_gain = float(arg)

		if opt in '-d':
			cli_vsd_gain = float(arg)


	if not cli_fly and not cli_calibrate and not cli_test and not cli_video:
		logger.critical('Must set at least one CLI parameter')
		logger.critical('  dronepi.py [-f|-t speed] [-c] [-v videolength]')
		logger.critical('  -f fly the drone on auto-pilot')
		logger.critical('  -t test the drone balancing at given blade spin speed')
		logger.critical('  -c calibrate and save the sensor outputs')
		logger.critical('  -v video the flight')
		logger.critical('  -i set the vertical speed integral gain for take off speed')
		logger.critical('  -p set the vertical speed proportional gain for take off speed')
		sys.exit(2)

	if cli_test and cli_fly:
		logger.crical('Choose between flying (-f) or testing (-t)')
		sys.exit(2)

	if cli_test and (cli_test_speed < 0 or cli_test_speed > 1000):
		logger.critical('Test speed must lie in the following range')
		logger.critical('0 <= test speed <= 1000')
		sys.exit(2)

	return cli_calibrate, cli_fly, cli_test, cli_test_speed, cli_video, cli_vsp_gain, cli_vsi_gain, cli_vsd_gain

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
	global blade_list
	global shoot_video
	global video

	#-----------------------------------------------------------------------------------
	# Time for teddy bye byes
	#-----------------------------------------------------------------------------------
	for blade in blade_list:
		logger.info('Stop blade %d spinning', blade_index)
		blade.spinStop()

	#-----------------------------------------------------------------------------------
	# Stop the video if it's running
	#-----------------------------------------------------------------------------------
	if shoot_video:
		video.send_signal(signal.SIGINT)

	PWM.cleanup()
	RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)
	RPIO.cleanup()
	sys.exit(0)

############################################################################################
#
# Signal handler for Ctrl-C, ABORT and unrecognised commands - orderly descent
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

G_FORCE = 9.80665

RPIO_DMA_CHANNEL = 1

use_mpu6050 = True
use_pca9685 = False
use_sockets = False

if use_pca9685:
	BLADE_CHAN_BL = 0
	BLADE_CHAN_FL = 7
	BLADE_CHAN_FR = 8
	BLADE_CHAN_BR = 15
else:
	BLADE_RPIO_BL = 22
	BLADE_RPIO_FL = 17
	BLADE_RPIO_FR = 18
	BLADE_RPIO_BR = 23

BLADE_LOC_FRONT = 0b00000000
BLADE_LOC_BACK = 0b00000010
BLADE_LOC_LEFT = 0b00000000
BLADE_LOC_RIGHT = 0b00000001

BLADE_ROT_CW = 0
BLADE_ROT_ACW = 1

NUM_SOCK = 5
RC_SILENCE_LIMIT = 10


#-------------------------------------------------------------------------------------------
# Through testing, take-off happens @ 587
#-------------------------------------------------------------------------------------------
BLADE_TAKEOFF_READY_RATE = 550

#-------------------------------------------------------------------------------------------
# Set the BCM outputs assigned to LED and sensor interrupt
#-------------------------------------------------------------------------------------------
RPIO_PCA9685_OE_BAR = 14
RPIO_STATUS_SOUNDER = 27
RPIO_SENSOR_DATA_RDY = 25

silent_scan_count = 0

#-------------------------------------------------------------------------------------------
# Set up the base logging
#-------------------------------------------------------------------------------------------
logger = logging.getLogger('drone logger')
logger.setLevel(logging.INFO)

#-------------------------------------------------------------------------------------------
# Create file and console logger handlers
#-------------------------------------------------------------------------------------------
file_handler = logging.FileHandler('dronestats.csv', 'w')
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
normalize_sensors, fly_drone, test_drone, testing_speed, shoot_video, vsp_gain, vsi_gain, vsd_gain = CheckCLI(sys.argv[1:])

if test_drone:
	#-------------------------------------------------------------------------------------------
	# The PITCH PID controls rotation about the Y-axis
	#-------------------------------------------------------------------------------------------
	PID_PITCH_ANGLE_P_GAIN = 0.0
	PID_PITCH_ANGLE_I_GAIN = 0.0
	PID_PITCH_ANGLE_D_GAIN = -0.0
	
	#-------------------------------------------------------------------------------------------
	# The ROLL PID controls rotation about the X-axis
	#-------------------------------------------------------------------------------------------
	PID_ROLL_ANGLE_P_GAIN = 0.0
	PID_ROLL_ANGLE_I_GAIN = 0.0
	PID_ROLL_ANGLE_D_GAIN = -0.0
	
	#-------------------------------------------------------------------------------------------
	# The YAW PID controls rotation about the Z-axis
	#-------------------------------------------------------------------------------------------
	PID_YAW_ANGLE_P_GAIN = 0.0
	PID_YAW_ANGLE_I_GAIN = 0.0
	PID_YAW_ANGLE_D_GAIN = -0.0
	
	#-------------------------------------------------------------------------------------------
	# The vertical speed controls stable rise / fall rate in the z direction
	#-------------------------------------------------------------------------------------------
	PID_VERT_SPEED_P_GAIN = vsp_gain  # 150 default guess based on stats
	PID_VERT_SPEED_I_GAIN = vsi_gain  # 200 default guess base don P/I ratio that work
	PID_VERT_SPEED_D_GAIN = vsd_gain  # 0.1 default guess based on stats

	#-------------------------------------------------------------------------------------------
	# The PITCH PID controls rotation about the Y-axis
	#-------------------------------------------------------------------------------------------
	PID_PITCH_RATE_P_GAIN = 2.5
	PID_PITCH_RATE_I_GAIN = 5.0
	PID_PITCH_RATE_D_GAIN = 0.15

	#-------------------------------------------------------------------------------------------
	# The ROLL PID controls rotation about the X-axis
	#-------------------------------------------------------------------------------------------
	PID_ROLL_RATE_P_GAIN = 2.5
	PID_ROLL_RATE_I_GAIN = 5.0
	PID_ROLL_RATE_D_GAIN = 0.15

	#-------------------------------------------------------------------------------------------
	# The YAW PID controls rotation about the Z-axis
	#-------------------------------------------------------------------------------------------
	PID_YAW_RATE_P_GAIN = 2.5         # 0 yaw value proven in test
	PID_YAW_RATE_I_GAIN = 4.0         # 0 yaw yalue proven in test
	PID_YAW_RATE_D_GAIN = 0.12        # 0 yaw value proven in test

	#-------------------------------------------------------------------------------------------
	# The vertical acceleration controls rise / fall acceleration in the z direction
	#-------------------------------------------------------------------------------------------
	PID_VERT_ACCEL_P_GAIN = 20.0
	PID_VERT_ACCEL_I_GAIN = 13.3
	PID_VERT_ACCEL_D_GAIN = -0.0

else:
	#-------------------------------------------------------------------------------------------
	# The PITCH PID controls rotation about the Y-axis
	#-------------------------------------------------------------------------------------------
	PID_PITCH_ANGLE_P_GAIN = 0.0
	PID_PITCH_ANGLE_I_GAIN = 0.0
	PID_PITCH_ANGLE_D_GAIN = -0.0
	
	#-------------------------------------------------------------------------------------------
	# The ROLL PID controls rotation about the X-axis
	#-------------------------------------------------------------------------------------------
	PID_ROLL_ANGLE_P_GAIN = 0.0
	PID_ROLL_ANGLE_I_GAIN = 0.0
	PID_ROLL_ANGLE_D_GAIN = -0.0
	
	#-------------------------------------------------------------------------------------------
	# The YAW PID controls rotation about the Z-axis
	#-------------------------------------------------------------------------------------------
	PID_YAW_ANGLE_P_GAIN = 0.0
	PID_YAW_ANGLE_I_GAIN = 0.0
	PID_YAW_ANGLE_D_GAIN = -0.0
	
	#-------------------------------------------------------------------------------------------
	# The vertical speed controls stable rise / fall rate in the z direction
	#-------------------------------------------------------------------------------------------
	PID_VERT_SPEED_P_GAIN = 0.0
	PID_VERT_SPEED_I_GAIN = 10.0
	PID_VERT_SPEED_D_GAIN = -0.0

	#-------------------------------------------------------------------------------------------
	# The PITCH PID controls rotation about the Y-axis
	#-------------------------------------------------------------------------------------------
	PID_PITCH_RATE_P_GAIN = 3.3
	PID_PITCH_RATE_I_GAIN = 2.2
	PID_PITCH_RATE_D_GAIN = 0.2

	#-------------------------------------------------------------------------------------------
	# The ROLL PID controls rotation about the X-axis
	#-------------------------------------------------------------------------------------------
	PID_ROLL_RATE_P_GAIN = 3.3
	PID_ROLL_RATE_I_GAIN = 2.2
	PID_ROLL_RATE_D_GAIN = 0.2

	#-------------------------------------------------------------------------------------------
	# The YAW PID controls rotation about the Z-axis
	#-------------------------------------------------------------------------------------------
	PID_YAW_RATE_P_GAIN = 0.0
	PID_YAW_RATE_I_GAIN = 0.0
	PID_YAW_RATE_D_GAIN = -0.0

	#-------------------------------------------------------------------------------------------
	# The vertical acceleration controls rise / fall acceleration in the z direction
	#-------------------------------------------------------------------------------------------
	PID_VERT_ACCEL_P_GAIN = 0.0
	PID_VERT_ACCEL_I_GAIN = 0.0
	PID_VERT_ACCEL_D_GAIN = -0.0


#-------------------------------------------------------------------------------------------
# Assign blade names according to the channel, physical location and spin start calibration
#-------------------------------------------------------------------------------------------
blade_list = []
if use_pca9685:
	pin_list = [BLADE_CHAN_FL, BLADE_CHAN_FR, BLADE_CHAN_BL, BLADE_CHAN_BR]
else:
	pin_list = [BLADE_RPIO_FL, BLADE_RPIO_FR, BLADE_RPIO_BL, BLADE_RPIO_BR]

location_list = [BLADE_LOC_FRONT | BLADE_LOC_LEFT, BLADE_LOC_FRONT | BLADE_LOC_RIGHT, BLADE_LOC_BACK | BLADE_LOC_LEFT, BLADE_LOC_BACK | BLADE_LOC_RIGHT]
rotation_list = [BLADE_ROT_ACW, BLADE_ROT_CW, BLADE_ROT_CW, BLADE_ROT_ACW]
name_list = ['front left', 'front right', 'back left', 'back right']

#-------------------------------------------------------------------------------------------
# Prime the blades with the default 0 spin rotors
#-------------------------------------------------------------------------------------------
for blade_index in range(0, 4):
	blade = QUADBLADE(pin_list[blade_index], location_list[blade_index], rotation_list[blade_index], name_list[blade_index])
	blade_list.append(blade)

#-------------------------------------------------------------------------------------------
# Now the PCA9685 is primed, enable the output to shut the ESCs up.
#-------------------------------------------------------------------------------------------
RpioSetup()
if use_pca9685:
	RPIO.output(RPIO_PCA9685_OE_BAR, RPIO.LOW)

#-------------------------------------------------------------------------------------------
# Initialize the gyroscope / accelerometer I2C object
#-------------------------------------------------------------------------------------------
if use_mpu6050:
	mpu6050 = MPU6050(0x68)

	#-------------------------------------------------------------------------------------------
	# 5 beeps + 2s wait
	#-------------------------------------------------------------------------------------------
	CountdownBeep(5)

	#-----------------------------------------------------------------------------------
	# Check the sensors for any offsets to ensure compensated accurate readings
	#-----------------------------------------------------------------------------------
	if normalize_sensors:
		if not mpu6050.updateOffsets('./dronesensors.cfg'):
			print 'Sensor normalization error'
			sys.exit(1)


	else:
		if not mpu6050.readOffsets('./dronesensors.cfg'):
			print 'Sensor config error'
			print '- try running dronepi -c on a flat, horizontal surface first'
			sys.exit(1)

#-------------------------------------------------------------------------------------------
# Are we flying tonight?
#-------------------------------------------------------------------------------------------
if not fly_drone and not test_drone:
	print 'Not flying tonight, sleep well, drone'
	sys.exit(0)

#-------------------------------------------------------------------------------------------
# 4 beeps + 2s wait
#-------------------------------------------------------------------------------------------
CountdownBeep(4)

#-------------------------------------------------------------------------------------------
# Wait pending a sockets connection with the RC if required
#-------------------------------------------------------------------------------------------
inputs = []
outputs = []
if use_sockets:
	try:
		serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	except socket.error as msg:
		serversock = None

	try:
#		serversock.setblocking(False)  # <=====???????????
		serversock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		serversock.bind((socket.gethostname(), 12345))
		logger.info('Listening as host %s', socket.gethostname())
		serversock.listen(NUM_SOCK)
	except socket.error as msg:
		serversock.close()
		serversock = None

	if serversock is None:
		sys.exit(1)

	#-----------------------------------------------------------------------------------
	# Wait to accept a connection from the drone RC
	#-----------------------------------------------------------------------------------
	drc_socket, drc_addr = serversock.accept()
	drc_socket.setblocking(False)
	inputs = [drc_socket]
	logger.info('Connected to %s', drc_socket.getpeername())

#-------------------------------------------------------------------------------------------
# Now we have a connection, power up the TCP data TLV parsing engine
#-------------------------------------------------------------------------------------------
tlvstream = TLVSTREAM()

#-------------------------------------------------------------------------------------------
# 3 beeps + 2s wait
#-------------------------------------------------------------------------------------------
CountdownBeep(3)

#---------------------------------------------------------------------------
# Set the signal handler here so the spin can be cancelled when loop_count = 0
# or moved on when > 0
#---------------------------------------------------------------------------
loop_count = 0
signal.signal(signal.SIGINT, SignalHandler)

#-------------------------------------------------------------------------------------------
# 2 beeps + 2s wait
#-------------------------------------------------------------------------------------------
CountdownBeep(2)

#-------------------------------------------------------------------------------------------
# Only now we have an RC connection, and have accel / gyro calibration on a stable platform
# do we engage the blades to spinning point.  10s to takeoff and counting...
#-------------------------------------------------------------------------------------------
for blade in blade_list:
	blade.spinStart()

#-------------------------------------------------------------------------------------------
# 1 beep + 5s wait
#-------------------------------------------------------------------------------------------
CountdownBeep(1)

#-------------------------------------------------------------------------------------------
# Start up the video camera if required - this runs from take-off through to shutdown automatically
#-------------------------------------------------------------------------------------------
if shoot_video:
	now = datetime.now()
	now_string = now.strftime("%y%m%d-%H:%M:%S")
	video = subprocess.Popen(["raspivid", "-rot", "180", "-o", "/home/pi/Videos/dronevid_" + now_string + ".h264", "-n", "-t", "0", "&"])

#-------------------------------------------------------------------------------------------
# Last bits of state setup before takeoff
#-------------------------------------------------------------------------------------------
keep_looping = True
delta_time = 0.0
SCAN_TIME = 0.05
sleep_time = SCAN_TIME / 2
last_int_gyro_angle = 0.0
int_gyro_angle = 0.0

pitch_angle = 0.0
roll_angle = 0.0
yaw_angle = 0.0

pitch_angle_target = 0.0
roll_angle_target = 0.0
yaw_angle_target = 0.0

prev_fgx = 0.0
prev_fgy = 0.0
prev_fgz = 0.0

vert_speed = 0.0
vert_speed_target = 0.0
prev_faz = 0.0

INPUT_NONE = 0
INPUT_TAKEOFF = 1
INPUT_LEVELLED = 2
INPUT_HOVER = 3
INPUT_LAND = 4
INPUT_STOP = 5
INPUT_SIGNAL = 6
fsm_input = INPUT_NONE

STATE_OFF = 0
STATE_ASCENDING = 1
STATE_LEVELLING = 2
STATE_HOVERING = 3
STATE_DESCENDING = 4
fsm_state = STATE_OFF

for beep_count in range(0, BLADE_TAKEOFF_READY_RATE, 10):
	for blade in blade_list:
		#--------------------------------------------------------------------------
		# Spin up to just under take-off / hover speeds
		#--------------------------------------------------------------------------
		blade.spinUpdate(beep_count);
	
	RPIO.output(RPIO_STATUS_SOUNDER, not RPIO.input(RPIO_STATUS_SOUNDER))
	time.sleep(0.01)

#---------------------------------------------------------------------------
# Diagnostic statistics log header
#---------------------------------------------------------------------------
logger.warning(', Time, DT, Loop, fgx, fgy, fgz, fax, fay, faz, pitch, roll, yaw, vert, prp, pri, prd, pap, pai, pad, rrp, rri, rrd, rap, rai, rad, yrp, yri, yrd, yap, yai, yad, vsp, vsi, vsd, pitch out, rollout, yaw out, vert speed_out, FL spin, FR spin, BL spin, BR spin')

#-------------------------------------------------------------------------------------------
# Enable time dependent factors PIDs - everything beyond here and "while keep_looping:" is time
# critical and should be kept to a minimum.
#-------------------------------------------------------------------------------------------
if use_mpu6050:
	pitch_angle_pid = PID(PID_PITCH_ANGLE_P_GAIN, PID_PITCH_ANGLE_I_GAIN, PID_PITCH_ANGLE_D_GAIN)
	roll_angle_pid = PID(PID_ROLL_ANGLE_P_GAIN, PID_ROLL_ANGLE_I_GAIN, PID_ROLL_ANGLE_D_GAIN)
	yaw_angle_pid = PID(PID_YAW_ANGLE_P_GAIN, PID_YAW_ANGLE_I_GAIN, PID_YAW_ANGLE_D_GAIN)

	pitch_rate_pid = PID(PID_PITCH_RATE_P_GAIN, PID_PITCH_RATE_I_GAIN, PID_PITCH_RATE_D_GAIN)
	roll_rate_pid = PID(PID_ROLL_RATE_P_GAIN, PID_ROLL_RATE_I_GAIN, PID_ROLL_RATE_D_GAIN)
	yaw_rate_pid = PID(PID_YAW_RATE_P_GAIN, PID_YAW_RATE_I_GAIN, PID_YAW_RATE_D_GAIN)

        vert_speed_pid_active = False
#	vert_speed_pid = PID(PID_VERT_SPEED_P_GAIN, PID_VERT_SPEED_I_GAIN, PID_VERT_SPEED_D_GAIN)
#	vert_accel_pid = PID(PID_VERT_ACCEL_P_GAIN, PID_VERT_ACCEL_I_GAIN, PID_VERT_ACCEL_D_GAIN)

levelled_time = 0.0
elapsed_time = 0.0
start_time = time.time()
last_log_time = start_time
current_time = start_time

while keep_looping:
	loop_count += 1

        #---------------------------------------------------------------------------
        # FSM inputs are mostly generated on a timer; the exceptions are
        # - SIGNAL generated by a Ctrl-C
        # - LEVELLED triggered when vertical speed post take-off becomes < 0
        #---------------------------------------------------------------------------
	if fsm_input != INPUT_SIGNAL and fsm_input != INPUT_LEVELLED:
		if elapsed_time >= 0.0:
			fsm_input = INPUT_TAKEOFF

		if elapsed_time >= 2.0:
			fsm_input = INPUT_HOVER

		if elapsed_time >= levelled_time + 4.0:
			fsm_input = INPUT_LAND

		if elapsed_time >= levelled_time + 6.0:
			fsm_input = INPUT_STOP

	if fsm_state == STATE_OFF and fsm_input == INPUT_TAKEOFF:
		# -------------------------------EXPERIMENTAL-----------------------
		# testing_speed as defined by the CLI
		# -------------------------------EXPERIMENTAL-----------------------
		logger.info('FSM action takeoff')
		fsm_state = STATE_ASCENDING
		fsm_input = INPUT_NONE
		vert_speed_target = 0.0
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)

	elif fsm_state == STATE_ASCENDING and (fsm_input == INPUT_HOVER or fsm_input == INPUT_SIGNAL):
		logger.info('FSM action handover')
		fsm_state = STATE_LEVELLING
		fsm_input = INPUT_NONE
                #--------------------------NOT STRICTLY NECESSARY--------------------
		vert_speed_target = 0.0

		# -------------------------------EXPERIMENTAL------------------------
		testing_speed = 580
		# -------------------------------EXPERIMENTAL------------------------

		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)

	elif fsm_state == STATE_LEVELLING and (fsm_input == INPUT_LEVELLED or fsm_input == INPUT_SIGNAL):
		logger.info('FSM action handover')
		fsm_state = STATE_HOVERING
		fsm_input = INPUT_NONE
                #--------------------------NOT STRICTLY NECESSARY--------------------
		vert_speed_target = 0.0

		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)


	elif fsm_state == STATE_HOVERING and (fsm_input == INPUT_LAND or fsm_input == INPUT_SIGNAL):
		logger.info('FSM action landing')
		fsm_state = STATE_DESCENDING
		fsm_input = INPUT_NONE
		vert_speed_target = -0.15


		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)

	elif fsm_state == STATE_DESCENDING and (fsm_input == INPUT_STOP or fsm_input == INPUT_SIGNAL):
		logger.info('FSM action shutdown')
		fsm_state = STATE_OFF
		fsm_input = INPUT_NONE
		vert_speed_target = 0.0

		# -------------------------------EXPERIMENTAL------------------------
		testing_speed = 0
		# -------------------------------EXPERIMENTAL------------------------

		keep_looping = False
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)

	#---------------------------------------------------------------------------
	# Make sure the TLV stream is empty from the last run before extracting more RC command data
	#---------------------------------------------------------------------------
	if use_sockets:

		#---------------------------------------------------------------------------
		# Select on waiting for a command, or a hello
		#---------------------------------------------------------------------------
		readable, writeable, exceptional = select.select(inputs, outputs, inputs, sleep_time)

		#-----------------------------------------------------------------------------------
		# HELLO timeout - check for receipt and send
		#-----------------------------------------------------------------------------------
		if not (readable or writeable or exceptional):

			#---------------------------------------------------------------------------
			# The timer's popped, which means we've received nothing in the last KEEPALIVE_TIMER seconds.
			# For safety's sake, commit suicide.
			#---------------------------------------------------------------------------
			silent_scan_count += 1
			if silent_scan_count == RC_SILENCE_LIMIT:
				#-----------------------------------------------------------
				# We've not receive a message from RC for 10 scans, close the socket and
				# enforce an automatic landing
				#-----------------------------------------------------------
				logger.error('No message from RC for 10 scans')
				drc_sck.shutdown(socket.SHUT_RDWR)
				drc_sck.close()

#				fax_target = 0.0
#				fay_target = 0.0
#				faz_target = 0.95

				use_sockets = False

				break

		#-----------------------------------------------------------------------------------
		# Now check whether we have errors on anything
		#-----------------------------------------------------------------------------------
		for drc_sck in exceptional:

			#---------------------------------------------------------------------------
			# Don't care about the details, set auto-landing
			#---------------------------------------------------------------------------
			logger.error('Select socket error')
			drc_sck.shutdown(socket.SHUT_RDWR)
			drc_sck.close()

#			fax_target = 0.0
#			fay_target = 0.0
#			faz_target = 0.95

			use_sockets = False

			break

		#-----------------------------------------------------------------------------------
		# Now check whether we have received anything
		#-----------------------------------------------------------------------------------
		for drc_sck in readable:

			#---------------------------------------------------------------------------
			# Check to see what we've received
			#---------------------------------------------------------------------------
			drc_data = drc_sck.recv(4096)
			if not drc_data:
				#-------------------------------------------------------------------
				# Client shutdown processing
				#-------------------------------------------------------------------
				logger.error('0 data received')
				drc_sck.shutdown(socket.SHUT_RDWR)
				drc_sck.close()

#				fax_target = 0.0
#				fay_target = 0.0
#				faz_target = 0.95

				use_sockets = False

				break

			#-------------------------------------------------------------------
			# Parse the control message
			#-------------------------------------------------------------------
			fax_target, fay_target, faz_target = tlvstream.Parse(drc_data)

			#-------------------------------------------------------------------
			# Cycle through each PID applying the appropriate new targets
			#-------------------------------------------------------------------
			if fax_target == 0 and fay_target == 0 and faz_target == 0:
				logger.warning('Nothing parsed!')
				silent_scan_count += 1
				if silent_scan_count == RC_SILENCE_LIMIT:
					#-----------------------------------------------------------
					# We've not receive a message from RC for 10 scans, close the socket and
					# enforce an automatic landing
					#-----------------------------------------------------------
					logger.error('No message from RC for 10 scans')
					drc_sck.shutdown(socket.SHUT_RDWR)
					drc_sck.close()

#       				fax_target = 0.0
#					fay_target = 0.0
#					faz_target = 0.95

					use_sockets = False
					break
			else:
				silent_scan_count = 0
#	else:
#		#-----------------------------------------------------------------------------------
#		# Now check whether we have received anything
#		#-----------------------------------------------------------------------------------
#		time.sleep(sleep_time)
#
#		#-----------------------------------------------------------------------------------
#		# Simulate acclerometer targets for testing purposes - HOVER
#		#-----------------------------------------------------------------------------------
#		if silent_scan_count >= RC_SILENCE_LIMIT:
#			fax_target = 0.0
#			fay_target = 0.0
#			faz_target = 0.95


	if use_mpu6050:
		#---------------------------------------------------------------------------
		# Update the elapsed time since start, the time for the last interaction, and
		# set the next sleep time to compensate for any overrun in scheduling.
		#---------------------------------------------------------------------------
		[fax, fay, faz, fgx, fgy, fgz] = mpu6050.readSensors()

		#---------------------------------------------------------------------------
		# Update the elapsed time since start, the time for the last interaction, and
		# set the next sleep time to compensate for any overrun in scheduling.
		#---------------------------------------------------------------------------
		current_time = time.time()
		delta_time = current_time - start_time - elapsed_time
		elapsed_time = current_time - start_time

		#---------------------------------------------------------------------------
		# Obtain the Euler angles
		#---------------------------------------------------------------------------
#		[theta, psi, phi] = mpu6050.getEulerAngles(fax, fay, faz)

		#---------------------------------------------------------------------------
		# Integrate the gyros angular velocity to determine absolute angle of tilt in 3D
		#---------------------------------------------------------------------------
		pitch_angle += (fgy + prev_fgy) * delta_time / 2.0
		prev_fgy = fgy
		roll_angle += (fgx + prev_fgx) * delta_time / 2.0
		prev_fgx = fgx
		yaw_angle += (fgz + prev_fgz) * delta_time / 2.0
		prev_fgz = fgz

		#---------------------------------------------------------------------------
		# Compensate faz readings to any angular deviation from the horizontal in pitch and roll
		# This ensure correct vertical speed with respect to earth horizon when drone is tilted
		# either intentionally via controller or unintentionally via imbalance / wind etc
		#---------------------------------------------------------------------------
#		faz = faz / (math.cos(pitch_angle * math.pi / 180) * math.cos(roll_angle * math.pi / 180))

		#---------------------------------------------------------------------------
		# Integrate the accelerometer g force to determine absolute linear velocity in m/s
		#---------------------------------------------------------------------------
		vert_speed += (faz + prev_faz) * delta_time * G_FORCE / 2.0
		prev_faz = faz

		#---------------------------------------------------------------------------
		# If we are in levelling state, and the vertical speed has now gone negative
                # (the drone has just started to fall), engage the vertical speed PID and
                # set the fsm_input to INPUT_LEVELLED so we then transit into HOVERING state
                # and saving the levelled time from which two seconds of hover starts.
		#---------------------------------------------------------------------------
                if fsm_state == STATE_LEVELLING and vert_speed < 0.0:
                        fsm_input = INPUT_LEVELLED
                        levelled_time = elapsed_time
                        vert_speed_pid = PID(PID_VERT_SPEED_P_GAIN, PID_VERT_SPEED_I_GAIN, PID_VERT_SPEED_D_GAIN)
                        vert_speed_pid_active = True
                        vert_speed_target = 0.0


		#---------------------------------------------------------------------------
		# Run the angle PIDs each rotation axis to determine targets for rate PID.
		#---------------------------------------------------------------------------
		[p_out, i_out, d_out] = pitch_angle_pid.Compute(pitch_angle, pitch_angle_target)
		pa_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		pitch_rate_target = p_out + i_out + d_out
		[p_out, i_out, d_out] = roll_angle_pid.Compute(roll_angle, roll_angle_target)
		ra_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		roll_rate_target = p_out + i_out + d_out
		[p_out, i_out, d_out] = yaw_angle_pid.Compute(yaw_angle, yaw_angle_target)
		ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		yaw_rate_target = p_out + i_out + d_out

		#---------------------------------------------------------------------------
		# Run the rate PIDs each rotation axis.
		#---------------------------------------------------------------------------
		[p_out, i_out, d_out] = pitch_rate_pid.Compute(fgy, pitch_rate_target)
		pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		pitch_out = p_out + i_out + d_out
		[p_out, i_out, d_out] = roll_rate_pid.Compute(fgx, roll_rate_target)
		rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		roll_out = p_out + i_out + d_out
		[p_out, i_out, d_out] = yaw_rate_pid.Compute(fgz, yaw_rate_target)
		yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		yaw_out = p_out + i_out + d_out

		#---------------------------------------------------------------------------
		# Run the vertical velocity PID for the z dimension to maintain height.
		#---------------------------------------------------------------------------
                if vert_speed_pid_active:
                        [p_out, i_out, d_out] = vert_speed_pid.Compute(vert_speed, vert_speed_target)
                else:
                        p_out = 0.0
                        i_out = 0.0
                        d_out = 0.0

                vs_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		vert_speed_out = p_out + i_out + d_out


#		[p_out, i_out, d_out] = vert_accel_pid.Compute(faz, vert_accel_target)
#		va_diags = "%f, %f, %f" % (p_out, i_out, d_out)
#		vert_accel_out = p_out + i_out + d_out

		pitch_out = int(round(pitch_out / 2))
		roll_out = int(round(roll_out / 2))
		yaw_out = int(round(yaw_out / 2))

		vert_out = testing_speed + vert_speed_out

		#---------------------------------------------------------------------------
		# Walk through the blades, and depending on their location, apply the output
		#---------------------------------------------------------------------------
		for blade in blade_list:
			#-------------------------------------------------------------------
			# Update all blades' power in accordance with the z error
			#-------------------------------------------------------------------
			delta_spin = vert_out

			#-------------------------------------------------------------------
			# For a left downwards roll, the x gyro goes negative, so the PID error is positive, meaning
			# PID output is positive, meaning this needs to be added to the left blades and subtracted from the right.
			#-------------------------------------------------------------------
			if blade.location & BLADE_LOC_RIGHT:
				delta_spin -= roll_out
			else:
				delta_spin += roll_out

			#-------------------------------------------------------------------
			# For a forward downwards pitch, the y gyro goes positive, so the PID error is negative, meaning
			# PID output is negative, meaning this needs to be subtracted from the front blades and added to the back.
			#-------------------------------------------------------------------
			if blade.location & BLADE_LOC_BACK:
				delta_spin += pitch_out
			else:
				delta_spin -= pitch_out

			#-------------------------------------------------------------------
			# The CW rotating of the front-right and back-left (FR & BL) blades
			# results in an ACW rotation of the quadcopter body. The z gyro produces
			# a positive output as a result. This then leads to the PID error
			# (target - gyro) being negative, meaning PID  output is negative
			# (assuming positive gains). Since the PID output needs to reduce the
			# over-enthusiastic CW rotation of the FR & BL blades, the negative PID
			# output needs to be added to those blades (thus slowing their rotation)
			# and subtracted from the ACW FL & BR blades (thus speeding them up) to
			# compensate for the yaw.
			#-------------------------------------------------------------------
			if blade.location == (BLADE_LOC_FRONT | BLADE_LOC_RIGHT) or blade.location == (BLADE_LOC_BACK | BLADE_LOC_LEFT):
				delta_spin += yaw_out
			else:
				delta_spin -= yaw_out

			blade.spinUpdate(delta_spin)

		#---------------------------------------------------------------------------
		# Update the elapsed time since start, the time for the last interaction, and
		# set the next sleep time to compensate for any overrun in scheduling.
		#---------------------------------------------------------------------------
#		sleep_time = SCAN_TIME - (delta_time - sleep_time)
#		if sleep_time < 0.0:
#			sleep_time = 0

		#---------------------------------------------------------------------------
		# Diagnostic statistics log
		#---------------------------------------------------------------------------
#		if (current_time - last_log_time) >= 0.1:
		logger.warning(', %f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s, %s, %s, %s, %s, %d, %d, %d, %d, %d, %d, %d, %d', elapsed_time, delta_time, loop_count, fgx, fgy, fgz, fax, fay, faz, pitch_angle, roll_angle, yaw_angle, vert_speed, pr_diags, pa_diags, rr_diags, ra_diags, yr_diags, ya_diags, vs_diags, pitch_out, roll_out, yaw_out, vert_speed_out, blade_list[0].current_spin, blade_list[1].current_spin, blade_list[2].current_spin, blade_list[3].current_spin)
#			last_log_time = last_log_time + 0.1


#-------------------------------------------------------------------------------------------
# Stop the blades during shutdown analysis
#-------------------------------------------------------------------------------------------
for blade in blade_list:
	blade.spinStop()

#-------------------------------------------------------------------------------------------
# Dump the loops per second
#-------------------------------------------------------------------------------------------
logger.critical("loop speed %f loops per second", loop_count / elapsed_time)

#-------------------------------------------------------------------------------------------
# Compare current Euler angles from accel vs intergrated gyro output
#-------------------------------------------------------------------------------------------
itheta = 0.0
ipsi = 0.0
iphi = 0.0
for loop_count in range(0, 100):
	[fax, fay, faz, fgx, fgy, fgz] = mpu6050.readSensors()
	[theta, psi, phi] = mpu6050.getEulerAngles(fax, fay, faz)
	itheta += theta
	ipsi += psi
	iphi += phi
	time.sleep(0.01)
itheta /= 100
ipsi /= 100
iphi /= 100
logger.critical("pitch angle - interated: %f; euler: %f", pitch_angle, itheta)
logger.critical("roll angle - integrated: %f; euler %f", roll_angle, ipsi)
logger.critical("yaw angle - integrated: %f; euler %f", yaw_angle, iphi)

#-------------------------------------------------------------------------------------------
# Time for telly bye byes
#-------------------------------------------------------------------------------------------
CleanShutdown()


