#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2017 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################


from __future__ import division
from __future__ import with_statement
import signal
import socket
import time
import string
import sys
import getopt
import math
import thread
from array import *
import smbus
import select
import os
import io
import logging
import csv
from RPIO import PWM
import RPi.GPIO as GPIO
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library
import picamera
import struct
import gps
import serial


####################################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
####################################################################################################
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

    def writeByte(self, value):
        while True:
            try:
                self.bus.write_byte(self.address, value)
                break
            except IOError, err:
                self.misses += 1

    def write8(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        while True:
            try:
                self.bus.write_byte_data(self.address, reg, value)
                break
            except IOError, err:
                self.misses += 1

    def writeList(self, reg, list):
        "Writes an array of bytes using I2C format"
        while True:
            try:
                self.bus.write_i2c_block_data(self.address, reg, list)
                break
            except IOError, err:
                self.misses += 1

    def readU8(self, reg):
        "Read an unsigned byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                return result
            except IOError, err:
                self.misses += 1

    def readS8(self, reg):
        "Reads a signed byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                if (result > 127):
                    return result - 256
                else:
                    return result
            except IOError, err:
                self.misses += 1

    def readU16(self, reg):
        "Reads an unsigned 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readS16(self, reg):
        "Reads a signed 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                if (hibyte > 127):
                    hibyte -= 256
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readList(self, reg, length):
        "Reads a byte array value from the I2C device"
        result = self.bus.read_i2c_block_data(self.address, reg, length)
        return result

    def getMisses(self):
        return self.misses


####################################################################################################
#
#  Gyroscope / Accelerometer class for reading position / movement.  Works with the Invensense IMUs:
#
#  - MPU-6050
#  - MPU-9150
#  - MPU-9250
#
####################################################################################################
class MPU6050:
    i2c = None

    # Registers/etc.
    __MPU6050_RA_SELF_TEST_XG = 0x00
    __MPU6050_RA_SELF_TEST_YG = 0x01
    __MPU6050_RA_SELF_TEST_ZG = 0x02
    __MPU6050_RA_SELF_TEST_XA = 0x0D
    __MPU6050_RA_SELF_TEST_YA = 0x0E
    __MPU6050_RA_SELF_TEST_ZA = 0x0F
    __MPU6050_RA_XG_OFFS_USRH = 0x13
    __MPU6050_RA_XG_OFFS_USRL = 0x14
    __MPU6050_RA_YG_OFFS_USRH = 0x15
    __MPU6050_RA_YG_OFFS_USRL = 0x16
    __MPU6050_RA_ZG_OFFS_USRH = 0x17
    __MPU6050_RA_ZG_OFFS_USRL = 0x18
    __MPU6050_RA_SMPLRT_DIV = 0x19
    __MPU6050_RA_CONFIG = 0x1A
    __MPU6050_RA_GYRO_CONFIG = 0x1B
    __MPU6050_RA_ACCEL_CONFIG = 0x1C
    __MPU9250_RA_ACCEL_CFG_2 = 0x1D
    __MPU6050_RA_FF_THR = 0x1D
    __MPU6050_RA_FF_DUR = 0x1E
    __MPU6050_RA_MOT_THR = 0x1F
    __MPU6050_RA_MOT_DUR = 0x20
    __MPU6050_RA_ZRMOT_THR = 0x21
    __MPU6050_RA_ZRMOT_DUR = 0x22
    __MPU6050_RA_FIFO_EN = 0x23
    __MPU6050_RA_I2C_MST_CTRL = 0x24
    __MPU6050_RA_I2C_SLV0_ADDR = 0x25
    __MPU6050_RA_I2C_SLV0_REG = 0x26
    __MPU6050_RA_I2C_SLV0_CTRL = 0x27
    __MPU6050_RA_I2C_SLV1_ADDR = 0x28
    __MPU6050_RA_I2C_SLV1_REG = 0x29
    __MPU6050_RA_I2C_SLV1_CTRL = 0x2A
    __MPU6050_RA_I2C_SLV2_ADDR = 0x2B
    __MPU6050_RA_I2C_SLV2_REG = 0x2C
    __MPU6050_RA_I2C_SLV2_CTRL = 0x2D
    __MPU6050_RA_I2C_SLV3_ADDR = 0x2E
    __MPU6050_RA_I2C_SLV3_REG = 0x2F
    __MPU6050_RA_I2C_SLV3_CTRL = 0x30
    __MPU6050_RA_I2C_SLV4_ADDR = 0x31
    __MPU6050_RA_I2C_SLV4_REG = 0x32
    __MPU6050_RA_I2C_SLV4_DO = 0x33
    __MPU6050_RA_I2C_SLV4_CTRL = 0x34
    __MPU6050_RA_I2C_SLV4_DI = 0x35
    __MPU6050_RA_I2C_MST_STATUS = 0x36
    __MPU6050_RA_INT_PIN_CFG = 0x37
    __MPU6050_RA_INT_ENABLE = 0x38
    __MPU6050_RA_DMP_INT_STATUS = 0x39
    __MPU6050_RA_INT_STATUS = 0x3A
    __MPU6050_RA_ACCEL_XOUT_H = 0x3B
    __MPU6050_RA_ACCEL_XOUT_L = 0x3C
    __MPU6050_RA_ACCEL_YOUT_H = 0x3D
    __MPU6050_RA_ACCEL_YOUT_L = 0x3E
    __MPU6050_RA_ACCEL_ZOUT_H = 0x3F
    __MPU6050_RA_ACCEL_ZOUT_L = 0x40
    __MPU6050_RA_TEMP_OUT_H = 0x41
    __MPU6050_RA_TEMP_OUT_L = 0x42
    __MPU6050_RA_GYRO_XOUT_H = 0x43
    __MPU6050_RA_GYRO_XOUT_L = 0x44
    __MPU6050_RA_GYRO_YOUT_H = 0x45
    __MPU6050_RA_GYRO_YOUT_L = 0x46
    __MPU6050_RA_GYRO_ZOUT_H = 0x47
    __MPU6050_RA_GYRO_ZOUT_L = 0x48
    __MPU6050_RA_EXT_SENS_DATA_00 = 0x49
    __MPU6050_RA_EXT_SENS_DATA_01 = 0x4A
    __MPU6050_RA_EXT_SENS_DATA_02 = 0x4B
    __MPU6050_RA_EXT_SENS_DATA_03 = 0x4C
    __MPU6050_RA_EXT_SENS_DATA_04 = 0x4D
    __MPU6050_RA_EXT_SENS_DATA_05 = 0x4E
    __MPU6050_RA_EXT_SENS_DATA_06 = 0x4F
    __MPU6050_RA_EXT_SENS_DATA_07 = 0x50
    __MPU6050_RA_EXT_SENS_DATA_08 = 0x51
    __MPU6050_RA_EXT_SENS_DATA_09 = 0x52
    __MPU6050_RA_EXT_SENS_DATA_10 = 0x53
    __MPU6050_RA_EXT_SENS_DATA_11 = 0x54
    __MPU6050_RA_EXT_SENS_DATA_12 = 0x55
    __MPU6050_RA_EXT_SENS_DATA_13 = 0x56
    __MPU6050_RA_EXT_SENS_DATA_14 = 0x57
    __MPU6050_RA_EXT_SENS_DATA_15 = 0x58
    __MPU6050_RA_EXT_SENS_DATA_16 = 0x59
    __MPU6050_RA_EXT_SENS_DATA_17 = 0x5A
    __MPU6050_RA_EXT_SENS_DATA_18 = 0x5B
    __MPU6050_RA_EXT_SENS_DATA_19 = 0x5C
    __MPU6050_RA_EXT_SENS_DATA_20 = 0x5D
    __MPU6050_RA_EXT_SENS_DATA_21 = 0x5E
    __MPU6050_RA_EXT_SENS_DATA_22 = 0x5F
    __MPU6050_RA_EXT_SENS_DATA_23 = 0x60
    __MPU6050_RA_MOT_DETECT_STATUS = 0x61
    __MPU6050_RA_I2C_SLV0_DO = 0x63
    __MPU6050_RA_I2C_SLV1_DO = 0x64
    __MPU6050_RA_I2C_SLV2_DO = 0x65
    __MPU6050_RA_I2C_SLV3_DO = 0x66
    __MPU6050_RA_I2C_MST_DELAY_CTRL = 0x67
    __MPU6050_RA_SIGNAL_PATH_RESET = 0x68
    __MPU6050_RA_MOT_DETECT_CTRL = 0x69
    __MPU6050_RA_USER_CTRL = 0x6A
    __MPU6050_RA_PWR_MGMT_1 = 0x6B
    __MPU6050_RA_PWR_MGMT_2 = 0x6C
    __MPU6050_RA_BANK_SEL = 0x6D
    __MPU6050_RA_MEM_START_ADDR = 0x6E
    __MPU6050_RA_MEM_R_W = 0x6F
    __MPU6050_RA_DMP_CFG_1 = 0x70
    __MPU6050_RA_DMP_CFG_2 = 0x71
    __MPU6050_RA_FIFO_COUNTH = 0x72
    __MPU6050_RA_FIFO_COUNTL = 0x73
    __MPU6050_RA_FIFO_R_W = 0x74
    __MPU6050_RA_WHO_AM_I = 0x75

    #-----------------------------------------------------------------------------------------------
    # Compass output registers when using the I2C master / slave
    #-----------------------------------------------------------------------------------------------
    __MPU9250_RA_MAG_XOUT_L = 0x4A
    __MPU9250_RA_MAG_XOUT_H = 0x4B
    __MPU9250_RA_MAG_YOUT_L = 0x4C
    __MPU9250_RA_MAG_YOUT_H = 0x4D
    __MPU9250_RA_MAG_ZOUT_L = 0x4E
    __MPU9250_RA_MAG_ZOUT_H = 0x4F

    #-----------------------------------------------------------------------------------------------
    # Compass output registers when directly accessing via IMU bypass
    #-----------------------------------------------------------------------------------------------
    __AK893_RA_WIA = 0x00
    __AK893_RA_INFO = 0x01
    __AK893_RA_ST1 = 0x00
    __AK893_RA_X_LO = 0x03
    __AK893_RA_X_HI = 0x04
    __AK893_RA_Y_LO = 0x05
    __AK893_RA_Y_HI = 0x06
    __AK893_RA_Z_LO = 0x07
    __AK893_RA_Z_HI = 0x08
    __AK893_RA_ST2 = 0x09
    __AK893_RA_CNTL1 = 0x0A
    __AK893_RA_RSV = 0x0B
    __AK893_RA_ASTC = 0x0C
    __AK893_RA_TS1 = 0x0D
    __AK893_RA_TS2 = 0x0E
    __AK893_RA_I2CDIS = 0x0F
    __AK893_RA_ASAX = 0x10
    __AK893_RA_ASAY = 0x11
    __AK893_RA_ASAZ = 0x12

    __SCALE_GYRO = (500.0 * math.pi) / (65536 * 180)
    __SCALE_ACCEL = 8.0 / 65536                                                          #AB: +/-4g

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address

        self.max_az = int(65536 / 8)                                                     #AB: +/-4g
        self.min_az = int(65536 / 8)                                                     #AB: +/-4g

        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.az_offset = 0.0

        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0

        logger.info('Reseting MPU-6050')

        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets sample rate to 1kHz/(1+0) = 1kHz or 1ms (note 1kHz assumes dlpf is on - setting
        # dlpf to 0 or 7 changes 1kHz to 8kHz and therefore will require sample rate divider
        # to be changed to 7 to obtain the same 1kHz sample rate.
        #-------------------------------------------------------------------------------------------
        sample_rate_divisor = int(round(adc_frequency / sampling_rate))
        logger.warning("SRD:, %d", sample_rate_divisor)
        self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, sample_rate_divisor - 1)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets clock source to gyro reference w/ PLL
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Gyro DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 =  250Hz @ 8kHz sampling - DO NOT USE, THE ACCELEROMETER STILL SAMPLES AT 1kHz WHICH PRODUCES EXPECTED BUT NOT CODED FOR TIMING AND FIFO CONTENT PROBLEMS
        # 0x01 =  184Hz
        # 0x02 =   92Hz
        # 0x03 =   41Hz
        # 0x04 =   20Hz
        # 0x05 =   10Hz
        # 0x06 =    5Hz
        # 0x07 = 3600Hz @ 8kHz
        #
        # 0x0* FIFO overflow overwrites oldest FIFO contents
        # 0x4* FIFO overflow does not overwrite full FIFO contents
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x40 | glpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable gyro self tests, scale of +/- 250 degrees/s
        #
        # 0x00 =  +/- 250 degrees/s
        # 0x08 =  +/- 500 degrees/s
        # 0x10 = +/- 1000 degrees/s
        # 0x18 = +/- 2000 degrees/s
        # See SCALE_GYRO for conversion from raw data to units of radians per second
        #-------------------------------------------------------------------------------------------
        # int(math.log(degrees / 250, 2)) << 3
        self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Accel DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 = 460Hz
        # 0x01 = 184Hz
        # 0x02 =  92Hz
        # 0x03 =  41Hz
        # 0x04 =  20Hz
        # 0x05 =  10Hz
        # 0x06 =   5Hz
        # 0x07 = 460Hz
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU9250_RA_ACCEL_CFG_2, alpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable accel self tests, scale of +/-4g
        #
        # 0x00 =  +/- 2g
        # 0x08 =  +/- 4g
        # 0x10 =  +/- 8g
        # 0x18 = +/- 16g
        # See SCALE_ACCEL for convertion from raw data to units of meters per second squared
        #-------------------------------------------------------------------------------------------
        # int(math.log(g / 2, 2)) << 3?
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x08)                             #AB: +/-4g
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set INT pin to push/pull, latch 'til read, any read to clear
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x30)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Initialize the FIFO overflow interrupt 0x10 (turned off at startup).
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enabled the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x40)

        #-------------------------------------------------------------------------------------------
        # Accelerometer / gyro goes into FIFO later on - see flushFIFO()
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Read ambient temperature
        #-------------------------------------------------------------------------------------------
        temp = self.readTemperature()
        logger.critical("IMU core temp (boot): ,%f", temp / 333.86 + 21.0)

    def readTemperature(self):
        temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
        return temp

    def enableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Clear the interrupt status register and enable the FIFO overflow interrupt 0x10
        #-------------------------------------------------------------------------------------------
        '''
        #AB! Something odd here: can't clear the GPIO pin if the ISR is enabled, and then status read
        #AB! in that order
        '''

        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x10)
        self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)

    def disableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Disable the FIFO overflow interrupt.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)

    def numFIFOBatches(self):
        #-------------------------------------------------------------------------------------------
        # The FIFO is 512 bytes long, and we're storing 6 signed shorts (ax, ay, az, gx, gy, gz) i.e.
        # 12 bytes per batch of sensor readings
        #-------------------------------------------------------------------------------------------
        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)
        fifo_batches = int(fifo_bytes / 12)  # This rounds down

        return fifo_batches

    def readFIFO(self, fifo_batches):
        #-------------------------------------------------------------------------------------------
        # Read n x 12 bytes of FIFO data averaging, and return the averaged values and inferred time
        # based upon the sampling rate and the number of samples.
        #-------------------------------------------------------------------------------------------
        ax = 0.0
        ay = 0.0
        az = 0.0
        gx = 0.0
        gy = 0.0
        gz = 0.0

        batch_size = 6   # signed shorts: ax, ay, az, gx, gy, gz

        for ii in range(fifo_batches):
            sensor_data = []
            fifo_batch = self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, 12)
            for jj in range(0, 12, 2):
                hibyte = fifo_batch[jj]
                lobyte = fifo_batch[jj + 1]
                if (hibyte > 127):
                    hibyte -= 256

                sensor_data.append((hibyte << 8) + lobyte)

            ax += sensor_data[0]
            ay += sensor_data[1]
            az += sensor_data[2]
            gx += sensor_data[3]
            gy += sensor_data[4]
            gz += sensor_data[5]

            if sensor_data[2] > self.max_az:
                self.max_az = sensor_data[2]

            if sensor_data[2] < self.min_az:
                self.min_az = sensor_data[2]

        ax /= fifo_batches
        ay /= fifo_batches
        az /= fifo_batches
        gx /= fifo_batches
        gy /= fifo_batches
        gz /= fifo_batches

        '''
        if az < (65536 / 8 * 0.8) or az > (65536 / 8 * 2.25):                             #AB: +/-4g
            error_text = "FIFO, %d, %f, %f, %f, %f, %f, %f" % (fifo_batches, ax * self.__SCALE_ACCEL, ay * self.__SCALE_ACCEL, az * self.__SCALE_ACCEL, gx * self.__SCALE_GYRO, gy * self.__SCALE_GYRO, gz * self.__SCALE_GYRO)
            raise IOError(error_text)
        '''

        return ax, ay, az, gx, gy, gz, fifo_batches / sampling_rate

    def flushFIFO(self):
        #-------------------------------------------------------------------------------------------
        # First shut off the feed in the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Empty the FIFO by reading whatever is there
        #-------------------------------------------------------------------------------------------
        SMBUS_MAX_BUF_SIZE = 32

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(int(fifo_bytes / SMBUS_MAX_BUF_SIZE)):
            self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, SMBUS_MAX_BUF_SIZE)

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(fifo_bytes):
            self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)

        #-------------------------------------------------------------------------------------------
        # Finally start feeding the FIFO with sensor data again
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x78)

    def setGyroOffsets(self, gx, gy, gz):
        self.gx_offset = gx
        self.gy_offset = gy
        self.gz_offset = gz

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = (ax - self.ax_offset) * self.__SCALE_ACCEL
        qay = (ay - self.ay_offset) * self.__SCALE_ACCEL
        qaz = (az - self.az_offset) * self.__SCALE_ACCEL

        qrx = (gx - self.gx_offset) * self.__SCALE_GYRO
        qry = (gy - self.gy_offset) * self.__SCALE_GYRO
        qrz = (gz - self.gz_offset) * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz

    def initCompass(self):
        #-------------------------------------------------------------------------------------------
        # Set up the I2C master pass through.
        #-------------------------------------------------------------------------------------------
        int_bypass = self.i2c.readU8(self.__MPU6050_RA_INT_PIN_CFG)
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, int_bypass | 0x02)

        #-------------------------------------------------------------------------------------------
        # Connect directly to the bypassed magnetometer, and configured it for 16 bit continuous data
        #-------------------------------------------------------------------------------------------
        self.i2c_compass = I2C(0x0C)
        self.i2c_compass.write8(self.__AK893_RA_CNTL1, 0x16);

    def readCompass(self):
        compass_bytes = self.i2c_compass.readList(self.__AK893_RA_X_LO, 7)

        #-------------------------------------------------------------------------------------------
        # Convert the array of 6 bytes to 3 shorts - 7th byte kicks off another read.
        # Note compass X, Y, Z are aligned with GPS not IMU i.e. X = 0, Y = 1 => 0 degrees North
        #-------------------------------------------------------------------------------------------
        compass_data = []
        for ii in range(0, 6, 2):
            lobyte = compass_bytes[ii]
            hibyte = compass_bytes[ii + 1]
            if (hibyte > 127):
                hibyte -= 256

            compass_data.append((hibyte << 8) + lobyte)

        [mgx, mgy, mgz] = compass_data

        mgx = (mgx - self.mgx_offset) * self.mgx_gain
        mgy = (mgy - self.mgy_offset) * self.mgy_gain
        mgz = (mgz - self.mgz_offset) * self.mgz_gain

        return mgx, mgy, mgz

    def compassCheckCalibrate(self):
        rc = True
        while True:
            coc = raw_input("'check' or 'calibrate'? ")
            if coc == "check":
                self.checkCompass()
                break
            elif coc == "calibrate":
                rc = self.calibrateCompass()
                break
        return rc

    def checkCompass(self):
        print "Pop me on the ground pointing in a known direction based on another compass."
        raw_input("Press enter when that's done, and I'll tell you which way I think I'm pointing")

        self.loadCompassCalibration()
        mgx, mgy, mgz = self.readCompass()

        #-------------------------------------------------------------------------------
        # Convert compass vector into N, S, E, W variants.  Get the compass angle in the
        # range of 0 - 359.99.
        #-------------------------------------------------------------------------------
        compass_angle = math.atan2(mgx, mgy) * 180 / math.pi
        if compass_angle < 0:
            compass_angle += 360

        #-------------------------------------------------------------------------------
        # There are 16 possible compass directions when you include things like NNE at
        # 22.5 degrees.
        #-------------------------------------------------------------------------------
        compass_points = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")
        num_compass_points = len(compass_points)
        for ii in range(len(compass_points)):

            angle_range_min = 360 * (ii - 0.5) / num_compass_points
            angle_range_max = 360 * (ii + 0.5) / num_compass_points
            if angle_range_max < angle_range_min:
                angle_angle_min -= 360

            if compass_angle > angle_range_min and compass_angle <= angle_range_max:
                break

        print "I think I'm pointing %s?" % compass_points[ii]


    def calibrateCompass(self):
        self.mgx_offset = 0.0
        self.mgy_offset = 0.0
        self.mgz_offset = 0.0
        self.mgx_gain = 1.0
        self.mgy_gain = 1.0
        self.mgz_gain = 1.0
        offs_rc = False

        #-------------------------------------------------------------------------------------------
        # First we need gyro offset calibration.  Flush the FIFO, collect roughly half a FIFO full
        # of samples and feed back to the gyro offset calibrations.
        #-------------------------------------------------------------------------------------------
        raw_input("First, put me on a stable surface, and press enter.")

        mpu6050.flushFIFO()
        time.sleep(20 / sampling_rate)
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO(nfb)
        mpu6050.setGyroOffsets(qrx, qry, qrz)

        print "OK, thanks.  That's the gyro calibrated."

        #-------------------------------------------------------------------------------------------
        # Open the offset file for this run
        #-------------------------------------------------------------------------------------------
        try:
            with open('CompassOffsets', 'ab') as offs_file:

                mgx, mgy, mgz = self.readCompass()
                max_mgx = mgx
                min_mgx = mgx
                max_mgy = mgy
                min_mgy = mgy
                max_mgz = mgz
                min_mgz = mgz

                #-----------------------------------------------------------------------------------
                # Collect compass X. Y compass values
                #-------------------------------------------------------------------------------
                GPIO.output(GPIO_LED, GPIO.HIGH)
                print "Now, pick me up and rotate me horizontally twice until the light stops flashing."
                raw_input("Press enter when you're ready to go.")

                self.flushFIFO()

                yaw = 0.0
                total_dt = 0.0

                print "ROTATION:    ",
                number_len = 0

                #-------------------------------------------------------------------------------
                # While integrated Z axis gyro < 2 pi i.e. 360 degrees, keep flashing the light
                #-------------------------------------------------------------------------------
                while abs(yaw) < 4 * math.pi:
                    time.sleep(10 / sampling_rate)

                    nfb = mpu6050.numFIFOBatches()
                    ax, ay, az, gx, gy, gz, dt = self.readFIFO(nfb)
                    ax, ay, az, gx, gy, gz = self.scaleSensors(ax, ay, az, gx, gy, gz)

                    '''
                    #AB! Do we get Eulers here to rotate back to earth?
                    '''

                    yaw += gz * dt
                    total_dt += dt

                    mgx, mgy, mgz = self.readCompass()

                    max_mgx = mgx if mgx > max_mgx else max_mgx
                    max_mgy = mgy if mgy > max_mgy else max_mgy
                    min_mgx = mgx if mgx < min_mgx else min_mgx
                    min_mgy = mgy if mgy < min_mgy else min_mgy

                    if total_dt > 0.2:
                        total_dt %= 0.2

                        number_text = str(abs(int(yaw * 180 / math.pi)))
                        if len(number_text) == 2:
                            number_text = " " + number_text
                        elif len(number_text) == 1:
                            number_text = "  " + number_text

                        print "\b\b\b\b%s" % number_text,
                        sys.stdout.flush()

                        GPIO.output(GPIO_LED, not GPIO.input(GPIO_LED))
                print

                #-------------------------------------------------------------------------------
                # Collect compass Z values
                #-------------------------------------------------------------------------------
                GPIO.output(GPIO_LED, GPIO.LOW)
                print "\nGreat!  Now do the same but with my nose down."
                raw_input("Press enter when you're ready to go.")

                self.flushFIFO()

                rotation = 0.0
                total_dt = 0.0

                print "ROTATION:    ",
                number_len = 0

                #-------------------------------------------------------------------------------
                # While integrated X+Y axis gyro < 2 pi i.e. 360 degrees, keep flashing the light
                #-------------------------------------------------------------------------------
                while abs(rotation) < 4 * math.pi:
                    time.sleep(10 / sampling_rate)

                    nfb = self.numFIFOBatches()
                    ax, ay, az, gx, gy, gz, dt = self.readFIFO(nfb)
                    ax, ay, az, gx, gy, gz = self.scaleSensors(ax, ay, az, gx, gy, gz)

                    rotation += math.pow(math.pow(gx, 2) + math.pow(gy, 2), 0.5) * dt
                    total_dt += dt

                    mgx, mgy, mgz = self.readCompass()

                    max_mgz = mgz if mgz > max_mgz else max_mgz
                    min_mgz = mgz if mgz < min_mgz else min_mgz

                    if total_dt > 0.2:
                        total_dt %= 0.2

                        number_text = str(abs(int(rotation * 180 / math.pi)))
                        if len(number_text) == 2:
                            number_text = " " + number_text
                        elif len(number_text) == 1:
                            number_text = "  " + number_text

                        print "\b\b\b\b%s" % number_text,
                        sys.stdout.flush()

                        GPIO.output(GPIO_LED, not GPIO.input(GPIO_LED))
                print

                #-------------------------------------------------------------------------------
                # Turn the light off regardless of the result
                #-------------------------------------------------------------------------------
                GPIO.output(GPIO_LED, GPIO.LOW)

                #-------------------------------------------------------------------------------
                # Write the good output to file.
                #-------------------------------------------------------------------------------
                mgx_offset = (max_mgx + min_mgx) / 2
                mgy_offset = (max_mgy + min_mgy) / 2
                mgz_offset = (max_mgz + min_mgz) / 2
                mgx_gain = 1 / (max_mgx - min_mgx)
                mgy_gain = 1 / (max_mgy - min_mgy)
                mgz_gain = 1 / (max_mgz - min_mgz)

                offs_file.write("%f %f %f %f %f %f\n" % (mgx_offset, mgy_offset, mgz_offset, mgx_gain, mgy_gain, mgz_gain))

                #-------------------------------------------------------------------------------
                # Sanity check.
                #-------------------------------------------------------------------------------
                print "\nLooking good, just one last check to confirm all's well."
                self.checkCompass()

                print "All done - ready to go!"
                offs_rc = True

        except EnvironmentError as e:
            print "Environment Error: '%s'" % e

        return offs_rc

    def loadCompassCalibration(self):

        self.mgx_offset = 0.0
        self.mgy_offset = 0.0
        self.mgz_offset = 0.0
        self.mgx_gain = 1.0
        self.mgy_gain = 1.0
        self.mgz_gain = 1.0

        offs_rc = False
        try:
            with open('CompassOffsets', 'rb') as offs_file:
                mgx_offset = 0.0
                mgy_offset = 0.0
                mgz_offset = 0.0
                mgx_gain = 1.0
                mgy_gain = 1.0
                mgz_gain = 1.0

                for line in offs_file:
                    mgx_offset, mgy_offset, mgz_offset, mgx_gain, mgy_gain, mgz_gain = line.split()

                self.mgx_offset = float(mgx_offset)
                self.mgy_offset = float(mgy_offset)
                self.mgz_offset = float(mgz_offset)
                self.mgx_gain = float(mgx_gain)
                self.mgy_gain = float(mgy_gain)
                self.mgz_gain = float(mgz_gain)

        except EnvironmentError:
            #---------------------------------------------------------------------------------------
            # Compass calibration is essential to exclude soft magnetic fields such as from local
            # metal; enforce a recalibration if not found.
            #---------------------------------------------------------------------------------------
            print "Oops, something went wrong reading the compass offsets file 'CompassOffsets'"
            print "Have you calibrated it (--cc)?"

            offs_rc = False
        else:
            #---------------------------------------------------------------------------------------
            # Calibration results were successful.
            #---------------------------------------------------------------------------------------
            offs_rc = True
        finally:
            pass

        logger.warning("Compass Offsets:, %f, %f, %f, Compass Gains:, %f, %f, %f", self.mgx_offset,
                                                                                   self.mgy_offset,
                                                                                   self.mgz_offset,
                                                                                   self.mgx_gain,
                                                                                   self.mgy_gain,
                                                                                   self.mgz_gain)
        return offs_rc

    def calibrate0g(self):
        ax_offset = 0
        ay_offset = 0
        az_offset = 0
        offs_rc = False

        #-------------------------------------------------------------------------------------------
        # Open the ofset file for this run
        #-------------------------------------------------------------------------------------------
        try:
            with open('0gOffsets', 'ab') as offs_file:
                raw_input("Rest me on my props and press enter.")
                self.flushFIFO()
                time.sleep(20 / sampling_rate)
                fifo_batches = self.numFIFOBatches()
                ax, ay, az, gx, gy, gz, dt = self.readFIFO(fifo_batches)
                offs_file.write("%f %f %f\n" % (ax, ay, az))

        except EnvironmentError:
            pass
        else:
            offs_rc = True

        return offs_rc

    def load0gCalibration(self):
        offs_rc = False
        try:
            with open('0gOffsets', 'rb') as offs_file:
                for line in offs_file:
                    ax_offset, ay_offset, az_offset = line.split()
            self.ax_offset = float(ax_offset)
            self.ay_offset = float(ay_offset)
            self.az_offset = float(az_offset)

        except EnvironmentError:
            pass
        else:
            pass
        finally:
            #---------------------------------------------------------------------------------------
            # For a while, I thought 0g calibration might help, but actually, it doesn't due to
            # temperature dependency, so it always returns default values now.
            #---------------------------------------------------------------------------------------
            self.ax_offset = 0.0
            self.ay_offset = 0.0
            self.az_offset = 0.0

            offs_rc = True

        logger.warning("0g Offsets:, %f, %f, %f", self.ax_offset, self.ay_offset, self.az_offset)
        return offs_rc

    def getStats(self):
        return self.max_az * self.__SCALE_ACCEL, self.min_az * self.__SCALE_ACCEL


####################################################################################################
#
#  Garmin LIDAR-Lite V3 range finder
#
####################################################################################################
class GLL:
    i2c = None

    __GLL_ACQ_COMMAND       = 0x00
    __GLL_STATUS            = 0x01
    __GLL_SIG_COUNT_VAL     = 0x02
    __GLL_ACQ_CONFIG_REG    = 0x04
    __GLL_VELOCITY          = 0x09
    __GLL_PEAK_CORR         = 0x0C
    __GLL_NOISE_PEAK        = 0x0D
    __GLL_SIGNAL_STRENGTH   = 0x0E
    __GLL_FULL_DELAY_HIGH   = 0x0F
    __GLL_FULL_DELAY_LOW    = 0x10
    __GLL_OUTER_LOOP_COUNT  = 0x11
    __GLL_REF_COUNT_VAL     = 0x12
    __GLL_LAST_DELAY_HIGH   = 0x14
    __GLL_LAST_DELAY_LOW    = 0x15
    __GLL_UNIT_ID_HIGH      = 0x16
    __GLL_UNIT_ID_LOW       = 0x17
    __GLL_I2C_ID_HIGHT      = 0x18
    __GLL_I2C_ID_LOW        = 0x19
    __GLL_I2C_SEC_ADDR      = 0x1A
    __GLL_THRESHOLD_BYPASS  = 0x1C
    __GLL_I2C_CONFIG        = 0x1E
    __GLL_COMMAND           = 0x40
    __GLL_MEASURE_DELAY     = 0x45
    __GLL_PEAK_BCK          = 0x4C
    __GLL_CORR_DATA         = 0x52
    __GLL_CORR_DATA_SIGN    = 0x53
    __GLL_ACQ_SETTINGS      = 0x5D
    __GLL_POWER_CONTROL     = 0x65

    def __init__(self, address=0x62, rate=10):
        self.i2c = I2C(address)
        self.rate = rate

        #-------------------------------------------------------------------------------------------
        # Set to continuous sampling after initial read.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__GLL_OUTER_LOOP_COUNT, 0xFF)

        #-------------------------------------------------------------------------------------------
        # Set the sampling frequency as 2000 / Hz:
        # 10Hz = 0xc8
        # 20Hz = 0x64
        # 100Hz = 0x14
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__GLL_MEASURE_DELAY, int(2000 / rate))

        #-------------------------------------------------------------------------------------------
        # Include receiver bias correction 0x04
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__GLL_ACQ_COMMAND, 0x04)

        #-------------------------------------------------------------------------------------------
        # Acquisition config register:
        # 0x01 Data ready interrupt
        # 0x20 Take sampling rate from MEASURE_DELAY
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__GLL_ACQ_CONFIG_REG, 0x21)


    def read(self):
        #-------------------------------------------------------------------------------------------
        # Distance is in cm hence the 100s to convert to meters.
        # Velocity is in cm between consecutive reads; sampling rate converts these to a velocity.
        # Reading the list from 0x8F seems to get the previous reading, probably cached for the sake
        # of calculating the velocity next time round.
        #-------------------------------------------------------------------------------------------
        '''
        gll_bytes = self.i2c.readList(0x80 | self.__GLL_LAST_DELAY_HIGH, 2)
        dist1 = gll_bytes[0]
        dist2 = gll_bytes[1]
        distance = ((dist1 << 8) + dist2) / 100
        '''

        '''
        #AB! # BUSY | HEALTHY flags only
        #AB! status = self.i2c.readU8(self.__GLL_STATUS)
        #AB! if status != 0x21:
        #AB!    raise ValueError("0x%x" % status)
        '''

        dist1 = self.i2c.readU8(self.__GLL_FULL_DELAY_HIGH)
        dist2 = self.i2c.readU8(self.__GLL_FULL_DELAY_LOW)
        distance = ((dist1 << 8) + dist2) / 100

        '''
        #AB! Worth trying incase I can drop the I2C usage as a result as it's 
        #AB! used above correctly for the previous reading.
        
        gll_bytes = self.i2c.readList(0x80 | self.__GLL_LAST_DELAY_HIGH, 2)
        dist1 = gll_bytes[0]
        dist2 = gll_bytes[1]
        distance_1 = ((dist1 << 8) + dist2) / 100

        if distance != distance_1:
            print "================WORTH A GO BUT NO===================="
        '''

        velocity = -self.i2c.readS8(self.__GLL_VELOCITY) * self.rate / 100
        return distance, velocity


####################################################################################################
#
# PID algorithm to take input sensor readings, and target requirements, and output an arbirtrary
# corrective value.
#
####################################################################################################
class PID:

    def __init__(self, p_gain, i_gain, d_gain):
        self.last_error = 0.0
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.i_error = 0.0

    def Error(self, input, target):
        return (target - input)


    def Compute(self, input, target, dt):
        #-------------------------------------------------------------------------------------------
        # Error is what the PID alogithm acts upon to derive the output
        #-------------------------------------------------------------------------------------------
        error = self.Error(input, target)

        #-------------------------------------------------------------------------------------------
        # The proportional term takes the distance between current input and target
        # and uses this proportially (based on Kp) to control the ESC pulse width
        #-------------------------------------------------------------------------------------------
        p_error = error

        #-------------------------------------------------------------------------------------------
        # The integral term sums the errors across many compute calls to allow for
        # external factors like wind speed and friction
        #-------------------------------------------------------------------------------------------
        self.i_error += (error + self.last_error) * dt
        i_error = self.i_error

        #-------------------------------------------------------------------------------------------
        # The differential term accounts for the fact that as error approaches 0,
        # the output needs to be reduced proportionally to ensure factors such as
        # momentum do not cause overshoot.
        #-------------------------------------------------------------------------------------------
        d_error = (error - self.last_error) / dt

        #-------------------------------------------------------------------------------------------
        # The overall output is the sum of the (P)roportional, (I)ntegral and (D)iffertial terms
        #-------------------------------------------------------------------------------------------
        p_output = self.p_gain * p_error
        i_output = self.i_gain * i_error
        d_output = self.d_gain * d_error

        #-------------------------------------------------------------------------------------------
        # Store off last error for integral and differential processing next time.
        #-------------------------------------------------------------------------------------------
        self.last_error = error

        #-------------------------------------------------------------------------------------------
        # Return the output, which has been tuned to be the increment / decrement in ESC PWM
        #-------------------------------------------------------------------------------------------
        return p_output, i_output, d_output


####################################################################################################
#
# PID algorithm subclass to come with the yaw angles error calculations.
#
####################################################################################################
class YAW_PID(PID):

    def Error(self, input, target):
        #-------------------------------------------------------------------------------------------
        # An example in degrees is the best way to explain this.  If the target is -179 degrees
        # and the input is +179 degrees, the standard PID output would be -358 degrees leading to
        # a very high yaw rotation rate to correct the -358 degrees error.  However, +2 degrees
        # achieves the same result, with a much lower rotation rate to fix the error.
        #-------------------------------------------------------------------------------------------
        error = ((target - input) + math.pi) % (2 * math.pi) - math.pi
        return error


####################################################################################################
#
#  Class for managing each blade + motor configuration via its ESC
#
####################################################################################################
class ESC:

    def __init__(self, pin, location, rotation, name):
        #-------------------------------------------------------------------------------------------
        # The GPIO BCM numbered pin providing PWM signal for this ESC
        #-------------------------------------------------------------------------------------------
        self.bcm_pin = pin

        #-------------------------------------------------------------------------------------------
        # Physical parameters of the ESC / motors / propellers
        #-------------------------------------------------------------------------------------------
        self.motor_location = location
        self.motor_rotation = rotation

        #-------------------------------------------------------------------------------------------
        # Name - for logging purposes only
        #-------------------------------------------------------------------------------------------
        self.name = name

        #-------------------------------------------------------------------------------------------
        # Pulse width - for logging purposes only
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 0

        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        #-------------------------------------------------------------------------------------------
        self.set(1000)

    def set(self, pulse_width):
        pulse_width = pulse_width if pulse_width >= 1000 else 1000
        pulse_width = pulse_width if pulse_width <= 1999 else 1999

        self.pulse_width = pulse_width

        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, pulse_width)


####################################################################################################
#
# Get the rotation angles of pitch, roll and yaw from the fixed point of earth reference frame
# gravity + lateral orientation (ultimately compass derived, but currently just the take-off
# orientation) of 0, 0, 1 compared to where gravity is distrubuted across the X, Y and Z axes of the
# accelerometer all based upon the right hand rule.
#
####################################################################################################
def GetRotationAngles(ax, ay, az):

    #-----------------------------------------------------------------------------------------------
    # What's the angle in the x and y plane from horizontal in radians?
    #-----------------------------------------------------------------------------------------------
    pitch = math.atan2(-ax, math.pow(math.pow(ay, 2) + math.pow(az, 2), 0.5))
    roll = math. atan2(ay, az)

    return pitch, roll


####################################################################################################
#
# Absolute angles of tilt compared to the earth gravity reference frame.
#
####################################################################################################
def GetAbsoluteAngles(ax, ay, az):

    pitch = math.atan2(-ax, az)
    roll = math.atan2(ay, az)

    return pitch, roll


####################################################################################################
#
# Convert a body frame rotation rate to the rotation frames
#
####################################################################################################
def Body2EulerRates(qry, qrx, qrz, pa, ra):

    #===============================================================================================
    # Axes: Convert a set of gyro body frame rotation rates into Euler frames
    #
    # Matrix
    # ---------
    # |err|   | 1 ,  sin(ra) * tan(pa) , cos(ra) * tan(pa) | |qrx|
    # |epr| = | 0 ,  cos(ra)           ,     -sin(ra)      | |qry|
    # |eyr|   | 0 ,  sin(ra) / cos(pa) , cos(ra) / cos(pa) | |qrz|
    #
    #===============================================================================================
    c_pa = math.cos(pa)
    t_pa = math.tan(pa)
    c_ra = math.cos(ra)
    s_ra = math.sin(ra)

    err = qrx + qry * s_ra * t_pa + qrz * c_ra * t_pa
    epr =       qry * c_ra        - qrz * s_ra
    eyr =       qry * s_ra / c_pa + qrz * c_ra / c_pa

    return epr, err, eyr


####################################################################################################
#
# Rotate a vector using Euler angles wrt Earth frame co-ordinate system, for example to take the
# earth frame target flight plan vectors, and move it to the quad frame orientations vectors.
#
####################################################################################################
def RotateVector(evx, evy, evz, pa, ra, ya):

    #===============================================================================================
    # Axes: Convert a vector from earth- to quadcopter frame
    #
    # Matrix
    # ---------
    # |qvx|   | cos(pa) * cos(ya),                                 cos(pa) * sin(ya),                               -sin(pa)          | |evx|
    # |qvy| = | sin(ra) * sin(pa) * cos(ya) - cos(ra) * sin(ya),   sin(ra) * sin(pa) * sin(ya) + cos(ra) * cos(ya),  sin(ra) * cos(pa)| |evy|
    # |qvz|   | cos(ra) * sin(pa) * cos(ya) + sin(ra) * sin(ya),   cos(ra) * sin(pa) * sin(ya) - sin(ra) * cos(ya),  cos(pa) * cos(ra)| |evz|
    #
    #===============================================================================================
    c_pa = math.cos(pa)
    s_pa = math.sin(pa)
    c_ra = math.cos(ra)
    s_ra = math.sin(ra)
    c_ya = math.cos(ya)
    s_ya = math.sin(ya)

    qvx = evx * c_pa * c_ya                        + evy * c_pa * s_ya                        - evz * s_pa
    qvy = evx * (s_ra * s_pa * c_ya - c_ra * s_ya) + evy * (s_ra * s_pa * s_ya + c_ra * c_ya) + evz * s_ra * c_pa
    qvz = evx * (c_ra * s_pa * c_ya + s_ra * s_ya) + evy * (c_ra * s_pa * s_ya - s_ra * c_ya) + evz * c_pa * c_ra

    return qvx, qvy, qvz


####################################################################################################
#
# Initialize hardware PWM
#
####################################################################################################
RPIO_DMA_CHANNEL = 1

def PWMInit():
    #-----------------------------------------------------------------------------------------------
    # Set up the globally shared single PWM channel
    #-----------------------------------------------------------------------------------------------
    PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
    PWM.setup(1)                                    # 1us resolution pulses
    PWM.init_channel(RPIO_DMA_CHANNEL, 3000)        # pulse every 3ms


####################################################################################################
#
# Cleanup hardware PWM
#
####################################################################################################
def PWMTerm():
    PWM.cleanup()


####################################################################################################
#
# GPIO pins initialization for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOInit(FIFOOverflowISR):
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
    GPIO.add_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.RISING) #, FIFOOverflowISR)

#AB:    GPIO.setup(GPIO_POWER_BROWN_OUT_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
#AB:    GPIO.add_event_detect(GPIO_POWER_BROWN_OUT_INTERRUPT, GPIO.FALLING)

    GPIO.setup(GPIO_GARMIN_BUSY, GPIO.IN, GPIO.PUD_DOWN)
#AB:    GPIO.add_event_detect(GPIO_GARMIN_BUSY, GPIO.FALLING)

    GPIO.setup(GPIO_BUTTON, GPIO.IN, GPIO.PUD_UP)

    GPIO.setup(GPIO_LED, GPIO.OUT)
    GPIO.output(GPIO_LED, GPIO.LOW)


####################################################################################################
#
# GPIO pins cleanup for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOTerm():
#AB:    GPIO.remove_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT)
    GPIO.cleanup()


####################################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
####################################################################################################
def CheckCLI(argv):
    cli_fly = False
    cli_hover_pwm = 1000

    #-----------------------------------------------------------------------------------------------
    # Other configuration defaults
    #-----------------------------------------------------------------------------------------------
    cli_test_case = 0
    cli_diagnostics = False
    cli_rtf_period = 0.1
    cli_tau = 7.5
    cli_calibrate_0g = False
    cli_flight_plan = ''
    cli_cc_compass = False
    cli_yaw_control = False

    hover_pwm_defaulted = True

    #-----------------------------------------------------------------------------------------------
    # Defaults for vertical distance PIDs
    #-----------------------------------------------------------------------------------------------
    cli_vdp_gain = 1.0
    cli_vdi_gain = 0.0
    cli_vdd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Defaults for horizontal distance PIDs
    #-----------------------------------------------------------------------------------------------
    cli_hdp_gain = 1.0
    cli_hdi_gain = 0.0
    cli_hdd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Defaults for horizontal velocity PIDs.  Note the I gain to balance out weight imbalanced.
    #-----------------------------------------------------------------------------------------------
    cli_hvp_gain = 1.5
    cli_hvi_gain = 0.0
    cli_hvd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Per frame specific values.  This is the only place where PID integrals are used to compansate
    # for stable forces such as gravity, weight imbalance in the frame.  Yaw is included here to
    # account for frame unique momentum for required for rotation; however this does not need a
    # integral as there should not be a constant force that needs to be counteracted.
    #-----------------------------------------------------------------------------------------------
    if i_am_hermione:
        #-------------------------------------------------------------------------------------------
        # Hermione's PID configuration due to using her frame / ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_pwm = 1600 # XOAR 12x4 1640 /1600 depending on 150/75g feet; T-Motor Beech 12x4.7 1500

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs.
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 360.0
        cli_vvi_gain = 180.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 90.0
        cli_pri_gain = 0.9
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 90.0
        cli_rri_gain = 0.9
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 180.0
        cli_yri_gain = 0.0
        cli_yrd_gain = 0.0

    elif i_am_zoe:
        #-------------------------------------------------------------------------------------------
        # Zoe's PID configuration due to using her ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_pwm = 1500

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 400.0
        cli_vvi_gain = 200.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 80.0
        cli_pri_gain = 0.8
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 80.0
        cli_rri_gain = 0.8
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 160.0
        cli_yri_gain = 0.0
        cli_yrd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Right, let's get on with reading the command line and checking consistency
    #-----------------------------------------------------------------------------------------------
    try:
        opts, args = getopt.getopt(argv,'df:gh:r:y', ['cc', 'tc=', 'tau=', 'vdp=', 'vdi=', 'vdd=', 'vvp=', 'vvi=', 'vvd=', 'hdp=', 'hdi=', 'hdd=', 'hvp=', 'hvi=', 'hvd=', 'prp=', 'pri=', 'prd=', 'rrp=', 'rri=', 'rrd=', 'tau=', 'yrp=', 'yri=', 'yrd='])
    except getopt.GetoptError:
        logger.critical('Must specify one of -f or -g or --tc')
        logger.critical('  qc.py')
        logger.critical('  -f set the flight plan CSV file')
        logger.critical('  -h set the hover PWM pulse width - default: %dus', cli_hover_pwm)
        logger.critical('  -d enable diagnostics')
        logger.critical('  -g calibrate X, Y axis 0g')
        logger.critical('  -r ??  set the ready-to-fly period - default: %fs', cli_rtf_period)
        logger.critical('  -y use yaw to control the direction of flight')
        logger.critical('  --cc   check or calibrate compass')
        logger.critical('  --tc   select which testcase to run')
        logger.critical('  --tau  set the angle CF -3dB point - default: %fs', cli_tau)
        logger.critical('  --vdp  set vertical distance PID P gain - default: %f', cli_vvp_gain)
        logger.critical('  --vdi  set vertical distance PID P gain - default: %f', cli_vvi_gain)
        logger.critical('  --vdd  set vertical distance PID P gain - default: %f', cli_vvd_gain)
        logger.critical('  --vvp  set vertical speed PID P gain - default: %f', cli_vvp_gain)
        logger.critical('  --vvi  set vertical speed PID P gain - default: %f', cli_vvi_gain)
        logger.critical('  --vvd  set vertical speed PID P gain - default: %f', cli_vvd_gain)
        logger.critical('  --hdp  set horizontal speed PID P gain - default: %f', cli_hdp_gain)
        logger.critical('  --hdi  set horizontal speed PID I gain - default: %f', cli_hdi_gain)
        logger.critical('  --hdd  set horizontal speed PID D gain - default: %f', cli_hdd_gain)
        logger.critical('  --hvp  set horizontal speed PID P gain - default: %f', cli_hvp_gain)
        logger.critical('  --hvi  set horizontal speed PID I gain - default: %f', cli_hvi_gain)
        logger.critical('  --hvd  set horizontal speed PID D gain - default: %f', cli_hvd_gain)
        logger.critical('  --prp  set pitch rotation rate PID P gain - default: %f', cli_prp_gain)
        logger.critical('  --pri  set pitch rotation rate PID I gain - default: %f', cli_pri_gain)
        logger.critical('  --prd  set pitch rotation rate PID D gain - default: %f', cli_prd_gain)
        logger.critical('  --rrp  set roll rotation rate PID P gain - default: %f', cli_rrp_gain)
        logger.critical('  --rri  set roll rotation rate PID I gain - default: %f', cli_rri_gain)
        logger.critical('  --rrd  set roll rotation rate PID D gain - default: %f', cli_rrd_gain)
        logger.critical('  --yrp  set yaw rotation rate PID P gain - default: %f', cli_yrp_gain)
        logger.critical('  --yri  set yaw rotation rate PID I gain - default: %f', cli_yri_gain)
        logger.critical('  --yrd  set yaw rotation rate PID D gain - default: %f', cli_yrd_gain)
        raise ValueError("Invalid command line")

    for opt, arg in opts:
        if opt == '-f':
            cli_fly = True
            cli_flight_plan = arg

        elif opt in '-h':
            cli_hover_pwm = int(arg)
            hover_pwm_defaulted = False

        elif opt in '-d':
            cli_diagnostics = True

        elif opt in '-g':
            cli_calibrate_0g = True

        elif opt in '-r':
            cli_rtf_period = float(arg)

        elif opt in '-y':
            cli_yaw_control = True

        elif opt in '--cc':
            cli_cc_compass = True

        elif opt in '--tc':
            cli_test_case = int(arg)

        elif opt in '--tau':
            cli_tau = float(arg)

        elif opt in '--vdp':
            cli_vdp_gain = float(arg)

        elif opt in '--vdi':
            cli_vdi_gain = float(arg)

        elif opt in '--vdd':
            cli_vdd_gain = float(arg)

        elif opt in '--vvp':
            cli_vvp_gain = float(arg)

        elif opt in '--vvi':
            cli_vvi_gain = float(arg)

        elif opt in '--vvd':
            cli_vvd_gain = float(arg)

        elif opt in '--hdp':
            cli_hdp_gain = float(arg)

        elif opt in '--hdi':
            cli_hdi_gain = float(arg)

        elif opt in '--hdd':
            cli_hdd_gain = float(arg)

        elif opt in '--hvp':
            cli_hvp_gain = float(arg)

        elif opt in '--hvi':
            cli_hvi_gain = float(arg)

        elif opt in '--hvd':
            cli_hvd_gain = float(arg)

        elif opt in '--prp':
            cli_prp_gain = float(arg)

        elif opt in '--pri':
            cli_pri_gain = float(arg)

        elif opt in '--prd':
            cli_prd_gain = float(arg)

        elif opt in '--rrp':
            cli_rrp_gain = float(arg)

        elif opt in '--rri':
            cli_rri_gain = float(arg)

        elif opt in '--rrd':
            cli_rrd_gain = float(arg)

        elif opt in '--yrp':
            cli_yrp_gain = float(arg)

        elif opt in '--yri':
            cli_yri_gain = float(arg)

        elif opt in '--yrd':
            cli_yrd_gain = float(arg)

    if not cli_fly and cli_test_case == 0 and not cli_calibrate_0g and not cli_cc_compass:
        raise ValueError('Must specify one of -f or --tc or --cc')

    elif cli_hover_pwm < 1000 or cli_hover_pwm > 1999:
        raise ValueError('Hover speed must lie in the following range: 1000 <= hover pwm < 2000')

    elif cli_test_case == 0 and cli_fly:
        if not os.path.isfile(cli_flight_plan):
            raise ValueError('The flight plan file "%s" does not exist.' % cli_flight_plan)
        print 'Pre-flight checks passed, enjoy your flight, sir!'

    elif cli_test_case == 0 and cli_calibrate_0g:
        print 'Proceeding with 0g calibration'

    elif cli_test_case == 0 and cli_cc_compass:
        print "Proceeding with compass calibration"

    elif cli_test_case != 1 and cli_test_case != 2:
        raise ValueError('Only 1 or 2 are valid testcases')

    elif cli_test_case == 1 and hover_pwm_defaulted:
        raise ValueError('You must choose a specific hover speed (-h) for test case 1 - try 1150')

    return cli_fly, cli_flight_plan, cli_calibrate_0g, cli_cc_compass, cli_yaw_control, cli_hover_pwm, cli_vdp_gain, cli_vdi_gain, cli_vdd_gain, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hdp_gain, cli_hdi_gain, cli_hdd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_prp_gain, cli_pri_gain, cli_prd_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_yrp_gain, cli_yri_gain, cli_yrd_gain, cli_test_case, cli_rtf_period, cli_tau, cli_diagnostics


####################################################################################################
#
# Functions to lock memory to prevent paging, and move child processes in different process groups
# such that a Ctrl-C / SIGINT to one isn't distributed automatically to all children.
#
####################################################################################################
MCL_CURRENT = 1
MCL_FUTURE  = 2
def mlockall(flags = MCL_CURRENT| MCL_FUTURE):
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.mlockall(flags)
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def munlockall():
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.munlockall()
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def Daemonize():
    os.setpgrp()


####################################################################################################
#
# Start the Scanse Sweep reading process.
#
####################################################################################################
def RecordSweep():

    SWEEP_IGNORE_BOUNDARY = 0.5 # 50cm from Sweep central and the prop tips.
    SWEEP_CRITICAL_BOUNDARY = 1.0 # 50cm or less beyond the ignore zone: Hermione's personal space encroached.
    SWEEP_WARNING_BOUNDARY = 2.0 # 50cm or less beyond the critical zone: Pause for thought what to do next.

    with serial.Serial("/dev/ttySWEEP",
                          baudrate = 115200,
                          parity=serial.PARITY_NONE,
                          bytesize = serial.EIGHTBITS,
                          stopbits = serial.STOPBITS_ONE,
                          xonxoff = False,
                          rtscts = False,
                          dsrdtr = False) as sweep:

        try:
            print "Scanse Sweep open"
            sweep.write("ID\n")
            print "Query device information"
            resp = sweep.readline()
            print "Response: " + resp

            print "Starting scanning...",
            sweep.write("DS\n")
            resp = sweep.readline()
            assert (len(resp) == 6), "Bad data"

            status = resp[2:4]
            if  status == "00":
                print "OK"
            else:
                print "Failed %s" % status
                assert (False), "Corrupt status"

            with io.open("/dev/shm/sweep_stream", mode = "wb", buffering = 0) as sweep_fifo:
                log = open("sweep.csv", "wb")
                log.write("angle, distance, x, y\n")

                unpack_format = '=' + 'B' * 7
                unpack_size = struct.calcsize(unpack_format)

                pack_format = '=??ff'

                while True:
                    raw = sweep.read(unpack_size)
                    assert (len(raw) == unpack_size), "Bad data read: %d" % len(raw)
                    formatted = struct.unpack(unpack_format, raw)
                    assert (len(formatted) == 7), "Bad data type conversion: %d" % len(formatted)

                    #-------------------------------------------------------------------------------
                    # Read the azimuth and convert to degrees.
                    #-------------------------------------------------------------------------------
                    azimuth_lo = formatted[1]
                    azimuth_hi = formatted[2]
                    angle_int = (azimuth_hi << 8) + azimuth_lo
                    degrees = (angle_int >> 4) + (angle_int & 15) / 16

                    #-------------------------------------------------------------------------------
                    # Sweep rotates ACW = - 360, which when slung underneath equates to CW in the piDrone
                    # frame POV.  Do the transformation to +/- 180, and convert to radians.
                    #-------------------------------------------------------------------------------
                    radians = -((degrees + 180) % 360 - 180) * math.pi / 180

                    #-------------------------------------------------------------------------------
                    # Read the distance and convert to meters.
                    #-------------------------------------------------------------------------------
                    distance_lo = formatted[3]
                    distance_hi = formatted[4]
                    distance = ((distance_hi << 8) + distance_lo) / 100

                    #-------------------------------------------------------------------------------
                    # Convert the results to a vector aligned with quad frame.
                    #-------------------------------------------------------------------------------
                    x = distance * math.cos(radians)
                    y = distance * math.sin(radians)

                    log.write("%f, %f, %f, %f\n" % (degrees, distance, x, y))

                    #-------------------------------------------------------------------------------
                    # If a reported distance lies inside the danger zone, pass it over to the autopilot
                    # to react to.
                    #-------------------------------------------------------------------------------
                    if distance < SWEEP_IGNORE_BOUNDARY:
                        pass
                    elif distance < SWEEP_CRITICAL_BOUNDARY:
                        output = struct.pack(pack_format, True, False, distance, radians)
                        sweep_fifo.write(output)
                    elif distance < SWEEP_WARNING_BOUNDARY:
                        output = struct.pack(pack_format, False, True, distance, radians)
                        sweep_fifo.write(output)


                    '''
                    #AB! Full scan (per loop) with timestamp sent to AutopilotProcessor
                    #AB! Combined with compass and GPS (including their timestamps) from other processes, MapConstructor
                    #AB! passes the Ronseal test.
                    '''

        #-------------------------------------------------------------------------------------------
        # Catch Ctrl-C - the 'with' wrapped around the FIFO should have closed that by here.  Has it?
        #-------------------------------------------------------------------------------------------
        except KeyboardInterrupt as e:
            if not sweep_fifo.closed:
                print "Sweep FIFO not closed! WTF!"

        #-------------------------------------------------------------------------------------------
        # Catch incorrect assumption bugs
        #-------------------------------------------------------------------------------------------
        except AssertionError as e:
            print e

        #-------------------------------------------------------------------------------------------
        # Cleanup regardless otherwise the next run picks up data from this
        #-------------------------------------------------------------------------------------------
        finally:
            print "Stop scanning"
            sweep.write("DX\n")
            resp = sweep.read()
            print "Response: %s" % resp
            log.close()


####################################################################################################
#
# Process the Scanse Sweep data.
#
####################################################################################################
class SweepProcessor():

    def __init__(self):
        #-------------------------------------------------------------------------------------------
        # Setup a shared memory based data stream for the Sweep output
        #-------------------------------------------------------------------------------------------
        os.mkfifo("/dev/shm/sweep_stream")

        self.sweep_process = subprocess.Popen(["python", __file__, "SWEEP"], preexec_fn =  Daemonize)
        while True:
            try:
                self.sweep_fifo = io.open("/dev/shm/sweep_stream", mode="rb")
            except:
                continue
            else:
                break

        self.unpack_format = "=??ff"
        self.unpack_size = struct.calcsize(self.unpack_format)

    def flush(self):
        #-------------------------------------------------------------------------------------------
        # Read what should be the backlog of reads, and return how many there are.
        #-------------------------------------------------------------------------------------------
        raw_bytes = self.sweep_fifo.read(self.unpack_size)
        assert (len(raw_bytes) % self.unpack_size == 0), "Incomplete Sweep data received"
        return int(len(raw_bytes) / self.unpack_size)

    def read(self):
        raw_bytes = self.sweep_fifo.read(self.unpack_size)
        assert (len(raw_bytes) == self.unpack_size), "Incomplete data received from Sweep reader"
        critical, warning, distance, direction = struct.unpack(self.unpack_format, raw_bytes)
        return critical, warning, distance, direction

    def cleanup(self):
        #-------------------------------------------------------------------------------------------
        # Stop the Sweep process if it's still running, and clean up the FIFO.
        #-------------------------------------------------------------------------------------------
        '''
        #AB! There's a race here: RecordAutopilot has already finished, and asked Sweep to cleanup,
        #AB! but in the meantime motion processing has caught up and sent RecordAutopilot Ctrl-C via
        #AB: AutopilotProcessor;  RecordAutpilot is waiting here when that happens, so the wait for
        #AB! RecordSweep to exit gets interrupts by the Ctrl-C unless we catch it.  Catching the
        #AB! exception will do for now, but a better solution would be preferable, especially when
        #AB! GPS also feeds into the Autopilot.
        '''
        try:
            if self.sweep_process.poll() == None:
                self.sweep_process.send_signal(signal.SIGINT)
                self.sweep_process.wait()
        except KeyboardInterrupt as e:
            print "===================KB IRQ===================="
        self.sweep_fifo.close()
        os.unlink("/dev/shm/sweep_stream")


####################################################################################################
#
# Start the GPS reading process.
#
####################################################################################################
def RecordGPS():
    session = gps.gps()
    session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

    num_sats = 0
    latitude = 0.0
    longitude = 0.0
    time = ""
    epx = 0.0
    epy = 0.0
    epv = 0.0
    ept = 0.0
    eps = 0.0
    climb = 0.0
    altitude = 0.0
    speed = 0.0
    direction = 0.0

    new_lat = False
    new_lon = False
    new_alt = False

    pack_format = '=dddb' # latitude, longitude, altitude, num satellites

    with io.open("/dev/shm/gps_stream", mode = "wb", buffering = 0) as gps_fifo:
        while True:
            try:
                report = session.next()
                if report['class'] == 'TPV':
                    if hasattr(report, 'time'):  # Time
                        time = report.time

                    if hasattr(report, 'ept'):   # Estimated timestamp error - seconds
                        ept = report.ept

                    if hasattr(report, 'lon'):   # Longitude in degrees
                        longitude = report.lon
                        new_lon = True

                    if hasattr(report, 'epx'):   # Estimated longitude error - meters
                        epx = report.epx

                    if hasattr(report, 'lat'):   # Latitude in degrees
                        latitude = report.lat
                        new_lat = True

                    if hasattr(report, 'epy'):   # Estimated latitude error - meters
                        epy = report.epy

                    if hasattr(report, 'alt'):   # Altitude - meters
                        altitude = report.alt
                        new_alt = True

                    if hasattr(report, 'epv'):   # Estimated altitude error - meters
                        epv = report.epv

                    if hasattr(report, 'track'): # Direction - degrees from true north
                        direction = report.track

                    if hasattr(report, 'epd'):   # Estimated direction error - degrees
                        epd = report.epd

                    if hasattr(report, 'climb'): # Climb velocity - meters per second
                        climb = report.climb

                    if hasattr(report, 'epc'):   # Estimated climb error - meters per seconds
                        epc = report.epc

                    if hasattr(report, 'speed'): # Speed over ground - meters per second
                        speed = report.speed

                    if hasattr(report, 'eps'):   # Estimated speed error - meters per second
                        eps = report.eps


                if report['class'] == 'SKY':
                    if hasattr(report, 'satellites'):
                        num_sats = 0
                        for satellite in report.satellites:
                            if hasattr(satellite, 'used') and satellite.used:
                                num_sats += 1

                #-----------------------------------------------------------------------------
                # Send the new batch.
                #-----------------------------------------------------------------------------
                if new_lon and new_lat and new_alt:

                    new_lon = False
                    new_lat = False
                    new_alt = False

                    output = struct.pack(pack_format,
                                         latitude,
                                         longitude,
                                         altitude,
                                         num_sats)

                    gps_fifo.write(output)
            except KeyError:
                pass
            except KeyboardInterrupt:
                break
            except StopIteration:
                session = None
                break
            finally:
                pass


####################################################################################################
#
# Process the GPS data.
#
####################################################################################################
class GPSProcessor():

    def __init__(self):
        #-------------------------------------------------------------------------------------------
        # Setup a shared memory based data stream for the GPS output
        #-------------------------------------------------------------------------------------------
        os.mkfifo("/dev/shm/gps_stream")

        self.gps_process = subprocess.Popen(["python", __file__, "GPS"], preexec_fn = Daemonize)
        while True:
            try:
                self.gps_fifo = io.open("/dev/shm/gps_stream", mode="rb")
            except:
                continue
            else:
                break

        self.waypoints = []
        self.min_satellites = 7 #AB! Seems to be available whatever the weather.

        self.unpack_format = '=dddb' # latitude, longitude, altitude, num satellites
        self.unpack_size = struct.calcsize(self.unpack_format)

    def flush(self):
        #-------------------------------------------------------------------------------------------
        # Read what should be the backlog of reads, and return how many there are.
        #-------------------------------------------------------------------------------------------
        raw_bytes = self.gps_fifo.read(self.unpack_size)
        assert (len(raw_bytes) % self.unpack_size == 0), "Incomplete GPS data received"
        return int(len(raw_bytes) / self.unpack_size)

    def cleanup(self):
        #-------------------------------------------------------------------------------------------
        # Stop the GPS process
        #-------------------------------------------------------------------------------------------
        self.gps_process.send_signal(signal.SIGINT)
        self.gps_process.wait()
        self.gps_fifo.close()
        os.unlink("/dev/shm/gps_stream")

    def acquireSatellites(self):
        gps_lat = 0.0
        gps_lon = 0.0
        gps_alt = 0.0
        gps_sats = 0

        start_time = time.time()
        print "Gimme up to a minutes to acquire satellites... 0",
        sys.stdout.flush()

        while time.time() - start_time < 60:
            gps_lat, gps_lon, gps_alt, gps_sats = self.read()
            print "\b\b%d" % gps_sats,
            sys.stdout.flush()

            #---------------------------------------------------------------------------------------
            # If we've gpt enough satellites, give up.
            #---------------------------------------------------------------------------------------
            if gps_sats >= self.min_satellites:
                print
                break
        else:
            #---------------------------------------------------------------------------------------
            # We ran out of time trying to get the minimum number of satellites.  Is what we did get
            # enough?
            #---------------------------------------------------------------------------------------
            print
            rsp = raw_input("I only got %d.  Good enough? " % gps_sats)
            if len(rsp) != 0 and (rsp[0] != "y" or rsp[0] != "Y"):
                raise EnvironmentError("I can't see enough satellites, I give up!")

        return gps_lat, gps_lon, gps_alt, gps_sats

    def read(self):
        raw_bytes = self.gps_fifo.read(self.unpack_size)
        assert (len(raw_bytes) == self.unpack_size), "Invalid data block received from GPS reader"
        latitude, longitude, altitude, satellites = struct.unpack(self.unpack_format, raw_bytes)
        return latitude, longitude, altitude, satellites

    def addWaypoint(self):
        latitude, longitude, altitude, num_sats = self.acquireSatellites(min_satellites)
        self.waypoints.append((latitude, longitude))
        logger.critical("WAYPOINT:,lat: %f;,long %f.", latitude, longitude)

    def clearWaypoints(self):
        self.waypoints = []


####################################################################################################
#
# Start the Autopilot reading process
#
####################################################################################################
def RecordAutopilot(flight_plan, motion_rate, sweep_installed):

    phase = 0
    prev_phase = 0

    edx_target = 0.0
    edy_target = 0.0
    edz_target = 0.0

    #-----------------------------------------------------------------------------------------------
    # Create our poll object
    #-----------------------------------------------------------------------------------------------
    poll = select.poll()

    #-----------------------------------------------------------------------------------------------
    # Build the file based flight plan.
    #-----------------------------------------------------------------------------------------------
    X = 0
    Y = 1
    Z = 2
    PERIOD = 3
    NAME = 4

    file_fp = []

    file_fp.append((0.0, 0.0, 0.0, 0.0, "RTF"))

    with open(flight_plan, 'rb') as fp_csv:
        fp_reader = csv.reader(fp_csv)
        for fp_row in fp_reader:

            if len(fp_row) == 0 or (fp_row[0] != '' and fp_row[0][0] == '#'):
                continue
            if len(fp_row) != 5:
                break

            file_fp.append((float(fp_row[0]),
                            float(fp_row[1]),
                            float(fp_row[2]),
                            float(fp_row[3]),
                            fp_row[4].strip()))
        else:
            file_fp.append((0.0, 0.0, 0.0, 0.0, "STOP"))

    #-----------------------------------------------------------------------------------------------
    # Set up the file base flight plan as the active flight plan
    #-----------------------------------------------------------------------------------------------
    active_fp = file_fp
    abort_fp = []

    #-----------------------------------------------------------------------------------------------
    # Kick off sweep if necessary
    #-----------------------------------------------------------------------------------------------
    if sweep_installed:
        sweepp = SweepProcessor()
        sweep_fd = sweepp.sweep_fifo.fileno()
        poll.register(sweep_fd, select.POLLIN | select.POLLPRI)

    #-----------------------------------------------------------------------------------------------
    # Loop for the period of the flight defined by the flight plan contents
    #-----------------------------------------------------------------------------------------------
    pack_format = '=3f?20s?' # edx, edy and edz float targets, bool phase change, string state name, bool running

    try:
        with io.open("/dev/shm/autopilot_stream", mode = "wb", buffering = 0) as autopilot_fifo:
            running = True
            update_time = 0.1 # seconds
            prev_phase = []
            elapsed_time = 0.0
            total_time = 0.0
            for phase in active_fp:
                total_time += phase[PERIOD]

            start_time = time.time()
            while elapsed_time <= total_time:

                #-----------------------------------------------------------------------------------
                # How long is it since we were last here?  Based on that, how long should we sleep (if
                # at all) before working out the next step in the flight plan.
                #-----------------------------------------------------------------------------------
                delta_time = time.time() - start_time - elapsed_time
                elapsed_time += delta_time
                sleep_time = update_time - delta_time if delta_time < update_time else 0.0
                results = poll.poll(sleep_time * 1000)

                #----------------------------------------------------------------------------------
                # Placeholder for Sweep and GPS data receipt
                #----------------------------------------------------------------------------------
                for fd, event in results:
                    if sweep_installed and fd == sweep_fd:
                        try:
                            sweep_critical, sweep_warning, sweep_distance, sweep_direction = sweepp.read()

                            if sweep_critical and active_fp != abort_fp:
                                print "SWEEP ABORT: DISTANCE %fm; DIRECTION %do" % (sweep_distance, sweep_direction * 180 / math.pi)

                                #-------------------------------------------------------------------
                                # What target height has the flight achieved so far? Use this to
                                # determine how long the descent must be at fixed velocity of 0.25m/s
                                #-------------------------------------------------------------------
                                descent_time = edz_target / 0.25

                                #-------------------------------------------------------------------
                                # Build the DESCENT, STOP abort flight plan based upon the
                                # current height.
                                #AB! WIBNI there's a RETREAT state first, where the direction
                                #AB! is the opposite direction of the obstruction detected.
                                #-------------------------------------------------------------------
                                abort_fp.append((0.0, 0.0, -0.25, descent_time, "ABORT (%.2fm)" % edz_target))
                                abort_fp.append((0.0, 0.0, 0.0, 0.0, "STOP"))
                                active_fp = abort_fp

                                #-------------------------------------------------------------------
                                # Need to reset time to start this new flight plan
                                #-------------------------------------------------------------------
                                total_time = descent_time
                                start_time = time.time()
                                elapsed_time = 0.0
                                print "==============OA ABORT==============="

                            elif sweep_warning:
                                #-------------------------------------------------------------------
                                #AB! For obstacle avoidance in the future: hover while considering
                                #AB! what to do about where to go next.
                                #-------------------------------------------------------------------
                                pass

                        except AssertionError as e:
                            print "FMR SWEEP"
                            break

                #-----------------------------------------------------------------------------------
                # Based on the elapsed time since the flight started, find which of the flight plan
                # phases we are in.
                #-----------------------------------------------------------------------------------
                phase_time = 0.0
                for phase in active_fp:
                    phase_time += phase[PERIOD]
                    if elapsed_time <= phase_time:
                        break
                else:
                    running = False

                #-----------------------------------------------------------------------------------
                # Have we crossed into a new phase of the flight plan? Log it if so.
                #-----------------------------------------------------------------------------------
                phase_name = phase[NAME]
                phase_changed = False
                if phase != prev_phase:
                    phase_changed = True
                    prev_phase = phase

                #-----------------------------------------------------------------------------------
                # Get the velocity targets for this phase, and integrate to get distance.  Distance
                # is used in the abort fp generation.
                #-----------------------------------------------------------------------------------
                evx_target = phase[X]
                evy_target = phase[Y]
                evz_target = phase[Z]

                edx_target += evx_target * delta_time
                edy_target += evy_target * delta_time
                edz_target += evz_target * delta_time

                #-----------------------------------------------------------------------------------
                # No point updating the main processor if nothing has changed.
                #-----------------------------------------------------------------------------------
                if not phase_changed:
                    continue

                print "AP phase change: %s" % phase_name
                output = struct.pack(pack_format,
                                     evx_target,
                                     evy_target,
                                     evz_target,
                                     phase_changed,
                                     phase_name,
                                     running)

                autopilot_fifo.write(output)

            else:
                #-----------------------------------------------------------------------------------
                # Ideally the end of flight processing should be here, but duplicates lots from above
                # for no benefit
                #-----------------------------------------------------------------------------------
                pass

    except KeyboardInterrupt as e:
        #-------------------------------------------------------------------------------------------
        # The motion processor is finished with us, we should too, and we have by breaking out of
        # the with.
        #-------------------------------------------------------------------------------------------
        if not autopilot_fifo.closed:
            print "Autopilot FIFO not closed! WTF!"

    finally:
        #-------------------------------------------------------------------------------------------
        # Cleanup Sweep if installed.
        #-------------------------------------------------------------------------------------------
        if sweep_installed:
            print "Stopping Sweep... ",
            sweepp.cleanup()
            print "stopped."



####################################################################################################
#
# Process the Autopilot data.
#
####################################################################################################
class AutopilotProcessor():

    def __init__(self, flight_plan, motion_rate, sweep_installed):
        #-------------------------------------------------------------------------------------------
        # Setup a shared memory based data stream for the Sweep output
        #-------------------------------------------------------------------------------------------
        os.mkfifo("/dev/shm/autopilot_stream")

        self.autopilot_process = subprocess.Popen(["python", __file__, "AUTOPILOT", flight_plan, str(motion_rate), "True" if sweep_installed else "False"], preexec_fn =  Daemonize)
        while True:
            try:
                self.autopilot_fifo = io.open("/dev/shm/autopilot_stream", mode="rb")
            except:
                continue
            else:
                break

        self.unpack_format = "=3f?20s?"
        self.unpack_size = struct.calcsize(self.unpack_format)

    def flush(self):
        #-------------------------------------------------------------------------------------------
        # Read what should be the backlog of reads, and return how many there are.
        #-------------------------------------------------------------------------------------------
        raw_bytes = self.autopilot_fifo.read(self.unpack_size)
        assert (len(raw_bytes) % self.unpack_size == 0), "Incomplete Autopilot data received"
        return int(len(raw_bytes) / self.unpack_size)

    def read(self):
        raw_bytes = self.autopilot_fifo.read(self.unpack_size)
        assert (len(raw_bytes) == self.unpack_size), "Incomplete data received from Autopilot reader"
        evx_target, evy_target, evz_target, state_change, state_name, keep_looping = struct.unpack(self.unpack_format, raw_bytes)
        return evx_target, evy_target, evz_target, state_change, state_name, keep_looping

    def cleanup(self):
        #-------------------------------------------------------------------------------------------
        # Stop the Autopilot process if it's still running, and cleanup the FIFO.
        #-------------------------------------------------------------------------------------------
        if self.autopilot_process.poll() == None:
            self.autopilot_process.send_signal(signal.SIGINT)
            self.autopilot_process.wait()
        self.autopilot_fifo.close()
        os.unlink("/dev/shm/autopilot_stream")


####################################################################################################
#
# Video at 10fps. Each frame is 320 x 320 pixels.  Each macro-block is 16 x 16 pixels.  Due to an
# extra column of macro-blocks (dunno why), that means each frame breaks down into 21 columns by
# 20 rows = 420 macro-blocks, each of which is 4 bytes - 1 signed byte X, 1 signed byte Y and 2 unsigned
# bytes SAD (sum of absolute differences).
#
####################################################################################################
def RecordVideo(frame_width, frame_height, frame_rate):
    with picamera.PiCamera() as camera:
        camera.resolution = (frame_width, frame_height)
        camera.framerate = frame_rate

        '''
        #AB! I have no idea what level is best in range 0 - 100?
        '''
        camera.contrast = 50

        with io.open("/dev/shm/motion_stream", mode = "wb", buffering = 0) as vofi:
            camera.start_recording('/dev/null', format='h264', motion_output=vofi, quality=23)
            try:
                while True:
                    camera.wait_recording(1.0)
            except KeyboardInterrupt:
                pass
            finally:
                camera.stop_recording()

####################################################################################################
#
# Class to process video frame macro-block motion tracking.
#
####################################################################################################
class VideoMotionProcessor:

    def __init__(self, motion_fifo, yaw_increment):
        self.motion_fifo = motion_fifo
        self.yaw_increment = yaw_increment
        self.phase = 0

        mb_size = 16           # 16 x 16 pixels are combined to make a macro-block
        bytes_per_mb = 4       # Each macro-block is 4 bytes, 1 X, 1 Y and 2 SAD

        self.mbs_per_frame = int(round((frame_width / mb_size + 1) * (frame_height / mb_size)))
        self.bytes_per_frame = self.mbs_per_frame * bytes_per_mb

    def flush(self):
        #-------------------------------------------------------------------------------------------
        # Read what should be the backlog of frames, and return how many there are.
        #-------------------------------------------------------------------------------------------
        frame_bytes = self.motion_fifo.read(self.bytes_per_frame)
        assert (len(frame_bytes) % self.bytes_per_frame == 0), "Incomplete video frames received: %f" % (len(frame_bytes) / self.bytes_per_frame)
        return (len(frame_bytes) / self.bytes_per_frame)

    def phase0(self):
        #-------------------------------------------------------------------------------------------
        # Read the video stream and parse into a list of macro-block vectors
        #-------------------------------------------------------------------------------------------
        self.vector_dict = {}
        self.vector_list = []
        self.c_yaw = math.cos(self.yaw_increment)
        self.s_yaw = math.sin(self.yaw_increment)

        sign = 1 # Was '-1 if i_am_chloe else 1' as Chloe had the camera twisted by 180 degrees

        frames = self.motion_fifo.read(self.bytes_per_frame)
        assert (len(frames) != 0), "Shouldn't be here, no bytes to read"
        assert (len(frames) % self.bytes_per_frame == 0), "Incomplete frame bytes read"

        num_frames = int(len(frames) / self.bytes_per_frame)
        assert (num_frames == 1), "Read more than one frame somehow?"

        #-------------------------------------------------------------------------------------------
        # Convert the data to byte, byte, ushort of x, y, sad structure and process them.  The
        # exception here happens when a macro-block is filled with zeros, indicating either a full
        # reset or no movement, and hence no processing required.
        #-------------------------------------------------------------------------------------------
        format = '=' + 'bbH' * self.mbs_per_frame * num_frames
        iframe = struct.unpack(format, frames)
        assert (len(iframe) % 3 == 0), "iFrame size error"

        self.mbs_per_iframe = int(round(len(iframe) / 3 / num_frames))
        assert (self.mbs_per_iframe == self.mbs_per_frame), "iframe mb count different to frame mb count"

        #---------------------------------------------------------------------------------------
        # Split the iframe into a list of macro-block vectors.  The mapping from each iframe
        # in idx, idy depends on how the camera is orientated WRT the frame.
        # This must be checked callibrated.
        #
        # Note: all macro-block vectors are even integers, so we divide them by 2 here for use
        #       walking the vector dictionary for neighbours; we reinstate this at the end.
        #
        #---------------------------------------------------------------------------------------
        for ii in range(self.mbs_per_iframe):
            idx = iframe[3 * ii + 1]
            idy = iframe[3 * ii]
            assert (idx % 2 == 0 and idy % 2 == 0), "Odd (not even) MB vector"

            idx = int(round(sign * idx / 2))
            idy = int(round(sign * idy / 2))

            if idx == 0 and idy == 0:
                continue

            self.vector_list.append((idx, idy))

        #-------------------------------------------------------------------------------------------
        # If the dictionary is empty, this indicates a new frame; this is not an error strictly,
        # more of a reset to tell the outer world about.
        #-------------------------------------------------------------------------------------------
        if len(self.vector_list) == 0:
            raise ValueError("Empty Video Frame Object")

    def phase1(self):
        #-------------------------------------------------------------------------------------------
        # Unyaw the list of vectors, overwriting the yaw list
        #-------------------------------------------------------------------------------------------
        unyawed_vectors = []

        #-------------------------------------------------------------------------------------------
        # Undo the yaw increment - we could use the full rotation matrix here (as elsewhere), but
        # this is more efficient in this very time sensitive function.
        #-------------------------------------------------------------------------------------------
        for vector in self.vector_list:
            idx, idy = vector
            uvx = self.c_yaw * idx - self.s_yaw * idy
            uvy = self.s_yaw * idx + self.c_yaw * idy
            unyawed_vectors.append((int(round(uvx)), int(round(uvy))))

        self.vector_list = unyawed_vectors

    def phase2(self):
        #-------------------------------------------------------------------------------------------
        # Build the dictionary of unyawed vectors; they score 2 because of the next phase
        #-------------------------------------------------------------------------------------------
        for (idx, idy) in self.vector_list:
            if (idx, idy) in self.vector_dict:
                self.vector_dict[(idx, idy)] += 2
            else:
                self.vector_dict[(idx, idy)] = 2

    def phase3(self):
        #-------------------------------------------------------------------------------------------
        # Pass again through the dictionary of vectors, building up clusters based on neighbours.
        #-------------------------------------------------------------------------------------------
        best_score = 0
        self.best_vectors = []

        for vector in self.vector_dict.keys():
            vector_score = self.vector_dict[vector]
            for ii in range(-1, 2):
                for jj in range(-1, 2):
                    if ii == 0 and jj == 0:
                        continue

                    vector_x, vector_y = vector
                    neighbour = (vector_x + ii, vector_y + jj)
                    if neighbour in self.vector_dict:
                        vector_score += self.vector_dict[neighbour]

            if vector_score > best_score:
                best_score = vector_score
                self.best_vectors = [(vector, vector_score)]

            elif vector_score == best_score:
                self.best_vectors.append((vector, vector_score))

    def phase4(self):
        #-------------------------------------------------------------------------------------------
        # Now we've collected the clusters of the best score vectors in the frame, average and reyaw
        # it before returning the result.
        #-------------------------------------------------------------------------------------------
        sum_score = 0
        sum_x = 0
        sum_y = 0

        for (vector_x, vector_y), vector_score in self.best_vectors:
            sum_x += vector_x * vector_score
            sum_y += vector_y * vector_score
            sum_score += vector_score

        best_x = sum_x / sum_score
        best_y = sum_y / sum_score

        #-------------------------------------------------------------------------------------------
        # Redo the yaw increment - we could use the full rotation matrix here (as elsewhere), but
        # this is more efficient in this very time sensitive function.
        #-------------------------------------------------------------------------------------------
        idx = self.c_yaw * best_x + self.s_yaw * best_y
        idy = -self.s_yaw * best_x + self.c_yaw * best_y

        return 2 * idx, 2 * idy

    def process(self):
        assert(self.phase < 5), "Phase shift in motion vector processing"

        #-------------------------------------------------------------------------------------------
        # Phase 0 - load the data and convert into a macro-block vector list
        #-------------------------------------------------------------------------------------------
        if self.phase == 0:
            self.phase0()
            rv = None

        #-------------------------------------------------------------------------------------------
        # Phase 1 - take the list of macro blocks and undo yaw
        #-------------------------------------------------------------------------------------------
        elif self.phase == 1:
            self.phase1()
            rv = None

        #-------------------------------------------------------------------------------------------
        # Phase 2 - build the dictionary of int rounded unyawed vectors
        #-------------------------------------------------------------------------------------------
        elif self.phase == 2:
            self.phase2()
            rv = None

        #-------------------------------------------------------------------------------------------
        # Phase 3 - walk the dictionary, looking for neighbouring clusters and score them
        #-------------------------------------------------------------------------------------------
        elif self.phase == 3:
            self.phase3()
            rv = None

        #-------------------------------------------------------------------------------------------
        # Phase 4 - average highest peak clusters, redo yaw, and return result
        #-------------------------------------------------------------------------------------------
        elif self.phase == 4:
            idx, idy = self.phase4()
            rv = idx, idy

        self.phase += 1
        return rv


####################################################################################################
#
# Class to split initialation, flight startup and flight control
#
####################################################################################################
class Quadcopter:

    MOTOR_LOCATION_FRONT = 0b00000001
    MOTOR_LOCATION_BACK =  0b00000010
    MOTOR_LOCATION_LEFT =  0b00000100
    MOTOR_LOCATION_RIGHT = 0b00001000

    MOTOR_ROTATION_CW = 1
    MOTOR_ROTATION_ACW = 2

    keep_looping = False

    #===============================================================================================
    # One-off initialization
    #===============================================================================================
    def __init__(self):

        #-------------------------------------------------------------------------------------------
        # Who am I?
        #-------------------------------------------------------------------------------------------
        global i_am_zoe
        global i_am_hermione
        i_am_zoe = False
        i_am_hermione = False

        my_name = os.uname()[1]
        if my_name == "zoe.local" or my_name == "zoe":
            print "Hi, I'm Zoe.  Nice to meet you!"
            i_am_zoe = True
        elif my_name == "hermione.local" or my_name == "hermione":
            print "Hi, I'm Hermione.  Nice to meet you!"
            i_am_hermione = True
        else:
            print "Sorry, I'm not qualified to fly this quadcopter."
            return

        #-------------------------------------------------------------------------------------------
        # Set up the global poll object
        #-------------------------------------------------------------------------------------------
        global poll
        poll = select.poll()

        #-------------------------------------------------------------------------------------------
        # Set up extra sensors based on quad identify.
        # -  compass_installed can only be used with an MPU9250
        #-------------------------------------------------------------------------------------------
        X8 = False
        if i_am_zoe:
            self.compass_installed = True
            self.camera_installed = True
            self.gll_installed = True
            self.gps_installed = True
            self.sweep_installed = False
        elif i_am_hermione:
            self.compass_installed = True
            self.camera_installed = True
            self.gll_installed = True
            self.gps_installed = True
            self.sweep_installed = True
            X8 = True

        #-------------------------------------------------------------------------------------------
        # Lock code permanently in memory - no swapping to disk
        #-------------------------------------------------------------------------------------------
        mlockall()

        #-------------------------------------------------------------------------------------------
        # Set up the base logging
        #-------------------------------------------------------------------------------------------
        global logger
        logger = logging.getLogger('QC logger')
        logger.setLevel(logging.INFO)

        #-------------------------------------------------------------------------------------------
        # Create file and console logger handlers - the file is written into shared memory and only
        # dumped to disk / SD card at the end of a flight for performance reasons
        #-------------------------------------------------------------------------------------------
        global file_handler
        file_handler = logging.FileHandler("qcstats.csv", 'w')
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
        # First log, whose flying and under what configuration
        #-------------------------------------------------------------------------------------------
        logger.warning("%s is flying.", "Zoe" if i_am_zoe else "Hermione" if i_am_hermione else "Chloe")

        #-------------------------------------------------------------------------------------------
        # Set the BCM pin assigned to the FIFO overflow
        #-------------------------------------------------------------------------------------------
        global GPIO_POWER_BROWN_OUT_INTERRUPT
        GPIO_POWER_BROWN_OUT_INTERRUPT = 35

        global GPIO_FIFO_OVERFLOW_INTERRUPT
        GPIO_FIFO_OVERFLOW_INTERRUPT = 24 if X8 else 22

        global GPIO_GARMIN_BUSY
        GPIO_GARMIN_BUSY = 5

        global GPIO_BUTTON
        GPIO_BUTTON = 6

        global GPIO_LED
        GPIO_LED = 25

        #-------------------------------------------------------------------------------------------
        # Enable RPIO for ESC PWM.  This must be set up prior to adding the SignalHandler below or it
        # will override what we set thus killing the "Kill Switch"..
        #-------------------------------------------------------------------------------------------
        PWMInit()

        #-------------------------------------------------------------------------------------------
        # Enable GPIO for the FIFO overflow interrupt.
        #-------------------------------------------------------------------------------------------
        GPIOInit(self.fifoOverflowISR)

        #-------------------------------------------------------------------------------------------
        # Set the signal handler here so the core processing loop can be stopped (or not started) by
        # Ctrl-C.
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, self.shutdownSignalHandler)

        #-------------------------------------------------------------------------------------------
        # Phoebe, Chloe and Zoe have similar custom PCBs so share the same PWM pin layouts.
        #-------------------------------------------------------------------------------------------
        ESC_BCM_FLT = 0
        ESC_BCM_FRT = 0
        ESC_BCM_BLT = 0
        ESC_BCM_BRT = 0
        ESC_BCM_FLU = 0
        ESC_BCM_FRU = 0
        ESC_BCM_BLU = 0
        ESC_BCM_BRU = 0

        if not X8:
            ESC_BCM_FLT = 27
            ESC_BCM_FRT = 17
            ESC_BCM_BLT = 26
            ESC_BCM_BRT = 19
        else:
            ESC_BCM_FLT = 27
            ESC_BCM_FRT = 17
            ESC_BCM_BLT = 26
            ESC_BCM_BRT = 20
            ESC_BCM_FLU = 22
            ESC_BCM_FRU = 23
            ESC_BCM_BLU = 16
            ESC_BCM_BRU = 19


        pin_list = [ESC_BCM_FLT,
                    ESC_BCM_FRT,
                    ESC_BCM_BLT,
                    ESC_BCM_BRT,
                    ESC_BCM_FLU,
                    ESC_BCM_FRU,
                    ESC_BCM_BLU,
                    ESC_BCM_BRU]


        location_list = [self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_RIGHT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_RIGHT,
                         self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_RIGHT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_RIGHT]

        rotation_list = [self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_CW,
                         self.MOTOR_ROTATION_CW,
                         self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_CW,
                         self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_CW]

        name_list = ['front left topside',
                     'front right topside',
                     'back left topside',
                     'back right topside',
                     'front left underside',
                     'front right underside',
                     'back left underside',
                     'back right underside']

        #-------------------------------------------------------------------------------------------
        # Prime the ESCs to stop their anonying beeping!  All 4 of P, C, H & Z  use the T-motor ESCs
        # with the same ESC firmware so have the same spin_pwm
        #-------------------------------------------------------------------------------------------
        global stfu_pwm
        global spin_pwm
        stfu_pwm = 1000
        spin_pwm = 0
        if i_am_zoe:
            spin_pwm = 1150
        elif i_am_hermione:
            spin_pwm = 1150

        self.esc_list = []
        for esc_index in range(8 if X8 else 4):
            esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
            self.esc_list.append(esc)

        #===========================================================================================
        # Globals for the IMU setup
        # adc_frequency      - the sampling rate of the ADC
        # sampling_rate      - the data sampling rate and thus data ready interrupt rate
        #                      motion processing.
        # motion_rate        - the target frequency motion processing occurs under perfect conditions.
        # alpf               - the accelerometer low pass filter
        # glpf               - the gyro low pass filter
        #===========================================================================================
        global adc_frequency
        global sampling_rate
        global motion_rate
        adc_frequency = 1000        #AB: defined by dlpf >= 1; DO NOT USE ZERO => 8000 adc_frequency

        if self.camera_installed or self.gll_installed:
            if i_am_hermione:
                '''
                #AB? Can this be moved back to 500/100 Hz when GPS etc move to autopilot?
                #AB? Or even 1000/100?  Does 500/100 break external inputs plus logging?
                '''
                sampling_rate = 500 # Hz
                motion_rate = 75    # Hz
            elif i_am_zoe:
                sampling_rate = 500 # Hz
                motion_rate = 75    # Hz
        else:
            sampling_rate = 1000    # Hz
            motion_rate = 100       # Hz

        glpf = 1                    #AB: 184Hz    

        #-------------------------------------------------------------------------------------------
        # This is not for antialiasing: the accelerometer low pass filter happens between the ADC
        # rate and our IMU sampling rate.  ADC rate is 1kHz through this case.  However, I've seen poor 
        # behavious in double integration when IMU sampling rate is 500Hz and alpf = 460Hz. 
        #-------------------------------------------------------------------------------------------
        if sampling_rate == 1000:   #AB: SRD = 0 (1kHz)
            alpf = 0                #AB: alpf = 460Hz
        elif sampling_rate == 500:  #AB: SRD = 1 (500Hz)
            alpf = 1                #AB: alpf = 184Hz
        elif sampling_rate >= 200:  #AB: SRD = 2, 3, 4 (333, 250, 200Hz)
            alpf = 2                #AB: alpf = 92Hz
        elif sampling_rate >= 100:  #AB: SRD = 5, 6, 7, 8, 9 (166, 143, 125, 111, 100Hz)
            alpf = 3                #ABL alpf = 41Hz    
        else:
            #--------------------------------------------------------------------------------------
            # There's no point going less than 100Hz IMU sampling; we need about 100Hz motion 
            # processing for an level of stability. 
            #--------------------------------------------------------------------------------------
            print "SRD + alpf useless: forget it!"
            return
                 
        global mpu6050
        mpu6050 = MPU6050(0x68, alpf, glpf)

        #-------------------------------------------------------------------------------------------
        # Scheduling parameters defining standard, and critical FIFO block counts
        #
        # FIFO_SPARE_LIMIT    - The aim is to run the motion processing every 10ms (100Hz) regardless
        #                       of the frequency the IMU is sampled.
        # FIFO_OVERFLOW_LIMIT - The aim is to abort processing when the FIFO is nearly full so that
        #                       it doesn't overflow during the processing.  We set an arbitrary limit
        #                       of 10ms for the processing time - it's less than this
        #
        # 512/12 is the maximum number of batches in the IMU FIFO
        # 1000 / sampling_rate is the time fraction - slower sampling means more spare time available
        #
        #-------------------------------------------------------------------------------------------
        self.FIFO_PRIORITY = int(round(sampling_rate / motion_rate))
        self.FIFO_OVERFLOW = int(round(512 / 12 - sampling_rate / motion_rate))

        #-------------------------------------------------------------------------------------------
        # Initialize the compass object.
        #-------------------------------------------------------------------------------------------
        if self.compass_installed:
            mpu6050.initCompass()

        #-------------------------------------------------------------------------------------------
        # Initialize the Garmin LiDAR-Lite V3 at 10Hz - this is also used for the camera frame rate.
        #AB? The only time I tried 20 on a dimly lit lawn, it leapt up and crashed down.
        #-------------------------------------------------------------------------------------------
        self.tracking_rate = 10
        if self.gll_installed:
            global gll
            gll = GLL(rate = self.tracking_rate)

        #-------------------------------------------------------------------------------------------
        # Initialise GPS
        #-------------------------------------------------------------------------------------------
        if self.gps_installed:
            global gpsp
            gpsp = GPSProcessor()

    #===============================================================================================
    # Keyboard input between flights for CLI update etc
    #===============================================================================================
    def go(self):
        while True:
            print "============================================"
            cli_argv = raw_input("Wassup? ")
            print "============================================"
            if len(cli_argv) != 0 and (cli_argv == 'exit' or cli_argv == 'quit'):
                self.shutdown()

            self.argv = sys.argv[1:] + cli_argv.split()
            self.fly()

    #===============================================================================================
    # Per-flight configuration, initializations and flight control itself
    #===============================================================================================
    def fly(self):
        #-------------------------------------------------------------------------------------------
        # Check the command line for calibration or flight parameters
        #-------------------------------------------------------------------------------------------
        print "Just checking a few details.  Gimme a few seconds..."
        try:
            flying, flight_plan, calibrate_0g, cc_compass, yaw_control, hover_pwm, vdp_gain, vdi_gain, vdd_gain, vvp_gain, vvi_gain, vvd_gain, hdp_gain, hdi_gain, hdd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, rtf_period, atau, diagnostics = CheckCLI(self.argv)
        except ValueError, err:
            print "Command line error: %s" % err
            return

        logger.warning("fly = %s, flight plan = %s, calibrate_0g = %d, check / calibrate compass = %s, yaw_control = %s, hover_pwm = %d, vdp_gain = %f, vdi_gain = %f, vdd_gain= %f, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hdp_gain = %f, hdi_gain = %f, hdd_gain = %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, prp_gain = %f, pri_gain = %f, prd_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, yrp_gain = %f, yri_gain = %f, yrd_gain = %f, test_case = %d, rtf_period = %f, atau = %f, diagnostics = %s",
                flying, flight_plan, calibrate_0g, cc_compass, yaw_control, hover_pwm, vdp_gain, vdi_gain, vdd_gain, vvp_gain, vvi_gain, vvd_gain, hdp_gain, hdi_gain, hdd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, rtf_period, atau, diagnostics)

        #-------------------------------------------------------------------------------------------
        # Calibrate gravity or use previous settings
        #-------------------------------------------------------------------------------------------
        if calibrate_0g:
            if not mpu6050.calibrate0g():
                print "0g calibration error, abort"
            return
        elif not mpu6050.load0gCalibration():
            print "0g calibration data not found."
            return

        #-------------------------------------------------------------------------------------------
        # Calibrate compass.
        #-------------------------------------------------------------------------------------------
        if self.compass_installed:
            if cc_compass:
                if not mpu6050.compassCheckCalibrate():
                    print "Compass check / calibration error, abort"
                return
            elif not mpu6050.loadCompassCalibration():
                print "Compass calibration data not found"
                return
        elif cc_compass:
            print "Compass not installed, check / calibration not possible."
            return

        #-------------------------------------------------------------------------------------------
        # START TESTCASE 1 CODE: spin up each blade individually for 5s each and check they all turn
        #                        the right way.  At the same time, log X, Y and Z accelerometer readings
        #                        to measure noise from the motors and props due to possible prop and motor
        #                        damage.
        #-------------------------------------------------------------------------------------------
        if test_case == 1:
            print "TESTCASE 1: Check props are spinning as expected"
            for esc in self.esc_list:
                print "%s prop should rotate %s." % (esc.name, "anti-clockwise" if esc.motor_rotation == self.MOTOR_ROTATION_ACW else "clockwise")

                #-----------------------------------------------------------------------------------
                # Get the prop up to the configured spin rate.  Sleep for 5s then stop and move
                # on to the next prop.
                #-----------------------------------------------------------------------------------
                esc.set(hover_pwm)
                time.sleep(5)
                esc.set(stfu_pwm)
            return
        #-------------------------------------------------------------------------------------------
        # END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the
        #                      right way
        #-------------------------------------------------------------------------------------------

        #===========================================================================================
        # OK, we're in flight mode, better get on with it
        #===========================================================================================
        self.keep_looping = True

        edx_target = 0.0
        edy_target = 0.0
        edz_target = 0.0

        evx_target = 0.0
        evy_target = 0.0
        evz_target = 0.0

        ya_target = 0.0

        qdx_input = 0.0
        qdy_input = 0.0
        qdz_input = 0.0

        qvx_input = 0.0
        qvy_input = 0.0
        qvz_input = 0.0

        edx_fuse = 0.0
        edy_fuse = 0.0
        edz_fuse = 0.0

        evx_fuse = 0.0
        evy_fuse = 0.0
        evz_fuse = 0.0

        qdx_fuse = 0.0
        qdy_fuse = 0.0
        qdz_fuse = 0.0

        qvx_fuse = 0.0
        qvy_fuse = 0.0
        qvz_fuse = 0.0

        #===========================================================================================
        # Tuning: Set up the PID gains - some are hard coded mathematical approximations, some come
        # from the CLI parameters to allow for tuning  - 10 in all
        # - Quad X axis distance
        # - Quad Y axis distance
        # - Quad Z axis distance
        # - Quad X axis velocity
        # - Quad Y axis velocity
        # - Quad Z axis velocity
        # - Pitch rotation rate
        # - Roll Rotation rate
        # - Yaw angle
        # = Yaw Rotation rate
        #===========================================================================================

        #-------------------------------------------------------------------------------------------
        # The quad X axis PID controls fore / aft distance
        #-------------------------------------------------------------------------------------------
        PID_QDX_P_GAIN = hdp_gain
        PID_QDX_I_GAIN = hdi_gain
        PID_QDX_D_GAIN = hdd_gain

        #-------------------------------------------------------------------------------------------
        # The quad Y axis PID controls left / right distance
        #-------------------------------------------------------------------------------------------
        PID_QDY_P_GAIN = hdp_gain
        PID_QDY_I_GAIN = hdi_gain
        PID_QDY_D_GAIN = hdd_gain

        #-------------------------------------------------------------------------------------------
        # The quad Z axis PID controls up / down distance
        #-------------------------------------------------------------------------------------------
        PID_QDZ_P_GAIN = vdp_gain
        PID_QDZ_I_GAIN = vdi_gain
        PID_QDZ_D_GAIN = vdd_gain

        #-------------------------------------------------------------------------------------------
        # The quad X axis speed controls fore / aft speed
        #-------------------------------------------------------------------------------------------
        PID_QVX_P_GAIN = hvp_gain
        PID_QVX_I_GAIN = hvi_gain
        PID_QVX_D_GAIN = hvd_gain

        #-------------------------------------------------------------------------------------------
        # The quad Y axis speed PID controls left / right speed
        #-------------------------------------------------------------------------------------------
        PID_QVY_P_GAIN = hvp_gain
        PID_QVY_I_GAIN = hvi_gain
        PID_QVY_D_GAIN = hvd_gain

        #-------------------------------------------------------------------------------------------
        # The quad Z axis speed PID controls up / down speed
        #-------------------------------------------------------------------------------------------
        PID_QVZ_P_GAIN = vvp_gain
        PID_QVZ_I_GAIN = vvi_gain
        PID_QVZ_D_GAIN = vvd_gain

        #-------------------------------------------------------------------------------------------
        # The roll angle PID controls stable angles around the X-axis
        #-------------------------------------------------------------------------------------------
        PID_PA_P_GAIN = 2.0 # pap_gain
        PID_PA_I_GAIN = 0.0 # pai_gain
        PID_PA_D_GAIN = 0.0 # pad_gain

        #-------------------------------------------------------------------------------------------
        # The pitch rate PID controls stable rotation rate around the Y-axis
        #-------------------------------------------------------------------------------------------
        PID_PR_P_GAIN = prp_gain
        PID_PR_I_GAIN = pri_gain
        PID_PR_D_GAIN = prd_gain

        #-------------------------------------------------------------------------------------------
        # The roll angle PID controls stable angles around the X-axis
        #-------------------------------------------------------------------------------------------
        PID_RA_P_GAIN = 2.0 # rap_gain
        PID_RA_I_GAIN = 0.0 # rai_gain
        PID_RA_D_GAIN = 0.0 # rad_gain

        #-------------------------------------------------------------------------------------------
        # The roll rate PID controls stable rotation rate around the X-axis
        #-------------------------------------------------------------------------------------------
        PID_RR_P_GAIN = rrp_gain
        PID_RR_I_GAIN = rri_gain
        PID_RR_D_GAIN = rrd_gain

        #-------------------------------------------------------------------------------------------
        # The yaw angle PID controls stable angles around the Z-axis
        #-------------------------------------------------------------------------------------------
        PID_YA_P_GAIN = 6.0 # yap_gain
        PID_YA_I_GAIN = 0.0 # yai_gain
        PID_YA_D_GAIN = 0.0 # yad_gain

        #-------------------------------------------------------------------------------------------
        # The yaw rate PID controls stable rotation speed around the Z-axis
        #-------------------------------------------------------------------------------------------
        PID_YR_P_GAIN = yrp_gain
        PID_YR_I_GAIN = yri_gain
        PID_YR_D_GAIN = yrd_gain

        #-------------------------------------------------------------------------------------------
        # Start the X, Y (horizontal) and Z (vertical) distance PID
        #-------------------------------------------------------------------------------------------
        qdx_pid = PID(PID_QDX_P_GAIN, PID_QDX_I_GAIN, PID_QDX_D_GAIN)
        qdy_pid = PID(PID_QDY_P_GAIN, PID_QDY_I_GAIN, PID_QDY_D_GAIN)
        qdz_pid = PID(PID_QDZ_P_GAIN, PID_QDZ_I_GAIN, PID_QDZ_D_GAIN)

        #-------------------------------------------------------------------------------------------
        # Start the X, Y (horizontal) and Z (vertical) velocity PIDs
        #-------------------------------------------------------------------------------------------
        qvx_pid = PID(PID_QVX_P_GAIN, PID_QVX_I_GAIN, PID_QVX_D_GAIN)
        qvy_pid = PID(PID_QVY_P_GAIN, PID_QVY_I_GAIN, PID_QVY_D_GAIN)
        qvz_pid = PID(PID_QVZ_P_GAIN, PID_QVZ_I_GAIN, PID_QVZ_D_GAIN)

        #-------------------------------------------------------------------------------------------
        # Start the pitch, roll and yaw angle PID - note the different PID class for yaw.
        #-------------------------------------------------------------------------------------------
        pa_pid = PID(PID_PA_P_GAIN, PID_PA_I_GAIN, PID_PA_D_GAIN)
        ra_pid = PID(PID_RA_P_GAIN, PID_RA_I_GAIN, PID_RA_D_GAIN)
        ya_pid = YAW_PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN)

        #-------------------------------------------------------------------------------------------
        # Start the pitch, roll and yaw rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN)
        rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN)
        yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN)

        #-------------------------------------------------------------------------------------------
        # Set up the global constants - gravity in meters per second squared
        #-------------------------------------------------------------------------------------------
        GRAV_ACCEL = 9.80665
        EARTH_RADIUS = 6371000 # meters

        #-------------------------------------------------------------------------------------------
        # Set up the constants for motion fusion if we have lateral and vertical distance / velocity
        # sensors.
        # - vvf, hvf, vdf, hdf flag which must all be set to true for fusion to be triggered
        # - fusion_tau used for the fusion complementary filter - it's set to 10 samples at 10 Hz = 1s
        #-------------------------------------------------------------------------------------------
        if self.gll_installed or self.camera_installed:
            vvf = False
            hvf = False
            vdf = False
            hdf = False
            fusion_tau = 10 / self.tracking_rate

        #------------------------------------------------------------------------------------------
        # Set the props spinning at their base rate to ensure initial kick-start doesn't get spotted
        # by the sensors messing up the flight thereafter. spin_pwm is determined by running testcase 1
        # multiple times incrementing -h slowly until a level of PWM is found where all props just spin.
        # This depends on the firmware in the ESCs
        #------------------------------------------------------------------------------------------
        print "Starting up the motors..."

        for esc in self.esc_list:
            esc.set(spin_pwm)

        #-------------------------------------------------------------------------------------------
        # Initialize the base setting of earth frame take-off height - i.e. the vertical distance from
        # the height sensor or the take-off platform / leg height if no sensor is available.
        #-------------------------------------------------------------------------------------------
        eftoh = 0.0

        #-------------------------------------------------------------------------------------------
        # Get an initial take-off height
        #-------------------------------------------------------------------------------------------
        g_dist = 0.0
        if self.gll_installed:
            print "Couple of seconds to let the LiDAR settle..."
            for ii in range(2 * self.tracking_rate):
                time.sleep(1 / self.tracking_rate)
                try:
                    g_dist, g_vel = gll.read()
                except ValueError as e:
                    print "GLL status 1: %s" % e
                    continue
                eftoh += g_dist
            eftoh /= (2 * self.tracking_rate)

        #AB!----------------------------------------------------------------------------------------
        #AB! Because Hermione's GLL is too close to the ground, the above typically comes up with
        #AB! 5cm so here we override it with the ruler measured closer approximation.
        #AB!----------------------------------------------------------------------------------------
        # if i_am_hermione:
        #     eftoh = 0.13

        #-------------------------------------------------------------------------------------------
        # Set up the video macro-block parameters
        # Video supported upto 1080p @ 30Hz but restricted by speed of macro-block processing.
        #-------------------------------------------------------------------------------------------
        vmp = None
        vmpt = 0.0
        pvmpt = 0.0

        if self.camera_installed:
            print "Couple of seconds to let the video settle..."

            global frame_width
            global frame_height

            if i_am_hermione:
                frame_width = 320    # an exact multiple of mb_size (320 = well lit gravel 1msquare.csv passed)
            elif i_am_zoe:
                frame_width = 320    # an exact multiple of mb_size
            frame_height = frame_width
            frame_rate = self.tracking_rate

            camera_data_update = False

            #------------------------------------------------------------------------------------------
            # Scale is the convertion from macro-blocks to meters at a given height.
            # - V1 camera angle of view (aov): 54 x 41 degrees
            # - V2 camera angle of view (aov): 62.2 x 48.8 degrees.
            # Because we're shooting a 400 x 400 video from with a V2 camera this means a macro-block is
            # 2 x height x tan ( aov / 2) / 400 meters.
            #
            # The macro-block vectors represent the movement of a macro-block between frames.  This is
            # implied by the fact that each vector can only be between +/- 128 in X and Y which allows for
            # shifts up to +/- 2048 pixels in a frame which seems reasonable given the h.264 compression.
            #
            # scale just needs to be multiplied by (macro-block shift x height) to produce the increment of
            # horizontal movement in meters.
            #------------------------------------------------------------------------------------------
            camera_version = 2
            aov = (48.8 if camera_version == 2 else 41) * math.pi / 180
            scale = 2 * math.tan(aov / 2) / frame_width

            #---------------------------------------------------------------------------------------
            # Setup a shared memory based data stream for the PiCamera video motion output
            #---------------------------------------------------------------------------------------
            os.mkfifo("/dev/shm/motion_stream")
            video_process = subprocess.Popen(["python", __file__, "MOTION", str(frame_width), str(frame_height), str(frame_rate)], preexec_fn = Daemonize)

            while True:
                try:
                    motion_fifo = io.open("/dev/shm/motion_stream", mode="rb")
                except:
                    continue
                else:
                    break

            #---------------------------------------------------------------------------------------
            # Register fd for polling
            #---------------------------------------------------------------------------------------
            motion_fd = motion_fifo.fileno()
            poll.register(motion_fd, select.POLLIN | select.POLLPRI)
            logger.warning("Video @, %d, %d,  pixels, %d, fps", frame_width, frame_height, frame_rate)

        #-------------------------------------------------------------------------------------------
        # See how many satellites GPS can pick up.
        #-------------------------------------------------------------------------------------------
        gps_lat = 0.0
        gps_lon = 0.0
        gps_alt = 0.0
        gps_sats = 0
        gps_ns = 0.0
        gps_ew = 0.0
        gps_dalt = 0.0
        if self.gps_installed:
            print "Couple of seconds to set up the GPS..."

            gps_fifo = gpsp.gps_fifo
            gps_fd = gps_fifo.fileno()
            poll.register(gps_fd, select.POLLIN | select.POLLPRI)

            #---------------------------------------------------------------------------------------
            # Get an initial GPS result.  If we don't get enough and the user is insistent we should
            # the EnvironmentError is raised, and we abort the flight.  We're don't return here as
            # we need to do the tidy up later at the end of the while self.keep_looping:
            #---------------------------------------------------------------------------------------
            try:
                gps_lat, gps_lon, gps_alt, gps_sats = gpsp.acquireSatellites()
            except EnvironmentError as e:
                print e
                self.keep_looping = False
            base_lat = gps_lat
            base_lon = gps_lon
            base_alt = gps_alt

            logger.warning("GPS location: latitude %f; longitude %f; altitude %f, satellites %d.", gps_lon, gps_lat, gps_alt, gps_sats)

        #--------------------------------------------------------------------------------------------
        # Last chance to change your mind about the flight if all's ok so far
        #--------------------------------------------------------------------------------------------
        rtg = raw_input("Ready when you are!")
        if len(rtg) != 0:
            print "OK, I'll skip"
            self.keep_looping = False

        print ""
        print "################################################################################"
        print "#                                                                              #"
        print "#                             Thunderbirds are go!                             #"
        print "#                                                                              #"
        print "################################################################################"
        print ""

        #-------------------------------------------------------------------------------------------
        # Used for total flight length only
        #-------------------------------------------------------------------------------------------
        start_flight = time.time()

        #-------------------------------------------------------------------------------------------
        # Flush any OS FIFO content
        #-------------------------------------------------------------------------------------------
        gps_flush = 0
        video_flush = 0
        flushing = True
        while flushing:
            results = poll.poll(0.0)
            for fd, event in results:

                if self.gps_installed and fd == gps_fd:
                    gps_flush += gpsp.flush()

                if self.camera_installed and fd == motion_fd:
                    video_flush += VideoMotionProcessor(motion_fifo, 0).flush()

            else:
                if len(results) == 0:
                    flushing = False
        else:
            print "GPS Flush: %d" % gps_flush
            print "Video Flush: %d" % video_flush

        #-------------------------------------------------------------------------------------------
        # Start the autopilot
        #-------------------------------------------------------------------------------------------
        app = AutopilotProcessor(flight_plan, motion_rate, self.sweep_installed)
        autopilot_fifo = app.autopilot_fifo
        autopilot_fd = autopilot_fifo.fileno()
        poll.register(autopilot_fd, select.POLLIN | select.POLLPRI)


        ################################### INITIAL IMU READINGS ###################################


        #-------------------------------------------------------------------------------------------
        # Flush the IMU FIFO and enable the FIFO overflow interrupt
        #-------------------------------------------------------------------------------------------
        GPIO.event_detected(GPIO_FIFO_OVERFLOW_INTERRUPT)
        mpu6050.flushFIFO()
        mpu6050.enableFIFOOverflowISR()

        temp = mpu6050.readTemperature()
        logger.critical("IMU core temp (start): ,%f", temp / 333.86 + 21.0)

        time.sleep(20 / sampling_rate)
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO(nfb)

        #-------------------------------------------------------------------------------------------
        # Feed back the gyro offset calibration
        #-------------------------------------------------------------------------------------------
        mpu6050.setGyroOffsets(qrx, qry, qrz)

        #-------------------------------------------------------------------------------------------
        # Read the IMU acceleration to obtain angles and gravity.
        #-------------------------------------------------------------------------------------------
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)

        #-------------------------------------------------------------------------------------------
        # Calculate the angles - ideally takeoff should be on a horizontal surface but a few degrees
        # here or there won't matter.
        #-------------------------------------------------------------------------------------------
        pa, ra = GetRotationAngles(qax, qay, qaz)
        ya = 0.0

        apa, ara = GetAbsoluteAngles(qax, qay, qaz)
        aya = 0.0

        papa = apa
        para = ara
        paya = aya

        #-------------------------------------------------------------------------------------------
        # Get the value for gravity.
        #-------------------------------------------------------------------------------------------
        egx, egy, egz = RotateVector(qax, qay, qaz, -pa, -ra, -ya)
        '''
        egx = 0.0
        egy = 0.0
        egz = math.pow(math.pow(qax, 2) + math.pow(qay, 2) + math.pow(qaz, 2), 0.5)
        '''

        #-------------------------------------------------------------------------------------------
        # The tilt ratio is used to compensate sensor height (and thus velocity) for the fact the
        # sensors are leaning.
        #
        # tilt ratio is derived from cos(tilt angle);
        # - tilt angle a = arctan(sqrt(x*x + y*y) / z)
        # - cos(arctan(a)) = 1 / (sqrt(1 + a*a))
        # This all collapses down to the following.  0 <= Tilt ratio <= 1
        #-------------------------------------------------------------------------------------------
        tilt_ratio = qaz / egz
        eftoh *= tilt_ratio

        #-------------------------------------------------------------------------------------------
        # Log the critical parameters from this warm-up: the take-off surface tilt, and gravity.
        # Note that some of the variables used above are used in the main processing loop.  Messing
        # with the above code can have very unexpected effects in flight.
        #-------------------------------------------------------------------------------------------
        logger.warning("pitch, %f, roll, %f", math.degrees(pa), math.degrees(ra))
        logger.warning("egx, %f, egy, %f, egz %f", egx, egy, egz)
        logger.warning("based upon %d samples", dt * sampling_rate)
        logger.warning("EFTOH:, %f", eftoh)


        #-------------------------------------------------------------------------------------------
        # Prime the direction vector of the earth's magnetic core to provide long term yaw stability.
        #-------------------------------------------------------------------------------------------
        mgx = 0.0
        mgy = 0.0
        mgz = 0.0

        if self.compass_installed:
            mgx, mgy, mgz = mpu6050.readCompass()

            #-----------------------------------------------------------------------------------
            # Rotate compass readings back to earth plan.  Note the compass coodintates are N,E
            # positive, which is different to that from the IMU.  Overide the init yaw angle PID
            # input and target with the new compass orientation values.
            #-----------------------------------------------------------------------------------
            cax, cay, caz = RotateVector(mgx, mgy, mgz, -pa, -ra, 0)
            cya_base = math.atan2(cax, cay)
            logger.critical("Initial orientation:, %f." % (cya_base * 180 / math.pi))


        ######################################### GO GO GO! ########################################


        #-------------------------------------------------------------------------------------------
        # Set up the various timing constants and stats.
        #-------------------------------------------------------------------------------------------
        motion_dt = 0.0
        fusion_dt = 0.0
        sampling_loops = 0
        motion_loops = 0
        fusion_loops = 0
        garmin_loops = 0
        video_loops = 0
        gps_loops = 0
        autopilot_loops = 0

        #-------------------------------------------------------------------------------------------
        # Diagnostic log header
        #-------------------------------------------------------------------------------------------
        if diagnostics:
            logger.warning("time, dt, loops, " +
                           "temperature, " +
                           "latitude, longitude, altitude, satellites, north, east, height, " +
                           "mgx, mgy, mgz, cya, " +
#                           "sweep proximity, sweep direction, " +
                           "qdx_fuse, qdy_fuze, qdz_fuse, " +
                           "qvx_fuse, qvy_fuse, qvz_fuse, " +
                           "edx_target, edy_target, edz_target, " +
                           "evx_target, evy_target, evz_target, " +
                           "qrx, qry, qrz, " +
                           "qax, qay, qaz, " +
                           "qgx, qgy, qgz, " +
                           "pitch, roll, yaw" +
#                            "qdx_input, qdx_target, qvx_input, qvx_target, pa_input, pa_target, pr_input, pr_target, pr_out, " +
#                            "qdy_input, qdy_target, qvy_input, qvy_target, ra_input, ra_target, rr_input, rr_target, rr_out, " +
#                            "qdz_input, qdz_target, qvz_input, qvz_target, qvz_out, " +
#                            "ya_input, ya_target, yr_input, yr_target, yr_out, " +
                           "FL PWM, FR PWM, BL PWM, BR PWM")

        #===========================================================================================
        #
        # Motion and PID processing loop naming conventions
        #
        # qa? = quad frame acceleration
        # qg? = quad frame gravity
        # qr? = quad motion rotation
        # ea? = earth frame acceleration
        # eg? = earth frame gravity
        # ua? = euler angles between reference frames
        # ur? = euler rotation between frames
        #
        #===========================================================================================
        while self.keep_looping:


            ############################### SENSOR INPUT SCHEDULING ################################


            #---------------------------------------------------------------------------------------
            # Check on the number of IMU batches already stashed in the FIFO, and if not enough,
            # check autopilot, GPS and video, and ultimate sleep.
            #---------------------------------------------------------------------------------------
            nfb = mpu6050.numFIFOBatches()

            if nfb >= self.FIFO_OVERFLOW:
                logger.critical("ABORT: FIFO too full risking overflow: %d.", nfb)
                if vmp != None:
                    logger.critical("       Next VFP phase: %d", vmp.phase)
                break

            if nfb < self.FIFO_PRIORITY:

                #-----------------------------------------------------------------------------------
                # We have some spare time before we need to run the next motion processing; see if
                # there's any processing we can do.  First, have we already got a video frame we can
                # continue processing?
                #-----------------------------------------------------------------------------------
                if vmp != None:
                    result = vmp.process()
                    if result != None:
                        vvx, vvy = result
                        vvx *= scale
                        vvy *= scale
                        camera_data_update = True
                        vmp = None
                    continue

                #-----------------------------------------------------------------------------------
                # Nowt else to do; sleep until either we timeout, or more data comes in from GPS
                # video, or autopilot.
                #-----------------------------------------------------------------------------------
                timeout = (self.FIFO_PRIORITY - nfb) / sampling_rate
                try:
                    results = poll.poll(timeout * 1000)
                except:
                    logger.critical("ERROR: poll error")
                    break

                #===================================================================================
                # Priority puts new video processing at the end because once running, everything
                # else is not services until it's finished.  Video, GPS and Sweep are low
                # frequency, low overhead.  Breaking from this for loop means the else at the end of
                # the for loop is not called, and hence all_ok remains set to False.
                #===================================================================================
                all_ok = False
                for fd, event in results:
                    if fd == autopilot_fd:
                        #---------------------------------------------------------------------------
                        # Run the Autopilot Processor to get the latest stage of the flight plan.
                        #---------------------------------------------------------------------------
                        autopilot_loops += 1
                        evx_target, evy_target, evz_target, state_change, state_name, self.keep_looping = app.read()
                        if state_change:
                            logger.critical(state_name)


                    if self.gps_installed and fd == gps_fd:
                        #---------------------------------------------------------------------------
                        # Run the GPS Processor, and convert response to X, Y coordinate in earth NSEW
                        # frame.
                        #---------------------------------------------------------------------------
                        gps_loops += 1
                        try:
                            gps_lat, gps_lon, gps_alt, gps_sats = gpsp.read()

                            #---------------------------------------------------------------------------
                            # Latitude = North (+) / South (-) - 0.0 running E/W around the equator;
                            #            range is +/- 90 degrees
                            # Longitude = East (+) / West (-) - 0.0 running N/S through Greenwich;
                            #             range is +/- 180 degrees
                            #
                            # With a base level longitude and latitude in degrees, we can calculate the
                            # current X and Y coordinates in meters using equirectangular approximation:
                            #
                            # ns = movement North / South - movement in a northerly direction is positive
                            # ew = movement East / West - movement in an easterly direction is positive
                            # R = average radius of earth in meters = 6,371,000 meters
                            #
                            # ns = (long2 - long1) * cos ((lat1 + lat2) / 2) * R meters
                            # ew = (lat2 - lat1) * R meters
                            #
                            # Note longitude / latitude are in degrees and need to be converted into
                            # radians i.e degrees * pi / 180 both for the cos and also the EARTH_RADIUS scale
                            #
                            # More at http://www.movable-type.co.uk/scripts/latlong.html
                            #
                            #---------------------------------------------------------------------------
                            gps_ns = (gps_lon - base_lon) * math.cos((gps_lat + base_lat) * math.pi / 360) * EARTH_RADIUS * math.pi / 180
                            gps_ew = (gps_lat - base_lat) * EARTH_RADIUS * math.pi / 180
                            gps_dalt = (gps_alt - base_alt)

                        except AssertionError as e:
                            print "FMR GPS"
                            break

                    if self.camera_installed and fd == motion_fd:
                        #---------------------------------------------------------------------------
                        # Run the Video Motion Processor
                        #---------------------------------------------------------------------------
                        '''
                        #AB! BUG? If aya and paya are either side of zero as aya is cropped lower down,
                        #AB! so paya needs to be so also.  Or do we just get away with it due to how
                        #AB! sin and cos works?  Guessing we do.
                        #AB! apa and ara aren't a problem because they are worked out from the accelerometer
                        #AB! rather than integrated yaw rate + initial compass orientation.
                        '''

                        #---------------------------------------------------------------------------
                        # Not quite sure how this happens, but it does, so protect against it
                        #---------------------------------------------------------------------------
                        if vmpt == pvmpt:
                            all_ok = True
                            break

                        vmp_dt = vmpt - pvmpt
                        apa_increment = apa - papa
                        ara_increment = ara - para
                        aya_increment = aya - paya
                        papa = apa
                        para = ara
                        paya = aya
                        pvmpt = vmpt
                        video_loops += 1

                        try:
                            vmp = VideoMotionProcessor(motion_fifo, aya_increment)
                            vmp.process()
                        except ValueError as e:
                            #-----------------------------------------------------------------------
                            # First pass of the video frame shows no movement detected, and thus no
                            # further processing.
                            #-----------------------------------------------------------------------
                            vmp = None

                else:
                    all_ok = True

                if not all_ok:
                    break

                #-----------------------------------------------------------------------------------
                # We had free time, do we still?  Better check.
                #-----------------------------------------------------------------------------------
                continue


            ####################################### IMU FIFO #######################################


            #---------------------------------------------------------------------------------------
            # Before proceeding further, check the FIFO overflow interrupt to ensure we didn't sleep
            # too long
            #---------------------------------------------------------------------------------------
            if GPIO.event_detected(GPIO_FIFO_OVERFLOW_INTERRUPT):
                logger.critical("ABORT: FIFO overflow.")
                break

            #---------------------------------------------------------------------------------------
            # Power brownout check
            #---------------------------------------------------------------------------------------
            '''
            #AB! if GPIO.event_detected(GPIO_POWER_BROWN_OUT_INTERRUPT):
            #AB!     logger.critical("BROWN-OUT, ABORT!")
            #AB!     break
            '''

            #---------------------------------------------------------------------------------------
            # Now get the batch of averaged data from the FIFO.
            #---------------------------------------------------------------------------------------
            try:
                qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
            except IOError as err:
                logger.critical("ABORT:")
                for arg in err.args:
                    logger.critical("%s", arg)
                break

            #---------------------------------------------------------------------------------------
            # Sort out units and calibration for the incoming data
            #---------------------------------------------------------------------------------------
            qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax,
                                                                qay,
                                                                qaz,
                                                                qrx,
                                                                qry,
                                                                qrz)

            #---------------------------------------------------------------------------------------
            # Track the number of motion loops and sampling loops.  motion_dt on which their are based
            # are the core timing provided by the IMU and are used for all timing events later such
            # as integration and PID Intergral and Differential factors.
            #---------------------------------------------------------------------------------------
            motion_loops += 1
            sampling_loops += motion_dt * sampling_rate
            fusion_dt += motion_dt
            vmpt += motion_dt


            ################################## ANGLES PROCESSING ###################################


            #---------------------------------------------------------------------------------------
            # Euler angle fusion: Merge the 'integral' of the previous euler rotation rates with
            # the noisy accelermeter current values.  Keep yaw within +/- pi radians
            #---------------------------------------------------------------------------------------
            urp, urr, ury = Body2EulerRates(qry, qrx, qrz, pa, ra)
            pa += urp * motion_dt
            ra += urr * motion_dt
            ya += ury * motion_dt
            ya = (ya + math.pi) % (2 * math.pi) - math.pi

            upa, ura = GetRotationAngles(qax, qay, qaz)

            atau_fraction = atau / (atau + motion_dt)
            pa = atau_fraction * pa + (1 - atau_fraction) * upa
            ra = atau_fraction * ra + (1 - atau_fraction) * ura

            #---------------------------------------------------------------------------------------
            # Absolute angle fusion: Merge the 'integral' of the gyro rotation rates with
            # the noisy accelermeter current values.  Keep yaw within +/- pi radians
            #---------------------------------------------------------------------------------------
            apa += qry * motion_dt
            ara += qrx * motion_dt
            aya += qrz * motion_dt
            aya = (aya + math.pi) % (2 * math.pi) - math.pi

            upa, ura = GetAbsoluteAngles(qax, qay, qaz)

            atau_fraction = atau / (atau + motion_dt)
            apa = atau_fraction * apa + (1 - atau_fraction) * upa
            ara = atau_fraction * ara + (1 - atau_fraction) * ura

            '''
            #---------------------------------------------------------------------------------------
            # The temperature drifts throughout the flight, and if below about 20 degrees, this drift
            # causes major problems with double integration of accelerometer readings vs gravity.
            # However, because the flights are mostly fixed velocity = zero acceleration, a simple
            # complementary filter can keep gravity in sync with this drift.
            #---------------------------------------------------------------------------------------
            eax, eay, eaz = RotateVector(qax, qay, qaz, -pa, -ra, -ya)
            gtau = 10 # seconds
            gtau_fraction = gtau / (gtau + motion_dt)
            egx = atau_fraction * egx + (1 - atau_fraction) * eax
            egy = atau_fraction * egy + (1 - atau_fraction) * eay
            egz = atau_fraction * egz + (1 - atau_fraction) * eaz
            '''

            ############################### IMU VELOCITY / DISTANCE ################################


            #---------------------------------------------------------------------------------------
            # Rotate gravity to the new quadframe
            #---------------------------------------------------------------------------------------
            qgx, qgy, qgz = RotateVector(egx, egy, egz, pa, ra, ya)

            #---------------------------------------------------------------------------------------
            # The tilt ratio is the ratio of gravity measured in the quad-frame Z axis and total gravity.
            # It's used to compensate for LiDAR height (and thus velocity) for the fact the laser may
            # not be pointing directly vertically down.
            #
            # - tilt angle a = arctan(sqrt(x*x + y*y) / z)
            # - compensated height = cos(arctan(a)) = 1 / (sqrt(1 + a*a))
            #
            # http://www.rapidtables.com/math/trigonometry/arctan/cos-of-arctan.htm
            #
            #---------------------------------------------------------------------------------------
            tilt_ratio = qgz / egz

            # =================== Velocity and Distance Increment processing =======================

            #---------------------------------------------------------------------------------------
            # Delete reorientated gravity from raw accelerometer readings and integrate over time
            # to make velocity all in quad frame.
            #---------------------------------------------------------------------------------------
            qvx_increment = (qax - qgx) * GRAV_ACCEL * motion_dt
            qvy_increment = (qay - qgy) * GRAV_ACCEL * motion_dt
            qvz_increment = (qaz - qgz) * GRAV_ACCEL * motion_dt

            qvx_input += qvx_increment
            qvy_input += qvy_increment
            qvz_input += qvz_increment

            #---------------------------------------------------------------------------------------
            # Integrate again the velocities to get distance.
            #---------------------------------------------------------------------------------------
            qdx_increment = qvx_input * motion_dt
            qdy_increment = qvy_input * motion_dt
            qdz_increment = qvz_input * motion_dt

            qdx_input += qdx_increment
            qdy_input += qdy_increment
            qdz_input += qdz_increment


            ######################## ABSOLUTE DISTANCE / ORIENTATION SENSORS #######################


            #---------------------------------------------------------------------------------------
            # Read the compass to determine yaw and orientation.
            #---------------------------------------------------------------------------------------
            if self.compass_installed:
                mgx, mgy, mgz = mpu6050.readCompass()

                #-----------------------------------------------------------------------------------
                # Rotate compass readings back to earth plan and convert to quad coordinates.  Note
                # minus sign for cya means rotation direction is now quad yaw not earth NSEW and
                # can be fused with gyro yaw
                #-----------------------------------------------------------------------------------
                cax, cay, caz = RotateVector(mgx, mgy, mgz, -pa, -ra, 0)
                cya = math.atan2(cax, cay) - cya_base
                cya = -((cya + math.pi) % (2 * math.pi) - math.pi)

                '''
                #AB! Fuse aya gyro increment and compase absolute
                '''


            #=======================================================================================
            # Acquire vertical distance (height) first, prioritizing the best sensors,
            # Garmin LiDAR-Lite first.  We need get this every motion processing loop so it's always
            # up to date at the point we use it for camera lateral tracking.
            #=======================================================================================
            if self.gll_installed:
                garmin_loops += 1
                try:
                    g_distance, g_velocity = gll.read()
                except ValueError as e:
                    print "GLL status 2: %s" % e
                    break

                edz_fuse = g_distance * tilt_ratio - eftoh
                evz_fuse = g_velocity * tilt_ratio

                #-----------------------------------------------------------------------------------
                # Set the flags for vertical velocity and distance fusion
                #-----------------------------------------------------------------------------------
                vvf = True
                vdf = True

            #=======================================================================================
            # Acquire horizontal distance next, again with prioritization of accuracy
            #=======================================================================================

            #---------------------------------------------------------------------------------------
            # If the camera is installed, and we have an absolute height measurement, get the horizontal
            # distance and velocity.  Note we always have height measurement, even if only from the
            # double integrated accelerometer.
            #---------------------------------------------------------------------------------------
            if self.camera_installed and self.gll_installed and camera_data_update:

                #-----------------------------------------------------------------------------------
                # Take the increment of the scaled X and Y distance, and muliply by the height to
                # get the absolute position, allowing for tilt increment.
                #-----------------------------------------------------------------------------------
                '''
                #AB! Don't like the nomenclature of ed?_increment and vv?: ed*_increment should be
                #AB! qd*_increment but that exists.  vv* = video 'velocity' but it's an increment
                #AB! between video frames, in kinda pixel units until height is know.
                '''

                edx_increment = (g_distance * tilt_ratio) * (vvx + apa_increment / (2 * math.pi))
                edy_increment = (g_distance * tilt_ratio) * (vvy - ara_increment / (2 * math.pi))

                #-----------------------------------------------------------------------------------
                # Add the incremental distance to the total distance, and differentiate against time for
                # velocity.  The camera output is already in the quad frame, so all e??_increment need
                # renaming.
                #-----------------------------------------------------------------------------------
                qdx_fuse += edx_increment
                qdy_fuse += edy_increment

                qvx_fuse = edx_increment / vmp_dt
                qvy_fuse = edy_increment / vmp_dt

                #-----------------------------------------------------------------------------------
                # Set the flags for horizontal distance and velocity fusion
                #-----------------------------------------------------------------------------------
                hdf = True
                hvf = True

                camera_data_update = False


            ######################################## FUSION ########################################


            #---------------------------------------------------------------------------------------
            # If we have new vertical data, fuse it.
            #---------------------------------------------------------------------------------------
            if vvf and vdf:
                #-----------------------------------------------------------------------------------
                # We need to rotate e*z_fuse into the quad frame.  First get best estimates of e?x +
                # e?y by rotations of the q?x / q?y values back to the earth frame.  Then use the new
                # Z value as part of the rerotation back to quad frame.
                #-----------------------------------------------------------------------------------
                edx_fuse, edy_fuse, __ = RotateVector(qdx_input, qdy_input, qdz_input, -pa, -ra, -ya)
                evx_fuse, evy_fuse, __ = RotateVector(qvx_input, qvy_input, qvz_input, -pa, -ra, -ya)

                __, __, qdz_fuse = RotateVector(edx_fuse, edy_fuse, edz_fuse, pa, ra, ya)
                __, __, qvz_fuse = RotateVector(evx_fuse, evy_fuse, evz_fuse, pa, ra, ya)

                #-----------------------------------------------------------------------------------
                # Now fuse with complementary filters.
                #-----------------------------------------------------------------------------------
                '''
                #AB! Note that because we always get an update from the GLL each motion cycle, we can
                #AB! us the motion_dt.  If we ever get the GLL interrupt to work, this needs changing
                #AB! to an equivalent of the fusion_dt used in the horizontal fusion lower down.
                '''
                fusion_fraction = fusion_tau / (fusion_tau + motion_dt)

                qvz_input = fusion_fraction * qvz_input + (1 - fusion_fraction) * qvz_fuse
                qdz_input = fusion_fraction * qdz_input + (1 - fusion_fraction) * qdz_fuse

                #-----------------------------------------------------------------------------------
                # Clear the flags for vertical distance and velocity fusion
                #-----------------------------------------------------------------------------------
                vdf = False
                vvf = False

            #---------------------------------------------------------------------------------------
            # If we have new horizontal data, fuse it.
            #---------------------------------------------------------------------------------------
            if hdf and hvf:
                #-----------------------------------------------------------------------------------
                # Now fuse with complementary filters
                #-----------------------------------------------------------------------------------
                fusion_fraction = fusion_tau / (fusion_tau + fusion_dt)

                qvx_input = fusion_fraction * qvx_input + (1 - fusion_fraction) * qvx_fuse
                qvy_input = fusion_fraction * qvy_input + (1 - fusion_fraction) * qvy_fuse

                qdx_input = fusion_fraction * qdx_input + (1 - fusion_fraction) * qdx_fuse
                qdy_input = fusion_fraction * qdy_input + (1 - fusion_fraction) * qdy_fuse

                fusion_loops += 1
                fusion_dt = 0.0

                #-----------------------------------------------------------------------------------
                # Clear the flags for horizontal distance and velocity fusion
                #-----------------------------------------------------------------------------------
                hdf = False
                hvf = False


            ########################### VELOCITY / DISTANCE PID TARGETS ############################


            if edz_fuse > edz_target + 0.5:
                logger.critical("ABORT: Height breach! %f target, %f actual", edz_target, edz_fuse)
                break

            #---------------------------------------------------------------------------------------
            # Convert earth-frame distance targets to quadcopter frame.
            #---------------------------------------------------------------------------------------
            edx_target += evx_target * motion_dt
            edy_target += evy_target * motion_dt
            edz_target += evz_target * motion_dt
            qdx_target, qdy_target, qdz_target = RotateVector(edx_target, edy_target, edz_target, pa, ra, ya)

            #---------------------------------------------------------------------------------------
            # If we're doing yaw control, then from the iDrone POV, she's always flying forward, and
            # it's the responsibility of the yaw target to keep her facing in the right direction.
            #---------------------------------------------------------------------------------------
            if yaw_control:
                qdx_target = math.pow(math.pow(qdx_target, 2) + math.pow(qdy_target, 2), 0.5)
                qdy_target = 0.0


            ########### QUAD FRAME VELOCITY / DISTANCE / ANGLE / ROTATION PID PROCESSING ###########


            #=======================================================================================
            # Distance PIDs
            #=======================================================================================
            [p_out, i_out, d_out] = qdx_pid.Compute(qdx_input, qdx_target, motion_dt)
            qvx_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = qdy_pid.Compute(qdy_input, qdy_target, motion_dt)
            qvy_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = qdz_pid.Compute(qdz_input, qdz_target, motion_dt)
            qvz_target = p_out + i_out + d_out

            '''
            '''
            #---------------------------------------------------------------------------------------
            # For control reasons, limit the maximum target speed to 1m/s
            #---------------------------------------------------------------------------------------
            MAX_VEL = 1.0
            qvx_target = qvx_target if abs(qvx_target) < MAX_VEL else (qvx_target / abs(qvx_target) * MAX_VEL)
            qvy_target = qvy_target if abs(qvy_target) < MAX_VEL else (qvy_target / abs(qvy_target) * MAX_VEL)
            qvz_target = qvz_target if abs(qvz_target) < MAX_VEL else (qvz_target / abs(qvz_target) * MAX_VEL)
            '''
            '''

            #=======================================================================================
            # Velocity PIDs
            #=======================================================================================
            [p_out, i_out, d_out] = qvx_pid.Compute(qvx_input, qvx_target, motion_dt)
            qax_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = qvy_pid.Compute(qvy_input, qvy_target, motion_dt)
            qay_target =  p_out + i_out + d_out

            [p_out, i_out, d_out] = qvz_pid.Compute(qvz_input, qvz_target, motion_dt)
            qaz_out = p_out + i_out + d_out

            #---------------------------------------------------------------------------------------
            # We now need to convert desired acceleration to desired angles before running the angular PIDs
            #
            # Convert the horizontal velocity PID output i.e. the horizontal acceleration target in
            # m/s/s into the pitch and roll angle PID targets in radians
            # - A forward unintentional drift is a positive input and negative output from the
            #   velocity PID.  This represents corrective acceleration.  To achieve corrective
            #   backward acceleration, the negative velocity PID output needs to trigger a negative
            #   pitch angles target.
            # - A left unintentional drift is a positive input and negative output from the velocity
            #   PID.  To achieve corrective right acceleration, the negative velocity PID output needs
            #   to trigger a positive roll angle target.
            # Use a bit of hokey trigonometry to convert desired quad frame acceleration (qv*_out)
            # into the target quad frame angles that provides that acceleration (*a_target)
            #
            # The yaw angle target is set such that she's facing the way she should be travelling
            # based upon the earth frame velocity targets.  If these targets are zero, then no yaw happens.
            # Note this must use atan2 to safely handle division by 0.
            #---------------------------------------------------------------------------------------
            pa_target = math.atan(qax_target)
            ra_target = -math.atan(qay_target)
            ya_target = ya_target if not yaw_control else (ya_target if (abs(evx_target) + abs(evy_target)) == 0 else math.atan2(evy_target, evx_target))

            #---------------------------------------------------------------------------------------
            # For safety reasons, limit the maximum target angle to 30 degrees.  Note yaw is deliberately
            # not included as it needs full rotation to track the direction of flight.
            #---------------------------------------------------------------------------------------
            MAX_ANGLE = 30 * math.pi / 180
            pa_target = pa_target if abs(pa_target) < MAX_ANGLE else (pa_target / abs(pa_target) * MAX_ANGLE)
            ra_target = ra_target if abs(ra_target) < MAX_ANGLE else (ra_target / abs(ra_target) * MAX_ANGLE)

            #======================================================================================
            # Angle PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pa_pid.Compute(apa, pa_target, motion_dt)
            pr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ra_pid.Compute(ara, ra_target, motion_dt)
            rr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ya_pid.Compute(aya, ya_target, motion_dt)
            yr_target = p_out + i_out + d_out

            #---------------------------------------------------------------------------------------
            # For safety reasons, limit the maximum target rotation rate to 180 degrees / second.
            # Additionally, if we're under yaw control, and there has been a significant course change
            # more than 10 degrees, then limit it to 30 degrees / second.
            #---------------------------------------------------------------------------------------
            MAX_RATE = math.pi
            if yaw_control and abs(ya_target - aya) > math.pi / 9:
                YAW_MAX_RATE = math.pi / 3
            else:     
                YAW_MAX_RATE = MAX_RATE 

            pr_target = pr_target if abs(pr_target) < MAX_RATE else (pr_target / abs(pr_target) * MAX_RATE)
            rr_target = rr_target if abs(rr_target) < MAX_RATE else (rr_target / abs(rr_target) * MAX_RATE)
            yr_target = yr_target if abs(yr_target) < YAW_MAX_RATE else (yr_target / abs(yr_target) * YAW_MAX_RATE)

            #=======================================================================================
            # Rotation rate PIDs
            #=======================================================================================

            #---------------------------------------------------------------------------------------
            # START TESTCASE 2 CODE: Override motion processing results; take-off from horizontal
            #                        platform, tune the pr*_gain and rr*_gain PID gains for
            #                        stability.
            #---------------------------------------------------------------------------------------
            if test_case == 2:
                pr_target = 0.0
                rr_target = 0.0
                yr_target = 0.0
            #---------------------------------------------------------------------------------------
            # END TESTCASE 2 CODE: Override motion processing results; take-off from horizontal
            #                      platform, tune the pr*_gain and rr*_gain PID gains for
            #                      stability.
            #---------------------------------------------------------------------------------------

            [p_out, i_out, d_out] = pr_pid.Compute(qry, pr_target, motion_dt)
            pr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = rr_pid.Compute(qrx, rr_target, motion_dt)
            rr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = yr_pid.Compute(qrz, yr_target, motion_dt)
            yr_out = p_out + i_out + d_out


            ################################## PID OUTPUT -> PWM CONVERTION ########################


            #---------------------------------------------------------------------------------------
            # Convert the vertical velocity PID output direct to ESC input PWM pulse width.
            #---------------------------------------------------------------------------------------
            vert_out = hover_pwm + int(round(qaz_out))

            #---------------------------------------------------------------------------------------
            # Convert the rotation rate PID outputs direct to ESC input PWM pulse width
            #---------------------------------------------------------------------------------------
            pr_out = int(round(pr_out / 2))
            rr_out = int(round(rr_out / 2))
            yr_out = int(round(yr_out / 2))


            #=======================================================================================
            # PID output distribution: Walk through the ESCs, and apply the PID outputs i.e. the
            # updates PWM pulse widths according to where the ESC is sited on the frame
            #=======================================================================================
            for esc in self.esc_list:

                #-----------------------------------------------------------------------------------
                # Update all blades' power in accordance with the z error
                #-----------------------------------------------------------------------------------
                pulse_width = vert_out

                #-----------------------------------------------------------------------------------
                # For a left downwards roll, the x gyro goes negative, so the PID error is positive,
                # meaning PID output is positive, meaning this needs to be added to the left blades
                # and subtracted from the right.
                #-----------------------------------------------------------------------------------
                if esc.motor_location & self.MOTOR_LOCATION_RIGHT:
                    pulse_width -= rr_out
                else:
                    pulse_width += rr_out

                #-----------------------------------------------------------------------------------
                # For a forward downwards pitch, the y gyro goes positive The PID error is negative as a
                # result, meaning PID output is negative, meaning this needs to be subtracted from the
                # front blades and added to the back.
                #-----------------------------------------------------------------------------------
                if esc.motor_location & self.MOTOR_LOCATION_BACK:
                    pulse_width += pr_out
                else:
                    pulse_width -= pr_out

                #-----------------------------------------------------------------------------------
                # For CW yaw, the z gyro goes negative, so the PID error is postitive, meaning PID
                # output is positive, meaning this need to be added to the ACW (FL and BR) blades and
                # subtracted from the CW (FR & BL) blades.
                #-----------------------------------------------------------------------------------
                if esc.motor_rotation == self.MOTOR_ROTATION_CW:
                    pulse_width += yr_out
                else:
                    pulse_width -= yr_out

                #-----------------------------------------------------------------------------------
                # Apply the blended outputs to the esc PWM signal
                #-----------------------------------------------------------------------------------
                esc.set(pulse_width)

            #---------------------------------------------------------------------------------------
            # Diagnostic log - every motion loop
            #---------------------------------------------------------------------------------------
            if diagnostics:
                temp = mpu6050.readTemperature()
                logger.warning("%f, %f, %d, " % (sampling_loops / sampling_rate, motion_dt, sampling_loops) +
                               "%f, " % (temp / 333.86 + 21) +
                               "%f, %f, %f, %d, %f, %f, %f, " % (gps_lat, gps_lon, gps_alt, gps_sats, gps_ns, gps_ew, gps_dalt) +
                               "%f, %f, %f, %f, " % (mgx, mgy, mgz, cya * 180 / math.pi) +
#                               "%f, %f, " % (sweep_distance, sweep_direction * 180 / math.pi) +
                               "%f, %f, %f, " % (qdx_fuse, qdy_fuse, qdz_fuse) +
                               "%f, %f, %f, " % (qvx_fuse, qvy_fuse, qvz_fuse) +
                               "%f, %f, %f, " % (edx_target, edy_target, edz_target) +
                               "%f, %f, %f, " % (evx_target, evy_target, evz_target) +
                               "%f, %f, %f, " % (qrx, qry, qrz) +
                               "%f, %f, %f, " % (qax, qay, qaz) +
                               "%f, %f, %f, " % (qgx, qgy, qgz) +
                               "%f, %f, %f, " % (math.degrees(pa), math.degrees(ra), math.degrees(ya)) +
#                               "%f, %f, %f, %f, %f, %f, %f, %f, %d, " % (qdx_input, qdx_target, qvx_input, qvx_target, math.degrees(apa), math.degrees(pa_target), math.degrees(qry), math.degrees(pr_target), pr_out) +
#                               "%f, %f, %f, %f, %f, %f, %f, %f, %d, " % (qdy_input, qdy_target, qvy_input, qvy_target, math.degrees(ara), math.degrees(ra_target), math.degrees(qrx), math.degrees(rr_target), rr_out) +
#                               "%f, %f, %f, %f, %d, " % (qdz_input, qdz_target, qvz_input, qvz_target, qaz_out) +
#                               "%f, %f, %f, %f, %d, " % (math.degrees(aya), math.degrees(ya_target), math.degrees(qrz), math.degrees(yr_target), yr_out) +
                               "%d, %d, %d, %d, " % (self.esc_list[0].pulse_width, self.esc_list[1].pulse_width, self.esc_list[2].pulse_width, self.esc_list[3].pulse_width))

        logger.critical("Flight time %f", time.time() - start_flight)
        logger.critical("Sampling loops: %d", sampling_loops)
        logger.critical("Motion processing loops: %d", motion_loops)
        logger.critical("Fusion processing loops: %d", fusion_loops)
        logger.critical("LiDAR processing loops: %d", garmin_loops)
        logger.critical("GPS processing loops: %d", gps_loops)
        logger.critical("Autopilot processing loops: %d.", autopilot_loops)
        if sampling_loops != 0:
            logger.critical("Video frame rate: %f", video_loops * sampling_rate / sampling_loops )

        temp = mpu6050.readTemperature()
        logger.critical("IMU core temp (end): ,%f", temp / 333.86 + 21.0)
        max_az, min_az = mpu6050.getStats()
        logger.critical("Min Z acceleration: %f", min_az)
        logger.critical("Max Z acceleration: %f", max_az)

        #-------------------------------------------------------------------------------------------
        # Stop the PWM and FIFO overflow interrupt between flights
        #-------------------------------------------------------------------------------------------
        for esc in self.esc_list:
            esc.set(0)
        mpu6050.disableFIFOOverflowISR()

        #-------------------------------------------------------------------------------------------
        # Unregister poll registrars
        #-------------------------------------------------------------------------------------------
        poll.unregister(autopilot_fd)
        if self.gps_installed:
            poll.unregister(gps_fd)
        if self.camera_installed:
            poll.unregister(motion_fd)

        #-------------------------------------------------------------------------------------------
        # Stop the camera motion processing
        #-------------------------------------------------------------------------------------------
        if self.camera_installed:
            print "Stopping video... ",
            video_process.send_signal(signal.SIGINT)
            '''
            #AB! video_process.wait() NEED TO TIDY THIS UP - COMMUNICATE OR SPEED UP VIDEO CYCLE?
            '''
            motion_fifo.close()
            os.unlink("/dev/shm/motion_stream")
            print "stopped."

        #-------------------------------------------------------------------------------------------
        # Stop the autopilot
        #-------------------------------------------------------------------------------------------
        print "Stopping autopilot... ",
        app.cleanup()
        print "stopped."


    ################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ################################################################################################
    def shutdown(self):

        #-------------------------------------------------------------------------------------------
        # Stop the child GPS process
        #-------------------------------------------------------------------------------------------
        if self.gps_installed:
            print "Stopping GPS... ",
            gpsp.cleanup()
            print "stopped."

        #-------------------------------------------------------------------------------------------
        # Stop the signal handler
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        #-------------------------------------------------------------------------------------------
        # Stop the blades spinning
        #-------------------------------------------------------------------------------------------
        for esc in self.esc_list:
            esc.set(stfu_pwm)

        #-------------------------------------------------------------------------------------------
        # Copy logs from /dev/shm (shared / virtual memory) to disk.
        #-------------------------------------------------------------------------------------------
        file_handler.close()

        #-------------------------------------------------------------------------------------------
        # Unlock memory we've used from RAM
        #-------------------------------------------------------------------------------------------
        munlockall()

        #-------------------------------------------------------------------------------------------
        # Clean up PWM / GPIO, but pause beforehand to give the ESCs time to stop properly
        #-------------------------------------------------------------------------------------------
        time.sleep(1.0)
        PWMTerm()

        #-------------------------------------------------------------------------------------------
        # Clean up the GPIO FIFO Overflow ISR
        #-------------------------------------------------------------------------------------------
        GPIOTerm()

        #-------------------------------------------------------------------------------------------
        # Reset the signal handler to default
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        sys.exit(0)

    ####################################################################################################
    #
    # Signal handler for Ctrl-C => abort cleanly; should really be just a "try: except KeyboardInterrupt:"
    #
    ####################################################################################################
    def shutdownSignalHandler(self, signal, frame):
        if not self.keep_looping:
            self.shutdown()
        self.keep_looping = False

    ####################################################################################################
    #
    # Interrupt Service Routine for FIFO overflow => abort flight cleanly - RETIRED, JUST POLL NOW
    #
    ####################################################################################################
    def fifoOverflowISR(self, pin):
        if self.keep_looping:
            print "FIFO OVERFLOW, ABORT"
            self.keep_looping = False

####################################################################################################
# If we've been called directly, this is the spawned video, GPS, Sweep or autopilot process or a
# misinformed user trying to start the code.
####################################################################################################
if __name__ == '__main__':
    if len(sys.argv) >= 2:

        #-------------------------------------------------------------------------------------------
        # Start the process recording video macro-blocks
        #-------------------------------------------------------------------------------------------
        if sys.argv[1] == "MOTION":
            assert (len(sys.argv) == 5), "Bad parameters for MOTION"
            frame_width = int(sys.argv[2])
            frame_height = int(sys.argv[3])
            frame_rate = int(sys.argv[4])
            RecordVideo(frame_width, frame_height, frame_rate)

        #-------------------------------------------------------------------------------------------
        # Start the process recording GPS
        #-------------------------------------------------------------------------------------------
        elif sys.argv[1] == "GPS":
            assert (len(sys.argv) == 2), "Bad parameters for GPS"
            RecordGPS()

        #-------------------------------------------------------------------------------------------
        # Start the process recording Sweep
        #-------------------------------------------------------------------------------------------
        elif sys.argv[1] == "SWEEP":
            assert (len(sys.argv) == 2), "Bad parameters for Sweep"
            RecordSweep()

        #-------------------------------------------------------------------------------------------
        # Start the process recording Autopilot
        #-------------------------------------------------------------------------------------------
        elif sys.argv[1] == "AUTOPILOT":
            assert (len(sys.argv) == 5), "Bad parameters for AUTOPILOT"
            flight_plan = sys.argv[2]
            motion_rate = int(sys.argv[3])
            sweep_installed = True if (sys.argv[4] == "True") else False
            RecordAutopilot(flight_plan, motion_rate, sweep_installed)
        else:
            assert (False), "Invalid process request."

    else:
        print "If you're trying to run me, use 'sudo python ./qc.py'"
