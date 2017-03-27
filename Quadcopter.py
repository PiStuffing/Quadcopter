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
#AB! import minimalmodbus # Only for LEDDAR
import picamera
import struct


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
        "Reads a a byte array value from the I2C device"
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
#  The compass / magnetometer of the MPU-9250 is not used
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
    __SCALE_ACCEL = 8.0 / 65536                                                           #AB! +/-4g

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address

        self.max_az = 0

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
        sample_rate_divisor = int(math.trunc(adc_frequency / sampling_rate))
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
        # int(math.log(g / 2, 2)) << 3
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x08)                             #AB! +/-4g
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set INT pin to push/pull, latch 'til read, any read to clear
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x30)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Initialize the FIFO overflow interrupt 0x10 (turned off at startup).                  #AB!
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
        logger.warning("IMU core temp: %f", temp / 333.86 + 21.0)

    def readTemperature(self):
        temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
        return temp

    def enableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Clear the interrupt status register and enable the FIFO overflow interrupt 0x10
        #AB! Something odd here: can't clear the GPIO pin if the ISR is enabled, and then status read
        #AB! in that order
        #-------------------------------------------------------------------------------------------
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

        ax /= fifo_batches
        ay /= fifo_batches
        az /= fifo_batches
        gx /= fifo_batches
        gy /= fifo_batches
        gz /= fifo_batches

        '''
        if az < (65536 / 8 * 0.8) or az > (65536 / 8 * 2.25):               #AB! +/-4g
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
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG , int_bypass | 0x02)

        #-------------------------------------------------------------------------------------------
        # Connect directly to the bypassed magnetometer, and configured it for 16 bit continuous data
        #-------------------------------------------------------------------------------------------
        self.i2c_compass = I2C(0x0C)
        self.i2c_compass.write8(self.__AK893_RA_CNTL1, 0x16);

    def readCompass(self):
        compass_bytes = self.i2c_compass.readList(self.__AK893_RA_X_LO, 7)

        #-------------------------------------------------------------------------------------------
        # Convert the array of 6 bytes to 3 shorts - 7th byte kicks off another read
        #-------------------------------------------------------------------------------------------
        compass_data = []
        for ii in range(0, 6, 2):
            lobyte = compass_bytes[ii]
            hibyte = compass_bytes[ii + 1]
            if (hibyte > 127):
                hibyte -= 256

            compass_data.append((hibyte << 8) + lobyte)

        [mgx, mgy, mgz] = compass_data

        mgx -= self.mgx_offset
        mgy -= self.mgy_offset
        mgz -= self.mgz_offset

        return mgx, mgy, mgz

    def calibrateCompass(self):
        self.mgx_offset = 0.0
        self.mgy_offset = 0.0
        self.mgz_offset = 0.0
        offs_rc = False

        #-------------------------------------------------------------------------------------------
        # First we need gyro offset calibration.  Flush the FIFO, collect roughly half a FIFO full
        # of samples and feed back to the gyro offset calibrations.
        #-------------------------------------------------------------------------------------------
        print "First, put me on a stable surface, and press my button."
        GPIO.wait_for_edge(GPIO_BUTTON, GPIO.FALLING)
        time.sleep(0.2)

        mpu6050.flushFIFO()
        time.sleep(20 / sampling_rate)
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO(nfb)
        mpu6050.setGyroOffsets(qrx, qry, qrz)
        print nfb

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
                # While integrated Z axis gyro < 2 pi i.e. 360 degrees, keep flashing the light
                #-----------------------------------------------------------------------------------
                try:
                    #-------------------------------------------------------------------------------
                    # Collect compass X. Y values
                    #-------------------------------------------------------------------------------
                    GPIO.output(GPIO_LED, GPIO.HIGH)
                    print "Now, pick me up and rotate me horizontally until the light stops flashing."
                    print "Press the button when you're ready to go."
                    GPIO.wait_for_edge(GPIO_BUTTON, GPIO.FALLING)

                    self.flushFIFO()

                    yaw = 0.0
                    total_dt = 0.0
                    while abs(yaw) < 2 * math.pi:
                        time.sleep(0.01)

                        nfb = mpu6050.numFIFOBatches()
                        ax, ay, az, gx, gy, gz, dt = self.readFIFO(nfb)
                        ax, ay, az, gx, gy, gz = self.scaleSensors(ax, ay, az, gx, gy, gz)
                        yaw += gz * dt
                        total_dt += dt

                        mgx, mgy, mgz = self.readCompass()

                        max_mgx = mgx if mgx > max_mgx else max_mgx
                        max_mgy = mgy if mgy > max_mgy else max_mgy
                        min_mgx = mgx if mgx < min_mgx else min_mgx
                        min_mgy = mgy if mgy < min_mgy else min_mgy

                        if total_dt > 0.2:
                            total_dt %= 0.2
                            print "YAW: %f" % (yaw * 180 / math.pi)
                            GPIO.output(GPIO_LED, not GPIO.input(GPIO_LED))

                    #-------------------------------------------------------------------------------
                    # Collect compass X. Z values
                    #-------------------------------------------------------------------------------
                    GPIO.output(GPIO_LED, GPIO.HIGH)
                    print "Great!  Now do the same but with my nose down."
                    print "Press the button when you're ready to go."
                    GPIO.wait_for_edge(GPIO_BUTTON, GPIO.FALLING)

                    self.flushFIFO()

                    pitch = 0.0
                    total_dt = 0.0
                    while abs(pitch) < 2 * math.pi:
                        time.sleep(0.01)

                        nfb = self.numFIFOBatches()
                        ax, ay, az, gx, gy, gz, dt = self.readFIFO(nfb)
                        ax, ay, az, gx, gy, gz = self.scaleSensors(ax, ay, az, gx, gy, gz)

                        pitch += gy * dt
                        total_dt += dt

                        mgx, mgy, mgz = self.readCompass()

                        max_mgz = mgz if mgz > max_mgz else max_mgz
                        min_mgz = mgz if mgz < min_mgz else min_mgz

                        if total_dt > 0.2:
                            total_dt %= 0.2
                            print "PITCH: %f" % (pitch * 180 / math.pi)
                            GPIO.output(GPIO_LED, not GPIO.input(GPIO_LED))

                except KeyboardInterrupt as e:
                    pass

                else:
                    #-------------------------------------------------------------------------------
                    # Write the good output to file.
                    #-------------------------------------------------------------------------------
                    mgx_offset = (max_mgx + min_mgx) / 2
                    mgy_offset = (max_mgy + min_mgy) / 2
                    mgz_offset = (max_mgz + min_mgz) / 2
                    offs_file.write("%f %f %f\n" % (float(mgx_offset), float(mgy_offset), float(mgz_offset)))
                    offs_rc = True

                finally:
                    #-------------------------------------------------------------------------------
                    # Turn the light off regardless of the result
                    #-------------------------------------------------------------------------------
                    GPIO.output(GPIO_LED, GPIO.LOW)

            GPIO.output(GPIO_LED, GPIO.HIGH)
            print "Looking go, just one last check to confirm all's well."
            print "Pop me on the ground again pointing north, based on another compass."
            print "Press the button when that's done, and I'll tell you which way I think I'm pointing"
            GPIO.wait_for_edge(GPIO_BUTTON, GPIO.FALLING)

            self.loadCompassCalibration()
            mgx, mgy, mgz = self.readCompass()
            print "%f, %f, %f" % (mgx, mgy, mgz)

            GPIO.output(GPIO_LED, GPIO.HIGH)
            print "All done - ready to go!"

        except EnvironmentError:
            pass

        return offs_rc

    def loadCompassCalibration(self):
        offs_rc = False
        try:
            with open('CompassOffsets', 'rb') as offs_file:
                for line in offs_file:
                    mgx_offset, mgy_offset, mgz_offset = line.split()
                self.mgx_offset = float(mgx_offset)
                self.mgy_offset = float(mgy_offset)
                self.mgz_offset = float(mgz_offset)

        except EnvironmentError:
            #---------------------------------------------------------------------------------------
            # Compass calibration is essential to exclude soft magnetic fields such as from local
            # metal; enfore a recalibration if not found.
            #---------------------------------------------------------------------------------------
            # print "Oops, something went wrong reading the compass offsets file 'CompassOffsets'"
            pass
        else:
            #---------------------------------------------------------------------------------------
            # Once the testing is complete, the finally code moves to here.
            #---------------------------------------------------------------------------------------
            pass
        finally:
            #---------------------------------------------------------------------------------------
            # Currently just fill in zero offsets until the whole  process has been tested
            #---------------------------------------------------------------------------------------
            self.mgx_offset = 0.0
            self.mgy_offset = 0.0
            self.mgz_offset = 0.0
            offs_rc = True

        logger.warning("Compass Offsets:, %f, %f, %f", self.mgx_offset, self.mgy_offset, self.mgz_offset)
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
        return self.max_az


####################################################################################################
#
#  Barometer / Altimeter - from DroTek MPU-9250 breakout
#
####################################################################################################
class MS5611:
    i2c = None

    # Registers/etc.
    __MS5611_RESET     = 0x1E
    __MS5611_CONV_D1_1 = 0x40
    __MS5611_CONV_D1_2 = 0x42
    __MS5611_CONV_D1_3 = 0x44
    __MS5611_CONV_D1_4 = 0x46
    __MS5611_CONV_D1_5 = 0x48
    __MS5611_CONV_D2_1 = 0x50
    __MS5611_CONV_D2_2 = 0x52
    __MS5611_CONV_D2_3 = 0x54
    __MS5611_CONV_D2_4 = 0x56
    __MS5611_CONV_D2_5 = 0x58

    def __init__(self, address=0x77):
        self.i2c = I2C(address)
        self.address = address

        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.writeByte(self.__MS5611_RESET)
        time.sleep(0.1)


####################################################################################################
#
#  Ultrasonic range finder - only supports up to 100kbps baudrate
#
####################################################################################################
class SRF02:
    i2c = None

    #Reisters/etc
    __SRF02_RA_CONFIG = 0x00
    __SRF02_RA_RNG_HI = 0x02
    __SRF02_RA_RNG_LO = 0x03
    __SRF02_RA_AUTO_HI = 0x04
    __SRF02_RA_AUTO_LO = 0x05

    def __init__(self, address=0x70):
        self.i2c = I2C(address)


    def pingProximity(self):
        #-------------------------------------------------------------------------------------------
        # Set up range units as centimeters and ping
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__SRF02_RA_CONFIG, 0x51)

    def pingProcessed(self):
        #-------------------------------------------------------------------------------------------
        # Check if data is available
        #-------------------------------------------------------------------------------------------
        rc = self.i2c.read8(self.__SRF02_RA_CONFIG)
        if rc == 0xFF:
            return False
        else:
            return True

    def readProximity(self):
        #-------------------------------------------------------------------------------------------
        # Read proximity - sensor units are centimeters so convert to meters.
        #-------------------------------------------------------------------------------------------
        range = self.i2c.readU16(self.__SRF02_RA_RNG_HI)
        return range / 100


####################################################################################################
#
#  LEDDAR range finder
#
####################################################################################################
class LEDDAR:

    def __init__(self):
        #-------------------------------------------------------------------------------------------
        # Connect to the LEDDAR
        #-------------------------------------------------------------------------------------------
        minimalmodbus.BAUDRATE=115200
        self.mmb = minimalmodbus.Instrument("/dev/ttyAMA0", 1, 'rtu')
        self.mmb.BAUDRATE=115200
        self.mmb.serial.baudrate = 115200

    def reset(self):
        #-------------------------------------------------------------------------------------------
        # Set up the base readings
        #-------------------------------------------------------------------------------------------
        (time_lss, time_mss, temperature, num_detections, distance) = self.mmb.read_registers(20, 5, 4)

        self.prev_timestamp = ((time_mss << 16) + time_lss) / 1000

        distance /= 1000
        self.init_distance = distance
        self.prev_distance = self.init_distance

        return self.init_distance

    def read(self):
        #-------------------------------------------------------------------------------------------
        # Read the current height and timestamp registers
        #-------------------------------------------------------------------------------------------
        (time_lss, time_mss, temperature, num_detections, distance) = self.mmb.read_registers(20, 5, 4)

        #-------------------------------------------------------------------------------------------
        # Convert units for returned values.
        #-------------------------------------------------------------------------------------------
        timestamp = ((time_mss << 16) + time_lss) / 1000
        distance /= 1000
        dd = (distance - self.prev_distance)
        dt = (timestamp - self.prev_timestamp)

        self.prev_timestamp = timestamp
        self.prev_distance = distance

        #-------------------------------------------------------------------------------------------
        # Finally remember to subtract the sensor height off the ground
        #-------------------------------------------------------------------------------------------
        distance -= self.init_distance

        return distance, dd, dt


####################################################################################################
#
#  PIX4FLOW 3D motion
#
####################################################################################################
class PX4FLOW:
    i2c = None

    # Registers/etc.
    __PX4FLOW_FRAME_COUNT      = 0x0                  # 2 bytes unsigned - number
    __PX4FLOW_PIXEL_FLOW_X     = 0x2                  # 2 bytes signed - latest x flow (pixels x 10)
    __PX4FLOW_PIXEL_FLOW_Y     = 0x4                  # 2 bytes signed - latest y flow (pixels x 10)
    __PX4FLOW_FLOW_COMP_M_X    = 0x6                  # 2 bytes signed - x velocity * 1000 (m/s)
    __PX4FLOW_FLOW_COMP_M_Y    = 0x8                  # 2 bytes signed - y velocity * 1000 (m/s)
    __PX4FLOW_QUAL_REG         = 0x0A                 # 2 bytes signed - optical flow quality (0:bad 255:max)
    __PX4FLOW_GYRO_X_RATE      = 0x0C                 # 2 bytes signed - gyro x rate (rad/sec)
    __PX4FLOW_GYRO_Y_RATE      = 0x0E                 # 2 bytes signed - gyro y rate (rad/sec)
    __PX4FLOW_GYRO_Z_RATE      = 0x10                 # 2 bytes signed - gyro z rate (rad/sec)
    __PX4FLOW_GYRO_RANGE       = 0x12                 # 1 byte unsigned - 0 - 7 = 50 - 2000 (degrees / second)
    __PX4FLOW_SONAR_TIMESTAMP  = 0x13                 # 1 byte unsigned - time since last sonar sample (ms)
    __PX4FLOW_GROUND_DISTANCE  = 0x14                 # 2 bytes signed  - ground distance (meters <0 = error)

    #--------------------------------------------------------------------------------------------------------
    # All integrals since the previous I2C read
    #--------------------------------------------------------------------------------------------------------
    __PX4FLOW_FRAME_COUNT_INTEGRAL        = 0x16      # 2 unsigned - number of reads since last I2C read
    __PX4FLOW_PIXEL_FLOW_X_INTEGRAL       = 0x18      # 2 signed - integrated flow around x axis (rad * 1000)
    __PX4FLOW_PIXEL_FLOW_Y_INTEGRAL       = 0x1A      # 2 signed - integrated flow around y axis (rad * 1000)
    __PX4FLOW_GYRO_X_RATE_INTEGRAL        = 0x1C      # 2 signed - integrated gyro X axis roll (rad * 1000)          # Would this be better for incremental roll rather than rate * dt
    __PX4FLOW_GYRO_Y_RATE_INTEGRAL        = 0x1E      # 2 signed - integrated gyro Y axis pitch (rad * 1000)         # Would this be better for incremental roll rather than rate * dt
    __PX4FLOW_GYRO_Z_RATE_INTEGRAL        = 0x20      # 2 signed - integrated gyro Z axis yaw (rad * 1000)           # Would this be better for incremental roll rather than rate * dt
    __PX4FLOW_TIMESPAN_INTEGRAL           = 0x22      # 4 unsigned - integrated time lapse (microseconds)
    __PX4FLOW_SONAR_TIMESTAMP_2           = 0x26      # 4 unsigned - time since last sonar update (microseconds)
    __PX4FLOW_GROUND_DISTANCE_2           = 0x2A      # 2 signed - ground distance (meters * 1000)
    __PX4FLOW_GYRO_TEMPERATURE            = 0x2C      # 2 signed - temperature (celsius * 100)
    __PX4FLOW_QUALITY_AVERAGE             = 0x2E      # 1 signed - averaged quality (0:bad 255:max)

    def __init__(self, address=0x42):
        self.i2c = I2C(address)
        self.address = address

    def read(self):
            #AB! Isn't SMBUS limited to 32 bytes?
            sensor_data =  self.i2c.readList(self.__PX4FLOW_FRAME_COUNT, 47)  # 22 if not using the integral registers

            hibyte = sensor_data[self.__PX4FLOW_FLOW_COMP_M_X + 1]
            if (hibyte > 127):
               hibyte -= 256
            x_velocity = ((hibyte << 8) + sensor_data[self.__PX4FLOW_FLOW_COMP_M_X]) / 1000

            hibyte = sensor_data[self.__PX4FLOW_FLOW_COMP_M_Y + 1]
            if (hibyte > 127):
               hibyte -= 256
            y_velocity = ((hibyte << 8) + sensor_data[self.__PX4FLOW_FLOW_COMP_M_Y]) / 1000

            hibyte = sensor_data[self.__PX4FLOW_QUAL_REG + 1]
            if (hibyte > 127):
               hibyte -= 256
            qual = (hibyte << 8) + sensor_data[self.__PX4FLOW_QUAL_REG]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_X_RATE + 1]
            if (hibyte > 127):
               hibyte -= 256
            pitch_rate = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_X_RATE]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_Y_RATE + 1]
            if (hibyte > 127):
               hibyte -= 256
            roll_rate = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_Y_RATE]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_Z_RATE + 1]
            if (hibyte > 127):
               hibyte -= 256
            yaw_rate = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_Z_RATE]

            hibyte = sensor_data[self.__PX4FLOW_GROUND_DISTANCE + 1]
            if (hibyte > 127):
               hibyte -= 256
            sonar_height = ((hibyte << 8) + sensor_data[self.__PX4FLOW_GROUND_DISTANCE]) / 1000

            sonar_dt = (sensor_data[self.__PX4FLOW_SONAR_TIMESTAMP]) / 1000

            #---------------------------------------------------------------------------------------
            # This is the flow integral set of registers; we don't need the contents
            # currently, but when we do, increase the i2c.readList above to 47.
            #---------------------------------------------------------------------------------------
            hibyte = sensor_data[self.__PX4FLOW_PIXEL_FLOW_X_INTEGRAL + 1]
            if (hibyte > 127):
               hibyte -= 256
            x_rps = ((hibyte << 8) + sensor_data[self.__PX4FLOW_PIXEL_FLOW_X_INTEGRAL]) / 1000

            hibyte = sensor_data[self.__PX4FLOW_PIXEL_FLOW_Y_INTEGRAL + 1]
            if (hibyte > 127):
               hibyte -= 256
            y_rps = ((hibyte << 8) + sensor_data[self.__PX4FLOW_PIXEL_FLOW_Y_INTEGRAL]) / 1000

            hibyte = sensor_data[self.__PX4FLOW_GYRO_X_RATE_INTEGRAL + 1]
            if (hibyte > 127):
               hibyte -= 256
            roll_increment = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_X_RATE_INTEGRAL]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_Y_RATE_INTEGRAL + 1]
            if (hibyte > 127):
               hibyte -= 256
            pitch_increment = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_Y_RATE_INTEGRAL]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_Z_RATE_INTEGRAL + 1]
            if (hibyte > 127):
               hibyte -= 256
            yaw_increment = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_Z_RATE_INTEGRAL]

            time_elapsed = 0
            for ii in range(4):
                time_elapsed += sensor_data[self.__PX4FLOW_TIMESPAN_INTEGRAL + ii] << (8 * ii)

            return x_rps, y_rps


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
        # Distance is in cm
        # Velocity is in cm between consecutive reads; sampling rate converts these to a velocity
        # Reading the list from 0x8F seems to get the previous reading, probably cached for the sake
        # of calculating the velocity next time round.
        #-------------------------------------------------------------------------------------------
        '''
        gll_bytes = self.i2c.readList(0x80 | self.__GLL_LAST_DELAY_HIGH, 2)
        dist1 = gll_bytes[0]
        dist2 = gll_bytes[1]
        distance = ((dist1 << 8) + dist2) / 100
        '''

        dist1 = self.i2c.readU8(self.__GLL_FULL_DELAY_HIGH)
        dist2 = self.i2c.readU8(self.__GLL_FULL_DELAY_LOW)
        distance = ((dist1 << 8) + dist2) / 100

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
        error = target - input
        return (error if abs(error) < math.pi else (error - 2 * math.pi * error / abs(error)))


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
        # Initialize the RPIO DMA PWM for this ESC in microseconds - 1ms - 2ms of
        # pulse widths with 3ms carrier.
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 1000
        self.min_pulse_width = 1000 + base_pwm
        self.max_pulse_width = 1999

        #-------------------------------------------------------------------------------------------
        # Name - for logging purposes only
        #-------------------------------------------------------------------------------------------
        self.name = name

        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        #-------------------------------------------------------------------------------------------
        self.set(0)

    def set(self, pw):
        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, 1000 + pw)

    def update(self, pw):
        self.pulse_width = int(self.min_pulse_width + pw)

        if self.pulse_width < self.min_pulse_width:
            self.pulse_width = self.min_pulse_width
        if self.pulse_width > self.max_pulse_width:
            self.pulse_width = self.max_pulse_width

        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.pulse_width)


####################################################################################################
#
# Angles required to convert between Earth (interal reference frame) and Quadcopter (body reference
# frame).
#
# This is used for reorientating gravity into the quad frame to calculated quad frame velocities
#
####################################################################################################
def GetRotationAngles(ax, ay, az):

    #-----------------------------------------------------------------------------------------------
    # What's the angle in the x and y plane from horizontal in radians?
    #-----------------------------------------------------------------------------------------------
    pitch = math.atan2(-ax, math.pow(math.pow(ay, 2) + math.pow(az, 2), 0.5))
    roll = math.atan2(ay, az)

    return pitch, roll


####################################################################################################
#
# Absolute angles of tilt compared to the earth reference frame.
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
# Convert a vector to quadcopter-frame coordinates from earth-frame coordinates
#
####################################################################################################
def RotateE2Q(evx, evy, evz, pa, ra, ya):

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
# Convert a vector to earth-frame coordinates from quadcopter-frame coordinates.
#
####################################################################################################
def RotateQ2E(qvx, qvy, qvz, pa, ra, ya):

    #===============================================================================================
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
    #===============================================================================================
    c_pa = math.cos(pa)
    s_pa = math.sin(pa)
    c_ra = math.cos(ra)
    s_ra = math.sin(ra)
    c_ya = math.cos(ya)
    s_ya = math.sin(ya)

    evx = qvx * c_pa * c_ya + qvy * (s_ra * s_pa * c_ya - c_ra * s_ya) + qvz * (c_ra * s_pa * c_ya + s_ra * s_ya)
    evy = qvx * c_pa * s_ya + qvy * (s_ra * s_pa * s_ya + c_ra * c_ya) + qvz * (c_ra * s_pa * s_ya - s_ra * c_ya)
    evz = -qvx * s_pa       + qvy *  s_ra * c_pa                       + qvz * c_pa * c_ra

    return evx, evy, evz

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
def GPIOInit(FIFOOverflowISR, LEDDARDataReadyISR):
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
    GPIO.add_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.RISING) #, FIFOOverflowISR)

#AB!    GPIO.setup(GPIO_POWER_BROWN_OUT_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
#AB!    GPIO.add_event_detect(GPIO_POWER_BROWN_OUT_INTERRUPT, GPIO.FALLING)

    GPIO.setup(GPIO_LEDDAR_DR_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
#    GPIO.add_event_detect(GPIO_LEDDAR_DR_INTERRUPT, GPIO.RISING, LEDDARDataReadyISR)

    GPIO.setup(GPIO_GARMIN_BUSY, GPIO.IN, GPIO.PUD_DOWN)
#    GPIO.add_event_detect(GPIO_GARMIN_BUSY, GPIO.FALLING)

    GPIO.setup(GPIO_BUTTON, GPIO.IN, GPIO.PUD_UP)

    GPIO.setup(GPIO_LED, GPIO.OUT)
    GPIO.output(GPIO_LED, GPIO.LOW)


####################################################################################################
#
# GPIO pins cleanup for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOTerm():
#    GPIO.remove_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT)
#    GPIO.remove_event_detect(GPIO_LEDDAR_DR_INTERRUPT)
    GPIO.cleanup()


####################################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
####################################################################################################
def CheckCLI(argv):
    cli_fly = False
    cli_hover_target = 0

    #-----------------------------------------------------------------------------------------------
    # Other configuration defaults
    #-----------------------------------------------------------------------------------------------
    cli_test_case = 0
    cli_diagnostics = False
    cli_rtf_period = 0.1
    cli_tau = 7.5
    cli_calibrate_0g = False
    cli_flight_plan = ''
    cli_calibrate_compass = False

    hover_target_defaulted = True

    #-------------------------------------------------------------------------------------------
    # Defaults for vertical distance PIDs
    #-------------------------------------------------------------------------------------------
    cli_vdp_gain = 1.0
    cli_vdi_gain = 0.0
    cli_vdd_gain = 0.0

    #-------------------------------------------------------------------------------------------
    # Defaults for horizontal distance PIDs
    #-------------------------------------------------------------------------------------------
    cli_hdp_gain = 1.0
    cli_hdi_gain = 0.0
    cli_hdd_gain = 0.0

    #-------------------------------------------------------------------------------------------
    # Defaults for horizontal velocity PIDs
    #-------------------------------------------------------------------------------------------
    cli_hvp_gain = 1.6
    cli_hvi_gain = 0.0
    cli_hvd_gain = 0.0


    #-------------------------------------------------------------------------------------------
    # Per frame specific values
    #-------------------------------------------------------------------------------------------
    if i_am_hermione:
        #-------------------------------------------------------------------------------------------
        # Chloe's PID configuration due to using her frame / ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 500 # 500 = CCF 1355 + Quadframe; 420 = T-motor 1240 + F450 Alloy Arms; 370 = CCF 1355

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 360.0
        cli_vvi_gain = 180.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 100.0
        cli_pri_gain = 0.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 100.0
        cli_rri_gain = 0.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 200.0
        cli_yri_gain = 0.0
        cli_yrd_gain = 0.0

    elif i_am_chloe:
        #-------------------------------------------------------------------------------------------
        # Chloe's PID configuration due to using her frame / ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 400

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 320.0
        cli_vvi_gain = 160.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 50.0
        cli_pri_gain = 0.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 50.0
        cli_rri_gain = 0.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 100.0
        cli_yri_gain = 0.0
        cli_yrd_gain = 0.0

    elif i_am_zoe:
        #-------------------------------------------------------------------------------------------
        # Zoe's PID configuration due to using her ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 300 # 300 = CCF | AirGear white 0903; 380 = AirGear Floppy props

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 300.0 # Floppy props: 400.0
        cli_vvi_gain = 150.0 # Floppy props: 200.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 25.0  # Floppy props: 100.0
        cli_pri_gain = 0.0   # Floppy props: 10.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 25.0  # Floppy props: 90.0
        cli_rri_gain = 0.0   # Floppy props: 9.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 50.0 # Floppy props: 80.0
        cli_yri_gain = 0.0  # Floppy props: 8.0
        cli_yrd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Right, let's get on with reading the command line and checking consistency
    #-----------------------------------------------------------------------------------------------
    try:
        opts, args = getopt.getopt(argv,'df:gh:r:', ['cc', 'tc=', 'tau=', 'vdp=', 'vdi=', 'vdd=', 'vvp=', 'vvi=', 'vvd=', 'hdp=', 'hdi=', 'hdd=', 'hvp=', 'hvi=', 'hvd=', 'prp=', 'pri=', 'prd=', 'rrp=', 'rri=', 'rrd=', 'tau=', 'yrp=', 'yri=', 'yrd='])
    except getopt.GetoptError:
        logger.critical('Must specify one of -f or -g or --tc')
        logger.critical('  qc.py')
        logger.critical('  -f set the flight plan CSV file')
        logger.critical('  -h set the hover PWM pulse width - default: %dus', cli_hover_target)
        logger.critical('  -d enable diagnostics')
        logger.critical('  -g calibrate X, Y axis 0g')
        logger.critical('  -r ??  set the ready-to-fly period - default: %fs', cli_rtf_period)
        logger.critical('  --cc   calibrate compass')
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
            cli_hover_target = int(arg)
            hover_target_defaulted = False

        elif opt in '-d':
            cli_diagnostics = True

        elif opt in '-g':
            cli_calibrate_0g = True

        elif opt in '-r':
            cli_rtf_period = float(arg)

        elif opt in '--cc':
            cli_calibrate_compass = True

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

    if not cli_fly and cli_test_case == 0 and not cli_calibrate_0g and not cli_calibrate_compass:
        raise ValueError('Must specify one of -f or --tc or --cc')

    elif cli_hover_target < 0 or cli_hover_target > 1000:
        raise ValueError('Hover speed must lie in the following range: 0 <= hover speed <= 1000')

    elif cli_test_case == 0 and cli_fly:
        print 'Pre-flight checks passed, enjoy your flight, sir!'

    elif cli_test_case == 0 and cli_calibrate_0g:
        print 'Proceeding with 0g calibration'

    elif cli_test_case == 0 and cli_calibrate_compass:
        print "Proceeding with compass calibration"

    elif cli_test_case != 1 and cli_test_case != 2:
        raise ValueError('Only 1 or 2 are valid testcases')

    elif cli_test_case == 1 and hover_target_defaulted:
        raise ValueError('You must choose a specific hover speed (-h) for test case 1 - try 200')

    return cli_fly, cli_flight_plan, cli_calibrate_0g, cli_calibrate_compass, cli_hover_target, cli_vdp_gain, cli_vdi_gain, cli_vdd_gain, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hdp_gain, cli_hdi_gain, cli_hdd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_prp_gain, cli_pri_gain, cli_prd_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_yrp_gain, cli_yri_gain, cli_yrd_gain, cli_test_case, cli_rtf_period, cli_tau, cli_diagnostics


####################################################################################################
#
# Flight plan management
#
####################################################################################################
class FlightPlan():

    def __init__(self, quadcopter, fp_filename):

        self.quadcopter = quadcopter
        self.fp_index = 0
        self.fp_prev_index = 0
        self.elapsed_time = 0.0
        self.fp_steps = 2

        self.fp_evx_target = array('f', [])
        self.fp_evy_target = array('f', [])
        self.fp_evz_target = array('f', [])
        self.fp_time       = array('f', [])
        self.fp_name = []

        self.fp_evx_target.append(0.0)
        self.fp_evy_target.append(0.0)
        self.fp_evz_target.append(0.0)
        self.fp_time.append(0.0)
        self.fp_name.append("RTF")

        self.edx_target = 0.0
        self.edy_target = 0.0
        self.edz_target = 0.0

        with open(fp_filename, 'rb') as fp_csv:
            fp_reader = csv.reader(fp_csv)
            for fp_row in fp_reader:

                if len(fp_row) == 0 or (fp_row[0] != '' and fp_row[0][0] == '#'):
                    continue
                if len(fp_row) != 5:
                    break

                self.fp_evx_target.append(float(fp_row[0]))
                self.fp_evy_target.append(float(fp_row[1]))
                self.fp_evz_target.append(float(fp_row[2]))
                self.fp_time.append(float(fp_row[3]))
                self.fp_name.append(fp_row[4].strip())
                self.fp_steps += 1
            else:
                self.fp_evx_target.append(0.0)
                self.fp_evy_target.append(0.0)
                self.fp_evz_target.append(0.0)
                self.fp_time.append(0.0)
                self.fp_name.append("STOP")
                return

        raise ValueError("Error in CSV file; '%s'" % fp_row)


    def getTargets(self, delta_time):
        self.elapsed_time += delta_time

        fp_total_time = 0.0
        for fp_index in range(0, self.fp_steps):
            fp_total_time += self.fp_time[fp_index]
            if self.elapsed_time < fp_total_time:
                break
        else:
            self.quadcopter.keep_looping = False

        if fp_index != self.fp_prev_index:
            logger.critical("%s", self.fp_name[fp_index])
            self.fp_prev_index = fp_index

        evx_target = self.fp_evx_target[fp_index]
        evy_target = self.fp_evy_target[fp_index]
        evz_target = self.fp_evz_target[fp_index]

        self.edx_target += evx_target * delta_time
        self.edy_target += evy_target * delta_time
        self.edz_target += evz_target * delta_time

        return evx_target, evy_target, evz_target, self.edx_target, self.edy_target, self.edz_target


####################################################################################################
#
# Functions to lock memory to prevent paging
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
        camera.contrast = 50

        vofi = io.open("/dev/shm/motion_stream", mode = "wb", buffering = 0)
        try:
            camera.start_recording('/dev/null', format='h264', motion_output=vofi, quality=23)

            try:
                while True:
                    camera.wait_recording(1.0)
            except KeyboardInterrupt:
                pass
            finally:
                camera.stop_recording()
        finally:
            vofi.close()

####################################################################################################
#
# Two phase video frame macro-block processing.  The first phase (__init__) takes a frame of macro-blocks and
# dumps them into a dictionary indexed by the vector, and the value is the count of macro-blocks at
# this vector point in the frame.  The second phase (process) takes the dictionary, and for each entry,
# checks for neighbouring entries, adding half their value to the current directory area.  It also tracks
# the highest value so far.  At the end, merges one or more vectors with the maximum score.
#
####################################################################################################
class VideoFrameProcessor:

    def __init__(self, iframe, yaw_increment):
        self.vector_dict = {}
        self.vector_list = []
        self.yaw_increment = yaw_increment
        self.next_phase = 0

        if len(iframe) % 3 != 0:
            raise ValueError("iFrame size calculation wrong")
        self.num_mbs = int(len(iframe) / 3)

        #-----------------------------------------------------------------------------------
        # Split the iframe into a list of macro-block vectors.  The mapping from each iframe
        # in idy, idy depends on how the camera is orientated WRT the frame.
        # This must be checked callibrated.
        #-----------------------------------------------------------------------------------
        for ii in range(self.num_mbs):
            idy = iframe[3 * ii]
            idx = iframe[3 * ii + 1]
            sad = iframe[3 * ii + 2]

            if idx == 0 and idy == 0: # and sad == 0:
                continue

            self.vector_list.append((idx, idy))

        if len(self.vector_list) == 0:
            raise ValueError("Empty Video Frame Object")

        self.phase = 1

    def phase1(self):
        #----------------------------------------------------------------------------------
        # Unyaw the list of vectors, overwriting the yaw list
        #----------------------------------------------------------------------------------
        unyawed_vectors = []

        s_yaw = math.sin(self.yaw_increment)
        c_yaw = math.cos(self.yaw_increment)

        #----------------------------------------------------------------------------------
        # Undo the yaw with the inverse rotation matrix
        #----------------------------------------------------------------------------------
        for vector in self.vector_list:
            idx, idy = vector
            uvx = c_yaw * idx - s_yaw * idy
            uvy = s_yaw * idx + c_yaw * idy
            unyawed_vectors.append((int(round(uvx)), int(round(uvy))))

        self.vector_list = unyawed_vectors

        self.phase = 2

    def phase2(self):
        #-----------------------------------------------------------------------------------
        # Build the dictionary of unyawed vectors; they score 2 because of the next phase
        #-----------------------------------------------------------------------------------
        for (idx, idy) in self.vector_list:
            if (idx, idy) in self.vector_dict:
                self.vector_dict[(idx, idy)] += 2
            else:
                self.vector_dict[(idx, idy)] = 2

        self.phase = 3

    def phase3(self):
        #-----------------------------------------------------------------------------------
        # Pass again through the dictionary of vectors, building up clusters based on neighbours.
        #-----------------------------------------------------------------------------------
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

        self.phase = 4

    def phase4(self):
        #-----------------------------------------------------------------------------------
        # Now we've collected the clusters of the best score vectors in the frame, average
        # and reyaw it
        #-----------------------------------------------------------------------------------
        sum_score = 0
        sum_x = 0
        sum_y = 0
        for (vector_x, vector_y), vector_score in self.best_vectors:
            sum_x += vector_x * vector_score
            sum_y += vector_y * vector_score
            sum_score += vector_score

        best_x = sum_x / sum_score
        best_y = sum_y / sum_score

        #-----------------------------------------------------------------------------------
        # Reyaw the results with the standard rotation matrix
        #-----------------------------------------------------------------------------------
        s_yaw = math.sin(self.yaw_increment)
        c_yaw = math.cos(self.yaw_increment)

        idx = c_yaw * best_x + s_yaw * best_y
        idy = -s_yaw * best_x + c_yaw * best_y

        self.phase = 0

        return idx, idy


    def process(self, yaw_increment = None):

        #------------------------------------------------------------------------------------
        # Phase one - take the list of macro blocks and undo yaw
        #------------------------------------------------------------------------------------
        if self.phase == 1:
            self.phase1()
            return None

        #------------------------------------------------------------------------------------
        # Phase 2 - build the dictionary of int rounded unyawed vectors
        #------------------------------------------------------------------------------------
        if self.phase == 2:
            self.phase2()
            return None

        #------------------------------------------------------------------------------------
        # Phase 3 - walk the dictionary, looking for neighbouring clusters and score them
        #------------------------------------------------------------------------------------
        if self.phase == 3:
            self.phase3()
            return None

        #------------------------------------------------------------------------------------
        # Phase 4 - average highest peak clusters, redo yaw, and return result
        #------------------------------------------------------------------------------------
        if self.phase == 4:
            idx, idy = self.phase4()
            return idx, idy



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
        global i_am_chloe
        global i_am_hermione
        i_am_zoe = False
        i_am_chloe = False
        i_am_hermione = False

        my_name = os.uname()[1]
        if my_name == "zoe.local" or my_name == "zoe":
            print "Hi, I'm Zoe.  Nice to meet you!"
            i_am_zoe = True
        elif my_name == "chloe.local" or my_name == "chloe":
            print "Hi, I'm Chloe.  Nice to meet you!"
            i_am_chloe = True
        elif my_name == "hermione.local" or my_name == "hermione":
            print "Hi, I'm Hermione.  Nice to meet you!"
            i_am_hermione = True
        else:
            print "Sorry, I'm not qualified to fly this quadcopter."
            return

        #-------------------------------------------------------------------------------------------
        # Set up extra sensors based on quad identify.
        # -  urf_installed can only be used once the RPi kernel I2C driver supports clock stretching
        # -  compass_installed can only be used with an MPU9250
        # -  barometer_installed can only be used with MS5611 as included on DroTek MPU9250 breakout
        # -  px4flow_installed only works with the original; clones don't work
        #-------------------------------------------------------------------------------------------
        X8 = False
        if i_am_zoe:
            self.urf_installed = False
            self.leddar_installed = False
            self.px4flow_installed = False
            self.compass_installed = True
            self.camera_installed = False
            self.barometer_installed = False
            self.gll_installed = False
        elif i_am_chloe:
            self.urf_installed = False
            self.leddar_installed = False
            self.px4flow_installed = False
            self.compass_installed = True
            self.camera_installed = True
            self.barometer_installed = False
            self.gll_installed = True
        elif i_am_hermione:
            self.urf_installed = False
            self.leddar_installed = False
            self.px4flow_installed = False
            self.compass_installed = True
            self.camera_installed = True
            self.barometer_installed = False
            self.gll_installed = True
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
        # First log, whose flying and under what configuration
        #-------------------------------------------------------------------------------------------
        logger.warning("%s is flying.", "Zoe" if i_am_zoe else "Hermione" if i_am_hermione else "Chloe")

        #-------------------------------------------------------------------------------------------
        # Set the BCM pin assigned to the FIFO overflow and LEDDAR data ready interrupts
        #-------------------------------------------------------------------------------------------
        global GPIO_POWER_BROWN_OUT_INTERRUPT
        GPIO_POWER_BROWN_OUT_INTERRUPT = 35

        global GPIO_FIFO_OVERFLOW_INTERRUPT
        GPIO_FIFO_OVERFLOW_INTERRUPT = 24 if X8 else 22

        global GPIO_LEDDAR_DR_INTERRUPT
        GPIO_LEDDAR_DR_INTERRUPT = 18

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
        # Enable GPIO for the FIFO overflow and LEDDAR data ready hardware interrupts.
        #-------------------------------------------------------------------------------------------
        GPIOInit(self.fifoOverflowISR, self.LEDDARDataReadyISR)

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
        # with the same ESC firmware so have the same base_pwm
        #-------------------------------------------------------------------------------------------
        global base_pwm
        base_pwm = 0
        if i_am_zoe:
            base_pwm = 150
        elif i_am_chloe:
            base_pwm = 150
        elif i_am_hermione:
            base_pwm = 150

        self.esc_list = []
        for esc_index in range(8 if X8 else 4):
            esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
            self.esc_list.append(esc)

        #===========================================================================================
        # Globals for the IMU setup
        # alpf               - the accelerometer low pass filter
        # glpf               - the gyro low pass filter
        # adc_frequency      - the sampling rate of the ADC
        # sampling_rate      - the data sampling rate and thus data ready interrupt rate
        #                      motion processing.
        # samples_per_motion - how often motion processing is triggered compared to the sampling_rate
        #
        # If LEDDAR is installed, reading the modbus slows the motion processing down significantly
        # more than it should, and this causes FIFO overflows.  By reducing the sampling frequency,
        # the FIFO overflow time increases.  Samples per motion follows suit to maintain motion
        # processing at about 100Hz.
        #===========================================================================================
        alpf = 0
        glpf = 1

        global adc_frequency
        global sampling_rate
        global samples_per_motion
        adc_frequency = 1000        #AB! defined by dlpf >= 1; DO NOT USE ZERO => 8000 adc_frequency

        if self.leddar_installed or self.camera_installed or self.gll_installed:
            sampling_rate = 500        #AB! <= 500 to prevent FIFO overflow with extra sensors
            samples_per_motion = 5
        else:
            sampling_rate = 1000
            samples_per_motion = 10

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
        # 512/12 is the maximum number of batches in the FIFO
        # 1000 / sampling_rate is the time fraction - slower sampling means more spare time available
        #
        #-------------------------------------------------------------------------------------------
        self.FIFO_PRIORITY = int(round(0.010 * sampling_rate))
        self.FIFO_OVERFLOW = int(round(512 / 12 - 0.010 * sampling_rate))

        #-------------------------------------------------------------------------------------------
        # Initialize the compass object.
        #-------------------------------------------------------------------------------------------
        if self.compass_installed:
            mpu6050.initCompass()

        #-------------------------------------------------------------------------------------------
        # Initialize the barometer / altimeter I2C object
        #-------------------------------------------------------------------------------------------
        if self.barometer_installed:
            global ms5611
            ms5611 = MS5611(0x77)

        #-------------------------------------------------------------------------------------------
        # Initialize the ultrasonic range finder I2C object and calibrate the sensor
        #-------------------------------------------------------------------------------------------
        if self.urf_installed:
            global srf02
            srf02 = SRF02(0x70)
            for ii in range(10):
                srf02.pingProximity()
                time.sleep(0.070)  # 70ms sleep guarantees a valid result
                sfr02.readProximity()

        #-------------------------------------------------------------------------------------------
        # Start up the LEDDAR now to give it time to self-tune and settle; kill off the terminal
        # running on the serial bus so LEDDAR can use it
        #-------------------------------------------------------------------------------------------
        if self.leddar_installed:
            os.system("systemctl stop serial-getty@ttyAMA0.service")
            global leddar
            leddar = LEDDAR()

        #-------------------------------------------------------------------------------------------
        # Initialize PX4FLOW
        #-------------------------------------------------------------------------------------------
        if self.px4flow_installed:
            global px4flow
            px4flow = PX4FLOW()

        #-------------------------------------------------------------------------------------------
        # Initialize the Garmin LiDAR-Lite V3
        #-------------------------------------------------------------------------------------------
        if i_am_zoe:
            self.tracking_rate = 10
        else:
            self.tracking_rate = 20

        if self.gll_installed:
            global gll
            gll = GLL(rate = self.tracking_rate)

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
        #-----------------------------------------------------------------------------------------------
        # Give the PWM 5s to allow the ESCs to synchronize.
        #-----------------------------------------------------------------------------------------------
        print "Just checking a few details.  Gimme a few seconds..."

        #-------------------------------------------------------------------------------------------
        # Check the command line for calibration or flight parameters
        #-------------------------------------------------------------------------------------------
        try:
            flying, flight_plan, calibrate_0g, calibrate_compass, hover_target, vdp_gain, vdi_gain, vdd_gain, vvp_gain, vvi_gain, vvd_gain, hdp_gain, hdi_gain, hdd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, rtf_period, atau, diagnostics = CheckCLI(self.argv)
        except ValueError, err:
            print "Command line error: %s" % err
            return

        logger.warning("fly = %s, flight plan = %s, calibrate_0g = %d, calibrate_compass = %s, hover_target = %d, vdp_gain = %f, vdi_gain = %f, vdd_gain= %f, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hdp_gain = %f, hdi_gain = %f, hdd_gain = %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, prp_gain = %f, pri_gain = %f, prd_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, yrp_gain = %f, yri_gain = %f, yrd_gain = %f, test_case = %d, rtf_period = %f, atau = %f, diagnostics = %s",
                flying, flight_plan, calibrate_0g, calibrate_compass, hover_target, vdp_gain, vdi_gain, vdd_gain, vvp_gain, vvi_gain, vvd_gain, hdp_gain, hdi_gain, hdd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, rtf_period, atau, diagnostics)


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
            if calibrate_compass:
                if not mpu6050.calibrateCompass():
                    print "Compass calibration error, abort"
                return
            elif not mpu6050.loadCompassCalibration():
                print "Compass calibration data not found"
                return
        elif calibrate_compass:
            print "Compass not installed, calibration not necessary."
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
                for count in range(0, hover_target, 10):
                    #-------------------------------------------------------------------------------
                    # Spin up to user determined (-h) hover speeds ~200
                    #-------------------------------------------------------------------------------
                    esc.set(count)
                    time.sleep(0.01)

                #-----------------------------------------------------------------------------------
                # The prop is now up to the configured spin rate.  Sleep for 5s then stop and move
                # on to the next prop.
                #-----------------------------------------------------------------------------------
                time.sleep(5)
                esc.set(0)
            return
        #-------------------------------------------------------------------------------------------
        # END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the
        #                      right way
        #-------------------------------------------------------------------------------------------

        #===========================================================================================
        # OK, we're in flight mode, better get on with it
        #===========================================================================================
        edx_target = 0.0
        edy_target = 0.0
        edz_target = 0.0

        evx_target = 0.0
        evy_target = 0.0
        evz_target = 0.0

        ya_target = 0.0

        edx_input = 0.0
        edy_input = 0.0
        edz_input = 0.0

        evx_input = 0.0
        evy_input = 0.0
        evz_input = 0.0

        qdx_input = 0.0
        qdy_input = 0.0
        qdz_input = 0.0

        qvx_input = 0.0
        qvy_input = 0.0
        qvz_input = 0.0

        qdx_increment = 0.0
        qdy_increment = 0.0
        qdz_increment = 0.0

        qvx_increment = 0.0
        qvy_increment = 0.0
        qvz_increment = 0.0

        qvx_integral = 0.0
        qvy_integral = 0.0
        qvz_integral = 0.0

        #-------------------------------------------------------------------------------------------
        # Set up the global constants - gravity in meters per second squared
        #-------------------------------------------------------------------------------------------
        GRAV_ACCEL = 9.80665

        #-------------------------------------------------------------------------------------------
        # Register the flight plan with the authorities
        #-------------------------------------------------------------------------------------------
        try:
            fp = FlightPlan(self, flight_plan)
        except Exception, err:
            print "%s error: %s" % (flight_plan, err)
            return

        #-------------------------------------------------------------------------------------------
        # Flush the FIFO, collect roughly half a FIFO full of samples
        #-------------------------------------------------------------------------------------------
        mpu6050.flushFIFO()
        time.sleep(20 / sampling_rate)
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO(nfb)

        #-------------------------------------------------------------------------------------------
        # Feed back the gyro offset calibration
        #-------------------------------------------------------------------------------------------
        mpu6050.setGyroOffsets(qrx, qry, qrz)

        #-------------------------------------------------------------------------------------------
        # Calculate gravity, scaling the accelerometer readings to produce (0, 0, 1) in the earth frame.
        # This is only really important when (double) integrated (acceleration - gravity) produces
        # velocities and distances.  By adding the scaling, there's more chance the integrated distance
        # and velocity readings will align better with readings from other sensors providing direct
        # velocity and distance readings (e.g. LiDAR, Video, URF, PX4FLOW etc.)
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
        # Rotate gravity back to earth frame.
        #-------------------------------------------------------------------------------------------
        egx, egy, egz = RotateQ2E(qax, qay, qaz, pa, ra, ya)

        '''
        g_scale = 1 / math.pow(math.pow(qax, 2) + math.pow(qay, 2) + math.pow(qaz, 2), 0.5)

        qax *= g_scale
        qay *= g_scale
        qaz *= g_scale

        egx = 0.0
        egy = 0.0
        egz = 1.0
        '''

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

        #-------------------------------------------------------------------------------------------
        # Log the critical parameters from this warm-up: the take-off surface tilt, and gravity.
        # Note that some of the variables used above are used in the main processing loop.  Messing
        # with the above code can have very unexpected effects in flight.
        #-------------------------------------------------------------------------------------------
        logger.warning("pitch, %f, roll, %f", math.degrees(pa), math.degrees(ra))
        logger.warning("egx, %f, egy, %f, egz %f", egx, egy, egz)
        logger.warning("based upon %d samples", dt * sampling_rate)

        #-------------------------------------------------------------------------------------------
        # Initialize the base setting of earth frame take-off height - i.e. the vertical distance from
        # the height sensor or the take-off platform / leg height if no sensor is available.
        #-------------------------------------------------------------------------------------------
        eftoh = 0.0

        #-------------------------------------------------------------------------------------------
        # Read the initial height of the URF from the ground
        #-------------------------------------------------------------------------------------------
        urfz = 0
        if self.urf_installed:
            srf02.pingProximity()
            time.sleep(0.070)  # 70ms sleep guarantees a valid result
            if not srf02.pingProcessed():
                print "URF FMR FECK"
                return
            else:
                eftoh = srf02.readProximity()
                eftoh *= tilt_ratio
                srf02.pingProximity()

        #-------------------------------------------------------------------------------------------
        # Get an initial set of readings from the LEDDAR thus clearing the DR interrupt
        #-------------------------------------------------------------------------------------------
        l_dist = 0.0
        l_dd = 0.0
        l_dt = 0.0
        if self.leddar_installed:
            eftoh = leddar.reset() * tilt_ratio

        #-------------------------------------------------------------------------------------------
        # Reset the take-off height
        #-------------------------------------------------------------------------------------------
        px_qvx = 0.0
        px_qvy = 0.0
        if self.px4flow_installed:
            px4flow.get()

        #-------------------------------------------------------------------------------------------
        # Get an initial take-off height
        #-------------------------------------------------------------------------------------------
        g_dist = 0.0
        if self.gll_installed:
            print "Couple of seconds to let the LiDAR settle..."
            for ii in range(2 * self.tracking_rate):
                time.sleep(1 / self.tracking_rate)
                g_dist, g_vel = gll.read()
                eftoh += g_dist * tilt_ratio
            eftoh /= (2 * self.tracking_rate)

        logger.warning("EFTOH:, %f", eftoh)

        #-------------------------------------------------------------------------------------------
        # Prime the direction vector of the earth's magnotic core to provide long term yaw stability.
        #-------------------------------------------------------------------------------------------
        magx = 0.0
        magy = 0.0
        magz = 0.0

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
        # Set up the video macro-block parameters
        # Video supported upto 1080p @ 30Hz but restricted by processing speed.
        #-------------------------------------------------------------------------------------------
        camera_version = 2

        if i_am_hermione:
            frame_width = 640    # an exact multiple of mb_size
            frame_height = 640   # an exact multiple of mb size
        elif i_am_chloe:
            frame_width = 480    # an exact multiple of mb_size
            frame_height = 480   # an exact multiple of mb size
        elif i_am_zoe:
            frame_width = 560    # an exact multiple of mb_size
            frame_height = 560   # an exact multiple of mb size

        frame_rate = self.tracking_rate
        mb_size = 16
        bytes_per_mb = 4
        mbs_per_frame = int((frame_width / mb_size + 1) * (frame_height / mb_size))
        mbs_frame_bytes = mbs_per_frame * bytes_per_mb
        vfp = None

        #------------------------------------------------------------------------------------------
        # The complementary filter for vertical and horizontal fusion processing depends on the
        # Garmin LiDAR-lite sampling frequency and video fps.  Only Hermione can manage these faster
        # speeds and resilutions.
        #------------------------------------------------------------------------------------------
        vtau = 20 / frame_rate
        dtau = 20 / frame_rate

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
        aov = (48.8 if camera_version == 2 else 41) * math.pi / 180
        scale = 2 * math.tan(aov / 2) / frame_width

        format = '=' + 'bbH' * mbs_per_frame

        read_list = []
        write_list = []
        exception_list = []

        mbx = 0.0
        mby = 0.0

        vvf = False
        hvf = False
        vdf = False
        hdf = False

        camera_data_update = False

        if self.camera_installed:
            #---------------------------------------------------------------------------------------
            # Setup a shared memory based data stream for the PiCamera video motion output
            #---------------------------------------------------------------------------------------
            os.mkfifo("/dev/shm/motion_stream")

            def Daemonize():
                os.setpgrp()

            video = subprocess.Popen(["python", __file__, str(frame_width), str(frame_height), str(frame_rate)], preexec_fn =  Daemonize)
            # video = subprocess.Popen(["raspivid", "-w", "320", "-h", "320", "-fps", "10", "-fl", "-n", "-t", "0", "-o", "/dev/null", "-x", "/dev/shm/motion_stream"], preexec_fn = Daemonize)
            while True:
                try:
                    py_fifo = open("/dev/shm/motion_stream", "rb")
                except:
                    continue
                else:
                    break
            read_list = [py_fifo]

            #---------------------------------------------------------------------------------------
            # Capture a couple of seconds' samples to let the video settle
            #---------------------------------------------------------------------------------------
            print "Couple of seconds to let the video settle..."
            for ii in range(int(2 * frame_rate)):
                frame = py_fifo.read(mbs_frame_bytes)
                if len(frame) != mbs_frame_bytes:
                    print "Feck"
                    return

            logger.warning("Video @, %d, %d,  pixels, %d, fps", frame_width, frame_height, frame_rate)

        #------------------------------------------------------------------------------------------
        # Set the props spinning at their base rate to ensure initial kick-start doesn't get spotted
        # by the sensors messing up the flight thereafter. base_pwm is determined by running testcase 1
        # multiple times incrementing -h slowly until a level of PWM is found where all props just spin.
        # This depends on the firmware in the ESCs
        #------------------------------------------------------------------------------------------
        print "Couple of seconds to spin up the motors..."

        for esc in self.esc_list:
            esc.set(base_pwm)

        hover_speed = base_pwm
        hsf = float(base_pwm)
        ready_to_fly = False
        time.sleep(2.0)

        print ""
        print "################################################################################"
        print "#                                                                              #"
        print "#                             Thunderbirds are go!                             #"
        print "#                                                                              #"
        print "################################################################################"
        print ""

        #-------------------------------------------------------------------------------------------
        # Diagnostic log header
        #-------------------------------------------------------------------------------------------
        if diagnostics:
            logger.warning("time, dt, loops, " +
                            "temperature, " +
                            "magx, magy, magz, " +
                            "edx_input, edy_input, edz_input, " +
                            "evx_input, evy_input, evz_input, " +
                            "edx_target, edy_target, edz_target, " +
                            "evx_target, evy_target, evz_target, " +
                            "qrx, qry, qrz, " +
                            "qax, qay, qaz, " +
                            "qgx, qgy, qgz, " +
                            "pitch, roll, yaw, " +
                            "qdx_input, qdx_target, qvx_input, qvx_target, pa_input, pa_target, pr_input, pr_target, pr_out, " +
                            "qdy_input, qdy_target, qvy_input, qvy_target, ra_input, ra_target, rr_input, rr_target, rr_out, " +
                            "qdz_input, qdz_target, qvz_input, qvz_target, qvz_out, " +
                            "ya_input, ya_target, yr_input, yr_target, yr_out, " +
                            "FL PWM, FR PWM, BL PWM, BR PWM")

        #-------------------------------------------------------------------------------------------
        # Used for total flight length only
        #-------------------------------------------------------------------------------------------
        start_flight = time.time()

        #-------------------------------------------------------------------------------------------
        # Flush the FIFO and enable the FIFO overflow interrupt
        #-------------------------------------------------------------------------------------------
        GPIO.event_detected(GPIO_FIFO_OVERFLOW_INTERRUPT)
        mpu6050.flushFIFO()
        mpu6050.enableFIFOOverflowISR()

        #-------------------------------------------------------------------------------------------
        # Set up the various timing constants and stats.
        #-------------------------------------------------------------------------------------------
        motion_dt = 0.0
        fusion_dt = 0.0
        sampling_loops = 0
        motion_loops = 0
        garmin_loops = 0
        video_count = 0

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
        self.keep_looping = True
        while self.keep_looping:

            #---------------------------------------------------------------------------------------
            # Check on the number of batches already stashed in the FIFO, and if not enough, wait for
            # more or for camera motion batch info.
            #---------------------------------------------------------------------------------------
            nfb = mpu6050.numFIFOBatches()

            if nfb >= self.FIFO_OVERFLOW:
                logger.critical("ABORT: FIFO too full risking overflow: %d.", nfb)
                break

            if nfb < self.FIFO_PRIORITY:

                #-------------------------------------------------------------------------------
                # We have some spare time before we need to run the next motion processing; see
                # if there's any processing we can do.  First, have we already got a video frame
                # we can continue processing?
                #-------------------------------------------------------------------------------
                if vfp != None:
                    try:
                        vvx, vvy = vfp.process()
                    except TypeError as e:
                        #-----------------------------------------------------------------------
                        # An exception occurs when the processing is not complete.  Processing is
                        # broken into 4 phases, all but the final stage returning None.
                        #-----------------------------------------------------------------------
                        continue

                    vvx *= scale
                    vvy *= scale
                    camera_data_update = True
                    vfp = None
                    video_count += 1
                    continue

                #-------------------------------------------------------------------------------
                # Nowt else to do; sleep until either we timeout, or more data comes in from the
                # video process.
                #-------------------------------------------------------------------------------
                timeout = (self.FIFO_PRIORITY - nfb) / sampling_rate
                try:
                    read_out, write_out, exception_out = select.select(read_list, write_list, exception_list, timeout)
                except:
                    logger.critical("ERROR: select error")
                    break

                if len(read_out) != 0:

                    #---------------------------------------------------------------------------
                    # 1680 = (frame_width / macro_block_size + 1) * (frame_height / macro_block_size)
                    # * 4 bytes per macro-block
                    #---------------------------------------------------------------------------
                    frame = py_fifo.read(mbs_frame_bytes)
                    if len(frame) != mbs_frame_bytes:
                        logger.critical("ERROR: incomplete frame received")
                        break

                    #---------------------------------------------------------------------------
                    # Convert the data to byte, byte, ushort of x, y, sad structure and process them.
                    # The exception here happens when a macro-block is filled with zeros, indicating
                    # either a full reset or no movement, and hence no processing required,
                    #---------------------------------------------------------------------------
                    iframe = struct.unpack(format, frame)
                    try:
                        vfp = VideoFrameProcessor(iframe, aya - paya)
                        '''
                        '''
                        paya = aya
                        '''
                        '''
                    except ValueError as e:
                        #-----------------------------------------------------------------------
                        # First pass of the video frame shows no movement detected, and thus no
                        # further processing.
                        #-----------------------------------------------------------------------
                        video_count += 1
                        vfp = None

                #-------------------------------------------------------------------------------
                # We had free time, do we still?  Better check.
                #-------------------------------------------------------------------------------
                continue

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
#AB!            if GPIO.event_detected(GPIO_POWER_BROWN_OUT_INTERRUPT):
#AB!                logger.critical("BROWN-OUT, ABORT!")
#AB!                break

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

            '''
            qax *= g_scale
            qay *= g_scale
            qaz *= g_scale
            '''

            #---------------------------------------------------------------------------------------
            # Track the number of motion loops and sampling loops; any discrepancy between these are
            # the missed samples or sampling errors.
            #---------------------------------------------------------------------------------------
            motion_loops += 1
            sampling_loops += motion_dt * sampling_rate
            fusion_dt += motion_dt


            ################################## ANGLES PROCESSING ###################################


            #---------------------------------------------------------------------------------------
            # Euler angle fusion: Merge the 'integral' of the previous euler rotation rates with
            # the noisy accelermeter current values.  Keep yaw within +/- pi radians
            #---------------------------------------------------------------------------------------
            urp, urr, ury = Body2EulerRates(qry, qrx, qrz, pa, ra)
            pa += urp * motion_dt
            ra += urr * motion_dt
            ya += ury * motion_dt
            ya_sign = ya / abs(ya)
            ya = ya % (ya_sign * math.pi if abs(ya) < math.pi else -ya_sign * math.pi)

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
            aya_sign = aya / abs(aya)
            aya = aya % (aya_sign * math.pi if abs(aya) < math.pi else -aya_sign * math.pi)

            upa, ura = GetAbsoluteAngles(qax, qay, qaz)

            atau_fraction = atau / (atau + motion_dt)
            apa = atau_fraction * apa + (1 - atau_fraction) * upa
            ara = atau_fraction * ara + (1 - atau_fraction) * ura


            ############################### IMU VELOCITY / DISTANCE ################################


            #---------------------------------------------------------------------------------------
            # Rotate gravity to the new quadframe
            #---------------------------------------------------------------------------------------
            qgx, qgy, qgz = RotateE2Q(egx, egy, egz, pa, ra, ya)

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

            qvx_integral += qvx_increment
            qvy_integral += qvy_increment
            qvz_integral += qvz_increment

            #---------------------------------------------------------------------------------------
            # Integrate again the velocities to get distance. Rotate to earth frame for possible later
            # use for earth-frame sensor filtering.
            #---------------------------------------------------------------------------------------
            qdx_increment = qvx_integral * motion_dt
            qdy_increment = qvy_integral * motion_dt
            qdz_increment = qvz_integral * motion_dt


            ######################## ABSOLUTE DISTANCE / ORIENTATION SENSORS #######################


            #---------------------------------------------------------------------------------------
            # Read the compass to determine yaw and orientation wrt GPS.  Yaw from the compass will be fused
            # (using another complementary filter) with integrated gyro Z axis yaw to provide long term
            # stability.  For now though, we're just collecting the data for logging.
            #---------------------------------------------------------------------------------------
            if self.compass_installed:
                magx, magy, magz = mpu6050.readCompass()

            #=======================================================================================
            # Acquire vertical distance (height) first, prioritizing the best sensors,
            # Garmin LiDAR-Lite first - make sure it's not busy.
            #AB! Ideally, we also want to check that it had previously been busy acquiring a new height
            #AB! value, but it is no longer busy now.  However the interrupt just doesn't work
            #=======================================================================================
            if self.gll_installed and GPIO.input(GPIO_GARMIN_BUSY): #AB: and GPIO.event_detected(GPIO_GARMIN_BUSY):
                garmin_loops += 1
                g_distance, g_velocity = gll.read()

                edz_input = g_distance * tilt_ratio - eftoh
                evz_input = g_velocity * tilt_ratio

                #-------------------------------------------------------------------------------
                # Set the flags for vertical velocity and distance fusion
                #-------------------------------------------------------------------------------
                vvf = True
                vdf = True

            #---------------------------------------------------------------------------------------
            # Get the latest set of LEDDAR velocities if available
            #---------------------------------------------------------------------------------------
            elif self.leddar_installed and GPIO.input(GPIO_LEDDAR_DR_INTERRUPT):
                error = ""
                try:
                    l_dist, l_dd, l_dt = leddar.read()
                except IOError, e:
                    error = "IOError"
                except RuntimeError, e:
                    error = "RuntimeError"
                except ZeroDivisionError, e:
                    error = "ZeroDivisionError"
                except Exception, e:
                    error = "OtherError"
                else:
                    #-------------------------------------------------------------------------------
                    # Compensate for tilt
                    #-------------------------------------------------------------------------------
                    edz_input = l_dist * tilt_ratio - eftoh
                    evz_input = l_dd / l_dt * tilt_ratio

                    #-------------------------------------------------------------------------------
                    # Set the flags for vertical velocity and distance fusion
                    #-------------------------------------------------------------------------------
                    vvf = True
                    vdf = True

                finally:
                    if error != "":
                        l_dist = 0.0
                        l_dd = 0.0
                        l_dt = 0.0
                        logger.critical("############# LEDDAR ERROR (%s)#################" % error)
                        break

            #---------------------------------------------------------------------------------------
            # Check if there's a URF reading available, and if so, read and re-ping.
            # DUE TO THE FACT SRF02 DOES NOT SUPPORT THE REQUIRED 400kbps BAUDRATE,
            # THIS CODE IS HERE FOR REFERENCE ONLY AND IS INCOMPLETE.
            #---------------------------------------------------------------------------------------
            elif self.urf_installed and srf02.pingProcessed():
                urf_z, urf_dt = srf02.readProximity()
                srf02.pingProximity()

                #-----------------------------------------------------------------------------------
                # Compensate for tilt
                #-----------------------------------------------------------------------------------
                edz_input = urfz * tilt_ratio - eftoh

                #-----------------------------------------------------------------------------------
                # Set the flags for vertical distance fusion
                #-----------------------------------------------------------------------------------
                vdf = True

            #=======================================================================================
            # Acquire horizontal distance next, again with prioritization of accuracy
            #=======================================================================================

            #---------------------------------------------------------------------------------------
            # DUE TO THE FAULTY BEHAVIOUR OF THREE DIFFERENT PX4FLOWS TESTED, THIS CODE IS HERE FOR
            # REFERENCE ONLY AND IS INCOMPLETE.
            #---------------------------------------------------------------------------------------
            if self.px4flow_installed:
                error = ""
                try:
                    x_rps, y_rps = px4flow.get()
                except IOError, e:
                    error = "IOError"
                except RuntimeError, e:
                    error = "RuntimeError"
                except ZeroDivisionError, e:
                    error = "ZeroDivisionError"
                except Exception, e:
                    error = "OtherError"
                else:
                    #-------------------------------------------------------------------------------
                    # Convert the X and Y axis radians per second values to velocities using the height
                    # obtained ideally from the LEDDAR, but worst case, from integration.
                    # Note that a full functional PX4FLOW could provide distance and velocity increments
                    # and absolutes for X, Y and Z axes, but only one of the 3 I have does - the faulty
                    # URFs on two models prevent height being use.  What's below is what can be achieved
                    # with the minimum common support across all 3.
                    #-------------------------------------------------------------------------------
                    evx_input = 2 * (edz_input + eftoh) * math.tan(x_rps / 2)
                    evy_input = 2 * (edz_input + eftoh) * math.tan(y_rps / 2)

                    #-------------------------------------------------------------------------------
                    # Set the flags for horizontal velocity fusion
                    #-------------------------------------------------------------------------------
                    hvf = True

                finally:
                    if error != "":
                        l_dist = 0.0
                        l_dt = 0.0
                        l_vel = 0.0
                        logger.critical("############# PX4FLOW ERROR (%s)#################" % error)
                        break

            #---------------------------------------------------------------------------------------
            # If the camera is installed, and we have an absolute height measurement, get the horizontal
            # distance and velocity.  Note we always have height measurement, even if only from the
            # double integrated accelerometer.
            #---------------------------------------------------------------------------------------
            elif self.camera_installed and camera_data_update:

                #-----------------------------------------------------------------------------------
                # Take the increment of the scaled X and Y distance, and divide by the height to
                # get the absolute position, allowing for errors due to tilt.
                #-----------------------------------------------------------------------------------
                vvz = edz_input + eftoh
                edx_increment = vvz * (vvx + (apa - papa) / (2 * math.pi))
                edy_increment = vvz * (vvy - (ara - para) / (2 * math.pi))

                #-----------------------------------------------------------------------------------
                # Account for yaw - this is averaged out in the video macro-block vectors currently,
                # but in future, it may / should be possible to actually detect the incremental yaw
                # from the vectors too, and possibly incremental height changes too.
                #-----------------------------------------------------------------------------------
                edx_increment, edy_increment, __ = RotateQ2E(edx_increment, edy_increment, 0, 0, 0, aya)

                #-----------------------------------------------------------------------------------
                # Add the incremental distance to the total distance, and differentiate against time for
                # velocity
                #-----------------------------------------------------------------------------------
                edx_input += edx_increment
                edy_input += edy_increment

                evx_input = edx_increment * frame_rate
                evy_input = edy_increment * frame_rate

                papa = apa
                para = ara
                '''
                paya = aya
                '''

                #-----------------------------------------------------------------------------------
                # Set the flags for horizontal distance and velocity fusion
                #-----------------------------------------------------------------------------------
                hdf = True
                hvf = True

                camera_data_update = False


            ######################################## FUSION ########################################


            #---------------------------------------------------------------------------------------
            # Reorientate the values from the long term sensors only when we have a full set and overwrite
            # as the dominant factor to be updated incrementally by the accelerometer readings.
            #---------------------------------------------------------------------------------------
            if hdf and hvf and vvf and vdf:
                qvx_fuse, qvy_fuse, qvz_fuse = RotateE2Q(evx_input, evy_input, evz_input, pa, ra, ya)
                qdx_fuse, qdy_fuse, qdz_fuse = RotateE2Q(edx_input, edy_input, edz_input, pa, ra, ya)

                vtau_fraction = vtau / (vtau + fusion_dt)
                qvx_input = vtau_fraction * qvx_input + (1 - vtau_fraction) * qvx_fuse
                qvy_input = vtau_fraction * qvy_input + (1 - vtau_fraction) * qvy_fuse
                qvz_input = vtau_fraction * qvz_input + (1 - vtau_fraction) * qvz_fuse

                dtau_fraction = dtau / (dtau + fusion_dt)
                qdx_input = dtau_fraction * qdx_input + (1 - dtau_fraction) * qdx_fuse
                qdy_input = dtau_fraction * qdy_input + (1 - dtau_fraction) * qdy_fuse
                qdz_input = dtau_fraction * qdz_input + (1 - dtau_fraction) * qdz_fuse

                fusion_dt = 0.0

                #-----------------------------------------------------------------------------------
                # Clear the flags for horizontal and vertical distance and velocity fusion
                #-----------------------------------------------------------------------------------
                hdf = False
                hvf = False
                vdf = False
                vvf = False

            else:
                #-----------------------------------------------------------------------------------
                # Update the dominant sensors last reading with the intermediate IMU increments
                #-----------------------------------------------------------------------------------
                qvx_input += qvx_increment
                qvy_input += qvy_increment
                qvz_input += qvz_increment

                qdx_input += qdx_increment
                qdy_input += qdy_increment
                qdz_input += qdz_increment


            ########################### VELOCITY / DISTANCE PID TARGETS ############################


            #---------------------------------------------------------------------------------------
            # Check the flight plan for earth frame velocity and distance targets.
            #---------------------------------------------------------------------------------------
            if not ready_to_fly:
                if hover_speed >= hover_target:
                    hover_speed = hover_target
                    ready_to_fly = True
                    logger.critical("RTF @ %fs", motion_loops * samples_per_motion / sampling_rate)

                else:
                    hsf += (hover_target - base_pwm) * motion_dt / rtf_period
                    hover_speed = int(math.trunc(hsf))

            else:
                evx_target, evy_target, evz_target, edx_target, edy_target, edz_target = fp.getTargets(motion_dt)

            '''
            '''
            if edz_input > edz_target + 0.5:
                logger.critical("ABORT: Height breach! %f target, %f actual", edz_target, edz_input)
                break
            '''
            '''

            #---------------------------------------------------------------------------------------
            # Convert earth-frame distance targets to quadcopter frame.
            #---------------------------------------------------------------------------------------
            qdx_target, qdy_target, qdz_target = RotateE2Q(edx_target, edy_target, edz_target, pa, ra, ya)

            #---------------------------------------------------------------------------------------
            # Convert earth-frame velocity targets to quadcopter frame.
            #---------------------------------------------------------------------------------------
            qvx_target, qvy_target, qvz_target = RotateE2Q(evx_target, evy_target, evz_target, pa, ra, ya)


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
            ya_target = ya_target if (abs(evx_target) + abs(evy_target)) == 0 else math.atan2(evy_target, evx_target)

            '''
            '''
            #---------------------------------------------------------------------------------------
            # For safety reasons, limit the maximum target angle to 30 degrees.  Note yaw is deliberately
            # not included as it needs full rotation to track the direction of flight.
            #---------------------------------------------------------------------------------------
            MAX_ANGLE = 30 * math.pi / 180
            pa_target = pa_target if abs(pa_target) < MAX_ANGLE else (pa_target / abs(pa_target) * MAX_ANGLE)
            ra_target = ra_target if abs(ra_target) < MAX_ANGLE else (ra_target / abs(ra_target) * MAX_ANGLE)
            '''
            '''

            #======================================================================================
            # Angle PIDs
            #======================================================================================
            [p_out, i_out, d_out] = pa_pid.Compute(apa, pa_target, motion_dt)
            pr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ra_pid.Compute(ara, ra_target, motion_dt)
            rr_target = p_out + i_out + d_out

            [p_out, i_out, d_out] = ya_pid.Compute(aya, ya_target, motion_dt)
            yr_target = p_out + i_out + d_out

            '''
            '''
            #---------------------------------------------------------------------------------------
            # For safety reasons, limit the maximum target rotation rate to 180 degrees / second.
            #---------------------------------------------------------------------------------------
            MAX_RATE = math.pi # per-second
            pr_target = pr_target if abs(pr_target) < MAX_RATE else (pr_target / abs(pr_target) * MAX_RATE)
            rr_target = rr_target if abs(rr_target) < MAX_RATE else (rr_target / abs(rr_target) * MAX_RATE)
            yr_target = yr_target if abs(yr_target) < MAX_RATE else (yr_target / abs(yr_target) * MAX_RATE)
            '''
            '''

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
            vert_out = hover_speed - base_pwm + int(round(qaz_out))

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
                delta_spin = vert_out

                #-----------------------------------------------------------------------------------
                # For a left downwards roll, the x gyro goes negative, so the PID error is positive,
                # meaning PID output is positive, meaning this needs to be added to the left blades
                # and subtracted from the right.
                #-----------------------------------------------------------------------------------
                if esc.motor_location & self.MOTOR_LOCATION_RIGHT:
                    delta_spin -= rr_out
                else:
                    delta_spin += rr_out

                #-----------------------------------------------------------------------------------
                # For a forward downwards pitch, the y gyro goes positive The PID error is negative as a
                # result, meaning PID output is negative, meaning this needs to be subtracted from the
                # front blades and added to the back.
                #-----------------------------------------------------------------------------------
                if esc.motor_location & self.MOTOR_LOCATION_BACK:
                    delta_spin += pr_out
                else:
                    delta_spin -= pr_out

                #-----------------------------------------------------------------------------------
                # For CW yaw, the z gyro goes negative, so the PID error is postitive, meaning PID
                # output is positive, meaning this need to be added to the ACW (FL and BR) blades and
                # subtracted from the CW (FR & BL) blades.
                #-----------------------------------------------------------------------------------
                if esc.motor_rotation == self.MOTOR_ROTATION_CW:
                    delta_spin += yr_out
                else:
                    delta_spin -= yr_out

                #-----------------------------------------------------------------------------------
                # Apply the blended outputs to the esc PWM signal
                #-----------------------------------------------------------------------------------
                esc.update(delta_spin)

            #---------------------------------------------------------------------------------------
            # Diagnostic log - every motion loop
            #---------------------------------------------------------------------------------------
            if diagnostics:
                temp = mpu6050.readTemperature()
                logger.warning("%f, %f, %d, " % (sampling_loops / sampling_rate, motion_dt, sampling_loops) +
                               "%f, " % (temp / 333.86 + 21) +
                               "%f, %f, %f, " % (magx, magy, magz) +
                               "%f, %f, %f, " % (edx_input, edy_input, edz_input) +
                               "%f, %f, %f, " % (evx_input, evy_input, evz_input) +
                               "%f, %f, %f, " % (edx_target, edy_target, edz_target) +
                               "%f, %f, %f, " % (evx_target, evy_target, evz_target) +
                               "%f, %f, %f, " % (qrx, qry, qrz) +
                               "%f, %f, %f, " % (qax, qay, qaz) +
                               "%f, %f, %f, " % (qgx, qgy, qgz) +
                               "%f, %f, %f, " % (math.degrees(pa), math.degrees(ra), math.degrees(ya)) +
                               "%f, %f, %f, %f, %f, %f, %f, %f, %d, " % (qdx_input, qdx_target, qvx_input, qvx_target, math.degrees(apa), math.degrees(pa_target), math.degrees(qry), math.degrees(pr_target), pr_out) +
                               "%f, %f, %f, %f, %f, %f, %f, %f, %d, " % (qdy_input, qdy_target, qvy_input, qvy_target, math.degrees(ara), math.degrees(ra_target), math.degrees(qrx), math.degrees(rr_target), rr_out) +
                               "%f, %f, %f, %f, %d, " % (qdz_input, qdz_target, qvz_input, qvz_target, qaz_out) +
                               "%f, %f, %f, %f, %d, " % (math.degrees(aya), math.degrees(ya_target), math.degrees(qrz), math.degrees(yr_target), yr_out) +
                               "%d, %d, %d, %d, " % (self.esc_list[0].pulse_width, self.esc_list[1].pulse_width, self.esc_list[2].pulse_width, self.esc_list[3].pulse_width))

        logger.critical("Flight time %f", time.time() - start_flight)
        logger.critical("Sampling_loops %d", sampling_loops)
        logger.critical("Motion processing loops %d", motion_loops)
        logger.critical("Video frame rate %f", video_count / sampling_loops * sampling_rate)
        logger.critical("LiDAR processing loops %d", garmin_loops)

        temp = mpu6050.readTemperature()
        logger.warning("IMU core temp: %f", temp / 333.86 + 21.0)
        max_az = mpu6050.getStats() / 8 #AB! +/-4g
        logger.warning("Max Z acceleration: %f", max_az)

        #-------------------------------------------------------------------------------------------
        # Stop the PWM and FIFO overflow interrupt between flights
        #-------------------------------------------------------------------------------------------
        mpu6050.disableFIFOOverflowISR()
        for esc in self.esc_list:
            esc.set(0)

        #-------------------------------------------------------------------------------------------
        # Stop the camera motion processing
        #-------------------------------------------------------------------------------------------
        if self.camera_installed:
            video.send_signal(signal.SIGINT)
            py_fifo.close()
            os.unlink("/dev/shm/motion_stream")


    ################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ################################################################################################
    def shutdown(self):

        #-------------------------------------------------------------------------------------------
        # Stop the signal handler
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        #-------------------------------------------------------------------------------------------
        # Stop the blades spinning
        #-------------------------------------------------------------------------------------------
        for esc in self.esc_list:
            esc.set(0)

        #-------------------------------------------------------------------------------------------
        # Copy logs from /dev/shm (shared / virtual memory) to the Logs directory.
        #-------------------------------------------------------------------------------------------
        now = datetime.now()
        now_string = now.strftime("%y%m%d-%H:%M:%S")
        log_file_name = "qcstats.csv"
        shutil.move("/dev/shm/qclogs", log_file_name)

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
    # Signal handler for Ctrl-C => abort cleanly
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
    #
    # Interrupt Service Routine for LEDDAR data ready - RETIRED, JUST POLL NOW
    #
    ####################################################################################################
    def LEDDARDataReadyISR(self, pin):
        print "LEDDAR ISR"
        self.leddar_dr = True


####################################################################################################
# If we've been called directly, this is the spawned video process for gathering and FIFOing macro-block
# frames
####################################################################################################
if __name__ == '__main__':
    if len(sys.argv) == 4:
        frame_width = int(sys.argv[1])
        frame_height = int(sys.argv[2])
        frame_rate = int(sys.argv[3])

        RecordVideo(frame_width, frame_height, frame_rate)
