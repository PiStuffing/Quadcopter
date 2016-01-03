#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2016 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
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
import struct
import logging
import csv
from RPIO import PWM
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library
import random

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
class MPU6050 :
    i2c = None

    # Registers/etc.
    __MPU6050_RA_MAG_WIA = 0x00
    __MPU6050_RA_MAG_INFO = 0x01
    __MPU6050_RA_MAG_ST1 = 0x02          # 0x01 = Data ready for one shot polling
    __MPU6050_RA_MAG_HXL = 0x03          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HXH = 0x04          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HYL = 0x05          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HYH = 0x06          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HZL = 0x07          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HZH = 0x08          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_ST2 = 0x09          # Unlatch register for next sample - read 7 bytes each time
    __MPU6050_RA_MAG_CNTL1 = 0x0A        # 0x10 = 16-bit, 0x01 = one-shot polling, 0x02 = continuous
    __MPU6050_RA_MAG_CNTL2 = 0x0B        # Reset
    __MPU6050_RA_MAG_ASTC = 0x0C         # Self test
    __MPU6050_RA_MAG_TS1 = 0x0D          # Shipment test register
    __MPU6050_RA_MAG_TS2 = 0x0E          # Shipment test register
    __MPU6050_RA_MAG_I2CDIS = 0x0F       # I2C disable
    __MPU6050_RA_MAG_ASAX = 0x10         # Factory settings
    __MPU6050_RA_MAG_ASAY = 0x11         # Factory settings
    __MPU6050_RA_MAG_ASAZ = 0x12         # Factory settings
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

    __SCALE_GYRO = 500.0 * math.pi / (65536 * 180)
    __SCALE_ACCEL = 4.0 / 65536 #AB! Try 4 if I shows no 2g hits

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address
        self.result_array = array('h', [0, 0, 0, 0, 0, 0, 0])
        self.ambient = 0

        self.num_i2c_errs = 0
        self.num_data_errs = 0
        self.num_2g_hits = 0

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
        # 0x00 =  250Hz @ 8kHz sampling
        # 0x01 =  184Hz
        # 0x02 =   92Hz
        # 0x03 =   41Hz
        # 0x04 =   20Hz
        # 0x05 =   10Hz
        # 0x06 =    5Hz
        # 0x07 = 3600Hz @ 8kHz
        #
        # 0x40 FIFO overflow does not overwrite full FIFO contents
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
        # See SCALE_GYRO for converstion from raw data to units of radians per second
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
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x00)  #AB! Try 0x00 if 0x08 shows no 2g hits
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set up interrupts as disabled.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable interrupts
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enable FIFO mode, and reset it
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x44)

        #-------------------------------------------------------------------------------------------
        # Accelerometer / gyro goes into FIFO
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x78)

        #-------------------------------------------------------------------------------------------
        # Set up the magnetometer
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_MAG_CNTL1, 0x12)  # 0x10 = 16-bit, 0x01 = one-shot, 0x02 = continuous

        #-------------------------------------------------------------------------------------------
        # Read ambient temperature
        #-------------------------------------------------------------------------------------------
        temp = self.readTemperature()
        logger.warning("IMU core temp: %f", temp / 333.86 + 21.0)

    def readTemperature(self):
        temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
        return temp

    def readCompass(self):
        compass_data = self.i2c.readList(self.__MPU6050_RA_MAG_HXL, 7)

        #---------------------------------------------------------------------------------------
        # Convert the array of 6 bytes to 3 shorts - 7th byte kicks off another read
        #---------------------------------------------------------------------------------------
        for index in range(0, 6, 2):
            if (compass_data[index + 1] > 127):
                compass_data[index + 1] -= 256
            self.result_array[int(index / 2)] = (compass_data[index + 1] << 8) + compass_data[index]

        [mgx, mgy, mgz] = self.result_array
        return mgx, mgy, mgz

    def readFIFO(self):
        #------------------------------------------------------------------------------------------------
        # Read n x 12 bytes of FIFO data (n * smBus.readS16()), averaging, and return the averaged values and
        # inferred time
        #------------------------------------------------------------------------------------------------
        ax = 0.0
        ay = 0.0
        az = 0.0
        gx = 0.0
        gy = 0.0
        gz = 0.0

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)
        fifo_batches = int(fifo_bytes / 12)  # This rounds down

        batch_size = 6   # ax, ay, az, gx, gy, gz
        for ii in range(fifo_batches):
            sensor_data = []
            for jj in range(batch_size):
                hibyte = self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)
                if (hibyte > 127):
                    hibyte -= 256
                lobyte = self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)
                sensor_data.append((hibyte << 8) + lobyte)


            ax += sensor_data[0]
            ay += sensor_data[1]
            az += sensor_data[2]
            gx += sensor_data[3]
            gy += sensor_data[4]
            gz += sensor_data[5]

        ax /= fifo_batches
        ay /= fifo_batches
        az /= fifo_batches
        gx /= fifo_batches
        gy /= fifo_batches
        gz /= fifo_batches

        return ax, ay, az, gx, gy, gz, fifo_batches / sampling_rate

    def resetFIFO(self):
        #-------------------------------------------------------------------------------------------
        # Disable the feed to the FIFO and reset the FIFO itself.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x44)

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = (ax - self.ax_offset) * self.__SCALE_ACCEL
        qay = (ay - self.ay_offset) * self.__SCALE_ACCEL
        qaz = (az - self.az_offset) * self.__SCALE_ACCEL

        qrx = (gx - self.gx_offset) * self.__SCALE_GYRO
        qry = (gy - self.gy_offset) * self.__SCALE_GYRO
        qrz = (gz - self.gz_offset) * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz

    def calibrateGyros(self):
        gx_offset = 0.0
        gy_offset = 0.0
        gz_offset = 0.0

        self.resetFIFO()
        time.sleep(42 / sampling_rate) # 42 samples just fits into the FIFO (512 / 12)
        ax, ay, az, self.gx_offset, self.gy_offset, self.gz_offset, dt = self.readFIFO()

    def calibrate0g(self):
        ax_offset = 0
        ay_offset = 0
        az_offset = 0
        offs_rc = True

        #-------------------------------------------------------------------------------------------
        # Open the ofset file for this run
        #-------------------------------------------------------------------------------------------
        try:
            with open('0goffsets', 'wb') as offs_file:
                raw_input("Rest me on my props and press enter.")
                self.resetFIFO()
                time.sleep(42 / sampling_rate) # 42 samples just fits into the FIFO (512 / 12)
                ax, ay, az, gx, gy, gz, dt = self.readFIFO()
                offs_file.write("%f %f %f" % (ax, ay, az))

        except EnvironmentError:
            offs_rc = False
        return offs_rc


    def load0gCalibration(self):
        offs_rc = True
        try:
            with open('0goffsets', 'rb') as offs_file:
                for line in offs_file:
                    ax_offset, ay_offset, az_offset = line.split()
            self.ax_offset = float(ax_offset)
            self.ay_offset = float(ay_offset)
            self.az_offset = 0.0 # float(az_offset)
        except EnvironmentError:
            offs_rc = False
        logger.warning("0g Offsets:, %f, %f, %f", self.ax_offset, self.ay_offset, self.az_offset)
        return offs_rc

    def getMisses(self):
        self.num_i2c_errs += self.i2c.getMisses()
        return (self.num_data_errs, self.num_i2c_errs, self.num_2g_hits)

####################################################################################################
#
#  Barometer / Altimeter class for reading air pressure / height
#
####################################################################################################
class MS5611 :
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
        self.result_array = array('h', [0, 0, 0, 0, 0, 0, 0])

        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.writeByte(self.__MS5611_RESET)
        time.sleep(0.1)

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


    def Compute(self, input, target, dt):
        #-------------------------------------------------------------------------------------------
        # Error is what the PID alogithm acts upon to derive the output
        #-------------------------------------------------------------------------------------------
        error = target - input

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
        # Initialize the RPIO DMa PWM for this ESC in microseconds - 1ms - 2ms of
        # pulse widths with 3ms carrier.
        #-------------------------------------------------------------------------------------------
        self.min_pulse_width = 1000
        self.max_pulse_width = 2000

        #-------------------------------------------------------------------------------------------
        # The PWM pulse range required by this ESC
        #-------------------------------------------------------------------------------------------
        self.pulse_width = self.min_pulse_width

        #-------------------------------------------------------------------------------------------
        # Name - for logging purposes only
        #-------------------------------------------------------------------------------------------
        self.name = name

        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        #-------------------------------------------------------------------------------------------
        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.pulse_width)


    def update(self, spin_rate):
        self.pulse_width = int(self.min_pulse_width + spin_rate)

        if self.pulse_width < self.min_pulse_width:
            self.pulse_width = self.min_pulse_width
        if self.pulse_width > self.max_pulse_width:
            self.pulse_width = self.max_pulse_width

        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.pulse_width)



####################################################################################################
#
# Angles required to convert between Earth (interal reference frame) and Quadcopter (body # reference
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
# Convert a vector to earth-frame coordingates from quadcopter-frame coordinates.
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
# GPIO pins initialization for MPU6050 interrupt, sounder and hardware PWM
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
# GPIO pins cleanup for MPU6050 interrupt, sounder and hardware PWM
#
####################################################################################################
def PWMTerm():
    PWM.cleanup()


####################################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
####################################################################################################
def CheckCLI(argv):
    cli_fly = False
    cli_video = False
    cli_hover_target = 0

    #-----------------------------------------------------------------------------------------------
    # Other configuration defaults
    #-----------------------------------------------------------------------------------------------
    cli_test_case = 0
    cli_diagnostics = False
    cli_rtf_period = 1.5
    cli_tau = 5
    cli_calibrate_0g = False
    cli_flight_plan = ''

    hover_target_defaulted = True

    if i_am_phoebe:
        #-------------------------------------------------------------------------------------------
        # Phoebe's PID configuration due to using her ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 430

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 360.0
        cli_vvi_gain = 180.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for horizontal velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_hvp_gain = 1.2
        cli_hvi_gain = 0.1
        cli_hvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 105.0
        cli_pri_gain = 9.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 105.0
        cli_rri_gain = 9.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 60.0
        cli_yri_gain = 30.0
        cli_yrd_gain = 0.0

    elif i_am_chloe:
        #-------------------------------------------------------------------------------------------
        # Chloe's PID configuration due to using her frame / ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 450

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 360.0
        cli_vvi_gain = 180.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for horizontal velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_hvp_gain = 1.2
        cli_hvi_gain = 0.1
        cli_hvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 100.0
        cli_pri_gain = 9.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 100.0
        cli_rri_gain = 9.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 50.0
        cli_yri_gain = 25.0
        cli_yrd_gain = 0.0

    elif i_am_zoe:
        #-------------------------------------------------------------------------------------------
        # Phoebe's PID configuration due to using her ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 380

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 400.0
        cli_vvi_gain = 180.0
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for horizontal velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_hvp_gain = 1.5
        cli_hvi_gain = 0.1
        cli_hvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 90.0
        cli_pri_gain = 9.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 90.0
        cli_rri_gain = 9.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 80.0
        cli_yri_gain = 8.0
        cli_yrd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Right, let's get on with reading the command line and checking consistency
    #-----------------------------------------------------------------------------------------------
    try:
        opts, args = getopt.getopt(argv,'df:gvh:r:', ['tc=', 'tau=', 'vvp=', 'vvi=', 'vvd=', 'hvp=', 'hvi=', 'hvd=', 'prp=', 'pri=', 'prd=', 'rrp=', 'rri=', 'rrd=', 'tau=', 'yrp=', 'yri=', 'yrd='])
    except getopt.GetoptError:
        logger.critical('Must specify one of -f or -g or --tc')
        logger.critical('  qcpi.py')
        logger.critical('  -f set the flight plan CSV file')
        logger.critical('  -h set the hover speed for manual testing')
        logger.critical('  -d enable diagnostics')
        logger.critical('  -g calibrate X, Y axis 0g')
        logger.critical('  -v video the flight')
        logger.critical('  -r ??  set the ready-to-fly period')
        logger.critical('  --tc   select which testcase to run')
        logger.critical('  --tau ??  set the angle CF -3dB point')
        logger.critical('  --vvp  set vertical speed PID P gain')
        logger.critical('  --vvi  set vertical speed PID P gain')
        logger.critical('  --vvd  set vertical speed PID P gain')
        logger.critical('  --hvp  set horizontal speed PID P gain')
        logger.critical('  --hvi  set horizontal speed PID I gain')
        logger.critical('  --hvd  set horizontal speed PID D gain')
        logger.critical('  --prp  set pitch rotation rate PID P gain')
        logger.critical('  --pri  set pitch rotation rate PID I gain')
        logger.critical('  --prd  set pitch rotation rate PID D gain')
        logger.critical('  --rrp  set roll rotation rate PID P gain')
        logger.critical('  --rri  set roll rotation rate PID I gain')
        logger.critical('  --rrd  set roll rotation rate PID D gain')
        logger.critical('  --yrp  set yaw rotation rate PID P gain')
        logger.critical('  --yri  set yaw rotation rate PID I gain')
        logger.critical('  --yrd  set yaw rotation rate PID D gain')
        raise ValueError("Invalid command line")

    for opt, arg in opts:
        if opt == '-f':
            cli_fly = True
            cli_flight_plan = arg

        elif opt in '-h':
            cli_hover_target = int(arg)
            hover_target_defaulted = False

        elif opt in '-v':
            cli_video = True

        elif opt in '-d':
            cli_diagnostics = True

        elif opt in '-g':
            cli_calibrate_0g = True

        elif opt in '-r':
            cli_rtf_period = float(arg)

        elif opt in '--tc':
            cli_test_case = int(arg)

        elif opt in '--tau':
            cli_tau = float(arg)

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

        elif opt in '--yrp':
            cli_yrp_gain = float(arg)
            yrp_set = True

        elif opt in '--yri':
            cli_yri_gain = float(arg)
            yri_set = True

        elif opt in '--yrd':
            cli_yrd_gain = float(arg)
            yrd_set = True

    if not cli_fly and cli_test_case == 0 and not cli_calibrate_0g:
        raise ValueError('Must specify one of -f or --tc')

    elif cli_hover_target < 0 or cli_hover_target > 1000:
        raise ValueError('Hover speed must lie in the following range: 0 <= hover speed <= 1000')

    elif cli_test_case == 0 and cli_fly:
        print 'Pre-flight checks passed, enjoy your flight, sir!'

    elif cli_test_case == 0 and cli_calibrate_0g:
        print 'Proceeding with 0g calibration'

    elif cli_test_case != 1 and cli_test_case != 2:
        raise ValueError('Only 1 or 2 are valid testcases')

    elif cli_test_case == 1 and hover_target_defaulted:
        raise ValueError('You must choose a specific hover speed (-h) for test case 1 - try 200')

    return cli_fly, cli_flight_plan, cli_calibrate_0g, cli_hover_target, cli_video, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_prp_gain, cli_pri_gain, cli_prd_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_yrp_gain, cli_yri_gain, cli_yrd_gain, cli_test_case, cli_rtf_period, cli_tau, cli_diagnostics


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
            print "%s" % self.fp_name[fp_index]
            self.fp_prev_index = fp_index

        return self.fp_evx_target[fp_index], self.fp_evy_target[fp_index], self.fp_evz_target[fp_index]


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
    shoot_video = False

    #===============================================================================================
    # One-off initialization
    #===============================================================================================
    def __init__(self):

        #-------------------------------------------------------------------------------------------
        # Who am I?
        #-------------------------------------------------------------------------------------------
        global i_am_phoebe
        global i_am_chloe
        global i_am_zoe
        i_am_phoebe = False
        i_am_chloe = False
        i_am_zoe = False

        my_name = os.uname()[1]
        if my_name == "phoebe.local":
            print "Hi, I'm Phoebe. Nice to meet you!"
            i_am_phoebe = True
        elif my_name == "chloe.local" or my_name == "chloe":
            print "Hi, I'm Chloe.  Nice to meet you!"
            i_am_chloe = True
        elif my_name == "zoe.local":
            print "Hi, I'm Zoe.  Nice to meet you!"
            i_am_zoe = True
        else:
            print "Sorry, I'm not qualified to fly this quadcopter."
            return

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
        logger.warning("%s is flying.", "Phoebe" if i_am_phoebe else "Chloe" if i_am_chloe else "Zoe")

        #-------------------------------------------------------------------------------------------
        # Set the BCM input assigned to sensor data ready interrupt
        #-------------------------------------------------------------------------------------------
        RPIO_DATA_READY_INTERRUPT = 22

        #-------------------------------------------------------------------------------------------
        # Set up the ESC to GPIO pin and location mappings and assign to each ESC
        #-------------------------------------------------------------------------------------------
        alpf = 2
        glpf = 1
        if i_am_phoebe:
            ESC_BCM_BL = 5
            ESC_BCM_FL = 27
            ESC_BCM_FR = 17
            ESC_BCM_BR = 19
        elif i_am_chloe:
            ESC_BCM_BL = 5
            ESC_BCM_FL = 27
            ESC_BCM_FR = 17
            ESC_BCM_BR = 19
        elif i_am_zoe:
            ESC_BCM_BL = 26
            ESC_BCM_FL = 27
            ESC_BCM_FR = 17
            ESC_BCM_BR = 19
            alpf = 3

        pin_list = [ESC_BCM_FL, ESC_BCM_FR, ESC_BCM_BL, ESC_BCM_BR]
        location_list = [self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_LEFT, self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_RIGHT, self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_LEFT, self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_RIGHT]
        rotation_list = [self.MOTOR_ROTATION_ACW, self.MOTOR_ROTATION_CW, self.MOTOR_ROTATION_CW, self.MOTOR_ROTATION_ACW]
        name_list = ['front left', 'front right', 'back left', 'back right']

        #-------------------------------------------------------------------------------------------
        # Enable RPIO for ESC PWM.  This must be set up prior to adding the SignalHandler below or it
        # will override what we set thus killing the "Kill Switch"..
        #-------------------------------------------------------------------------------------------
        PWMInit()

        #-------------------------------------------------------------------------------------------
        # Set the signal handler here so the core processing loop can be stopped (or not started) by
        # Ctrl-C.
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, self.shutdownSignalHandler)

        #-------------------------------------------------------------------------------------------
        # Prime the ESCs with the default 0 spin rotors to stop their whining!
        #-------------------------------------------------------------------------------------------
        self.esc_list = []
        for esc_index in range(0, 4):
            esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
            self.esc_list.append(esc)

        #===============================================================================================
        # adc_frequency      - the sampling rate of the ADC
        # sampling_rate      - the data sampling rate and thus data ready interrupt rate
        #                      motion processing.
        #===============================================================================================
        global adc_frequency
        global sampling_rate
        if glpf == 0:
            adc_frequency = 8000
        else:
            adc_frequency = 1000

        sampling_rate = 333

        #-------------------------------------------------------------------------------------------
        # Initialize the gyroscope / accelerometer I2C object
        #-------------------------------------------------------------------------------------------
        global mpu6050
        mpu6050 = MPU6050(0x68, alpf, glpf)

        #-------------------------------------------------------------------------------------------
        # Initialize the barometer / altimeter I2C object
        #-------------------------------------------------------------------------------------------
        ms5611 = MS5611(0x77)

    #===============================================================================================
    # Keyboard input between flights for CLI update etc
    #===============================================================================================
    def go(self):
        while True:
            print "============================================"
            cli_argv = raw_input("Wassup? ")
            print "============================================"
            if len(cli_argv) != '' and (cli_argv == 'exit' or cli_argv == 'quit'):
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
        print "Just checking a few details.  Gimme a seconds or two..."

        #-------------------------------------------------------------------------------------------
        # Check the command line for calibration or flight parameters
        #-------------------------------------------------------------------------------------------
        try:
            flying, flight_plan, calibrate_0g, hover_target, self.shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, rtf_period, tau, diagnostics = CheckCLI(self.argv)
        except ValueError, err:
            print "Command line error: %s" % err
            return

        logger.warning("fly = %s, flight plan = %s, calibrate_0g = %d, hover_target = %d, shoot_video = %s, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, prp_gain = %f, pri_gain = %f, prd_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, yrp_gain = %f, yri_gain = %f, yrd_gain = %f, test_case = %d, rtf_period = %f, tau = %f, diagnostics = %s",
                flying, flight_plan, calibrate_0g, hover_target, self.shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, rtf_period, tau, diagnostics)

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

        #===========================================================================================
        # START TESTCASE 1 CODE: spin up each blade individually for 5s each and check they all turn
        #                        the right way.  At the same time, log X, Y and Z accelerometer readings
        #                        to measure noise from the motors and props due to possible prop and motor
        #                        damage.
        #===========================================================================================
        if test_case == 1:
            print "TESTCASE 1: Check props are spinning as expected"
            elapsed_time = 0
            for esc in self.esc_list:
                print "%s prop should rotate %s." % (esc.name, "anti-clockwise" if esc.motor_rotation == self.MOTOR_ROTATION_ACW else "clockwise")
                for count in range(0, hover_target, 10):
                    #-------------------------------------------------------------------------------
                    # Spin up to user determined (-h) hover speeds ~200
                    #-------------------------------------------------------------------------------
                    esc.update(count)
                    time.sleep(0.01)

                #-----------------------------------------------------------------------------------
                # The prop is now up to the configured spin rate.  Start a 5s loop based upon the
                # sampling_rate logging the noise from the accelerometer.
                #-----------------------------------------------------------------------------------
                time.sleep(5)
                esc.update(0)
            return
        #===========================================================================================
        # END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the
        #                      right way
        #===========================================================================================

        #===========================================================================================
        # OK, we're in flight mode, better get on with it
        #===========================================================================================
        evx_target = 0.0
        evy_target = 0.0
        evz_target = 0.0

        qvx_input = 0.0
        qvy_input = 0.0
        qvz_input = 0.0

        qvx_diags = "0.0, 0.0, 0.0"
        qvy_diags = "0.0, 0.0, 0.0"
        qvz_diags = "0.0, 0.0, 0.0"
        ya_diags = "0.0, 0.0, 0.0"
        pr_diags = "0.0, 0.0, 0.0"
        rr_diags = "0.0, 0.0, 0.0"
        yr_diags = "0.0, 0.0, 0.0"

        hover_speed = 0
        hsf = 0.0
        ready_to_fly = False

        #-------------------------------------------------------------------------------------------
        # Register the flight plan with the authorities
        #-------------------------------------------------------------------------------------------
        try:
            fp = FlightPlan(self, flight_plan)
        except Exception, err:
            print "%s error: %s" % (flight_plan, err)
            return

        #-------------------------------------------------------------------------------------------
        # Set up the global constants
        # - gravity in meters per second squared
        # - accelerometer in g's
        # - gyroscope in radians per second
        #-------------------------------------------------------------------------------------------
        GRAV_ACCEL = 9.80665

        #-------------------------------------------------------------------------------------------
        # Calibrate gyros - this is a one-off
        #-------------------------------------------------------------------------------------------
        mpu6050.calibrateGyros()

        #-------------------------------------------------------------------------------------------
        # Enable the FIFO, collect a FIFO full of samples and calculate startup gravity and angles
        #-------------------------------------------------------------------------------------------
        pa = 0.0
        ra = 0.0
        ya = 0.0

        mpu6050.resetFIFO()
        time.sleep(42 / sampling_rate) # 42 samples just fits into the FIFO (512 / 12)

        #-------------------------------------------------------------------------------------------
        # Now get the batch over averaged data from the FIFO.
        #-------------------------------------------------------------------------------------------
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO()

        #-------------------------------------------------------------------------------------------
        # Sort out units and calibration for the incoming data
        #-------------------------------------------------------------------------------------------
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)

        egx = 0.0
        egy = 0.0
        egz = math.pow(math.pow(qax, 2) + math.pow(qay, 2) + math.pow(qaz, 2), 0.5)

        pa, ra = GetRotationAngles(qax, qay, qaz)

        #-------------------------------------------------------------------------------------------
        # Log the critical parameters from this warm-up: the take-off surface tilt, and gravity.
        # Note that some of the variables used above are used in the main processing loop.  Messing
        # with the above code can have very unexpected effects in flight.
        #-------------------------------------------------------------------------------------------
        logger.warning("pitch %f, roll %f", math.degrees(pa), math.degrees(ra))
        logger.warning("egx %f, egy %f, egz %f", egx, egy, egz)

        #-------------------------------------------------------------------------------------------
        # Start up the video camera if required - this runs from take-off through to shutdown
        # automatically.  Run it in its own process group so that Ctrl-C for QC doesn't get through and
        # stop the video
        #-------------------------------------------------------------------------------------------
        def Daemonize():
            os.setpgrp()

        if self.shoot_video:
            now = datetime.now()
            now_string = now.strftime("%y%m%d-%H:%M:%S")
            video = subprocess.Popen(["raspivid", "-rot", "180", "-w", "1280", "-h", "720", "-o", "/home/pi/Videos/qcvid_" + now_string + ".h264", "-n", "-t", "0", "-fps", "30", "-b", "5000000"], preexec_fn =  Daemonize)

        #===============================================================================================
        # Tuning: Set up the PID gains - some are hard coded mathematical approximations, some come
        # from the CLI parameters to allow for tuning  - 7 in all
        # - Quad X axis speed speed
        # - Quad Y axis speed speed
        # - Quad Z axis speed speed
        # - Pitch rotation rate
        # - Roll Rotation rate
        # - Yaw angle
        # = Yaw Rotation rate
        #===============================================================================================

        #-----------------------------------------------------------------------------------------------
        # The quad X axis speed controls forward / backward speed
        #-----------------------------------------------------------------------------------------------
        PID_QVX_P_GAIN = hvp_gain
        PID_QVX_I_GAIN = hvi_gain
        PID_QVX_D_GAIN = hvd_gain

        #-----------------------------------------------------------------------------------------------
        # The quad Y axis speed controls left / right speed
        #-----------------------------------------------------------------------------------------------
        PID_QVY_P_GAIN = hvp_gain
        PID_QVY_I_GAIN = hvi_gain
        PID_QVY_D_GAIN = hvd_gain

        #-----------------------------------------------------------------------------------------------
        # The quad Z axis speed controls rise / fall speed
        #-----------------------------------------------------------------------------------------------
        PID_QVZ_P_GAIN = vvp_gain
        PID_QVZ_I_GAIN = vvi_gain
        PID_QVZ_D_GAIN = vvd_gain

        #-----------------------------------------------------------------------------------------------
        # The pitch rate PID controls stable rotation rate around the Y-axis
        #-----------------------------------------------------------------------------------------------
        PID_PR_P_GAIN = prp_gain
        PID_PR_I_GAIN = pri_gain
        PID_PR_D_GAIN = prd_gain

        #-----------------------------------------------------------------------------------------------
        # The roll rate PID controls stable rotation rate around the X-axis
        #-----------------------------------------------------------------------------------------------
        PID_RR_P_GAIN = rrp_gain
        PID_RR_I_GAIN = rri_gain
        PID_RR_D_GAIN = rrd_gain

        #-------------------------------------------------------------------------------------------
        # The yaw angle PID controls stable angles around the Z-axis
        #-------------------------------------------------------------------------------------------
        PID_YA_P_GAIN = 6.0 # yap_gain
        PID_YA_I_GAIN = 3.0 # yai_gain
        PID_YA_D_GAIN = 1.0 # yad_gain

        #-------------------------------------------------------------------------------------------
        # The yaw rate PID controls stable rotation speed around the Z-axis
        #-------------------------------------------------------------------------------------------
        PID_YR_P_GAIN = yrp_gain
        PID_YR_I_GAIN = yri_gain
        PID_YR_D_GAIN = yrd_gain

        (data_errors, i2c_errors, num_2g_hits) = mpu6050.getMisses()
        logger.warning("%d data errors; %d i2c errors; %d 2g hits", data_errors, i2c_errors, num_2g_hits)
        print "Thunderbirds are go!"

        #-------------------------------------------------------------------------------------------
        # Diagnostic log header
        #-------------------------------------------------------------------------------------------
        if diagnostics:
            logger.warning('time, dt, loops, sleep, temp, qrx, qry, qrz, qax, qay, qaz, efrgv_x, efrgv_y, efrgv_z, qfrgv_x, qfrgv_y, qfrgv_z, qvx_input, qvy_input, qvz_input, pitch, roll, yaw, evx_target, qvx_target, qxp, qxi, qxd, pr_target, prp, pri, prd, pr_out, evy_yarget, qvy_target, qyp, qyi, qyd, rr_target, rrp, rri, rrd, rr_out, evz_target, qvz_target, qzp, qzi, qzd, qvz_out, yr_target, yrp, yri, yrd, yr_out, FL spin, FR spin, BL spin, BR spin')

        #===========================================================================================
        # Initialize critical timing immediately before starting the PIDs.  This is done by reading the
        # sensors, and that also gives us a starting position of the rolling average from.
        #===========================================================================================

        #-------------------------------------------------------------------------------------------
        # Start the X, Y (horizontal) and Z (vertical) velocity PIDs
        #-------------------------------------------------------------------------------------------
        qvx_pid = PID(PID_QVX_P_GAIN, PID_QVX_I_GAIN, PID_QVX_D_GAIN)
        qvy_pid = PID(PID_QVY_P_GAIN, PID_QVY_I_GAIN, PID_QVY_D_GAIN)
        qvz_pid = PID(PID_QVZ_P_GAIN, PID_QVZ_I_GAIN, PID_QVZ_D_GAIN)

        #-------------------------------------------------------------------------------------------
        # Start the yaw angle PID
        #-------------------------------------------------------------------------------------------
        ya_pid = PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN)

        #-------------------------------------------------------------------------------------------
        # Start the pitch, roll and yaw rate PIDs
        #-------------------------------------------------------------------------------------------
        pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN)
        rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN)
        yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN)

        #===========================================================================================
        #
        # Motion and PID processing loop
        #
        # qa? = quad frame acceleration
        # qg? = quad frame gravity
        # qr? = quad frame rotation
        # ea? = earth frame acceleration
        # eg? = earth frame gravity
        # ua? = euler angles between reference frames
        # ur? = euler rotation between frames
        #
        #===========================================================================================
        esc_period = 0.01
        sleep_time = esc_period
        sampling_loops = 0
        motion_loops = 0
        self.keep_looping = True
        start_flight = time.time()
        start_motion = start_flight

        #-------------------------------------------------------------------------------------------
        # Enable the FIFO and go!
        #-------------------------------------------------------------------------------------------
        mpu6050.resetFIFO()
        while self.keep_looping:

            #---------------------------------------------------------------------------------------
            # Sleep for a while waiting for the FIFO to collect several batches of data
            #---------------------------------------------------------------------------------------
            motion_time = time.time() - start_motion

            #---------------------------------------------------------------------------------------
            # Sleep for up to 10ms before collectedin another batch of data from the FIFO
            #---------------------------------------------------------------------------------------
            sleep_time = esc_period - motion_time
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            start_motion = time.time()

            #---------------------------------------------------------------------------------------
            # Now get the batch over averaged data from the FIFO.
            #---------------------------------------------------------------------------------------
            qax, qay, qaz, qrx, qry, qrz, i_time = mpu6050.readFIFO()

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
            # Track the number of motion loops and sampling loops; any discrepancy between these are the
            # missed samples or sampling errors.
            #---------------------------------------------------------------------------------------
            motion_loops += 1
            sampling_loops += i_time * sampling_rate

            #---------------------------------------------------------------------------------------
            # Angular predication: Now we know the time since the last batch of samples, update the
            # previous angles with the 'integral' of the previous euler rotation rates.
            #---------------------------------------------------------------------------------------
            urp, urr, ury = Body2EulerRates(qry, qrx, qrz, pa, ra)
            pa += urp * i_time
            ra += urr * i_time
            ya += ury * i_time

            #---------------------------------------------------------------------------------------
            # Rotate gravity to the new quadframe
            #---------------------------------------------------------------------------------------
            qgx, qgy, qgz = RotateE2Q(egx, egy, egz, pa, ra, ya)

            #---------------------------------------------------------------------------------------
            # Using a complementary filter,  merge the short-term noise free predicted angles with the
            # long-term accurate acclerometer which short term are tainted with acceleration as well as
            # gravity. tau is the period during which we expect acceleration to average out leading
            # only the net gravity value. tau can be small if acceleration is short and sharp.  There is
            # a balance here between the period to trust the integrated, reorientated gyro readings and
            # the period where accelerometer spikes average out.
            #---------------------------------------------------------------------------------------
            upa, ura = GetRotationAngles(qax, qay, qaz)

            tau_fraction = tau / (tau + i_time)
            pa = tau_fraction * pa + (1 - tau_fraction) * upa
            ra = tau_fraction * ra + (1 - tau_fraction) * ura

            #---------------------------------------------------------------------------------------
            # Get the current flight plan targets
            #---------------------------------------------------------------------------------------
            if not ready_to_fly:
                if hover_speed >= hover_target:
                    hover_speed = hover_target
                    ready_to_fly = True
                    print "RTF @ %fs" % (time.time() - start_flight)

                else:
                    hsf += hover_target * i_time / rtf_period
                    hover_speed = int(math.trunc(hsf))

            else:
                evx_target, evy_target, evz_target = fp.getTargets(i_time)

            #---------------------------------------------------------------------------------------
            # Convert earth-frame velocity targets to quadcopter frame.
            #---------------------------------------------------------------------------------------
            qvx_target, qvy_target, qvz_target = RotateE2Q(evx_target, evy_target, evz_target, pa, ra, ya)

            #---------------------------------------------------------------------------------------
            # Redistribute gravity around the new orientation of the quad
            #---------------------------------------------------------------------------------------
            qgx, qgy, qgz = RotateE2Q(egx, egy, egz, pa, ra, ya)

            #---------------------------------------------------------------------------------------
            # Delete reorientated gravity from raw accelerometer readings and sum to make velocity all
            # in quad frame
            #---------------------------------------------------------------------------------------
            qvx_input += (qax - qgx) * i_time * GRAV_ACCEL
            qvy_input += (qay - qgy) * i_time * GRAV_ACCEL
            qvz_input += (qaz - qgz) * i_time * GRAV_ACCEL

            #=======================================================================================
            # Motion PIDs: Run the horizontal speed PIDs to determine targets for rotation rate PIDs
            # and the vertical speed PID to control height.
            #=======================================================================================
            [p_out, i_out, d_out] = qvx_pid.Compute(qvx_input, qvx_target, i_time)
    #        qvx_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            qvx_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = qvy_pid.Compute(qvy_input, qvy_target, i_time)
    #        qvy_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            qvy_out =  p_out + i_out + d_out

            [p_out, i_out, d_out] = qvz_pid.Compute(qvz_input, qvz_target, i_time)
    #        qvz_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            qvz_out = p_out + i_out + d_out

            #---------------------------------------------------------------------------------------
            # Convert the horizontal velocity PID output i.e. the horizontal acceleration target in
            # g's into the pitch and roll angle PID targets in radians
            # - A forward unintentional drift is a positive input and negative output from the
            #   velocity PID.  This represents corrective acceleration.  To achieve corrective
            #   backward acceleration, the negative velocity PID output needs to trigger a negative
            #   pitch rotation rate
            # - A left unintentional drift is a positive input and negative output from the velocity
            #   PID.  To achieve corrective right acceleration, the negative velocity PID output needs
            #   to trigger a positive roll rotation rate
            #---------------------------------------------------------------------------------------

            #---------------------------------------------------------------------------------------
            # Use a bit of hokey trigonometry to convert desired quad frame acceleration (qv*_out)
            # into the target quad frame rotation rate that provides that acceleration (*r_target)
            #---------------------------------------------------------------------------------------
            pr_target = math.atan(qvx_out)
            rr_target = -math.atan(qvy_out)

            #---------------------------------------------------------------------------------------
            # Convert the vertical velocity PID output direct to PWM pulse width.
            #---------------------------------------------------------------------------------------
            vert_out = hover_speed + int(round(qvz_out))

            #=======================================================================================
            # START TESTCASE 3 CODE: Override motion processing results; instead use angles to maintain
            #                        horizontal flight regardless of take-off platform angle.
            # NOTE: This code is currently using the wrong type of angles (rotation versus absolute) so
            #       will not work safely.  Work needs to be done above to also produce absolute angles
            #       as a fusion of gyro and absolute accelerometer angles.
            #---------------------------------------------------------------------------------------
            if test_case == 3:
                    pa_target = 0.0
                    [p_out, i_out, d_out] = pa_pid.Compute(pa, pa_target, i_time)
                    pa_diags = "%f, %f, %f" % (p_out, i_out, d_out)
                    pa_out = p_out + i_out + d_out
                    pr_target = pa_out

                    ra_target = 0.0
                    [p_out, i_out, d_out] = ra_pid.Compute(ra, ra_target, i_time)
                    ra_diags = "%f, %f, %f" % (p_out, i_out, d_out)
                    ra_out = p_out + i_out + d_out
                    rr_target = ra_out
            #=======================================================================================
            # END TESTCASE 3 CODE: Override motion processing results; instead use angles to maintain
            #                      horizontal flight regardless of take-off platform angle.
            #=======================================================================================

            #---------------------------------------------------------------------------------------
            # For the moment, we just want yaw to not exist.  It's only required if we want the front
            # of the quad to face the direction it's travelling.  This only really becomes important
            # if videoing a flight.
            #---------------------------------------------------------------------------------------
            ya_target = 0.0
            [p_out, i_out, d_out] = ya_pid.Compute(ya, ya_target, i_time)
    #        ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            ya_out = p_out + i_out + d_out
            yr_target = ya_out

            #=======================================================================================
            # START TESTCASE 2 CODE: Override motion processing results; take-off from horizontal
            #                        platform, tune the pr*_gain and rr*_gain PID gains for
            #                        stability.
            #=======================================================================================
            if test_case == 2:
                pr_target = 0.0
                rr_target = 0.0
                yr_target = 0.0
            #=======================================================================================
            # END TESTCASE 2 CODE: Override motion processing results; take-off from horizontal
            #                      platform, tune the pr*_gain and rr*_gain PID gains for
            #                      stability.
            #=======================================================================================

            #=======================================================================================
            # Attitude PIDs: Run the rotation rate PIDs each rotation axis to determine overall PWM
            # output.
            #=======================================================================================
            [p_out, i_out, d_out] = pr_pid.Compute(qry, pr_target, i_time)
    #        pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            pr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = rr_pid.Compute(qrx, rr_target, i_time)
    #        rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            rr_out = p_out + i_out + d_out

            [p_out, i_out, d_out] = yr_pid.Compute(qrz, yr_target, i_time)
    #        yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
            yr_out = p_out + i_out + d_out

            #---------------------------------------------------------------------------------------
            # Convert the rotation rate PID outputs direct to PWM pulse width
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
                logger.warning('%f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %f, %s, %d, %f, %f, %s, %f, %s, %d, %f, %f, %s, %d, %f, %s, %d, %d, %d, %d, %d',
                                sampling_loops / sampling_rate, i_time, sampling_loops, sleep_time, temp / 333.86 + 21, qrx, qry, qrz, qax, qay, qaz, egx, egy, egz, qgx, qgy, qgz, qvx_input, qvy_input, qvz_input, math.degrees(pa), math.degrees(ra), math.degrees(ya), evx_target, qvx_target, qvx_diags, math.degrees(pr_target), pr_diags, pr_out, evy_target, qvy_target, qvy_diags, math.degrees(rr_target), rr_diags, rr_out, evz_target, qvz_target, qvz_diags, qvz_out, math.degrees(yr_target), yr_diags, yr_out, esc_list[0].pulse_width, esc_list[1].pulse_width, esc_list[2].pulse_width, esc_list[3].pulse_width)

        print "flight time %f" % (time.time() - start_flight)

        temp = mpu6050.readTemperature()
        logger.warning("IMU core temp: %f", temp / 333.86 + 21.0)
        logger.warning("motion_loops %d", motion_loops)
        logger.warning("sampling_loops %d", sampling_loops)
        (data_errors, i2c_errors, num_2g_hits) = mpu6050.getMisses()
        logger.warning("%d data errors; %d i2c errors; %d 2g hits", data_errors, i2c_errors, num_2g_hits)

        #-------------------------------------------------------------------------------------------
        # Stop the PWM between flights
        #-------------------------------------------------------------------------------------------
        for esc in self.esc_list:
            esc.update(0)

    ####################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ####################################################################################################
    def shutdown(self):

        #-----------------------------------------------------------------------------------------------
        # Stop the signal handler
        #-----------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        #-----------------------------------------------------------------------------------------------
        # Stop the blades spinning
        #-----------------------------------------------------------------------------------------------
        for esc in self.esc_list:
            esc.update(0)

        #-----------------------------------------------------------------------------------------------
        # Stop the video if it's running
        #-----------------------------------------------------------------------------------------------
        if self.shoot_video:
            video.send_signal(signal.SIGINT)

        #-----------------------------------------------------------------------------------------------
        # Copy logs from /dev/shm (shared / virtual memory) to the Logs directory.
        #-----------------------------------------------------------------------------------------------
        now = datetime.now()
        now_string = now.strftime("%y%m%d-%H:%M:%S")
        log_file_name = "qcstats.csv"
        shutil.move("/dev/shm/qclogs", log_file_name)

        #-----------------------------------------------------------------------------------------------
        # Unlock memory we've used from RAM
        #-----------------------------------------------------------------------------------------------
        munlockall()

        #-----------------------------------------------------------------------------------------------
        # Clean up PWM / GPIO, but pause beforehand to give the ESCs time to stop properly
        #-----------------------------------------------------------------------------------------------
        time.sleep(1.0)
        PWMTerm()

        #-----------------------------------------------------------------------------------------------
        # Reset the signal handler to default
        #-----------------------------------------------------------------------------------------------
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

