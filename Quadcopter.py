#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2015 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################

from __future__ import division
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

from RPIO import PWM
import GPIO.GPIO as RPIO

import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library
import random


dri_frequency = 0
adc_frequency = 0
samples_per_motion = 0

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
                logger.critical("i2c miss")
                self.misses += 1

    def write8(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        while True:
            try:
                self.bus.write_byte_data(self.address, reg, value)
                break
            except IOError, err:
                logger.critical("i2c miss")
                self.misses += 1

    def writeList(self, reg, list):
        "Writes an array of bytes using I2C format"
        while True:
            try:
                self.bus.write_i2c_block_data(self.address, reg, list)
                break
            except IOError, err:
                logger.critical("i2c miss")
                self.misses += 1

    def readU8(self, reg):
        "Read an unsigned byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                return result
            except IOError, err:
                logger.critical("i2c miss")
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
                logger.critical("i2c miss")
                self.misses += 1

    def readU16(self, reg):
        "Reads an unsigned 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                logger.critical("i2c miss")
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
                logger.critical("i2c miss")
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
    __MPU6050_RA_XG_OFFS_TC= 0x00       # [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
    __MPU6050_RA_YG_OFFS_TC= 0x01       # [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
    __MPU6050_RA_ZG_OFFS_TC= 0x02       # [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
    __MPU6050_RA_X_FINE_GAIN= 0x03      # [7:0] X_FINE_GAIN
    __MPU6050_RA_Y_FINE_GAIN= 0x04      # [7:0] Y_FINE_GAIN
    __MPU6050_RA_Z_FINE_GAIN= 0x05      # [7:0] Z_FINE_GAIN
    __MPU6050_RA_XA_OFFS_H= 0x06    # [15:0] XA_OFFS
    __MPU6050_RA_XA_OFFS_L_TC= 0x07
    __MPU6050_RA_YA_OFFS_H= 0x08    # [15:0] YA_OFFS
    __MPU6050_RA_YA_OFFS_L_TC= 0x09
    __MPU6050_RA_ZA_OFFS_H= 0x0A    # [15:0] ZA_OFFS
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
    __MPU9250_RA_ACCEL_CFG_2= 0x1D
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

    __CALIBRATION_ITERATIONS = 50

    __SCALE_GYRO = 500.0 * math.pi / (65536 * 180)
    __SCALE_ACCEL = 8.0 / 65536

    def __init__(self, address=0x68, alpf=1, glpf=1):
        global adc_frequency

        self.i2c = I2C(address)
        self.address = address
        self.sensor_data = array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.result_array = array('h', [0, 0, 0, 0, 0, 0, 0])
        self.ambient = 0

        self.num_i2c_errs = 0
        self.num_data_errs = 0

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
        self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, math.trunc(1000 / adc_frequency) - 1)
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
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_CONFIG, glpf)
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
        # Disable accel self tests, scale of +/-2g
        #
        # 0x00 =  +/- 2g
        # 0x08 =  +/- 4g
        # 0x10 =  +/- 8g
        # 0x18 = +/- 16g
        # See SCALE_ACCEL for convertion from raw data to units of meters per second squared
        #-------------------------------------------------------------------------------------------
        # int(math.log(g / 2, 2)) << 3
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x08)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Setup INT pin to push / pull,  50us pulse.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x10)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enable data ready interrupt
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x01)
        time.sleep(0.1)

    def readSensors(self):
        #-------------------------------------------------------------------------------------------
        # For the sake of always getting good date wrap the interrupt and the register read in a try
        # except loop. dt is the time taken to get a good reading in ms.
        #-------------------------------------------------------------------------------------------
        dt = 1
        while True:
            try:
                #-----------------------------------------------------------------------------------
                # Wait for the data ready interrupt
                #-----------------------------------------------------------------------------------
                RPIO.edge_detect_wait(RPIO_DATA_READY_INTERRUPT)

                #-----------------------------------------------------------------------------------
                # For speed of reading, read all the sensors and parse to SHORTs after.  This also
                # ensures a self consistent set of sensor data compared to reading each individually
                # where the sensor data registers could be updated between reads.
                #-----------------------------------------------------------------------------------
                sensor_data = self.i2c.readList(self.__MPU6050_RA_ACCEL_XOUT_H, 14)
            except:
                self.num_i2c_errs += 1
                dt += 1
                continue

            #---------------------------------------------------------------------------------------
            # Convert the array of 14 bytes to 7 shorts
            #---------------------------------------------------------------------------------------
            for index in range(0, 14, 2):
                if (sensor_data[index] > 127):
                    sensor_data[index] -= 256
                self.result_array[int(index / 2)] = (sensor_data[index] << 8) + sensor_data[index + 1]

            #-------------------------------------------------------------------------------------------
            # +/- 4g * 16 bit range for the accelerometer
            # +/- 250 degrees per second * 16 bit range for the gyroscope
            #
            # If the code is a bit slow reacting the the data ready interrupt to read the data, it is
            # possible that the data has already been overwritten.  This shows as field set to -1,
            # starting at the gyroscope values.  However, -1 values in gyro readings have no long term
            # effect, and miniscule short term effect.  In comparison, a -1 for z-axis acceleration is
            # effectively free-fall in a vacuum, and therefore is definitely actually a duff data read.
            # Hence this is the check we use for duff data.
            #-------------------------------------------------------------------------------------------
            [ax, ay, az, temp, gx, gy, gz] = self.result_array
            if az == -1:
               self.num_data_errs += 1
               dt += 1
               continue
            break

        return ax, ay, az, gx, gy, gz, dt

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = ax * self.__SCALE_ACCEL
        qay = ay * self.__SCALE_ACCEL
        qaz = az * self.__SCALE_ACCEL

        qrx = (gx - self.gx_offset) * self.__SCALE_GYRO
        qry = (gy - self.gy_offset) * self.__SCALE_GYRO
        qrz = (gz - self.gz_offset) * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz


    def calibrateGyros(self):
        gx_offset = 0.0
        gy_offset = 0.0
        gz_offset = 0.0

        for iteration in range(0, self.__CALIBRATION_ITERATIONS):
            [ax, ay, az, gx, gy, gz, dt] = self.readSensors()

            gx_offset += gx
            gy_offset += gy
            gz_offset += gz

        self.gx_offset = gx_offset / self.__CALIBRATION_ITERATIONS
        self.gy_offset = gy_offset / self.__CALIBRATION_ITERATIONS
        self.gz_offset = gz_offset / self.__CALIBRATION_ITERATIONS


    def getMisses(self):
        self.num_i2c_errs += self.i2c.getMisses()
        return (self.num_data_errs, self.num_i2c_errs)

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
        self.sensor_data = array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
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
# Butterwork IIR Filter calculator and actor - this is carried out in the earth frame as we are track
# gravity drift over time from 0, 0, 1 (the primer values for egx, egy and egz)
#
####################################################################################################
class BUTTERWORTH:
    def __init__(self, sampling, cutoff, order):

        self.n = int(round(order / 2))
        self.A = array("f", [])
        self.d1 = array("f", [])
        self.d2 = array("f", [])
        self.w0 = array("f", [])
        self.w1 = array("f", [])
        self.w2 = array("f", [])

        for ii in range(0, self.n):
            self.A.append(0.0)
            self.d1.append(0.0)
            self.d2.append(0.0)
            self.w0.append(0.0)
            self.w1.append(0.0)
            self.w2.append(0.0)


        a = math.tan(math.pi * cutoff / sampling)
        a2 = math.pow(a, 2.0)

        for ii in range(0, self.n):
            r = math.sin(math.pi * (2.0 * ii + 1.0) / (4.0 * self.n))
            s = a2 + 2.0 * a * r + 1.0
            self.A[ii] = a2 / s
            self.d1[ii] = 2.0 * (1 - a2) / s
            self.d2[ii] = -(a2 - 2.0 * a * r + 1.0) / s

    def filter(self, input):
        for ii in range(0, self.n):
            self.w0[ii] = self.d1[ii] * self.w1[ii] + self.d2[ii] * self.w2[ii] + input
            output = self.A[ii] * (self.w0[ii] + 2.0 * self.w1[ii] + self.w2[ii])
            self.w2[ii] = self.w1[ii]
            self.w1[ii] = self.w0[ii]

        return output

####################################################################################################
#
# GPIO pins initialization for MPU6050 interrupt, sounder and hardware PWM
#
####################################################################################################
def RpioSetup():
    RPIO.setmode(RPIO.BCM)

    #-----------------------------------------------------------------------------------------------
    # Set the MPU6050 interrupt input - this is a floating input; the IMU interrupt drives this
    # input up when data is ready, and reading the data drives it down.
    #-----------------------------------------------------------------------------------------------
    logger.info('Setup MPU6050 interrupt input %s', RPIO_DATA_READY_INTERRUPT)
    RPIO.setup(RPIO_DATA_READY_INTERRUPT, RPIO.IN, RPIO.PUD_DOWN)
    RPIO.edge_detect_init(RPIO_DATA_READY_INTERRUPT, RPIO.RISING)

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
def RpioCleanup():
    PWM.cleanup()
    RPIO.edge_detect_term(RPIO_DATA_READY_INTERRUPT)
    RPIO.cleanup()


####################################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
####################################################################################################
def CheckCLI(argv):
    global adc_frequency
    global dri_frequency
    global samples_per_motion

    #===============================================================================================
    # adc_frequency      - the value programmed into the IMU to sample the ADC sampling of the sensors
    # dri_frequency      - the rate the data ready interrupt fires for those samples to be read
    # samples_per_motion - the number of dri triggered samples are to be batched before invoking
    #                      motion processing.  This is about 1/100 of the dri_frequency so that
    #                      motion processing is invoked roughly every 10ms
    #===============================================================================================
    adc_frequency = 500
    dri_frequency = 500
    samples_per_motion = 10

    cli_fly = False
    cli_video = False
    cli_hover_target = 0

    #-----------------------------------------------------------------------------------------------
    # Other configuration defaults
    #-----------------------------------------------------------------------------------------------
    cli_test_case = 0
    cli_alpf = 3
    cli_glpf = 2
    cli_diagnostics = False
    cli_rtf_period = 1.5
    cli_tau = 0.5

    hover_target_defaulted = True

    if i_am_phoebe:
        #-------------------------------------------------------------------------------------------
        # Phoebe's PID configuration due to using her ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_target = 500

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
        cli_prp_gain = 90.0
        cli_pri_gain = 20.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 90.0
        cli_rri_gain = 20.0
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
        cli_prp_gain = 140.0
        cli_pri_gain = 16.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 140.0
        cli_rri_gain = 16.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw angle PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 50.0
        cli_yri_gain = 25.0
        cli_yrd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Right, let's get on with reading the command line and checking consistency
    #-----------------------------------------------------------------------------------------------
    try:
        opts, args = getopt.getopt(argv,'dfvh:r:', ['tc=', 'tau=', 'vvp=', 'vvi=', 'vvd=', 'hvp=', 'hvi=', 'hvd=', 'prp=', 'pri=', 'prd=', 'rrp=', 'rri=', 'rrd=', 'tau=', 'yrp=', 'yri=', 'yrd=', 'alpf=', 'glpf='])
    except getopt.GetoptError:
        logger.critical('Must specify one of -f or -g or --tc')
        logger.critical('  qcpi.py')
        logger.critical('  -f set whether to fly')
        logger.critical('  -h set the hover speed for manual testing')
        logger.critical('  -d enable diagnostics')
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
        logger.critical('  --alpf set the accelerometer low pass filter')
        logger.critical('  --glpf set the gyroscope low pass filter')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-f':
            cli_fly = True

        elif opt in '-h':
            cli_hover_target = int(arg)
            hover_target_defaulted = False

        elif opt in '-v':
            cli_video = True

        elif opt in '-d':
            cli_diagnostics = True

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

        elif opt in '--alpf':
            cli_alpf = int(arg)

        elif opt in '--glpf':
            cli_glpf = int(arg)

    if not cli_fly and cli_test_case == 0:
        logger.critical('Must specify one of -f or --tc')
        sys.exit(2)

    elif cli_hover_target < 0 or cli_hover_target > 1000:
        logger.critical('Hover speed must lie in the following range')
        logger.critical('0 <= hover speed <= 1000')
        sys.exit(2)

    elif cli_test_case == 0 and cli_fly:
        logger.critical('Pre-flight checks passed, enjoy your flight, sir!')


    elif cli_test_case != 1 and cli_test_case != 2:
        logger.critical('Only 1 or 2 are valid testcases')
        sys.exit(2)

    elif cli_test_case == 1 and hover_target_defaulted:
        logger.critical('You must choose a specific hover speed (-h) for test case 1 - try 200')
        sys.exit(2)


    return cli_fly, cli_hover_target, cli_video, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_prp_gain, cli_pri_gain, cli_prd_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_yrp_gain, cli_yri_gain, cli_yrd_gain, cli_test_case, cli_alpf, cli_glpf, cli_rtf_period, cli_tau, cli_diagnostics

####################################################################################################
#
# Shutdown triggered by early Ctrl-C or end of script
#
####################################################################################################
def CleanShutdown():

    #-----------------------------------------------------------------------------------------------
    # Stop the signal handler
    #-----------------------------------------------------------------------------------------------
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    #-----------------------------------------------------------------------------------------------
    # Stop the blades spinning
    #-----------------------------------------------------------------------------------------------
    for esc in esc_list:
        esc.update(0)

    #-----------------------------------------------------------------------------------------------
    # Stop the video if it's running
    #-----------------------------------------------------------------------------------------------
    if shoot_video:
        video.send_signal(signal.SIGINT)

    #-----------------------------------------------------------------------------------------------
    # If the sensor data acquisition is running, then stop it
    #-----------------------------------------------------------------------------------------------
    if sampling is not None:
        sampling.go = False;

    #-----------------------------------------------------------------------------------------------
    # Copy logs from /dev/shm (shared / virtual memory) to the Logs directory.
    #-----------------------------------------------------------------------------------------------
    now = datetime.now()
    now_string = now.strftime("%y%m%d-%H:%M:%S")
    log_file_name = "qcstats" + now_string + ".csv"
    shutil.move("/dev/shm/qclogs", log_file_name)

    #-----------------------------------------------------------------------------------------------
    # Unlock memory we've used from RAM
    #-----------------------------------------------------------------------------------------------
    munlockall()

    #-----------------------------------------------------------------------------------------------
    # Clean up PWM / GPIO, but pause beforehand to give the ESCs time to stop properly
    #-----------------------------------------------------------------------------------------------
    time.sleep(1.0)
    RpioCleanup()

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
def ShutdownSignalHandler(signal, frame):
    global keep_looping
    global woken_by
    global SIG_SHUTDOWN

    if not keep_looping:
        CleanShutdown()
    keep_looping = False
    woken_by = SIG_SHUTDOWN

####################################################################################################
#
# Signal handler for new data ready to process in multi-threaded operation
#
####################################################################################################
def DataReadySignalHandler(signal, frame):
    global woken_by
    global SIG_DATA_READY

    #-----------------------------------------------------------------------------------------------
    # This does nothing other than wake the main thread up
    #-----------------------------------------------------------------------------------------------
    woken_by = SIG_DATA_READY

####################################################################################################
#
# Flight plan management
#
####################################################################################################
class FlightPlan:

    #-----------------------------------------------------------------------------------------------
    # The flight plan - move to file at some point to allow various FP's to be saved, and selected
    # per flight.
    #-----------------------------------------------------------------------------------------------
    fp_evx_target  = [0.0,       0.0,       0.0,       0.0,       0.0]
    fp_evy_target  = [0.0,       0.0,       0.0,       0.0,       0.0]
    fp_evz_target  = [0.0,       0.5,       0.0,      -0.5,       0.0]
    fp_time        = [0.0,       2.0,       4.0,       2.0,       0.0]
    fp_name        = ["RTF",  "ASCENT",   "HOVER", "DESCENT",    "STOP"]
    _FP_STEPS = 5

    def __init__(self):

        self.fp_index = 0
        self.fp_prev_index = 0
        self.elapsed_time = 0.0


    def getTargets(self, delta_time):
        global keep_looping

        self.elapsed_time += delta_time

        fp_total_time = 0.0
        for fp_index in range(0, self._FP_STEPS):
            fp_total_time += self.fp_time[fp_index]
            if self.elapsed_time < fp_total_time:
                break
        else:
            keep_looping = False

        if fp_index != self.fp_prev_index:
            logger.critical("%s", self.fp_name[fp_index])
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
# Main
#
# Variable naming conventions
# ---------------------------
# qa* = quad frame acceleration
# qg* = quad frame gravity
# qr* = quad frame rotation
# ea* = earth frame acceleration
# eg* = earth frame gravity
# ua* = euler angles between reference frames
# ur* = euler rotation between frames
# ??x = fore / aft acceleration and rotation axis
# ??y = port / starboard acceleration and rotation axis
# ??z = up / down acceleration and rotation axis
#
####################################################################################################
def go(name):

    #-----------------------------------------------------------------------------------------------
    # Global variables
    #-----------------------------------------------------------------------------------------------
    global logger
    global mpu6050
    global woken_by
    global threading
    global sampling
    global keep_looping
    global i_am_phoebe
    global i_am_chloe
    global esc_list
    global shoot_video

    global adc_frequency
    global dri_frequency
    global samples_per_motion

    #-----------------------------------------------------------------------------------------------
    # Global constants
    #-----------------------------------------------------------------------------------------------
    global RPIO_DATA_READY_INTERRUPT
    global RPIO_DMA_CHANNEL
    global SIG_DATA_READY
    global SIG_SHUTDOWN
    global SIG_NONE

    #-----------------------------------------------------------------------------------------------
    # Who am I?
    #-----------------------------------------------------------------------------------------------
    i_am_phoebe = False
    i_am_chloe = False

    if name == "phoebe":
        print "Hi, I'm Phoebe. Nice to meet you!"
        i_am_phoebe = True
    elif name == "chloe":
        print "Hi, I'm Chloe.  Nice to meet you!"
        i_am_chloe = True
    else:
        print "Sorry, I'm not qualified to fly this quadcopter."
        sys.exit(0)

    #-----------------------------------------------------------------------------------------------
    # Lock code permanently in memory - no swapping to disk
    #-----------------------------------------------------------------------------------------------
    mlockall()

    #-----------------------------------------------------------------------------------------------
    # Set up the base logging
    #-----------------------------------------------------------------------------------------------
    logger = logging.getLogger('QC logger')
    logger.setLevel(logging.INFO)

    #-----------------------------------------------------------------------------------------------
    # Create file and console logger handlers - the file is written into shared memory and only
    # dumped to disk / SD card at the end of a flight for performance reasons
    #-----------------------------------------------------------------------------------------------
    file_handler = logging.FileHandler("/dev/shm/qclogs", 'w')
    file_handler.setLevel(logging.WARNING)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.CRITICAL)

    #-----------------------------------------------------------------------------------------------
    # Create a formatter and add it to both handlers
    #-----------------------------------------------------------------------------------------------
    console_formatter = logging.Formatter('%(message)s')
    console_handler.setFormatter(console_formatter)

    file_formatter = logging.Formatter('[%(levelname)s] (%(threadName)-10s) %(funcName)s %(lineno)d, %(message)s')
    file_handler.setFormatter(file_formatter)

    #-----------------------------------------------------------------------------------------------
    # Add both handlers to the logger
    #-----------------------------------------------------------------------------------------------
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    #-----------------------------------------------------------------------------------------------
    # First log, whose flying and under what configuration
    #-----------------------------------------------------------------------------------------------
    logger.warning("%s is flying.", "Phoebe" if i_am_phoebe else "Chloe")

    #-----------------------------------------------------------------------------------------------
    # Check the command line for calibration or flight parameters
    #-----------------------------------------------------------------------------------------------
    flying, hover_target, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, alpf, glpf, rtf_period, tau, diagnostics = CheckCLI(sys.argv[1:])
    logger.warning("fly = %s, hover_target = %d, shoot_video = %s, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, prp_gain = %f, pri_gain = %f, prd_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, yrp_gain = %f, yri_gain = %f, yrd_gain = %f, test_case = %d, alpf = %d, glpf = %d, rtf_period = %f, tau = %f, diagnostics = %s",
            flying, hover_target, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, alpf, glpf, rtf_period, tau, diagnostics)

    #-----------------------------------------------------------------------------------------------
    # Initialize the numeric globals
    #-----------------------------------------------------------------------------------------------
    SIG_NONE = 0
    SIG_DATA_READY = 1
    SIG_SHUTDOWN = 2
    woken_by = SIG_NONE

    #-----------------------------------------------------------------------------------------------
    # Set the BCM input assigned to sensor data ready interrupt
    #-----------------------------------------------------------------------------------------------
    RPIO_DMA_CHANNEL = 1

    if i_am_phoebe:
        RPIO_DATA_READY_INTERRUPT = 22
    elif i_am_chloe:
        RPIO_DATA_READY_INTERRUPT = 22

    #-----------------------------------------------------------------------------------------------
    # Set up the bits of state setup before takeoff
    #-----------------------------------------------------------------------------------------------
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
    ready_to_fly = False
    keep_looping = False

    mpu6050 = None
    ms5611 = None
    sampling = None

    #-----------------------------------------------------------------------------------------------
    # Initialize the gyroscope / accelerometer I2C object - this must be done before the data ready
    # interrupt handler so the handler doesn't block
    #-----------------------------------------------------------------------------------------------
    mpu6050 = MPU6050(0x68, alpf, glpf)

    #-----------------------------------------------------------------------------------------------
    # Initialize the barometer / altimeter I2C object
    #-----------------------------------------------------------------------------------------------
    ms5611 = MS5611(0x77)

    #-----------------------------------------------------------------------------------------------
    # Enable RPIO for beeper, MPU 6050 interrupts and PWM.  This must be set up prior to adding
    # the SignalHandler below or it will override what we set thus killing the "Kill Switch"..
    #-----------------------------------------------------------------------------------------------
    RpioSetup()

    #-----------------------------------------------------------------------------------------------
    # Set the signal handler here so the core processing loop can be stopped (or not started) by
    # Ctrl-C.
    #-----------------------------------------------------------------------------------------------
    signal.signal(signal.SIGINT, ShutdownSignalHandler)

    #-----------------------------------------------------------------------------------------------
    # Set up the ESC to GPIO pin and location mappings and assign to each ESC
    #-----------------------------------------------------------------------------------------------
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

    #-----------------------------------------------------------------------------------------------
    # Prime the ESCs with the default 0 spin rotors to stop their whining!
    #-----------------------------------------------------------------------------------------------
    esc_list = []
    for esc_index in range(0, 4):
        esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
        esc_list.append(esc)

    #-----------------------------------------------------------------------------------------------
    # Give the PWM 5s to allow the ESCs to synchronize.
    #-----------------------------------------------------------------------------------------------
    logger.critical("Just warming up and chilling out.  Gimme 20s or so...")

    #===============================================================================================
    # START TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn
    #                        the right way
    #===============================================================================================
    if test_case == 1:
        logger.critical("TESTCASE 1: Check props are spinning as expected")
        for esc in esc_list:
            logger.critical("%s prop should rotate %s.", esc.name, "anti-clockwise" if esc.motor_rotation == MOTOR_ROTATION_ACW else "clockwise")
            for count in range(0, hover_target, 10):
                #-----------------------------------------------------------------------------------
                # Spin up to user determined (-h) hover speeds ~200
                #-----------------------------------------------------------------------------------
                esc.update(count)
                time.sleep(0.01)
            time.sleep(5.0)
            esc.update(0)
        CleanShutdown()
    #===============================================================================================
    # END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the
    #                      right way
    #===============================================================================================

    #-----------------------------------------------------------------------------------------------
    # Initialize the butterworth LP filters.
    # 100Hz = 1kHz pulses / 10 loops = sampling freqency
    # 0.1 = Cut-off frequency
    # 4 = order of filter
    #
    # " + 1" is required for motion processing at sampling frequencies <= 500Hz.  Sampling at >500Hz
    # leads to a less predicable set of lost data samples which can't be accounted for in the same
    # predictable way.
    #-----------------------------------------------------------------------------------------------
    bfx = BUTTERWORTH(dri_frequency / samples_per_motion, 0.05, 6)
    bfy = BUTTERWORTH(dri_frequency / samples_per_motion, 0.05, 6)
    bfz = BUTTERWORTH(dri_frequency / samples_per_motion, 0.05, 6)

    #-----------------------------------------------------------------------------------------------
    # Set up the global constants
    # - gravity in meters per second squared
    # - accelerometer in g's
    # - gyroscope in radians per second
    #-----------------------------------------------------------------------------------------------
    GRAV_ACCEL = 9.80665

    #-----------------------------------------------------------------------------------------------
    # Initialize the gyroscope / accelerometer I2C object
    #-----------------------------------------------------------------------------------------------
    # mpu6050 = MPU6050(0x68, alpf, glpf)

    #-----------------------------------------------------------------------------------------------
    # Initialize the barometer / altimeter I2C object
    #-----------------------------------------------------------------------------------------------
    # ms5611 = MS5611(0x77)

    #-----------------------------------------------------------------------------------------------
    # Calibrate gyros - this is a one-off
    #-----------------------------------------------------------------------------------------------
    mpu6050.calibrateGyros()

    #-----------------------------------------------------------------------------------------------
    # 20 seconds of loops here to fill up the butterworth filter with valid values, and get an
    # iterative increasingly accurate measure of the tilt of the take-off surface and hence gravity.
    #-----------------------------------------------------------------------------------------------
    pa = 0.0
    ra = 0.0
    ya = 0.0

    for ii in range(20 * dri_frequency):
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readSensors()
	qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)

	bpa, bra = GetRotationAngles(qax, qay, qaz)

        eax, eay, eaz = RotateQ2E(qax, qay, qaz, bpa, bra, ya)
        egx = bfx.filter(eax)
        egy = bfy.filter(eay)
        egz = bfz.filter(eaz)

        pa = 0.1 * bpa + 0.9 * pa
        ra = 0.1 * bra + 0.9 * ra

        if ii % dri_frequency == 0:
          logger.critical("%d...", 20 - int(ii / dri_frequency))

    #-----------------------------------------------------------------------------------------------
    # Log the critical parameters from this warm-up: the take-off surface tilt, and gravity. Note
    # that some of the variables used above are used in the main processing loop.  Messing with the
    # above code can have very unexpected effects in flight.
    #-----------------------------------------------------------------------------------------------
    logger.critical("pitch %f, roll %f", math.degrees(pa), math.degrees(ra))
    logger.critical("egx %f, egy %f, egz %f", egx, egy, egz)

    #-----------------------------------------------------------------------------------------------
    # Start up the video camera if required - this runs from take-off through to shutdown
    # automatically.  Run it in its own process group so that Ctrl-C for QC doesn't get through and
    # stop the video
    #-----------------------------------------------------------------------------------------------
    def Daemonize():
        os.setpgrp()

    if shoot_video:
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

    #-----------------------------------------------------------------------------------------------
    # The yaw angle PID controls stable angles around the Z-axis
    #-----------------------------------------------------------------------------------------------
    PID_YA_P_GAIN = 6.0 # yap_gain
    PID_YA_I_GAIN = 3.0 # yai_gain
    PID_YA_D_GAIN = 1.0 # yad_gain

    #-----------------------------------------------------------------------------------------------
    # The yaw rate PID controls stable rotation speed around the Z-axis
    #-----------------------------------------------------------------------------------------------
    PID_YR_P_GAIN = yrp_gain
    PID_YR_I_GAIN = yri_gain
    PID_YR_D_GAIN = yrd_gain

    print "%d data errors; %d i2c errors" % mpu6050.getMisses()
    logger.critical('Thunderbirds are go!')

    #-----------------------------------------------------------------------------------------------
    # Diagnostic log header
    #-----------------------------------------------------------------------------------------------
    if diagnostics:
        logger.warning('time, dt, loop, qrx, qry, qrz, qax, qay, qaz, efrgv_x, efrgv_y, efrgv_z, qfrgv_x, qfrgv_y, qfrgv_z, qvx_input, qvy_input, qvz_input, pitch, roll, yaw, evx_target, qvx_target, qxp, qxi, qxd, pr_target, prp, pri, prd, pr_out, evy_yarget, qvy_target, qyp, qyi, qyd, rr_target, rrp, rri, rrd, rr_out, evz_target, qvz_target, qzp, qzi, qzd, qvz_out, yr_target, yrp, yri, yrd, yr_out, FL spin, FR spin, BL spin, BR spin')

    #===============================================================================================
    # Initialize critical timing immediately before starting the PIDs.  This is done by reading the
    # sensors, and that also gives us a starting position of the rolling average from.
    #===============================================================================================

    #-----------------------------------------------------------------------------------------------
    # Start the X, Y (horizontal) and Z (vertical) velocity PIDs
    #-----------------------------------------------------------------------------------------------
    qvx_pid = PID(PID_QVX_P_GAIN, PID_QVX_I_GAIN, PID_QVX_D_GAIN)
    qvy_pid = PID(PID_QVY_P_GAIN, PID_QVY_I_GAIN, PID_QVY_D_GAIN)
    qvz_pid = PID(PID_QVZ_P_GAIN, PID_QVZ_I_GAIN, PID_QVZ_D_GAIN)

    #-----------------------------------------------------------------------------------------------
    # Start the yaw angle PID
    #-----------------------------------------------------------------------------------------------
    ya_pid = PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN)

    #-----------------------------------------------------------------------------------------------
    # Start the pitch, roll and yaw rate PIDs
    #-----------------------------------------------------------------------------------------------
    pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN)
    rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN)
    yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN)

    #-----------------------------------------------------------------------------------------------
    # Set up the data ready signal handler if using the multithreaded model. Note that using threading
    # on a single core Raspberry Pi (B, B+, A and A+) will have a detrimental effect on performance.
    # This code has not been tested on a B2 for performance, but should just work.
    #-----------------------------------------------------------------------------------------------
    threading = False
    if threading:
        signal.signal(signal.SIGUSR1, DataReadySignalHandler)

    #-----------------------------------------------------------------------------------------------
    # Set up the sensor data retrieval thread
    #-----------------------------------------------------------------------------------------------
    sampling = SAMPLING()

    #===============================================================================================
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
    #===============================================================================================
    sampling_loops = 0
    motion_loops = 0
    start_flight = time.time()

    keep_looping = True
    while keep_looping:
        #-------------------------------------------------------------------------------------------
        # Wait for the next batch of data to be available either from the separate thread or by
        # getting the data directly
        #-------------------------------------------------------------------------------------------
        if threading:
            signal.pause()
            if woken_by == SIG_DATA_READY:
                pass
            elif woken_by == SIG_SHUTDOWN:
                break
            woken_by = SIG_NONE
        else:
            sampling.integrator()

        #-------------------------------------------------------------------------------------------
        # Copy the latest data into the local copy and run with it
        #-------------------------------------------------------------------------------------------
        qax, qay, qaz, qrx, qry, qrz, i_time = sampling.collect()

        motion_loops += 1
        sampling_loops += i_time * dri_frequency

        #-------------------------------------------------------------------------------------------
        # Sort out units and calibration for the incoming data
        #-------------------------------------------------------------------------------------------
        qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax,
                                                            qay,
                                                            qaz,
                                                            qrx,
                                                            qry,
                                                            qrz)

        i_qrx = qrx * i_time
        i_qry = qry * i_time
        i_qrz = qrz * i_time

        #-------------------------------------------------------------------------------------------
        # Update the previous pitch, roll and yaw angles with the latest gyro output
        #-------------------------------------------------------------------------------------------
        urp, urr, ury = Body2EulerRates(qry, qrx, qrz, pa, ra)
        pa += urp * i_time
        ra += urr * i_time
        ya += ury * i_time

        #-------------------------------------------------------------------------------------------
        # Based upon the revised angles, rotate the latest accelerometer readings to earth frame.
        # Next, run the earth frame acceleration through the Butterworth LPF to extract gravity.
        # Next, rotate and revise gravity back to the quad frame.
        # Finally, based upon the new distribution of gravity around the quad frame, update the Euler
        # angles.
        #-------------------------------------------------------------------------------------------
        eax, eay, eaz = RotateQ2E(qax, qay, qaz, pa, ra, ya)

        egx = bfx.filter(eax)
        egy = bfy.filter(eay)
        egz = bfz.filter(eaz)

        qgx, qgy, qgz = RotateE2Q(egx, egy, egz, pa, ra, ya)

        #-------------------------------------------------------------------------------------------
        # Merge the short-term noise free gyro angles with the long-term accurate acclerometer angles
        #-------------------------------------------------------------------------------------------
        uap, uar = GetRotationAngles(qgx, qgy, qgz)
        tau_fraction = tau / (tau + i_time)
        pa = tau_fraction * pa + (1 - tau_fraction) * uap
        ra = tau_fraction * ra + (1 - tau_fraction) * uar

        #-------------------------------------------------------------------------------------------
        # Get the curent flight plan targets
        #-------------------------------------------------------------------------------------------
        if not ready_to_fly:
            if hover_speed >= hover_target:
                hover_speed = hover_target
                ready_to_fly = True

                #-----------------------------------------------------------------------------------
                # Register the flight plan with the authorities
                #-----------------------------------------------------------------------------------
                fp = FlightPlan()

            else:
                hover_speed += int(hover_target * i_time / rtf_period)

        else:
            evx_target, evy_target, evz_target = fp.getTargets(i_time)

        #-------------------------------------------------------------------------------------------
        # Convert earth-frame velocity targets to quadcopter frame.
        #-------------------------------------------------------------------------------------------
        qvx_target, qvy_target, qvz_target = RotateE2Q(evx_target, evy_target, evz_target, pa, ra, ya)

        #-------------------------------------------------------------------------------------------
        # Redistribute gravity around the new orientation of the quad
        #-------------------------------------------------------------------------------------------
        qgx, qgy, qgz = RotateE2Q(egx, egy, egz, pa, ra, ya)

        #-------------------------------------------------------------------------------------------
        # Delete reorientated gravity from raw accelerometer readings and sum to make velocity all
        # in quad frame
        #-------------------------------------------------------------------------------------------
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

        #-------------------------------------------------------------------------------------------
        # Convert the horizontal velocity PID output i.e. the horizontal acceleration target in g's
        # into the pitch and roll angle PID targets in radians
        # - A forward unintentional drift is a positive input and negative output from the velocity
        #   PID.  This represents corrective acceleration.  To achieve corrective backward
        #   acceleration, the negative velocity PID output needs to trigger a negative pitch
        #   rotation rate
        # - A left unintentional drift is a positive input and negative output from the velocity
        #   PID.  To achieve corrective right acceleration, the negative velocity PID output needs
        #   to trigger a positive roll rotation rate
        #-------------------------------------------------------------------------------------------

        #-------------------------------------------------------------------------------------------
        # Use a bit of hokey trigonometry to convert desired quad frame acceleration (qv*_out) into
        # the target quad frame rotation rate that provides that acceleration (*r_target
        #-------------------------------------------------------------------------------------------
        pr_target = math.atan(qvx_out)
        rr_target = -math.atan(qvy_out)

        #-------------------------------------------------------------------------------------------
        # Convert the vertical velocity PID output direct to PWM pulse width.
        #-------------------------------------------------------------------------------------------
        vert_out = hover_speed + int(round(qvz_out))

        #===========================================================================================
        # START TESTCASE 3 CODE: Override motion processing results; instead use angles to maintain
        #                        horizontal flight regardless of take-off platform angle.
        # NOTE: This code is currently using the wrong type of angles (rotation versus absolute) so
        #       will not work safely.  Work needs to be done above to also produce absolute angles
        #       as a fusion of gyro and absolute accelerometer angles.
        #-------------------------------------------------------------------------------------------
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
        #===========================================================================================
        # END TESTCASE 3 CODE: Override motion processing results; instead use angles to maintain
        #                      horizontal flight regardless of take-off platform angle.
        #===========================================================================================

        #-------------------------------------------------------------------------------------------
        # For the moment, we just want yaw to not exist.  It's only required if we want the front of
        # the quad to face the direction it's travelling.
        #-------------------------------------------------------------------------------------------
        ya_target = 0.0
        [p_out, i_out, d_out] = ya_pid.Compute(ya, ya_target, i_time)
#        ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
        ya_out = p_out + i_out + d_out
        yr_target = ya_out

        #===========================================================================================
        # START TESTCASE 2 CODE: Override motion processing results; take-off from horizontal
        #                        platform, tune the pr*_gain and rr*_gain PID gains for
        #                        stability.
        #===========================================================================================
        if test_case == 2:
            pr_target = 0.0
            rr_target = 0.0
            yr_target = 0.0
        #===========================================================================================
        # END TESTCASE 2 CODE: Override motion processing results; take-off from horizontal
        #                      platform, tune the pr*_gain and rr*_gain PID gains for
        #                      stability.
        #===========================================================================================

        #===========================================================================================
        # Attitude PIDs: Run the rotation rate PIDs each rotation axis to determine overall PWM
        # output.
        #===========================================================================================
        [p_out, i_out, d_out] = pr_pid.Compute(qry, pr_target, i_time)
#        pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
        pr_out = p_out + i_out + d_out

        [p_out, i_out, d_out] = rr_pid.Compute(qrx, rr_target, i_time)
#        rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
        rr_out = p_out + i_out + d_out

        [p_out, i_out, d_out] = yr_pid.Compute(qrz, yr_target, i_time)
#        yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
        yr_out = p_out + i_out + d_out

        #-------------------------------------------------------------------------------------------
        # Convert the rotation rate PID outputs direct to PWM pulse width
        #-------------------------------------------------------------------------------------------
        pr_out = int(round(pr_out / 2))
        rr_out = int(round(rr_out / 2))
        yr_out = int(round(yr_out / 2))

        #===========================================================================================
        # PID output distribution: Walk through the ESCs, and apply the PID outputs i.e. the updates
        # PWM pulse widths according to where the ESC is sited on the frame
        #===========================================================================================
        for esc in esc_list:
            #---------------------------------------------------------------------------------------
            # Update all blades' power in accordance with the z error
            #---------------------------------------------------------------------------------------
            delta_spin = vert_out

            #---------------------------------------------------------------------------------------
            # For a left downwards roll, the x gyro goes negative, so the PID error is positive,
            # meaning PID output is positive, meaning this needs to be added to the left blades
            # and subtracted from the right.
            #---------------------------------------------------------------------------------------
            if esc.motor_location & MOTOR_LOCATION_RIGHT:
                delta_spin -= rr_out
            else:
                delta_spin += rr_out

            #---------------------------------------------------------------------------------------
            # For a forward downwards pitch, the y gyro goes positive The PID error is negative as a
            # result, meaning PID output is negative, meaning this needs to be subtracted from the
            # front blades and added to the back.
            #---------------------------------------------------------------------------------------
            if esc.motor_location & MOTOR_LOCATION_BACK:
                delta_spin += pr_out
            else:
                delta_spin -= pr_out

            #---------------------------------------------------------------------------------------
            # For CW yaw, the z gyro goes negative, so the PID error is postitive, meaning PID
            # output is positive, meaning this need to be added to the ACW (FL and BR) blades and
            # subtracted from the CW (FR & BL) blades.
            #---------------------------------------------------------------------------------------
            if esc.motor_rotation == MOTOR_ROTATION_CW:
                delta_spin += yr_out
            else:
                delta_spin -= yr_out

            #---------------------------------------------------------------------------------------
            # Apply the blended outputs to the esc PWM signal
            #---------------------------------------------------------------------------------------
            esc.update(delta_spin)

        #-------------------------------------------------------------------------------------------
        # Diagnostic log - every motion loop
        #-------------------------------------------------------------------------------------------
        if diagnostics:
            logger.warning('%f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %f, %s, %d, %f, %f, %s, %f, %s, %d, %f, %f, %s, %d, %f, %s, %d, %d, %d, %d, %d',
                            sampling_loops / dri_frequency, i_time, sampling.total_loops, qrx, qry, qrz, qax, qay, qaz, egx, egy, egz, qgx, qgy, qgz, qvx_input, qvy_input, qvz_input, math.degrees(pa), math.degrees(ra), math.degrees(ya), evx_target, qvx_target, qvx_diags, math.degrees(pr_target), pr_diags, pr_out, evy_target, qvy_target, qvy_diags, math.degrees(rr_target), rr_diags, rr_out, evz_target, qvz_target, qvz_diags, qvz_out, math.degrees(yr_target), yr_diags, yr_out, esc_list[0].pulse_width, esc_list[1].pulse_width, esc_list[2].pulse_width, esc_list[3].pulse_width)


    #-----------------------------------------------------------------------------------------------
    # Time for telly bye byes - can't just 'pass' in the while loop as it locks out the sensor
    # thread from setting integrator_running to False - i.e. deadlock!
    #-----------------------------------------------------------------------------------------------
    sampling.go = False

    print "motion_loops %d" % motion_loops
    print "sampling_loops %d" % sampling_loops
    print "flight time %f" % (time.time() - start_flight)
    print "%d data errors; %d i2c errors" % mpu6050.getMisses()

    CleanShutdown()


####################################################################################################
#
# Class for managing sensor data collection / integration / transfer thread.
#
####################################################################################################
class SAMPLING():

    def __init__(self):
        #-------------------------------------------------------------------------------------------
        # Main thread
        #-------------------------------------------------------------------------------------------
        self.i_qax = 0.0
        self.i_qay = 0.0
        self.i_qaz = 0.0
        self.i_qrx = 0.0
        self.i_qry = 0.0
        self.i_qrz = 0.0
        self.i_time = 0.0

        #-------------------------------------------------------------------------------------------
        # Set up performance tracking.
        #-------------------------------------------------------------------------------------------
        self.elapsed_time = 0.0
        self.total_loops = 0

        #-------------------------------------------------------------------------------------------
        # Get a snapshot of the starting time and initialize the variables
        #-------------------------------------------------------------------------------------------
        if threading:
            self.pid = os.getpid()
            thread.start_new_thread(self.integrator, ())

    def collect(self):
        #-------------------------------------------------------------------------------------------
        # Main thread
        #-------------------------------------------------------------------------------------------
        return self.i_qax, self.i_qay, self.i_qaz, self.i_qrx, self.i_qry, self.i_qrz, self.i_time

    def integrator(self):
        #-------------------------------------------------------------------------------------------
        # Data collection + integration thread
        #-------------------------------------------------------------------------------------------
        iax = 0
        iay = 0
        iaz = 0
        igx = 0
        igy = 0
        igz = 0

        sampling_period = 0
        sampling_loops = 0

        self.go = True
        while self.go:
            #=======================================================================================
            # Sensors: Read the sensor values; note that this also sets the time_now to be as
            # accurate a time stamp for the sensor data as possible.
            #=======================================================================================
            ax, ay, az, gx, gy, gz, dt = mpu6050.readSensors()

            #---------------------------------------------------------------------------------------
            # Now we have the sensor snapshot, tidy up the rest of the variable so that
            # processing takes zero time.
            #---------------------------------------------------------------------------------------
            sampling_period += dt
            sampling_loops += 1

            #=======================================================================================
            # Integration: Sensor data is integrated over time, and later averaged to produce
            # smoother yet still accurate acceleration and rotation since the last PID updates.
            #=======================================================================================

            #---------------------------------------------------------------------------------------
            # Integrate the accelerometer readings.
            #---------------------------------------------------------------------------------------
            iax += ax
            iay += ay
            iaz += az

            #---------------------------------------------------------------------------------------
            # Integrate the gyros readings.
            #---------------------------------------------------------------------------------------
            igx += gx
            igy += gy
            igz += gz

            #=======================================================================================
            # Motion Processing:  Use the recorded data to produce motion data and feed in the
            # motion PIDs every 10 successful sensor reads.
            #=======================================================================================
            if sampling_loops == samples_per_motion:

                #-----------------------------------------------------------------------------------
                # Maintained for diagnostic purposes only
                #-----------------------------------------------------------------------------------
                self.elapsed_time += sampling_period / dri_frequency
                self.total_loops += samples_per_motion

                #-----------------------------------------------------------------------------------
                # Take the average of the last "samples per motion" valid samples allowing for missed
                # reads, and average.
                #-----------------------------------------------------------------------------------
                self.i_qax = iax / sampling_loops
                self.i_qay = iay / sampling_loops
                self.i_qaz = iaz / sampling_loops
                self.i_qrx = igx / sampling_loops
                self.i_qry = igy / sampling_loops
                self.i_qrz = igz / sampling_loops
                self.i_time = sampling_period / dri_frequency

                #-----------------------------------------------------------------------------------
                # Clear the integration for next time round
                #-----------------------------------------------------------------------------------
                iax = 0
                iay = 0
                iaz = 0
                igx = 0
                igy = 0
                igz = 0

                #-----------------------------------------------------------------------------------
                # Reset the timings for the next run around if we are running multithreaded
                #-----------------------------------------------------------------------------------
                sampling_period = 0
                sampling_loops = 0

                #-----------------------------------------------------------------------------------
                # If we are multithreaded, we need to kick motion processing thread.  If single
                # threaded, we just exit here, and wait to be called directly again once motion
                # processing is complete.
                #-----------------------------------------------------------------------------------
                if threading:
                    os.kill(self.pid, signal.SIGUSR1)
                else:
                    self.go = False

if __name__ == '__main__':
    print "Quadcopter.py must be run via qc.py.  For example:"
    print "  sudo python ./qc.py -f"
