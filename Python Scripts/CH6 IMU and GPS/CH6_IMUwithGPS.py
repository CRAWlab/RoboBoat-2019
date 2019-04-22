
#! /usr/bin/env python

##########################################################################################
# CH6_IMUwithGPS.py
#
# Code to read and process the CHRobotics UM6 IMU with attached GPS
#
# Modified from code on the Clearpath Robotics GitHub and used on the prior 
# CRAWLAB project with Swiftships
#   https://github.com/clearpathrobotics/imu_um6
#
#
# Created: 04/22/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 05/20/14 - Joshua Vaughan - joshua.vaughan@louisiana.edu
#       - Added show_imu_data() function with nicely formatted data
#   * 05/23/14 - Joshua Vaughan - joshua.vaughan@louisiana.edu
#       - Added reading gps_heading
#       - Began to switch from print statements to true logging
#   * 05/25/14 - Joshua Vaughan - joshua.vaughan@louisiana.edu
#       - Changed initial (before real data) values to NaN instead of 0s
#       - Added calculation of speed and heading from GPS - workaround for not getting 
#           that data yet
#       - Improved printing/display of data
#       - TODO: Fix getting heading and speed data from GPS
#
##########################################################################################

import numpy as np
# from matplotlib.pyplot import *

import os, sys

import serial
import struct
import math
from select import select

import time
import datetime
import csv

import threading, logging

logger = logging.getLogger(__name__)


# Function to ask if IMU is correctly oriented (or other yes/no questions)
# TODO: Move to tools or misc file
def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is one of "yes" or "no".
    """
    valid = {"yes":True,   "y":True,  "ye":True,
             "no":False,     "n":False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                             "(or 'y' or 'n').\n")


"""
    Simple serial (RS232) driver for the CH Robotics UM6 IMU

    NOTE: this driver expects a number of things to be true:
    1. Broadcast mode is enabled
    2. QUATERNION output is enabled
    2. EULER output is enabled
    2. ANGULAR VELOCITY output is enabled
    2. LINEAR ACCEL output is enabled

    Use CHRobotics Serial Interface program 
    to set the appropriate outputs

    see main() example at end of file for usage
            
"""
#----- Do NOT touch this class!!! -----
class Um6Drv:
    
    # data ouput mask values
    DATA_QUATERNION       = 0x01
    DATA_ROLL_PITCH_YAW   = 0x02
    DATA_ANGULAR_VEL      = 0x04
    DATA_LINEAR_ACCEL     = 0x08
    DATA_MAGNETOMETER     = 0x10
    DATA_GPS              = 0x12

    # configuration registers
    UM6_COMMUNICATION   = 0x00

    # configuration data
    # [0-7] broadcast_rate: 0 (20 Hz)
    # [8-10] baud rate: 5 (115200)
    # [11-13] GPS_baud_rate: 6 (N/A)
    # [14] RESERVED
    # [15] Satellite status
    # [16] Satellite summary
    # [17] GPS velocity 
    # [18] GPS relative position
    # [19] GPS position
    # [20] Temperature
    # [21] Covariance
    # [22] Euler angle: 1
    # [23] Quaternion: 1
    # [24] Proc. magnetometer: 1
    # [25] Proc. accelerometer: 1
    # [26] Proc. gyro: 1
    # [27] Raw magnetometer
    # [28] Raw accelerometer
    # [29] Raw gyro 
    # [30] Broadcast mode: 1
    # [31] RESERVED
    
    # Default value, do funky stuff with the magnetometer data
    # UM6_COMMUNICATION_DATA = 0x47c00500
    
    # Magnetometer is RAW
#     UM6_COMMUNICATION_DATA = 0x4Ec00500

    # With GPS - 19200 bps 
    # UM6_COMMUNICATION_DATA = 0x47ceb500 
    
    # With GPS - 9600 bps, *ALL* GPS data, except relative pos  
    # UM6_COMMUNICATION_DATA = 0x47cb8500

    # With GPS - 9600 bps -*ALL* GPS data
    # UM6_COMMUNICATION_DATA = 0x47cf8500

    # With GPS - 9600 bps -GPS position, sat status, heading and vel
    # UM6_COMMUNICATION_DATA = 0x47ca8500
    
    # With GPS - 9600 bps - only GPS position
    UM6_COMMUNICATION_DATA = 0x47c80500
    
    # With GPS - 9600 bps - GPS position and satellite summary
    # UM6_COMMUNICATION_DATA = 0x47c90500

    # configuration registers
    UM6_MISC   = 0x01
    # configuration data
    # [0-26] reserved
    # [27] PPS enabled
    # [28] Quaternion state estimation: 1
    # [29] Auto calibration of gyros
    # [30] EKF uses accelerometer
    # [31] EKF uses magnetometer
    UM6_MISC_DATA = 0x78000000

    UM6_MAG_REF_X = 0x02
    UM6_MAG_REF_Y = 0x03
    UM6_MAG_REF_Z = 0x04

    # data registers
    UM6_RAW_MAG_PROC_XY  = 0x5A
    UM6_RAW_MAG_PROC_Z   = 0x5B
    UM6_GYRO_PROC_XY     = 0x5C
    UM6_GYRO_PROC_Z      = 0x5D
    UM6_ACCEL_PROC_XY    = 0x5E
    UM6_ACCEL_PROC_Z     = 0x5F
    UM6_MAG_PROC_XY      = 0x60
    UM6_MAG_PROC_Z       = 0x61
    UM6_EULER_PHI_THETA  = 0x62
    UM6_EULER_PSI        = 0x63
    UM6_QUAT_AB          = 0x64
    UM6_QUAT_CD          = 0x65
    
    # Added 05/22/14 - joshua.vaughan@louisiana.edu
    UM6_GPS_LONGITUDE    = 0X77
    UM6_GPS_LATITUDE     = 0X78
    # UM6_GPS_COURSE_SPEED = 0X7D
    # UM6_GPS_SAT_SUMMARY  = 0X7E

    # command registers
    CMD_ZERO_GYROS       = 0xAC
    CMD_RESET_EKF        = 0xAD
    CMD_SET_ACCEL_REF    = 0xAF
    CMD_SET_MAG_REF      = 0xB0
    CMD_BAD_CHECKSUM     = 0xFD # sent by UM6 when it receives corrupted pkt

    # set default command callbacks to 'None'
    cmd_cb = { UM6_COMMUNICATION: None,
               CMD_ZERO_GYROS: None,
               CMD_RESET_EKF: None,
               CMD_SET_ACCEL_REF: None,
               CMD_SET_MAG_REF: None,
               CMD_BAD_CHECKSUM: None }

    #index values for packet arrays
    PKT_TYPE_IDX = 0
    PKT_ADDR_IDX = 1
    PKT_DATA_IDX = 2

    # scale factors for register output
    SCALE_QUAT  = 0.0000335693
    SCALE_EULER = 0.0109863
    SCALE_GYRO  = 0.0610352
    SCALE_MAG  =  0.000305176
    SCALE_ACCEL = 0.000183105

    # Threshold (m) for calculating heading and speed from GPS
    # If less than this value, speed is set to 0 and heading nan
    GPS_DISTANCE_THRESHOLD = 2.0
    
    NO_DATA_PACKET       = 0x00

    quaternion = []
    wip_quat = [] # tmp var for building quats from register reads

    valid = {'quaternion': False,
               'mag_x': False,
               'mag_y': False,
               'mag_z': False,
               'lin_acc_x': False,
               'lin_acc_y': False,
               'lin_acc_z': False,
               'ang_vel_x': False,
               'ang_vel_y': False,
               'ang_vel_z': False,
               'yaw': False,
               'roll': False,
               'pitch': False,
               'latitude': False,
               'longitude': False,
               'gps_heading': False,
               'gps_speed': False }
    


    def __init__(self, port, outputDataMask, cb=None):
        self.ser = serial.Serial(port, 
                                    baudrate=115200, 
                                    bytesize=8, parity='N', 
                                    stopbits=1, timeout=None)
        self.ser.flushInput()
        self.ser.flushOutput()
        self.buf = ""
        self.data_callback = cb
        self.output = outputDataMask
        
        # Assign nan gps data; we don't to wait for a lock before processing IMU
        self.latitude = np.nan
        self.longitude = np.nan
        self.last_latitude = np.nan
        self.last_longitude = np.nan
        self.gps_heading = np.nan
        self.gps_speed = np.nan
        self.time = np.nan
        self.curr_time = time.time()
        self.last_time = self.curr_time

    def __del__(self):
        logging.info('Closing serial port...\n')
        self.ser.close()

    def update(self):
        pkt = self.readPacket()
        if (pkt != Um6Drv.NO_DATA_PACKET):       
            self.decodePacket(pkt)
        else:
            time.sleep(0.01)

    def waitData(self, nbytes=1, timeout=1.0):
        while True:
            self.buf += self.ser.read(self.ser.inWaiting())
            if len(self.buf)>=nbytes:
                return True
            rlist, _, _ = select([ self.ser ], [], [], timeout)
            if rlist<=0:
                return False
            # Now sleep for the expected time necessary to receive these bits
            time.sleep(1e-4*(nbytes-len(self.buf)))

    def updateBlocking(self, timeout=1.0):
        rlist, _, _ = select([ self.ser ], [], [], timeout)
        if rlist:
            self.update()

    def syncToHeader(self):
        if self.waitData(3):
            idx = self.buf.find("snp")
            if idx>=0:
                self.buf = self.buf[idx+3:]
                return True
        return False 

    def sendConfig(self, reg, data, callback):
        """ Sends a 4 byte configuration message """
        self.cmd_cb[reg] = callback
        pkt = [0x80,reg]
        for i in [3,2,1,0]:
            # Splits input into bytes
            pkt.append((data & (0xFF << 8*i)) >> 8*i)
        self.writePacket(pkt)

    def sendCommand(self, cmd, callback):
        self.cmd_cb[cmd] = callback
        pkt = [0, cmd]
        self.writePacket(pkt)

    def writePacket(self, pkt):

        #header
        self.ser.write('s')
        self.ser.write('n')
        self.ser.write('p')

        chkSum = (self.chToByte("s") +
                  self.chToByte("n") +
                  self.chToByte("p"))

        for i in range(0, len(pkt)):
            self.ser.write(chr(pkt[i]))
            chkSum += pkt[i]

        strChkSum = ("%s"%(chkSum))

        high = (chkSum >> 8) & 0x00FF
        low = 0x00FF & chkSum

        self.ser.write(chr(high))
        self.ser.write(chr(low))

    def readPacket(self):
        if (not self.syncToHeader()):
            return Um6Drv.NO_DATA_PACKET
        if not self.waitData(2):
            return Um6Drv.NO_DATA_PACKET

        packetType = self.chToByte(self.buf[0])

        hasData = packetType >> 7
        isBatch = (packetType >> 6) & 0x01
        batchLen = (packetType >> 2) & 0x0F
        
        logging.debug('Packet Type = %s'.format(bin(packetType)))
        logging.debug('hasData=%s, isBatch=%s, batchLen=%s' % (hasData,isBatch,batchLen))
        
        addr = self.chToByte(self.buf[1])

        calcChkSum = (self.chToByte("s") +
                      self.chToByte("n") +
                      self.chToByte("p") +
                      packetType + addr)

        packet = []
        packet.append(packetType)
        packet.append(addr)

        self.buf = self.buf[2:]
        if not self.waitData(batchLen*4+2):
            return Um6Drv.NO_DATA_PACKET

        for i in range(0, batchLen * 4): # 4 bytes per register
            byte = self.chToByte(self.buf[i])
            packet.append(byte)
            calcChkSum += byte
        self.buf = self.buf[batchLen*4:]

        high = self.chToByte(self.buf[0])
        low  = self.chToByte(self.buf[1]) 

        chkSum = self.bytesToShort(high,low)
        self.buf = self.buf[2:]
        
        logging.debug('Calculated: {:d} -- Actual: {:d}'.format(calcChkSum, chkSum))
        logging.debug(packet)
        
        if (calcChkSum != chkSum):
            logging.error('Read Pkt: BAD CHECKSUM for {:d}/{:x}'.format(addr, addr))
            logging.error('Calculated: {:d} -- Actual: {:d} for Packet: {}'.format(calcChkSum, chkSum, packet))
        
        return packet

    def decodePacket(self, pkt):
        pt = pkt[Um6Drv.PKT_TYPE_IDX] 
        batchLen = (pt >> 2) & 0x0F
        startAddr = pkt[Um6Drv.PKT_ADDR_IDX]
        dataIdx = Um6Drv.PKT_DATA_IDX

        logging.debug('Packet: {:d} -- batchLen: {:d} -- startAddr:{:d} -- dataIdx: {:d}'.format(pt, batchLen, startAddr, dataIdx))
        addr = startAddr
        
        result = 0 == (pt & 0x01)

        try:
            if (self.cmd_cb[addr] is not None):
                self.cmd_cb[addr](addr, result)
        except KeyError:
            self.cmd_cb[addr] = None

        if (addr == Um6Drv.CMD_BAD_CHECKSUM):
            logging.error('Rx Packet: BAD CHECKSUM for {:d}'.format(addr))

        for i in range(0, batchLen):
            self.parseAndSendData(addr+i, pkt[dataIdx+(i*4):dataIdx+((i+1)*4)])

    def parseAndSendData(self, addr, data):
        logging.debug('addr = {:d}'.format(addr))

        
        if (addr == Um6Drv.UM6_GYRO_PROC_XY):
            self.ang_vel_x = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_GYRO)
            self.ang_vel_y = ((self.bytesToShort(data[2],data[3])) * 
                                Um6Drv.SCALE_GYRO)
            self.valid['ang_vel_x'] = True 
            self.valid['ang_vel_y'] = True 
            logging.debug('Assigning x and y ang vel data...')

        if (addr == Um6Drv.UM6_GYRO_PROC_Z):
            self.ang_vel_z = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_GYRO)
            self.valid['ang_vel_z'] = True 
            logging.debug('Assigning z ang vel data...')

        if (addr == Um6Drv.UM6_RAW_MAG_PROC_XY):
            self.mag_x = (self.bytesToShort(data[0],data[1]))
            self.mag_y = (self.bytesToShort(data[2],data[3]))
            self.valid['mag_x'] = True 
            self.valid['mag_y'] = True 
            logging.debug('Assigning RAW x and y mag data...')

        if (addr == Um6Drv.UM6_RAW_MAG_PROC_Z):
            self.mag_z = (self.bytesToShort(data[0],data[1])) 
            self.valid['mag_z'] = True 
            logging.debug('Assigning RAW z-mag data...')

        if (addr == Um6Drv.UM6_MAG_PROC_XY):
            self.mag_x = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_MAG)
            self.mag_y = ((self.bytesToShort(data[2],data[3])) * 
                                Um6Drv.SCALE_MAG)
            self.valid['mag_x'] = True 
            self.valid['mag_y'] = True 
            logging.debug('Assigning x and y mag data...')

        if (addr == Um6Drv.UM6_MAG_PROC_Z):
            self.mag_z = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_MAG)
            self.valid['mag_z'] = True 
            logging.debug('Assigning z-mag data...')

        if (addr == Um6Drv.UM6_ACCEL_PROC_XY):
            self.lin_acc_x = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_ACCEL)
            self.lin_acc_y = ((self.bytesToShort(data[2],data[3])) *
                                Um6Drv.SCALE_ACCEL)
            self.valid['lin_acc_x'] = True 
            self.valid['lin_acc_y'] = True 
            logging.debug('Assigning x and y-accel data...')

        if (addr == Um6Drv.UM6_ACCEL_PROC_Z):
            self.lin_acc_z = ((self.bytesToShort(data[0],data[1])) * 
                                Um6Drv.SCALE_ACCEL)
            self.valid['lin_acc_z'] = True 
            logging.debug('Assigning z-accel data...')

        if (addr == Um6Drv.UM6_QUAT_AB):
            a = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_QUAT
            b = (self.bytesToShort(data[2],data[3]))*Um6Drv.SCALE_QUAT
            self.wip_quat.append(a)
            self.wip_quat.append(b)

        if (addr == Um6Drv.UM6_EULER_PHI_THETA):
            self.roll = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_EULER
            self.pitch = (self.bytesToShort(data[2],data[3]))*Um6Drv.SCALE_EULER
            self.valid['roll'] = True
            self.valid['pitch'] = True
            logging.debug('Assigning roll and pitch data...')

        if (addr == Um6Drv.UM6_EULER_PSI):
            self.yaw = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_EULER
            self.valid['yaw'] = True
            logging.debug('Assigning yaw data...')

        if (addr == Um6Drv.UM6_QUAT_CD):
            c = (self.bytesToShort(data[0],data[1]))*Um6Drv.SCALE_QUAT
            d = (self.bytesToShort(data[2],data[3]))*Um6Drv.SCALE_QUAT
            self.wip_quat.append(c)
            self.wip_quat.append(d)
            self.quaternion = []
            self.quaternion.append(self.wip_quat[0])
            self.quaternion.append(self.wip_quat[1])
            self.quaternion.append(self.wip_quat[2])
            self.quaternion.append(self.wip_quat[3])
            self.wip_quat = []
            self.valid['quaternion'] = True 
            logging.debug('Assigning quaternion data...')
            
        ## Added parsing of GPS data - JEV - 05/22/14
        if (addr == Um6Drv.UM6_GPS_LATITUDE):
            self.latitude = self.bytesToLong(data[0], data[1], data[2], data[3])
            self.valid['latitude'] = True
            logging.debug('Assigning latitude data...')
        
        if (addr == Um6Drv.UM6_GPS_LONGITUDE):
            self.longitude = self.bytesToLong(data[0], data[1], data[2], data[3])
            self.valid['longitude'] = True
            self.curr_time = time.time()
            logging.debug('Assigning longitude data...')

#         TODO: Getting speed and heading directly from GPS doesn't seem to work 
#               06/01/14 - JEV - joshua.vaughan@louisiana.edu        
#         if (addr == Um6Drv.UM6_GPS_COURSE_SPEED):
#             self.gps_heading = self.bytesToShort(data[0],data[1]) / 100
#             self.valid['gps_heading'] = True
#             logging.debug('Assigning gps_heading data...')
#             print 'Assigning gps_heading data...'
#             
#             self.gps_speed = self.bytesToShort(data[2],data[3]) / 100
#             self.valid['gps_speed'] = True
#             logging.debug('Assigning gps_speed data...')

        if (self.valid['quaternion'] == True and
                self.valid['ang_vel_x'] == True and
                self.valid['ang_vel_y'] == True and
                self.valid['ang_vel_z'] == True and
                self.valid['lin_acc_x'] == True and
                self.valid['lin_acc_y'] == True and
                self.valid['lin_acc_z'] == True and
                self.valid['pitch'] == True and
                self.valid['roll'] == True and
                self.valid['yaw'] == True):  # and  # Don't wait for GPS data
#                 self.valid['latitude'] == True and
#                 self.valid['longitude'] == True):#  and
#                 self.valid['gps_heading'] == True and
#                 self.valid['gps_speed'] == True):
#             
            self.valid['quaternion'] = False
            self.valid['ang_vel_x'] = False
            self.valid['ang_vel_y'] = False
            self.valid['ang_vel_z'] = False
            self.valid['lin_acc_x'] = False
            self.valid['lin_acc_y'] = False
            self.valid['lin_acc_z'] = False
            self.valid['pitch'] = False
            self.valid['roll'] = False
            self.valid['yaw'] = False
            # Don't wait for GPS data
#             self.valid['latitude'] = False
#             self.valid['longitude'] = False
#             self.valid['gps_heading'] = False
#             self.valid['gps_speed'] = False

            if (self.latitude <> self.last_latitude or 
                self.longitude <> self.last_longitude):
                # Check if moving, if so assign a speed and heading
                distance_moved = self.calculate_simple_distance(
                                        (self.last_latitude, self.last_longitude),
                                        (self.latitude, self.longitude))
                
                logging.info('Distance moved since last GPS reading: {:.4f}'.format(distance_moved))
                # print 'Distance moved since last GPS reading: {:.4f}'.format(distance_moved)
                
                # For the first time we get GPS data, assign last values equal to current
                if np.isnan(distance_moved):
                    self.last_latitude = self.latitude
                    self.last_longitude = self.longitude
                
                # otherwise, only do calculations if we're above the 
                #   GPS_DISTANCE_THRESHOLD value
                elif distance_moved > Um6Drv.GPS_DISTANCE_THRESHOLD:
                    self.gps_heading = self.calculate_bearing(
                                        (self.last_latitude, self.last_longitude),
                                        (self.latitude, self.longitude))
                
                    self.last_latitude = self.latitude
                    self.last_longitude = self.longitude
                
                    delta_time = self.curr_time - self.last_time
                
                    if delta_time > 0.0:
                        self.gps_speed = distance_moved / delta_time
                        self.last_time = time.time()
                else:
                    self.gps_heading = np.nan
                    self.gps_speed = 0.0


            results = {}
            if ((self.output & Um6Drv.DATA_QUATERNION) != 0):
                results['DATA_QUATERNION'] = self.quaternion
            if ((self.output & Um6Drv.DATA_ANGULAR_VEL) != 0):
                results['DATA_ANGULAR_VEL'] = [self.ang_vel_x, 
                                               self.ang_vel_y, 
                                               self.ang_vel_z]
            if ((self.output & Um6Drv.DATA_MAGNETOMETER) != 0):
                results['DATA_MAGNETOMETER'] = [self.mag_x, 
                                               self.mag_y, 
                                               self.mag_z]
            if ((self.output & Um6Drv.DATA_LINEAR_ACCEL) != 0):
                results['DATA_LINEAR_ACCEL'] = [self.lin_acc_x, 
                                                self.lin_acc_y, 
                                                self.lin_acc_z]
            if ((self.output & Um6Drv.DATA_ROLL_PITCH_YAW) != 0):
                results['DATA_ROLL_PITCH_YAW'] = [self.roll, 
                                                  self.pitch, 
                                                  self.yaw]
            if ((self.output & Um6Drv.DATA_GPS) != 0):
                results['DATA_GPS'] = [self.latitude, 
                                       self.longitude,
                                       self.gps_heading,
                                       self.gps_speed]
            if (self.data_callback is not None):
                # print results
                self.data_callback(results)


    def bytesToShort(self, high, low):
        # convert string of low and high bytes to signed short
        return struct.unpack("h", chr(low) + chr(high))[0]
    
    def bytesToLong(self, high, high_mid, low_mid, low):
        # convert string of bytes to signed long
        return struct.unpack("f", chr(low) + chr(low_mid) + chr(high_mid) + chr(high))[0]

    def chToByte(self, ch):
        # convert single char string to unsigned byte
        return struct.unpack("B", ch)[0]
        
        
    def calculate_bearing(self,position1, position2):
        ''' Calculate the bearing between two GPS coordinates 
    
        Equations from: http://www.movable-type.co.uk/scripts/latlong.html
    
        Input arguments:
            position1 = lat/long pair in decimal degrees DD.dddddd
            position2 = lat/long pair in decimal degrees DD.dddddd
    
        Returns:
            bearing = initial bearing from position 1 to position 2 in degrees
            
        Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/23/14
    
        Modified:
            *
        
        '''
    
        lat1, long1 = np.deg2rad(position1)
        lat2, long2 = np.deg2rad(position2)
    
        dLon = long2 - long1
    
        y = np.sin(dLon) * np.cos(lat2)
        x = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(dLon)
    
        bearing = (np.rad2deg(np.arctan2(y, x)) + 360) % 360
    
        return bearing


    def calculate_simple_distance(self,position1, position2):
        ''' Calculate the distance between two lat/long coords using simple cartesian math
    
        Equation from: http://www.movable-type.co.uk/scripts/latlong.html
    
        Input arguments:
            position1 = lat/long pair in decimal degrees DD.dddddd
            position2 = lat/long pair in decimal degrees DD.dddddd
    
        Returns:
            distance = distance from position 1 to position 2 in meters
    
    
        Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/24/14
    
        Modified:
            *
    
        '''
    
        R = 6373000        # Radius of the earth in m
    
        lat1, long1 = np.deg2rad(position1)
        lat2, long2 = np.deg2rad(position2)
    
        dLat = lat2 - lat1
        dLon = long2 - long1
    
        x = dLon * np.cos((lat1+lat2)/2)
        distance = np.sqrt(x**2 + dLat**2) * R
    
        return distance
#----- Do NOT touch this class!!! -----


class IMUPoller(threading.Thread):
    def __init__(self, IMU_object):
        threading.Thread.__init__(self, name = 'IMU Thread')
        
        print('Starting IMU Thread...')
        self.IMU_object = IMU_object
        
        self.running = True
        
    def run(self):
        while self.running:
            self.IMU_object.continuous_update()            


class IMU(object):
    ''' Processing of low-level IMU data to return "usable" data'''
    
    def __init__(self, port, dataMask = (Um6Drv.DATA_QUATERNION | 
                                         Um6Drv.DATA_ROLL_PITCH_YAW | 
                                         Um6Drv.DATA_MAGNETOMETER | 
                                         Um6Drv.DATA_LINEAR_ACCEL | 
                                         Um6Drv.DATA_GPS)):
        self.um6 = Um6Drv(port, dataMask, self.handle_IMU_data) 
        
        logging.info('IMU initialization...')
        self.data = {}
        self.first_write = True
        self.okay_to_print = False
        _SETUP_DELAY = 1.0

        # set up communication
        print('Comm. setup...')
        self.um6.sendConfig(Um6Drv.UM6_COMMUNICATION, Um6Drv.UM6_COMMUNICATION_DATA, self.cmd_cb)  
        time.sleep(_SETUP_DELAY)
        
        # do basic calibration
        print('Gyro Calibration... ')
        logging.info('Gyro was calibrated.')
        self.um6.sendCommand(Um6Drv.CMD_ZERO_GYROS, self.cmd_cb)
        time.sleep(_SETUP_DELAY)
        
        # Only run this when the X is pointing at 0deg (N)
        if query_yes_no('Reset the Kalman Filter?', default='no'):
            print('EKF reset...')
            logging.info('Kalman Filter was reset.')
            self.um6.sendCommand(Um6Drv.CMD_RESET_EKF, self.cmd_cb)
            time.sleep(_SETUP_DELAY)
        
        # Only run this when the X is pointing at 0deg (N)
        if query_yes_no('Is the x-axis of the IMU pointed North (0deg)?', default='no'):
            print('MAG reference reset...')
            logging.info('Magnometer reference was reset.')
            self.um6.sendCommand(Um6Drv.CMD_SET_MAG_REF, self.cmd_cb)
            time.sleep(_SETUP_DELAY)
        
        
        print('Accel reference reset...')
        logging.info('Acceleromoter reference was reset.')
        self.um6.sendCommand(Um6Drv.CMD_SET_ACCEL_REF, self.cmd_cb)
        
        time.sleep(_SETUP_DELAY)
        
        
    def __del__(self):
        self.um6.__del__()
        
    def cmd_cb(self, cmd, result): 
        logging.debug("Command %s Success: %s"%(cmd, result))
        
    def setup_data_file(self):
        ''' Set up the csv file to write to. 
         The filename contains a date/time string of format imuData_wihtGPS_YYYY-MM-DD_HHMMSS.csv'''
        self.data_filename = 'CH6imuData_withGPS' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')+'.csv'
        self.start_time = time.time()
#         with open(self.data_filename, 'a') as data_file:  # Just use 'w' mode in 3.x
#             writer = csv.DictWriter(data_file, self.data.keys())
#             writer.writeheader()


    def append_to_data_file(self):
#         print 'Writing IMU data...'
        with open(self.data_filename, 'a') as data_file:  # Just use 'w' mode in 3.x
            writer = csv.writer(data_file)
            
            if self.first_write is True:
                            # Define and write the header row immediately after opening
                header = ('Elapsed Time (s)', 
                          'QUATERNION 0', 'QUATERNION 1', 
                          'QUATERNION 2', 'QUATERNION 3', 
                          'Linear Accel X (m/s)', 'Linear Accel Y (m/s)',
                          'Linear Accel Z (m/s)', 
                          'Magnometer X', 'Magnometer Y', 'Magnometer Z', 
                          'Roll (deg)', 'Pitch (deg)', 
                          'Yaw (deg)',
                          'IMU Heading (deg from N)',
                          'Latitude (+/- = North/South)', 'Longitude (+/- = East/West)',
                          'GPS Heading (deg from N)', 'Speed (m/s)')
                writer.writerow(header) 
                self.first_write = False
            
            write_data = [self.data['Time'], 
                          self.data['DATA_QUATERNION'][0], self.data['DATA_QUATERNION'][1], 
                          self.data['DATA_QUATERNION'][2], self.data['DATA_QUATERNION'][3],
                          self.data['DATA_LINEAR_ACCEL'][0], self.data['DATA_LINEAR_ACCEL'][1],
                          self.data['DATA_LINEAR_ACCEL'][2], 
                          self.data['DATA_MAGNETOMETER'][0], self.data['DATA_MAGNETOMETER'][1],
                          self.data['DATA_MAGNETOMETER'][2],
                          self.data['DATA_ROLL_PITCH_YAW'][0], self.data['DATA_ROLL_PITCH_YAW'][1],
                          self.data['DATA_ROLL_PITCH_YAW'][2],
                          self.data['Heading'],
                          self.data['DATA_GPS'][0], self.data['DATA_GPS'][1],
                          self.data['DATA_GPS'][2], self.data['DATA_GPS'][3]]
                
            writer.writerow(write_data)


    def update(self):
        self.um6.update()
        
        
    def handle_IMU_data(self, data):
        ''' function to handle the data resulting from the update() call '''
        # Add time to data dictionary
        data['Time'] = time.time() - self.start_time
        
        # Calculate the heading
        self.heading = self.calculate_heading(data)
        data['Heading'] = self.heading
        
        logging.debug('Handling Data...')
        self.data = data
        self.okay_to_print = True

    def show_imu_data(self):
        if self.okay_to_print:
            # Print IMU data to the screen
            print('')
            print('                    UM6 Data - Combined IMU and GPS                   ')
            print('======================================================================')
            print('')
            print('Elapsed Time (s)                                            {:10.4f}'.format(self.data['Time']))
            print('')
            print('IMU Calculated Heading (deg)                                {:10.4f}'.format(self.data['Heading']))
            print('GPS Calculated Heading (deg)                                {:10.4f}'.format(self.data['DATA_GPS'][2]))
            print(''       
            print('Magnometer - X                                              {:10.4f}'.format(self.data['DATA_MAGNETOMETER'][0]))
            print('Magnometer - Y                                              {:10.4f}'.format(self.data['DATA_MAGNETOMETER'][1]))
            print('Magnometer - Z                                              {:10.4f}'.format(self.data['DATA_MAGNETOMETER'][2]))
            print(''       
            print('Acceleration - X (g)                                        {:10.4f}'.format(self.data['DATA_LINEAR_ACCEL'][0]))
            print('Acceleration - Y (g)                                        {:10.4f}'.format(self.data['DATA_LINEAR_ACCEL'][1]))
            print('Acceleration - Z (g)                                        {:10.4f}'.format(self.data['DATA_LINEAR_ACCEL'][2]))
            print('')
            print('Roll  (deg)                                                 {:10.4f}'.format(self.data['DATA_ROLL_PITCH_YAW'][0]))
            print('Pitch (deg)                                                 {:10.4f}'.format(self.data['DATA_ROLL_PITCH_YAW'][1]))
            print('Yaw   (deg)                                                 {:10.4f}'.format(self.data['DATA_ROLL_PITCH_YAW'][2]))
            print('')
            print('Latitude (+/- = North/South)                                {:10.4f}'.format(self.data['DATA_GPS'][0]))
            print('Longitude (+/- = East/West)                                 {:10.4f}'.format(self.data['DATA_GPS'][1]))
            print('GPS Calculated Speed (m/s)                                  {:10.4f}'.format(self.data['DATA_GPS'][3]))
            print('')
            print('======================================================================')
        else:
            print('\nWaiting for IMU data...\n')
    
    
    def create_thread(self):
        ''' Creates a thread to read the IMU data so that other processes can happen
        at the same time'''
        print('Creating IMU Thread...')
        self.imu_thread = IMUPoller(self)
        self.imu_thread.daemon = True
        self.imu_thread.start()
    
    
    def continuous_update(self):
        # update UM6 state
        try:
            self.update()
                
        except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
            self.imu_thread.running = False
    
    def stop_thread(self):
        print('\nStopping IMU Thread...')
        self.imu_thread.running = False
        self.imu_thread.join(5)   # wait for the thread to finish with timeout 5s
        
        
    def calculate_heading(self, data):
        ''' function calculates heading assuming X is forward and Z is up '''
        heading = (360.0 - np.arctan2(data['DATA_MAGNETOMETER'][1], 
                                      data['DATA_MAGNETOMETER'][0]) * 180.0/np.pi) % 360 

        return heading





if __name__ == '__main__':
    #-----  Choose the level of logging -----
    # Debug level logging
    # logging.basicConfig(level=logging.DEBUG,
    #                     format='From %(threadName)-10s: %(message)s',
    #                     )

    # Info level logging
    # logging.basicConfig(level=logging.INFO,
    #                     format='From %(threadName)-10s: %(message)s',
    #                     )

    # Only Error level debugging
    # logging.basicConfig(level=logging.ERROR,
    #                     format='From %(threadName)-10s: %(message)s',
    #                     )
    #                     

    # Critical level logging
    logging.basicConfig(level=logging.CRITICAL,
                        format='From %(threadName)-10s: %(message)s',
                        )

    ### EXAMPLE USAGE ###
    print('\nStarting up...')
    
#     what to output in the data callback - Unless specified it defaults to below
#     dataMask = (Um6Drv.DATA_QUATERNION | 
#                 Um6Drv.DATA_ROLL_PITCH_YAW | 
#                 Um6Drv.DATA_MAGNETOMETER | 
#                 Um6Drv.DATA_LINEAR_ACCEL | 
#                 Um6Drv.DATA_GPS)

    IMUport = '/dev/tty.usbserial-FTGSQ1XH'
    imu = IMU(IMUport)

    imu.setup_data_file()    
    imu.create_thread()
    
    try:
        while True:
            # Clear the terminal (optional)
            os.system('clear')
            
            imu.show_imu_data()
            
            if imu.data <> {}:
                imu.append_to_data_file()

            # 20Hz update (GPS data will be provided, but update at a slower rate)
            time.sleep(0.05)
            
    except (KeyboardInterrupt, SystemExit):  # when you press ctrl+c
        imu.stop_thread()
        imu.__del__()
