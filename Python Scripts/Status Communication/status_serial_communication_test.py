#! /usr/bin/env python

###############################################################################
# basic_serial_communication.py
#
# script to achieve basic send/receive communication
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 05/27/16
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
###############################################################################

import random
import serial
import time


# serial port will have to change based on configuration
PORT = '/dev/tty.usbmodem1A12101'

# define the serial communication parameters, 8 bits, no parity, 1 stop bit
BPS = 115200

# Define a Timeout for serial communication
TIMEOUT = 0.5

# Open the serial port
ser = serial.Serial(PORT, BPS, timeout=TIMEOUT)

colors = ['red', 'green', 'blue', 'yellow', 'white']

start_time = time.time()
counter = 0

try:
    while True:

        # Change the color status sent every 5 seconds        
        if time.time() - start_time > 5:
            # Choose a random color from our list of colors
            data = colors[counter % len(colors)]
    
            data_string = '{}\n'.format(data).encode('utf-8')
    
            ser.write(data_string)
            # print("Sending: {}".format(data_string))
            
            start_time = time.time()
            counter = counter + 1
        
        # We're assuming here that we get properly terminated lines 
        # over the serial connection. If we are not, either because they
        # are not properly terminated or we aren't receiving any data at
        # all, then this call will block indefinitely.
        line = ser.readline()
        print(line.decode('utf-8'))
        
        # Run the loop roughly every 0.01s, if all data is sent and received
        # without blocking
        time.sleep(0.01)


except (KeyboardInterrupt, SystemExit):
    ser.close()
        