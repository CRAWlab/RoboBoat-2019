#! /usr/bin/env python

###############################################################################
# T200_basic_control.py
#
# Script for basic control of a BlueRobotics T200 thruster via a BlueRobotics
# Basic ESC. The script is set up to control the ESC that is connected to a 
# Adafruit 16-channel PWM hat on a Raspberry Pi. This requires installation of 
# the libraries to communicate with that board.
#
# You should hear three beeps once the ESC is powered on. Then, two more once
# this script initializes it. Then, this script will cycle through forward and 
# reverse throttle.
#
# Relevant Links:
#   T200 - https://www.bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
#   Basic ESC - https://www.bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/
#   Adafruit 16-channel PWM Hat - https://www.adafruit.com/product/2327
#   Adafruit software HowTo - https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/
#
#
# Created: 04/20/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 04/20/19 - JEV - joshua.vaughan@louisiana.edu
#         - 0.4 throttle is being interpreted as zero. Likely due to:
#           + PWM timing in Adafruit library or
#           + range of pulse width set
#         - Account for deadzone around zero throttle
#
###############################################################################

import time

from adafruit_servokit import ServoKit

# Operation Constants
ESC_STARTUP_DELAY = 1.0         # Time to sleep during initialization
MAX_THROTTLE = 50               # Maximum magnitude of throttle to use in test

# TODO: 04/20/19 - JEV 
#       * 0.4 throttle is being interpreted as zero. 
#       * Likely due to:
#           - PWM timing in Adafruit library or
#           - range of pulse width set
THROTTLE_OFFSET = 0.4           # Hack for now

# The 16-channel PWM hat has (surprise!!) 16 channels
kit = ServoKit(channels=16)

# The ESC is connected to channel 0. We can map a name to that channel to make
# the code easier to understand
thruster = kit.continuous_servo[0]

# The BlueRobotics ESC expects pulses in the range 1100-1900us, so we modify
# the range of pulses for the pin connected to the ESC
thruster.set_pulse_width_range(1100, 1900)

# To initialize the thruster, we need to send zero speed for some time
# You should hear two beeps once the ESC is initialized
print('Initializing...')
thruster.throttle = 0.0 + THROTTLE_OFFSET
time.sleep(ESC_STARTUP_DELAY)


try:
    print('Starting positive throttle cycle.')
    for speed in range(MAX_THROTTLE):
        # The throttle command expects commands in the range -1 <= speed <= 1
        # So, we scale by 100
        throttle_command = speed/100

        print('  Current speed: {:+4d}% = {:+4.2f} command'.format(speed, throttle_command))
        thruster.throttle = throttle_command + THROTTLE_OFFSET
        
        # Then sleep a short time before looping
        time.sleep(0.1)

    for speed in range(MAX_THROTTLE):
        # The throttle command expects commands in the range -1 <= speed <= 1
        # So, we scale by 100
        speed = MAX_THROTTLE - speed
        throttle_command = speed/100

        print('  Current speed: {:+4d}% = {:+4.2f} command'.format(speed, throttle_command))
        thruster.throttle = throttle_command + THROTTLE_OFFSET

        # Then sleep a short time before looping
        time.sleep(0.1)


    print('Starting negative throttle cycle.')
    for speed in range(MAX_THROTTLE):
        # The throttle command expects commands in the range -1 <= speed <= 1
        # So, we scale by 100
        throttle_command = -speed/100 # Just the negative of the loop above

        print('  Current speed: {:+4d}% = {:+4.2f} command'.format(-speed, throttle_command))
        thruster.throttle = throttle_command + THROTTLE_OFFSET

        # Then sleep a short time before looping
        time.sleep(0.1)

    for speed in range(MAX_THROTTLE):
        # The throttle command expects commands in the range -1 <= speed <= 1
        # So, we scale by 100
        speed = MAX_THROTTLE - speed
        throttle_command = -speed/100 # Just the negative of the loop above

        print('  Current speed: {:+4d}% = {:+4.2f} command'.format(-speed, throttle_command))
        thruster.throttle = throttle_command + THROTTLE_OFFSET
    
        # Then sleep a short time before looping
        time.sleep(0.1)
    
    # Thruster needs to be stopped
    print('Stopping...')
    thruster.throttle = 0.0 + THROTTLE_OFFSET

finally:
    print('Stopping...')
    # Stop the thruster on any exceptions or program exit
    thruster.throttle = 0.0 + THROTTLE_OFFSET

