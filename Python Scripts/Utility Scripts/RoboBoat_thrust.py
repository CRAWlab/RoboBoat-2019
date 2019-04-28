#! /usr/bin/env python

###############################################################################
# RoboBoat_thrust.py
#
# Quick calculation for RoboBoat 2019 X-configuration thrust mapping
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 04/27/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 
###############################################################################

import numpy as np

PHI = np.deg2rad(45)          # angle of thruster (rad)

# TODO: 04/27/19 - JEV - Check these values
WIDTH = 11 * 2.54/100         # Approx width to thruster (m)
LENGTH = 13 * 2.54/100        # Approx length to thruster (m)

# Thruster limits from BlueRobotics T-200 datasheet. 16V version here.
MAX_FORWARD_PER_THRUSTER = 5.1 * 9.8066    # N
MAX_REVERSE_PER_THRUSTER = -4.1 * 9.8066   # N

A = np.array([[np.cos(PHI), np.cos(PHI), np.cos(PHI), np.cos(PHI)],
              [-np.sin(PHI), np.sin(PHI), np.sin(PHI), -np.sin(PHI)],
              [LENGTH * np.sin(PHI) + WIDTH * np.cos(PHI), LENGTH * np.sin(PHI) + WIDTH * np.cos(PHI), 
              -LENGTH * np.sin(PHI) - WIDTH * np.cos(PHI), -LENGTH * np.sin(PHI) - WIDTH * np.cos(PHI)]])

# Calculate the maximum posdible net forces/torques in each direction
MAX_SURGE_FORCE = np.dot(A[0,:], MAX_FORWARD_PER_THRUSTER * np.ones(4))
MAX_SWAY_FORCE = np.dot(A[1,:], np.array([MAX_REVERSE_PER_THRUSTER, 
                                          MAX_FORWARD_PER_THRUSTER, 
                                          MAX_FORWARD_PER_THRUSTER, 
                                          MAX_REVERSE_PER_THRUSTER]) * np.sin(PHI))
MAX_YAW_TORQUE = (WIDTH * 2 * MAX_FORWARD_PER_THRUSTER) - (WIDTH * 2 * MAX_REVERSE_PER_THRUSTER)

# Define the desired net forces/torques as a proportion of max
desired_input_percentage = np.array([75, 75, 75])
desired_net_inputs = desired_input_percentage / 100 * np.array([MAX_SURGE_FORCE,
                                                                MAX_SWAY_FORCE,
                                                                MAX_YAW_TORQUE])

# Then, solve for the thruster inputs to generate those
x, residuals, rank, s = np.linalg.lstsq(A, desired_net_inputs)

print('Raw solution = {} N'.format(x))

# TODO: 04/27/19 - JEV - Be more elegant here. We can do this without scaling
#                        twice, as often happens here.
#
# If max needed thrust for any thruster is greater than capable, scale all
# of the trusters equally
if np.max(x) > MAX_FORWARD_PER_THRUSTER: 
    print('Scaling due to forward thrust')
    x = x / (np.max(x) / MAX_FORWARD_PER_THRUSTER)
    print('Scaled solution = {} N'.format(x))

# If max reverse thrust needed for any thruster is greater than capable, scale
# all of the trusters equally
if np.min(x) < MAX_REVERSE_PER_THRUSTER:
    print('Scaling due to reverse thrust')
    x = x / (np.min(x) / MAX_REVERSE_PER_THRUSTER)
    print('Scaled solution = {} N'.format(x))

# TODO: 04/27/19 - JEV - map these Newton values to +/-1 to match hardware 
#                        input range
