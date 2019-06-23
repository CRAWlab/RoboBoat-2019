#!/usr/bin/env python

###############################################################################
# navigation_supervisor.py
#
# Script to run basic GPS-aided navigation at the 2019 RoboBoat contest
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 06/23/19
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

import rospy
import actionlib
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

mode_pub = rospy.Publisher('/mode', String, queue_size=1, latch=True)

# We'll define the course elements withing a dictionary to make them easy to call
# in the script
course = {'A': {'speedgate_waypoint', [29.15166, -81.01732],
                'speedgate_heading', 330.0,
                'autodocking_waypoint', [29.15165, -81.01716],
                'findthepath_waypoint', [29.15148, -81.01705],
                'raisetheflag_waypoint', [29.15158, -81.01710],
                'dock', [29.1513, -81.0175],
                'coursemarker_1', [29.15141, -81.01679],
                'coursemarker_2', [29.15181, -81.01714],
                'coursemarker_3', [29.15164, -81.01740],
                'coursemarker_4', [29.15136, -81.01731]},
          'B', {'speedgate_waypoint', [29.15195, -81.01638],
                'autodocking_waypoint', [29.15208, -81.01660],
                'findthepath_waypoint', [29.15182, -81.01672],
                'raisetheflag_waypoint', [29.15198, -81.01659],
                'dock', [29.1519, -81.0162],
                'coursemarker_1', [29.15173, -81.01675],
                'coursemarker_2', [29.15198, -81.01695],
                'coursemarker_3', [29.15217, -81.01649],
                'coursemarker_4', [29.15195, -81.01633]},
          'C', {'speedgate_waypoint', [29.15154, -81.01610],
                'autodocking_waypoint ', [29.15125, -81.01597],
                'findthepath_waypoint ', [29.15137, -81.01626],
                'raisetheflag_waypoint ', [29.15133, -81.01613],
                'dock', [29.1519, -81.0162],
                'coursemarker_1', [29.15098, -81.01620],
                'coursemarker_2', [29.15139, -81.01637],
                'coursemarker_3', [29.15165, -81.01608],
                'coursemarker_4', [29.15123, -81.01585]}}

# TODO: Update this to match Saturday's successful run
autonav_waypoints = [[(10, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                     [(10, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                     [(10, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]}

speedgate_waypoints = [[(15, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                       [(15, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                       [(0, 10, 0), (0, 0 ,0, 1)],
                       [(0, 0, 0), (0, 0, 0, -1)],
                       [(15, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                       [(15, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]]
                       
# TODO: Add base_link framed waypoint lists for the other tasks


class NavigationSupervisor():
    def __init__(self):
        rospy.init_node('auto_with_GPS')

        # Used by gps_goal
        self.gps_goal_pub = rospy.Publisher('/gps_goal_pose', PoseStamped, queue_size=1)
        
        # Used to publish the current mode of the boat
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=1, latch=True)
        
        # subscribe to fix to get current GPS info
        rospy.Subscriber('/fix', NavSatFix, self.gps_fix_callback)
        
        # Define a start position. It should get updated in the /fix callback
        # as soon as GPS data is published
        self.current_latitude = course['A']['dock'][0]
        self.current_longitude = course['A']['dock'][1] 
        
        # Tolerance on GPS based waypoint to consider it reached
        self.distance_tolerance = 1.0  # meters
        
        # Time to allow for a given move. Gets updated based on the distance
        # between the starting and ending locations
        self.move_timeout = 120.0  # seconds

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
        self.client.wait_for_server()
        

    def form_goal_pose(self, pose):  
        """ Forms a goal pose in the base_link frame """
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'base_link'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]

        return goal_pose

    def gps_fix_callback(self, fix_message):
        """ 
        Handles getting the current location information from the gps fix 
        topic
        """
        self.current_latitude = fix_message.latitude
        self.current_longitude = fix_message.longitude
        

    def calculate_bearing(self, position1, position2):
        """
        Calculate the bearing between two GPS coordinates 
    
        Equations from: http://www.movable-type.co.uk/scripts/latlong.html
    
        Input arguments:
            position1 = lat/long pair in decimal degrees DD.dddddd
            position2 = lat/long pair in decimal degrees DD.dddddd
    
        Returns:
            bearing = initial bearing from position 1 to position 2 in degrees
            
        Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/23/14
        """

        lat1, long1 = np.deg2rad(position1)
        lat2, long2 = np.deg2rad(position2)
    
        dLon = long2 - long1
    
        y = np.sin(dLon) * np.cos(lat2)
        x = np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(dLon)
    
        bearing = (np.rad2deg(np.arctan2(y, x)) + 360) % 360
    
        return bearing


    def calculate_distance(self, position1, position2):
        """
        Calculate the distance between two lat/long coordinates using a unit sphere
    
        Copied from: John Cook at http://www.johndcook.com/python_longitude_latitude.html
    
        Input arguments:
            position1 = lat/long pair in decimal degrees DD.dddddd
            position2 = lat/long pair in decimal degrees DD.dddddd
    
        Returns:
            distance = distance from position 1 to position 2 in meters
    
    
        Modified:
            *Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/23/14
                - Additional commenting
                - Modified to match "theme" of CRAWLAB
                - Inputs change to long/lat array slices
        """
    
        lat1, long1 = position1
        lat2, long2 = position2
    
        R = 6373000        # Radius of the earth in m

        # phi = 90 - latitude
        phi1 = np.deg2rad(90.0 - lat1)
        phi2 = np.deg2rad(90.0 - lat2)
        
        # theta = longitude
        theta1 = np.deg2rad(long1)
        theta2 = np.deg2rad(long2)
        
        # Compute spherical distance from spherical coordinates.
        
        # For two locations in spherical coordinates 
        # (1, theta, phi) and (1, theta, phi)
        # cosine( arc length ) = 
        #    sin phi sin phi' cos(theta-theta') + cos phi cos phi'
        # distance = rho * arc length
    
        cos = (np.sin(phi1) * np.sin(phi2) * np.cos(theta1 - theta2) + 
               np.cos(phi1) * np.cos(phi2))
    
        arc = np.arccos(cos)

        # Multiply arc by the radius of the earth 
        distance = arc * R
    
        return distance

    def form_GPS_pose(self, start_position, goal_position, heading=None):
        """ 
        Forms a desired GPS goal position based on a start position and desired
        final location. The orientation is calculated such that the yaw is 
        aligned with the direction of travel between the two points.
        
        Arguments:
            start_position : (latitude, longitude) pair in decimal degrees
            goal_position : (latitude, longitude) pair in decimal degrees
            heading : heading in degrees - use compass heading 0deg N. 
                      The function converts to ROS desired axes
        
        Returns:
            GPS pose
        """
        
        GPS_goal_pose = MoveBaseGoal()
        GPS_goal_pose.target_pose.header.frame_id = 'map'
        GPS_goal_pose.target_pose.pose.position.x = goal_position[0]
        GPS_goal_pose.target_pose.pose.position.y = goal_position[1]
        GPS_goal_pose.target_pose.pose.position.z = 0.0
        
        # If heading angle is not provided, align the pose heading with the 
        # straight-line path between the two points
        if heading is None:
            bearing = self.calculate_bearing(start_position, goal_position)
        
            # Add 90deg to align the bearing (0deg = N) to the ROS convention on
            # the frame (0deg = E)
            pose_from_bearing = quaternion_from_euler(0, 0, bearing + np.deg2rad(90))

            GPS_goal_pose.target_pose.pose.orientation.x = pose_from_bearing[0]
            GPS_goal_pose.target_pose.pose.orientation.y = pose_from_bearing[1]
            GPS_goal_pose.target_pose.pose.orientation.z = pose_from_bearing[2]
            GPS_goal_pose.target_pose.pose.orientation.w = pose_from_bearing[3]

        else: # Otherwise, use the provided heading for the pose
            # Add 90deg to align the bearing (0deg = N) to the ROS convention on
            # the frame (0deg = E)
            pose_from_heading = quaternion_from_euler(0, 0, heading + np.deg2rad(90))
            
            GPS_goal_pose.target_pose.pose.orientation.x = pose_from_heading[0]
            GPS_goal_pose.target_pose.pose.orientation.y = pose_from_heading[1]
            GPS_goal_pose.target_pose.pose.orientation.z = pose_from_heading[2]
            GPS_goal_pose.target_pose.pose.orientation.w = pose_from_heading[3]
        
        return GPS_goal_pose
        
        
    def go_to_GPS_location(self, goal_position):
        """ 
        Form a GPS pose message, then pass it to the move_base ActionServer
        for execution. Currently (as of 06/23/19), this is a blocking function
        and will block unil the goal is reached.
        
        TODO: 06/23/19 - Add a timeout on reaching the location.
        
        Arguments:
          goal_position : (latitude, longitude) pair in decimal degrees 
        
        """
        
        current_position = (self.current_latitude, self.current_longitude)
        
        GPS_pose = self.form_GPS_pose_yawAligned(current_position,
                                                 goal_position)
        
        # The gps_goal node will wait for the move_base action to complete
        # If we don't wait here and issue another goal before it reaches the 
        # destination this goal ia preempted. So, we stay here until we are 
        # within our define tolerance of the goal or the move times out
        distance_from_goal = self.calculate_distance(current_position,
                                                     goal_position)

        # Set the move timeout to be 5x the time it should take at nominal
        # speed
        self.move_timeout = 5.0 * (distance_from_goal / self.vehicle_speed_nominal)
        
        # Publish the desired location
        self.gps_goal_pub.publish(GPS_pose)

        move_start_time = rospy.Time.now()

        while ((distance_from_goal >= self.distance_tolerance) and
               (rospy.Time.now() - move_start_time < self.move_timeout)):
            
            rospy.sleep(1)  # Sleep 1 second
            
            # Then check the distance again
            distance_from_goal = self.calculate_distance(current_position,
                                                         goal_position)


    def go_to_dock(self, course_letter):
        """ Convenience funciton to go to the dock on the given course
        
        Arguments:
          course_letter : a string representation the course letter the trial is on
        """
        
        goal_position = course['course_letter']['dock']
            
        self.go_to_GPS_location(goal_position)
        
    
    def go_to_speedgate(self, course_letter):
        """ Convenience funciton to go to the speedgate on the given course 
        
        Arguments:
          course_letter : a string representation the course letter the trial is on
        """
        
        goal_position = course['course_letter']['speedgate_waypoint']
        
        if course_letter == 'A':
            heading = course['course_letter']['speedgate_heading']
        else:
            heading = None
            
        self.go_to_GPS_location(goal_position, heading=heading)


    def go_to_autodocking(self, course_letter):
        """ Convenience funciton to go to the dock on the given course 
        
        Arguments:
          course_letter : a string representation the course letter the trial is on
        """
        
        goal_position = course['course_letter']['autodocking_waypoint']
            
        self.go_to_GPS_location(goal_position)


    def go_to_findthepath(self, course_letter):
        """ Convenience funciton to go to the dock on the given course 
        
        Arguments:
          course_letter : a string representation the course letter the trial is on
        """
        
        goal_position = course['course_letter']['findthepath_waypoint']
            
        self.go_to_GPS_location(goal_position)


    def go_to_raisetheflag(self, course_letter):
        """ 
        Convenience funciton to go to the raise the flag area on 
        the given course 
        
        Arguments:
          course_letter : a string representation the course letter the trial is on
        """
        
        goal_position = course['course_letter']['raisetheflag_waypoint']
            
        self.go_to_GPS_location(goal_position)


    def go_through_waypoint_list(self, waypoint_pos_list):
        """
        Cycles through a series of pose waypoints
        
        Note: There is currently no timeout on this function. So it could block
        forever if a waypoint is not reached.
        
        Arugments:
          waypoing_pose_list : A list of poses in
                               [(x_pos, y_pos, z_position), (q.x, q.y, q.z, q.w)]
                               format
        """
        
        for pose in waypoint_post_list:   
            goal = self.form_goal_pose(pose)
            
            rospy.loginfo('Sending goal {}'.format(pose))
            self.client.send_goal(goal)

            # Check the current state
            state = self.move_base.get_state()
            
            # TODO: Add timeout to this loop
            while state != GoalStatus.SUCCEEDED:
                # Wait one second
                rospy.sleep(1.0) 
                
                # I think we should sleep rather than the below. It seems
                # like it would end the pursuit of that goal
                # client.wait_for_result(rospy.Duration(1.0))
            
                # Check the current state
                state = self.move_base.get_state()
                
            rospy.loginfo("Goal succeeded!")



if __name__ == '__main__':
    try:
        # Create an instance of the navigation supervisor
        navigator = NavigationSupervisor()
        
        # Publish the AUTONOMOUS message since we are going into autonomous mode
        navigator.mode_pub.publish('AUTONOMOUS')
        rospy.loginfo('Going autonomous.')
        
        # TODO: Check the order of what you want to try.
        
        # Now, go to the autonavigation with loca, base_link waypoints
        rospy.loginfo('Moving through navigation channel')
        navigator.go_through_waypoint_list(autonav_waypoints)
        
        # Now, move to the speed test using GPS coordinates
        rospy.loginfo('Moving to the speedgate GPS location')
        navigator.go_to_speedgate()
        
        # Then, do the speed test with local, base_link waypoints
        rospy.loginfo('Moving through speed gate task')
        navigator.go_through_waypoint_list(speedgate_waypoints)

        # TODO: What else do you want to do?

    except (KeyboardInterrupt, SystemExit):
        # If we get a keyboard interrupt cancel the goal request and go back
        # to REMOTE on the light tower
        rospy.loginfo('Cancelled goals.')
        client.cancel_goal()
        
        rospy.logdebug('Switching back to REMOTE mode.')
        mode_pub.pulish('REMOTE')


