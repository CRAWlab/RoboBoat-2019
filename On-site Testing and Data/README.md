# On-site Testing 
This folder contains the plan and description of data collected on-site at the 2019 International Roboboat competition.

This is the minimum list of topics that should be present on our system:

topic                                      | Meaning  
------------------------------------------ | -----  
/clock                                     |  
/diagnostics                               |  
/dock                                      | Number of the dock identified  
/fix                                       |  
/flag                                      | Number of the flag identified 
/imu/data                                  |  
/imu/mag                                   |  
/imu/rpy                                   |  
/mode                                      | Current operations mode 0=teleop, 1=auto, 2=e-stop 
/rosout                                    |  
/rosout_agg                                |  
/scan                                      |  
/tf                                        |  
/tf_static                                 |   
/urg_node/parameter_descriptions           |  
/urg_node/parameter_updates                |  
/zed/zed_node/confidence/camera_info       |  
/zed/zed_node/confidence/confidence_image  |  
/zed/zed_node/confidence/confidence_map    |  
/zed/zed_node/depth/camera_info            |  
/zed/zed_node/depth/depth_registered       |  
/zed/zed_node/disparity/disparity_image    |  
/zed/zed_node/left/camera_info             |  
/zed/zed_node/left/image_rect_color        |  
/zed/zed_node/left_raw/camera_info         |  
/zed/zed_node/left_raw/image_raw_color     |  
/zed/zed_node/odom                         |  
/zed/zed_node/parameter_descriptions       |  
/zed/zed_node/parameter_updates            |  
/zed/zed_node/path_map                     |  
/zed/zed_node/path_odom                    |  
/zed/zed_node/point_cloud/cloud_registered |  
/zed/zed_node/pose                         |  
/zed/zed_node/pose_with_covariance         |  
/zed/zed_node/rgb/camera_info              |  
/zed/zed_node/rgb/image_rect_color         |  
/zed/zed_node/rgb_raw/camera_info          |  
/zed/zed_node/rgb_raw/image_raw_color      |  
/zed/zed_node/right/camera_info            |  
/zed/zed_node/right/image_rect_color       |  
/zed/zed_node/right_raw/camera_info        |  
/zed/zed_node/right_raw/image_raw_color    |  
/zed/zed_node/stereo/image_rect_color      |  
/zed/zed_node/stereo_raw/image_raw_color   |  


## Topics to `rosbag`
/imu/data
/imu/rpy
/imu/mag
/cmd_vel
/fix
/zed/zed_node/rgb/image_rect_color
/zed/zed_node/depth/depth_registered 
/zed/point_cloud/cloud_registered
/scan
/odom 
/mode
/dock

# Flags to always add:
Flag                     | Meaning  
------------------------ | ----------------------------------------------------------------------------------  
`--split=10240`          | split into 10240 (10gb) chunks  
`--output-name=FILENAME` | save to FILENAME. Be sure to give the full path if recording to a different location  

So, our `rosbag` commands always begin with the same pieces. We only change how we specify what we want to save/bag:

    rosbag record --output-name=FILENAME --split --size=10240 (remainder of command)

We can then decide how to define the remainder. The options include 

* Listing the topics by what to include, splitting into 10240mb (10gB) chunks, and saving to `FILE_NAME`

        rosbag record --output-name=FILENAME --split --size=10240

* Listing the topics by what to include, splitting into 10240mb (10gB) chunks, and saving to `FILE_NAME`

        rosbag record --output-name=FILENAME --split --size=10240 imu/data imu/rpy imu/mag cmd_vel fix scan odom mode dock tf tf_static zed/zed_node/rgb/image_rect_color zed/zed_node/depth/depth_registered zed/zed_node/point_cloud/cloud_registered

* Using regex to get all the imu data rather than listing it three times, splitting into 10240mb (10gB) chunks, and saving to `FILE_NAME`. Give the abslute

        rosbag record --output-name=FILENAME --split --size=10240 --regex imu/.* cmd_vel fix scan odom mode dock tf tf_static zed/zed_node/rgb/image_rect_color /zed/zed_node/depth/depth_registered zed/zed_node/point_cloud/cloud_registered

* We can also use regex to exclude topics from the list of those matched using the `-x` flag

        rosbag record --output-name=FILENAME --split --size=10240 -x "/wide_stereo(.*)" -x "(.*)/points(.*)"


# Saving to USB drive on Jetson  
The path to the root of the USB flash drive is:

    /media/crawlab/ROBOBOAT/

So, to bag files to the USB drive, the filename should begin with that:

    rosbag record --output-name=/media/crawlab/ROBOBOAT/bag_filename.bag --split --size=10240 (remainder of command)


# Saving to the micro-sd card on the Jetson  
The path to the micro-sd card on the Jetson is:

    /media/crawlab/9016-4EF8/
    
So, to bag files to the USB drive, the filename should begin with that:

    rosbag record --output-name=/media/crawlab/9016-4EF8/bag_filename.bag --split --size=10240 (remainder of command)


