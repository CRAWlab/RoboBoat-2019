# Tuesday Morning Testing
The "script" for Tuesday (June 18) testing at 2019 International RoboBoat. There are two possible courses that we could be assigned. The "script" below diverges based on which course we are assigned

## For Both Courses
### Prior to Leaving Setup Area
* Power on the boat
* Connect all networking 
    - Move to control tent?
    - How to manage transition if not?
* SSH/mosh to Jetson
* Launch Jetson nodes
* SSH/mosh to RaspPi
* Launch RaspPi nodes
* Test throttle
* Test E-stops
* Check that all necessary topics are being published/received
    - echo /imu/data from Jetson
    - echo /fix on Jetson
    - echo /cmd_vel send from Jetson on RaspPi
    - on shore computer(s), echo
        + /imu/data
        + /fix
        + /odom
        + /zed/...  -- single image
        + /zed/...  -- depth
        + /scan
        + /mode


### Hardware Process
* Move to control tent
    - networking
    - power strips
    - laptop power cords
* Walk to dock
* Undo the straps to trailer
* Attach crane
* Get weight measurement
* Let them put the boat in the water
* Keyboard_teleop thrust testing

### Software Process
* Start `rosbag`
* On Raspberry Pi

    python ~/roboboat_ws/src/estop_comms/nodes/estop_comms.py 
    
    python ~/roboboat_ws/src/imu_um7/src/imu_um7_node_v2.py
    
    python3 ~/Test/src/thrust_map/src/RoboBoat_cmd_vel_to_thrusters.py
    
    rostopic pub -r 1 -l /mode std_msgs/String REMOTE
    
* On shore computer acccording to current mode

    rostopic pub -r 1 -l /mode std_msgs/String REMOTE
    
    rostopic pub -r 1 -l /mode std_msgs/String AUTONOMOUS

    rostopic pub -r 1 -l /mode std_msgs/String STOPPED
    
* To check temperature of Raspberry Pi
Using a script based on that at [this link](https://www.cyberciti.biz/faq/linux-find-out-raspberry-pi-gpu-and-arm-cpu-temperature-command/).

The script a bash script:

    #!/bin/bash

    cpu=$(</sys/class/thermal/thermal_zone0/temp)

    echo ""
    echo "$(date) @ $(hostname)"
    echo "------------------------------"
    echo "CPU: temp=$((cpu/1000))'C"
    echo "GPU: $(/opt/vc/bin/vcgencmd measure_temp)"
    echo ""


On the Raspberry Pi, it's saved in `/home/ubuntu/RoboBoat-2019/Python Scripts/Utility Scripts` as `pi_temp.sh` so, it can be run by first `cd`-ing to the proper directory:

    cd "/home/ubuntu/RoboBoat-2019/Python Scripts/Utility Scripts"

then running the command:

    sudo ./pi_temp.sh
