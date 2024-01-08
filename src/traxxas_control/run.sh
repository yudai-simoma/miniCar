# In case roslaunch fails

rosrun rosserial_python serial_node.py /dev/ttyACM0

rosrun topic_tools transform /joy /servo std_msgs/Int16 'int(180*(m.axes[0]+1)/2)'
rosrun topic_tools transform /joy /esc std_msgs/Int16 'int(102*m.axes[3])'

rosrun joy joy_node


# SD card
/media/ubuntu/3C52-B9FD


# ZED
/usr/local/zed/tools


# Realsense
In the /home/ubuntu/git-projects/buildLibrealsense2TX directory I modified the installLiibrealsense.sh so that the version is 2.16.0 (required by the /home/ubuntu/git-projects/traxxas_control/ros/src/realsense2_camera repo)
After that, after `source devel/setup.sh`:
roslaunch realsense2_camera rs_rgbd.launch


# Logitech f310 pad
sudo apt install xboxdrv
i zeby wystawic /dev/input/js0
sudo xboxdrv --detach-kernel-driver


# Roslaunch
roslaunch launch/traxxas.launch
