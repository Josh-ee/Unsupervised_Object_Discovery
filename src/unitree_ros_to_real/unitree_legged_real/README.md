### How to use user-defined ROS messages in Python

We need to import the unitree messages located at `unitree_ros_to_real/unitree_legged_msgs`.
After compiling the workspace, a Python module containing all messages is generated and located at
`<path/to/workspace>/devel/lib/python3/dist-packages`

So, inside our `.py` file we simply need to add the following imports
```Python
import unitree_legged_msgs.msg
```

This imports all messages. Each message is a class and its members can be accessed as follows:
```Python
msg_low_cmd = unitree_legged_msgs.msg.LowCmd()
for ii in range(12):
    print(msg_low_cmd.motorCmd[ii].q)
```

### Dependencies for the ROS Python files in this repo
sudo apt-get install ros-${ROS_DISTRO}-tf

http://wiki.ros.org/plotjuggler

rosrun plotjuggler plotjuggler




* Consider this: http://wiki.ros.org/roslibjs