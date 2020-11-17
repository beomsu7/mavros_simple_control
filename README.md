# mavros_simple_control

When we use px4 and mavros there are some topics that we can publish to contorl the drone
Among these I used setpoint_position

Arming, Offboard, Land, move position...

I don't know bye

# warning
It woks but when git clone and build then I can't run the py.py
also it doesn't work with you then make it
```
cd ~/catkin_ws/src # if your work space is catkin_ws
catkin_create_pkg mavros_simple_control rospy mavros mavros_msgs geometry_msgs
cd mavros_simple_control
touch py.py
chmod 777 py.py
``cd ~/catkin_ws/src # if your work space is catkin_ws
catkin_create_package mavros_simple_control rospy mavros mavros_msgs geometry_msgs
cd mavros_simple_control
touch py.py
chmod 777 py.py`
and fill the py.py with copy and paste
