# mavros_simple_control

When we use px4 and mavros there are some topics that we can publish to contorl the drone
Among these I used setpoint_position

Arming, Offboard, Land, move position...

I don't know bye

if u want u can add other set_point topic or service easily with this

# warning
It woks but when git clone and build then I can't run the py.py
also it doesn't work with you then make it
```
cd ~/catkin_ws/src # if your work space is catkin_ws
catkin_create_pkg mavros_simple_control rospy mavros mavros_msgs geometry_msgs
cd mavros_simple_control
touch py.py
chmod 777 py.py
```
copy and paste the contents of py.py and build and run

# image
![image1](https://user-images.githubusercontent.com/72853382/99606499-37af0f00-2a4d-11eb-892a-a3bf923be681.png)
As what u can see this is just a keyboard controller which can call service set_mode, arming and publish set_point local pose topic
