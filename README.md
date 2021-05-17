# mavros_simple_control

When we use px4 and mavros there are some topics that we can publish to contorl the drone  in "OFFBOARD" mode

Arming, Offboard, Land, move position(x, y, z), rotation(only with Z axis)

# warning
mayb some setup is not perfect so
should make the py.py excutable
and this is simple controller with keyboard

The initial position is 0,0,0 so please make your drone near the zero position
```
cd ~/catkin_ws/src # if your work space is catkin_ws
git clone https://github.com/beomsu7/mavros_simple_control
cd mavros_simple_control
chmod 777 py.py
cd ~/catkin_ws && catkin build mavros_simple_control && source devel/setup.bash
```
# explainment
To control the px4 at the offboard side(with remote pc) with mavros   
Then Topics like /mavros/setpoint ~~  is the way to control   
And in my case I used the /mavros/setpoint_raw/local   

The code reference is https://github.com/ros-teleop/teleop_twist_keyboard

# control
- rosrun mavros_simple_control py.py # publish /mavros/setpoint_raw/local   
or
- rosrun mavros_simple_control py_pos.py # publish /mavros/setpoint_position/local   
   
(1) make your drone to OFFBOARD mode - press 'q'   
(2) make your drone arming           - press 'e'   
(3) make your drone take off         - press 'iiiiii..'
(4) move                             - w(+x position), s(-x position), a(+y), d(-y), k(-z), j(+yaw), l(-yaw)   

if you test real drone then   
(1) make the take off height         - press 'iiiii...'   
(2) make the drone arming            - with your controller   
(3) make the drone OFFBOARD mode     - with your controller   
(4) move   
# image
![image1](https://user-images.githubusercontent.com/72853382/99606499-37af0f00-2a4d-11eb-892a-a3bf923be681.png)
As what u can see this is just a keyboard controller which can call service set_mode, arming and publish set_point local pose topic

