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

# image
![image1](https://user-images.githubusercontent.com/72853382/99606499-37af0f00-2a4d-11eb-892a-a3bf923be681.png)
As what u can see this is just a keyboard controller which can call service set_mode, arming and publish set_point local pose topic

