#!/usr/bin/env python
"""
@@@ warning @@@
U can control the local position of the drone
actually the problem is when u exit the controller
then from next time u run this then the local x, y pos are different
so u need to know if u run this again then u need to check 
the local position of the drone and fix it near to the origin local position
"""
from __future__ import print_function

import threading

import roslib
import rospy
import mavros

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode, CommandBool

import sys, select, termios, tty

msg = """
--------------------------
"POSE CONTORLLER"
--------------------------
offboard            : q
arming              : e

 +x            +z(i) 
 ^              ^             
 |   w      < j | l > 
 | a s d        v  
 @----> -y     -z(k)
CTRL-C to quit------------
--------------------------
"""

moveBindings = {
        'w':(1,0,0,0),
        's':(-1,0,0,0),
        'a':(0,1,0,0),
        'd':(0,-1,0,0),
        'i':(0,0,1,0),
        'k':(0,0,-1,0),
        'j':(0,0,0,1), 'l':(0,0,0,-1),
        'q':(0,0,0,0), 'e':(0,0,0,0),
}


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def arming(bool_):
    """
    arming(0) >> disarming
    arming(1) > arming
    """
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arming(value=bool_)

def set_mode(mode):
    """
    There are many modes u can set
    set_mode("OFFBOARD") >> Offboard mode
    
    MANUAL, ACRO, ALTCTL, POSCTL,
    OFFBOARD, STABILIZED, RATTITUDE,
    AUTO.MISSION ,AUTO.LOITER, AUTO.RTL,
    AUTO.LAND, AUTO.RTGS, AUTO.READY,
    AUTO.TAKEOFF 
    """
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_mode(0, mode)
    
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=3)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, yaw):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):
        pos = PoseStamped()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            pos.pose.position.x = pos_x
            pos.pose.position.y = pos_y
            pos.pose.position.z = pos_z
            q = quaternion_from_euler(0, 0, yaw) # RPY
            pos.pose.orientation.x=q[0]
            pos.pose.orientation.y=q[1]
            pos.pose.orientation.z=q[2]
            pos.pose.orientation.w=q[3]
            self.condition.release()

            # Publish.
            self.publisher.publish(pos)

            

        # Publish stop message when thread exits.
        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 0
        pos.pose.orientation.x=0
        pos.pose.orientation.y=0
        pos.pose.orientation.z=0
        pos.pose.orientation.w=1
        self.publisher.publish(pos)



if __name__=="__main__":
    msg_cnt = 0
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_setpoint_keyboard')
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    repeat = rospy.get_param("~repeat_rate", 20)
    pub_thread = PublishThread(repeat)
    pos_x = 0.0
    pos_y = 0.0
    pos_z = 0.0
    yaw = 0.0

    print(msg)
    txt = ""
    txt += 'x : ' + str(pos_x)
    txt += ', y : ' + str(pos_y)
    txt += ', z : ' + str(pos_z)
    txt += ', deg : ' + str(yaw*180/3.1415)
    print(txt, end="")

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(pos_x, pos_y, pos_z, yaw)
        while(1):
            key = getKey(key_timeout)
            
            if key == 'q':
                set_mode("OFFBOARD")
            if key == 'e':
                arming(1)
            if key == 'm':
                set_mode("MANUAL")


            if key in moveBindings.keys():
                pos_x += moveBindings[key][0] * 0.1
                pos_y += moveBindings[key][1] * 0.1
                pos_z += moveBindings[key][2] * 0.1
                yaw += moveBindings[key][3] * 3.1415 / 20          
                if pos_z <= 0 : pos_z = 0
                

                

            else:
                # stopped.
                if (key == '\x03'):
                    break
                
            pub_thread.update(pos_x, pos_y, pos_z, yaw)
            print(msg)
            print('pos_x : ',pos_x)
            print('pos_y : ',pos_y)
            print('pos_z : ',pos_z)
            print('yaw : ',yaw*180/3.1415)
            
    except Exception as e:
        print(e)

    finally:

        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
