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
from mavros_msgs.srv import SetMode, CommandBool

import sys, select, termios, tty, math

init_msg="""
--------------------------
"POSE CONTORLLER"
--------------------------
1. press any key to initialize
the value

2. if there is big gap from z to 0
check your mav

3. press 'n' to start

CTRL-C to quit------------
--------------------------
$$initial pose$$
"""
init = True;
msg = """
--------------------------
"POSE CONTORLLER"
--------------------------
offboard            : q
arming              : e
landing or reset    : r
newvalue            : n
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
        'n':(0,0,0,0),
        'q':(0,0,0,0), 'e':(0,0,0,0), 'r':(0,0,0,0),
        'j':(0,0,0,1), 'l':(0,0,0,-1)
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
    
def callback(msg):

    pub_thread.x = round(msg.pose.position.x,3)
    pub_thread.y = round(msg.pose.position.y,3)
    pub_thread.z = round(msg.pose.position.z,3)
    pub_thread.degree = round(quarterion_to_degree(msg.pose.orientation.z, msg.pose.orientation.w),3)
 

    
def degree_to_quarterion(degree):
    #the degree should be under 180 and over -180
    z = math.sin(math.radians(degree/2))
    w = math.cos(math.radians(degree/2))
    return z, w

def quarterion_to_degree(z, w):
    return math.degrees(math.atan2(2*w*z, 1-2*z*z))
    

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.Z = 0.0
        self.W = 0.0
        self.degree = 0.0
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

    def update(self, x, y, z, degree):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.degree = degree
        Z, W = degree_to_quarterion(degree)
        self.Z = Z
        self.W = W
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
            pos.pose.orientation.z = self.Z
            pos.pose.orientation.w = self.W

            self.condition.release()

            # Publish.
            self.publisher.publish(pos)

        # Publish stop message when thread exits.
        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 1
        self.publisher.publish(pos)
        #print("\r" + txt, end="")
        



if __name__=="__main__":
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
    degree = 0.0
    w = 0.0
    z = 0.0
    pos = PoseStamped()
    #print(init_msg)

    try:
        pub_thread.wait_for_subscribers()
        print(init_msg)
        while(init):
            txt = ""
            txt += 'x : ' + str(pub_thread.x)
            txt += ', y : ' + str(pub_thread.y)
            txt += ', z : ' + str(pub_thread.z)
            txt += ', deg : ' + str(pub_thread.degree)
            print("\r                                       ", end="")
            print("\r" + txt, end="")
            key = getKey(key_timeout)
            
            if key == 'n': 
                init = False
                pos_x = pub_thread.x
                pos_y = pub_thread.y
                degree = pub_thread.degree
                pub_thread.subscriber.unregister()
                break
            else:
                # stopped.
                if (key == '\x03'):
                    break
        
        print(msg)

        
        
        pub_thread.update(pos_x, pos_y, pos_z, degree)
        while(1):
            
            
            
            key = getKey(key_timeout)
            
            if key == 'q':
                set_mode("OFFBOARD")
            if key == 'e':
                arming(1)
                
            if key == 'r':
                pos_z = 0
                set_mode("AUTO.LAND")   
                init = True
            if key == 'm':
                set_mode("MANUAL")


            if key in moveBindings.keys():
                pos_x += moveBindings[key][0] * 0.1
                pos_y += moveBindings[key][1] * 0.1
                pos_z += moveBindings[key][2] * 0.1
                degree += moveBindings[key][3] * 5
                if pos_z <= 0 : pos_z = 0
                if degree > 180 : degree = 179
                if degree <-180 : degree = -179

            else:
                # stopped.
                if (key == '\x03'):
                    break
                
            
            txt = ""
            txt += 'x : ' + str(pos_x)
            txt += ', y : ' + str(pos_y)
            txt += ', z : ' + str(pos_z)
            txt += ', deg : ' + str(pub_thread.degree)
            print("\r                                                    ", end="")
            print("\r" + txt, end="")
            pub_thread.update(pos_x, pos_y, pos_z, degree)

  
            #print('degree : ',mav_status_turn_deg)
        
 

    except Exception as e:
        print(e)

    finally:

        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

