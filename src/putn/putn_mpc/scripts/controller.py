#!/usr/bin/env python
import numpy as np
import sys
import rospy
import tf
import math
import sys
import select
import os
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int16


class Controller():
    def __init__(self):
        # rospy.init_node('control')
        self.N = 10
        self.rate = rospy.Rate(50)
        self.curr_state = np.zeros(4)
        self.sub1 = rospy.Subscriber(
            '/local_plan', Float32MultiArray, self.local_planner_cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher(
            '/curr_state', Float32MultiArray, queue_size=10)
        self.__timer_localization = rospy.Timer(
            rospy.Duration(0.01), self.get_current_state)
        self.listener = tf.TransformListener()
        # self.input_func = interp1d(np.arange(0, 10),np.arange(0, 10))
        self.have_plan = 0
        self.curr_time = 0
        self.time_sol = 0
        self.local_plan = np.zeros([self.N, 2])
        self.control_cmd = Twist()
        self.control_loop()

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y

    def get_current_state(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform(
                'world', 'base_link', rospy.Time(0))

            self.curr_state[0] = trans[0]
            self.curr_state[1] = trans[1]
            self.curr_state[2] = trans[2]
            roll, pitch, self.curr_state[3] = self.quart_to_rpy(
                rot[0], rot[1], rot[2], rot[3])  # r,p,y
            c = Float32MultiArray()
            c.data = [self.curr_state[0], self.curr_state[1], self.curr_state[2],
                      (self.curr_state[3]+np.pi) % (2*np.pi)-np.pi, roll, pitch]
            self.pub2.publish(c)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def cmd(self, data):
        self.control_cmd.linear.x = data[0]
        self.control_cmd.angular.z = data[1]
        print("control input: ", data)
        self.pub.publish(self.control_cmd)

    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def control_loop(self):
        while not rospy.is_shutdown():
            start_auto = self.manual()
            if(start_auto):
                end_auto = self.auto()
                if not end_auto:
                    break
        self.cmd(np.array([0.0, 0.0]))

    def auto(self):
        while not rospy.is_shutdown():
            key = self.getKey()
            if key == 'q':
                return True
            ref_inputs = self.local_plan[0]
            self.cmd(ref_inputs)
            self.rate.sleep()

    def manual(self):
        data = np.array([0.0, 0.0])
        while not rospy.is_shutdown():
            key = self.getKey()
            if key == 'w':
                if(data[0] < 0.6):
                    data[0] += 0.2
                else:
                    data = data
            elif key == 'x':
                if(data[0] > -0.6):
                    data[0] -= 0.2
                else:
                    data = data
            elif key == 'a':
                if(data[1] < 0.6):
                    data[1] += 0.2
                else:
                    data = data
            elif key == 'd':
                if(data[1] > -0.6):
                    data[1] -= 0.2
                else:
                    data = data
            elif key == 'q':
                if(data[0] < 0.6):
                    data[0] += 0.2
                else:
                    data = data
                if(data[1] < 0.6):
                    data[1] += 0.2
                else:
                    data = data
            elif key == 'e':
                if(data[0] < 0.6):
                    data[0] += 0.2
                else:
                    data = data
                if(data[1] > -0.6):
                    data[1] -= 0.2
                else:
                    data = data
            elif key == 'c':
                if(data[0] > -0.6):
                    data[0] -= 0.2
                else:
                    data = data
                if(data[1] > -0.6):
                    data[1] -= 0.2
                else:
                    data = data
            elif key == 'z':
                if(data[0] > -0.6):
                    data[0] -= 0.2
                else:
                    data = data
                if(data[1] < 0.6):
                    data[1] += 0.2
                else:
                    data = data      
            elif key == 's':
                data = np.array([0.0, 0.0])
            elif key == 'i':
                return True
            elif (key == '\x03'):
                return False
            else:
                data = data
            self.cmd(data)
            self.rate.sleep()

    def local_planner_cb(self, msg):
        for i in range(self.N):
            self.local_plan[i, 0] = msg.data[0+2*i]
            self.local_plan[i, 1] = msg.data[1+2*i]


if __name__ == '__main__':
    rospy.init_node('control')
    controller = Controller()
