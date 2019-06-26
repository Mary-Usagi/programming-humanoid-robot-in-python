'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib

import os
import sys
import numpy as np 
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import *
import threading

import time

print "Connected to localhost"
print ""

class PostHandler(object):
    '''the post hander wraps function to be excuted in parallel
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes_thread(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        ret = self.proxy.execute_keyframes(keyframes)
        if ret is False:
            print "Error: Couldn't execute keyframes"

        return ret

    def set_transform_thread(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        ret = self.proxy.set_transform(effector_name, np.array(transform).tolist())
        if ret is False:
            print "Error: Couldn't set transform"

        return ret

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        keyframe_thread = threading.Thread(target=self.execute_keyframes_thread, args=[keyframes])
        keyframe_thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        transform_thread = threading.Thread(target=self.set_transform_thread, args=[effector_name, transform])
        transform_thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.robot = xmlrpclib.ServerProxy("http://localhost:8000/")
        self.post = PostHandler(self.robot)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE

        angle = self.robot.get_angle(joint_name)
        if angle is False:
            print "Error: Couldn't get angle"

        return angle
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        ret = self.robot.set_angle(joint_name, angle)
        if ret is False:
            print "Error: Couldn't set angle"

        return ret

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        posture = self.robot.get_posture()

        if posture is False:
            print "Error: Couldn't get posture"

        return posture

    def execute_keyframes_nonblocking(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.post.execute_keyframes(keyframes)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        ret = self.robot.execute_keyframes(keyframes)
        if ret is False:
            print "Error: Couldn't execute keyframes"

        return ret

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        transform = self.robot.get_transform(name)

        if transform is False:
            print "Error: Couldn't get transform"

        return transform

    def set_transform_nonblocking(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.post.set_transform(effector_name, transform)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        ret = self.robot.set_transform(effector_name, np.array(transform).tolist())
        if ret is False:
            print "Error: Couldn't set transform"

        return ret
        # YOUR CODE HERE

from numpy.matlib import identity

if __name__ == '__main__':
    agent = ClientAgent()
    #print proxy.system.listMethods()

    print "Current Posture:", agent.get_posture()
    print ""

    print "==== Moving Head ===="
    print "angle before:",agent.get_angle("HeadYaw")

    agent.set_angle("HeadYaw", 1.0)
    time.sleep(0.2)

    print "angle after:",agent.get_angle("HeadYaw")
    

    print ""
    print "==== Waving ===="
    agent.execute_keyframes(hello())

    print ""
    print "==== Wipe forehead ===="
    agent.execute_keyframes(wipe_forehead())

    print ""
    print "==== Bending Knee ===="
    print "transform before:",agent.get_transform("LKneePitch")

    T = identity(4)
    T[0,-1] = 0
    T[1,-1] = 100
    T[2,-1] = -100
    agent.set_transform("LLeg", T)

    print "transform after:",agent.get_transform("LKneePitch")
    print ""
    print "Current Posture:", agent.get_posture()

    print ""
    print "==== Trying to stand up ===="
    agent.execute_keyframes(leftBackToStand())
    # TEST CODE HERE