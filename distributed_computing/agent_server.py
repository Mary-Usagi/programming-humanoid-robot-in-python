'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import sys
import os
import time
import numpy as np

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
server = SimpleXMLRPCServer(("localhost", 8000))

import threading

print "Listening on port 8000..."
server.register_introspection_functions()

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        ret = False
        print ""
        print "==== Incoming request: get_angle ===="
        if joint_name in self.perception.joint:
            ret = self.perception.joint[joint_name]
            print "Returning value \"", str(ret),"\" for joint",joint_name
        else:
            print "Error: \"", joint_name, "\" is not a valid joint"

        print ""
        return ret

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        ret = False

        real_angle = float(angle)

        print ""
        print "==== Incoming request: set_angle ===="
        if joint_name in self.perception.joint:
            self.target_joints[joint_name] = real_angle

            print "Setting angle of joint", joint_name, "to", str(real_angle)
            ret = True
        else:
            print "Error: \"", joint_name, "\" is not a valid joint"

        print ""
        return ret

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE

        print ""
        print "==== Incoming request: get_posture ===="
        print "Robot has posture \"", self.posture,"\""
        print ""
        return str(self.posture)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        # TODO: check if keyframe is correct 
        print ""
        print "==== Incoming request: execute_keyframes ===="
        print "Executing keyframes..."
        print ""
        self.ended = False

        self.keyframes = keyframes

        start_time = time.time()
        current_time = time.time()

        ended = False
        times = keyframes[1]
        while not ended:
            time.sleep(0.05)
            current_time = time.time()

            ended = True
            for i in range(len(times)):
                if current_time - start_time < times[i][len(times[i]) -1]:
                    ended = False
            
        #self.keyframes = ([],[],[])

        #self.keyframes = ([],[],[])

        print ""
        print "...Execution ended."
        print ""
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print ""
        print "==== Incoming request: get_transform ===="
        self.forward_kinematics(self.perception.joint)

        transform = False
        if name in self.transforms:
            transform = self.from_trans(self.transforms[name])
            print "Returning value", transform ,"for joint "+name
        else :
            print "Error: \"", name, "\" is not a valid joint or has no transform"
            print ""
            return False
        print ""

        # convert to native python list
        py_transform = []

        for i in range(len(transform)):
            py_transform.append(np.float64(transform[i]).item())

        return py_transform

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        ret = False

        transform = np.array(transform)
        print ""
        print "==== Incoming request: set_transform ===="
        if effector_name in self.chains:
            print "Setting transform of", effector_name,"..."
            print ""
            try:
                self.set_transforms(effector_name, transform)
                ret = True
            except Exception, e1:
                e2 = sys.exc_info()[0]
                print "Error: Couldn't set transform"
                print "Exception:", e2, e1
                print ""
                return False  
        else:
            print "\"", effector_name, "\" is not a valid effector_name"
            print ""
            return False

        """ 
        start_time = time.time()
        current_time = time.time()

        ended = False
        times = self.keyframes[1]
        while not ended:
            time.sleep(0.05)
            current_time = time.time()

            ended = True
            for i in range(len(times)):
                if current_time - start_time < times[i][len(times[i]) -1]:
                    ended = False

        #print self.keyframes
        self.keyframes = ([],[],[])
        """

        print ""
        print "...Setting transform done"
        print ""

        return ret
        

def server_thread():
    print "Waiting for connections..."
    try:
        server.serve_forever()
    except:
        server.server_close() 
        return

if __name__ == '__main__':
    global agent
    agent = ServerAgent()

    server.register_function(agent.get_angle, "get_angle")
    server.register_function(agent.set_angle, "set_angle")
    server.register_function(agent.get_posture, "get_posture")
    server.register_function(agent.execute_keyframes, "execute_keyframes")
    server.register_function(agent.get_transform, "get_transform")
    server.register_function(agent.set_transform, "set_transform")

    rpc_thread = threading.Thread(target=server_thread)
    rpc_thread.start()

    try:
        agent.run()
    except KeyboardInterrupt:
        print 'Error: Interrupted'
        rpc_thread._Thread__stop()
        sys.exit(0)
    except Exception, e1:
        e2 = sys.exc_info()[0]
        print "Error: Something went wrong. Aborting..."
        print "Exception:", e2, e1
        rpc_thread._Thread__stop()
        sys.exit(0)
    


