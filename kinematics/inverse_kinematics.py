'''In this exercise you need to implement inverse kinematics for NAO's legs
* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from scipy.linalg import pinv
from math import atan2
from scipy.optimize import fmin
import time




class InverseKinematicsAgent(ForwardKinematicsAgent):

    def from_trans(self,m):
        return [m[0, -1], m[1, -1], m[2, -1], atan2(m[1, 0], m[0, 0])]

    def error_func(self, joint_angles, target, effector_name):
        test_angles = {}
        for name in self.perception.joint:
            test_angles[name] = self.perception.joint[name]

        Ts = [identity(len(self.chains[effector_name]) - 1)]

        k = 0
        for name in self.chains[effector_name]:
            if k == len(self.chains[effector_name]) - 1:
                break
            test_angles[name] = joint_angles[k]
            k += 1

        forward = self.forward_kinematics(test_angles)

        for name in self.chains[effector_name]:
            Ts.append(forward[name])

        Te = np.matrix([self.from_trans(Ts[-1])]).T
        e = target - Te

        return np.linalg.norm(e)


    def inverse_kinematics(self, effector_name, transform):
        joint_angles = np.random.random(len(self.chains[effector_name]) - 1)


        target = np.matrix([self.from_trans(transform)]).T
        #target = matrix([[x_e, y_e, theta_e]]).T
        func = lambda t: self.error_func(t, target, effector_name)

        result = fmin(func, joint_angles)  
        return result

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        names = []
        times = []
        keys = []
        self.forward_kinematics(self.perception.joint)

        angles = self.inverse_kinematics(effector_name, transform)
        print angles

        j = 0
        for joint in self.chains[effector_name]:
            if j == len(angles):
                break
            #print joint
            names.append(joint)
            times.append([2.0])
            keys.append([[angles[j],[3,0.0,0.0],[3,0.0,0.0]]])
            j += 1

        self.keyframes = (names, times, keys) # the result joint angles have to fill in


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
            
        self.keyframes = ([],[],[])

        print self.keyframes

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[0,-1] = 0
    T[1,-1] = 100
    T[2,-1] = -100
    agent.set_transforms('LLeg', T)
    agent.run()