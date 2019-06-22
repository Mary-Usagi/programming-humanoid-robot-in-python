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

def from_trans(m):
    return [m[0, -1], m[1, -1], m[2, -1], atan2(m[1, 0], m[0, 0])]

class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        lambda_ = 1
        max_step = 0.1
        joint_angles = np.random.random(len(self.chains[effector_name]))

        target = np.matrix([from_trans(transform)]).T
        #Ts = [identity(len(self.chains[effector_name]))]
        for i in range(1000):
            Ts = [identity(len(self.chains[effector_name]))]
            for name in self.chains[effector_name]:
                Ts.append(self.transforms[name])
                #print k
                #print name
                #print "Ts:", Ts
                #print self.transforms[name]
            	#print "Ts:", len(Ts)
            Te = np.matrix([from_trans(Ts[-1])]).T
            #print "Te:", Te
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.matrix([from_trans(j) for j in Ts[0:-1]]).T
            J = Te - T
            dT = Te - T
            #print "T:", T
            #print "J:", J
            #print "dT:", dT
            J[0, :] = dT[2, :] # x
            J[1, :] = dT[1, :] # y
            J[2, :] = dT[0, :] # z
            #J[2, :] = dT[0, :] # z
            J[-1, :] = 1  # angular

            d_theta = lambda_ * pinv(J) * e
            #print "d_theta: " , d_theta
            joint_angles += np.asarray(d_theta.T)[0]
            if  np.linalg.norm(d_theta) < 1e-4:
                break
                
        return joint_angles

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
            print joint
            names.append(joint)
            times.append([2.0])
            keys.append([[angles[j],[3,0.0,0.0],[3,0.0,0.0]]])
            j += 1

        self.keyframes = (names, times, keys) # the result joint angles have to fill in
        print self.keyframes

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[0,-1] = 0
    T[1,-1] = -100
    agent.set_transforms('RLeg', T)
    agent.run()