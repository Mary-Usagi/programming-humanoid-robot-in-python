'''In this exercise you need to use the learned classifier to recognize current posture of robot
* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`
* Hints:
    Let the robot execute different keyframes, and recognize these postures.
'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import *

import pickle
from os import listdir, path
import numpy as np




class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        # get posture_classifier        
        self.posture_classifier = pickle.load(open('robot_pose.pkl'))  # LOAD YOUR CLASSIFIER       
        self.classes = listdir('robot_pose_data')
        self.joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        print self.posture
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        
        angles = []

        for joint in self.joints:
            angles.append(perception.joint[joint])
        
        angles.append(perception.imu[0])
        angles.append(perception.imu[1])
        angles = np.array(angles).reshape(1, -1)
        
        posture = self.posture_classifier.predict(angles)[0]

        return self.classes[posture]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()