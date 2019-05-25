'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import *


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = 0 # start of animation
        self.last_time = 0
        self.start_joints = []
        self.current_keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)


    def bezier(self,p0,p1,p2,p3,t):
        return (1-t)**3 *p0 + 3*(1-t)**2 * t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3

        
    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        time_change = 0

        if not keyframes[0]:
            return target_joints

        if self.start_time == 0 or not self.current_keyframes[2] == keyframes[2]:
            print "new keyframe"
            if hasattr(self, 'posture'):
                print self.posture

            self.start_time = perception.time
            self.last_time = perception.time
            self.start_joints = perception.joint
            self.current_keyframes = keyframes
            time_change = 0
        else:
            time_change = perception.time - self.last_time
            self.last_time = perception.time

        names, times, keys = self.current_keyframes

        relative_time = perception.time - self.start_time + time_change
        
        done = 0
        for i in range(len(names)):
            current_frame_id = -1
            for j in range(len(times[i])):
                if relative_time < times[i][j]:
                    current_frame_id = j
                    break
            if relative_time >= times[i][len(times[i]) - 1]:
                target_joints[names[i]] = keys[i][len(keys[i]) - 1][0]# perception.joint[names[i]] # get last angle
                done += 1
                continue

            last_frame_time = 0
            current_frame_time = times[i][current_frame_id] 
            current_key = keys[i][current_frame_id]

            last_frame_id = -1
            if current_frame_id == 0:
                last_frame_time = 0.0
            else:
                last_frame_id = current_frame_id-1
                last_frame_time = times[i][last_frame_id]

            p0 = 0
            p1 = 0
            p2 = 0
            p3 = 0
            if last_frame_id == -1:
                if names[i] in self.joint_names:
                    p0 = self.start_joints[names[i]]
                    p1 = self.start_joints[names[i]]
                else:
                    p0 = 0
                    p1 = 0
            else:
                p0 = keys[i][last_frame_id][0]
                p1 = p0 + keys[i][last_frame_id][2][2]
            p3 = current_key[0]
            p2 = p3 + current_key[1][2]

            max_time = current_frame_time - last_frame_time
            rel_to_last = relative_time - last_frame_time 
            frame_percent = rel_to_last / max_time

            if 1 - frame_percent < 0.05:
                frame_percent = 1
            new_angle = self.bezier(p0, p1, p2, p3, frame_percent)

            target_joints[names[i]] = new_angle

            if False:
                print "---------------------------------"
                print "joint: {}".format(names[i]) 
                print "frame: {}".format(current_frame_id)
                print "rel. time: {}, curr_time: {}, last_time: {}".format(relative_time,current_frame_time,last_frame_time)
                print "t: {}".format(frame_percent)
                print "p0: {}, p1: {}, p2: {}, p3: {}".format(p0,p1,p2,p3)
                print "angle: {}".format(new_angle)

        # waits for new keyframes if done. Comment if repeating not wanted
        if done == len(names):
            self.start_time = 0

        # YOUR CODE HERE

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = wipe_forehead()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
