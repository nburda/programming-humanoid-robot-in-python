'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello, leftBackToStand
import numpy as np
import pickle
from os import listdir


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('robot_pose.pkl'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        posturedata = []
        # YOUR CODE HERE

        #get pose data
        pose_data = listdir('robot_pose_data')

        #get all data about joints and imu
        posturedata.append(perception.imu[0])
        posturedata.append(perception.imu[1])
        posturedata.append(perception.joint['LHipPitch'])
        posturedata.append(perception.joint['RHipPitch'])
        posturedata.append(perception.joint['LHipRoll'])
        posturedata.append(perception.joint['RHipRoll'])
        posturedata.append(perception.joint['LHipYawPitch'])
        posturedata.append(perception.joint['RHipYawPitch'])
        posturedata.append(perception.joint['LKneePitch'])
        posturedata.append(perception.joint['RKneePitch'])
        
        #restructure data
        posturedata = np.array(posturedata).reshape(1, -1)
        
        #predict and get index of classes
        element = self.posture_classifier.predict(posturedata)
        
        #get posture
        posture = pose_data[element[0]]
        
        #print(posture)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
