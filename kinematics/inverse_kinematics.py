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


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        #list of joints of effectors
        chains = {'Head': ['HeadYaw', 'HeadPitch'], 
                  'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'], 
                  'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'], 
                  'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'], 
                  'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                 }
        #get X_e, Y_e, Theta_e
        #create empty Matrix
        #calculate new joint angle for each joint in current effector
        for x in chains[effector_name]:
            #get x,y value from angle?
            #create jacobian matrix
            print
        #inverse jacobian matrix
        #use numpy.linalg.solve for inverse jacobian matrix * [X_e, Y_e, Theta_e]
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
