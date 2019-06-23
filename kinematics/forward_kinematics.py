'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
import math
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        # YOUR CODE HERE
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'], 
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'], 
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'], 
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'], 
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                       }

        self.links = {'HeadYaw': (0.0, 0.0, 126.5), 
                      'LShoulderPitch': (0.0, 98.0, 100.0), 'LElbowYaw': (105.0, 15.0, 0.0), 'LWristYaw': (55.95, 0.0, 0.0), 
                      'RShoulderPitch': (0.0, -98.0, 100.0), 'RElbowYaw': (105.0, -15.0, 0.0), 'RWristYaw': (55.95, 0.0, 0.0), 
                      'LHipYawPitch': (0.0, 50.0, -85.0), 'LKneePitch': (0.0, 0.0, -100.0), 'LAnklePitch': (0.0, 0.0, -102.9),
                      'RHipYawPitch': (0.0, -50.0, -85.0), 'RKneePitch': (0.0, 0.0, -100.0), 'RAnklePitch': (0.0, 0.0, -102.9) 
                      }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        #get sin and cos of joint angle
        sin = math.sin(joint_angle)
        cos = math.cos(joint_angle)
        #get link values for joint
        joint_links = (0.0, 0.0, 0.0)
        if joint_name in self.links:
            joint_links = self.links[joint_name]
        
        #Check if X,Y or Z and set T according to 3D forward kinematics
        if(joint_name.endswith("Roll")):
            T = matrix([
                [1, 0, 0, 0],
                [0, cos, -sin, 0],
                [0, sin, cos, 0],
                [joint_links[0], joint_links[1], joint_links[2], 1]
                ])

        if(joint_name.endswith("Pitch")):
            T = matrix([
                [cos, 0, sin, 0],
                [0, 1, 0, 0],
                [-sin, 0, cos, 0],
                [joint_links[0], joint_links[1], joint_links[2], 1]
                ])

        if(joint_name.endswith("Yaw")):
            T = matrix([
                [cos, sin, 0, 0],
                [-sin, cos, 0, 0],
                [0, 0, 1, 0],
                [joint_links[0], joint_links[1], joint_links[2], 1]
                ])

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                #Some joints are not in the array (for Example RWristYaw)
                if joint in joints:
                    angle = joints[joint]
                    Tl = self.local_trans(joint, angle)
                    # YOUR CODE HERE
                    T = T * Tl
                    self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
