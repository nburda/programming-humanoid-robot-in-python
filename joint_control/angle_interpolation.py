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
from keyframes import hello
from keyframes import leftBackToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.starttime = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        #get time of first execution
        if(self.starttime == None):
        	self.starttime = perception.time

        #get data from keyframes
        (names, times, keys) = keyframes

        #get current time relative to first execution
      	currenttime = perception.time - self.starttime

      	#uncomment the following if the angle interpolation should loop (useful for visualizing)
      	#reset time after 1 excution 
        #max = 0
        #for x in range(0, len(times)):
        #	for y in range(0, len(times[x])):
        #		if(times[x][y] > max):
        #			max = times[x][y]

        #if(currenttime > max):
        #	self.starttime = perception.time
        #	currenttime = perception.time - self.starttime

        interpolation = 0

        #do angle interpolation for each joint
        for x in range(0, len(names)):
        	#get data for current joint
        	curname = names[x]
        	curtime = times[x]
        	curkeys = keys[x]

        	target_joints[curname] = 0

        	#check if the joint even exists
        	if not(curname in perception.joint):
        		continue

        	#search for correct time
        	if(curtime[0] > currenttime):
        		#compute interpolation
        		t = currenttime/curtime[0]
        		#current angle
        		P0 = perception.joint[curname]
        		#current angle + left handle of first angle
        		P1 = perception.joint[curname] + curkeys[0][1][2]
        		#target angle
        		P3 = curkeys[0][0]
        		#target angle + left handle of first angle
        		P2 = curkeys[0][0] + curkeys[0][1][2]
        		interpolation = ((1-t)**3) * P0 + 3 * ((1-t)**2)* t * P1 + 3 * (1-t) * (t ** 2) * P2 + (t ** 3) * P3
        	
        	#search for correct time
        	for i in range(1,len(curtime)-1):
        		if(curtime[i] <= currenttime and currenttime < curtime[i+1]):
        			#compute interpolation
        			t = (currenttime - curtime[i])/(curtime[i+1]- curtime[i])
        			#current angle
        			P0 = curkeys[i][0]
        			#current angle + right handle of current angle
        			P1 = curkeys[i][0] + curkeys[i][2][2]
            		#target angle
        			P3 = curkeys[i+1][0]
        			#target angle + left handle of target angle
        			P2 = curkeys[i+1][0] + curkeys[i+1][1][2]
        			interpolation = ((1-t)**3) * P0 + 3 * ((1-t)**2)* t * P1 + 3 * (1-t) * (t ** 2) * P2 + (t ** 3) * P3

        	target_joints[curname] = interpolation
        	
        #print(target_joints)
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
