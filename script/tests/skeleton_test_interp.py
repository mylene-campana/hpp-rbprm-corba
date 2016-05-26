#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer, PathPlayer
from viewer_library import *

import skeleton_test_path as tp



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "skeleton"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = tp.ecsSize

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-8, 6, -2, 2, 0, 3])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
#ps = ProblemSolver( fullBody )
#r = Viewer (ps)

ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps); gui = r.client.gui

#~ AFTER loading obstacles

x = 0.01 # contact surface width
y = 0.01 # contact surface length

nbSamples = 8000
#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RHip_J1'
rfoot = 'RFootSphere'
rLegOffset = [0, 0, 0]
rLegNormal = [0,0,1]
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.01)

lLegId = 'lfoot'
lLeg = 'LHip_J1'
lfoot = 'LFootSphere'
lLegOffset = [0, 0, 0]
lLegNormal = [0,0,1]
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,lLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.01)

q_0 = fullBody.getCurrentConfig(); r(q_0)

fullBody.client.basic.robot.setJointConfig('LShoulder_J1',[1])
fullBody.client.basic.robot.setJointConfig('RShoulder_J1',[-1])
confsize = len(tp.q11)-ecsSize
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]
fullConfSize = len(q_init)-ecsSize

q_init[0:confsize] = tp.q11[0:confsize]
q_goal[0:confsize] = tp.q22[0:confsize]
q_init[fullConfSize:fullConfSize+ecsSize] = tp.q11[confsize:confsize+ecsSize]
q_goal[fullConfSize:fullConfSize+ecsSize] = tp.q22[confsize:confsize+ecsSize]

fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, [0,0,1]); r (q_init_test)


fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, [0,0,1]); r (q_goal_test)


fullBody.setStartState(q_init_test,[rLegId,lLegId])
fullBody.setEndState(q_goal_test,[rLegId,lLegId])


fullBody.generateWaypointContacts(tp.solutionPathId)

[-1.55719,0.845334,0.600855,0.992593,0.00423725,0.0740776,0.0961943,-0.230936,0.949657,-1.43268,0.296132,0.882572,-0.247241,0.0171693,-0.0825482,-0.0419792,-0.0108934,-0.0483608,0.0234773,-0.0109934,0.034957,0.0572652,-0.0578159,0.0612876,-0.0424992,-0.00586502,0.1,-0.0344698,0.1,0.1,-0.1,0,0,0.6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.118115,0.571518,-1.59635,0.875772,0.54584,-0.151522,0,0,0,0]

qt = [2.458539, 0.726329, 0.626443, 0.9946358490305908, -0.03129096926045146, 0.04766404686503832, -0.08630493440345526, 0.1388, -0.0846, 0.2457, -0.0162, 0.08825, 0.09928, -0.032699, -0.02511, -0.004535, -0.0950, 0.0301, 0.00442, 0.03061, 0.00062, -0.08391, 0.00844, 0.08017, 0.07035, 0.07338, -0.5080, -0.3888, -0.0634, 1.1029, -0.1095, 0.0, -0.0, 1.0, 0.0]

r(qt)
fullBody.setCurrentConfig (qt); qtt = fullBody.generateContacts(qt, [math.sqrt(2)/2.0,0,math.sqrt(2)/2.0]); r (qtt)

fullBody.interpolateBallisticPath(tp.solutionPathId)

pp = PathPlayer (fullBody.client.basic, r)
pp(ps.numberPaths ()-1)


q11 [robot.rankInConfiguration ['RElbow_J1']] = -1.5

# verify given offset position of contact-point
q = q_init_test
r(q)
fullBody.setCurrentConfig (q)
#posAtester = fullBody.client.basic.robot.computeGlobalPosition(fullBody.client.basic.robot.getJointPosition(rfoot),[0,0,0.2]); sphereName = "machin2"
posAtester = fullBody.client.basic.robot.computeGlobalPosition(fullBody.client.basic.robot.getJointPosition(rHand),[0.1,0,0]); sphereName = "machin2"


r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()


## Video recording
import time
pp.dt = 0.01
pp.speed=0.6
r(q_init)
r.startCapture ("capture","png")
r(q_init_test); time.sleep(0.2)
r(q_init_test)
pp(ps.numberPaths ()-1)
r(q_goal_test); time.sleep(1);
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 30 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4


## Export path to BLENDER
pathId = 0; dt = 0.01; gui.setCaptureTransform ("skeleton_path.yaml", ["skeleton"])
PL = ps.pathLength(pathId)
FrameRange = np.arange(0,PL,dt)
numberFrame = len(FrameRange)

# test frame capture
q = q_init_test; r (q); gui.refresh (); gui.captureTransform ()
q = q_goal_test; r (q); gui.refresh (); gui.captureTransform ()

# capture path
for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); gui.refresh (); gui.captureTransform ()

r (q_goal); robot.setCurrentConfig(q_goal); gui.refresh (); gui.captureTransform ()



