#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

import skeleton_desert_path as tp


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
fullBody.setJointBounds ("base_joint_xyz", tp.base_joint_xyz_limits)
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
#ps = ProblemSolver( fullBody )
#r = Viewer (ps)

ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps); gui = r.client.gui; cl = fullBody.client.basic

#r.loadObstacleModel (packageName,"desert","desert")

#~ AFTER loading obstacles

x = 0.01 # contact surface width
y = 0.01 # contact surface length
eps = 0.15

nbSamples = 8000
#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RHip_J1'
rfoot = 'RFootSphere'
rLegOffset = [0,0,0]
rLegNormal = [-eps, 0, math.sqrt(1-eps**2)]
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.1)

lLegId = 'lfoot'
lLeg = 'LHip_J1'
lfoot = 'LFootSphere'
lLegOffset = [0,0,0]
lLegNormal = [-eps, 0, math.sqrt(1-eps**2)]
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,lLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.1)

rarmId = 'rhand'
rarm = 'RShoulder_J1'
rHand = 'RHandSphere'
rArmOffset = [0,0,0]
rArmNormal = [-1,0,0]
rArmx = x; rArmy = y
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.1)

larmId = 'lhand'
larm = 'LShoulder_J1'
lHand = 'LHandSphere'
lArmOffset = [0,0,0]
lArmNormal = [-1,0,0]
lArmx = x; lArmy = y
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.1)


q_0 = fullBody.getCurrentConfig(); r(q_0)
#~ fullBody.createOctreeBoxes(r.client.gui, 1, larmId, q_0)

confsize = len(tp.q11)-ecsSize
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]
fullConfSize = len(q_init)-ecsSize

q_init[0:confsize] = tp.q11[0:confsize]
q_goal[0:confsize] = tp.q22[0:confsize]
q_init[fullConfSize:fullConfSize+ecsSize] = tp.q11[confsize:confsize+ecsSize]
q_goal[fullConfSize:fullConfSize+ecsSize] = tp.q22[confsize:confsize+ecsSize]

fullBody.setCurrentConfig (q_init)
q_init_test = fullBody.generateContacts(q_init, [0,0,1]); r (q_init_test)


fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, [0,0,1]); r (q_goal_test)


fullBody.setStartState(q_init_test,[rLegId,lLegId,rarmId,larmId])
fullBody.setEndState(q_goal_test,[rLegId,lLegId,rarmId,larmId])


#fullBody.generateWaypointContacts(0)

fullBody.interpolateBallisticPath(0)

pp = PathPlayer (fullBody.client.basic, r)
pp(ps.numberPaths ()-1)


#i = 0;
#fullBody.draw(configs[i],r); i=i+1; i-1


q11 [robot.rankInConfiguration ['RElbow_J1']] = -1.5


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



## Compute/Plot SKEL direction from quaternion
epsi = 0.08
plotStraightLine ([-epsi, 0, math.sqrt(1-epsi**2)], [0,0,0], r, "dir")

q = q_init
quatMat = quatToRotationMatrix(q[3:7])
quatMatVect = np.dot(quatMat,np.array([0,0,1]))
skelDir = quatMatVect.tolist ()
plotStraightLine (skelDir, q[0:3], r, "dir")

q = q_goal_test
plotJointFrame (r, cl, q, "RFootSphere", 0.15)
plotVectorInJointFrame (r, cl, q, "RFoot", rLegNormal, [0.8,0.1,0.8,1])

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

## Saved contact configs:
q_init_test= [2.55, 0.8, -1.0, 0.8924, -0.09905, 0.23912, -0.36964, 0.5802941022689296, -0.7678040748805127, -1.9112812887758985, 0.5288470039178176, 0.5122179724943084, -0.12264477229396568, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.144136488810569, -0.28476395798588583, -1.8329507705770391, -0.4446071686879998, -3.020765952006681, 0.14479726330276507, -0.2398157671495898, 0.0, -0.9753266449896946, 0.03187776407050937, -1.111318403593693, -0.6236408784973689, 1.5912222826974178, 0.8708185003942343, -0.1900668550403559, 0.0, 0.0, 0.0, 0.0, 0.04231494579522234, -0.664162196045603, -1.7533159381962216, 1.0911900343156016, 0.30465127667258374, -0.0828398699274074, 0.0, 0.0, 1.0, 0.0]

q_goal_test= [0.3, 1.2, -1.1, -0.08419, 0.25783, 0.02256, -0.96225, -0.13566681453262025, 0.8918321693375578, -2.196990921473544, 1.467302912137039, -0.307320210380855, -0.2043602912470264, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9570603345127786, 0.196343442076341, -1.5933954166534157, -0.23125126289854514, -3.123502117907883, 1.1890110376697733, -0.14475240548335797, 0.0, 0.09907386713361122, -0.22195810920397968, -1.1840582672327937, -0.8400025826859674, 2.1957081621821493, 1.0798181761599677, -0.1864060131442655, 0.0, 0.0, 0.0, 0.0, 0.03089525070343213, 0.6574658894300599, -1.887888834764048, 1.0706277985062902, -0.09322955061174772, -0.1370168167150897, 0.0, 0.0, 1.0, 0.0]

# verify given offset position of contact-point
q = q_goal_test
r(q)
fullBody.setCurrentConfig (q)
posAtester = fullBody.client.basic.robot.computeGlobalPosition(fullBody.client.basic.robot.getJointPosition("RFoot"),[-0.095,0.15,-0.023]); sphereName = "sphi"

r.client.gui.addSphere (sphereName,0.02,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

