#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import ant_test_path as tp


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "ant"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-6, 0, -2, 2, -0.3, 2.4])
#fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize) # BUG !!
#fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
#ps = ProblemSolver( fullBody )
r = tp.r; ps = tp.ps

psf = tp.ProblemSolver( fullBody )
rr = tp.Viewer (psf); gui = rr.client.gui

#~ AFTER loading obstacles
nbSamples = 10000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.03 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,1]

lfLegId = 'lffoot'
lfLeg = 'ThoraxLFThigh_J1'
lffoot = 'LFFootSphere'
fullBody.addLimb(lfLegId,lfLeg,lffoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

lmLegId = 'lmfoot'
lmLeg = 'ThoraxLMThigh_J1'
lmfoot = 'LMFootSphere'
fullBody.addLimb(lmLegId,lmLeg,lmfoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

lbLegId = 'lbfoot'
lbLeg = 'ThoraxLBThigh_J1'
lbfoot = 'LBFootSphere'
fullBody.addLimb(lbLegId,lbLeg,lbfoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

rfLegId = 'rffoot'
rfLeg = 'ThoraxRFThigh_J1'
rffoot = 'RFFootSphere'
fullBody.addLimb(rfLegId,rfLeg,rffoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)


rmLegId = 'rmfoot'
rmLeg = 'ThoraxRMThigh_J1'
rmfoot = 'RMFootSphere'
fullBody.addLimb(rmLegId,rmLeg,rmfoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

rbLegId = 'rbfoot'
rbLeg = 'ThoraxRBThigh_J1'
rbfoot = 'RBFootSphere'
fullBody.addLimb(rbLegId,rbLeg,rbfoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)
print("Limbs added to fullbody")

q_0 = fullBody.getCurrentConfig(); rr(q_0)


confsize = len(tp.q11)-ecsSize
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
trunkPathwaypoints = ps.getWaypoints (tp.orientedpathId)
q_init[0:confsize] = trunkPathwaypoints[0][0:confsize]
q_goal[0:confsize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize]
#q_init[fullConfSize:fullConfSize+ecsSize] = tp.q11[confsize:confsize+ecsSize]
#q_goal[fullConfSize:fullConfSize+ecsSize] = tp.q22[confsize:confsize+ecsSize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp reversed
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

fullBody.setStartState(q_init_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])
fullBody.setEndState(q_goal_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])

extending = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0]
flexion = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0]
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")

print("Start ballistic-interpolation")
#fullBody.interpolateBallisticPath(tp.solutionPathId, 0.03)
fullBody.interpolateBallisticPath(tp.orientedpathId, 0.03)


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=0.2
pp(psf.numberPaths ()-1)


"""
fullBody.interpolateBetweenStates(state1, state2) # TODO tester
pp(psf.numberPaths ()-1)

fullBody.rotateAlongPath (psf.numberPaths ()-1) # TODO tester

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
pp.speed=0.5
rr(q_init_test)
rr.startCapture ("capture","png")
rr(q_init_test); time.sleep(2)
rr(q_init_test)
pp(psf.numberPaths ()-1)
rr(q_goal_test); time.sleep(2);
rr.stopCapture ()

## ffmpeg commands
ffmpeg -r 30 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
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






# flexion
q [fullBody.rankInConfiguration ['ThoraxLFThigh_J2']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLFShank_J2']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLFFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLMThigh_J2']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLMShank_J2']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLMFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLBThigh_J2']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLBShank_J2']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLBFoot_J2']] = 0.2; rr(q)

q [fullBody.rankInConfiguration ['ThoraxRFThigh_J2']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRFShank_J2']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRFFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRMThigh_J2']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRMShank_J2']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRMFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRBThigh_J2']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRBShank_J2']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRBFoot_J2']] = 0.2; rr(q)

flexion = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0]

# extending
q [fullBody.rankInConfiguration ['ThoraxLFThigh_J2']] = 1.1; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLFShank_J2']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLFFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLMThigh_J2']] = 1.1; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLMShank_J2']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLMFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLBThigh_J2']] = 1.1; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLBShank_J2']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['ThoraxLBFoot_J2']] = 0.2; rr(q)

q [fullBody.rankInConfiguration ['ThoraxRFThigh_J2']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRFShank_J2']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRFFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRMThigh_J2']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRMShank_J2']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRMFoot_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRBThigh_J2']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRBShank_J2']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['ThoraxRBFoot_J2']] = 0.2; rr(q)

extending = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0]

"""

