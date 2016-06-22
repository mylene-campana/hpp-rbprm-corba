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
nbSamples = 2000
x = 0.03 # contact surface width
y = 0.03 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,1]

lfLegId = 'lffoot'
lfLeg = 'ThoraxLFThigh_J1'
lffoot = 'LFFootSphere'
fullBody.addLimb(lfLegId,lfLeg,lffoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

lmLegId = 'lmfoot'
lmLeg = 'ThoraxLMThigh_J1'
lmfoot = 'LMFootSphere'
fullBody.addLimb(lmLegId,lmLeg,lmfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

lbLegId = 'lbfoot'
lbLeg = 'ThoraxLBThigh_J1'
lbfoot = 'LBFootSphere'
fullBody.addLimb(lbLegId,lbLeg,lbfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

rfLegId = 'rffoot'
rfLeg = 'ThoraxRFThigh_J1'
rffoot = 'RFFootSphere'
fullBody.addLimb(rfLegId,rfLeg,rffoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)


rmLegId = 'rmfoot'
rmLeg = 'ThoraxRMThigh_J1'
rmfoot = 'RMFootSphere'
fullBody.addLimb(rmLegId,rmLeg,rmfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

rbLegId = 'rbfoot'
rbLeg = 'ThoraxRBThigh_J1'
rbfoot = 'RBFootSphere'
fullBody.addLimb(rbLegId,rbLeg,rbfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)
print("Limbs added to fullbody")

q_0 = fullBody.getCurrentConfig(); rr(q_0)


confsize = len(tp.q11)-ecsSize
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init may have changed in orientedPath

q_init[0:confsize] = ps.configAtParam(tp.orientedpathId,0)[0:confsize]
q_goal[0:confsize] = ps.configAtParam(tp.orientedpathId,ps.pathLength(tp.orientedpathId))[0:confsize] #tp.q22[0:confsize]
#q_init[fullConfSize:fullConfSize+ecsSize] = tp.q11[confsize:confsize+ecsSize]
#q_goal[fullConfSize:fullConfSize+ecsSize] = tp.q22[confsize:confsize+ecsSize]


dir_init = V0list [0] # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

dir_goal = (-1*np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp reversed
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

fullBody.setStartState(q_init_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])
fullBody.setEndState(q_goal_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])

print("Start ballistic-interpolation")
#fullBody.interpolateBallisticPath(tp.solutionPathId, 0.03)
fullBody.interpolateBallisticPath(tp.orientedpathId, 0.03)


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=0.5
pp(psf.numberPaths ()-1)

## EXTEND GAIT:
q = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; rr(q)

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


rr((-1.85049,1.72708,0.121676,0.882875,-0,0,-0.469608,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00135299,-0.00813696,0.390055,-0.202998,0.0637806,0.149891,0.00262369,0.198911,-0.745635,0.213932,0.766207,-0.0283826,-0.0568401,0.1,0.155055,0.246105,-0.282811,0.133985,-0.00149704,0.018923,0.013833,0.111456,0.5535,0.160529,-0.540398,-0.0326272,-0.190441,-0.0208561,0.134171,0.564777,-0.385487,-0.715292,-0.0643031,-0.0606793,-0.0409097,-0.00705443,0.635377,0.199697,-0.622771,0.018432,-0.115078,-0.014202,))

rr((-1.85049,1.72708,0.121676,0.882875,-0,0,-0.469608,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,-0.977693,0,0,0,0.17578,-0.0200687,-0.0356915,-0.789442,0.017821,0.0528335,-0.0135675,0,0,0,0,0,0,0,-0.165717,0.248323,-0.481317,-0.573927,-0.083776,-0.00329866,0.0366513,0,0,0,0,0,0,0,-0.0360778,0.252139,0.460751,0.221883,-0.0229585,-0.0729576,0.0822674))


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
rr(q_init_test); time.sleep(1)
rr(q_init_test)
pp(psf.numberPaths ()-1)
rr(q_goal_test); time.sleep(1);
rr.stopCapture ()

## ffmpeg commands
ffmpeg -r 32 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
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
"""

