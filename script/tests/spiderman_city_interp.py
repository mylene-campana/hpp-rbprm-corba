#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import spiderman_city1_path as tp



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 0 # tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-140, 120, -80, 65, 10, 170])
#fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize) 
#fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

#ps = ProblemSolver( fullBody ); r = Viewer (ps)
r = tp.r; ps = tp.ps

psf = tp.ProblemSolver( fullBody )
rr = tp.Viewer (psf); gui = rr.client.gui


# test heuristic
"""
q_jump= fullBody.getCurrentConfig()
q_jump = [-11.6, 38.5, 121.17, 0.9659, 0, 0.25881, 0, 0, 0, 0.2,
 0.0, 0.0, 0.0, 0.4, 0.5, 0.7, 0, -0.6, 0.0, 0.0, 0.4, 0.5,
 0.7, 0, -0.6, 0.0, 0.0, -0.2, 0.3, -1.9, 1.9,-0.6, 0, -0.2, 0.3,
 -1.9, 1.9, -0.6, 0] #; rr(q_jump)

q_feet = q_jump[27:32]
q_arm = q_jump[13:19]


fullBody.addRefConfigAnalysisWeight(q_feet,"RefPoseFeet",[1.,1.,1.,5.,1.,1.])
fullBody.addRefConfigAnalysis(q_arm,"RefPoseArm")
"""

#~ AFTER loading obstacles
nbSamples = 5000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.08 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_rx'
rfoot = 'SpidermanRFootSphere'
rLegx = x; rLegy = y
fullBody.addLimbDatabase('./Spiderman_rleg.db',rLegId,'static')

lLegId = 'lfoot'
lLeg = 'LThigh_rx'
lfoot = 'SpidermanLFootSphere'
lLegx = x; lLegy = y
fullBody.addLimbDatabase('./Spiderman_lleg.db',lLegId,'static')

rarmId = 'rhand'
rLeg = 'RHumerus_rx'
rfoot = 'SpidermanRHandSphere'
rarmx = x; rarmy = y
fullBody.addLimbDatabase('./Spiderman_rarm.db',rarmId,'static')

larmId = 'lhand'
lLeg = 'LHumerus_rx'
lfoot = 'SpidermanLHandSphere'
larmx = x; larmy = y
fullBody.addLimbDatabase('./Spiderman_larm.db',larmId,'static')


print("Limbs added to fullbody")

extending = [0, 0, 0, 1, 0, 0, 0, 0.8, 0.0, 0, -0.6, 0.0, 0,0.4, -0.9, 0.9, 0, 0, 0.0, 0.0, 0.4, 0.9, -0.9, 0, 0, 0.0,-0.0, -2, -0.5, 0.3, 2.2, 0.7, 0, 0.0, -2, 0.5, -0.3, 2.2, 0.7, 0.0, 0.0]

flexion = [0, 0, 0, 1, 0, 0, 0,  0.0, 0, 0, 0.0, 0.0, 0.0, -2, 0.3, -0.3, 0, -0.6, 0.0, 0.0, -2, -0.3, 0.3, 0, 0.6, 0.0,-0.0, -1.9, -0.3, -0.2, 1.9, -0.6, 0, 0.0, -1.9, 0.3, 0.2, 1.9, -0.6, 0, 0.0]
q_contact = [0, 0, 0, 1, 0.0, 0.0, 0.0, 0.7, 0, 0, -0.7, 0.0,0, 0.5, 0.7, 0.5, 0, -0.6, 0.0, 0.0, 0.5, -0.7, -0.5, 0,0.6, 0.0, -0.0, -1.2, -0.3, -0.2, 2.2, -0.9, 0, 0.0, -1.2,0.3, 0.2, 2.2, -0.9, 0, 0.0]



"""
extending[0:7] = q_init_test[0:7] ; rr(extending)
flexion[0:7] = q_init_test[0:7] ; rr(flexion)
q_contact[0:7] = q_init_test[0:7] ; rr(q_contact)

"""
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact, "contact")


#fullBody.runSampleAnalysis( "RefPoseFeet", True) #done in compute_db now


#id = r.client.gui.getWindowID("window_hpp_")
#rr.client.gui.attachCameraToNode("spiderman/Thorax",id)

q_0 = fullBody.getCurrentConfig(); rr(q_0)

confsize = len(tp.q11)-ecsSize
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathId # tp.orientedpathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize] = trunkPathwaypoints[0][0:confsize]
q_goal[0:confsize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize]
#q_init[fullConfSize:fullConfSize+ecsSize] = tp.q11[confsize:confsize+ecsSize]
#q_goal[fullConfSize:fullConfSize+ecsSize] = tp.q22[confsize:confsize+ecsSize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init,[0,0,1], True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp reversed
fullBody.setCurrentConfig (q_goal)
fullBody.isConfigValid(q_goal)
q_goal_test = fullBody.generateContacts(q_goal, [0,0,-1], True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

fullBody.setStartState(q_init_test,[rLegId,lLegId])
fullBody.setEndState(q_goal_test,[rLegId,lLegId])


"""
qe = extending[::]
qe[0:7] = [-2.025129887082707,
40.59097542330351,
128.97577375406138,
 1, 0, 0, 0, 0.0, 0.0, 0.8, 0.0, 0.0, -0.6, -0.9, 0.9, 0.4,
 0, 0, 0.0, 0.0, -0.9, 0.9, 0.4, 0, 0, 0.0, 0.0, 0.5, 0.5,
 -2, 2.2, 0.7, 0, 0.5, 0.5, -2, 2.2, 0.7, 0.0]

rr(qe)
"""



print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(tp.orientedpathId, 0.03)


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=1
pathId = psf.numberPaths () -1
rr(pp.client.problem.configAtParam(pathId,0))
pp(pathId)



"""
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

id = r.client.gui.getWindowID("window_hpp_")
rr.client.gui.attachCameraToNode("spiderman/Skull",id)


import time
pp.dt = 0.01
pp.speed=2
rr(q_init_test)
rr.startCapture ("capture/capture","png")
rr(q_init_test); time.sleep(5)
rr(q_init_test)
pp(psf.numberPaths ()-1)
rr(q_goal_test); time.sleep(2);
rr.stopCapture ()


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

cl = tp.rbprmBuilder.client.basic
plotJointFrame (r, cl, q_init_test, "RFootSphere", 0.15)

q_0 = fullBody.getCurrentConfig()
q = q_0
q [fullBody.rankInConfiguration ['RAnkle_J1']] = 0.6; r(q)
"""

"""
rr.startCapture("screen/spiderman_assymetric_reference","png") ; rr.stopCapture()


"""

