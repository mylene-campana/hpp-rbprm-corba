#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import spiderman_cube_infiniteU_path as tp

packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 0 # tp.ecsSize


fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody )
rr = tp.Viewer (psf); gui = rr.client.gui


heuristicName = "static"
rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./Spiderman_rleg.db',rLegId,heuristicName)
fullBody.addLimbDatabase('./Spiderman_lleg.db',lLegId,heuristicName)
fullBody.addLimbDatabase('./Spiderman_rarm.db',rarmId,heuristicName)
fullBody.addLimbDatabase('./Spiderman_larm.db',larmId,heuristicName)
print("Limbs added to fullbody")


extending = [0, 0, 0, 1, 0, 0, 0, 0.8, 0.0, 0, -0.6, 0.0, 0,0.4, -0.9, 0.9, 0, 0, 0.0, 0.0, 0.4, 0.9, -0.9, 0, 0, 0.0,-0.0, -2, -0.5, 0.3, 2.2, 0.7, 0, 0.0, -2, 0.5, -0.3, 2.2, 0.7, 0.0, 0.0]
q_contact_takeoff = [0, 0, 0, 1, 0, 0, 0,  0.0, 0, 0, 0.0, 0.0, 0.0, -2, 0.3, -0.3, 0, -0.6, 0.0, 0.0, -2, -0.3, 0.3, 0, 0.6, 0.0,-0.0, -1.9, -0.3, -0.2, 1.9, -0.6, 0, 0.0, -1.9, 0.3, 0.2, 1.9, -0.6, 0, 0.0]
flexion = [0, 0, 0, 1, 0.0, 0.0, 0.0, 0.7, 0, 0, -0.7, 0.0,0, 0.5, 0.7, 0.5, 0, -0.6, 0.0, 0.0, 0.5, -0.7, -0.5, 0,0.6, 0.0, -0.0, -1.2, -0.3, -0.2, 2.2, -0.9, 0, 0.0, -1.2,0.3, 0.2, 2.2, -0.9, 0, 0.0]
q_contact_landing = []

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")
fullBody.setPose (q_contact_landing, "landingContact")


#id = r.client.gui.getWindowID("window_hpp_")
#rr.client.gui.attachCameraToNode("spiderman/Thorax",id)

confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = flexion; q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.solutionPathId # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
trunkPathwaypoints = ps.getWaypoints (entryPathId)


q_init[0:confsize-ecsSize] = trunkPathwaypoints[0][0:confsize-ecsSize]
q_goal[0:confsize-ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[0][confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][confsize-ecsSize:confsize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, False); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp reversed
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, False); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)


fullBody.setStartState(q_init_test,[rLegId,lLegId])
fullBody.setEndState(q_goal_test,[rLegId,lLegId])

psf.setPlannerIterLimit (50)

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, 0.005)



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
rr.addLandmark('spiderman/SpidermanLHandSphere',1)

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

""" # without solving path
q_init[0:confsize-ecsSize] = tp.q11[0:confsize-ecsSize]
q_goal[0:confsize-ecsSize] = tp.q22[0:confsize-ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = tp.q11[confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = tp.q22[confsize-ecsSize:confsize]

dir_init = [0,0,-1]; dir_goal = [0,0, 1]"""
