#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *
import time

import ant_test_path as tp


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "ant"
urdfSuffix = ""
srdfSuffix = ""
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
fullBody.setFullbodyFrictionCoef (tp.frictionCoef)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr); pp.speed = 0.6
q_0 = fullBody.getCurrentConfig(); rr(q_0)

extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1, 0.0, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, 0.0, -0.5, 0.0, 0.0, 0, 0.0, 0.0, 1.1, 0.0, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0, 0.0, -1.1, 0.0, 0.0, 0.5, 0.0, 0.0, 0, 0.0, 0.0, -1.1, 0.0, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0]
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.4, 0.0, 1.0, -0.5, -0.3, 0.1, -0.2, 0.0, -1.1, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.0, 0.6, -1.1, 0.0, -0.6, 1.1, -0.6, -0.1, 0.0, 0.0, 0.0, 1.0, 0.4, 0.0, -1.0, 0.5, -0.3, -0.1, 0.2, 0.0, 1.1, 0.0, 0.0, -1.1, 0.0, 0.0, 0.0, 0.0, 0.6, 1.1, 0.0, -0.5, -1.1, 0.6, 0.1, 0.0, 0.0]
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")

heuristicName = 'static'
lfLegId = 'LFFoot'
lmLegId = 'LMFoot'
lbLegId = 'LBFoot'
rfLegId = 'RFFoot'
rmLegId = 'RMFoot'
rbLegId = 'RBFoot'
"""fullBody.addLimbDatabase('./ant_LFleg_6DOF.db',lfLegId,heuristicName) #50k samples
fullBody.addLimbDatabase('./ant_LMleg_6DOF.db',lmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_LBleg_6DOF.db',lbLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RFleg_6DOF.db',rfLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RMleg_6DOF.db',rmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RBleg_6DOF.db',rbLegId,heuristicName)"""
fullBody.addLimbDatabase('./ant_LFleg_6DOF_low.db',lfLegId,heuristicName) #10k samples
fullBody.addLimbDatabase('./ant_LMleg_6DOF_low.db',lmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_LBleg_6DOF_low.db',lbLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RFleg_6DOF_low.db',rfLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RMleg_6DOF_low.db',rmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RBleg_6DOF_low.db',rbLegId,heuristicName)
"""fullBody.addLimbDatabase('./ant_LFleg_6DOF_low.db',lfLegId,heuristicName) #10k samples
fullBody.addLimbDatabase('./ant_LMleg_3DOF_low.db',lmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_LBleg_3DOF_low.db',lbLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RFleg_3DOF_low.db',rfLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RMleg_3DOF_low.db',rmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RBleg_3DOF_low.db',rbLegId,heuristicName)"""
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig())
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathIdBis # tp.orientedpathIdBis or tp.solutionPathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]

fullBody.setFillGenerateContactState (True)
dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])

sphereColor = [1,0,0,1]; sphereSize = 0.05;
comInit = fullBody.getCenterOfMass (); sphereName = "comInit"; plotSphere (comInit, r, sphereName, sphereColor, sphereSize)

#comInitRef = q_init_test[0:3]; sphereName = "comInitRef"; plotSphere (comInitRef, r, sphereName, [0,1,0,1], sphereSize)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[len(Vimplist)-1], theta_goal)
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])

#comGoal = fullBody.getCenterOfMass (); sphereName = "comGoal"; plotSphere (comGoal, r, sphereName, sphereColor, sphereSize)

psf.setPlannerIterLimit (50)
timeStep = 0.002
maxIter = 100

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter) # no timed-interpolation
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True) # timed-interpolation
print("ballistic-interpolation finished")

#pp(psf.numberPaths ()-1)

#comGoal = fullBody.getCenterOfMass (); sphereName = "comGoal"; plotSphere (comGoal, r, sphereName, sphereColor, sphereSize)


statesTime = fullBody.getlastStatesComputedTime ()
numberOfStatesComputed = len(statesTime)-1
configs = statesTime [:numberOfStatesComputed]
times = statesTime [numberOfStatesComputed]

#print("Start comRRT")
#fullBody.comRRT(0, 1, psf.numberPaths ()-1, 0) # path = COM path (parabola ?)    NOT WORKING


#fullBody.projectStateToCOM (0, q_init_test[0:3])
#rr(fullBody.getlastStatesComputedTime ()[0])    # compare to rr(q_init_test)

#i = 6; fullBody.projectStateToCOM (i, configs[i][0:3])
#rr(fullBody.getlastStatesComputedTime ()[i]); fullBody.setCurrentConfig (fullBody.getlastStatesComputedTime ()[i])    # compare to rr(configs[i])
# plotSphere (fullBody.getCenterOfMass (), r, "test"+str(i), [0,0,1,1], sphereSize); rr(configs[i]); fullBody.setCurrentConfig (configs[i])
# plotSphere (configs[i][0:3], r, "testRef"+str(i), [0,1,0,1], sphereSize)


fullBody.projectLastStatesComputedToCOM()
statesTimeCOM = fullBody.getlastStatesComputedTime ()
numberOfStatesComputedCOM = len(statesTimeCOM)-1
configsCOM = statesTimeCOM [:numberOfStatesComputedCOM]
#fullBody.interpolatePathFromLastStatesComputed()


"""
for i in range (0,numberOfStatesComputedCOM):
	rr(configsCOM[i]); time.sleep(0.5);

for i in range (0,numberOfStatesComputedCOM):
	rr(configs[i]); time.sleep(0.5);

"""


#ID = gui.createWindow("w"); gui.addSceneToWindow("0_scene_hpp_",ID)

"""
## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
pathId = psf.numberPaths()-1 # path to export
plotCone (q_init_test, psf, rr, "cone_start", "friction_cone2")
plotCone (q_goal_test, psf, rr, "cone_goal", "friction_cone2")
plotConeWaypoints (psf, pathId, r, "cone_wp_group", "friction_cone2")
pathSamples = plotSampleSubPath (psf, rr, pathId, 70, "sampledPath", [1,0,0,1])

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_start','cone_start.dae')
gui.writeNodeFile('cone_goal','cone_goal.dae')
writePathSamples (pathSamples, 'path.txt')
pathJointConfigsToFile (psf, rr, "antTestInDirect_jointConfigs.txt", psf.numberPaths()-1, q_goal_test, 0.005)
"""

"""
## Video recording
import time
pp.dt = 0.01
pp.speed=0.1
rr(q_init_test)
rr.startCapture ("capture","png")
rr(q_init_test); time.sleep(2)
rr(q_init_test)
pp(psf.numberPaths ()-1)
rr(q_goal_test); time.sleep(2);
rr.stopCapture ()

## avconv commands
avconv -r 50 -i capture_0_%d.png -r 15 -vcodec mpeg4 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
avconv -r 30 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
avconv -i video.mp4 -vcodec mpeg4 -crf 24 spiderman_jump_slowmo.mp4
rm capture*.png
rm video.mp4
"""
