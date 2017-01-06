#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import skeleton_parkourWalls_path as tp


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "skeleton"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 0 #tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
#fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
#fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

#psf = ProblemSolver(fullBody); rr = Viewer (psf)
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr)
q_0 = fullBody.getCurrentConfig(); rr(q_0)

#q_top= [-3.75,0,1.64135,1,0,0,0,0,-0.0227288,0.0226894,-0.0050107,1,0,0,0]
#rr(q_top); fullBody.isConfigValid(q_top)

flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -0.2, -1, -2.5, 0.0, 0.0, -0.2, -0.1, -0.2, -1, 2.5, 0.0, 0.0, 0, 0.1, -2, 2.5, -0.7, 0, -0.2, 0, -0.1, -2, 2.5, -0.7, 0, -0.2]
extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.6, -1, -1.5, 0.0, 0.0, -0.2, -0.1, -1.7, -1.1, 1.5, 0.0, 0.0, 0, 0.0, -1.2, 1.5, 0.0, 0, -0.0, 0, -0.0, -1.4, 1.7, 0.1, 0, -0.0]
q_contact_takeoff =[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.4, -1.3, -1.5, 0.0, 0.0, -0.2, -0.1, -1.5, -1.2, 1.5, 0.0, 0.0, 0.0, 0.1, -1.4, 2.0, -0.6, 0.0, -0.2, 0.0, -0.1, -1.4, 2.0, -0.6, 0.0, -0.2]
q_contact_landing = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.4, -1, -1.2, 0.0, 0.0, -0.0, 0.2, 1.5, -1.1, 1.2, 0.0, 0.0, 0, 0.1, -1.1, 0.9, 0, 0, -0.2, 0, -0.1, -1.1, 0.9, 0, 0, -0.2]

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")
fullBody.setPose (q_contact_landing, "landingContact")

#~ AFTER loading obstacles
rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./skeleton_rleg.db',rLegId,'static')
fullBody.addLimbDatabase('./skeleton_lleg.db',lLegId,'static')
fullBody.addLimbDatabase('./skeleton_rarm.db',rarmId,'static')
fullBody.addLimbDatabase('./skeleton_larm.db',larmId,'static')
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathId # tp.orientedpathId or tp.solutionPathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize-ecsSize] = trunkPathwaypoints[0][0:confsize-ecsSize]
q_goal[0:confsize-ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[0][confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][confsize-ecsSize:confsize]


dir_init = (-np.array(V0list [len(V0list)-1])).tolist() # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

com = fullBody.getCenterOfMass ();
sphereColor = [1,0,0,1]; sphereSize = 0.03; sphereName = "comInit"
plotSphere (com, r, sphereName, sphereColor, sphereSize)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

com = fullBody.getCenterOfMass (); sphereName = "comGoal"
plotSphere (com, r, sphereName, sphereColor, sphereSize)

fullBody.setStartState(q_init_test,[LegId])
fullBody.setEndState(q_goal_test,[LegId])

## Interpolation

extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, -0.3, 0.0, 0,0,0,0]
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.58, 1.1, -0.5, 0.0, 0,0,0,0]
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
timeStep = 0.001

print("Start ballistic-interpolation")
psf.setPlannerIterLimit (50)
fullBody.interpolateBallisticPath(entryPathId, 0.005) #  -> now also set lastComputedStates_ stack

#configs = fullBody.interpolate(0.01,entryPathId,100, True)   # Steve  # timeStep, pathId, robThreshold   -> to set lastComputedStates_ stack
#numberOfStatesComputed = len(configs)


#fullBody.timeParametrizedPath(psf.numberPaths() -1) # TODO debug !
pp.speed=2
#pp(psf.numberPaths ()-1)

#test = []; rr(test); fullBody.isConfigValid(test)

statesTime = fullBody.getlastStatesComputedTime ()
numberOfStatesComputed = len(statesTime)-1
configs = statesTime [:numberOfStatesComputed]
times = statesTime [numberOfStatesComputed]

#fullBody.comRRT(0, 1, entryPathId, 0) # path = COM path (parabola ?)


"""
for i in range (0,numberOfStatesComputed):
	rr(configs[i]); time.sleep(0.5);
"""

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
pathToYamlFile (psf, rr, "lampTest_frames.yaml ", "lamp", pathId, q_goal_test, 0.01)
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

"""
q = q_0 [::]
q [fullBody.rankInConfiguration ['ShankJoint']] = -0.6;r(q)
q [fullBody.rankInConfiguration ['AnkleJoint']] = 0.6;r(q)
"""

""" # Visualize geometrical origin and COM
psf = ProblemSolver(fullBody); rr = Viewer (psf)
q_0 = fullBody.getCurrentConfig(); rr(q_0)
origin = [0,0,0];
sphName = "test"+str(origin[0])+str(origin[1])+str(origin[2]);
plotSphere (origin, rr, sphName, [0,0,1,1], 0.03)
com = fullBody.getCenterOfMass ();
plotSphere (com, rr, "com", [1,0,0,1], 0.03)
plotFrame(rr,'framy', origin,0.4)
"""