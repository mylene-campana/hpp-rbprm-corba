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
urdfName = "skeleton" # configSize = 44 (with 4 quaternions, no extra-config -> 43 true configs)
urdfSuffix = ""
srdfSuffix = ""
#V0list = tp.V0list
#Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)

#psf = ProblemSolver(fullBody); rr = Viewer (psf)
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr)
q_0 = fullBody.getCurrentConfig(); rr(q_0)

#q_top= [-3.75,0,1.64135,1,0,0,0,0,-0.0227288,0.0226894,-0.0050107,1,0,0,0]
#rr(q_top); fullBody.isConfigValid(q_top)

flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -0.2, -1, -2.5, 0.0, 0.0, -0.2, -0.1, -0.2, -1, 2.5, 0.0, 0.0, 0, 0.1, -2, 2.5, -0.7, 0, -0.2, 0, -0.1, -2, 2.5, -0.7, 0, -0.2]
extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.6, -1, -1.5, 0.0, 0.0, -0.2, -0.1, -1.7, -1.1, 1.5, 0.0, 0.0, 0, 0.0, -1.2, 1.5, 0.0, 0, -0.0, 0, -0.0, -1.4, 1.7, 0.1, 0, -0.0]
q_contact_takeoff =[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.4, -1.3, -1.5, 0.0, 0.0, -0.2, -0.1, -1.5, -1.2, 1.5, 0.0, 0.0, 0.0, 0.1, -1.4, 2.0, -0.6, 0.0, -0.2, 0.0, -0.1, -1.4, 2.0, -0.6, 0.0, -0.2]
q_contact_landing = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.4, -1, -1.2, 0.0, 0.0, -0.0, 0.2, 1.5, -1.1, 1.2, 0.0, 0.0, 0, 0.1, -1.1, 0.9, 0, 0, -0.2, 0, -0.1, -1.1, 0.9, 0, 0, -0.2]

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")
fullBody.setPose (q_contact_landing, "landingContact")

#~ AFTER loading obstacles
rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
print("Adding limbs ...")
fullBody.addLimbDatabase('./skeleton_rleg_3DOF.db',rLegId,'static')
fullBody.addLimbDatabase('./skeleton_lleg_3DOF.db',lLegId,'static')
fullBody.addLimbDatabase('./skeleton_rarm_3DOF.db',rarmId,'static')
fullBody.addLimbDatabase('./skeleton_larm_3DOF.db',larmId,'static')
"""
fullBody.addLimb(rLegId,'RHip_J1','RFootSphere',[0,0,0],[0,0,1], 0.03, 0.08, 50000, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(lLegId,'LHip_J1','LFootSphere',[0,0,0],[0,0,1], 0.03, 0.08, 50000, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(rarmId,'RShoulder_J1','RHandSphere',[0,0,0],[1,0,1], 0.03, 0.08, 50000, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(larmId,'LShoulder_J1','LHandSphere',[0,0,0],[1,0,1], 0.03, 0.08, 50000, "EFORT_Normal", 0.01,"_6_DOF")
"""
print("Limbs added to fullbody")

confsize = len(tp.q11)

## try contacts !!
fullBody.setFullbodyInteriorPoint([0,0,6])
qt = flexion [::]
qt[0:confsize-tp.ecsSize] = tp.qt[0:confsize-tp.ecsSize]
fullBody.setCurrentConfig (qt); rr(qt)
qtt = fullBody.generateContacts(qt, [0,0,1], True); rr (qtt); fullBody.isConfigValid(qtt)
#qtt = fullBody.generateContacts(qt, [0.1/0.9055,0,0.9/0.9055], True); rr (qtt); fullBody.isConfigValid(qtt)


"""
pos = rr.client.gui.getNodeGlobalTransform("skeleton/RHandSphere")[0:3]
v = []
plotStraightLine (v, pos, rr, "vec")
plotJointFrame (rr, psf, q_0, "RHandSphere", 0.2,"RHandSphere_frame")
plotVectorInJointFrame(rr, psf, q_0, "RHandSphere", v, [0.8,0.8,0.8,1],"v_inJframe")
"""


"""
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = q_0[::]; q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathIdBis # tp.orientedpathId or tp.solutionPathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]

dir_init = (-np.array(V0list [len(V0list)-1])).tolist() # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[rLegId,lLegId,rarmId,larmId])

#com = fullBody.getCenterOfMass (); sphereColor = [1,0,0,1]; sphereSize = 0.03; sphereName = "comInit"; plotSphere (com, r, sphereName, sphereColor, sphereSize)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[len(Vimplist)-1], theta_goal)
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[rLegId,lLegId,rarmId,larmId])

#com = fullBody.getCenterOfMass (); sphereName = "comGoal"; plotSphere (com, r, sphereName, sphereColor, sphereSize)

psf.setPlannerIterLimit (5)
timeStep = 0.0005
maxIter = 400

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter) #  -> now also set lastComputedStates_ stack
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True) # timed-interpolation
print("ballistic-interpolation finished")


pp.speed=2
#pp(psf.numberPaths ()-1)


statesTime = fullBody.getlastStatesComputedTime ()
numberOfStatesComputed = len(statesTime)-1
configs = statesTime [:numberOfStatesComputed]
times = statesTime [numberOfStatesComputed]

#fullBody.comRRT(0, 1, entryPathId, 0) # path = COM path (parabola ?)
"""

#r.startCapture ("skeleton_parkourWalls_EFORTcontactConfig_strange","png"); r.stopCapture()

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
