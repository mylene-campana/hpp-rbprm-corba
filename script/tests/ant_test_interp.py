#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
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
ecsSize = 0 #tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
fullBody.setFullbodyFrictionCoef (tp.frictionCoef)

#psf = ProblemSolver(fullBody); rr = Viewer (psf)
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr)
q_0 = fullBody.getCurrentConfig(); rr(q_0)


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

q_0 = fullBody.getCurrentConfig(); rr(q_0)


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathId # tp.orientedpathId or tp.solutionPathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[0][confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][confsize-ecsSize:confsize]

fullBody.setFillGenerateContactState (True)
dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])


dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[0], theta_goal)
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])



extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1, 0.0, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, 0.0, -0.5, 0.0, 0.0, 0, 0.0, 0.0, 1.1, 0.0, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0, 0.0, -1.1, 0.0, 0.0, 0.5, 0.0, 0.0, 0, 0.0, 0.0, -1.1, 0.0, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0]
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.4, 0.0, 1.0, -0.5, -0.3, 0.1, -0.2, 0.0, -1.1, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.0, 0.6, -1.1, 0.0, -0.6, 1.1, -0.6, -0.1, 0.0, 0.0, 0.0, 1.0, 0.4, 0.0, -1.0, 0.5, -0.3, -0.1, 0.2, 0.0, 1.1, 0.0, 0.0, -1.1, 0.0, 0.0, 0.0, 0.0, 0.6, 1.1, 0.0, -0.5, -1.1, 0.6, 0.1, 0.0, 0.0]
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")

psf.setPlannerIterLimit (2)

print("Start ballistic-interpolation")
#fullBody.interpolateBallisticPath(entryPathId, 0.002) # no timed-interpolation
fullBody.interpolateBallisticPath(entryPathId, 0.002, True)
print("ballistic-interpolation finished")



#fullBody.timeParametrizedPath(psf.numberPaths() -1)
pp.speed=0.6
pp(psf.numberPaths ()-1)

#pathJointConfigsToFile (psf, rr, "antTestInDirect_jointConfigs.txt", psf.numberPaths()-1, q_goal_test, 0.005)


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
pathJointConfigsToFile (psf, rr, "jointConfigs.txt", pathId, q_goal_test, 0.02)
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
q = extending[::]
q = q_0 [::]
q [fullBody.rankInConfiguration ['LFThigh_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LFShank_ry']] = -0.5 ; rr(q)
q [fullBody.rankInConfiguration ['LFFoot_ry']] = 0 ; rr(q)

q [fullBody.rankInConfiguration ['LMThigh_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LMShank_ry']] = -0.5 ; rr(q)
q [fullBody.rankInConfiguration ['LMFoot_ry']] = 0.2 ; rr(q)

q [fullBody.rankInConfiguration ['LBThigh_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LBShank_ry']] = -0.5 ; rr(q)
q [fullBody.rankInConfiguration ['LBFoot_ry']] = 0.2 ; rr(q)

q [fullBody.rankInConfiguration ['RFThigh_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RFShank_ry']] = 0.5 ; rr(q)
q [fullBody.rankInConfiguration ['RFFoot_ry']] = 0 ; rr(q)

q [fullBody.rankInConfiguration ['RMThigh_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RMShank_ry']] = 0.5 ; rr(q)
q [fullBody.rankInConfiguration ['RMFoot_ry']] = -0.2 ; rr(q)

q [fullBody.rankInConfiguration ['RBThigh_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RBShank_ry']] = 0.5 ; rr(q)
q [fullBody.rankInConfiguration ['RBFoot_ry']] = -0.2 ; rr(q)

p1= [-3.20403, -1.13039, -8.00642e-19]
p2= [-3.56603, -0.914465, 9.34082e-19]
p3= [-3.34256, -1.05825, -1.02043e-19]
p4= [-2.92027, -0.722541, -3.46945e-19]
p5= [-3.31152, -0.479053, 1.85037e-18]
p6= [-3.04735, -0.542559, 0]

sphereColor = [1,0,0,1]; sphereSize = 0.02; sphereName = "pointsPlane"
plotSphere (p1, rr, sphereName+"1", sphereColor, sphereSize)
plotSphere (p2, rr, sphereName+"2", sphereColor, sphereSize)
plotSphere (p3, rr, sphereName+"3", sphereColor, sphereSize)
plotSphere (p4, rr, sphereName+"4", sphereColor, sphereSize)
plotSphere (p5, rr, sphereName+"5", sphereColor, sphereSize)
plotSphere (p6, rr, sphereName+"6", sphereColor, sphereSize)

"""
