#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import ant_cave_path as tp


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

#psf = ProblemSolver(fullBody); rr = tp.Viewer (psf); gui = rr.client.gui
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
fullBody.addLimbDatabase('./ant_LFleg_6DOF.db',lfLegId,heuristicName) #50k samples
fullBody.addLimbDatabase('./ant_LMleg_6DOF.db',lmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_LBleg_6DOF.db',lbLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RFleg_6DOF.db',rfLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RMleg_6DOF.db',rmLegId,heuristicName)
fullBody.addLimbDatabase('./ant_RBleg_6DOF.db',rbLegId,heuristicName)

print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig())
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathIdBis # or tp.solutionPathId or tp.orientedpathIdBis
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

fullBody.getcentroidalConeFails ()

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[len(Vimplist)-1], theta_goal)
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])



psf.setPlannerIterLimit (50)
timeStep = 0.002
maxIter = 100

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter) # no timed-interpolation
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True) # timed-interpolation
print("ballistic-interpolation finished")


fullBody.getPathPlannerFails ()



#pp(psf.numberPaths ()-1)

## Save data

nbWaypoints = len (trunkPathwaypoints)
nbParabolas = nbWaypoints - 1
nbCentroidalFails = fullBody.getcentroidalConeFails ()
nbFailsLimbRRT = fullBody.getPathPlannerFails ()

print('nbFailsLimbRRT: '+str(nbFailsLimbRRT))

# Write important results #
f = open('results_ant_runs.txt','a')
f.write('-------------------------'+'\n')
f.write('nbWaypoints: '+str(nbWaypoints)+'\n')
f.write('nbParabolas: '+str(nbParabolas)+'\n')
f.write('nbCentroidalFails: '+str(nbCentroidalFails)+'\n')
f.write('nbFailsLimbRRT: '+str(nbFailsLimbRRT)+'\n')
f.write("path length= " + str(psf.pathLength(psf.numberPaths ()-1))+'\n') # to verify that not same paths

f.close()

"""
from parseRuns import main
main("results_ant_runs.txt")
"""



## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
"""
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
"""

#pathSamples = plotSampleSubPath (ps.client.problem, r, solutionPathId, 70, "sampledPath", [1,0,0,1])
#writePathSamples (pathSamples, 'antCave_path.txt')

#plotFrame (rr, "frame", [0,0,0], 0.4)
