#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import frog_test_path as tp



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "frog"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])


#psf = ProblemSolver( fullBody ); rr = Viewer (psf)
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
q_0 = fullBody.getCurrentConfig(); rr(q_0)

#extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 0.4, 0.3, 0.6, 0.4, 0.3, -0.1, 0.3, 0.1, 0.2, 0.4, -0.3, -0.6, 0.4, -0.3, 0.1, 0.3, -0.1, -0.2, -0.3, 0.2, 0, 0, -0.3, 0, 0, 0.3, 0.2, -0.3, -0.2, 0, 0, 0.3, 0, 0, -0.3, 0, 0, 0, 0]
extending = [0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 0.4, 0.3, 0.6, 0.4, 0.3, -0.1, 0.3, 0.1, 0.2, 0.4, -0.3, -0.6, 0.4, -0.3, 0.1, 0.3, -0.1, -0.2, -0.3, 0.4, 0, 0, -0.6, 0, 0, 0.8, 0.2, -0.3, -0.4, 0, 0, 0.9, 0, 0, -0.8, 0.0, 0.0, 0.0, 0.0]
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.2, 0.2, -0.2, 0.1, -0.1, -0.4, -0.2, 0.2, -0.5, 0.2, -0.2, 0.2, 0.1, 0.1, 0.4, -0.2, -0.2, -0.3, -0.6, -1.2, -0.1, 0.4, 1, -0.2, 0.1, -1.5, 0.3, -0.6, 1.2, 0.1, 0.4, -1, 0.2, 0.1, 1.5,0,0,0,0]
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")

rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./Frog_rleg.db',rLegId,'static')
fullBody.addLimbDatabase('./Frog_lleg.db',lLegId,'static')
fullBody.addLimbDatabase('./Frog_rarm.db',rarmId,'static')
fullBody.addLimbDatabase('./Frog_larm.db',larmId,'static')
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathIdBis # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize-ecsSize] = trunkPathwaypoints[0][0:confsize-ecsSize]
q_goal[0:confsize-ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[0][confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][confsize-ecsSize:confsize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init); rr(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp reversed
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

fullBody.setStartState(q_init_test,[lLegId,rLegId,larmId,rarmId])
fullBody.setEndState(q_goal_test,[lLegId,rLegId,larmId,rarmId])

psf.setPlannerIterLimit (200)

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, 0.002)


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=0.8

#fullBody.timeParametrizedPath(psf.numberPaths() -1 )
pp(psf.numberPaths ()-1)

#pathJointConfigsToFile (psf, rr, "frog_pond_jointConfigs_new2.txt", psf.numberPaths ()-1, q_goal_test, 0.008)

#pathSamples = plotSampleSubPath (tp.ps.client.problem, tp.r, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])
#writePathSamples (pathSamples, 'frog_path2.txt')

"""
## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
pathId = psf.numberPaths()-1 # path to export
pathInterpWaypoints = ps.getWaypoints(pathId)
plotCone (q_init_test, psf, rr, "cone_start", "friction_cone2_blue")
plotCone (q_goal_test, psf, rr, "cone_goal", "friction_cone2_blue")
plotConeWaypoints (psf, pathId, r, "cone_wp_group", "friction_cone2_blue")
pathSamples = plotSampleSubPath (psf.client.problem, rr, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_start','cone_start.dae')
gui.writeNodeFile('cone_goal','cone_goal.dae')
writePathSamples (pathSamples, 'frog_path.txt')
pathJointConfigsToFile (psf, rr, "frog_jointConfigs.txt", pathId, q_goal_test, 0.02)
"""


"""
## Video recording
import time
pp.dt = 0.02
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

"""
# extending
q = q_0; q[2] = 0.5
q [fullBody.rankInConfiguration ['LThigh_rx']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['LThigh_ry']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['LThigh_rz']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['LShank_rx']] = 0; rr(q)
q [fullBody.rankInConfiguration ['LShank_ry']] = 0; rr(q)
q [fullBody.rankInConfiguration ['LShank_rz']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['LFoot_rx']] = 0; rr(q)
q [fullBody.rankInConfiguration ['LFoot_ry']] = 0; rr(q)
q [fullBody.rankInConfiguration ['LFoot_rz']] = 0.8; rr(q)

q [fullBody.rankInConfiguration ['RThigh_rx']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RThigh_ry']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['RThigh_rz']] = -0.4; rr(q)
q [fullBody.rankInConfiguration ['RShank_rx']] = 0; rr(q)
q [fullBody.rankInConfiguration ['RShank_ry']] = 0; rr(q)
q [fullBody.rankInConfiguration ['RShank_rz']] = 0.9; rr(q)
q [fullBody.rankInConfiguration ['RFoot_rx']] = 0; rr(q)
q [fullBody.rankInConfiguration ['RFoot_ry']] = 0; rr(q)
q [fullBody.rankInConfiguration ['RFoot_rz']] = -0.8; rr(q)


q [fullBody.rankInConfiguration ['LHumerus_rx']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['LHumerus_ry']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['LHumerus_rz']] = 0.3; rr(q)
q [fullBody.rankInConfiguration ['LForearm_rx']] = 0.6; rr(q)
q [fullBody.rankInConfiguration ['LForearm_ry']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['LForearm_rz']] = 0.3; rr(q)
q [fullBody.rankInConfiguration ['LHand_rx']] = -0.1; rr(q)
q [fullBody.rankInConfiguration ['LHand_ry']] = 0.3; rr(q)
q [fullBody.rankInConfiguration ['LHand_rz']] = 0.1; rr(q)

q [fullBody.rankInConfiguration ['RHumerus_rx']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RHumerus_ry']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['RHumerus_rz']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['RForearm_rx']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['RForearm_ry']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['RForearm_rz']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['RHand_rx']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['RHand_ry']] = 0.3; rr(q)
q [fullBody.rankInConfiguration ['RHand_rz']] = -0.1; rr(q)

fullBody.isConfigValid(q)
"""
