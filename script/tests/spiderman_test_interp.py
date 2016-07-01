#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import spiderman_test_path as tp



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-8, 6, -2, 2, -0.3, 3])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

#ps = ProblemSolver( fullBody ); rr = Viewer (ps)
r = tp.r; ps = tp.ps

psf = tp.ProblemSolver( fullBody )
rr = tp.Viewer (psf); gui = rr.client.gui

q_0 = fullBody.getCurrentConfig(); rr(q_0)

#~ AFTER loading obstacles
nbSamples = 12000
cType = "_6_DOF" # disable rotation constraint on contacts ??
x = 0.04 # contact surface width
y = 0.1 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]
rLegId = 'rfoot'
rLeg = 'RThigh_ry'
rfoot = 'SpidermanRFootSphere'
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LThigh_ry'
lfoot = 'SpidermanLFootSphere'
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,-1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)
print("Limbs added to fullbody")
"""
rarmId = 'rhand'
rarm = 'RHumerus_ry'
rHand = 'SpidermanRHandSphere'
fullBody.addLimb(rarmId,rarm,rHand,[0,0,0],[-1,0,0], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

larmId = 'lhand'
larm = 'RHumerus_ry'
lHand = 'SpidermanLHandSphere'
fullBody.addLimb(larmId,larm,lHand,[0,0,0],[-1,0,0], x, y, nbSamples, "EFORT_Normal", 0.01,cType)
print("Limbs added to fullbody")
"""

confsize = len(tp.q11)-ecsSize
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathId # tp.orientedpathId or tp.solutionPathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize] = trunkPathwaypoints[0][0:confsize]
q_goal[0:confsize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[0][confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][confsize-ecsSize:confsize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp reversed
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

fullBody.setStartState(q_init_test,[lLegId,rLegId])#,larmId,rarmId])
fullBody.setEndState(q_goal_test,[lLegId,rLegId])#,larmId,rarmId])

extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0,0,0,0,0] # = q_0 + 'RAnkle_J1'=0.6 + 'LAnkle_J1'=0.6
flexion = q_0 # TODO
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")


print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, 0.03)


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=0.8

fullBody.timeParametrizedPath(psf.numberPaths() -1 )
pp(psf.numberPaths ()-1)


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

