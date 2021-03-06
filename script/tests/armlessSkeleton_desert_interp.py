#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

import armlessSkeleton_desert_path as tp

packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "armlessSkeleton"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", tp.base_joint_xyz_limits)
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])
fullBody.setFullbodyFrictionCoef(0.5)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
q_0 = fullBody.getCurrentConfig(); rr(q_0)


extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.2, 0.5, 0.0, -0.1, 0.0, 0.1, 0.2, 0.5, 0.0, 0,0,0,0]
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, -1.1, 2.2, -1.2, 0.1, -0.1, -0.2, -1.1, 2.2, -1.2, -0.1,0,0,0,0]
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")


rLegId = 'RFoot'
lLegId = 'LFoot'
fullBody.addLimbDatabase('./armlessSkeleton_rleg.db',rLegId,'static')
fullBody.addLimbDatabase('./armlessSkeleton_lleg.db',lLegId,'static')
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

""" # without solving path
q_init[0:confsize-ecsSize] = tp.q11[0:confsize-ecsSize]
q_goal[0:confsize-ecsSize] = tp.q22[0:confsize-ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = tp.q11[confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = tp.q22[confsize-ecsSize:confsize]

dir_init = [0,0,-1]; dir_goal = [0,0, 1]"""

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

psf.setPlannerIterLimit (1000)

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, 0.01)



pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=3

#fullBody.timeParametrizedPath(psf.numberPaths() -1 )
#pp(psf.numberPaths ()-1)


#r.startCapture("skeletonDesert","png") ; r.stopCapture()

## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
"""
pathId = psf.numberPaths()-1 # path to export
plotCone (q_init_test, psf, rr, "cone_start", "friction_cone_blue")
plotCone (q_goal_test, psf, rr, "cone_goal", "friction_cone_blue")
plotConeWaypoints (psf, pathId, r, "cone_wp_group", "friction_cone_blue")
"""
"""
pathSamples = plotSampleSubPath (psf.client.problem, rr, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_start','cone_start.dae')
gui.writeNodeFile('cone_goal','cone_goal.dae')
writePathSamples (pathSamples, 'skeletonDesert_path.txt')
pathToYamlFile (psf, rr, "skeletonDesert_frames.yaml ", "armlessSkeleton", pathId, q_goal_test, 0.01)
"""
#pathId = psf.numberPaths()-1 # path to export
#pathToYamlFile (psf, rr, "skeletonDesert_frames2.yaml ", "armlessSkeleton", pathId, q_goal_test, 0.01)

"""
## Video recording
import time
pp.dt = 0.01
pp.speed=1.5
rr(q_init_test)
rr.startCapture ("capture","png")
rr(q_init_test); time.sleep(2)
rr(q_init_test)
pp(psf.numberPaths ()-1)
rr(q_goal_test); time.sleep(2);
rr.stopCapture ()

## ffmpeg commands
ffmpeg -r 30 -i capture_0_%04d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 30 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4
"""
"""
# Export configs to Blender
gui = rr.client.gui; gui.setCaptureTransform ("skeleton_contact_config.yaml ", ["armlessSkeleton"])
q = flexion; rr (q); psf.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()

gui = rr.client.gui; gui.setCaptureTransform ("skeleton_apex_config.yaml ", ["armlessSkeleton"])
q = extending; rr (q); psf.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()

# extending
q = q_0
q [fullBody.rankInConfiguration ['RHip_J1']] = -0.1; rr(q)
q [fullBody.rankInConfiguration ['RHip_J2']] = 0.0; rr(q)
q [fullBody.rankInConfiguration ['RThigh']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['RShank']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RAnkle_J1']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['RFoot']] = 0.0; rr(q)

q [fullBody.rankInConfiguration ['LHip_J1']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['LHip_J2']] = 0.0; rr(q)
q [fullBody.rankInConfiguration ['LThigh']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['LShank']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LAnkle_J1']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['LFoot']] = 0.0; rr(q)
[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.2, 0.5, 0.0, -0.1, 0.0, 0.1, 0.2, 0.5, 0.0]
fullBody.isConfigValid(q)

# flexion
q = q_0
q [fullBody.rankInConfiguration ['RHip_J1']] = -0.1; rr(q)
q [fullBody.rankInConfiguration ['RHip_J2']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['RThigh']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['RShank']] = 2.2; rr(q)
q [fullBody.rankInConfiguration ['RAnkle_J1']] = -1.2; rr(q)
q [fullBody.rankInConfiguration ['RFoot']] = -0.1; rr(q)

q [fullBody.rankInConfiguration ['LHip_J1']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['LHip_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LThigh']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['LShank']] = 2.2; rr(q)
q [fullBody.rankInConfiguration ['LAnkle_J1']] = -1.2; rr(q)
q [fullBody.rankInConfiguration ['LFoot']] = 0.1; rr(q)
[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, -1.1, 2.2, -1.2, 0.1, -0.1, -0.2, -1.1, 2.2, -1.2, -0.1]
fullBody.isConfigValid(q)

start= [-0.926587,-4.4898,-0.122323,-0.249679,-0.15765,-0.195424,0.935209,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.24573,0.306502,-0.831351,1.77806,-0.551047,0.352681,0.208679,0.319051,0.213806,1.01011,-0.713777,-0.219794,0.0632728,-0.425726,0.902637,-2.65711]
end= [-1.0956,-4.57876,0.140881,-0.220252,-0.153921,-0.186192,0.945055,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.289627,0.383697,-0.0532497,0.295376,0.568455,0.177505,0.208679,0.319051,0.213806,1.01011,-0.713777,-0.219794,0.0632728,-0.425726,0.902637,-2.65711]
"""
