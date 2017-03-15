#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

import kangaroo_desert_path as tp # it's long because of Affordance generation

packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "kangaroo"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 0 #tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", tp.base_joint_xyz_limits)
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.setFullbodyFrictionCoef (tp.frictionCoef)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
q_0 = fullBody.getCurrentConfig(); rr(q_0)


flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, -0.1, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, -0.4, 0.0, -0.4, 0.0, -0.1, 0.0, -0.4, 0.0, -0.4, 0.0, -0.1, 0.0, -0.3, 0.0, 0.3, 0.1, -0.6, 0.0, -0.3, 0.0, 0.3, 0.1, -0.6]
extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0, 0.0, -0.1, 0.0, 0, 0.0, 0, 0.0, -0.1, 0.0, 0.3, 0.0, -0.3, 0.8, -0.8, 0.0, 0.3, 0.0, -0.3, 0.8, -0.8]
#q_contact_takeoff =
#q_contact_landing =

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
#fullBody.setPose (q_contact_takeoff, "takeoffContact")
#fullBody.setPose (q_contact_landing, "landingContact")


rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./kangaroo_rleg_6DOF.db',rLegId,'static')
fullBody.addLimbDatabase('./kangaroo_lleg_6DOF.db',lLegId,'static')
fullBody.addLimbDatabase('./kangaroo_rarm_3DOF.db',rarmId,'static')
fullBody.addLimbDatabase('./kangaroo_larm_3DOF.db',larmId,'static')
print("Limbs added to fullbody")

confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = flexion; q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathIdBis # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
trunkPathwaypoints = ps.getWaypoints (entryPathId)

#q = flexion [::]; q [0:confsize-ecsSize] = trunkPathwaypoints[1][0:confsize-ecsSize]; rr(q)
#fullBody.setCurrentConfig (q); qt = fullBody.generateContacts(q, [0,0,1], True); rr (qt); fullBody.isConfigValid(qt)

q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[0][confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][confsize-ecsSize:confsize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[rLegId,lLegId])

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[0], theta_goal)
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[rLegId,lLegId])


psf.setPlannerIterLimit (5)

print("Start ballistic-interpolation")
#fullBody.interpolateBallisticPath(entryPathId, 0.002) # no timed-interpolation
fullBody.interpolateBallisticPath(entryPathId, 0.002, True) # timed-interpolation
print("ballistic-interpolation finished")


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=1

#fullBody.timeParametrizedPath(psf.numberPaths() -1 )
#pp(psf.numberPaths ()-1)


#r.startCapture("skeletonDesert_pbIntersectionRomObst","png") ; r.stopCapture()

## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
"""
pathId = psf.numberPaths()-1 # path to export
#plotCone (q_init_test, psf, rr, "cone_start", "friction_cone_blue")
#plotCone (q_goal_test, psf, rr, "cone_goal", "friction_cone_blue")
#plotConeWaypoints (psf, pathId, r, "cone_wp_group", "friction_cone_blue")
"""
"""
pathSamples = plotSampleSubPath (psf.client.problem, rr, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])

#gui.writeNodeFile('cone_wp_group','cones_path.dae')
#gui.writeNodeFile('cone_start','cone_start.dae')
#gui.writeNodeFile('cone_goal','cone_goal.dae')
writePathSamples (pathSamples, 'kangaroo_desert_path.txt')
pathToYamlFile (psf, rr, "kangarooDesert_frames.yaml ", "kangaroo", psf.numberPaths()-1, q_goal_test, 0.025)
"""

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
q = q_0 [::]
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

q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['LHumerus']] = -2; rr(q)
q [fullBody.rankInConfiguration ['LElbow_J1']] = -0.4; rr(q)
q [fullBody.rankInConfiguration ['LForearm']] = -1.2; rr(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = -0.1; rr(q)
q [fullBody.rankInConfiguration ['RHumerus']] = -2; rr(q)
q [fullBody.rankInConfiguration ['RElbow_J1']] = -0.4; rr(q)
q [fullBody.rankInConfiguration ['RForearm']] = 1.2; rr(q)

fullBody.isConfigValid(q)

# flexion -> see skeleton_compute_DB.py

# q_contact  # legs don't account
q = q_0 [::]
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

q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = 0; rr(q)
q [fullBody.rankInConfiguration ['LHumerus']] = -0.7; rr(q)
q [fullBody.rankInConfiguration ['LElbow_J1']] = -1.; rr(q)
q [fullBody.rankInConfiguration ['LForearm']] = -1.2; rr(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = -0.4; rr(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = 0; rr(q)
q [fullBody.rankInConfiguration ['RHumerus']] = -0.7; rr(q)
q [fullBody.rankInConfiguration ['RElbow_J1']] = -1.; rr(q)
q [fullBody.rankInConfiguration ['RForearm']] = 1.2; rr(q)

fullBody.isConfigValid(q)


#q = [6.2666,-2.6362,-0.952683,-0.0849639,-0.13228,0.113626,0.981006,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.678865,0.0628397,0.3524,-1.64609,-1.32557,-0.00309284,-0.16359,-0.68511,-0.141783,0.349037,-1.59126,1.34176,0.142385,-0.01578,0.303685,1.07764,0.47237,-0.0472983,0.972411,-0.146209,0.334052,-0.227656,-0.326022,-1.14689,2.27182,-1.20909,0.0557013,0.124957,-0.278843,0.200458,0.939182,-2.93748,]
q = [6.2666,-2.6362,-0.952683,-0.0849639,-0.13228,0.113626,0.981006,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.678865,0.0628397,0.3524,-1.64609,-1.32557,-0.00309284,-0.16359,-0.68511,-0.141783,0.349037,-1.59126,1.34176,0.142385,-0.01578,0.303685,1.07764,0.47237,-0.0472983,0.972411,-0.146209,0.334052,-0.227656,-0.326022,-1.14689,2.27182,-1.20909,0.0557013,0.124957,-0.278843,0.200458,0.939182,-2.93748,]
fullBody.isConfigValid(q); rr(q)

"""

""" # without solving path
q_init[0:confsize-ecsSize] = tp.q11[0:confsize-ecsSize]
q_goal[0:confsize-ecsSize] = tp.q22[0:confsize-ecsSize]
if (ecsSize > 0):
    q_init[fullConfSize-ecsSize:fullConfSize] = tp.q11[confsize-ecsSize:confsize]
    q_goal[fullConfSize-ecsSize:fullConfSize] = tp.q22[confsize-ecsSize:confsize]

dir_init = [0,0,-1]; dir_goal = [0,0, 1]"""

