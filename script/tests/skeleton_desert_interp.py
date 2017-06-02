#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

import skeleton_desert_path as tp

packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "skeleton"
urdfSuffix = ""
srdfSuffix = ""
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", tp.base_joint_xyz_limits)
fullBody.setFullbodyFrictionCoef (tp.frictionCoef)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr); pp.speed = 0.6
q_0 = fullBody.getCurrentConfig(); rr(q_0)


flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 0.2, 0.1, -0.2, -1, -2.5, 0.0, 0.0, -0.2, -0.1, -0.2, -1, 2.5, 0.0, 0.0, 0, 0.1, -1.7, 2.5, -0.8, 0, -0.2, 0, -0.1, -1.7, 2.5, -0.8, 0, -0.2]
extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.6, -1, -1.5, 0.0, 0.0, -0.2, -0.1, -1.7, -1.1, 1.5, 0.0, 0.0, 0, 0.0, -1.2, 1.5, 0.0, 0, -0.0, 0, -0.0, -1.4, 1.7, 0.1, 0, -0.0]
q_contact_takeoff =[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.4, -1.3, -1.5, 0.0, 0.0, -0.2, -0.1, -1.5, -1.2, 1.5, 0.0, 0.0, 0.0, 0.1, -1.4, 2.0, -0.6, 0.0, -0.2, 0.0, -0.1, -1.4, 2.0, -0.6, 0.0, -0.2]
q_contact_landing = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.4, -1, -1.2, 0.0, 0.0, -0.0, 0.2, 1.5, -1.1, 1.2, 0.0, 0.0, 0, 0.1, -1.1, 0.9, 0, 0, -0.2, 0, -0.1, -1.1, 0.9, 0, 0, -0.2]

"""
q = q_contact_takeoff [::]
q [fullBody.rankInConfiguration ['Sacrum']] = 0; rr(q)
q [fullBody.rankInConfiguration ['Head']] = 0; rr(q)
"""

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")
fullBody.setPose (q_contact_landing, "landingContact")


rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./skeleton_rleg_3DOF.db',rLegId,'static')
fullBody.addLimbDatabase('./skeleton_lleg_3DOF.db',lLegId,'static')
fullBody.addLimbDatabase('./skeleton_rarm_3DOF.db',rarmId,'static')
fullBody.addLimbDatabase('./skeleton_larm_3DOF.db',larmId,'static')
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = flexion [::]; q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathIdBis # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
trunkPathwaypoints = ps.getWaypoints (entryPathId)


q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]


dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[rLegId, lLegId])
fullBody.setFullbodyV0fThetaCoefs ("V0", True, [0,0,0], 0)

#fullBody.getcentroidalConeFails () # TEST

dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[len(Vimplist)-1], theta_goal)
fullBody.setCurrentConfig (q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[rLegId, lLegId])
fullBody.setFullbodyV0fThetaCoefs ("Vimp", True, [0,0,0], 0)

psf.setPlannerIterLimit (100)
timeStep = 0.002
maxIter = 100

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter) # no timed-interpolation
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True) # timed-interpolation
print("ballistic-interpolation finished")


fullBody.getPathPlannerFails ()


pp.speed=0.6

#pp(psf.numberPaths ()-1)

## Save data

nbWaypoints = len (trunkPathwaypoints)
nbParabolas = nbWaypoints - 1
nbCentroidalFails = fullBody.getcentroidalConeFails ()
nbFailsLimbRRT = fullBody.getPathPlannerFails ()

# Write important results #
f = open('results_skeleton_runs.txt','a')
print('nbWaypoints: '+str(nbWaypoints))
print('nbParabolas: '+str(nbParabolas))
print('nbCentroidalFails: '+str(nbCentroidalFails))
print('nbFailsLimbRRT: '+str(nbFailsLimbRRT))
f.write('-------------------------'+'\n')
f.write('nbWaypoints: '+str(nbWaypoints)+'\n')
f.write('nbParabolas: '+str(nbParabolas)+'\n')
f.write('nbCentroidalFails: '+str(nbCentroidalFails)+'\n')
f.write('nbFailsLimbRRT: '+str(nbFailsLimbRRT)+'\n')
f.write("path length= " + str(psf.pathLength(psf.numberPaths ()-1))+'\n') # to verify that not same paths

f.close()


"""
from parseRuns import main
main("results_skeleton_runs.txt")
"""


## TODO try to connect directly those two configs with limb-RRT
#qt1 = [-4.98514,-6.36751,-0.808568,-0.37693,0.0910363,-0.106665,-0.915565,0,0,0.0712573,0,0,0,0,0,-0.0712573,0.0712573,-0.0931141,0.829942,-1,-1.66317,0,0,-0.0712573,0.0931141,0.894313,-1.06437,1.66317,0,0,0.189043,-0.517203,-1.14991,0.738904,0.296294,-0.0905964,-0.251554,0.0597942,-0.20037,-0.993151,1.23629,0.272293,-0.1332,-0.361487]
#qt2 = [-5.16011,-6.75111,-1.58784,-0.451129,0.078433,-0.11958,-0.880926,0,0,0,0,0,0,0,0,0,0,-0.2,1.4,-1,-1.2,0,0,-0,0.2,1.5,-1.1,1.2,0,0,0.211114,-0.576363,-1.11084,0.613016,0.376712,-0.0822658,-0.243191,-0.0687832,-0.649155,-1.01131,0.986387,0.0831689,-0.0524914,-0.299198]



"""
plotCone([4.42,-0.08,0.09,],psf,rr,"cone1","friction_cone06")
plotCone([-6.4,-5.1,-1.7,-0.230778,-0.364807,0.90203],psf,rr,"cone1","friction_cone06")
plotCone([-6.4,-5.1,-1.7,-0.316247,-0.354415,0.879988],psf,rr,"cone2","friction_cone06")
pos = [-6.4,-5.1,-1.7]
plotStraightLine (Vimplist[len(Vimplist)-1], pos, rr, "vf") # dans cones
plotStraightLine ((-np.array(Vimplist [len(Vimplist)-1])).tolist(), pos, rr, "-vf") # dans cones
plotStraightLine ([-0.128794,0.762169,0.634437], pos, rr, "v0") # clairement hors cones
plotStraightLine (vf, pos, rr, "vfsdf")

pos = [4.42,-0.08,0.09]
plotStraightLine ([-0.891853,5.27776,4.39326], pos, rr, "v0_initConfig")
plotStraightLine ([-0.128794  0.762169  0.634437], pos, rr, "v0normed_initConfig")


cones = [[-0.508069,-0.380334,0.772795],[-0.223381,-0.24573,0.943248]]
t = rbprmBuilder.convexConePlaneIntersection (len(cones), cones, 1.7382, 0.6)
black = [0.1,0.1,0.1,1]; red = [1,0,0,1]; blue = [0,0,1,1]; green = [0,1,0,1]; planeThetaColor = [0.7,0.2,0.2,0.2]
CC2D_dir = t [1:4]
plotConvexConeInters (psf, rr, pos, CC2D_dir, cones, "CC_center", black, 0.02, "CC_dir", "contactCones", "friction_cone06")


"""

#r.startCapture("skeletonDesert_pbIntersectionRomObst","png") ; r.stopCapture()

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
writePathSamples (pathSamples, 'fullSkeleton_newDesert_path2.txt')
pathToYamlFile (psf, rr, "fullSkeletonDesert_frames2.yaml ", "skeleton", psf.numberPaths()-1, q_goal_test, 0.025)
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


