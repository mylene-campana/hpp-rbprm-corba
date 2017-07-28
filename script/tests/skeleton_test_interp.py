#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
from viewer_library import *
import time

import skeleton_test_path as tp



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "skeleton" # black sphere for the root
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 0
V0list = tp.V0list
Vimplist = tp.Vimplist

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-8, 6, -2, 2, 0, 3])

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps;
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr); pp.speed = 0.6
q_0 = fullBody.getCurrentConfig(); rr(q_0)
#plotSphere (fullBody.getCenterOfMass (), rr, "test_com_q_0", [1,0,0,1], 0.02); plotFrame (rr, 'frame_group', [0,0,0], 0.6)

flexion_final = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3, 0.2, 0.1, -0.2, -1, -2.5, 0.0, 0.0, -0.2, -0.1, -0.2, -1, 2.5, 0.0, 0.0, 0, 0.1, -1.7, 2.5, -0.8, 0, -0.2, 0, -0.1, -1.7, 2.5, -0.8, 0, -0.2]
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0.2, 0.1, 0.4, -0.1, -2, 0.0, 0.0, -0.2, -0.1, 0.4, -0.1, 2, 0.0, 0.0, 0, 0.1, -1.7, 2.5, -0.8, 0, -0.2, 0, -0.1, -1.7, 2.5, -0.8, 0, -0.2]
extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.6, -1, -1.5, 0.0, 0.0, -0.2, -0.1, -1.7, -1.1, 1.5, 0.0, 0.0, 0, 0.0, -1.2, 1.5, 0.0, 0, -0.0, 0, -0.0, -1.4, 1.7, 0.1, 0, -0.0]
q_contact_takeoff = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.2, 0.1, -1.8, -0.7, -1.5, 0.0, 0.0, -0.2, -0.1, -1.8, -0.7, 1.5, 0.0, 0.0, 0.0, 0.1, -1.4, 2.0, -0.6, 0.0, -0.2, 0.0, -0.1, -1.4, 2.0, -0.6, 0.0, -0.2]
q_contact_landing = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.0, -0.2, 1.4, -1, -1.2, 0.0, 0.0, -0.0, 0.2, 1.5, -1.1, 1.2, 0.0, 0.0, 0, 0.1, -1.1, 0.9, 0, 0, -0.2, 0, -0.1, -1.1, 0.9, 0, 0, -0.2]
#q_contact_landing = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.0, -0.2, 0.5, -0.4, -1.2, 0.0, 0.0, 0, 0.2, 0.5, -0.4, 1.2, 0.0, 0.0, 0, 0.1, -1.1, 0.9, 0, 0, -0.2, 0, -0.1, -1.1, 0.9, 0, 0, -0.2]

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")
fullBody.setPose (q_contact_landing, "landingContact")
fullBody.setPose (flexion_final, "flexionFinal")


rLegId = 'RFootToe'
lLegId = 'LFootToe'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./skeleton_rleg_3DOF.db',rLegId,'static')
fullBody.addLimbDatabase('./skeleton_lleg_3DOF.db',lLegId,'static')
fullBody.addLimbDatabase('./skeleton_rarm_3DOF_light.db',rarmId,'static')
fullBody.addLimbDatabase('./skeleton_larm_3DOF_light.db',larmId,'static')
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.solutionPathId # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
trunkPathwaypoints = ps.getWaypoints (entryPathId)


q_init = flexion [::]
q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init); rr(q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init, dir_init, True, 1); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[rLegId, lLegId])
fullBody.setFullbodyV0fThetaCoefs ("V0", True, [0,0,0], 0)

#comInit = fullBody.getCenterOfMass (); plotSphere (comInit, rr, "comInit", [1,0,0,1], 0.02) # red = where the COM is at q_init_test
#plotSphere (q_init_test[0:3], rr, "comInitRef", [0,1,0,1], 0.02) # green = where we want the COM to be

q_goal = flexion [::]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]
dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[len(Vimplist)-1], theta_goal)
fullBody.setCurrentConfig (q_goal); rr(q_goal)
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True, 2); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[rLegId, lLegId])
fullBody.setFullbodyV0fThetaCoefs ("Vimp", True, [0,0,0], 0)

#comGoal = fullBody.getCenterOfMass (); plotSphere (comGoal, rr, "comGoal", [1,0,0,1], 0.02)
#plotSphere (q_goal_test[0:3], rr, "comGoalRef", [0,1,0,1], 0.02) # green = where we want the COM to be

psf.setPlannerIterLimit (100)
timeStep = 0.0015 # 0.003 pas mal
maxIter = 140
plotSphere ([0,0,0], rr, "COM", [0,0,1,1], 0.02) # COM blue sphere for the robot animation (pathPlayer)

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter); pathIdBall = psf.numberPaths ()-1 # no timed-interpolation
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True); pathIdBallTimed = psf.numberPaths ()-1 # timed-interpolation
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, False, True); pathIdCom = psf.numberPaths ()-1 # COM projection
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True, True); pathIdComTimed = psf.numberPaths ()-1 # COM projection + timed
print("ballistic-interpolation finished")

rr(q_init_test)


"""
print("Start projection of states to COM")
fullBody.projectLastStatesComputedToCOM()
statesTimeCOM = fullBody.getlastStatesComputedTime () # project states COM to their parabola positions
numberOfStatesComputedCOM = len(statesTimeCOM)-1
configsCOM = statesTimeCOM [:numberOfStatesComputedCOM]
q = [0,0,0,1,0,0,0]
for i in range (70,120):
    rr(configsCOM[i]); q[0:3]= psf.robot.getCenterOfMass (); rr.client.gui.applyConfiguration ("COM", q[0:7]); rr.client.gui.refresh (); time.sleep(0.1);

print("Start interpolation between COM states")
fullBody.interpolatePathFromLastStatesComputed()
"""


plotComTrajectory (psf, rr, pathIdBall, 70, "test_COM", [0.3,0.3,0.3,1]) # display COM path
#pp.plotRobotTrajectoryWithCOM (3, "COM") # animation with COM


writePathSamples (psf.sampleSubPathCom(pathIdBall, 70), 'skeleton_Com_notProj_path.txt') # write COM-path samples in file for Blender



"""
print("Start comRRT")
fullBody.comRRT(0, 1, psf.numberPaths ()-1, 0) # path = COM path (parabola ?)    NOT WORKING
"""

"""
statesTime = fullBody.getlastStatesComputedTime ()
numberOfStatesComputed = len(statesTime)-1
configs = statesTime [:numberOfStatesComputed]
times = statesTime [numberOfStatesComputed]

fullBody.projectLastStatesComputedToCOM()
statesTimeCOM = fullBody.getlastStatesComputedTime () # project states COM to their parabola positions
numberOfStatesComputedCOM = len(statesTimeCOM)-1
configsCOM = statesTimeCOM [:numberOfStatesComputedCOM]


q = [0,0,0,1,0,0,0]
for i in range (0,numberOfStatesComputedCOM):
    rr(configsCOM[i]); q[0:3]= psf.robot.getCenterOfMass (); rr.client.gui.applyConfiguration ("COM", q[0:7]); rr.client.gui.refresh (); time.sleep(0.2);

# without COM
for i in range (0,numberOfStatesComputed):
    rr(configs[i]); time.sleep(0.2);


### simple config projection to COM IS REALLY WORKING
comQ = [0,0,0,1,0,0,0]
for i in range (0,numberOfStatesComputedCOM):
	q = configsCOM[i]
	rr(q); time.sleep(0.2);
	fullBody.setCurrentConfig (q)
	comQ [0:3] = fullBody.getCenterOfMass ()
	gui.applyConfiguration ("COM", comQ)
	gui.addToGroup ("COM", rr.sceneName)

"""



pathAndCOMToYamlFile (psf, rr, "skeleton_test_frames.yaml", "skeleton", pathIdBall, q_goal_test, 0.02) # BLENDER
#pathToYamlFile (psf, rr, "skeleton_test_timedFrames.yaml", "skeleton", pathIdBallTimed, q_goal_test, 0.02) # BLENDER
#pathAndCOMToYamlFile (psf, rr, "skeleton_ComProj_frames.yaml", "skeleton", 3, configsCOM[numberOfStatesComputedCOM-1], 0.02) # BLENDER
#pathAndCOMToYamlFile (psf, rr, "skeleton_test_ComProj_timedFrames.yaml", "skeleton", pathIdComTimed, q_goal_test, 0.02) # BLENDER


#r.startCapture("skeletonDesert","png") ; r.stopCapture()

## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py


"""
pathToYamlFile (psf, rr, "skeleton_test_ComProj_frames.yaml", "skeleton", pathId, q_goal_test, 0.02)

pathSamples = plotSampleSubPath (psf.client.problem, rr, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])
writePathSamples (pathSamples, 'skeleton_test_ComProj_path.txt')
"""


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

fullBody.getPathPlannerFails ()


#fullBody.projectStateToCOM (0, q_init_test[0:3])
#rr(fullBody.getlastStatesComputedTime ()[0])    # compare to rr(q_init_test)

#i = 6; fullBody.projectStateToCOM (i, configs[i][0:3])
#rr(fullBody.getlastStatesComputedTime ()[i]); fullBody.setCurrentConfig (fullBody.getlastStatesComputedTime ()[i])    # compare to rr(configs[i])
# plotSphere (fullBody.getCenterOfMass (), r, "test"+str(i), [0,0,1,1], sphereSize); rr(configs[i]); fullBody.setCurrentConfig (configs[i])
# plotSphere (configs[i][0:3], r, "testRef"+str(i), [0,1,0,1], sphereSize)
"""

"""
# final projected to COM config
q=[-1.0875263725551425, 0.006735958106127448, 0.43805075939387766, 0.9994402080017847, -0.01325109463273018, -0.030493413227879438, -0.003718987778615565, -0.04200312077403028, 0.005603076566437497, 0.06155555370078577, -0.021458228341347477, 0.005439701388680359, 0.05173704899515491, -5.8541879700212505e-21, -7.572449228762932e-19, -5.529072595949309e-18, 0.19536717500739434, 0.09929665677123363, -0.19623136919088727, -0.9980989747445629, -2.4973934135182803, -0.00040420793355929645, -0.0002575470182509455, -0.19179625014979873, -0.09656347209848644, -0.19219617464300157, -0.9943451371504796, 2.4999999966589788, -0.0004740688291361102, 0.00012578111711163907, 0.3108690772554518, -0.32611153558044276, -1.5691897819034257, 2.5384004387680768, -0.7494690785414246, 0.24689193519547561, -0.17048447103387537, 0.055823465687198155, -0.33222383364673896, -1.3986998611185315, 2.648746814159253, -0.8920223503566381, -0.03247709720951528, -0.18544262656723548]
plotSphere (fullBody.getCenterOfMass (), rr, "projected_com1", [1,0,0,1], 0.05)
plotFrame (rr, 'projected_root', q[0:3], 0.3)  # ROOT HAS MOVED, IT SHOULD NOT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  idee: creer moi meme les contraintes du COM pour lockdof le root !
#plotSphere (q[0:3], rr, "projected_root", [0,0,1,1], 0.041)
"""

"""
q = q_contact_landing [::]
q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.0; rr(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['LHumerus']] = 0.5; rr(q) # 1.4
q [fullBody.rankInConfiguration ['LElbow_J1']] = -0.4; rr(q) # -1
q [fullBody.rankInConfiguration ['LForearm']] = -1.2; rr(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = 0; rr(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RHumerus']] = 0.5; rr(q) # 1.5
q [fullBody.rankInConfiguration ['RElbow_J1']] = -0.4; rr(q) # -1.1
q [fullBody.rankInConfiguration ['RForearm']] = 1.2; rr(q)
"""

