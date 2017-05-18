#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import spiderman_backJump_path as tp


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr); pp.speed = 0.6
q_0 = fullBody.getCurrentConfig(); rr(q_0)


flexion  = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0.4, 0, -0.0, -0.5, 0, 0.2, 0.9, 0, -0.6, 0, 0, 0, -0.2, 0.9, 0, -0.6, 0, 0, 0, 0.6, -1.3, 0.6, 2.2, -0.9, 0, 0.0, -0.6, -1.3, -0.6, 2.2, -0.9, 0, 0.0]
q_contact_takeoff = [0, 0, 0, 1, 0, 0, 0, 0, 0.0, 0, -0.0, 0.0, 0.0, 2.2, 0.1, 0.3, -1.5, 0.8, 0, 0, -2.2, 0.1, -0.3, -1.5, -0.8, 0, 0, 0.3, -1.1, 0.2, 2, -0.8, 0, 0.0, -0.3, -1.1, -0.2, 2, -0.8, 0, 0.0]
extending = [0, 0, 0, 1, 0, 0, 0, -0.0, 0.8, 0, -0.0, -0.6, 0, 1.5, 0.5, 1, 0, 0, 0, 0, -1.5, 0.5, -1, 0, 0, 0, 0, 1.4, -1.2, 1.6, 2.1, 0.4, 0, 0.0, -1.4, -1.2, -1.6, 2.1, 0.4, 0.0, 0.0]

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")

heuristicName = 'static'
rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./Spiderman_rleg_flexion_6DOF_EN.db',rLegId,heuristicName) # ok
fullBody.addLimbDatabase('./Spiderman_lleg_flexion_6DOF_EN.db',lLegId,heuristicName) # ok
#fullBody.addLimbDatabase('./Spiderman_rleg_flexion_3DOF_EN.db',rLegId,heuristicName) # 
#fullBody.addLimbDatabase('./Spiderman_lleg_flexion_3DOF_EN.db',lLegId,heuristicName) # 
print("Limbs added to fullbody")
#x=-0.07;y=0.03;z=0.04; norm=math.sqrt(x*x+y*y+z*z); x=x/norm; y=y/norm; z=z/norm; vectorR = [x,y,z]
#fullBody.addLimb(rarmId,'RHumerus_rx','SpidermanRHandSphere',[0,0,0],[0,0,1], 0.03, 0.08, 50000, "EFORT_Normal", 0.01,"_6_DOF")
#x=0.08541;y=0.03916;z=0.04994; norm=math.sqrt(x*x+y*y+z*z); x=x/norm; y=y/norm; z=z/norm; vectorL = [-x,-y,-z]
#fullBody.addLimb(larmId,'LHumerus_rx','SpidermanLHandSphere',[0,0,0],[0,0,1], 0.03, 0.08, 50000, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimbDatabase('./Spiderman_larm_contact_6DOF_EN.db',larmId,heuristicName) # ok
fullBody.addLimbDatabase('./Spiderman_rarm_contact_6DOF_EN.db',rarmId,heuristicName) # ok
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathId # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
trunkPathwaypoints = ps.getWaypoints (entryPathId)

q_init = flexion [::]
q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
dir_init = [-V0list [0][0],-V0list [0][1],-V0list [0][2]] # first V0
theta_0 = math.atan2(trunkPathwaypoints[1][1] - q_init[1], trunkPathwaypoints[1][0] - q_init[0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[0], theta_0)
fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
dir_init = (dir_init/np.linalg.norm(dir_init)).tolist()
q_init_test = fullBody.generateContacts(q_init, dir_init, True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)
fullBody.setStartState(q_init_test,[rLegId, lLegId,rarmId,larmId])
fullBody.setFullbodyV0fThetaCoefs ("V0", True, [0,0,0], 0)


q_goal = flexion [::]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]
dir_goal = (np.array(Vimplist [len(Vimplist)-1])).tolist() # last Vimp
theta_goal = math.atan2(q_goal[1] - trunkPathwaypoints[len(trunkPathwaypoints)-2][1], q_goal[0] - trunkPathwaypoints[len(trunkPathwaypoints)-2][0]) # first theta (of first path)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[len(Vimplist)-1], theta_goal)
fullBody.setCurrentConfig (q_goal)
dir_goal = (dir_goal/np.linalg.norm(dir_goal)).tolist()
q_goal_test = fullBody.generateContacts(q_goal, dir_goal, True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)
fullBody.setEndState(q_goal_test,[rLegId, lLegId,rarmId,larmId])
fullBody.setFullbodyV0fThetaCoefs ("Vimp", True, [0,0,0], 0)


### TEST OF CONTACT CREATION FOR INTERMEDIATE CONFIG, NOT USED FOR INTERPOLATION
"""q_tmp = q_contact_takeoff [::]; q_tmp[0:confsize-tp.ecsSize] = trunkPathwaypoints[1][0:confsize-tp.ecsSize]
theta_0 = math.atan2(q_goal[1] - q_tmp[1], q_goal[0] - q_tmp[0])
fullBody.setFullbodyV0fThetaCoefs ("V0", False, V0list[1], theta_0)
theta_goal = math.atan2(q_tmp[1] - q_init[1], q_tmp[0] - q_init[0])
fullBody.setFullbodyV0fThetaCoefs ("Vimp", False, Vimplist[0], theta_goal)
dir_array = np.array(Vimplist[0]) - np.array(V0list[1])
dir_tmp = (dir_array/np.linalg.norm(dir_array)).tolist()
fullBody.setCurrentConfig (q_tmp)
fullBody.isConfigValid(q_tmp)
q_tmp_test = fullBody.generateContacts(q_tmp, dir_tmp, True, False); rr (q_tmp_test) # last False parameter : do not use Flexion config to replace non-contacting limbs
fullBody.isConfigValid(q_tmp_test)
fullBody.setFullbodyV0fThetaCoefs ("V0", True, [0,0,0], 0)
fullBody.setFullbodyV0fThetaCoefs ("Vimp", True, [0,0,0], 0)"""

psf.setPlannerIterLimit (50)
timeStep = 0.002
maxIter = 100

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter) # no timed-interpolation
#fullBody.interpolateBallisticPath(entryPathId, timeStep, maxIter, True) # timed-interpolation
print("ballistic-interpolation finished")

rr(q_init_test)


## Save data

nbWaypoints = len (trunkPathwaypoints)
nbParabolas = nbWaypoints - 1
nbCentroidalFails = fullBody.getcentroidalConeFails ()
nbFailsLimbRRT = fullBody.getPathPlannerFails ()

# Write important results #
f = open('results_jumperman_runs.txt','a')
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
main("results_jumperman_runs.txt")
"""


#pp(psf.numberPaths ()-1)
#rr(pp.client.problem.configAtParam(psf.numberPaths () -1,0))

"""
# verify given offset position of contact-point
q = q_init_test
r(q)
fullBody.setCurrentConfig (q)
#posAtester = fullBody.client.basic.robot.computeGlobalPosition(fullBody.client.basic.robot.getJointPosition(rfoot),[0,0,0.2]); sphereName = "machin2"
posAtester = fullBody.client.basic.robot.computeGlobalPosition(fullBody.client.basic.robot.getJointPosition(rHand),[0.1,0,0]); sphereName = "machin2"


r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()


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


## Export path to BLENDER
pathId = 0; dt = 0.01; gui.setCaptureTransform ("skeleton_path.yaml", ["skeleton"])
PL = ps.pathLength(pathId)
FrameRange = np.arange(0,PL,dt)
numberFrame = len(FrameRange)

# test frame capture
q = q_init_test; r (q); gui.refresh (); gui.captureTransform ()
q = q_goal_test; r (q); gui.refresh (); gui.captureTransform ()

# capture path
for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); gui.refresh (); gui.captureTransform ()

r (q_goal); robot.setCurrentConfig(q_goal); gui.refresh (); gui.captureTransform ()

cl = tp.rbprmBuilder.client.basic
plotJointFrame (r, cl, q_init_test, "RFootSphere", 0.15)

q_0 = fullBody.getCurrentConfig()
q = q_0
q [fullBody.rankInConfiguration ['RAnkle_J1']] = 0.6; r(q)

jointConfigsToFile (psf, rr, "spiderman_apex_config.txt", extending)
"""
#rr.addLandmark('spiderman/SpidermanLHandSphere',1)
#id = r.client.gui.getWindowID("window_hpp_")
#rr.client.gui.attachCameraToNode("spiderman/Thorax",id)
#fullBody.runSampleAnalysis( "RefPoseFeet", True) #done in compute_db now


"""
q_cube = q_0[::]
q_cube[0:confsize-tp.ecsSize] = tp.q_cube[0:confsize-tp.ecsSize]
q_cube[confsize-tp.ecsSize:len(q_0)] = q_contact_takeoff[confsize-tp.ecsSize:len(q_0)]
fullBody.setCurrentConfig (q_cube); fullBody.isConfigValid(q_cube); rr(q_cube)
q_cube_cont = fullBody.generateContacts(q_cube, [0,-1,0], True, False); rr (q_cube_cont); fullBody.isConfigValid(q_cube_cont)

qt = [-1.2,-2.7,3.6,0.707107,0,0,0.707107,0,0,0,0,0,0,-1.79313,0.128186,-0.0767528,-0.359468,-0.636375,-0.231712,-0.0153261,-1.89126,-0.61356,-0.35924,-0.869512,0.278513,0.829376,0.280423,-1.40015,-0.844591,-0.0225695,1.62296,-0.920702,-0.466159,-0.719726,-1.21601,0.841949,0.2666,1.02717,-1.1,0.122667,-0.848427]
fullBody.setCurrentConfig(qt); rr(qt)
plotJointFrame (rr, psf, qt, "SpidermanRHandSphere", 0.2)
"""

### DEBUG HANDSPHERE FRAME !!!
"""
qt = [-1.2,-2.8,3.6,0.707107,0,0,0.707107,0,0,0,0,0,0,-1.79313,0.128186,-0.0767528,-0.359468,-0.636375,-0.231712,-0.0153261,-1.89126,-0.61356,-0.35924,-0.869512,0.278513,0.829376,0.280423,-1.40015,-0.844591,-0.0225695,1.62296,-0.920702,-0.466159,-0.719726,-1.21601,0.841949,0.2666,1.02717,-1.1,0.122667,-0.848427]
rr(qt)
plotJointFrame (rr, psf, qt, "SpidermanLHandSphere", 0.2,"qtLframe")
x=0.08541;y=0.03916;z=0.04994; norm=math.sqrt(x*x+y*y+z*z); x=x/norm; y=y/norm; z=z/norm;
vectorL = [x,y,z]
plotVectorInJointFrame (rr, psf, qt, "SpidermanLHandSphere", vectorL, [0.5,0.5,0.5,1],"qtLVec")

x=-0.07;y=0.03;z=0.04; norm=math.sqrt(x*x+y*y+z*z); x=x/norm; y=y/norm; z=z/norm;
vectorR = [x,y,z]
plotVectorInJointFrame (rr, psf, qt, "SpidermanRHandSphere", vectorR, [0.5,0.5,0.5,1],"qtRVec")
plotJointFrame (rr, psf, qt, "SpidermanRHandSphere", 0.2,"qt")


qt = [-1.2,-2.8,3.6,0.731863,0,0,0.681452,0,0,0,0,0,0,-0.963251,-0.483015,-1.35589,-0.267423,-0.183665,-0.712411,0.2636,-2,-0.3,0.3,0,0.6,0,0,-1.27398,-0.504298,0.111054,1.56807,-0.934061,-0.560327,-0.999254,-1.2,0.3,0.2,2.2,-0.9,0,0]
rr(qt)
vl = [0.368022, -0.802675, 0.469332]
vlimb = [0.368022, -0.802675, 0.569332]
plotVectorInJointFrame (rr, psf, qt, "SpidermanLHandSphere", vlimb, [0.5,0.5,1,1],"vlimb2")


nbSamples = 30000; x = 0.03; y = 0.08
rarmId = 'rhand'; rLeg = 'RHumerus_rx'; rfoot = 'SpidermanRHandSphere'
fullBody.addLimb(rarmId,rLeg,rfoot,[0,0,0],vectorR, x, y, nbSamples, "static", 0.01,"_6_DOF")

larmId = 'lhand'; lLeg = 'LHumerus_rx'; lfoot = 'SpidermanLHandSphere'
#fullBody.addLimb(larmId,lLeg,lfoot,[0,0,0],vectorL, x, y, nbSamples, "static", 0.01,"_6_DOF")
print("Arms added to fullbody")


fullBody.setCurrentConfig (qt)
qtt = fullBody.generateContacts(qt, [0,-1,0], True, False); #rr (qtt); fullBody.isConfigValid(qtt)

rr(qttt)
plotJointFrame (rr, psf, qttt, "SpidermanLHandSphere", 0.2,"afterContact")
#plotJointFrame (rr, psf, qttt, "SpidermanRHandSphere", 0.2,"afterContact")
plotVectorInJointFrame (rr, psf, qttt, "SpidermanLHandSphere", vectorL, [0.5,0.5,0.5,1],"afterContact1")
#plotVectorInJointFrame (rr, psf, qtt, "SpidermanRHandSphere", vectorL, [0.5,0.5,0.5,1],"afterContact")
"""
