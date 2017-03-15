#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import spiderman_cube_BackJump_path as tp


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 0 # tp.ecsSize
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)

r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody )
rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr)
q_0 = fullBody.getCurrentConfig(); rr(q_0)

rr([-1.48516,-6.79229,2.7369,0.731863,0,0,0.681452,0,0,0,0,0,0,-1.79313,0.128186,-0.0767528,-0.359468,-0.636375,-0.231712,-0.0153261,-1.9612,-0.258181,0.466964,0.0203902,0.500375,0.368077,-0.114644,-0.628298,-0.343483,0.154009,1.51713,-0.25917,-0.0876683,0.0725471,-0.782317,0.245577,-0.109287,1.56199,-0.351043,0.140441,0.055906])

### DEBUG HANDSPHERE FRAME !!!
"""
qt = [-1.2,-2.7,3.6,0.707107,0,0,0.707107,0,0,0,0,0,0,-1.79313,0.128186,-0.0767528,-0.359468,-0.636375,-0.231712,-0.0153261,-1.89126,-0.61356,-0.35924,-0.869512,0.278513,0.829376,0.280423,-1.40015,-0.844591,-0.0225695,1.62296,-0.920702,-0.466159,-0.719726,-1.21601,0.841949,0.2666,1.02717,-1.1,0.122667,-0.848427]
rr(qt)
plotJointFrame (rr, psf, qt, "SpidermanLHandSphere", 0.2,"qt")
x=0.08541;y=0.03916;z=0.04994; norm=math.sqrt(x*x+y*y+z*z); x=x/norm; y=y/norm; z=z/norm;
vectorL = [x,y,z]
plotVectorInJointFrame (rr, psf, qt, "SpidermanLHandSphere", vectorL, [0.5,0.5,0.5,1],"qt")

x=-0.0873;y=0.0176;z=0.03516; norm=math.sqrt(x*x+y*y+z*z); x=x/norm; y=y/norm; z=z/norm;
vectorR = [-x,-y,-z]
plotJointFrame (rr, psf, qt, "SpidermanRHandSphere", 0.2,"qt")
plotVectorInJointFrame (rr, psf, qt, "SpidermanRHandSphere", vectorR, [0.5,0.5,0.5,1],"qt")


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

#"""
heuristicName = 'static'
rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
fullBody.addLimbDatabase('./Spiderman_rleg_flexion_6DOF.db',rLegId,heuristicName)
fullBody.addLimbDatabase('./Spiderman_lleg_flexion_6DOF.db',lLegId,heuristicName)
fullBody.addLimbDatabase('./Spiderman_rarm_flexion_3DOF.db',rarmId,heuristicName)
fullBody.addLimbDatabase('./Spiderman_larm_flexion_3DOF.db',larmId,heuristicName)
#fullBody.addLimbDatabase('./Spiderman_rleg_qcontact_3DOF.db',rLegId,heuristicName)
#fullBody.addLimbDatabase('./Spiderman_lleg_qcontact_3DOF.db',lLegId,heuristicName)
#fullBody.addLimbDatabase('./Spiderman_rarm_qcontact_3DOF.db',rarmId,heuristicName)
#fullBody.addLimbDatabase('./Spiderman_larm_qcontact_3DOF.db',larmId,heuristicName)
print("Limbs added to fullbody")


#fullBody.runSampleAnalysis( "RefPoseFeet", True) #done in compute_db now


#id = r.client.gui.getWindowID("window_hpp_")
#rr.client.gui.attachCameraToNode("spiderman/Thorax",id)

confsize = len(tp.q11)-ecsSize
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = q_0 [::]; q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.orientedpathId # tp.orientedpathId or tp.solutionPathId
trunkPathwaypoints = ps.getWaypoints (entryPathId)
q_init[0:confsize-tp.ecsSize] = trunkPathwaypoints[0][0:confsize-tp.ecsSize]
q_goal[0:confsize-tp.ecsSize] = trunkPathwaypoints[len(trunkPathwaypoints)-1][0:confsize-tp.ecsSize]


fullBody.setCurrentConfig (q_init)
fullBody.isConfigValid(q_init)
q_init_test = fullBody.generateContacts(q_init,[0,0,1], True); rr (q_init_test)
fullBody.isConfigValid(q_init_test)


fullBody.setCurrentConfig (q_goal)
fullBody.isConfigValid(q_goal)
q_goal_test = fullBody.generateContacts(q_goal, [0,0,-1], True); rr (q_goal_test)
fullBody.isConfigValid(q_goal_test)

fullBody.setStartState(q_init_test,[rLegId,lLegId])
fullBody.setEndState(q_goal_test,[rLegId,lLegId])





extending = [0, 0, 0, 1, 0, 0, 0, 0.8, 0.0, 0, -0.6, 0.0, 0,0.4, -0.9, 0.9, 0, 0, 0.0, 0.0, 0.4, 0.9, -0.9, 0, 0, 0.0,-0.0, -2, -0.5, 0.3, 2.2, 0.7, 0, 0.0, -2, 0.5, -0.3, 2.2, 0.7, 0.0, 0.0]
q_contact_takeoff = [0, 0, 0, 1, 0, 0, 0,  0.0, 0, 0, 0.0, 0.0, 0.0, -2, 0.3, -0.3, 0, -0.6, 0.0, 0.0, -2, -0.3, 0.3, 0, 0.6, 0.0,-0.0, -1.9, -0.3, -0.2, 1.9, -0.6, 0, 0.0, -1.9, 0.3, 0.2, 1.9, -0.6, 0, 0.0]
flexion = [0, 0, 0, 1, 0.0, 0.0, 0.0, 0.7, 0, 0, -0.7, 0.0,0, 0.5, 0.7, 0.5, 0, -0.6, 0.0, 0.0, 0.5, -0.7, -0.5, 0,0.6, 0.0, -0.0, -1.2, -0.3, -0.2, 2.2, -0.9, 0, 0.0, -1.2,0.3, 0.2, 2.2, -0.9, 0, 0.0]

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")
timeStep = 0.005
psf.setPlannerIterLimit (10)


## DEBUG !!
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


print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, timeStep) #  -> now also set lastComputedStates_ stack
print("ballistic-interpolation finished")


pp.speed=2
#rr(pp.client.problem.configAtParam(psf.numberPaths () -1,0))
#pp(psf.numberPaths () -1)



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

"""
qe = extending[::]
qe[0:7] = [-2.025129887082707,
40.59097542330351,
128.97577375406138,
 1, 0, 0, 0, 0.0, 0.0, 0.8, 0.0, 0.0, -0.6, -0.9, 0.9, 0.4,
 0, 0, 0.0, 0.0, -0.9, 0.9, 0.4, 0, 0, 0.0, 0.0, 0.5, 0.5,
 -2, 2.2, 0.7, 0, 0.5, 0.5, -2, 2.2, 0.7, 0.0]

rr(qe)
"""
