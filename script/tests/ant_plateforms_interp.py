#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import ant_plateforms_path as tp



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "ant"
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

extending = [0.0, 0.0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.1, 0.0, -0.5, 0.0, 0.0, 0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0, 0.0, -1.1, 0.0, 0.5, 0.0, 0.0, -0.2, 0.0, 0,0,0,0]
flexion = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.0, 0.2, 0.0, 0,0,0,0]
fullBody.isConfigValid(extending); fullBody.isConfigValid(flexion)
fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")

nbSamples = 30000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.03 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,-1]
heuristic = "EFORT_Normal"
lfLegId = 'LFFoot'
lfLeg = 'LFThigh_rx'
lffoot = 'LFFootSphere'
fullBody.addLimb(lfLegId,lfLeg,lffoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

lmLegId = 'LMFoot'
lmLeg = 'LMThigh_rx'
lmfoot = 'LMFootSphere'
fullBody.addLimb(lmLegId,lmLeg,lmfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

lbLegId = 'LBFoot'
lbLeg = 'LBThigh_rx'
lbfoot = 'LBFootSphere'
fullBody.addLimb(lbLegId,lbLeg,lbfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

rfLegId = 'RFFoot'
rfLeg = 'RFThigh_rx'
rffoot = 'RFFootSphere'
fullBody.addLimb(rfLegId,rfLeg,rffoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)


rmLegId = 'RMFoot'
rmLeg = 'RMThigh_rx'
rmfoot = 'RMFootSphere'
fullBody.addLimb(rmLegId,rmLeg,rmfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

rbLegId = 'RBFoot'
rbLeg = 'RBThigh_rx'
rbfoot = 'RBFootSphere'
fullBody.addLimb(rbLegId,rbLeg,rbfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)
print("Limbs added to fullbody")


confsize = len(tp.q11)
fullConfSize = len(fullBody.getCurrentConfig()) # with or without ECS in fullbody
q_init = fullBody.getCurrentConfig(); q_goal = q_init [::]

# WARNING: q_init and q_goal may have changed in orientedPath
entryPathId = tp.solutionPathId # tp.orientedpathId or tp.solutionPathId or tp.orientedpathIdBis
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

fullBody.setStartState(q_init_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])
fullBody.setEndState(q_goal_test,[lfLegId,lmLegId,lbLegId,rfLegId,rmLegId,rbLegId])

psf.setPlannerIterLimit (500)

print("Start ballistic-interpolation")
fullBody.interpolateBallisticPath(entryPathId, 0.01)


pp = PathPlayer (fullBody.client.basic, rr)
pp.speed=1.5

#fullBody.timeParametrizedPath(psf.numberPaths() -1 )
#pp(psf.numberPaths ()-1)


"""
## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
pathId = psf.numberPaths()-1 # path to export
plotCone (q_init_test, psf, rr, "cone_start", "friction_cone2_blue")
plotCone (q_goal_test, psf, rr, "cone_goal", "friction_cone2_blue")
plotConeWaypoints (psf, pathId, r, "cone_wp_group", "friction_cone2_blue")

pathSamples = plotSampleSubPath (psf.client.problem, rr, entryPathId, 70, "sampledPath", [1,0,0,1])

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_start','cone_start.dae')
gui.writeNodeFile('cone_goal','cone_goal.dae')
writePathSamples (pathSamples, 'antPlateforms_path.txt')
pathJointConfigsToFile (psf, rr, "antPlateforms_jointConfigs.txt", pathId, q_goal_test, 0.01)
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

"""
# extending
q = q_0; q [2] = 1
q [fullBody.rankInConfiguration ['LFThigh_ry']] = 1.1; rr(q)
q [fullBody.rankInConfiguration ['LFShank_ry']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['LFFoot_ry']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LMThigh_ry']] = 1.1; rr(q)
q [fullBody.rankInConfiguration ['LMShank_ry']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['LMFoot_ry']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LBThigh_ry']] = 1.1; rr(q)
q [fullBody.rankInConfiguration ['LBShank_ry']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['LBFoot_ry']] = 0.2; rr(q)

q [fullBody.rankInConfiguration ['RFThigh_ry']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['RFShank_ry']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['RFFoot_ry']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['RMThigh_ry']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['RMShank_ry']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['RMFoot_ry']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['RBThigh_ry']] = -1.1; rr(q)
q [fullBody.rankInConfiguration ['RBShank_ry']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['RBFoot_ry']] = -0.2; rr(q)

biRRT converge pas:
start= [7.37092,1.00709,7.74434,0.38411,0.100199,-0.389878,0.830912,0,0,0,0,0,0,1.15668,0.671725,-0.597937,-0.297447,-0.785419,-0.160423,-0.0130058,-0.13424,-0.037199,1.01357,-0.112751,-0.510943,0.000110022,0.0622325,0.156809,-0.0238343,-0.387803,0.222064,-0.362172,0.243999,-0.046081,0.113324,-0.319184,0.0564558,-0.144555,-0.850792,0.0862704,0.200879,0.0246342,-0.0178707,-0.184449,0.00759846,0.156328,-0.781473,0.0677216,0.293257,0.0318036,-0.0068635,-0.160168,0.0036874,-0.113042,-0.828873,-0.0127002,0.17509,-0.0579396,0.0153663,-0.126013,-0.00185134,0.235942,-0.0315914,-0.162165,0.638267]
end= [7.94655,0.751815,6.93998,0.867979,0.274304,-0.383787,0.15517,0,0,0,0,0,0,-0.515819,-0.533339,0.266187,0.572353,-0.513911,0.16618,0.0693528,0.0507565,0.427327,1.13099,-0.7,-0.632192,-0.336571,0.143421,-0.152314,-0.182467,-0.478847,0.958497,-0.615771,0.0382872,-0.250416,0.00312143,-0.274915,0.146908,-0.973273,-0.584981,0.591201,-0.655292,0.617156,-0.157746,0.0738587,0.0428424,0.0944445,-0.468691,0.423731,0.920753,0.0827405,0.0603013,0.0639058,-0.131609,-0.197893,-0.212242,-0.199967,-0.56676,0.45163,0.153163,0.0460408,-0.0788465,3.2527e-86,4.32554e-91,1.17923e-95,8.54295e-72]


jointConfigsToFile (psf, rr, "ant_apex_config.txt", extending)
jointConfigsToFile (psf, rr, "ant_contact_config.txt", flexion)
"""
