#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a desert environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton.urdf -o skeleton_blend.py
#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton_trunk_flexible.urdf -o skeleton_trunk_blend.py


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
#import numpy as np
from viewer_library import *


rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'skeleton_trunk_flexible'
#urdfNameRoms = ['LHandSphere','RHandSphere','LFootSphere','RFootSphere']
#urdfNameRoms = ['LFootSphere','RFootSphere']
urdfNameRoms = ['LFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-2, 5, -3, 3, -1.7, 2.5] # first
#base_joint_xyz_limits = [-2, 4, -0.5, 6, -1.7, 2.5] # second

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
#rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.setFilter(urdfNameRoms)
#rbprmBuilder.setNormalFilter('LHandSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('RHandSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('LFootSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('RFootSphere', [0,0,1], 0.5)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

rbprmBuilder.getCurrentConfig ()


ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
#r(rbprmBuilder.getCurrentConfig ())


r.loadObstacleModel (packageName,"desert","desert")
#ps.loadObstacleFromUrdf(packageName,'desert','')
#addLight (r, [-3,0,3,1,0,0,0], "li"); addLight (r, [3,0,3,1,0,0,0], "li1")

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config  # Mylene
q11[0:7] = [2.55, 0.8, -1.0, 0.8924, -0.09905, 0.23912, -0.36964]; r(q11)

rbprmBuilder.isConfigValid(q11)


q22 = q11[::]
q22[0:7] = [0.3, 1.2, -1.1, -0.08419, 0.25783, 0.02256, -0.96225]; r(q22) # first
#q22[0:7] = [-0.5, 5, -1.22, 0.93612, 0.02981, 0.34072, 0.0819]; r(q22) # second

rbprmBuilder.isConfigValid(q22)

offsetOrientedPath = 1 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

ps.selectPathPlanner("PRMplanner")
ps.setInitialConfig (q11)
ps.addGoalConfig (q22)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

ps.client.problem.setFrictionCoef(1.2); ps.client.problem.setMaxVelocityLim(7)
ps.clearRoadmap();
t = ps.solve ()
pathId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.displayPath(0,[0.0, 0.0, 0.8, 1.0]) # blue

# Write data to log file
pfr = ps.client.problem.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("skeleton_desert_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

rob = rbprmBuilder.client.basic.robot
#r(q11)

q_far = q11[::]; q_far[0:3] = [-10,0,10]; r(q_far) # move RB-robot away in viewer

## DEBUG tools ##
"""
cl.obstacle.getObstaclePosition('decor_base')
rbprmBuilder.isConfigValid(q1)
rbprmBuilder.setCurrentConfig(q1)
res=rbprmBuilder.distancesToCollision()
r( ps.configAtParam(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(rbprmBuilder.distancesToCollision()[0])
rbprmBuilder.getJointNames ()
rbprmBuilder.getConfigSize ()
r.client.gui.getNodeList()
rbprmBuilder.client.rbprm.rbprm.setRbShooter ()
q = rbprmBuilder.client.rbprm.rbprm.rbShoot ()
r(q)
rbprmBuilder.client.rbprm.rbprm.isRbprmValid (q)

rbprmBuilder.client.rbprm.rbprm.setRbShooter ()
r(rbprmBuilder.client.rbprm.rbprm.rbShoot ())

ps.client.problem.getResultValues ()
"""

## 3D viewer tools ##
"""
plotFrame (r, 'frame_group', [0,0,0], 0.6)

gui.removeFromGroup("path0",r.sceneName)
gui.getNodeList()
ps.numberNodes()

pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path0", [0,0,1,1])
plotCone (q1, cl, r, "cone_first", "friction_cone_SG2"); plotCone (q2, cl, r, "cone_second", "friction_cone_SG2")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone_WP2")

# Plot cones and edges in viewer
plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone2")
plotEdgesRoadmap (cl, r, 'edgeGroup', 70, [0,1,0.2,1])

gui = r.client.gui
gui.setCaptureTransform ("frames.yaml ", ["skeleton_trunk_flexible"])
q = q11
r (q); cl.rbprmBuilder.setCurrentConfig(q)
gui.refresh (); gui.captureTransform ()

gui.setVisibility('skeleton_trunk_flexible/thorax_rhand_rom',"OFF")

q = q_goal_test [0:7]
q[0] = q[0] + 1; q[2] = q[2] + 1
gui.addLight ("li", r.windowId, 0.0001, [0.9,0.9,0.9,1])
gui.addToGroup ("li", r.sceneName)
gui.applyConfiguration ("li", q)
gui.refresh ()

rbprmBuilder.client.rbprm.rbprm.isRbprmValid (q11) ?? WHY not working ?


## Export path to BLENDER
pathId = 0; dt = 0.01; gui.setCaptureTransform ("skeleton_trunk_path.yaml", ["skeleton_trunk_flexible"])
PL = ps.pathLength(pathId)
FrameRange = np.arange(0,PL,dt)
numberFrame = len(FrameRange)

# test frame capture
q = q11; r (q); gui.refresh (); gui.captureTransform ()
q = q22; r (q); gui.refresh (); gui.captureTransform ()

# capture path
for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); gui.refresh (); gui.captureTransform ()

r (q_goal); robot.setCurrentConfig(q_goal); gui.refresh (); gui.captureTransform ()


"""




