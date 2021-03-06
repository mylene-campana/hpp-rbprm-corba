#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a ??? environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/animals_description/urdf/skeleton.urdf -o skeleton_blend.py

#TODO: Lombaire joints -> impose same values


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
#import numpy as np
from viewer_display_library import normalizeDir, plotCone, plotFrame, plotThetaPlane, shootNormPlot, plotStraightLine, plotConeWaypoints, plotSampleSubPath, contactPosition, addLight, plotSphere, plotSpheresWaypoints, plotConesRoadmap, plotEdgesRoadmap


rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'skeleton_trunk_flexible'
urdfNameRoms = ['LHandSphere','RHandSphere','LFootSphere','RFootSphere']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-8.5, 8.5, -3, 3, 0.1, 9])
#rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.setFilter(urdfNameRoms)
#rbprmBuilder.setNormalFilter('LHandSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('RHandSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('LFootSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('RFootSphere', [0,0,1], 0.5)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(4)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

rbprmBuilder.getCurrentConfig ()


ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

#addLight (r, [-2,0,1,1,0,0,0], "li")

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ("hpp-rbprm-corba","high_plateforms","high_plateforms")
#addLight (r, [-3,3,4,1,0,0,0], "li"); addLight (r, [3,-3,4,1,0,0,0], "li1")

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config  # Mylène
q11[0:7] = [6.3, 0, 0.4, 0, -math.sqrt(2)/2.0, 0, math.sqrt(2)/2.0]
r(q11)

rbprmBuilder.isConfigValid(q11)
rbprmBuilder.client.rbprm.rbprm.isRbprmValid (q11)

q22 = q11[::]
#q22[0:7] = [-7.3, -2, 7.9, 0, -math.sqrt(2)/2.0, 0, math.sqrt(2)/2.0] # final
q22[0:7] = [-6.5, 2.4, 0.4, 0, -math.sqrt(2)/2.0, 0, math.sqrt(2)/2.0] # interm
r(q22)

rbprmBuilder.isConfigValid(q22)
rbprmBuilder.client.rbprm.rbprm.isRbprmValid (q22)

#q1 = cl.rbprmBuilder.projectOnObstacle (q11, 0.001); q2 = cl.rbprmBuilder.projectOnObstacle (q22, 0.001)
#q1 = rbprmBuilder.getCurrentConfig (); q2 = rbprmBuilder.getCurrentConfig ();
#rbprmBuilder.isConfigValid(q1); rbprmBuilder.isConfigValid(q2); r(q1)

offsetOrientedPath = 1 # If remove oriented path computation in ProblemSolver, set to 1 instead of 2

#ps.addPathOptimizer("RandomShortcut")
#ps.client.problem.selectSteeringMethod("SteeringDynamic")
ps.selectPathPlanner("PRMplanner")
ps.setInitialConfig (q11)
ps.addGoalConfig (q22)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

ps.client.problem.setFrictionCoef(1.2); ps.client.problem.setMaxVelocityLim(6.5)
ps.clearRoadmap();
solveTime = ps.solve ()
pp.displayPath(0)

pathId = ps.numberPaths()-offsetOrientedPath # path without orientation stuff

pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path0", [0,0,1,1])
plotCone (q1, cl, r, "cone_first", "friction_cone_SG2"); plotCone (q2, cl, r, "cone_second", "friction_cone_SG2")
plotConeWaypoints (cl, pathId, r, "cone_wp_group", "friction_cone_WP2")


## 3D viewer tools ##
plotFrame (r, 'frame_group', [0,0,0], 0.6)

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

gui.removeFromGroup("path0",r.sceneName)
gui.getNodeList()
ps.numberNodes()


# Plot cones and edges in viewer
plotConesRoadmap (cl, r, 'cone_rm_group', "friction_cone2")
plotEdgesRoadmap (cl, r, 'edgeGroup', 70, [0,1,0.2,1])



gui = r.client.gui
gui.setCaptureTransform ("frames.yaml ", ["skeleton_trunk_flexible"])
q = q11
r (q); cl.rbprmBuilder.setCurrentConfig(q)
gui.refresh (); gui.captureTransform ()


gui.setVisibility('skeleton_trunk_flexible/thorax_rhand_rom',"OFF")
gui.setVisibility('skeleton_trunk_flexible/pelvis_rfoot_rom',"OFF")


gui.addLight ("li", r.windowId, 0.0001, [0.9,0.9,0.9,1])
gui.addToGroup ("li", r.sceneName)
gui.applyConfiguration ("li", [1,0,0,1,0,0,0])
gui.refresh ()


rbprmBuilder.isConfigValid(q11)


rbprmBuilder.client.rbprm.rbprm.setRbShooter ()
q = rbprmBuilder.client.rbprm.rbprm.rbShoot ()
r(q)
rbprmBuilder.client.rbprm.rbprm.isRbprmValid (q)

rbprmBuilder.client.rbprm.rbprm.setRbShooter ()
r(rbprmBuilder.client.rbprm.rbprm.rbShoot ())


