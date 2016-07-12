#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/animals_description/urdf/skeleton.urdf -o skeleton_blend.py


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'spiderman_trunk'
urdfNameRoms = ['SpidermanLFootSphere','SpidermanRFootSphere','SpidermanLHandSphere','SpidermanRHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-10, 10, -5, 15, 0, 20])
rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.setFilter(urdfNameRoms)
filterRange = 0.6
rbprmBuilder.setNormalFilter('SpidermanLFootSphere', [0,0,1], -1)
rbprmBuilder.setNormalFilter('SpidermanRFootSphere', [0,0,1], -1)
rbprmBuilder.setNormalFilter('SpidermanLHandSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('SpidermanRHandSphere', [0,0,1], filterRange)
rbprmBuilder.setContactSize (0.03,0.08)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
ps.selectPathPlanner("BallisticPlanner") # "PRMplanner"#rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(10000)
rbprmBuilder.setMaxTakeoffVelocity(12)#(8)
rbprmBuilder.setMaxLandingVelocity(15)
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectSteeringMethod("SteeringParabola")


rbprmBuilder.setNumberFilterMatch(2)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ("hpp-rbprm-corba", "cubeWorld", "cubeWorld")
r.addLandmark(r.sceneName,2)
r.addLandmark("spiderman_trunk/base_link",1)
addLight (r, [-10,-5,5,1,0,0,0], "li");
# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config

#q11[0:7] = [-8.5,0,2.6, 1, 0, 0, 0]; r(q11) # edge middle
#q11[0:7] =  [-2.7,0,3.9, 1, 0, 0, 0]; r(q11) # cube WORKING CONTACT
q11[0:7] =  [-7,0,2.5, 1, 0, 0, 0]

#q11[0:7] =  [-2.7,-2.25,4, 0.9217, 0.1022, 0.2192, 0.3034] #WORKING : SIDE
r(q11)


rbprmBuilder.isConfigValid(q11)

q22 = q11[::]

#q22[0:7] =  [-7.2,0,2.6, 1, 0, 0, 0]; r(q22) # back plateform
q22[0:7] =  [-3,8,0.8, 1, 0, 0, 0]; r(q22) # side plateform
#q22[0:7] =   [5,0,3.8,  0.9537, 0, 0.3, 0]; r(q22) # front plateform
rbprmBuilder.isConfigValid(q22)


ps.clearRoadmap();
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

## manually add way point (faster computation for test, work without but it's slow (~ <1minute )

q_cube = q11[::]

q_cube[0:7] =  [-2.6,0,3.9, 1, 0, 0, 0]

waypoints = [q_cube]
pbCl = rbprmBuilder.client.basic.problem
pbCl.prepareSolveStepByStep()
q11 = ps.node(0)
q22 = ps.node(1)
pbCl.addConfigToRoadmap (waypoints[0])
ps.directPath (q11, waypoints[0],False)
pathIds0 = ps.numberPaths () - 1
ps.directPath (waypoints[0], q22,False)
pathId2g = ps.numberPaths () - 1
pbCl.addEdgeToRoadmap (q11, waypoints[0], pathIds0, True)
pbCl.addEdgeToRoadmap (waypoints[0], q22, pathId2g, True)
#########


t = ps.solve ()


q11 = ps.node(0)
q22 = ps.node(1)
#plotCone (q11, ps, r, "cone_first", "friction_cone_SG2");
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
""" 
plotCone (q22, ps, r, "cone_second", "friction_cone_SG2")
qCone = [-2.7, 0.0, 3.9, 0.9537, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
 0.0, 0.0, -0.8689669946343358, -0.00045459419059697023,0.4948698370079671, 0.0]

plotCone (qCone, ps, r, "cone_Mid", "friction_cone_SG2")



"""

##############################

rbprmBuilder.rotateAlongPath (solutionPathId,True)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)
r(pp.client.problem.configAtParam(orientedpathId,0))

"""
V0list = rbprmBuilder.getsubPathsV0Vimp("V0",solutionPathId)
Vimplist = rbprmBuilder.getsubPathsV0Vimp("Vimp",solutionPathId)

print("Verify that all RB-waypoints are valid: ")
pathWaypoints = ps.getWaypoints(solutionPathId)
for i in range(1,len(pathWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

#plotConeWaypoints (rbprmBuilder, solutionPathId, r, "cone_wp_group", "friction_cone_WP2")
#plotCone (q11, rbprmBuilder, r, "cone_11", "friction_cone2"); plotCone (q22, rbprmBuilder, r, "cone_21", "friction_cone2")


rob = rbprmBuilder.client.basic.robot
r(q11)
"""
# Move RB-robot away in viewer
qAway = q11 [::]; qAway[2] = -5; 
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


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
rbprmBuilder.client.rbprm.rbprm.isRbprmValid (q22)
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

q11 = ps.node(0)
q22 = ps.node(1)
plotCone (q11, ps, r, "cone_first", "friction_cone_SG2"); plotCone (q22, ps, r, "cone_second", "friction_cone_SG2")
plotCone (ps.node(2), ps, r, "cone_2", "friction_cone_SG2")
plotCone (ps.node(3), ps, r, "cone_3", "friction_cone_SG2")
plotConeWaypoints (ps, pathId, r, "cone_wp_group", "friction_cone_WP2")

# Plot cones and edges in viewer
plotConesRoadmap (ps, r, 'cone_rm_group', "friction_coneSG2")
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
"""

## Export path to BLENDER ##
"""
import numpy as np
pathId = 0; dt = 0.05; gui.setCaptureTransform ("skeleton_trunk_path.yaml", ["skeleton_trunk_flexible"])
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

r (q22); gui.refresh (); gui.captureTransform ()
"""

""" # Manually add waypoints to roadmap:
pbCl = rbprmBuilder.client.basic.problem
pbCl.addConfigToRoadmap (waypoints[0])
pbCl.addConfigToRoadmap (waypoints[1])
pbCl.addConfigToRoadmap (waypoints[2])
ps.directPath (q11, waypoints[0]); pathIds0 = ps.numberPaths () - 1
ps.directPath (waypoints[0], waypoints[1]); pathId01 = ps.numberPaths () - 1
ps.directPath (waypoints[1], waypoints[2]); pathId12 = ps.numberPaths () - 1
ps.directPath (waypoints[2], q22); pathId2g = ps.numberPaths () - 1
pbCl.addEdgeToRoadmap (q11, waypoints[0], pathIds0, True)
pbCl.addEdgeToRoadmap (waypoints[0], waypoints[1], pathId01, True)
pbCl.addEdgeToRoadmap (waypoints[1], waypoints[2], pathId12, True)
pbCl.addEdgeToRoadmap (waypoints[2], q22, pathId2g, True)

pbCl.saveRoadmap ('/local/mcampana/devel/hpp/data/skeleton_test_path.rdm')
ps.readRoadmap ('/local/mcampana/devel/hpp/data/skeleton_test_path.rdm')
"""



""" #### display
id = r.client.gui.getWindowID("window_hpp_")
r.client.gui.attachCameraToNode("spiderman_trunk/base_link",id)


ps.clearRoadmap()
gui.removeFromGroup("path_1_root",r.sceneName)
ps.solve()

solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
rbprmBuilder.rotateAlongPath (solutionPathId)
orientedpathId = ps.numberPaths () - 1
r(pp.client.problem.configAtParam(orientedpathId,0))
pp(orientedpathId)
"""



