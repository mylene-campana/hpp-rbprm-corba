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
urdfName = 'hrp2_trunk_flexible'
urdfNameRoms =  ['hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
#rbprmBuilder.setJointBounds ("base_joint_xyz", [-140, 120, -80, 65, 1, 170])
rbprmBuilder.setJointBounds ("base_joint_xyz", [-140, 120, -80, 65, 10, 170])
rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])



rbprmBuilder.setContactSize (0.03,0.08)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
ps.selectPathPlanner("BallisticPlanner") # "PRMplanner"#rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(30)#(8)
rbprmBuilder.setMaxLandingVelocity(30)
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectSteeringMethod("SteeringParabola")


rbprmBuilder.setNumberFilterMatch(0)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ("iai_maps", "buildings_reduced", "buildings_reduced")
addLight (r, [-3,0,8,1,0,0,0], "li");

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
# q11[0:7] =  [16,45,100, 1, 0, 0, 0]; r(q11)# toit en X
#q11[0:7] = [0,27,72.3, 1, 0, 0, 0]; r(q11) # first roof of big tower
#q11[0:7] = [-100,45,0.4, 1, 0, 0, 0]; r(q11) # on floor
#q11[0:7] = [-105,20,29.4, 1, 0, 0, 0]; r(q11) # roof of house
#q11[0:7] = [55,60,0.3, 1, 0, 0, 0]; r(q11) # floor, right side
q11[0:7] = [-11.8,38.2,120.9, 1, 0, 0, 0]; r(q11) # highest tower

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
#q22[0:7] = [55,60,0.3, 1, 0, 0, 0]; r(q22) # floor, right side
#q22[0:7] = [-11.6,38.5,120.8, 1, 0, 0, 0]; r(q22) # highest tower
q22[0:7] =  [16,45,100.5, 1, 0, 0, 0]; r(q22) #toit en X
#q22[0:7] =  [-110,20,29.2, 1, 0, 0, 0]; r(q22) #house on left side
#q22[0:7] = [90,40,20.5, 1, 0, 0, 0]; r(q22) #right house

rbprmBuilder.isConfigValid(q22)


ps.clearRoadmap();
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

#r.solveAndDisplay("rm",1,1)

## manually add way point (faster computation for test, work without but it's slow (~ <1minute )
"""
waypoints = [[20.075492263329966,
 45.67270834760806,
 100.0368335278786,
 1,
 0,
 0,
 0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0] ,
 [4,
 24,
 72.36757488910698,
 0.6025437481958323,
 -0.014994289380592305,
 0.36339178566529046,
 -0.7103960957853586,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0],
 [17.90089886471105,
 20.51569231026736,
 37.4,
 0.9780744240181991,
 -0.009709317338437355,
 0.023538837001709934,
 0.20669318660975794,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0]]

pbCl = rbprmBuilder.client.basic.problem
pbCl.addConfigToRoadmap (waypoints[0])
pbCl.addConfigToRoadmap (waypoints[1])
pbCl.addConfigToRoadmap (waypoints[2])
ps.directPath (q11, waypoints[0],True); pathIds0 = ps.numberPaths () - 1
ps.directPath (waypoints[0], waypoints[1],True); pathId01 = ps.numberPaths () - 1
ps.directPath (waypoints[1], waypoints[2],True); pathId12 = ps.numberPaths () - 1
ps.directPath (waypoints[2], q22,True); pathId2g = ps.numberPaths () - 1
pbCl.addEdgeToRoadmap (q11, waypoints[0], pathIds0, True)
pbCl.addEdgeToRoadmap (waypoints[0], waypoints[1], pathId01, True)
pbCl.addEdgeToRoadmap (waypoints[1], waypoints[2], pathId12, True)
pbCl.addEdgeToRoadmap (waypoints[2], q22, pathId2g, True)
##########
"""

t = ps.solve ()

solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])


rbprmBuilder.rotateAlongPath (solutionPathId,True)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)
r(pp.client.problem.configAtParam(orientedpathId,0))


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

q11 = ps.node(0)
q22 = ps.node(1)
plotCone (q11, ps, r, "cone_first", "friction_cone_SG2"); 
plotCone (q22, ps, r, "cone_second", "friction_cone_SG2")
"""



