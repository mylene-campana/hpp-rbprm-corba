#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors


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
base_joint_xyz_limits = [-4, 3.5, -9, 2, 0, 7]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
rbprmBuilder.setAffordanceFilter('SpidermanLFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('SpidermanRFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('SpidermanLHandSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('SpidermanRHandSphere', affordanceType)
rbprmBuilder.setContactSize (0.03,0.08)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "cubeWorld2"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
#r.addLandmark(r.sceneName,2); r.addLandmark("spiderman_trunk/base_link",1) # frames
addLight (r, [-10,-5,5,1,0,0,0], "li");


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [6., 0.001, 0.04]) # default (0.3,0.3,0.05) error, angle and area
afftool.setNeighbouringTriangleMargin ('Support', 0.1)
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(2)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
orientX = 0.707107 #math.sqrt(2)/2.0
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config

#q11[0:7] = [-8.5,0,2.6, 1, 0, 0, 0]; r(q11) # edge middle
#q11[0:7] =  [-2.7,0,3.9, 1, 0, 0, 0]; r(q11) # cube WORKING CONTACT
#q11[0:7] =  [-2.7,-2.3,3.9, 0.9512, 0.1677, 0.0449, 0.2549]
#q11[0:7] =  [-2.7,-2.25,4, 0.9217, 0.1022, 0.2192, 0.3034] #WORKING : SIDE


q11[0:7] =  [-1.5,-7,2.44, orientX, 0, 0, orientX]; r(q11) # back plateform R
#q11[0:7] = [-1.0, -2.7, 4.0,0.8866176165698721, 0.0, 0.0,0.4625031913273241]; r(q11)

rbprmBuilder.isConfigValid(q11)


q22 = q11[::]
#q22[0:7] =  [-7.2,0,2.6, 1, 0, 0, 0]; r(q22) # back plateform
q22[0:7] =  [1.5,-7,2.44, -orientX, 0, 0, orientX]; r(q22) # back plateform L
#q22[0:7] =   [5,0,3.8,  0.9537, 0, 0.3, 0]; r(q22) # front plateform
rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner") # "PRMplanner"#rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
frictionCoef = 1.2; rbprmBuilder.setFrictionCoef(frictionCoef)
rbprmBuilder.setMaxTakeoffVelocity(10)#(10)
rbprmBuilder.setMaxLandingVelocity(10)#15
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectSteeringMethod("SteeringParabola")

ps.clearRoadmap();


## manually add way point (faster computation for test, work without but it's slow (~ <1minute )

q_cube = q11[::]
#q_cube[0:7] =  [-1.2,-2.7,3.9, orientX, 0, 0, orientX]; r(q_cube); rbprmBuilder.isConfigValid(q_cube)
q_cube[0:7] =  [-1.2,-2.8,3.6, orientX, 0, 0, orientX]; r(q_cube); rbprmBuilder.isConfigValid(q_cube) # lower
#q_cube[0:7] = [0,-1.5,5.4, 1, 0, 0, 0]
#cones = rbprmBuilder.getContactCones(q_cube)


"""waypoints = [q11,q_cube,q22]
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()"""

ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()

solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
print("path length= " + str(ps.pathLength(solutionPathId)))
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True) # Pierre
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

#rbprmBuilder.rotateAlongPath (solutionPathId) # not recommended here
#orientedpathIdBis = ps.numberPaths () - 1
#pp(orientedpathIdBis)

V0list = rbprmBuilder.getsubPathsV0Vimp("V0",solutionPathId)
Vimplist = rbprmBuilder.getsubPathsV0Vimp("Vimp",solutionPathId)

print("-- Verify that all RB-waypoints are valid (solution path): ")
pathWaypoints = ps.getWaypoints(solutionPathId)
for i in range(1,len(pathWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

print("number of waypoints= " + str(len(pathWaypoints)))

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriWaypoints = ps.getWaypoints(orientedpathId)
for i in range(1,len(pathOriWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))



"""print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriBisWaypoints = ps.getWaypoints(orientedpathIdBis)
for i in range(1,len(pathOriBisWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriBisWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))"""


#plotConeWaypoints (ps, solutionPathId, r, "cone_planning_wp_group", "friction_cone")
#plotCone (q11, ps, r, "cone_11", "friction_cone"); plotCone (q22, ps, r, "cone_21", "friction_cone")


# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -10; rbprmBuilder.setCurrentConfig (qAway); r(qAway)


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

""" # Pierre's roadmap manual initialization
ps.setInitialConfig (q11); ps.addGoalConfig (q22)
waypoints = [q_cube]
pbCl = ps.client.problem
ps.client.problem.prepareSolveStepByStep()
q11 = ps.node(0)
q22 = ps.node(1)
pbCl.addConfigToRoadmap (waypoints[0])
ps.directPath (q11, waypoints[0],False)
pathIds0 = ps.numberPaths () - 1
ps.directPath (waypoints[0], q22,False)
pathId2g = ps.numberPaths () - 1
pbCl.addEdgeToRoadmap (q11, waypoints[0], pathIds0, True)
pbCl.addEdgeToRoadmap (waypoints[0], q22, pathId2g, True)
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

"""
sphereColor = [1,0,0,1]; sphereSize = 0.1; sphereName = "pointsPlane"
p1 = [-1.01465, -9.03957,  1.87606]
p2 = [-1.01465, -9.03957, -0.12394]
p3 = [-1.01465, -6.03957, -0.12394]
plotSphere (p1, r, sphereName+"1", sphereColor, sphereSize)
plotSphere (p2, r, sphereName+"2", sphereColor, sphereSize)
plotSphere (p3, r, sphereName+"3", sphereColor, sphereSize)
"""
