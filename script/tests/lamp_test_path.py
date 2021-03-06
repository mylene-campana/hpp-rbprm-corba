#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a lamp-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 6 end-effectors


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'lamp_trunk'
urdfNameRoms = ['LampFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-6, -1, -2, 2, 0.001, 2.4]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
#rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
rbprmBuilder.setAffordanceFilter('LampFootSphere', ['Support'])
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "groundcrouch"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [-3,0,8,1,0,0,0], "li");

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(1)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [-5.0, 0, 0.28, 1, 0, 0, 0]; r(q11)  # z = 0.59

rbprmBuilder.isConfigValid(q11)

#V = rbprmBuilder.computeConfigGIWC (q11,0.02,0.02)
#plotGIWC (q11, V, r, 0, [0,1,0.1,1]) # attente reponse Steve

q22 = q11[::]
q22[0:7] = [-2.5, 0, 0.29, 1, 0, 0, 0]; r(q22)

rbprmBuilder.isConfigValid(q22)


ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
frictionCoef = 1.; rbprmBuilder.setFrictionCoef(frictionCoef)
rbprmBuilder.setMaxTakeoffVelocity(8)#(4.5)
rbprmBuilder.setMaxLandingVelocity(8)
ps.clearRoadmap();
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])

rbprmBuilder.rotateAlongPath (solutionPathId)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

V0list = rbprmBuilder.getsubPathsV0Vimp("V0",solutionPathId)
Vimplist = rbprmBuilder.getsubPathsV0Vimp("Vimp",solutionPathId)

print("Verify that all RB-waypoints are valid: ")
pathWaypoints = ps.getWaypoints(solutionPathId)
for i in range(1,len(pathWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriWaypoints = ps.getWaypoints(orientedpathId)
for i in range(1,len(pathOriWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

""" # Not for contact-cones
plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")
"""


# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -8
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


"""
# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("lamp_test_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

rob = rbprmBuilder.client.basic.robot
"""

"""
## Export for Blender ##
# First display in Viewer, then export
# Don't change exported names, because harcoded in fullAnimationSkinning.py
pathId = ps.numberPaths()-1 # path to export
plotCone (q_init_test, ps, r, "cone_start", "friction_cone2")
plotCone (q_goal_test, ps, r, "cone_goal", "friction_cone2")
plotConeWaypoints (ps, pathId, r, "cone_wp_group", "friction_cone2")
pathSamples = plotSampleSubPath (ps.client.problem, r, pathId, 70, "sampledPath", [1,0,0,1])

gui.writeNodeFile('cone_wp_group','cones_path.dae')
gui.writeNodeFile('cone_11','cone_start2.dae')
gui.writeNodeFile('cone_21','cone_goal2.dae')
writePathSamples (pathSamples, 'lamp_path.txt')
pathToYamlFile (psf, rr, "lampTrunkTest_frames.yaml ", "lamp_trunk", pathId, q_goal_test, 0.01)
"""

"""
q = q11 # ideally, take waypoints
conesDirAndPositions = rbprmBuilder.getContactCones (q)
cones = conesDirAndPositions[0]
positions = conesDirAndPositions[1]
theta = math.atan2(q22 [1] - q11 [1] , q22 [0] - q11 [0])
origin = q[0:3]; mu = 1.2; coneURDFName = "friction_cone2"

t = rbprmBuilder.convexConePlaneIntersection (len(cones), cones, theta, mu)

# Display:
black = [0.1,0.1,0.1,1]; red = [1,0,0,1]; blue = [0,0,1,1]; green = [0,1,0,1]; planeThetaColor = [0.7,0.2,0.2,0.2]
logID = getNewestLogID ()
CC2D_dir = t [1:4]; phi_CC = t [0]

#plotThetaPlaneBis (origin, theta, 1, r, "thetaPlane", planeThetaColor)
plotConvexConeInters (ps, r, origin, CC2D_dir, cones, "CC_center", black, 0.02, "CC_dir", "contactCones", coneURDFName)

lineParsedBorders = "M_border = ("
lineEndBorders = "END of border points ---"
pointsIntersBorders = plotLogConvexConeInters (r, logID, lineParsedBorders, lineEndBorders, "IntersPointBorders_", 0.021, blue)
plotStraightLines (origin, pointsIntersBorders, r, "CC_borderLine", blue)
"""
