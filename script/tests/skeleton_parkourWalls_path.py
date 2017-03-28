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
urdfName = 'skeleton_trunk'
urdfNameRoms = ['LFootSphere','RFootSphere','LHandSphere','RHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-3.7, 1.3, -2.3, 2.3, 0.005, 6]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
#rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
#affordanceType = ['Support','Lean']
rbprmBuilder.setAffordanceFilter('LFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('LHandSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RHandSphere', affordanceType)
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())
q_0 = rbprmBuilder.getCurrentConfig ()

rbprmBuilder.getLinkPosition("Thorax")

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "parkour_walls"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [0,-1,7,1,0,0,0], "li"); addLight (r, [0,1,7,1,0,0,0], "li4");
gui.setCaptureTransform ("skeletonTrunk_frames.yaml", [urdfName]) # FOR BLENDER EXPORT ONLY


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [6., 0.005, 0.05]) # default (0.3,0.3,0.05) error, angle and area 
afftool.setNeighbouringTriangleMargin ('Support', 0.02)
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])


ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(2)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [-2.9, 0, 5.35, 1, 0, 0, 0]; r(q11)
rbprmBuilder.isConfigValid(q11)
#cones11 = rbprmBuilder.getContactCones (q11); plotContactCones (cones11, ps, r, "qCones11test", "friction_cone2"); qAway = q11 [::]; qAway[0] = -6.5; qAway[1] = -2; r(qAway)


qtest = q11 [::]; qtest[0:7] = [0.8, 0, 5.5, 1, 0, 0, 0]; r(qtest); rbprmBuilder.isConfigValid(qtest) # test long wall
qtest = q11 [::]; qtest[0:7] = [0.65, 0, 5.5, 1, 0, 0, 0]; r(qtest); rbprmBuilder.isConfigValid(qtest)
qtest = q11 [::]; qtest[0:7] = [-0.5, -1, 1.4, 1, 0, 0, 0]; r(qtest); rbprmBuilder.isConfigValid(qtest)

q22 = q11[::]
q22[0:7] = [0, 1, 0.6, 1, 0, 0, 0]; r(q22)
rbprmBuilder.isConfigValid(q22)

## TRY to give manual waypoints
qwp1 = q11 [::]; qwp1[0:7] = [-1.55, 0, 5.25, 1, 0, 0, 0]; qwp1[(len(qwp1)-4):]=[0.051,-0.0006,0.228,0]; r(qwp1); rbprmBuilder.isConfigValid(qwp1)
qwp2 = q11 [::]; qwp2[0:7] = [0.7, 0, 3.8, 1, 0, 0, 0]; r(qwp2); rbprmBuilder.isConfigValid(qwp2)
qwp3 = q11 [::]; qwp3[0:7] = [-0.8, 0, 2.6, 0, 0, 0, 1]; r(qwp3); rbprmBuilder.isConfigValid(qwp3)
qwp4 = q11 [::]; qwp3[0:7] = [0.7, 0, 1.6, 01, 0, 0, 0]; r(qwp3); rbprmBuilder.isConfigValid(qwp3)


qt = q11 [::]; 
qt[0:7] = [-1.887, 0, 5.444, 0, -0.27564, 0, 0.96126]; r(qt) # rot 180 z and -15 y
rbprmBuilder.isConfigValid(qt)

q = qt; conesWP = rbprmBuilder.getContactCones (q); plotContactCones (conesWP, ps, r, "contactCones_test", "friction_cone06"); qAway = q11 [::]; qAway[0] = -6.5; r(qAway); conesWP
#q = qt; r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform () # BLENDER EXPORT



#conesWP = rbprmBuilder.getContactCones (qwp1); plotContactCones (conesWP, ps, r, "qConesWPtest", "friction_cone"); qAway = q11 [::]; qAway[0] = -6.5; r(qAway)
#intersection3D = [[-1.27977,0.856698,4.59634],[-1.27932,0.792132,4.50904],[-1.27927,0.783744,4.49959],[-1.27923,0.774587,4.49188],[-1.27847,0.578437,4.34569],[-1.2782,0.470018,4.29326],[-1.27808,0.418282,4.26993],[-1.27789,0.272664,4.23112],[-1.27778,0.187655,4.2112],[-1.27778,0.15676,4.20937],[-1.27778,0.15676,4.20937],[-1.27781,0.0295551,4.2153],[-1.2779,-0.0797504,4.23182],[-1.27812,-0.275248,4.27282],[-1.27818,-0.299129,4.28436],[-1.27843,-0.358018,4.33299],[-1.27921,-0.521692,4.48153],[-1.27977,-0.563851,4.58897]]

#for i in range(0,len(intersection3D)-2):
#    gui.addLine ("line3Dinters"+str(i),intersection3D[i],intersection3D[i+1],[1,0,0,1]); r.client.gui.addToGroup ("line3Dinters"+str(i), r.sceneName)


## test re-orientation along normal (OK)
"""
qq = rbprmBuilder.setOrientation([0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0.339103,]); r(qq); rbprmBuilder.isConfigValid(qq)
qt = [-1.64681,1.59857,1.94602,0.00202022,0.999994,-5.46547e-06,0.00286334,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.999984,-6.38274e-07,-0.00572666,2.52715,]; r(qt); rbprmBuilder.isConfigValid(qt)
qt = [1.01616,1.46704,1.79813,0.172615,0.98489,-0.00664272,0.0123005,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.999609,0.0088382,-0.0265225,1.63519,]; r(qt); rbprmBuilder.isConfigValid(qt)
qt = []; r(qt); rbprmBuilder.isConfigValid(qt)

ps.setInitialConfig (q11); ps.addGoalConfig (qwp1); ps.solve (); pp.displayPath(ps.numberPaths ()-1, [0.0, 0.0, 0.8, 1.0]); ps.resetGoalConfigs()
ps.setInitialConfig (qwp1); ps.addGoalConfig (qwp2); ps.solve (); pp.displayPath(ps.numberPaths ()-1, [0.0, 0.0, 0.8, 1.0]); ps.resetGoalConfigs()
ps.setInitialConfig (qwp2); ps.addGoalConfig (qwp3); ps.solve (); pp.displayPath(ps.numberPaths ()-1, [0.0, 0.0, 0.8, 1.0]); ps.resetGoalConfigs()
ps.setInitialConfig (qwp3); ps.addGoalConfig (qwp4); ps.solve (); pp.displayPath(ps.numberPaths ()-1, [0.0, 0.0, 0.8, 1.0]); ps.resetGoalConfigs()
ps.setInitialConfig (qwp4); ps.addGoalConfig (q22); ps.solve (); pp.displayPath(ps.numberPaths ()-1, [0.0, 0.0, 0.8, 1.0]); ps.resetGoalConfigs()
"""

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setInteriorPoint([0,0,6])
frictionCoef = 0.6; rbprmBuilder.setFrictionCoef(frictionCoef)
rbprmBuilder.setMaxTakeoffVelocity(9.2)    #direct path will land with a speed norm of 
rbprmBuilder.setMaxLandingVelocity(9.2)
ps.clearRoadmap();

#ps.directPath ( q11, qwp1, False)

# 9.4 lim (ok with 9.3)
#waypoints = [q11, [0.8970849555733145, -0.5295706331948731, 1.9004868435490172, 0.9550302922058136, 0.23028287060758557, 0.18559178967829196, -0.02103872805130798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9302261190519835, -0.04529197497967405, 0.3641812796340932, -0.9144760180568767], [-0.2952067650662399, 0.8096100928352825, 0.5260294719785452, 0.9974445668009648, 0.0, 0.0, 0.07144463702221228, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.14301111325920662], q22]
#for i in range(0,len(waypoints)-1):
#    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()

"""
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

print('start solve')
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

"""

#qAway = q11 [::]; qAway[0] = -8; rbprmBuilder.setCurrentConfig (qAway); r(qAway)

#r.startCapture ("problem_ROM_obst_skel1","png"); r.stopCapture()

""" # Not for contact-cones
plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")
"""

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
# Move RB-robot away in viewer
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
cones = rbprmBuilder.getContactCones (q)
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

#affordanceType = ['Support','Lean']
#afftool.setAffordanceConfig('Lean', [0.5, 0.3, 0.05]) # default (0.1,0.3,0.05)
#afftool.setNeighbouringTriangleMargin ('Lean',0.5)
#afftool.visualiseAffordances('Lean', r, [0.6, 0.1, 0.1])

"""

"""
sphereColor = [1,0,0,1]; sphereSize = 0.1; sphereName = "pointsPlane"
p1 = [-1.25586 , 1.21305 , 2.14146]
p2 = [-1.25586 , 2.42703  ,2.14146]
p3 = [-1.25586  ,  2.42703 ,-0.0354225]
plotSphere (p1, r, sphereName+"1", sphereColor, sphereSize)
plotSphere (p2, r, sphereName+"2", sphereColor, sphereSize)
plotSphere (p3, r, sphereName+"3", sphereColor, sphereSize)

plotFrame (r, "framy", [0,0,0], 1)
"""


"""
q_goal = ps.configAtParam(orientedpathIdBis,ps.pathLength(orientedpathIdBis))
pathToYamlFile (ps, r, "kangarooTrunkDesert_forcedOrientation_frames.yaml", "kangaroo_trunk", orientedpathIdBis, q_goal, 0.015)

pathSamples = plotSampleSubPath (psf.client.problem, rr, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])
writePathSamples (pathSamples, 'kangaroo_desert_path.txt')
"""

# plot first of contact-cones of waypoints
"""coneGroupName = "ConesWP_contactCones"; r.client.gui.createGroup (coneGroupName)
for i in range(0,len(waypoints)):
    q = waypoints[i]; conesWP = rbprmBuilder.getContactCones (q); coneName = "ConesWP"+str(i); contactConeName = coneName+"_contactCones_1"; plotContactCones (conesWP, ps, r, coneName, "friction_cone2"); r.client.gui.addToGroup (contactConeName, coneGroupName)


#r.client.gui.writeNodeFile(coneGroupName,'ConesWP_contactCones2bis_kangarooDesert.dae')
"""

"""
# plot all contact-cones of ONE config
coneName = "contactCones_test"
q = qt; conesWP = rbprmBuilder.getContactCones (q); plotContactCones (conesWP, ps, r, coneName, "friction_cone06")
coneGroupName = "contactConesGroup"; r.client.gui.createGroup (coneGroupName)
for i in range(0,len(conesWP[0])):
    contactConeName = coneName+"_contactCones_"+str(i); r.client.gui.addToGroup (contactConeName, coneGroupName)

#r.client.gui.writeNodeFile(coneGroupName,'contactCones_skeletonParkour.dae')
"""
