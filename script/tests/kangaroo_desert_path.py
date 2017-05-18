#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a kangaroo-robot and a desert environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton.urdf -o skeleton_blend.py
#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton_trunk_flexible.urdf -o skeleton_trunk_blend.py
#blender/urdf_to_blender.py -p armlessSkeleton/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/armlessSkeleton.urdf -o armlessSkeleton_blend.py

from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *


rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'kangaroo_trunk'
urdfNameRoms = ['KangarooLFootSphere','KangarooRFootSphere','KangarooLHandSphere','KangarooRHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-9, 7, -7, 7, -1.7, 2.5]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
rbprmBuilder.setAffordanceFilter('KangarooLFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('KangarooRFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('KangarooLHandSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('KangarooRHandSphere', affordanceType)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())
len(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "desert_for_kangaroo"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [-4,-4,3,1,0,0,0], "li"); addLight (r, [4,4,3,1,0,0,0], "li2")
addLight (r, [-4,4,3,1,0,0,0], "li3"); addLight (r, [4,-4,3,1,0,0,0], "li4")

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [1., 0.0001, 0.05]) # error, angle and area   # default (0.3,0.3,0.05) 
afftool.setNeighbouringTriangleMargin ('Support', 0.01)
print("Load affordances")
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r); print("Affordances OK")
SupportColour = [0.0, 0.95, 0.80];
#afftool.visualiseAffordances('Support', r, SupportColour)

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(2)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0.128453,0.20242,0.970838,0] # set normal for init config
#q11[0:7] = [4.5, 2.5, 0.05, 0, 0.17365, 0, -0.98481]; r(q11)  # orientation -20deg y  -180deg z
q11[0:7] = [4.6, 2., 0.05, 0, 0.17365, 0, -0.98481]; r(q11)  # orientation -20deg y  -180deg z

rbprmBuilder.isConfigValid(q11)
cones1 = rbprmBuilder.getContactCones (q11); cones1


q22 = q11[::]
q22[(len(q22)-4):]=[-0.216356,-0.357682,0.908435,0] # set normal for goal config
#q22[0:7] = [-7.1, -3.8, 0.1, 0, 0.17365, 0, -0.98481]; r(q22)
q22[0:7] = [-7.1, -0.9, 0.1, 0, 0.17365, 0, -0.98481]; r(q22)

rbprmBuilder.isConfigValid(q22)
cones2 = rbprmBuilder.getContactCones (q22)


"""
rbprmBuilder.isConfigValid(q11)
cones1 = rbprmBuilder.getContactCones (q11); cones1
rbprmBuilder.isConfigValid(q22)
cones2 = rbprmBuilder.getContactCones (q22); cones2

"""

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.selectSteeringMethod("SteeringParabola")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
frictionCoef = 1.2; rbprmBuilder.setFrictionCoef(frictionCoef)
rbprmBuilder.setMaxTakeoffVelocity(7)
rbprmBuilder.setMaxLandingVelocity(8)
ps.clearRoadmap();

"""waypoints = [q11, [1.4682346589202155, 1.3721742911882595, -0.3177500175158776, -0.47980079307050877, -0.42470043369655597, -0.12553328442064984, 0.7574048686729129, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5228784135749911, -0.5977022514113965, 0.6077418722410429, -1.8515503036956853], [-2.2428656093342907, 3.705981508906953, -0.43171314034458386, -0.19511109369718505, -0.39497095366864815, -0.20585489417778283, 0.8738154092355094, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6099342639403275, -0.5138847866960039, 0.6032434166022648, -2.8579343885809663], [-4.980684904172251, 2.518180830037141, 0.7837057291683014, 0.9527709249186042, 0.12448725029738976, 0.27422176447773133, 0.03915243325595655, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5322890058654917, -0.21574276655074562, 0.8186106968002294, 0.16720644997193632], q22] # 7 - 8
#ps.directPath (waypoints[0] , waypoints[1], False)
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); t = ps.solve (); t; ps.resetGoalConfigs()
"""

ps.setInitialConfig (q11); ps.addGoalConfig (q22)

print("start solve")
t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

rbprmBuilder.rotateAlongPath (solutionPathId)
orientedpathIdBis = ps.numberPaths () - 1
#pp(orientedpathIdBis)

V0list = rbprmBuilder.getsubPathsV0Vimp("V0",solutionPathId)
Vimplist = rbprmBuilder.getsubPathsV0Vimp("Vimp",solutionPathId)

print("-- Verify that all RB-waypoints are valid (solution path): ")
pathWaypoints = ps.getWaypoints(solutionPathId)
for i in range(1,len(pathWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriWaypoints = ps.getWaypoints(orientedpathId)
for i in range(1,len(pathOriWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriBisWaypoints = ps.getWaypoints(orientedpathIdBis)
for i in range(1,len(pathOriBisWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriBisWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -8; rbprmBuilder.setCurrentConfig (qAway); r(qAway)


#plotConeWaypoints (ps, solutionPathId, r, "cone_planning_wp_group", "friction_cone2") # gui.writeNodeFile('cone_planning_wp_group','conesWP_COM_kangarooDesert.dae')
#plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")
# r.client.gui.writeNodeFile('cone_11','Cone_start_mu2_kangarooDesert.dae'); r.client.gui.writeNodeFile('cone_21','Cone_goal_mu1_kangarooDesert.dae')

"""
q_goal = ps.configAtParam(orientedpathIdBis,ps.pathLength(orientedpathIdBis))
pathToYamlFile (ps, r, "kangarooTrunkDesert_forcedOrientation_frames.yaml", "kangaroo_trunk", orientedpathIdBis, q_goal, 0.015)

pathSamples = plotSampleSubPath (psf.client.problem, rr, tp.solutionPathId, 70, "sampledPath", [1,0,0,1])
writePathSamples (pathSamples, 'kangaroo_desert_path.txt')
"""


"""
# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("kangaroo_desert_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()
"""


"""
for i in range(0,len(waypoints)):
    rbprmBuilder.isConfigValid(waypoints[i])

pathSamples = plotSampleSubPath (ps.client.problem, r, solutionPathId, 70, "sampledPath", [1,0,0,1])
writePathSamples (pathSamples, 'fullSkeleton_newDesert_path.txt')

"""

#r.startCapture("kangarooDesert_contactConesImage","png") ; r.stopCapture()

"""
p1= [-3.88244,   4.18888, -0.891968]
p2= [-3.61904,   3.97947, -0.971283]
p3= [-3.54959,   4.29865, -0.928852]

p4= [-7.04535, -5.65455, -2.70557]
p5= [-6.69796, -5.55828, -2.54195]
p6= [-7.17331, -5.29874, -2.61645]

 
sphereColor = [1,0,0,1]; sphereSize = 0.1; sphereName = "pointsPlane"
plotSphere (p1, r, sphereName+"11", sphereColor, sphereSize)
plotSphere (p2, r, sphereName+"22", sphereColor, sphereSize)
plotSphere (p3, r, sphereName+"33", sphereColor, sphereSize)
plotSphere (p4, r, sphereName+"eegfh", sphereColor, sphereSize)
plotSphere (p5, r, sphereName+"efghfghf", sphereColor, sphereSize)
plotSphere (p6, r, sphereName+"egfhfghfgg", sphereColor, sphereSize)


INFO:/local/mcampana/devel/hpp/src/hpp-rbprm/src/utils/algorithms.cc:498: p1 of plane=  
INFO:/local/mcampana/devel/hpp/src/hpp-rbprm/src/utils/algorithms.cc:499: p2 of plane=  
INFO:/local/mcampana/devel/hpp/src/hpp-rbprm/src/utils/algorithms.cc:500: p3 of plane=  

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

# plot contact-cones of waypoints
"""coneGroupName = "ConesWP_contactCones"; r.client.gui.createGroup (coneGroupName)
for i in range(0,len(waypoints)):
    q = waypoints[i]; conesWP = rbprmBuilder.getContactCones (q); coneName = "ConesWP"+str(i); contactConeName = coneName+"_contactCones_1"; plotContactCones (conesWP, ps, r, coneName, "friction_cone2"); r.client.gui.addToGroup (contactConeName, coneGroupName)


#r.client.gui.writeNodeFile(coneGroupName,'ConesWP_contactCones2bis_kangarooDesert.dae')
"""
