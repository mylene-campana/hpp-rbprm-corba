#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a desert environment.
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
urdfName = 'skeleton_trunk_flexible'
urdfNameRoms = ['LFootSphere','RFootSphere','LHandSphere','RHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-9, 7, -7, 7, -1.7, 2.5]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']#,'Lean']
rbprmBuilder.setAffordanceFilter('LFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('LHandSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RHandSphere', affordanceType)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())


pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "desert"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [-4,-4,3,1,0,0,0], "li"); addLight (r, [4,4,3,1,0,0,0], "li2")
addLight (r, [-4,4,3,1,0,0,0], "li3"); addLight (r, [4,-4,3,1,0,0,0], "li4")

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.6, 0.6, 0.03]) # error, angle and area   # default (0.3,0.3,0.05)   # all ground [0.6, 0.6, 0.06]
#afftool.setAffordanceConfig('Lean', [1.2, 1., 0.06]) # error, angle and area   # default (0.1,0.3,0.05)
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
SupportColour = [0.0, 0.95, 0.80]; LeanColour = [0.9, 0.5, 0]
afftool.visualiseAffordances('Support', r, SupportColour)
#afftool.visualiseAffordances('Lean', r, LeanColour)

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(2)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0.128453,0.20242,0.970838,0] # set normal for init config
q11[0:7] = [4.42, -0.08, 0.09, 0, 0, 0, -1]; r(q11)

rbprmBuilder.isConfigValid(q11)
cones = rbprmBuilder.getContactCones (q11)

"""
q22 = q11[::]
q22[(len(q22)-4):]=[-0.216356,-0.357682,0.908435,0] # set normal for goal config
q22[0:7] = [-6.4, -5.1, -1.7, 0, 0, 0, -1]; r(q22)

rbprmBuilder.isConfigValid(q22)
cones = rbprmBuilder.getContactCones (q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(0.5)
rbprmBuilder.setMaxTakeoffVelocity(8)
rbprmBuilder.setMaxLandingVelocity(12)
ps.clearRoadmap();

#waypoints = [q11,[4.969833181804385, 2.5093947489984854, -1.1286145993154524, 0.9428321883031424, -0.028410527192610304, -0.03980561408445277, 0.32966015793769415, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0937916662308081, 0.027328068995934207, 0.9952167100639951, 0.6749808185118427], [2.76202236321887, 6.888654696809633, -1.1706977196008705, 0.9665782974416167, 0.13356912600568827, 0.09057009829688505, 0.19920527299103988, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.22830153124012148, -0.22212595449857325, 0.9479126917451383, 0.42504688508392435], [1.3846117358435879, 1.9425736379138878, -1.654902127711619, 0.9694653021001891, -0.1828967149611175, -0.09679911857013053, 0.13158932450722063, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2358212838078935, 0.32914857679474824, 0.9143574446009551, 0.29780472819238824], [-1.0452970899258986, 3.7682795116264307, -1.3814288682033222, 0.5008201318423201, 0.013288436381679805, 0.004673092784616044, -0.8654367540171827, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.018319844607244007, -0.02139876542278078, 0.9996031593247129, -2.0923134289697916], [-4.514122576405046, 1.8085022744132144, -1.1250276293809998, -0.315880646125694, -0.016750532375229183, 0.06204160847278101, 0.9466201328339001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.07090826910627478, 0.10687733332819205, 0.9917405169665131, -2.4979250099490957], [-6.125009386288834, 1.7715804407208278, -1.1305618628875704, 0.7318765456292236, 0.021368813037807664, 0.054804681811020696, -0.6788936165842061, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05120622088561918, -0.10569176342147577, 0.993079641361896, -1.4980978526349344], [-7.739399951287547, -2.7184020873906904, -1.1682173693407376, -0.33406598881529387, 0.030675041612581885, -0.0053180425810179545, 0.9420353896547798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.061347103863149524, 0.01047540758649499, 0.9980615204903457, -2.4591910470660308], [-8.42813632698783, -4.667755225607967, -1.206383413641182, -0.13872135513808956, -0.04251131541753735, -0.042983120260813336, 0.988484509267984, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.07211820013639479, -0.09677075164248071, 0.9926904788682303, -2.866279777392365],q22] # limVelocity = 5
#waypoints = [q11,[6.266595627234895, -2.6361967252592584, -0.952682877362785, 0.5583071627002438, 0.05891810411379777, -0.1641267998219023, -0.8111005872673626, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2788433535580363, 0.20045788836481168, 0.9391821011757139, -1.9509605348171244], [3.975403732580819, -3.1104586286782396, -0.29643372603812945, 0.996432294923839, -0.05072676164893127, -0.04899774125583718, -0.046354057397924565, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.09294308108910582, 0.10563405526888181, 0.9900520340088771, -0.0879812582306644], [0.7700079870939243, -6.685431055123854, -1.082765132910233, 0.9970872027552659, 0.074759570763644, 0.005267432286566797, -0.014155240655721425, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00838769921767569, -0.14923274612712045, 0.9887665214726843, -0.02744606796024794], \
#[-0.8468649545907455, -4.447843973056711, -0.22458508564020466, 0.7083188238264719, 0.10527874034635032, -0.2327924980207103, 0.6580338011906733, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.19122867739796204, -0.4555124917945994, 0.8694480793927266, 1.5418201321037304], [-5.594392450859844, -6.946577816425062, -1.4412860226674031, 0.9627892205717546, -0.09609649436322329, 0.02185603335389082, -0.25164398329379106, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.09044971587942732, 0.17404145923094144, 0.9805755551542651, -0.5106798160105228],q22] # limVelocity 8 12, desert2.stl
waypoints = [q11,[6.266595627234895, -2.6361967252592584, -0.952682877362785, 0.5583071627002438, 0.05891810411379777, -0.1641267998219023, -0.8111005872673626, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2788433535580363, 0.20045788836481168, 0.9391821011757139, -1.9509605348171244]]
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()

"""

"""
ps.setInitialConfig (q11); ps.addGoalConfig (q22)


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


#plotConeWaypoints (ps, solutionPathId, r, "cone_planning_wp_group", "friction_cone")
#plotCone (q11, ps, r, "cone_11", "friction_cone"); plotCone (q22, ps, r, "cone_21", "friction_cone")



# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("skeleton_desert_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()


# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -8
rbprmBuilder.setCurrentConfig (qAway); r(qAway)
"""

"""
for i in range(0,len(waypoints)):
    rbprmBuilder.isConfigValid(waypoints[i])

pathSamples = plotSampleSubPath (ps.client.problem, r, solutionPathId, 70, "sampledPath", [1,0,0,1])
writePathSamples (pathSamples, 'fullSkeleton_newDesert_path.txt')

"""

#r.startCapture("skeletonDesert_pbIntersectionRomObst","png") ; r.stopCapture()

"""
p1= [4.56402, -0.298914, -0.414262]
p2= [4.94477, 0.0843232, -0.702477]
p3= [4.37058,   0.16654, -0.490454]

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
"""
