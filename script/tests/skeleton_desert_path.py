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
urdfName = 'armlessSkeleton_trunk'
urdfNameRoms = ['LFootSphere','RFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-9, 7, -7, 7, -1.7, 2.5]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
filterRange = 0.3
rbprmBuilder.setNormalFilter('LFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RFootSphere', [0,0,1], filterRange)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
rbprmBuilder.setNumberFilterMatch(2)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())


pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel (packageName,"desert","desert")
addLight (r, [-3,-4,3,1,0,0,0], "li")

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0.128453,0.20242,0.970838,0] # set normal for init config
q11[0:7] = [4.42, -0.08, 0.09, 0, 0, 0, -1]; r(q11)

rbprmBuilder.isConfigValid(q11)


q22 = q11[::]
q22[(len(q22)-4):]=[-0.216356,-0.357682,0.908435,0] # set normal for goal config
q22[0:7] = [-6.4, -5.1, -1.7, 0, 0, 0, -1]; r(q22)

rbprmBuilder.isConfigValid(q22)


ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(0.5)
rbprmBuilder.setMaxTakeoffVelocity(8)
rbprmBuilder.setMaxLandingVelocity(12)
ps.clearRoadmap();

waypoints = [q11,[4.969833181804385, 2.5093947489984854, -1.1286145993154524, 0.9428321883031424, -0.028410527192610304, -0.03980561408445277, 0.32966015793769415, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0937916662308081, 0.027328068995934207, 0.9952167100639951, 0.6749808185118427], [2.76202236321887, 6.888654696809633, -1.1706977196008705, 0.9665782974416167, 0.13356912600568827, 0.09057009829688505, 0.19920527299103988, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.22830153124012148, -0.22212595449857325, 0.9479126917451383, 0.42504688508392435], [1.3846117358435879, 1.9425736379138878, -1.654902127711619, 0.9694653021001891, -0.1828967149611175, -0.09679911857013053, 0.13158932450722063, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2358212838078935, 0.32914857679474824, 0.9143574446009551, 0.29780472819238824], [-1.0452970899258986, 3.7682795116264307, -1.3814288682033222, 0.5008201318423201, 0.013288436381679805, 0.004673092784616044, -0.8654367540171827, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.018319844607244007, -0.02139876542278078, 0.9996031593247129, -2.0923134289697916], [-4.514122576405046, 1.8085022744132144, -1.1250276293809998, -0.315880646125694, -0.016750532375229183, 0.06204160847278101, 0.9466201328339001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.07090826910627478, 0.10687733332819205, 0.9917405169665131, -2.4979250099490957], [-6.125009386288834, 1.7715804407208278, -1.1305618628875704, 0.7318765456292236, 0.021368813037807664, 0.054804681811020696, -0.6788936165842061, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05120622088561918, -0.10569176342147577, 0.993079641361896, -1.4980978526349344], [-7.739399951287547, -2.7184020873906904, -1.1682173693407376, -0.33406598881529387, 0.030675041612581885, -0.0053180425810179545, 0.9420353896547798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.061347103863149524, 0.01047540758649499, 0.9980615204903457, -2.4591910470660308], [-8.42813632698783, -4.667755225607967, -1.206383413641182, -0.13872135513808956, -0.04251131541753735, -0.042983120260813336, 0.988484509267984, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.07211820013639479, -0.09677075164248071, 0.9926904788682303, -2.866279777392365],q22] # limVelocity = 5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()

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


plotConeWaypoints (ps, solutionPathId, r, "cone_planning_wp_group", "friction_cone")
plotCone (q11, ps, r, "cone_11", "friction_cone"); plotCone (q22, ps, r, "cone_21", "friction_cone")



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


