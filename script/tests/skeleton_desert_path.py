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
urdfName = 'skeleton_trunk'
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
afftool.setAffordanceConfig('Support', [1., 0.0001, 0.05]) # error, angle and area   # default (0.3,0.3,0.05) 
afftool.setNeighbouringTriangleMargin ('Support', 0.01)
print("Load affordances")
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
SupportColour = [0.0, 0.95, 0.80]
#afftool.visualiseAffordances('Support', r, SupportColour)

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(2)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0.128453,0.20242,0.970838,0] # set normal for init config
q11[0:7] = [4.42, -0.08, 0.09, 0, 0, 0, -1]; r(q11)

rbprmBuilder.isConfigValid(q11)
cones = rbprmBuilder.getContactCones (q11)


q22 = q11[::]
q22[(len(q22)-4):]=[-0.216356,-0.357682,0.908435,0] # set normal for goal config
q22[0:7] = [-6.4, -5.1, -1.7, 0, 0, 0, -1]; r(q22)

rbprmBuilder.isConfigValid(q22)
cones = rbprmBuilder.getContactCones (q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
frictionCoef = 0.6; rbprmBuilder.setFrictionCoef(frictionCoef)
rbprmBuilder.setMaxTakeoffVelocity(8)
rbprmBuilder.setMaxLandingVelocity(12)
ps.clearRoadmap();

#waypoints = [q11,[6.266595627234895, -2.6361967252592584, -0.952682877362785, 0.5583071627002438, 0.05891810411379777, -0.1641267998219023, -0.8111005872673626, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2788433535580363, 0.20045788836481168, 0.9391821011757139, -1.9509605348171244]]
#waypoints = [q11,[3.407825079827114, 5.9097890518965945, -1.2417976054250657, 0.9449653880972131, 0.01951093894793174, -0.06413165885451191, -0.3202294004182346, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.13370034835932976, 0.0041993613354675335, 0.9910129071878772, -0.6577488606778932], [-0.2724756851907042, 4.477792184044092, -1.6629338197042949, 0.6798870706939689, -0.025452630501126266, 0.26498503695351566, 0.6832925178095332, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3255366171441372, 0.3967344149493732, 0.8582701875826342, 1.651668792821093], [-4.405118433433006, 3.1796110126725106, -1.0001043692710074, 0.9914449412160986, 0.06245169231208634, 0.06717294210724219, -0.0928682427583311, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.12159698943303937, -0.136311295015218, 0.9831751639520271, -0.1785769310091204], [-6.316584637438448, 1.1401280618656933, -1.1506063531346218, 0.20300485870525808, 0.03416087994784434, 0.03833028277849672, 0.977830686287441, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08236958064632964, 0.0610914042156594, 0.9947276474066189, 2.729908534770657], [-4.316439645610631, -1.0812763453270087, 0.18805197869769305, -0.38550981547323576, -0.16081418567277803, -0.12283474828609127, 0.9002403037376663, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1948348204198295, -0.3451524763016038, 0.9181008445997878, -2.3534725727437187], [-2.4700315851250694, -3.227626652947394, -0.308306735045228, 0.5891450356624319, -0.13964929648566865, -0.200139968044982, 0.7702922783829027, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4509644867517082, -0.14378516444513542, 0.8808841343640229, 1.840939562830797], [-6.741090568865635, -5.0734023845363065, -1.7, 0.6244243262187651, 0.026311517441432997, -0.2060539801454642, 0.7529566535614006, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.21770737118840489, -0.34315853378699207, 0.9136989226322835, 1.8019903695881379],q22] # 0.6 8-12

# from seed 1495215395, problem collision at psf.configAtParam(2,21.839) in fullBody-path:
waypoints = [q11, [0.6681212708790929, -4.714532557759675, -0.960054465809364, 0.3478615351212342, 0.18899617235908275, -0.2505225641314069, 0.8834654741854598, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15964885858238975, -0.574145069126757, 0.803037783389282, 2.493407456435754], [-2.9733257210896857, -1.956701816549712, 0.10953043083306638, 0.525979710926289, 0.17235969998530026, 0.06956781319715223, -0.8299384295728454, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.21291336090446936, -0.2967894136145435, 0.9309049063758869, -1.998720670364892], [-5.183394413315726, -6.802162358694671, -1.7, 0.4607771810197929, -0.07671191925798712, 0.12123777303281769, 0.8758430643016202, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.022648006280389436, 0.2830647290969613, 0.9588333676675981, 2.1913473609255227], q22]

for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs ()


#ps.setInitialConfig (waypoints[1]); ps.addGoalConfig (waypoints[2]) # orientation cannot be applied on this path
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

print("start solve")
t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
print("path length= " + str(ps.pathLength(solutionPathId)))
#pp(solutionPathId)

#rbprmBuilder.rotateAlongPath (solutionPathId,True)
#orientedpathId = ps.numberPaths () - 1
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

print("number of waypoints= " + str(len(pathWaypoints)))

"""print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriWaypoints = ps.getWaypoints(orientedpathId)
for i in range(1,len(pathOriWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))"""

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriBisWaypoints = ps.getWaypoints(orientedpathIdBis)
for i in range(1,len(pathOriBisWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriBisWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))


#plotConeWaypoints (ps, solutionPathId, r, "cone_planning_wp_group", "friction_cone")
#plotCone (q11, ps, r, "cone_11", "friction_cone"); plotCone (q22, ps, r, "cone_21", "friction_cone")


# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -8
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


"""
# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("skeleton_desert_path\n")
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

"""
gui.setVisibility('skeleton_trunk/thorax_lhand_rom',"OFF")
gui.setVisibility('skeleton_trunk/thorax_rhand_rom',"OFF")
gui.setVisibility('skeleton_trunk/pelvis_rfoot_rom',"OFF")
gui.setVisibility('skeleton_trunk/pelvis_lfoot_rom',"OFF")
"""

"""
r(ps.configAtParam(solutionPathId,5.7)); # if direct path between waypoint[1] and waypoint[2]
r(ps.configAtParam(orientedpathIdBis,13));
q=[0.44599324076883073, 4.7573468501743275, 0.14447678856891688, 0.8260426507482634, -0.017523400607537654, 0.2163703748459151, 0.5201253026578965, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q=[0.3, 4.7, 1.5, 0.8260426507482634, -0.017523400607537654, 0.2163703748459151, 0.5201253026578965, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qOrientatedConfigCausingCollision=[0.3114070642971426, 4.704979667308292, -0.13038551761029638, -0.18015551946655722, 0.1619221584414249, 0.146887257962494, 0.9590356285663557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
r(q); rbprmBuilder.isTrunkCollisionFree (q)

"""
