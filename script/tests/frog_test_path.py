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
urdfName = 'frog_trunk'
urdfNameRoms = ['FrogLFootSphere','FrogRFootSphere','FrogLHandSphere','FrogRHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
#base_joint_xyz_limits = [-2, 4, -1.5, 1.5, -0.1, 2.7] # groundcrouch
base_joint_xyz_limits = [-4.5, 3.6, -6.3, 4.9, -0.15, 2.7] # etang # all rocks
#base_joint_xyz_limits = [-2.3, 1.5, -3.9, 3.7, -0.15, 2.7]# etang juste middle

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
filterRange = -1
rbprmBuilder.setNormalFilter('FrogLFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('FrogRFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('FrogLHandSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('FrogRHandSphere', [0,0,1], filterRange)
rbprmBuilder.setContactSize (0.03,0.08)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
rbprmBuilder.setNumberFilterMatch(4)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ('hpp-rbprm-corba', "etang_envir", "etang_envir")
#r.loadObstacleModel ('hpp-rbprm-corba', "groundcrouch", "planning")
addLight (r, [0,0,6,1,0,0,0], "li");
plotFrame (r, "frameGroupName", [0,0,0], 0.5)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [-1.44, 2.78, -0.11, 1, 0, 0, 0]; r(q11) # etang
#q11[0:7] = [-3.2, 0, 0.05, 1, 0, 0, 0]; r(q11) # groundcrouch

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [-0.35, -3.45, 0.035, 1, 0, 0, 0]; r(q22) # etang
#q22[0:7] = [3, 0, 0.05, 1, 0, 0, 0]; r(q22) # groundcrouch

rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(5)
rbprmBuilder.setMaxLandingVelocity(9)
ps.clearRoadmap();

"""waypoints = [q11,[-3.4380851915994453, 1.7541831659078237, -0.0390517222368105, 0.7748868295565413, 0.019024435944985135, 0.01661127731028037, -0.6315952324732255, 0.0, 0.0, 0.0, 0.001712233933005589, -0.05046677681594956, 0.9987242726061922, -1.3675261146742888], [-3.3069263489529437, -0.2904785800906638, -0.035528087454390525, 0.9040177320866541, 0.00944267615582575, -0.010664217879556362, -0.427257592556219, 0.0, 0.0, 0.0, -0.05535433119848183, 0.06902607923142273, 0.9960779579950086, -0.8831575225526748], [-3.225143128914026, -2.2279746134359937, -0.020738544803490605, 0.7342500589396695, -0.005573924432437081, 0.027658275113167995, -0.6782925638183553, 0.0, 0.0, 0.0, 0.025202719386955046, 0.08019218350385152, 0.9964607551932925, -1.4923646883829613], [-3.2025236334650717, -2.9397046906642594, -0.04017919498616955, 0.6514176817322079, 0.01738285409104554, -0.025090214087024776, -0.7581050860318459, 0.0, 0.0, 0.0, 0.07375505369692835, 0.05331986766734805, 0.9958499805523418, -1.7220814508980045], [-1.8462329791441434, -4.587860060566859, -0.04269699434596526, 0.880911039220713, -0.00721905984877047, 0.20146514313498615, -0.42820021281590104, 0.0, 0.0, 0.0, 0.36112814313872915, -0.15981613530362024, 0.9187193625529989, -0.9408492769869656],q22] # limVelocity = 5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs ()"""


ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

rbprmBuilder.rotateAlongPath (solutionPathId,False)
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


plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")



# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("frog_spond_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()


# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -6.5; qAway[1] = -2
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


