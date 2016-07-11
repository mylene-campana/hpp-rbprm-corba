#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a desert environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton.urdf -o skeleton_blend.py
#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton_trunk_flexible.urdf -o skeleton_trunk_blend.py


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
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(5)
rbprmBuilder.setMaxLandingVelocity(8)
ps.clearRoadmap();

waypoints = [q11,[3.821948430653096, -0.6939786015649804, 0.08396498467097675, -0.08139900465881353, -0.10480461095619605, -0.11671791597748564, 0.9842596830373311, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1873084619483559, -0.2468234600005328, 0.950785843224386, -3.001784047946517], [2.5374093902827766, -2.6493147379803514, -0.2731840267166424, 0.9796647390895924, 0.0029784671281281446, -0.10255278275253044, -0.1724269540060362, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.20196182634535925, 0.029529929465794, 0.9789481109665579, -0.35269118799073396], [0.2305686891959538, -3.7435918044881844, -0.36377292451700016, 0.3500244623887609, 0.20216088290605927, 0.11306993726673238, 0.9076502864201601, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.44613725455351216, 0.0637334131808571, 0.8926923334182796, 2.350369937082188], [-1.2997346450434215, -3.5476555113015977, -0.0005785320620854059, 0.8341098515653678, 0.07335177016166292, -0.17744494278005196, 0.5171011174011664, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.22015658513868974, -0.30588082462070504, 0.9262656201918693, 1.122627270248954], [-2.646067518624198, -4.461664512091197, -0.6689846890841317, -0.2938260280750027, -0.1510302724312541, -0.14342191340300947, 0.9328913531547114, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1975074881231882, -0.35634737586826465, 0.9132400231303216, -2.5681181893523086], [-3.7680697400877543, -4.063264575690397, -0.782984284592559, 0.8039096970742332, -7.661423923264766e-05, -0.159160561192074, 0.5730594287168627, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25598924609250284, -0.1822937386822642, 0.9493358197825649, 1.2633657914730936],q22] # limVelocity = 5
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


plotConeWaypoints (ps, solutionPathId, r, "cone_planning_wp_group", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")



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


