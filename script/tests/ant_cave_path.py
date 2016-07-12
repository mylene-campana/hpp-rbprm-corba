#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a ant-robot and a groundcrouch environment.
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
urdfName = 'ant_trunk'
urdfNameRoms = ['LFFootSphere','LMFootSphere','LBFootSphere','RFFootSphere','RMFootSphere','RBFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-4.5, 6.8, -8.8, 14.3, -6.9, 3.2])
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
filterRange = 0.3
rbprmBuilder.setNormalFilter('LFFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('LMFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('LBFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RFFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RMFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RBFootSphere', [0,0,1], filterRange)
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
rbprmBuilder.setNumberFilterMatch(3)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ('hpp-rbprm-corba', "cave", "cave")
addLight (r, [-3,0,8,1,0,0,0], "li");
addLight (r, [-1,15,0,1,0,0,0], "li2");

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [0.955, -5.37, -3.076, 1, 0, 0, 0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [1.099, 12.393, -1.58, 1, 0, 0, 0]; r(q22)

rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(7)
rbprmBuilder.setMaxLandingVelocity(10)
ps.clearRoadmap();

"""waypoints = [q11,[4.374038684776649, -3.2811996652638915, -3.3249079374599915, 0.5112070400488823, -0.2893272443714401, 0.08767472229651477, 0.8045310751854887, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.37590564744460053, 0.4368863255928554, 0.817205777469316, 1.9526222049474145], [5.600942567103102, -0.46773937508992275, -1.9366449655370035, 0.6218491902614041, 0.19431091624338317, -0.09660289941313882, -0.7524724129327719, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.41257207756360104, -0.0962821382143858, 0.9058222953072638, -1.7247914686052386], [6.119830277735661, 3.6209698106610078, -2.0016302179753094, 0.2550230622393613, -0.32704525774387827, -0.22689220801941512, 0.881206311287756, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6921141817759956, -0.23306952513603602, 0.6831226506546293, 2.4025339004820774], [4.676754868932058, 4.059984369608627, -2.5441703987902606, -0.33423425546330615, 0.07787742969577696, 0.3128060457809936, 0.8856494487898289, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.07115678626559545, 0.6061316135116538, 0.7921749673336765, -2.5237071387931116], [2.7607200549147444, 8.122970264523747, -2.7542783837549423, 0.4519150149288567, -0.2112528996274209, -0.26985650890048574, 0.8236033610208862, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5918816128139278, -0.25357274086982506, 0.7650993540060979, 2.1002160041817346], [-2.015454657390148, 8.869957834783735, -2.7871303099515456, -0.3076486722266994, 0.460475184101702, 0.08204081121455944, 0.8286037681539407, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7126234520989787, 0.41928780859404724, 0.5624638202435213, -2.2982114537734257], [-0.8617345070893399, 10.340335015722236, -3.2927343024648708, -0.07254658758042087, -0.3801642494664952, 0.05947153306046564, 0.9201495926279871, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7082448721200659, 0.05428617579936653, 0.7038765603660287, -2.8917566508239827], [-2.088825446309726, 11.900865225981862, -1.207104256671644, 0.30133265820097394, 0.3535211438752491, -0.3612963593777511, 0.8085087325657364, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35390907920026965, -0.7972774552305962, 0.48897548306703675, 2.754384245585081], [2.2584427360266477, 11.272520544410806, -1.7104039460259073, -0.18005509196466535, -0.47117226464339523, -0.025296043006627605, 0.8630973126459247, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8042256681142168, -0.21333982436232438, 0.5547136144780423, -2.6128102811038447],q22] # limVelocity = 4.5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()"""

ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True) # <- rotate after jump
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


plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group_planning", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")


"""
# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("ant_cave_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

rob = rbprmBuilder.client.basic.robot
r(q11)

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[1] = -11
rbprmBuilder.setCurrentConfig (qAway); r(qAway)
"""
