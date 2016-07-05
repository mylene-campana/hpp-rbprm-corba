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
#addLight (r, [-3,0,3,1,0,0,0], "li"); addLight (r, [3,0,3,1,0,0,0], "li1")

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
rbprmBuilder.setMaxTakeoffVelocity(5.5)
rbprmBuilder.setMaxLandingVelocity(11)
ps.clearRoadmap();

waypoints = [q11,[4.380363841566166, -0.2395008521358996, 0.31822216644172663, 0.9881758731112289, -0.061038309083110046, 0.13595520313504933, 0.036040967878827185, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2642955436498991, 0.13043308296117012, 0.9555810151295644, 0.05682206280202427], [3.96715567497854, -1.3695822036353809, -0.07319167733446534, 0.06068297960615738, 0.05549025867439458, 0.1321121236942327, 0.9878181988358826, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.12566248936908186, 0.25427089167308314, 0.9589344359304971, 3.0060352732576665], [2.137867531069525, -3.1403826446408396, -0.3651051941628204, 0.8834361058712585, -0.44101679490008394, -0.10575480840432223, -0.11773170328526136, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.08301191534576367, 0.804121707268802, 0.5886402142308448, -0.12020547265068776], [0.754889018489108, -2.5975957910860954, -0.4566754367132074, 0.8459750854753072, -0.14802661293769492, 0.5017051830679712, -0.10347070068369106, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8794930049796569, 0.1466300794078084, 0.4527600622898547, -0.5978701431925089], [1.7225454289441677, -3.717102859904074, -0.6104892007383532, -0.19836467795767382, 0.06637467835232806, 0.2080128483550413, 0.9554980437089319, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.044316947284905706, 0.4238445227309368, 0.9046501139856952, -2.773939469473469], [-0.20221674201812587, -4.57136947021967, -0.3589872988014835, -0.19272334492386584, 0.1834792562591249, -0.2376994353518074, 0.9341798827192689, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.434425720722165, -0.37338658929797236, 0.8196686819120607, -2.661210308427507], [-1.4368384543193236, -4.172377887101355, -0.2563298470514175, 0.9694633130262547, 0.17572747277281497, -0.1254850897688729, 0.11625073012560272, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5069111598587975, -0.2576362259583688, 0.822596286816648, 0.19023996055226025], [-2.6266387285631687, -4.662743336261297, -0.40429011072077814, 0.8710395981353675, -0.007876220318188925, -0.2473682797781931, -0.42430757451788975, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4242512541397104, 0.22364146917225752, 0.8774937986261538, -0.9574759700203535], [-5.5739014926343895, -5.426648331782036, -1.7, 0.9512917744736231, -0.30266484925312903, -0.04053007923911961, 0.04237052657540177, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.10276000007531663, 0.572410601434593, 0.8135026034069027, 0.10548254806064082],q22] # limVelocity = 5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()

ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])

rbprmBuilder.rotateAlongPath (solutionPathId)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

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



plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group", "friction_cone2")
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


