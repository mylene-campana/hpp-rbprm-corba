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
base_joint_xyz_limits = [-4.5, 6.8, -8.8, 14.3, -6.9, 3.2]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
rbprmBuilder.setAffordanceFilter('LFFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('LMFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('LBFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RFFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RMFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RBFootSphere', affordanceType)
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "cave"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [-3,0,8,1,0,0,0], "li");
addLight (r, [-1,15,0,1,0,0,0], "li2");

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [6., 0.001, 0.04]) # default (0.3,0.3,0.05) error, angle and area 
afftool.setNeighbouringTriangleMargin ('Support', 0.01)
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(3)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [0.99, -5.4, -3.08, 1, 0, 0, 0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [1.15, 12.28, -1.51, 1, 0, 0, 0]; r(q22)

rbprmBuilder.isConfigValid(q22)


ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(8)
rbprmBuilder.setMaxLandingVelocity(11)
ps.clearRoadmap();

#waypoints = [q11, [0.6901380355999407, -0.5430509736537911, -4.670955841138794, 0.888750104182867, 0.08987665039267288, 0.3084133130675408, -0.32699643476930945, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.48942603974549675, -0.3614558724171437, 0.7936068320737927, -0.7229753483504328], [0.052893205768872645, 3.119172798455125, -5.320867914612529, 0.7616856768083492, 0.08812954780305178, 0.6065418244086808, -0.2101787995779447, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8869425149454085, -0.38921849362642846, 0.2486803960940375, -0.881234870762758], [-0.10786517776381836, 1.8600570149108293, -4.970744392159722, 0.5668198304960639, 0.43875007847956227, 0.0400380552946423, 0.6961397866224963, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6562514994577342, -0.44164032368865647, 0.6117906455248023, 1.5373591594499505], [-1.6151105484450805, 1.7060752558804404, -2.8270398811568986, 0.526587371003749, -0.35994531603579805, -0.29350744863662137, -0.7120382628469277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20347504360488344, 0.7970623830725164, 0.568585494117421, -1.9037905494959355], [-3.217094328277079, 5.1408293984596165, -0.9339678458751244, 0.6876198738876504, -0.13688731325941994, 0.6983841065285609, 0.14387637836577918, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9210558807606424, 0.389214826071623, -0.012956993566157572, -0.376007006650454], [-2.5097176322400685, 7.80455098581354, -1.0829888691453031, 0.13801735716163388, 0.5813843364335566, 0.06888939037005729, 0.7988727773345847, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9479201022817029, -0.0504145419918313, 0.31449301048830985, 2.331081324162198], [-2.4585572824697524, 7.846976773478817, -0.9842367220114518, 0.6274076438611802, 0.27405035818430146, 0.7021420765525543, -0.19558260131741126, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7738596479199611, -0.6185361267085651, -0.13621418897295395, -1.1493087718701605], [-2.5983361539001235, 9.27332260543447, -1.9112240506796074, 0.7855326532528609, -0.084337953713237, 0.6130457870534121, 0.000650548266410985, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9630252355199487, 0.1332980648485484, 0.23412394507905301, -0.39102939934897485], [0.9277702741372241, 14.118521539066277, -3.3105757953203043, -0.48425753934239796, 0.22141820862959743, 0.3267781230993855, 0.780823072623337, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.029287352387763373, 0.7247586700422395, 0.6883800703018058, -2.186843183974681], [3.456527271748119, 13.346152449565537, -1.9407486763381234, 0.351399034036947, -0.4613722432165139, 0.2710964894719139, 0.7682194123186858, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5183441380254123, 0.7407746928430297, 0.427284693170733, 2.416986299334944], q22] # 1.2   8   11
#waypoints = [q11,[0.1988379215748521, 1.8186672873534242, -5.292452719944538, 0.912813944840402, 0.177395575286985, 0.3678122140630282, -0.0039607019019601555, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6700830141738334, -0.3267718988214978, 0.6664899701093056, 0.16741496846337983], [3.6397593607277514, 1.5112407775121763, -2.9527686860172633, 0.8306097558483304, -0.3087155298944409, -0.021082246226291255, 0.4629661909747033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.32087194472144126, 0.4933235273592653, 0.8085005209920971, 0.9406367354708023], [-0.9192356007650062, 5.388406216073009, -2.4986747122562813, 0.7724304651394216, -0.1943192726962285, 0.46102638113198297, -0.3912107778214364, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8642612316730184, -0.06052072592908628, 0.49938939231831275, -1.2402983236382978], [2.7607200549147444, 8.122970264523747, -2.7542783837549423, 0.4519150149288567, -0.2112528996274209, -0.26985650890048574, 0.8236033610208862, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5918816128139278, -0.25357274086982506, 0.7650993540060979, 2.1002160041817346], [-2.683560646503113, 9.738073491923686, -2.2024921249453624, 0.1358664531052861, 0.5155435900327132, 0.2664172688061241, 0.8029800449442416, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9003367689077387, 0.2877653429619063, 0.32647329131613434, 2.289886689181118],q22] # limVelocity = 4.5
waypoints = [q11, [2.6758883579275548, 2.7463935440367706, -5.891508741334304, 0.7987845616168727, -0.223674366034081, -0.12909523216634144, 0.5433667482775429, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4493129827555979, 0.21704314779421183, 0.8666083980223407, 1.184857075564821], [-0.5580240383638044, 4.77246504825158, -3.918949621435615, -0.13274714177637556, 0.6682537203135972, 0.05866477527525011, 0.7296393669335173, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9595932803996758, 0.26302580169238016, 0.09999081885788191, -2.1266600507918763], [2.8740626680132277, 8.035714392200985, -2.624138526952736, 0.05840969740025473, -0.4212583293102765, -0.25866287339111527, 0.8673080451404663, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7609383165133803, -0.399469639057828, 0.5112698758315332, 2.6777990914017713], [1.1271307137614341, 8.534713877578017, -0.44994405590892816, 0.6600586126290492, 0.2721630323050994, -0.6798945454562594, 0.1673120402109025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8064681964059293, -0.5867961940815399, -0.07265861818936016, 1.7007468621939583], [-2.7134730069645943, 10.993611584305135, -1.5121898861356748, 0.21469803207591623, 0.6112184566085642, -0.17500722551456488, 0.7414102941971252, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8311798976316505, -0.5219591167154275, 0.19156893863747776, 2.5651667111643066], [0.9277702741372241, 14.118521539066277, -3.3105757953203043, -0.48425753934239796, 0.22141820862959743, 0.3267781230993855, 0.780823072623337, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.029287352387763373, 0.7247586700422395, 0.6883800703018058, -2.186843183974681], [3.456527271748119, 13.346152449565537, -1.9407486763381234, 0.351399034036947, -0.4613722432165139, 0.2710964894719139, 0.7682194123186858, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5183441380254123, 0.7407746928430297, 0.427284693170733, 2.416986299334944], q22]

for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()


ps.setInitialConfig (q11); ps.addGoalConfig (q22)

print('start solve')
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


#plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group_planning", "friction_cone2") # DO NOT CORRESPOND TO CONVEX-CONES
#plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")


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
