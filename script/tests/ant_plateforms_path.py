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
urdfName = 'ant_trunk'
urdfNameRoms = ['LFFootSphere','LMFootSphere','LBFootSphere','RFFootSphere','RMFootSphere','RBFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-7.6, 8.7, -4.2, 3.1, -0.7, 10.]

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
obstacleName = "high_plateforms"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [0,0,6,1,0,0,0], "li");

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [7.3, 2, 7.6, math.sqrt(2)/2.0, 0, 0, -math.sqrt(2)/2.0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [-6.3, 0, 0.15, 0, 0, 0, -1]; r(q22)

rbprmBuilder.isConfigValid(q22)

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(4)
ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(7)
rbprmBuilder.setMaxLandingVelocity(10)
ps.clearRoadmap();
ps.setInitialConfig (q11); ps.addGoalConfig (q22)


"""waypoints = [q11,[7.25378532443401, 1.3886649946659833, 7.5520801941761615, 0.985606670859744, 0.013397052633934705, -0.008947379422733893, 0.16828533429553966, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01312813872936618, -0.02941987436617791, 0.9994809267643785, 0.3379649806125156], [8.432877387013622, 1.0087764910838277, 7.372942474233586, 0.8216220972301194, 0.15422371110318778, -0.43385166010715964, 0.33604302298813227, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6092726175368997, -0.5450128647000856, 0.5759755679116442, 0.8107392418240241], [5.589574924594237, -1.5504027061520784, 5.637583782849159, -0.1644987314552547, 0.6877064543463381, 0.1644987314552547, 0.6877064543463381, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.891760669398448, 0.45250735741425463, 0.0, 2.5054248467394653], [7.652939724371075, -0.4284223045842288, 3.7431917501702694, 0.6877064599564451, 0.16449870800153552, -0.6877064599564451, 0.16449870800153552, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8917607002633022, -0.45250729658857974, 0.0, -2.9958769251798216], [6.289624667736375, -1.5760467697080667, -0.37130662025071826, 0.3786008094188104, -0.11019122520250761, 0.24675550355732426, 0.8852293727955276, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.008245351596474862, 0.5203074134008291, 0.8539392307050832, 2.4121701477263695], [1.9189887651269564, -1.418257975323879, -0.5165447991678072, 0.7606807990895826, -0.22244323484795459, 0.18739610664052983, 0.5803158005627478, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02692259247007357, 0.5559144385673098, 0.8308034129726978, 1.2606481344076457], [-0.9353164939128282, 1.3197688394469966, -0.4267681946222957, 0.5457130109686646, 0.15953229399828547, 0.23822788699057967, -0.7873971238778831, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.008777576092882308, -0.5492776030968608, 0.8355938420632965, -1.9946842420939663], [-1.725213943453129, 1.497826183684346, -0.34462248785963334, -0.18800310112648239, -0.078218400019738, -0.2765310026474743, 0.9391843910755547, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.042945628676202226, -0.5488378062076864, 0.8348249729462697, -2.8157373357989797], [-5.225976927018492, 0.5884927409540129, 0.08059319754618097, 0.9380279050911801, -0.01700497545545167, 0.0018258130588518619, -0.34613746761465397, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015197445479364642, 0.03063831838815104, 0.9994149944328672, -0.7068713383919936],q22] # limVelocity = 4.5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()"""

waypoints = [q11,[7.07959031147824, 1.1362842434027287, 7.538299642966478, -0.0007384758192959845, -0.026937938964754573, -0.27698156610775493, 0.9604972744023227, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05133854552940317, -0.532119864645465, 0.8451111189616531, 3.128069819217451], [8.010885515698124, 0.7232829625516763, 6.750120418960795, 0.8855687465175004, 0.28255986603872457, -0.3635548626388539, 0.06128441194337504, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6092726175368997, -0.5450128647000856, 0.5759755679116442, -0.1323127114171316], [5.567010552968113, -1.5405982384335797, 5.630911305234672, -0.1644987314552547, 0.6877064543463381, 0.1644987314552547, 0.6877064543463381, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.891760669398448, 0.45250735741425463, 0.0, 0.0021700005159563957], [7.744046701645998, -0.42903643900564103, 3.641336139125948, 0.659691830076613, 0.2545715799734312, -0.659691830076613, 0.2545715799734312, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7407732426793237, -0.6717551659126707, 0.0, 0.3131104413096165], [6.047819772608358, -2.0216989681163477, -0.13637668495654415, 0.4506437615764707, -0.13449486815983838, 0.2202264764881562, 0.8545944240658772, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.03138975326303659, 0.4976271842774687, 0.8668228589845586, 2.2359149013572432], [2.1902098197108066, -1.6710777266564258, -0.45410241012955144, 0.15581452434326565, -0.015596167629396446, 0.28104885742871843, 0.9468316288004798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05804909847577458, 0.5370721137849991, 0.8415365985865754, 2.8491651134042297], [-1.2506117944243902, 1.4777705996105117, -0.3633975080322681, 0.8403948638201185, 0.26213639113598103, 0.11777956515911042, -0.4595094768749598, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.042945628676202226, -0.5488378062076864, 0.8348249729462697, -0.9121454432637109], [-5.412405591614964, 0.5209427766668708, 0.06043558140394678, 0.9203122454471577, -0.011132005099516937, 0.012983884088773927, 0.39081052710545566, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015197445479364642, 0.03063831838815104, 0.9994149944328672, 0.8029794883783902],q22] # limVelocity = 4.5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()

"""waypoints = [q11,[7.1652870657160435, 1.3880080481758346, 7.525836746600447, 0.9747237505122599, 0.015804911406235227, -0.0031211205122254255, -0.22283194019148814, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01312813872936618, -0.02941987436617791, 0.9994809267643785, -0.4494808705488722], [7.172879382158492, 1.1321782797652622, 7.473735231099867, 0.904083873908351, 0.2697569010739118, 0.06837876016844123, -0.3243268542993249, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05133854552940317, -0.532119864645465, 0.8451111189616531, -0.6136059483735697], [8.15410239908786, 0.6973448199136761, 6.773742387903063, 0.6927937985914323, 0.02502290343227548, -0.45976740896674806, 0.554999582512524, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6092726175368997, -0.5450128647000856, 0.5759755679116442, 1.6227892070455492], [5.6090926173203925, -1.644469349862436, 5.4292282512444, -0.1644987314552547, 0.6877064543463381, 0.1644987314552547, 0.6877064543463381, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.891760669398448, 0.45250735741425463, 0.0, -0.6022245773741984], [7.52580352019293, -0.28714990276174335, 3.524342197640343, 0.6877064599564451, 0.16449870800153552, -0.6877064599564451, 0.16449870800153552, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8917607002633022, -0.45250729658857974, 0.0, 0.6274008211403866], [6.310039800833069, -1.9498744712726102, -0.16723046179584466, 0.760774499384863, -0.21280809786177063, 0.14594959401168534, 0.5955112010516599, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.03138975326303659, 0.4976271842774687, 0.8668228589845586, 1.2848533341140704], [1.8772353767368373, -1.6530184083698691, -0.4078760368150377, 0.7715708724315896, -0.24260333427445072, -0.16044287367337393, -0.5657562154333832, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02692259247007357, 0.5559144385673098, 0.8308034129726978, -1.2008734069910565], [1.2387979433354905, -1.5872722652164228, -0.4760566280390759, -0.3693831748977321, 0.09775029564706536, 0.2700466599234046, 0.883784901018113, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0267209145002048, 0.5495409503279662, 0.8350393623302488, -2.4384821776831402], [-1.614351268855471, 1.400297742253678, -0.4172051904749453, 0.8398872566567422, 0.2563880118022962, 0.10167835290376599, -0.46745705263741005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.06890406289389353, -0.5257345740754558, 0.8478533999096857, -0.9338948043534562], [-4.813122316186568, -0.270740267815124, -0.09466147147782422, -0.08245944519217718, 0.2872176484355056, 0.005967256120155657, 0.9542907597688312, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5471941826310961, 0.05875661063206428, 0.8349408285631496, -2.9563178004279256],q22] # limVelocity = 4.5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs ()"""

"""waypoints = [q11,[7.314827450132212, 1.5412454662863557, 7.5573732094390245, 0.9882600547501111, 0.015538550944622866, -0.004253240218014794, -0.1519293505794605, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01312813872936618, -0.02941987436617791, 0.9994809267643785, -0.30513887195518674], [5.879514993735331, 0.7679372792313417, 9.196279138343007, 0.0572013187447582, 0.7047893367055581, -0.0572013187447582, 0.7047893367055581, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9869120365354423, -0.16125951798720536, 0.0, 1.9959660158055832], [7.783877465161486, 0.8321374193122467, 7.293324900642288, -0.36114962399270356, -0.3477335978939522, -0.2887805806910488, 0.8156335391592211, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3586603740036937, -0.7222459703970519, 0.5913742422229357, -2.478670851675725], [5.902408698609108, -2.166907287293787, 5.5883696342690365, -0.16449872363734763, 0.687706456216374, 0.16449872363734763, 0.687706456216374, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8917606796867341, 0.4525073371390281, -0.0, -1.614501284582167], [8.151747289549167, -0.8343844530958608, 3.616082865773722, 0.6598970289963169, 0.2540391920980582, -0.6598970289963169, 0.2540391920980582, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7418563555126636, -0.6705588324565329, -0.0, 0.026962810527553952], [5.948865012793935, -2.1125589306026855, -0.05890191406387297, 0.1572707748048132, -0.057951057924417265, 0.2514562494613358, 0.9532456833810332, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.03138975326303659, 0.4976271842774687, 0.8668228589845586, 2.86228473536954], [1.041714609700018, -1.8492952236762288, -0.22982018142242727, 0.12479237129459392, -0.05120086480811822, 0.2825929763419049, 0.9497086633446051, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0267209145002048, 0.5495409503279662, 0.8350393623302488, 2.9289056121874912], [-2.012050860364957, 1.8420254816113901, -0.14647918576397997, 0.4407303407472952, 0.09354004263160068, -0.25946784091604774, 0.8542151173443466, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.06890406289389353, -0.5257345740754558, 0.8478533999096857, 2.271371412794059],q22] # limVelocity = 4.5
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
f.write("ant_plateforms_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -6.5; qAway[1] = -2
rbprmBuilder.setCurrentConfig (qAway); r(qAway)

#pathSamples = plotSampleSubPath (ps.client.problem, r, solutionPathId, 70, "sampledPath", [1,0,0,1])
#writePathSamples (pathSamples, 'antPlatforms_path.txt')
"""
