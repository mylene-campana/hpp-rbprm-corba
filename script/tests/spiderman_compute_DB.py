#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder

from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.tools.plot_analytics  import plotOctreeValues
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
from viewer_library import *



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])



q_jump= fullBody.getCurrentConfig()
q_jump = [0, 0, 0, 1, 0,0, 0, 0, 0, 0.2,
 0.0, 0.0, 0.0, 0.4, 0.5, 0.7, 0, -0.6, 0.0, 0.0, 0.4, 0.5,
 0.7, 0, -0.6, 0.0, 0.0, -0.2, 0.3, -1.9, 1.9,-0.6, 0, -0.2, 0.3,
 -1.9, 1.9, -0.6, 0] #; rr(q_jump)

q_feet = q_jump[27:32]
q_arm = q_jump[13:19]


fullBody.addRefConfigAnalysisWeight(q_feet,"RefPoseFeet",[1.,1.,1.,5.,1.,1.])
fullBody.addRefConfigAnalysis(q_arm,"RefPoseArm")



#~ AFTER loading obstacles
nbSamples = 20000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.08 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_ry'
rfoot = 'SpidermanRFootSphere'
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LThigh_ry'
lfoot = 'SpidermanLFootSphere'
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Legs added to fullbody")

rarmId = 'rhand'
rLeg = 'RHumerus_ry'
rfoot = 'SpidermanRHandSphere'
rarmx = x; rarmy = y
fullBody.addLimb(rarmId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

larmId = 'lhand'
lLeg = 'LHumerus_ry'
lfoot = 'SpidermanLHandSphere'
larmx = x; larmy = y
fullBody.addLimb(larmId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Arms added to fullbody")

def runallLeg(lid, dbName):

	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "RefPoseFeet", True)
	fullBody.saveLimbDatabase(lid, dbName)

def runallArm(lid, dbName):

	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "RefPoseArm", True)
	fullBody.saveLimbDatabase(lid, dbName)




runallLeg(lLegId, './Spiderman_lleg.db')
runallLeg(rLegId, './Spiderman_rleg.db')
runallArm(larmId, './Spiderman_larm.db')
runallArm(rarmId, './Spiderman_rarm.db')



##plotOctreeValues(fullBody, "isotropy", lLegId)

