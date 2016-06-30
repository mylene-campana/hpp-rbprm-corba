#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
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


#~ AFTER loading obstacles
nbSamples = 50000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.08 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_ry'
rfoot = 'SpidermanRFootSphere'
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LThigh_ry'
lfoot = 'SpidermanLFootSphere'
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,cType)
print("Limbs added to fullbody")




def runall(lid, dbName):
	fullBody.runLimbSampleAnalysis(lid, "isotropy", True)
	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	#fullBody.runLimbSampleAnalysis(lid, "selfCollisionProbability", False)
	#fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.saveLimbDatabase(lid, dbName)

runall(lLegId, './Spiderman_lleg.db')
runall(rLegId, './Spiderman_rleg.db')
