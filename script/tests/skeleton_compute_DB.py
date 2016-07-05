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
urdfName = "armlessSkeleton"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])



flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, -1.8, 2, -0.5, 0.1, -0.1, -0.2, -1.8, 2, -0.5, -0.1]

q_lfeet = flexion[31:37]
q_rfeet = flexion[37:43]

fullBody.addRefConfigAnalysisWeight(q_lfeet,"RefPoseLFeet",[1.,1.,1.,5.,1.,1.])
fullBody.addRefConfigAnalysisWeight(q_rfeet,"RefPoseRFeet",[1.,1.,1.,5.,1.,1.])

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.08 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RHip_J1'
rfoot = 'RFootSphere'
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LHip_J1'
lfoot = 'LFootSphere'
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Legs added to fullbody")


def runallLLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallRLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseRFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)



runallLLeg(lLegId, './armlessSkeleton_lleg.db')
runallRLeg(rLegId, './armlessSkeleton_rleg.db')



##plotOctreeValues(fullBody, "isotropy", lLegId)

