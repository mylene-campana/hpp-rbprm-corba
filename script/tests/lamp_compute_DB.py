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
urdfName = "lamp"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])
q_0 = fullBody.getCurrentConfig()


flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 2.0, -1.0, 0.0]

"""
from hpp.gepetto import Viewer
psf = ProblemSolver( fullBody ); rr = Viewer (psf)
rr(q_0)
"""

q_leg = flexion[7:12]

legWheights = [1.,2.,2.,2.,1.]

fullBody.addRefConfigAnalysisWeight(q_leg,"RefPoseLeg",legWheights)

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_6_DOF"
x = 0.05 # contact surface width
y = 0.05 # contact surface length
offset = [0,0,0]
normal = [0,0,1]

LegId = 'Foot'
Leg = 'ThighJoint'
foot = 'LampFootSphere'
fullBody.addLimb(LegId, Leg, foot, offset, normal, x, y, nbSamples, "manipulability", 0.01, cType)
print("Limbs added to fullbody")


def runallLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLeg", True)
    fullBody.saveLimbDatabase(lid, dbName)

runallLeg(LegId, './Lamp_leg_zPlus.db') # 6DOF
#runallLeg(LegId, './Lamp_leg_3DOF.db')


##plotOctreeValues(fullBody, "isotropy", lLegId)



"""
# flexion
legWheights = [1.,2.,2.,2.,1.]
q = q_0 [::]
q [fullBody.rankInConfiguration ['ThighJoint']] = 0.; rr(q)
q [fullBody.rankInConfiguration ['ThighJointBis']] = -1.; rr(q)
q [fullBody.rankInConfiguration ['ShankJoint']] = 2.; rr(q)
q [fullBody.rankInConfiguration ['AnkleJoint']] = -1.; rr(q)
q [fullBody.rankInConfiguration ['FootJoint']] = 0.; rr(q)

fullBody.isConfigValid(q)

fullBody.rankInConfiguration ['ThighJoint']
fullBody.rankInConfiguration ['ThighJoint'] # = 7
fullBody.rankInConfiguration ['FootJoint'] # = 11

## DEBUG joint orientation
fullBody.setCurrentConfig(q_0); rr(q_0)
psf.client.robot.getCurrentTransformation("LampFootSphere")
plotJointFrame (rr, psf, q_0, "LampFootSphere", 0.2)
plotFrame(rr,"framy",[0,0,0],0.2)
psf.client.robot.getCurrentTransformation("ThighJoint")
plotJointFrame (rr, psf, q_0, "ThighJoint", 0.2)

"""
