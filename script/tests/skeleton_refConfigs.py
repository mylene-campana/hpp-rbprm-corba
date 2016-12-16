#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
from viewer_library import *
from hpp.gepetto import Viewer

packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
urdfName = "skeleton"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])
q_0 = fullBody.getCurrentConfig()

# flexion: for waypoint computation (maybe then -> q_contact)
flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -0.2, -1, -2.5, 0.0, 0.0, -0.2, -0.1, -0.2, -1, 2.5, 0.0, 0.0, 0, 0.1, -2, 2.5, -0.7, 0, -0.2, 0, -0.1, -2, 2.5, -0.7, 0, -0.2]
extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.6, -1, -1.5, 0.0, 0.0, -0.2, -0.1, -1.7, -1.1, 1.5, 0.0, 0.0, 0, 0.0, -1.2, 1.5, 0.0, 0, -0.0, 0, -0.0, -1.4, 1.7, 0.1, 0, -0.0]
q_contact_landing = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.4, -1, -1.2, 0.0, 0.0, -0.0, 0.2, 1.5, -1.1, 1.2, 0.0, 0.0, 0, 0.1, -1.1, 0.9, 0, 0, -0.2, 0, -0.1, -1.1, 0.9, 0, 0, -0.2]
q_contact_takeoff =[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, -1.4, -1.3, -1.5, 0.0, 0.0, -0.2, -0.1, -1.5, -1.2, 1.5, 0.0, 0.0, 0.0, 0.1, -1.4, 2.0, -0.6, 0.0, -0.2, 0.0, -0.1, -1.4, 2.0, -0.6, 0.0, -0.2]

psf = ProblemSolver( fullBody ); r = Viewer (psf)
r(q_0)


"""
fullBody.isConfigValid(q)

fullBody.rankInConfiguration ['RShoulder_J1']
fullBody.rankInConfiguration ['RHand']

fullBody.rankInConfiguration ['LShoulder_J1']
fullBody.rankInConfiguration ['LHand']

fullBody.rankInConfiguration ['RHip_J1']
fullBody.rankInConfiguration ['RFootToe']

fullBody.rankInConfiguration ['LHip_J1']
fullBody.rankInConfiguration ['LFootToe']
"""
"""
# flexion
q = q_0 [::]
q [fullBody.rankInConfiguration ['RHip_J2']] = -0.1; r(q)
q [fullBody.rankInConfiguration ['RThigh']] = -2; r(q)
q [fullBody.rankInConfiguration ['RShank']] = 2.5; r(q)
q [fullBody.rankInConfiguration ['RAnkle_J1']] = -0.7; r(q)
q [fullBody.rankInConfiguration ['RFootToe']] = -0.2; r(q)

q [fullBody.rankInConfiguration ['LHip_J2']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['LThigh']] = -2; r(q)
q [fullBody.rankInConfiguration ['LShank']] = 2.5; r(q)
q [fullBody.rankInConfiguration ['LAnkle_J1']] = -0.7; r(q)
q [fullBody.rankInConfiguration ['LFootToe']] = -0.2; r(q)

q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.2; r(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['LHumerus']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['LElbow_J1']] = -1; r(q)
q [fullBody.rankInConfiguration ['LForearm']] = -2.5; r(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = -0.1; r(q)
q [fullBody.rankInConfiguration ['RHumerus']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['RElbow_J1']] = -1; r(q)
q [fullBody.rankInConfiguration ['RForearm']] = 2.5; r(q)

# extending
q = q_0 [::]
q [fullBody.rankInConfiguration ['RHip_J2']] = -0.0; r(q)
q [fullBody.rankInConfiguration ['RThigh']] = -1.4; r(q)
q [fullBody.rankInConfiguration ['RShank']] = 1.7; r(q)
q [fullBody.rankInConfiguration ['RAnkle_J1']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['RFootToe']] = -0.0; r(q)

q [fullBody.rankInConfiguration ['LHip_J2']] = 0.0; r(q)
q [fullBody.rankInConfiguration ['LThigh']] = -1.2; r(q)
q [fullBody.rankInConfiguration ['LShank']] = 1.5; r(q)
q [fullBody.rankInConfiguration ['LAnkle_J1']] = 0.0; r(q)
q [fullBody.rankInConfiguration ['LFootToe']] = -0.0; r(q)

q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.2; r(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['LHumerus']] = -1.6; r(q)
q [fullBody.rankInConfiguration ['LElbow_J1']] = -1; r(q)
q [fullBody.rankInConfiguration ['LForearm']] = -1.5; r(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = -0.1; r(q)
q [fullBody.rankInConfiguration ['RHumerus']] = -1.7; r(q)
q [fullBody.rankInConfiguration ['RElbow_J1']] = -1.1; r(q)
q [fullBody.rankInConfiguration ['RForearm']] = 1.5; r(q)

# q_contact landing
q = q_0 [::]
q [fullBody.rankInConfiguration ['RHip_J2']] = -0.1; r(q)
q [fullBody.rankInConfiguration ['RThigh']] = -1.1; r(q)
q [fullBody.rankInConfiguration ['RShank']] = 0.9; r(q)
q [fullBody.rankInConfiguration ['RAnkle_J1']] = 0; r(q)
q [fullBody.rankInConfiguration ['RFootToe']] = -0.2; r(q)

q [fullBody.rankInConfiguration ['LHip_J2']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['LThigh']] = -1.1; r(q)
q [fullBody.rankInConfiguration ['LShank']] = 0.9; r(q)
q [fullBody.rankInConfiguration ['LAnkle_J1']] = 0; r(q)
q [fullBody.rankInConfiguration ['LFootToe']] = -0.2; r(q)

q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.0; r(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['LHumerus']] = 1.4; r(q)
q [fullBody.rankInConfiguration ['LElbow_J1']] = -1; r(q)
q [fullBody.rankInConfiguration ['LForearm']] = -1.2; r(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = -0.0; r(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = 0.2; r(q)
q [fullBody.rankInConfiguration ['RHumerus']] = 1.5; r(q)
q [fullBody.rankInConfiguration ['RElbow_J1']] = -1.1; r(q)
q [fullBody.rankInConfiguration ['RForearm']] = 1.2; r(q)

"""

