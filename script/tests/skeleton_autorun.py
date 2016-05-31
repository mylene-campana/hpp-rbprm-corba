#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'skeleton_trunk_flexible'
#urdfNameRoms = ['LHandSphere','RHandSphere','LFootSphere','RFootSphere']
urdfNameRoms = ['LFootSphere','RFootSphere']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-8, 6, -2, 2, 0, 3])
#rbprmBuilder.setJointBounds ("base_joint_SO3", [-1, 1, -0.1, 0.1, -0.1, 0.1, -1,1])
rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.setFilter(urdfNameRoms)
#rbprmBuilder.setNormalFilter('LHandSphere', [0,0,1], 0.5)
#rbprmBuilder.setNormalFilter('RHandSphere', [0,0,1], 0.5)
rbprmBuilder.setNormalFilter('LFootSphere', [0,0,1], 0.5)
rbprmBuilder.setNormalFilter('RFootSphere', [0,0,1], 0.5)
rbprmBuilder.getCurrentConfig ()


ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

