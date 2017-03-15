#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a desert environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton.urdf -o skeleton_blend.py
#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/skeleton_trunk_flexible.urdf -o skeleton_trunk_blend.py
#blender/urdf_to_blender.py -p armlessSkeleton/ -i /local/mcampana/devel/hpp/src/hpp-rbprm-corba/data/urdf/skeleton/armlessSkeleton.urdf -o armlessSkeleton_blend.py

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

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
q_0 = rbprmBuilder.getCurrentConfig () [::]; q_0 [0] = -5
r(q_0)


 ################# TEST CONVEX-CONE #################

#cones = [[0,1,1],[0,-1,1]]; theta = 0   # OK
#cones = [[0.2,1,1],[0.2,-1,1]]; theta = 0   # OK
#cones = [[0.3,1,1],[0.3,-1,1]]; theta = 0.2   # OK
#cones = [[0.4,1,1],[0.3,-0.3,1]]; theta = 0.2   # OK
#cones = [[0,0,1],[1,0,0]]; theta = 0   # OK
#cones = [[0,0,1],[0,1,0]]; theta = math.pi/2.  # OK
#cones = [[0,0,1],[0.9,0,0.1]]; theta = 0  # test force-closure detection OK
#cones = [[0,0,1],[0.85,0,0.2]]; theta = 0  # OK
#cones = [[0,0,1],[0.9,0,-0.1]]; theta = 0  # OK
#cones = [[0.5,0.5,0.5],[-0.5,0.5,0.5]]; theta = 0.5   # PROBLEM I_MC
#cones = [[0,1,0],[1,0,0]]; theta = 0.1  # OK  IV_p
#cones = [[0,0,1],[0,0,1]]; theta = 0.2  #  OK
cones = [[1,0,0],[0.2,0.6,0.2]]; theta = 0.1  # PROBLEM VI_p  (P is OK...)
cones = [[1,0,0],[0,0,1]]; theta = math.atan(0.5) # PROBLEM (II_MC OK and P OK) -> I_M   moui ...
cones = [[0,0.5,0.5],[0,-0.5,0.5]]; theta = 0.0  # VI_p  problem P
cones = [[0,0.3,0.7],[0,-0.3,0.7]]; theta = math.pi/2
cones = [[0,0,1],[1,0,0]]; theta = 0
cones = [[0.2,0,0.8],[0.2,0.8,0]]; theta = math.pi/2. 
cones = [[0,0.4,0.6],[0,-0.4,0.6]]; theta = 0.0  # VI_p  problem P
cones = [[0,1,0],[0,-0.4,0.6]]; theta = 0.0  # PROBLEM III_p  (P is OK...)
cones = [[0,-0.4,0.6],[0,1,0]]; theta = 0.0  # PROBLEM III_p  (P is OK...)
cones = [[0.5,0.5,0.8],[-0.5,0.5,0.8],[0.5,-0.5,0.8]]; theta = 0  # PROBLEM mu = 1.2 and 0.5
cones = [[-7.42143e-07,1,0], [-1,-6.87383e-07,0]]; theta = 1.41649  # BUG skeleton parkourWalls
cones = [[1,6.38285e-07,-2.73808e-08], [0,-0,1]]; theta = -2.73553  # BUG skeleton parkourWalls
cones = [[-0.358692, 0.908778, -0.213219]]; theta = 2.84719

origin = [0,0,0]
mu = 1.2; coneURDFName = "friction_cone2"

t = rbprmBuilder.convexConePlaneIntersection (len(cones), cones, theta, mu)

# Display:
black = [0.1,0.1,0.1,1]; red = [1,0,0,1]; blue = [0,0,1,1]; green = [0,1,0,1]; planeThetaColor = [0.7,0.2,0.2,0.2]
logID = getNewestLogID ()
CC2D_dir = t [1:4]; phi_CC = t [0]

#plotThetaPlaneBis (origin, theta, 1, r, "thetaPlane", planeThetaColor)
#q = q_0 [::]; q [0] = 0; q [len(q_0)-4:len(q_0)-1] = cones[0]; plotCone (q, ps, r, "testCone", coneURDFName)
plotConvexConeInters (ps, r, origin, CC2D_dir, cones, "CC_center", black, 0.02, "CC_dir", "contactCones", coneURDFName)
#plotFrame (r, "frame", origin, 0.2)

lineParsedBorders = "M_border = ("
lineEndBorders = "END of border points ---"
pointsIntersBorders = plotLogConvexConeInters (r, logID, lineParsedBorders, lineEndBorders, "IntersPointBorders_", 0.021, blue)
plotStraightLines_OM (origin, pointsIntersBorders, r, "CC_borderLine", blue)

lineParsed = "M_i("
lineEnd = "END of intersection points ---"
pointsInters = plotLogConvexConeInters (r, logID, lineParsed, lineEnd, "IntersPoint_", 0.02, red)
plotStraightLines_OM (origin, pointsInters, r, "CC_intersLine", red) # overlay the borderLines if plotted before them

#plotConvexConeInterpolations (ps, r, origin, cones, 0.2, "CC_interpolation", coneURDFName)  # NOT the convex-cone, but a shape included in it !

#r.startCapture("convexConeIntersPlaneTheta1","png") ; r.stopCapture()


""" # Plot P points to check
pointtest = [0.5, -0.5547, 0.83205];
pointtest = [0.5, 1, 0];
pointtest = [1, 1-0.5547, 0.83205];
sphName = "test"+str(pointtest[0])+str(pointtest[1])+str(pointtest[2]);
lineName = "testline"+str(pointtest[0])+str(pointtest[1])+str(pointtest[2])
plotSphere (pointtest, r, sphName, green, 0.03)
plotStraightLines_OM (origin, [pointtest], r, lineName, green)

"""
