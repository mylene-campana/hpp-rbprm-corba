#/usr/bin/env python

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

import quaternion as quat

packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "lamp"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

ps = ProblemSolver( fullBody )
r = Viewer (ps)
q_0 = fullBody.getCurrentConfig (); r(q_0)


nbSamples = 20000
cType = "_6_DOF"
x = 0.05 # contact surface width
y = 0.05 # contact surface length
offset = [0,0,0]
normal = [0,0,1]

LegId = 'foot'
Leg = 'ThighJoint'
foot = 'LampFootSphere'
fullBody.addLimb(LegId, Leg, foot, offset, normal, x, y, nbSamples, "EFORT_Normal", 0.01, cType)
print("Limbs added to fullbody")


q_0 = fullBody.getCurrentConfig ()
fullBody.createOctreeBoxes(r.client.gui, r.windowId, LegId, q_0, [1,1,0.3,0.4])
fullBody.draw (q_0, r) # display robot with octrees

"""
def printEffPosition(limbId, nbSamples):
    limit = nbSamples-1;
    f1=open('./../../data/roms/lamp/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()


printEffPosition(LegId, nbSamples)
"""


#to generate .obj from .erom with Matlab scripts
#effectorRomToObj('../../data/roms/lamp/foot.erom', '../../data/roms/lamp/foot.obj')

# Then Decimate (object/Modifiers) the .obj in Blender and save it as .stl
# Decimate decrease the too high number of facets. (0.1 or 0.2)
# rotations: x -90 deg

"""
frameGroupName = "frame"
framePosition = [0,0,0]
ampl = 0.6
r.client.gui.createGroup (frameGroupName)
x = framePosition [0]; y = framePosition [1]; z = framePosition [2];
r.client.gui.addLine('frame1',[x,y,z], [x+ampl,y,z],[1,0,0,1])
r.client.gui.addToGroup ('frame1', frameGroupName)
r.client.gui.addLine('frame2',[x,y,z], [x,y+ampl,z],[0,1,0,1])
r.client.gui.addToGroup ("frame2", frameGroupName)
r.client.gui.addLine('frame3',[x,y,z], [x,y,z+ampl],[0,0,1,1])
r.client.gui.addToGroup ('frame3', frameGroupName)
r.client.gui.addSceneToWindow(frameGroupName,r.windowId)

q = q_0[::]
q [fullBody.rankInConfiguration ['ShankJoint']] = 0.5;r(q)

#r.startCapture("lampRom","png") ; r.stopCapture()

"""

