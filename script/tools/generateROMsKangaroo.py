#/usr/bin/env python

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from viewer_library import *
import quaternion as quat

packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "kangaroo"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)


ps = ProblemSolver( fullBody )
r = Viewer (ps)
q_0 = fullBody.getCurrentConfig (); r(q_0)

# Verify that normals will be correct
#cl = fullBody.client.basic
#plotJointFrame (r, cl, q_0, "KangarooRFootSphere", 0.15); plotJointFrame (r, cl, q_0, "KangarooLFootSphere", 0.15)
#plotJointFrame (r, cl, q_0, "KangarooRHandSphere", 0.15); plotJointFrame (r, cl, q_0, "KangarooLHandSphere", 0.15)

nbSamples = 10000
#nbSamples = 2000 # for verification
x = 0.03 # contact surface width
y = 0.04 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,1]

rLegId = 'rfoot'
rLeg = 'RThigh_rx'
rfoot = 'KangarooRFootSphere'
rLegNormal = [0,0,1]
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],rLegNormal, x, y, nbSamples, "EFORT", 0.01)

lLegId = 'lfoot'
lLeg = 'LThigh_rx'
lfoot = 'KangarooLFootSphere'
lLegNormal = [0,0,1]
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],lLegNormal, x, y, nbSamples, "EFORT", 0.01)


fullBody.createOctreeBoxes(r.client.gui, r.windowId, rLegId, q_0, [1,1,0.3,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, lLegId, q_0, [1,1,0.3,0.4])

fullBody.draw (q_0, r) # display robot with octrees

"""
def printEffPosition(limbId, nbSamples): 
    limit = nbSamples-1;
    f1=open('../../data/roms/kangaroo/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()


printEffPosition(rLegId, nbSamples)
printEffPosition(lLegId, nbSamples)
"""

#to generate .obj from .erom with Matlab scripts 
"""
effectorRomToObj('../../data/roms/kangaroo/rfoot.erom', '../../data/roms/kangaroo/rfoot_rom.obj')
effectorRomToObj('../../data/roms/kangaroo/lfoot.erom', '../../data/roms/kangaroo/lfoot_rom.obj')
"""

# Then Decimate (object/Modifiers) the .obj in Blender and save it as .stl
# Decimate decrease the too high number of facets.
# Rotation corrections in Blender (in degrees): -90x for feet, 90y+90x for hands

"""
plotFrame (r, 'frame_group', [0,0,0], 0.6)

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

r.client.gui.setVisibility('kangaroo/torso_lhand_rom',"OFF")

q = q_0 [::]
"""
