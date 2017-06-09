#/usr/bin/env python
# Author: Mylene Campana (mcampana@laas.fr), 2016

# Script which goes with hpp-rbprm-corba package.
# Functions library to display graphic things (trajectory, cone, frame...) on the Gepetto-viewer
# And functions to write data which will be parsed by Blender

from __future__ import division
import numpy as np
import math
import os
import glob

Pi = math.pi

# --------------------------------------------------------------------#

# Compute the rotation matrix from the given quaternion (list)
def quatToRotationMatrix(q):
    rm=np.zeros((3,3))
    rm[0,0]=1-2*(q[2]**2+q[3]**2)
    rm[0,1]=2*q[1]*q[2]-2*q[0]*q[3]
    rm[0,2]=2*q[1]*q[3]+2*q[0]*q[2]
    rm[1,0]=2*q[1]*q[2]+2*q[0]*q[3]
    rm[1,1]=1-2*(q[1]**2+q[3]**2)
    rm[1,2]=2*q[2]*q[3]-2*q[0]*q[1]
    rm[2,0]=2*q[1]*q[3]-2*q[0]*q[2]
    rm[2,1]=2*q[2]*q[3]+2*q[0]*q[1]
    rm[2,2]=1-2*(q[1]**2+q[2]**2)
    return rm

# --------------------------------------------------------------------#

# Normalize the dir part of the configuration (dir = direction of cone)
## Parameters:
# q: given configuration
# robot: the robot
def normalizeDir (q, robot):
    q_out = q[::] # copy
    index = robot.getConfigSize () - 3
    N_norm = math.sqrt (q [index]**2 + q [index+1]**2 + q [index+2]**2)
    q_out [index] = q [index]/N_norm
    q_out [index+1] = q [index+1]/N_norm
    q_out [index+2] = q [index+2]/N_norm
    return q_out;

# --------------------------------------------------------------------#

## Plot whole path in viewer (in blue) WARNING: NOT ADAPTED FOR PARABOLA ##
# For parabola, prefrer method plotSampleSubPath
## Parameters:
# ps: Problem Solver
# nPath: path number
# r: viewer server
# lineNamePrefix: string prefix used for line name
# dt: step time
def plotPath (ps, nPath, r, lineNamePrefix, dt):
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        lineName = lineNamePrefix+str(t)
        cl = rbprmBuilder.client.basic
        r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],cl.problem.configAtParam(nPath, t)[2]],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],cl.problem.configAtParam(nPath, t+dt)[2]],[0,0.3,1,1])
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot global frame (in white, x darker) ##
## Parameters:
# r: viewer server
# frameGroupName: string used for group name, allowing to plot several frames
# framePosition: the [x, y, z] absolute position of the frame
# ampl: amplitude of the frame axis
def plotFrame (r, frameGroupName, framePosition, ampl):
    x = framePosition [0]; y = framePosition [1]; z = framePosition [2];
    r.client.gui.addLine(frameGroupName+'frame1',[x,y,z], [x+ampl,y,z],[1,0,0,1])
    r.client.gui.addToGroup (frameGroupName+'frame1', r.sceneName)
    r.client.gui.addLine(frameGroupName+'frame2',[x,y,z], [x,y+ampl,z],[0,1,0,1])
    r.client.gui.addToGroup (frameGroupName+"frame2", r.sceneName)
    r.client.gui.addLine(frameGroupName+'frame3',[x,y,z], [x,y,z+ampl],[0,0,1,1])
    r.client.gui.addToGroup (frameGroupName+'frame3', r.sceneName)
    #r.client.gui.removeFromGroup('frame1',r.sceneName) # remove duplicata in sceneName NOT WORKING
    #r.client.gui.removeFromGroup('frame2',r.sceneName)
    #r.client.gui.removeFromGroup('frame3',r.sceneName)

# --------------------------------------------------------------------#

## Plot cone at each waypoint of the path ##
## Parameters:
# ps: Problem Solver
# nPath: path number
# r: viewer server
# coneGroupName: string used for cone group name ('cone_wp_group')
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
# To avoid problem with cone names in Blender, use also "friction_cone_WP"...
def plotConeWaypoints (ps, nPath, r, coneGroupName, coneURDFname, skip=1):
    wp = ps.getWaypoints (nPath)
    r.client.gui.createGroup (coneGroupName)
    for i in np.arange(1, len(wp)-1, skip): # avoid (re-)plot start and goal
        index = int(i)
        coneName = coneGroupName+'_'+"cone_"+str(index)
        plotCone (wp[index], ps, r, coneName, coneURDFname)
        r.client.gui.addToGroup (coneName, coneGroupName) # visually not needed, but names for Blender ?
    r.client.gui.addSceneToWindow(coneGroupName,r.windowId)


# --------------------------------------------------------------------#

## Plot cone ##
## Parameters:
# ps: Problem Solver
# q: configuration of cone (position, orientation)
# r: viewer server
# coneName: string used for cone name (e.g. "cone_wp0/this_cone")
# coneURDFname: "friction_cone" (mu = 0.5), "friction_cone2" (mu = 1.2)
# coneURDFname: "friction_cone06" (mu = 0.6), "friction_cone1" (mu = 1.)
# To avoid problem with cone names in Blender, use also "friction_cone_SG"...
def plotCone (q, ps, r, coneName, coneURDFname):
    robot = ps.robot # rbprmBuilder or fullBody
    cSize = robot.client.basic.robot.getConfigSize() # with Extra-configs
    ecSize = robot.client.basic.robot.getExtraConfigSize()
    if (len(q) < 6):
        print ('config must be at least of size 6 (position + extra-config normal)')
        return 0
    qCone = [0]*7
    qCone [0:3] = q [0:3]
    if (len(q) <= 6):
        normal = q [len(q) - 3 : len(q)] # 3 last values
    else:
        normal = q [len(q) - ecSize : len(q) - 1] # 3 before the last value
    print normal
    quat = robot.client.rbprm.rbprm.computeOrientationQuat (normal, 0, "") # rotate cone along normal direction
    qCone [3:7] = quat
    print qCone
    #r.loadObstacleModel ("animals_description",coneURDFname,coneName)
    r.loadObstacleModel ("hpp-rbprm-corba",coneURDFname,coneName)
    r.client.gui.applyConfiguration (coneName, qCone[0:7])
    r.client.gui.refresh ()
#Could be:
#gui.addMesh (coneName,"/local/mcampana/devel/hpp/src/animals_description/meshes/cone2.dae")
#gui.applyConfiguration (coneName, q11[0:7]); gui.addToGroup (coneName, r.sceneName)
#gui.refresh ()

# --------------------------------------------------------------------#

## Plot straight line ##
# Uses: plot cone direction or cone - plane_theta intersection
## Parameters:
# vector: direction vector
# pos: origin-position of straight line
# r: viewer server
# lineNamePrefix: string prefix used for line name
def plotStraightLine (vector, pos, r, lineNamePrefix):
    x0 = pos[0]
    y0 = pos[1]
    z0 = pos[2]
    x = vector[0]
    y = vector[1]
    z = vector[2]
    lineName=lineNamePrefix+"straight"
    r.client.gui.addLine(lineName,[x0,y0,z0], [x0+x,y0+y,z0+z],[1,0.3,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot straight lines ##
# Uses: plot cone direction or cone - plane_theta intersection
## Parameters:
# pos: origin-position of straight line
# vectors: list of direction vectors
# r: viewer server
# lineNamePrefix: string prefix used for line name
# color: osg-color of the curve (e.g. [0,1,0,1])
def plotStraightLines (pos, vectors, r, lineNamePrefix, color):
    x0 = pos[0]
    y0 = pos[1]
    z0 = pos[2]
    for i in range (0,len(vectors)):
        x = vectors [i][0]
        y = vectors [i][1]
        z = vectors [i][2]
        lineName=lineNamePrefix+"straightVector_"+str(i)
        r.client.gui.addLine(lineName,[x0,y0,z0], [x0+x,y0+y,z0+z],color)
        r.client.gui.addToGroup (lineName, r.sceneName)


# --------------------------------------------------------------------#

## Plot straight lines between origin and several points ##
## Parameters:
# origin: origin-position of straight line
# points: M points
# r: viewer server
# lineNamePrefix: string prefix used for line name
def plotStraightLines_OM (origin, points, r, lineNamePrefix, color):
    for i in range(0,len(points)):
        lineName=lineNamePrefix + "_StraightLines_OM_" + str(i)
        r.client.gui.addLine(lineName, origin[0:3], points[i][0:3], color)
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot plane theta ##
# (with two triangles, last pos not taken in addSquareFace)
## Parameters:
# q1: initial configuration
# q2: final configuration
# r : viewer server
# lineNamePrefix: string prefix used for line name
def plotThetaPlane (q1, q2, r, lineNamePrefix, planeColor):
    pos1 = [q1 [0], q1 [1], q1 [2]+5]
    pos2 = [q2 [0], q2 [1], q2 [2]-5]
    pos3 = [q1 [0], q1 [1], q1 [2]-5]
    pos4 = [q2 [0], q2 [1], q2 [2]+5]
    r.client.gui.addSquareFace (lineNamePrefix, pos1, pos2, pos4, pos4, planeColor)
    r.client.gui.addToGroup (lineNamePrefix, r.sceneName)
    lineNamePrefix_otherPlane = lineNamePrefix+"otherPlane"
    r.client.gui.addSquareFace (lineNamePrefix_otherPlane, pos1, pos3, pos2, pos2, planeColor)
    r.client.gui.addToGroup (lineNamePrefix_otherPlane, r.sceneName)

# --------------------------------------------------------------------#

## Plot plane theta ##
# (with two triangles, last pos not taken in addSquareFace)
## Parameters:
# q1: initial configuration
# q2: final configuration
# r : viewer server
# lineNamePrefix: string prefix used for line name
def plotThetaPlaneBis (origin, theta, length, r, lineNamePrefix, planeColor):
    x0 = origin [0]; y0 = origin [1]; z0 = origin [2]; 
    pos1 = [x0 + length*math.cos(theta), y0 + length*math.sin(theta), length]
    pos2 = [x0 + length*math.cos(theta), y0 + length*math.sin(theta), -length]
    pos3 = [x0 - length*math.cos(theta), y0 - length*math.sin(theta), -length]
    pos4 = [x0 - length*math.cos(theta), y0 - length*math.sin(theta), length]
    r.client.gui.addSquareFace (lineNamePrefix+"bis", pos1, pos2, pos3, pos3, planeColor) # fourth point ignored
    r.client.gui.addToGroup (lineNamePrefix+"bis", r.sceneName)
    lineNamePrefix_otherPlane = lineNamePrefix+"otherPlane"+"bis"
    r.client.gui.addSquareFace (lineNamePrefix_otherPlane, pos1, pos3, pos4, pos4, planeColor)
    r.client.gui.addToGroup (lineNamePrefix_otherPlane, r.sceneName)

# --------------------------------------------------------------------#

## Shoot random config, normalize 'dir' part and plot cone ##
## Parameters:
# ps: Problem Solver
# r: viewer server
# mu: cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def shootNormPlot (ps, r, mu, ampl, lineNamePrefix):
    robot = ps.robot
    q = robot.shootRandomConfig ()
    print q
    q = normalizeDir (q, robot)
    plotCone (q, ps, r, mu, ampl, lineNamePrefix)
    index = robot.getConfigSize () - 4
    plotStraightLine ([q [index],q [index+1],q [index+2]], q, r, lineNamePrefix+"normale")
    return q

# --------------------------------------------------------------------#

## Plot path choosing the number of samples by subpath ##
## Parameters:
# rbprmBuilder: rbprmBuilder instance (hpp.corbaserver.rbprm.rbprmbuilder)
# r: viewer server
# nPath: path number
# NbPointsPerSubPath: number of sampled points per subpath (each parabola)
# curvePrefix: string prefix used for curve name
# curveColor: osg-color of the curve (e.g. [0,1,0,1])
def plotSampleSubPath (ps, r, nPath, NbPointsPerSubPath, curvePrefix, curveColor):
    plotSampleConfigs = ps.sampleSubPath(nPath, NbPointsPerSubPath)
    pointsCurv = []
    for i in range(0, len(plotSampleConfigs)):
        pointsCurv.append ([plotSampleConfigs [i][0], plotSampleConfigs [i][1], plotSampleConfigs [i][2]])
    
    r.client.gui.addCurve (curvePrefix, pointsCurv, curveColor)
    r.client.gui.addToGroup (curvePrefix, r.sceneName)
    return plotSampleConfigs

# --------------------------------------------------------------------#

## Return contact position (projected on nearest obstacle) ##
# WARNING: assuming that q has been projected and contains "good" orientation and normal
# (if one wants to plot the cone at the contact...)
## Parameters:
# q: CoM configuration (not at the contact)
# ps: Problem Solver
# r: viewer server
def contactPosition (q, ps, r):
    robot = ps.robot
    qConeContact = q[::] # at least for orientation
    index = cl.robot.getConfigSize () - cl.robot.getExtraConfigSize ()
    n = np.array([q[index], q[index+1], q[index+2]]) # normal
    robot.setCurrentConfig (q)
    res = robot.distancesToCollision ()
    pCoM = q[0:3]
    pj = res[4][np.argmin(res[0])] # point on obstacle surface
    distContactCoM = np.dot(np.array(pCoM)-np.array(pj),n)
    qConeContact[0:3] = (np.array(q[0:3]) - distContactCoM*n).tolist ()
    return qConeContact

# --------------------------------------------------------------------#

## Add light in viewer scene ##
# giving the light configuration (position) and name
# "r.client.gui.removeFromGroup (lightName, r.sceneName)" to remove light 
# (if think it just removes the object carrying the light, not the light effect)
## Parameters:
# r: viewer server
# q: light configuration (list)
# lightName: light name (string)
def addLight (r, q, lightName):
    r.client.gui.addLight (lightName, r.windowId, 0.0001, [0.9,0.9,0.9,1])
    r.client.gui.addToGroup (lightName, r.sceneName)
    r.client.gui.applyConfiguration (lightName, q)
    r.client.gui.refresh ()

# --------------------------------------------------------------------#

## Plot sphere ##
# Example (plot small green sphere) "plotSphere (q, r, sphereName, [0,1,0,1], 0.02)"
## Parameters:
# q: configuration of cone (position, orientation)
# r: viewer server
# sphereName: string suffix used for cone name
# sphereColor: color of sphere
# sphereSize: size of sphere
def plotSphere (q, r, sphereName, sphereColor, sphereSize):
    r.client.gui.addSphere (sphereName,sphereSize,sphereColor)
    if len(q) == 3:
        q[3:7] = [1,0,0,0]
    r.client.gui.applyConfiguration (sphereName, q[0:7])
    r.client.gui.addToGroup (sphereName, r.sceneName)
    r.client.gui.refresh ()

# --------------------------------------------------------------------#

## Plot sphere at each waypoint of the path ##
## Parameters:
# ps: Problem Solver
# nPath: path number
# r: viewer server
# sphereGroupName: string used for sphere group name
# sphereColor: color of sphere
# sphereSize: size of sphere
def plotSpheresWaypoints (ps, nPath, r, sphereGroupName, sphereColor, sphereSize):
    wp = ps.getWaypoints (nPath)
    r.client.gui.createGroup (sphereGroupName)
    for i in np.arange(1, len(wp)-1, 1): # avoid (re-)plot start and goal
        sphereName = sphereGroupName+'_'+"sphere_"+str(i)
        plotSphere (wp[i][0:7], r, sphereName, sphereColor, sphereSize)
        r.client.gui.addToGroup (sphereName, sphereGroupName)
        #r.client.gui.removeFromGroup(sphereName,r.sceneName) # remove duplicata in sceneName NOT WORKING
    r.client.gui.addSceneToWindow(sphereGroupName,r.windowId)

# --------------------------------------------------------------------#

## Plot cone at each node of the roadmap ##
## Parameters:
# ps: Problem Solver
# r: viewer server
# coneGroupName: string used for cone group name
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
def plotConesRoadmap (ps, r, coneGroupName, coneURDFname):
    RM_nodes = ps.nodes ()
    r.client.gui.createGroup (coneGroupName)
    for i in range(2,len(RM_nodes)): # avoid first nodes
        node_i = RM_nodes [i]
        coneName_i = "cone_RM_"+str(i)
        plotCone (node_i, rbprmBuilder, r, coneName_i, coneURDFname)
        r.client.gui.addToGroup (coneName_i, coneGroupName)
    r.client.gui.addSceneToWindow(coneGroupName,r.windowId)

# --------------------------------------------------------------------#

## Plot each edge of the roadmap from sampled points ##
## Parameters:
# ps: Problem Solver
# r: viewer server
# edgeGroupName: edge group name
def plotEdgesRoadmap (ps, r, edgeGroupName, nbPointsPerEdge, curveColor):
    gui = r.client.gui
    gui.createGroup (edgeGroupName)
    numEdges = ps.numberEdges ()
    for k in range (0,numEdges,2): # one over two (avoid reverse edge)
        samplesEdge_k = ps.edgeToSampledPathVector (k, nbPointsPerEdge)
        pointsEdge_k = []
        for i in range(0, len(samplesEdge_k)):
            pointsEdge_k.append ([samplesEdge_k [i][0], samplesEdge_k [i][1], samplesEdge_k [i][2]])
        nameEdge_k = edgeGroupName+'edge_'+str(k)
        r.client.gui.addCurve (nameEdge_k, pointsEdge_k, curveColor)
        r.client.gui.addToGroup (nameEdge_k, edgeGroupName)
    
    r.client.gui.addSceneToWindow(edgeGroupName,r.windowId)


# --------------------------------------------------------------------#

## Plot path for given point in body DOF ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# cl: corbaserver client
# robot: robot instance
# dt: time step to sample the displayed path
# nPath: number of path on which configurations will be computed
# jointName: name of joint whose center will be displayed
# pathName: name of displayed path
# pathColor: osg-color of displayed path (e.g. [1,0.3,0,1])
def plotPointJointPath (r, ps, dt, nPath, jointName, pointInJoint, pathName, pathColor):
    robot = ps.robot
    points = []
    for t in np.arange(0., ps.pathLength(nPath), dt):
        config = ps.configAtParam(nPath, t)
        robot.setCurrentConfig (config)
        jointPosition = robot.getJointPosition (jointName)
        pointPosition = robot.computeGlobalPosition (jointPosition, pointInJoint)
        #print pointPosition
        points.append ([pointPosition[0],pointPosition[1],pointPosition[2]])
    
    r.client.gui.addCurve (pathName,points, pathColor)
    r.client.gui.addToGroup (pathName, r.sceneName)

# --------------------------------------------------------------------#

## Plot frame for given joint in given configuration ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# ps: Problem Solver
# q: configuration
# jointName: name of joint whose center will be displayed
# ampl: length of the frame lines
def plotJointFrame (r, ps, q, jointName, ampl, linePrefixName):
    robot = ps.robot
    classicRobot = ps.robot.client.basic.robot
    robot.setCurrentConfig (q)
    jointGlobTransf = robot.getJointPosition (jointName)
    frameOrigin = classicRobot.computeGlobalPosition (jointGlobTransf, [0,0,0])
    framePositionX = classicRobot.computeGlobalPosition (jointGlobTransf, [ampl,0,0])
    framePositionY = classicRobot.computeGlobalPosition (jointGlobTransf, [0,ampl,0])
    framePositionZ = classicRobot.computeGlobalPosition (jointGlobTransf, [0,0,ampl])
    r.client.gui.addLine(linePrefixName+'localFrameX'+jointName, frameOrigin, framePositionX,[1,0,0,1])
    r.client.gui.addLine(linePrefixName+'localFrameY'+jointName, frameOrigin, framePositionY,[0,1,0,1])
    r.client.gui.addLine(linePrefixName+'localFrameZ'+jointName, frameOrigin, framePositionZ,[0,0,1,1])
    r.client.gui.addToGroup(linePrefixName+'localFrameX'+jointName,r.sceneName)
    r.client.gui.addToGroup(linePrefixName+'localFrameY'+jointName,r.sceneName)
    r.client.gui.addToGroup(linePrefixName+'localFrameZ'+jointName,r.sceneName)

# --------------------------------------------------------------------#

## Plot frame for given joint in given configuration ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# ps: Problem Solver
# q: configuration
# jointName: name of joint whose center will be displayed
# ampl: length of the frame lines
# lineColor: osg-color of displayed line (e.g. [1,0.3,0,1])
def plotVectorInJointFrame (r, ps, q, jointName, vector, lineColor, linePrefixName):
    robot = ps.robot
    classicRobot = ps.robot.client.basic.robot
    robot.setCurrentConfig (q)
    jointPosition = robot.getJointPosition (jointName)
    framePosition = classicRobot.computeGlobalPosition (jointPosition, [0,0,0]) 
    extrPosition = classicRobot.computeGlobalPosition (jointPosition, [vector[0],vector[1],vector[2]])
    lineName = linePrefixName+'localLineIn'+jointName
    r.client.gui.addLine(lineName,[framePosition[0],framePosition[1],framePosition[2]], [extrPosition[0],extrPosition[1],extrPosition[2]],lineColor)
    r.client.gui.addToGroup(lineName,r.sceneName)


# --------------------------------------------------------------------#

## Plot path for given body ##
# Plot path of the given joint frame center with the viewer 'curve' function
## Parameters:
# r: viewer server
# ps: Problem Solver
# dt: time step to sample the displayed path
# nPath: number of path on which configurations will be computed
# bodyName: name of body whose center will be displayed (do not forget '_0')
# pathName: name of displayed path
# pathColor: osg-color of displayed path (e.g. [1,0.3,0,1])
def plotBodyCurvPath (r, ps, dt, nPath, bodyName, pathName, pathColor):
    robot = ps.robot
    points = []
    for t in np.arange(0., ps.pathLength(nPath), dt):
        config = ps.configAtParam(nPath, t)
        robot.setCurrentConfig (config)
        bodyPosition = robot.getObjectPosition (bodyName)
        print bodyPosition
        points.append ([bodyPosition[0],bodyPosition[1],bodyPosition[2]])
    
    r.client.gui.addCurve (pathName,points, pathColor)
    r.client.gui.addToGroup (pathName, r.sceneName)


# --------------------------------------------------------------------#

## Plot GIWC ##
## Parameters:
# Vmatrix: V-representation of GIWC, previously obtained from rbprmBuilder.computeConfigGIWC (config)
# q: current configuration TODO not sure if need the CoM one...
# r: viewer server
# giwcId: Id (for name) of giwc
# color: osg-color of displayed giwc (e.g. [0,0.9,0.1,1])
def plotGIWC (q, Vmatrix, r, giwcId, color):
    Vrows = len(Vmatrix[0]); Vcols = len(Vmatrix)
    origin = q[0:3] # TODO use also q's quaternion ?
    origin.append(1); origin.append(0); origin.append(0); origin.append(0); # quaternion
    plotSphere (origin, r, "giwc_" + str(giwcId) + "_origin", color, 0.05)
    for i in range (0,Vrows):
        lineName = "giwc_" + str(giwcId) + "_line_" + str(i)
        pos_i = [Vmatrix[i][0],Vmatrix[i][1],Vmatrix[i][2]]
        rot_i = [Vmatrix[i][3],Vmatrix[i][4],Vmatrix[i][5]]
        # TODO: compute point_i = origin + ... rot_i*pos_i ...
        #r.client.gui.addLine(lineName,[Vmatrix[i][0],Vmatrix[i][1],Vmatrix[i][2]], [Vmatrix[i][3],Vmatrix[i][4],Vmatrix[i][5]],color)
        r.client.gui.addLine(lineName,[origin[0],origin[1],origin[2]], [rot_i[0],rot_i[1],rot_i[2]],color)
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

##  Direct Plot convex-cone and plane_theta intersection, given list of contact-cones and origin ##
## Parameters:
# origin: unique position of cone origins
# ps: Problem Solver
# r: viewer server
# origin: unique position of cone origins
# cones: list of contact-cone directions
# CC2D_dir: direction of the 2D-convex-cone computed by the C++binded intersection function "convexConePlaneIntersection"
# color: osg-color of displayed lines and spheres (e.g. [0,0.9,0.1,1])
# sphereSize: size of the origin sphere
# lineNamePrefix: string used for line group name
# coneNamePrefix: string used for cone group name
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
def plotConvexConeInters (ps, r, origin, CC2D_dir, cones, sphereName, color, sphereSize, lineNamePrefix, coneNamePrefix, coneURDFname):
	x0 = origin [0]; y0 = origin [1]; z0 = origin [2]; originConfig = [x0,y0,z0,1,0,0,0];
        x = CC2D_dir [0]; y = CC2D_dir [1]; z = CC2D_dir [2]; 
        Ncones = len(cones)
        Nconfigs = ps.robot.getConfigSize ();
        ECSindex = Nconfigs - ps.robot.client.basic.robot.getExtraConfigSize()
        qConeInit = Nconfigs*[0]; qConeInit [0:3] = origin; qConeInit [3:7] = [1,0,0,0];
	plotSphere (originConfig, r, sphereName, color, sphereSize)
	r.client.gui.addLine(lineNamePrefix+"_lineConvexConeInters",[x0,y0,z0], [x0+x,y0+y,z0+z],color)
	r.client.gui.addToGroup (lineNamePrefix+"_lineConvexConeInters", r.sceneName)
        for i in range (0,Ncones):
            coneName_i =  coneNamePrefix + str(i)
            qCone_i = qConeInit [::]
            qCone_i [ECSindex:ECSindex+3] = cones [i] # WARNING: not normalized
            plotCone (qCone_i, ps, r, coneName_i, coneURDFname)

# --------------------------------------------------------------------#

## Plot cones with dir = (1 - alpha)*n1 + alpha*n2 (for each zone between two contact-cones)##
## Parameters:
# origin: unique position of cone origins
# ps: Problem Solver
# r: viewer server
# origin: unique position of cone origins
# cones: list of contact-cone directions
# alphaStep: step between two interpolated cones
# coneNamePrefix: string used for cone group name
# coneURDFname: "friction_cone" (mu = 0.5) or "friction_cone2" (mu = 1.2)
def plotConvexConeInterpolations (ps, r, origin, cones, alphaStep, coneNamePrefix, coneURDFname):
	x0 = origin [0]; y0 = origin [1]; z0 = origin [2]; originConfig = [x0,y0,z0,1,0,0,0];
        Ncones = len(cones)
        Nconfigs = ps.robot.getConfigSize ();
        ECSindex = Nconfigs - ps.robot.client.basic.robot.getExtraConfigSize()
        qConeInit = Nconfigs*[0]; qConeInit [0:3] = origin; qConeInit [3:7] = [1,0,0,0];
        for i in range (0,Ncones):
            for j in range(i + 1,Ncones):
                for alpha in np.arange(alphaStep, 1.-alphaStep, alphaStep):
                    coneName_ij_alpha =  coneNamePrefix + str(i) + str(j) + str(alpha)
                    qCone_ij_alpha = qConeInit [::]
                    dir_ij_alpha = (np.array(cones [i])*(1-alpha) + np.array(cones [j])*alpha).tolist ()
                    qCone_ij_alpha [ECSindex:ECSindex+3] = dir_ij_alpha
                    plotCone (qCone_ij_alpha, ps, r, coneName_ij_alpha, coneURDFname)
	

# --------------------------------------------------------------------#

# Parse (Vec3f vectors) and Plot points. (Used for the convex-cone and plane_theta intersection)
# Before calling this function, the C++binded intersection function "convexConePlaneIntersection" must be called in DEBUG mode
def plotLogConvexConeInters (r, logID, lineParsed, lineEnd, pointPrefix, pointSize, pointColor):
    logFile = "/local/mcampana/devel/hpp/install/var/log/hpp/"
    logLinePrefix = "INFO:/local/mcampana/devel/hpp/src/hpp-rbprm/src/fullbodyBallistic/convex-cone-intersection.cc:"
    skipChar = 5 # number of skipped characters after the end of logLinePrefix "000: "
    l_prefix = len(logLinePrefix)
    l = len(lineParsed)
    l_end = len(lineEnd)
    i = 0
    with open (logFile + "journal." + str(logID) + ".log") as f:
        points = []
        for line in f:
            if line [:l_prefix] == logLinePrefix and line [l_prefix+skipChar:l_prefix+skipChar+l] == lineParsed:
                suffix = line [l_prefix+skipChar+l:]
                st = suffix.strip (')\n') # remove end characters
                sp = st.split () # separate numbers with space
                try:
                    point = map (float, sp) # convert into float
                    points.append (point)
                    pointName = pointPrefix + str(logID) + '_' + str(i)
                    #plotSphere (point, r, pointName, pointColor, pointSize) # cone origin may have changed
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
                    print ("point=%s"%point)
                i = i + 1
            if line [:l_prefix] == logLinePrefix and line [l_prefix+skipChar:l_prefix+skipChar+l_end] == lineEnd:
                break
    return points # list, not array
	
# --------------------------------------------------------------------#

## get the ID (number) of the newest log file ##
def getNewestLogID ():
    journalPrefixName = "/local/mcampana/devel/hpp/install/var/log/hpp/journal."
    newest = max(glob.iglob('/local/mcampana/devel/hpp/install/var/log/hpp/*.log'), key=os.path.getctime)
    logID = newest[len(journalPrefixName):].strip('.log')
    print ("logID= " +str(logID))
    return logID

# --------------------------------------------------------------------#

## Plot contact-cones obtained from rbprmBuilder function ##
def plotContactCones (contactCones, ps, r, coneName, coneURDFname):
    numCones = len(contactCones[0])
    contactConesDirs = contactCones[0]
    contactConesPos = contactCones[1]
    for i in range(0, numCones):
        q = [0]*6
        q[0:3] = contactConesPos [i]
        q[3:6] = contactConesDirs [i]
        print ("q in plotContactCones")
        print q
        plotCone (q, ps, r, coneName + "_contactCones_" + str(i), coneURDFname)

# --------------------------------------------------------------------#
# ----------------------------## BLENDER ##------------------------------------#

## Write Path-motion in Yaml file ##
## Parameters:
# cl: corbaserver client
# fileName: name (string) of the file where samples will be written
def pathToYamlFile (ps, r, fileName, robotName, pathId, goalConfig, dt):
    gui = r.client.gui
    FrameRange = np.arange(0, ps.pathLength(pathId), dt)
    gui.setCaptureTransform (fileName, [robotName])
    for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); ps.robot.setCurrentConfig(q)
        gui.refresh ()
        gui.captureTransform ()
    
    r (goalConfig); ps.robot.setCurrentConfig(goalConfig)
    gui.refresh ()
    gui.captureTransform ()

# --------------------------------------------------------------------#

## Write Path-motion configurations to file ##
## Parameters:
# ps: Problem Solver
# fileName: name (string) of the file where samples will be written 'jointConfigs.txt'
# goalConfig: final config of path
# dt: time step
def pathJointConfigsToFile (ps, r, fileName, pathId, goalConfig, dt):
    robot = ps.robot
    pathToFile = '/local/mcampana/devel/hpp/videos/' # WARNING!
    gui = r.client.gui
    FrameRange = np.arange(0, ps.pathLength(pathId), dt)
    iFrame = 0 # int
    jointNames = robot.getJointNames ()
    rootPosJointName = robot.getJointNames () [0] # xyz position
    rootRotJointName = robot.getJointNames () [1] # so3 rotation
    nbInnerJoints = len(jointNames) - 2
    f = open(fileName,'a')
    print (str(nbInnerJoints))
    f.write(str(nbInnerJoints) + "\n")
    for t in FrameRange:
        print ("Frame " + str(iFrame))
        f.write ("Frame " + str(iFrame) + "\n")
        q = ps.configAtParam (pathId, t)
        ps.robot.setCurrentConfig(q)
        rootPos = robot.getLinkPosition(rootPosJointName)[0:3]
        rootRot = robot.getLinkPosition(rootRotJointName)[3:7]
        print (str(rootPos).strip('[]') + ', ' + str(rootRot).strip('[]'))
        f.write (str(rootPos).strip('[]') + ', ' + str(rootRot).strip('[]') + "\n")
        for jointName in jointNames:
            if (jointName != "base_joint_xyz" and jointName != "base_joint_SO3" ):
                jointNameBlender = jointName
                if jointName[0:2] == 'A_':
                    jointNameBlender = jointName[2:len(jointName)]
                qJoint = q [robot.rankInConfiguration [jointName]]
                f.write (jointNameBlender + " " + str(qJoint) + "\n")
        iFrame = iFrame + 1
    
    f.write ("Frame " + str(iFrame) + "\n")
    q = goalConfig
    ps.robot.setCurrentConfig(q)
    rootPos = robot.getLinkPosition(rootPosJointName)[0:3]
    rootRot = robot.getLinkPosition(rootRotJointName)[3:7]
    print (str(rootPos).strip('[]') + ', ' + str(rootRot).strip('[]'))
    f.write (str(rootPos).strip('[]') + ', ' + str(rootRot).strip('[]') + "\n")
    for jointName in jointNames:
        if (jointName != "base_joint_xyz" and jointName != "base_joint_SO3" ):
            jointNameBlender = jointName
            if jointName[0:2] == 'A_':
                jointNameBlender = jointName[2:len(jointName)]
        qJoint = q [robot.rankInConfiguration [jointName]]
        f.write (jointNameBlender + " " + str(qJoint) + "\n")


# --------------------------------------------------------------------#

## Write Edge samples to text file ##
# because exporting the edges directly in .obj creates BIG files
## Parameters:
# ps: Problem Solver
# fileName: name (string) of the file where samples will be written
# nbPointsPerEdge: number of points on each edge-path
def writeEdgeSamples (ps, fileName, nbPointsPerEdge):
    numEdges = ps.numberEdges ()
    f = open(fileName,'a')
    for k in range (0,numEdges,2):
        print ("Edge number: " + str(k))
        samplesEdge_k = ps.edgeToSampledPathVector (k, nbPointsPerEdge)
        f.write('e'+'\n')
        for i in range(0, len(samplesEdge_k)):
            f.write(str(samplesEdge_k [i]).strip('[]')+'\n') # write point i
    
    f.close()

# --------------------------------------------------------------------#

## Write Path samples to text file ##
# require samples from "pathSamples = plotSampleSubPath (cl, r, pathId, 70, "path0", [0,0,1,1])"
## Parameters:
# pathSamples: sample used to plot solution-path in viewer
# fileName: name (string) of the file where samples will be written
def writePathSamples (pathSamples, fileName):
    numPathSamples = len(pathSamples)
    f = open(fileName,'a')
    for i in range (0,numPathSamples):
        #print ("sample number: " + str(i))
        f.write(str(pathSamples [i]).strip('[]')+'\n') # write point i
    
    f.close()

# --------------------------------------------------------------------#

## Write RM edge and node index associated to solution-path:
# ps: Problem Solver
# fileName: name (string) of the file where samples will be written
def writeSkipList (ps, fileName):
    l1 = ps.getEdgeIndexVector() #[0, 3, 12, 16, 24, 21, 10, 8]
    l2 = ps.problem.getNodeIndexVector() #[0, 0, 6, 15, 17, 18, 13, 14]
    del l1 [0]; del l2 [0]; del l2 [0];
    f = open(fileName,'a')
    f.write(str(l1).strip('[]')+'\n')
    f.write(str(l2).strip('[]')+'\n')
    f.close()

# --------------------------------------------------------------------#

## Write one config to file ##
## Parameters:
# ps: Problem Solver
# fileName: name (string) of the file where samples will be written 'jointConfigs.txt'
# goalConfig: final config of path
# dt: time step
#q = [0, 0, 0, 1, 0,0, 0, 0, 0, 0.2, 0.0, 0.0, 0.0, 0.4, 0.5, 0.7, 0, -0.6, 0.0, 0.0, 0.4, 0.5, 0.7, 0, -0.6, 0.0, 0.0, -0.2, 0.3, -1.9, 1.9,-0.6, 0, -0.2, 0.3, -1.9, 1.9, -0.6, 0]
#pathJointConfigsToFile (ps, r, "spiderman_jointConfigs.txt", q)
def jointConfigsToFile (ps, r, fileName, config):
    robot = ps.robot
    pathToFile = '/local/mcampana/devel/hpp/videos/' # WARNING!
    gui = r.client.gui
    jointNames = robot.getJointNames ()
    rootPosJointName = robot.getJointNames () [0] # xyz position
    rootRotJointName = robot.getJointNames () [1] # so3 rotation
    nbInnerJoints = len(jointNames) - 2
    f = open(pathToFile + fileName,'a')
    print (str(nbInnerJoints))
    f.write(str(nbInnerJoints) + "\n")
    f.write ("Frame " + str(0) + "\n")
    q = config
    ps.robot.setCurrentConfig(q)
    rootPos = robot.getLinkPosition(rootPosJointName)[0:3]
    rootRot = robot.getLinkPosition(rootRotJointName)[3:7]
    print (str(rootPos).strip('[]') + ', ' + str(rootRot).strip('[]'))
    f.write (str(rootPos).strip('[]') + ', ' + str(rootRot).strip('[]') + "\n")
    for jointName in jointNames:
        if (jointName != "base_joint_xyz" and jointName != "base_joint_SO3" ):
            jointNameBlender = jointName
            if jointName[0:2] == 'A_':
                jointNameBlender = jointName[2:len(jointName)];
            qJoint = q [robot.rankInConfiguration [jointName]]
            f.write (jointNameBlender + " " + str(qJoint) + "\n")

