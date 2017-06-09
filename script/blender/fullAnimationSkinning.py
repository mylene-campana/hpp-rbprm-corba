import bpy
from math import *
import yaml #Windaube
"""
Contains all Blender actions to:
  - import initial and final cones, (and create associated spheres ?)
  - import roadmap and path cones-edges,
  - set their materials and visibilities along the animation,
  - import the robot and its path.
User has to fill the parameters (names of files mostly), for now files will have to be in the 
same directory where the script is.
"""

""" Update:
Not supported by Blender Python, or not convenient:
 - Texts (start, goal, roadmap...)
 - Camera poses
"""

# count lines in file
def file_len(fname):
	with open(fname) as f:
		for i, l in enumerate(f):
			pass
	return i + 1


def parseEdgeVector (filename, nbPointsPerEdge):
	lineNB = 0
	listOfEdges = []
	with open (filename) as f:
		lines=f.readlines()
		for line in lines:
			points = []
			lineNB = lineNB + 1
			if line [0] == 'e': # edge beginning
				for i in range(0, nbPointsPerEdge): # parse each point of the edge
					actualLine = lines[lineNB + i]
					st = actualLine.strip ('\n').split (',') # remove end character and separate
					point = list(map (float, st)) # map not subscriptable in Python 3
					points.append (point [0:3])
				listOfEdges.append(points)
	return listOfEdges

def parsePathPoints (filename):
	pathPoints = []
	with open (filename) as f:
		for line in f.readlines():
			st = line.strip ('\n').split (',')
			point = list(map (float, st))
			pathPoints.append (point [0:3])
	return pathPoints

def parseEdgeNodeIndexes (filename, lineNB):
	# lineNB = 0 for edge indexes, 1 for node indexes
	indexes = []
	with open (filename) as f:
		lines = f.readlines()
		line = lines [lineNB]
		st = line.strip ('\n').split (',')
		indexes = list(map (float, st))
	return indexes

def curveToMesh (selected_object):
	#http://gappyfacets.com/2016/03/03/blender-python-snippet-duplicate-object-curves-data-visualization/
	# Select and active only the curve per loop.
	bpy.ops.object.select_all(action='DESELECT')
	selected_object.select = True
	bpy.context.scene.objects.active = selected_object
	# Convert curve into mesh.
	bpy.ops.object.convert(target='MESH', keep_original=False)
	bpy.ops.object.select_all(action='DESELECT')

def plotEdges (listOfEdges, edgeNamePrefix, numPointsPerEdge, mat):
	w = 1 # weight for point plot
	curvedatas = []; objectdatas = []; polylines = []
	numEdges = len(listOfEdges)
	for i in range(numEdges):
		curvedatas.append (bpy.data.curves.new(name='Curve', type='CURVE'))
		curvedatas [i].dimensions = '3D'
		objectdatas.append (bpy.data.objects.new(edgeNamePrefix+str(i), curvedatas [i]))
		objectdatas [i].location = (0,0,0)
		objectdatas [i].data.materials.append (mat)
		bpy.context.scene.objects.link(objectdatas [i])
		polylines.append (curvedatas [i].splines.new('POLY'))
		polylines [i].points.add(numPointsPerEdge-1)
		edge = listOfEdges [i]
		for j in range(numPointsPerEdge):
			polylines[i].points[j].co = (edge [j][0], edge [j][1], edge [j][2], w)
		curveToMesh(objectdatas [i])

def plotPath (pathPoints, pathName, matPath):
	w = 1 # weight for point plot
	curvedatas = []; objectdatas = []; polylines = []
	curvedata =  bpy.data.curves.new(name='Curve', type='CURVE')
	curvedata.dimensions = '3D'
	#curvedata.extrude = 0.001; curvedata.bevel_depth = 0.001
	objectdata = bpy.data.objects.new(pathName, curvedata)
	objectdata.location = (0,0,0)
	objectdata.data.materials.append (matPath)
	bpy.context.scene.objects.link(objectdata)
	polyline = curvedata.splines.new('POLY')
	polyline.points.add(len(pathPoints)-1)
	for i in range(len(pathPoints)):
		polyline.points[i].co = (pathPoints [i][0], pathPoints [i][1], pathPoints [i][2], w)
	curveToMesh(objectdata)

def plotGlobalFrameLine (frameName, location, locOffset, mat):
	w = 1 # weight for point plot
	curvedatas = []; objectdatas = []; polylines = []
	curvedata =  bpy.data.curves.new(name='Curve', type='CURVE')
	curvedata.dimensions = '3D'
	objectdata = bpy.data.objects.new(frameName, curvedata)
	objectdata.location = location
	objectdata.data.materials.append (mat)
	bpy.context.scene.objects.link(objectdata)
	polyline = curvedata.splines.new('POLY')
	polyline.points.add(2)
	#polyline.points[0].co = (location[0], location[1], location[2], w)
	polyline.points[0].co = (locOffset[0], locOffset[1], locOffset[2], w)
	curveToMesh(objectdata)

def plotGlobalFrame (frameName, location):
	locOffsetVal = 0.4
	matExFrame = getOrCreateMaterial ("exFrame", 'WIRE', [1,0,0], 1, True, False, False)
	matEyFrame = getOrCreateMaterial ("eyFrame", 'WIRE', [0,1,0], 1, True, False, False)
	matEzFrame = getOrCreateMaterial ("ezFrame", 'WIRE', [0,0,1], 1, True, False, False)
	plotGlobalFrameLine (frameName+'_ex', location, [locOffsetVal,0,0], matExFrame)
	plotGlobalFrameLine (frameName+'_ey', location, [0,locOffsetVal,0], matEyFrame)
	plotGlobalFrameLine (frameName+'_ez', location, [0,0,locOffsetVal], matEzFrame)

def getOrCreateMaterial (materialName, matType, RGBcolor, alphaTransp, shadeless, traceable, receiveShadows):
	if bpy.data.materials.get(materialName) is not None:
		mat = bpy.data.materials[materialName]
	else:
		mat = bpy.data.materials.new(name=materialName)
	# Set properties
	if (alphaTransp == 1):
		mat.use_transparency = False
	else:
		mat.use_transparency = True
		mat.alpha = alphaTransp
	mat.type = matType # 'SURFACE' or 'WIRE'
	mat.diffuse_color[0:3] = RGBcolor # [0.5,0.5,0.5] or [0,0,1]
	mat.use_shadeless = shadeless # True or False
	mat.use_raytrace = traceable # True or False
	mat.use_shadows = receiveShadows # True or False
	return mat

def tagObjects (bpy):
  taggedObjects = list ()
  for obj in bpy.data.objects:
    taggedObjects.append (obj.name)
  return taggedObjects

def getNonTaggedObjects (taggedObjects):
  return [obj for obj in bpy.data.objects if obj.name not in taggedObjects]

def setParent (children, parent):
  for child in children:
    child.parent = parent

def importDaeRobot (daeFileName, robotName, material): # from jmirabel@laas.fr
	taggedObjects = tagObjects(bpy)
	bpy.ops.wm.collada_import (filepath=daeFileName)
	imported_objects = getNonTaggedObjects (taggedObjects)
	#print(imported_objects)
	bpy.ops.object.empty_add ()
	currentObj = bpy.context.object
	setParent (imported_objects, currentObj)
	currentObj.name = robotName
	currentObj.location = [0.0, 0.0, 0.0]
	currentObj.rotation_euler = [0.0, 0.0, 0.0]
	bpy.data.objects["Sphere"].data.materials [0] = material # "Sphere" hardcoded

def importDaeObjects (daeFileName, objNamePrefix, material):
	taggedObjects = tagObjects(bpy)
	bpy.ops.wm.collada_import (filepath=daeFileName)
	imported_objects = getNonTaggedObjects (taggedObjects)
	#print(imported_objects)
	for impObj in imported_objects:
		if (impObj.name[0:4] == objNamePrefix):
			impObj.data.materials [0] = material

def createSphereMesh (sphereName, spherePose, sphereMat, numSegments, numRings, sphereSize):
	bpy.ops.mesh.primitive_uv_sphere_add(location=spherePose, segments=numSegments, ring_count=numRings, size = sphereSize)
	bpy.context.object.name = sphereName
	bpy.context.object.data.materials.append(sphereMat)

def loadMotionBodies (filename, startFrame): # from stonneau@laas.fr
	#root = bpy.data.objects[rootName]
	#offsetArmatureQuarternion = [root.rotation_quaternion[0],root.rotation_quaternion[1],root.rotation_quaternion[2],root.rotation_quaternion[3]]
	with open (filename) as file:
		data = yaml.load (file)
		for frameId in range (len(data.keys())):
			frameKey = "frame_" + str (frameId)
			objPositions = data[frameKey]
			for objName, pos in objPositions.items ():
				currentObj = bpy.context.scene.objects.get(objName)
				if currentObj:
					currentObj.rotation_mode = 'QUATERNION'
					posF = [float(x) for x in pos]
					currentObj.location = posF[0:3]
					#quatProd = quaternionListProduct (posF[3:7],offsetArmatureQuarternion)
					currentObj.rotation_quaternion = posF[3:7] #quatProd
					currentObj.keyframe_insert (data_path="location", frame=frameId+startFrame)
					currentObj.keyframe_insert (data_path="rotation_quaternion", frame=frameId+startFrame)
				else:
					#print("Unknown object " + objName)
					frameKey
	return len(data.keys())+startFrame

class JointConfiguration(object):
    def __init__(self, name, rotation_euler):
        self.name = name
        self.rotation_euler = rotation_euler

def quaternionListProduct (a, b): # from https://en.wikipedia.org/wiki/Quaternion#Ordered_list_form
	# product of two quaternions expressed as lists
	r1 = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
	r2 = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
	r3 = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
	r4 = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
	return [r1, r2, r3, r4]

def loadMotionArmature (filename, startFrame = 0, reOrientFrames = True, rotationOrder = 'ZXY', armatureName = "Armature"):
	print ("loadMotionArmature")
	armature = bpy.data.objects[armatureName]
	offsetArmatureQuarternion = [armature.rotation_quaternion[0],armature.rotation_quaternion[1],armature.rotation_quaternion[2],armature.rotation_quaternion[3]]
	totalLineNumber = file_len(filename)
	totalFrameNumber = 0
	#print ("totalLineNumber= " + str(totalLineNumber))
	with open (filename) as f:
		lines=f.readlines() # lines [0] = nbInnerJoints ; lines [1] = freeflyerName
		nbInnerJoints = int (lines[0])
		#print ("nbInnerJoints= " + str(nbInnerJoints))
		lineNB = 0
		while (lineNB + 1 < totalLineNumber):
			lineNB = lineNB + 1
			line = lines [lineNB] # frame number line
			if line [0:6] == 'Frame ':
				st = line.split(' ')
				frameId = int(st [1])
				print ("frameId= " + str(frameId))
				totalFrameNumber = totalFrameNumber + 1
				lineNB = lineNB + 1
				line = lines [lineNB]
				st = line.strip ('\n').split (',')
				freeflyerPosRot = list(map (float, st))
				#freeflyerBone = armature.pose.bones[freeflyerName] # not used for now
				armature.location = freeflyerPosRot[0:3]
				#print ("freeflyerPosRot= " + str(freeflyerPosRot))
				quatProd = quaternionListProduct (freeflyerPosRot[3:7],offsetArmatureQuarternion)
				armature.rotation_quaternion = quatProd
				armature.keyframe_insert (data_path="location", frame=frameId+startFrame)
				armature.keyframe_insert (data_path="rotation_quaternion", frame=frameId+startFrame)
				lineNB = lineNB + 1
				lastShortJointName = ''
				for i in range(0, nbInnerJoints):
					line = lines[lineNB + i] # joint line
					st = line.strip ('\n').split (' ')
					jointName = st [0]
					jointConfigStr = st [1]
					jointConfig = float (jointConfigStr)
					print ("jointConfig= " + str(jointConfig))
					shortJointName = jointName[0:len(jointName)-3] # remove "_rx" or "_ry" or "_rz...
					#print ('shortJointName= ' + shortJointName)
					jointRotationName = jointName[len(jointName)-1:len(jointName)] # "x" or "y" or "z"
					#print ('jointRotationName= ' + jointRotationName)
					
					if (shortJointName == lastShortJointName): # still same Joint but other rotation, continue to fill config
						fullJointConfig = lastFullJointConfig
						if (reOrientFrames):
							if (jointRotationName == "x"): # x HPP -> y Blender
								fullJointConfig [1] = jointConfig
							if (jointRotationName == "y"): # y HPP -> -x Blender
								fullJointConfig [0] = -jointConfig
							if (jointRotationName == "z"):
								fullJointConfig [2] = jointConfig
						else:
							if (jointRotationName == "x"):
								fullJointConfig [0] = jointConfig
							if (jointRotationName == "y"):
								fullJointConfig [1] = jointConfig
							if (jointRotationName == "z"):
								fullJointConfig [2] = jointConfig
						#print ("same joint = " + shortJointName)
						#print ("same joint fullJointConfig= " + str(fullJointConfig))
					else: # new joint, add previous before erase, then fill new joint full config
						#print ("currentBone name= " + lastShortJointName)
						if (lastShortJointName != ''):
							currentBone = armature.pose.bones[lastShortJointName]
							currentBone.rotation_mode = rotationOrder
							currentBone.rotation_euler = lastFullJointConfig
							#print ("lastShortJointName= " + lastShortJointName)
							#print ("rotation euler= " + str(currentBone.rotation_euler))
							#bpy.ops.object.mode_set(mode='OBJECT')
							currentBone.keyframe_insert (data_path="rotation_euler", frame=frameId+startFrame)
						fullJointConfig= 3*[0] # zero by default
						if (reOrientFrames):
							if (jointRotationName == "x"): # x HPP -> y Blender
								fullJointConfig [1] = jointConfig
							if (jointRotationName == "y"): # y HPP -> -x Blender
								fullJointConfig [0] = -jointConfig
							if (jointRotationName == "z"):
								fullJointConfig [2] = jointConfig
						else:
							if (jointRotationName == "x"):
								fullJointConfig [0] = jointConfig
							if (jointRotationName == "y"):
								fullJointConfig [1] = jointConfig
							if (jointRotationName == "z"):
								fullJointConfig [2] = jointConfig
						#print ("new joint = " + shortJointName)
						#print ("new joint fullJointConfig= " + str(fullJointConfig))
					lastShortJointName = shortJointName
					lastFullJointConfig = fullJointConfig
					print (line)
		    
				currentBone = armature.pose.bones[lastShortJointName]
				currentBone.rotation_mode = rotationOrder
				currentBone.rotation_euler = lastFullJointConfig
				print ("lastShortJointName= " + lastShortJointName)
				print ("rotation euler= " + str(currentBone.rotation_euler))
				#bpy.ops.object.mode_set(mode='OBJECT')
				currentBone.keyframe_insert (data_path="rotation_euler", frame=frameId+startFrame)
	print ("total frame number in loaded motion= " + str(totalFrameNumber))
	return totalFrameNumber+startFrame

def setVisibility (objectName, frame, state):
	if (bpy.data.objects.find(objectName) != -1):
		bpy.context.scene.objects.active = bpy.data.objects[objectName] # set active object
		bpy.context.scene.frame_set(frame)
		bpy.context.active_object.hide = state
		bpy.context.active_object.hide_render = state
		bpy.context.active_object.keyframe_insert(data_path="hide", index=-1, frame=frame)
		bpy.context.active_object.keyframe_insert(data_path="hide_render", index=-1, frame=frame)
	else:
		print ("Object not found: " + objectName)

def setObjectNotInListVisibility (namePrefix, numberOfObjects, skipList, frame, state):
	for i in range(0, numberOfObjects):
		if (not (i in skipList)):
			#print ("set visibility on " + namePrefix + str(i))
			objectName_i = namePrefix+str(i)
			setVisibility (objectName_i, frame, state)

def setObjectInListVisibility (namePrefix, numberOfObjects, theList, frame, state):
	for i in range(0, numberOfObjects):
		if (i in theList):
			#print ("set visibility on " + namePrefix + str(i))
			objectName_i = namePrefix+str(i)
			setVisibility (objectName_i, frame, state)

def setObjectPose (objectName, pose, poseFrame):
	if (bpy.data.objects.find(objectName) != -1):
		bpy.context.scene.objects.active = bpy.data.objects[objectName] # set active object
		bpy.context.scene.frame_set(poseFrame)
		bpy.context.active_object.location = pose[0:3]
		bpy.context.active_object.rotation_euler = [x*pi/180.0 for x in pose[3:6]]
		#print ("pose: " + str(pose))
		bpy.context.active_object.keyframe_insert(data_path="location", index=-1, frame=poseFrame)
		bpy.context.active_object.keyframe_insert(data_path="rotation_euler", index=-1, frame=poseFrame)
	else:
		print ("Object not found: " + objectName)

#---------------------------------------------------------------------------#

def main ():
	# Parameters that do not change from one problem to another
	daeRobotFilePath = '/local/mcampana/devel/hpp/src/animals_description/meshes/'
	daeRobotFileName = daeRobotFilePath + 'sphere_1.dae'
	viewerFilePath = '/local/mcampana/devel/hpp/videos/'
	#daeRMConeFileName = viewerFilePath + 'cones_RM.dae' # No Roadmap for MIG 2016
	daeStartConeFileName = viewerFilePath + 'cone_start.dae'
	daeGoalConeFileName = viewerFilePath + 'cone_goal.dae'
	daePathConeFileName = viewerFilePath + 'cones_path.dae'
	yamlFileName = viewerFilePath + 'frames.yaml'
	scriptFilePath = '/local/mcampana/devel/hpp/src/hpp-rbprm-corba/script/tests/'
	edgeRMFilename = scriptFilePath + 'edges.txt'
	indexesFileName = scriptFilePath + 'indexes.txt'
	edgeNamePrefix = 'edge'; coneNamePrefix = 'Cone_'; coneWpNamePrefix = 'Cone_WP_'; pathName = 'path'
	numPointsPerEdge = 70
	initFrame = 0
	
	# Parameters that CAN change from one problem to another
	pathVisibFrame = 120; beginMotionFrame = 270
	#numberOfCones = 21; rmDisappearFrame = 260; # No Roadmap for MIG 2016
	cameraInitPose = [7.33,8.92,6.86,56.4,-2.16,162.9]
	cameraFollowPose = [-2.80,0.782,0.683,82.7,0,254]

	# Materials
	matConeSG = getOrCreateMaterial ("coneSG", 'SURFACE', [0,0.3,0], 0.4, True, False, False)
	matEdgeRM = getOrCreateMaterial ("edgeRM", 'WIRE', [0.5,0.5,0.5], 1, True, False, False)
	matConeRM = getOrCreateMaterial ("coneRM", 'SURFACE', [0.5,0.5,0.5], 0.4, True, False, False)
	matPath = getOrCreateMaterial ("path", 'WIRE', [0,0,1], 1, True, False, False)
	matConePath = getOrCreateMaterial ("cone_path", 'SURFACE', [0,0,1], 0.4, True, False, False)
	matText = getOrCreateMaterial ("text", 'SURFACE', [0,0,0], 1, True, False, False)
	matTextPath = getOrCreateMaterial ("path_text", 'SURFACE', [0,0,0.4], 1, True, False, False)
	
	# Set World
	world = bpy.data.worlds["World"]
	world.horizon_color[0:3] = [0.3,0.3,0.4] # I find it a lil'bit dark...
	
	# Import meshes
	#importDaeRobot (daeRobotFileName, robotName, matSphereRobot) # NOTE: robot (skin+armature or skeleton) should be loaded manually
	importDaeObjects (daeStartConeFileName, 'Cone', matConeSG)
	importDaeObjects (daeGoalConeFileName, 'Cone', matConeSG)
	importDaeObjects (daePathConeFileName, 'Cone', matConePath)
	#importDaeObjects (daeRMConeFileName, 'Cone', matConeRM) # No Roadmap for MIG 2016

	# Plot edges
	""" No Roadmap for MIG 2016
	listOfEdges = parseEdgeVector (edgeRMFilename, numPointsPerEdge)
	numberOfEdges = len(listOfEdges);
	print ("Number of edges: " + str(numberOfEdges))
	plotEdges (listOfEdges, edgeNamePrefix, numPointsPerEdge, matEdgeRM)
	"""
	
	# Import solution path
	pathPoints = parsePathPoints (pathFileName)
	plotPath ( pathPoints, pathName, matPath)
	startPos = pathPoints [0]
	goalPos = pathPoints [len(pathPoints)-1]
	
	# Import motion
	endMotionFrame = 0 # dummy init
	endMotionFrame = loadMotionBodies (yamlFileName, beginMotionFrame) # for skeleton
	print ("endMotionFrame bodies= " + str(endMotionFrame))

	bpy.data.scenes["Scene"].frame_end = endMotionFrame + 50
	
	# Visibilities
	# Roadmap
	""" No Roadmap for MIG 2016
	setObjectNotInListVisibility ( edgeNamePrefix, numberOfEdges, [], initFrame, False)
	setObjectNotInListVisibility (edgeNamePrefix, numberOfEdges, edgeSkipList, rmDisappearFrame,True)
	setObjectNotInListVisibility (coneNamePrefix, numberOfCones, [], initFrame, False)
	setObjectNotInListVisibility (coneNamePrefix, numberOfCones, coneSkipList, rmDisappearFrame, True)
	setObjectInListVisibility ( edgeNamePrefix, numberOfEdges, edgeSkipList, pathVisibFrame, True)
	setObjectInListVisibility (coneNamePrefix, numberOfCones, coneSkipList, pathVisibFrame, True)
	setVisibility ('Cone', initFrame, False) # 'cause first RM cone name is not following the prefix
	setVisibility ('Cone', rmDisappearFrame, True)
	"""
	# Path
	""" No Roadmap for MIG 2016
	setObjectNotInListVisibility (coneWpNamePrefix, numberOfCones, [], initFrame, True)
	setObjectNotInListVisibility (coneWpNamePrefix, numberOfCones, [], pathVisibFrame, False)
	"""
	setVisibility ('Cone_WP', initFrame, True) # 'cause first WP cone name is not following the prefix
	setVisibility ('Cone_WP', pathVisibFrame, False)
	setVisibility (pathName, initFrame, True)
	setVisibility (pathName, pathVisibFrame, False)
	
	# Set some camera poses
	cameraObj = bpy.data.objects["Camera"]
	cameraObj.constraints.new(type='COPY_LOCATION')
	cameraConstrLocRobot = cameraObj.constraints["Copy Location"]
	cameraConstrLocRobot.target = bpy.data.objects[robotName]
	cameraConstrLocRobot.use_offset = True
	setObjectPose ("Camera", cameraInitPose, initFrame)
	setObjectPose ("Camera", cameraFollowPose, beginMotionFrame-10)
	
	plotGlobalFrame ('GlobalFrame', [0,0,3])

#---------------------------------------------------------------------------#

# Test armature motion import from HPP:
def importArmatureMotion (character, fileName):
	# Parameters that do not change from one problem to another
	daeFilePath = '/local/mcampana/devel/hpp/videos/'
	#daeFilePath = 'C:/Users/Mylene/Desktop/tests_Blender/' # Windows Mylene
	beginMotionFrame = 0
	armatureName = "Armature"
	jointConfigsFileName = daeFilePath + fileName
	if (character == "ANT"):
		reOrientFrames = True; rotationOrder = 'ZXY' # ANT
	if (character == "FROG"):
		reOrientFrames = False; rotationOrder = 'ZXY' # FROG
	if (character == "JUMPERMAN"):
		reOrientFrames = False; rotationOrder = 'ZYX' # JUMPERMAN
	if (character == "KANGAROO"):
		reOrientFrames = False; rotationOrder = 'ZXY' # KANGAROO
	endMotionFrame = loadMotionArmature (jointConfigsFileName, beginMotionFrame, reOrientFrames, rotationOrder, armatureName) # for inner joints


def mainTestBis (): # no armature (no motion actually)
	daeFilePath = '/local/mcampana/devel/hpp/videos/'
	#daeFilePath = 'C:/Users/Mylene/Desktop/tests_Blender/' # Windows Mylene
	matPath = getOrCreateMaterial ("path", 'WIRE', [0,0,1], 1, True, False, False)
	matConeSG = getOrCreateMaterial ("coneSG", 'SURFACE', [0,0.3,0], 0.4, True, False, False)
	matConePath = getOrCreateMaterial ("cone_path", 'SURFACE', [0,0,1], 0.4, True, False, False)
	pathName = 'path'
	beginMotionFrame = 0
	pathFileName = daeFilePath + 'antInDirect_path.txt'
	daeStartConeFileName = daeFilePath + 'cone_start.dae'
	daeGoalConeFileName = daeFilePath + 'cone_goal.dae'
	daePathConeFileName = daeFilePath + 'cones_path.dae'
	pathPoints = parsePathPoints (pathFileName)
	plotPath (pathPoints, pathName, matPath)
	importDaeObjects (daeStartConeFileName, 'Cone', matConeSG)
	importDaeObjects (daeGoalConeFileName, 'Cone', matConeSG)
	importDaeObjects (daePathConeFileName, 'Cone', matConePath)

def importPath (fileName):
	daeFilePath = '/local/mcampana/devel/hpp/videos/'
	#pathFileName = daeFilePath + 'kangaroo_desert_path.txt'
	#pathFileName = daeFilePath + 'lampPlateforms_path.txt'
	pathFileName = daeFilePath + fileName
	matPath = getOrCreateMaterial ("path", 'WIRE', [0,0,1], 1, True, False, False)
	pathName = 'path'
	pathPoints = parsePathPoints (pathFileName)
	plotPath (pathPoints, pathName, matPath)

def irosRoadmapVisibility (): # IROS 2016 PRESENTATION: change Roadmap Visibilities at frame 0
	edgeNamePrefix = 'edge'; coneNamePrefix = 'Cone_'
	initFrame = 0; numPointsPerEdge = 70
	numberOfEdges = 1086; numberOfCones = 339;
	setObjectNotInListVisibility (edgeNamePrefix, numberOfEdges, [], initFrame, True) # Visibility off at frame 0
	setObjectNotInListVisibility (coneNamePrefix, numberOfCones, [], initFrame, True)
	setVisibility ('Cone', initFrame, True) # 'cause first RM cone name is not following the prefix

def displayAntContactConfig ():
	# Parameters that do not change from one problem to another
	daeFilePath = '/local/mcampana/devel/hpp/videos/'
	#daeFilePath = 'C:/Users/Mylene/Desktop/tests_Blender/' # Windows Mylene
	beginMotionFrame = 0
	print ("--------  load motion armature  --------")
	#fileName = 'ant_contactFlexionConfig.txt'
	#fileName = 'ant_contactEFORTbis.txt'
	fileName = 'ant_contactFromRefConfig.txt'
	reOrientFrames = True; rotationOrder = 'ZXY' # ANT
	armatureName = "Armature"
	jointConfigsFileName = daeFilePath + fileName
	endMotionFrame = loadMotionArmature (jointConfigsFileName, beginMotionFrame, reOrientFrames, rotationOrder, armatureName) # for inner joints

def importYamlMotion (fileName):
	viewerFilePath = '/local/mcampana/devel/hpp/videos/'
	#fileName = 'lampPlateforms_frames.yaml'
	#fileName = 'kangarooTrunkDesert_frames.yaml'
	#fileName = 'kangarooTrunkDesert_forcedOrientation_frames.yaml'
	yamlFileName = viewerFilePath + fileName
	beginMotionFrame = 0
	endMotionFrame = loadMotionBodies (yamlFileName, beginMotionFrame)


#---------------------------------------------------------------------------#
#main  ()
#importArmatureMotion  ("JUMPERMAN","spiderman_testJointConfigs.txt") # CHARACTER, filename
#mainTestBis ()
#importPath ('skeleton_test_ComProj_path.txt')
#irosRoadmapVisibility ()
#displayAntContactConfig ()
importYamlMotion ("skeleton_test_testCOMinYaml_frames.yaml")
