# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-rbprm-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-rbprm-corba.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-rbprm-corba)
SET(PROJECT_DESCRIPTION "Corba server for reachability based planning")
SET(PROJECT_URL "")

# Set to 1 for profiling
#~ add_definitions(-DPROFILE)

SET(CUSTOM_HEADER_DIR hpp/corbaserver/rbprm)

SETUP_PROJECT ()

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/rbprm/server.hh
)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

ADD_DOC_DEPENDENCY("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-rbprm")
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("hpp-model-urdf >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
ADD_SUBDIRECTORY(src)

CONFIG_FILES (include/hpp/corbaserver/rbprm/doc.hh)

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  data/package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  data/urdf/hrp2_trunk.urdf
  data/urdf/hrp2_trunk_flexible.urdf
  data/urdf/hrp2_rom.urdf
  data/urdf/hrp2_larm_rom.urdf
  data/urdf/hrp2_rarm_rom.urdf
  data/urdf/hrp2_lleg_rom.urdf
  data/urdf/hrp2_rleg_rom.urdf
  data/urdf/hyq/hyq_trunk.urdf
  data/urdf/hyq/hyq_trunk_large.urdf
  data/urdf/hyq/hyq_rhleg_rom.urdf
  data/urdf/hyq/hyq_rfleg_rom.urdf
  data/urdf/hyq/hyq_lhleg_rom.urdf
  data/urdf/hyq/hyq_lfleg_rom.urdf
  data/urdf/hyq/hyq_rom.urdf
  data/urdf/box_rom.urdf
  data/urdf/box.urdf
  data/urdf/scene.urdf
  data/urdf/scene_wall.urdf
  data/urdf/truck.urdf
  data/urdf/stair_bauzil.urdf
  data/urdf/climb.urdf
  data/urdf/stepladder.urdf
  data/urdf/ground.urdf
  data/urdf/groundcrouch.urdf
  data/urdf/darpa.urdf
  data/urdf/car.urdf
  #~ data/urdf/scene2.urdf
  data/urdf/robot_test/robot_test_lleg_rom.urdf
  data/urdf/robot_test/robot_test_rleg_rom.urdf
  data/urdf/robot_test/robot_test_trunk.urdf
	data/urdf/ground_bigStep.urdf
	data/urdf/ground_jump_easy.urdf
	data/urdf/ground_jump_bar.urdf
	data/urdf/ground_jump_med.urdf
	data/urdf/ground_with_bridge.urdf
  data/urdf/skeleton/skeleton.urdf
  data/urdf/skeleton/skeleton_trunk_flexible.urdf
  data/urdf/skeleton/RHandSphere.urdf
  data/urdf/skeleton/LHandSphere.urdf
  data/urdf/skeleton/RFootSphere.urdf
  data/urdf/skeleton/LFootSphere.urdf
  data/urdf/spiderman/spiderman.urdf
  data/urdf/spiderman/spiderman_trunk.urdf
  data/urdf/spiderman/SpidermanRHandSphere.urdf
  data/urdf/spiderman/SpidermanLHandSphere.urdf
  data/urdf/spiderman/SpidermanRFootSphere.urdf
  data/urdf/spiderman/SpidermanLFootSphere.urdf
  data/urdf/frog/frog.urdf
  data/urdf/frog/frog_trunk.urdf
  data/urdf/frog/FrogRHandSphere.urdf
  data/urdf/frog/FrogLHandSphere.urdf
  data/urdf/frog/FrogRFootSphere.urdf
  data/urdf/frog/FrogLFootSphere.urdf
  data/urdf/ant/ant.urdf
  data/urdf/ant/ant_trunk.urdf
  data/urdf/ant/LFFootSphere.urdf
  data/urdf/ant/LMFootSphere.urdf
  data/urdf/ant/LBFootSphere.urdf
  data/urdf/ant/RFFootSphere.urdf
  data/urdf/ant/RMFootSphere.urdf
  data/urdf/ant/RBFootSphere.urdf
  data/urdf/cave.urdf
  data/urdf/desert.urdf
  data/urdf/high_plateforms.urdf
  data/urdf/etang_envir.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  data/srdf/hrp2_trunk.srdf
  data/srdf/hrp2_trunk_flexible.srdf
  data/srdf/hrp2_rom.srdf
  data/srdf/hrp2_larm_rom.srdf
  data/srdf/hrp2_rarm_rom.srdf
  data/srdf/hrp2_lleg_rom.srdf
  data/srdf/hrp2_rleg_rom.srdf
  data/srdf/hyq/hyq_trunk.srdf
  data/srdf/hyq/hyq_trunk_large.srdf
  data/srdf/hyq/hyq_rhleg_rom.srdf
  data/srdf/hyq/hyq_rfleg_rom.srdf
  data/srdf/hyq/hyq_lhleg_rom.srdf
  data/srdf/hyq/hyq_lfleg_rom.srdf
  data/srdf/hyq/hyq_rom.srdf
  data/srdf/box_rom.srdf
  data/srdf/box.srdf
  data/srdf/scene.srdf
  data/srdf/scene_wall.srdf
  data/srdf/truck.srdf
  data/srdf/stair_bauzil.srdf
  data/srdf/climb.srdf
  data/srdf/stepladder.srdf
  data/srdf/ground.srdf
  data/srdf/groundcrouch.srdf
  data/srdf/darpa.srdf
  data/srdf/car.srdf
  #~ data/srdf/scene2.srdf
  data/srdf/robot_test/robot_test_lleg_rom.srdf
  data/srdf/robot_test/robot_test_rleg_rom.srdf
  data/srdf/robot_test/robot_test_trunk.srdf
	data/srdf/ground_bigStep.srdf
	data/srdf/ground_jump_easy.srdf
	data/srdf/ground_jump_bar.srdf
	data/srdf/ground_jump_med.srdf
	data/srdf/ground_with_bridge.srdf
  data/srdf/skeleton/skeleton.srdf
  data/srdf/skeleton/skeleton_trunk_flexible.srdf
  data/srdf/skeleton/RHandSphere.srdf
  data/srdf/skeleton/LHandSphere.srdf
  data/srdf/skeleton/RFootSphere.srdf
  data/srdf/skeleton/LFootSphere.srdf
  data/srdf/spiderman/spiderman.srdf
  data/srdf/spiderman/spiderman_trunk.srdf
  data/srdf/spiderman/SpidermanRHandSphere.srdf
  data/srdf/spiderman/SpidermanLHandSphere.srdf
  data/srdf/spiderman/SpidermanRFootSphere.srdf
  data/srdf/spiderman/SpidermanLFootSphere.srdf
  data/srdf/frog/frog.srdf
  data/srdf/frog/frog_trunk.srdf
  data/srdf/frog/FrogRHandSphere.srdf
  data/srdf/frog/FrogLHandSphere.srdf
  data/srdf/frog/FrogRFootSphere.srdf
  data/srdf/frog/FrogLFootSphere.srdf
  data/srdf/ant/ant.srdf
  data/srdf/ant/ant_trunk.srdf
  data/srdf/ant/LFFootSphere.srdf
  data/srdf/ant/LMFootSphere.srdf
  data/srdf/ant/LBFootSphere.srdf
  data/srdf/ant/RFFootSphere.srdf
  data/srdf/ant/RMFootSphere.srdf
  data/srdf/ant/RBFootSphere.srdf
  data/srdf/cave.srdf
  data/srdf/desert.srdf
  data/srdf/high_plateforms.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
	data/meshes/darpa.stl
	data/meshes/darpareduced.stl
	data/meshes/car.stl
	data/meshes/car_simple.stl
	data/meshes/hrp2_trunk.stl
	data/meshes/hrp2_trunk_body.stl
	data/meshes/hrp2_trunk_torso.stl
        data/meshes/hrp2_trunk_body_view.dae
        data/meshes/hrp2_trunk_torso_view.dae
	data/meshes/hrp2_rom.stl
	data/meshes/hrp2_larm_rom.stl
	data/meshes/hrp2_rarm_rom.stl
	data/meshes/hrp2_lleg_rom.stl
	data/meshes/hrp2_rleg_rom.stl
	data/meshes/ground.stl
	data/meshes/box.stl
	data/meshes/box_rom.stl
	data/meshes/robot_box.stl
	data/meshes/chair_simple.stl
	data/meshes/ground.stl
	data/meshes/groundcrouch.stl
	data/meshes/truck.stl
	data/meshes/truck_vision.stl
	data/meshes/stair_bauzil.stl
        data/meshes/stair_bauzil_reduced.stl
	data/meshes/climb.stl
	data/meshes/stepladder.stl
	data/meshes/chair.stl
	data/meshes/car.stl
	data/meshes/ground_bigStep.stl
	data/meshes/ground_jump_easy.stl
	data/meshes/ground_jump_easyStart.stl
	#data/meshes/ground_jump_easyStart2.stl
	data/meshes/ground_jump_easyGoal.stl
	data/meshes/ground_jump_bar.stl
	data/meshes/ground_jump_med.stl
	data/meshes/ground_with_bridge.stl
	data/meshes/cave.dae
	data/meshes/desert.dae
	data/meshes/high_plateforms.dae	
	data/meshes/cave.stl
	data/meshes/desert.stl
	data/meshes/high_plateforms.stl	
	data/meshes/etang_envir.dae
	data/meshes/etang_envir.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )
install(FILES
	data/meshes/hyq/hyq_all.stl
	data/meshes/hyq/hyq_trunk.stl
	data/meshes/hyq/hyq_trunk_large.stl
	data/meshes/hyq/hyq_rom.stl
	data/meshes/hyq/hyq_rhleg_rom.stl
	data/meshes/hyq/hyq_rfleg_rom.stl
	data/meshes/hyq/hyq_lhleg_rom.stl
	data/meshes/hyq/hyq_lfleg_rom.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/hyq
  )

install(FILES
	data/meshes/robot_test/robot_trunk_view.stl
	data/meshes/robot_test/robot_trunk.stl
	data/meshes/robot_test/Lleg_rom.stl
	data/meshes/robot_test/Rleg_rom.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/robot_test
  )

install(DIRECTORY
	data/meshes/skeleton
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/
  )

install(DIRECTORY
	data/meshes/ant
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/
  )

install(DIRECTORY
	data/meshes/spiderman
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/
  )

install(DIRECTORY
	data/meshes/frog
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/
  )
  
#install(DIRECTORY data/hyq_description
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/../)
  
SETUP_PROJECT_FINALIZE()
