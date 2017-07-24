cmake_minimum_required(VERSION 2.8.3)
project(newwrock)

find_package(catkin REQUIRED COMPONENTS

) 
find_package(PCL 1.7 REQUIRED)

catkin_package(
    DEPENDS # TODO
    INCLUDE_DIRS 
      common/include
      include
    LIBRARIES # TODO
CATKIN_DEPENDS sensor_msgs tf urdf diagnostic_updater diagnostic_msgs control_msgs  message_runtime roscpp
)    

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include/${PROJECT_NAME}
)
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})


qt4_wrap_cpp(MOC_FILES
  include/drive_widget.h
  include/teleop_panel.h
)

set(SOURCE_FILES
  src/drive_widget.cpp
  src/teleop_panel.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} 
   ${SOURCE_FILES}
 )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${QT_LIBRARIES} ${catkin_LIBRARIES}
 )

set(CATKIN_PACKAGE_LIB_DESTINATION ${PROJECT_SOURCE_DIR}/lib)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

