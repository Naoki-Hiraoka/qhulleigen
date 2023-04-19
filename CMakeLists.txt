cmake_minimum_required(VERSION 3.0.2)
project(qhulleigen)

set(CMAKE_BUILD_TYPE Release) #options are: None Debug Release RelWithDebInfo MinSizeRel.

find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3>=3.2.7)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qhulleigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED src/qhulleigen.cpp)
target_link_libraries(${PROJECT_NAME}
  qhull_r
  ${EIGEN_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
