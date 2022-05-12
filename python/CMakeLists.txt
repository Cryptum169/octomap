CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(pyoctomap)

include_directories("${CMAKE_SOURCE_DIR}/octomap/include/")
include_directories("${CMAKE_SOURCE_DIR}/octomap/python")

SET (SOURCE_FILES
  AbstractOcTree.cpp
  AbstractOccupancyOcTree.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeStamped.cpp
  ColorOcTree.cpp
)
list(TRANSFORM SOURCE_FILES PREPEND "${CMAKE_SOURCE_DIR}/octomap/src/")

SET (octomath_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )
list(TRANSFORM octomath_SRCS PREPEND "${CMAKE_SOURCE_DIR}/octomap/src/math/")

list(APPEND SOURCE_FILES ${octomath_SRCS})
file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/octomap/include/octomap/*.h" "${CMAKE_SOURCE_DIR}/octomap/include/octomap/math/*.h")
file(GLOB PYTHON_FILES "${CMAKE_SOURCE_DIR}/python/*.cpp" "${CMAKE_SOURCE_DIR}/python/*.h")

# message(STATUS "Source Files: ${CMAKE_SOURCE_DIR}")

source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${SOURCE_FILES} ${HEADER_FILES} ${PYTHON_FILES} )

# message(status "header files: ${HEADER_FILES}")

find_package(pybind11 2.9.1 REQUIRED)
pybind11_add_module(pyoctomap 
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_link_libraries(pyoctomap PUBLIC)

if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
  # Use older version of cmake's find_python
  find_package(PythonInterp)

  if(NOT ${PYTHONINTERP_FOUND})
    message(
      FATAL_ERROR
        "Cannot find Python interpreter. Please install Python >= 3.6.")
  endif()

  find_package(PythonLibs ${PYTHON_VERSION_STRING})

  set(Python_VERSION_MAJOR ${PYTHON_VERSION_MAJOR})
  set(Python_VERSION_MINOR ${PYTHON_VERSION_MINOR})
  set(Python_EXECUTABLE ${PYTHON_EXECUTABLE})

else()
  # Get info about the Python3 interpreter
  # https://cmake.org/cmake/help/latest/module/FindPython3.html#module:FindPython3
  find_package(Python3 COMPONENTS Interpreter Development)

  if(NOT ${Python3_FOUND})
    message(
      FATAL_ERROR
        "Cannot find Python3 interpreter. Please install Python >= 3.6.")
  endif()

  set(Python_VERSION_MAJOR ${Python3_VERSION_MAJOR})
  set(Python_VERSION_MINOR ${Python3_VERSION_MINOR})
endif()

install(TARGETS pyoctomap
  COMPONENT python
  LIBRARY DESTINATION "${Python3_SITELIB}"
)
