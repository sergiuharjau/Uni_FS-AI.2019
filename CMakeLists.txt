cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)

project(fspycan CXX)

#limit number of errors to 1
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")


# Find necessary packages
find_package( PythonLibs 3.6 REQUIRED )
include_directories( ${PYTHON_INCLUDE_DIRS} )
 
find_package( Boost COMPONENTS python-py36 REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

# tell cmake where to put the executables that it creates
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)    
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# where to put the object files it creates
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# add conversion lib
include_directories( opencv-ndarray-conversion )
add_library( conversion opencv-ndarray-conversion/conversion.cpp )

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

message( ${Boost_LIBRARIES} )

add_library( threading SHARED src/threading.cpp )

add_library( expose_threading SHARED src/expose_threading.cpp )
target_link_libraries( expose_threading conversion ${Boost_LIBRARIES} threading ${ZED_LIBRARIES}  ${OpenCV_LIBRARIES} )
set_target_properties( expose_threading PROPERTIES PREFIX "" )


#add_executable( fspycan src/fspycan.cpp )
#target_link_libraries( fspycan ${Boost_LIBRARIES} )

