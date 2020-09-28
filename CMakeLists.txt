project(gerber)

cmake_minimum_required(VERSION 3.14)

#--------#
# OPTION #
#--------#

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wextra")
option(USE_OPENMP "OpenMP?"      OFF)
set(CMAKE_BUILD_TYPE Release)

#----------#
# Packages #
#----------#

if(USE_OPENMP)
    find_package(OpenMP)
endif(USE_OPENMP)
if(USE_OPENMP AND OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(glfw3 3.3 REQUIRED)

#-------#
# Files #
#-------#

set(SRCS test.cpp glad.c)

#----------#
# INCLUDES #
#----------#

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#-------#
# Build #
#-------#

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME}
	glfw
	${CMAKE_DL_LIBS}
)