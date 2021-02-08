project(gerber)

cmake_minimum_required(VERSION 3.14)

#--------#
# OPTION #
#--------#

set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

#----------#
# Packages #
#----------#

find_package(glfw3 3.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#-------#
# Files #
#-------#

set(SRCS
	main.cpp
	object.cpp
	reader.cpp
	triangulate.cpp
	cvt.cpp
	glad.c
)

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
	Eigen3::Eigen
	${CMAKE_DL_LIBS}
)