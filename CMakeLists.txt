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
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost COMPONENTS REQUIRED)

#-------#
# Files #
#-------#

file(GLOB SOURCES "src/*.cpp" "src/*.c")

#----------#
# INCLUDES #
#----------#

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/includes
	${Boost_INCLUDE_DIRS}
)

#-------#
# Build #
#-------#

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
	glfw
	Eigen3::Eigen
	${CMAKE_DL_LIBS}
)