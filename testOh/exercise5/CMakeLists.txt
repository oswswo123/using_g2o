# 프로젝트 이름 설정
project(exercise)
cmake_minimum_required(VERSION 3.16)

# c++ 컴파일러 지정
set(CMAKE_CXX_COMPILER g++)
# 컴파일 타입 지정
set(CMAKE_BUILD_TYPE Debug)

#[[
# CSPARSE 관련 설정
find_path(CSPARSE_INCLUDE_DIR NAMES cs.h
	PATHS
	/usr/include/suitesparse
	/usr/include
	/usr/local/include
	PATH_SUFFIXES
	suitesparse
	)
find_library(CSPARSE_LIBRARY NAMES cxsparse
	PATHS
	/usr/lib
	/usr/local/lib
	)
	]]

# g2o의 CMAKE_MODULE 경로 추가
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../extraLib/g2o/cmake_modules)
#set(G2O_ROOT /usr/local/include/g2o)

#find_package(G2O REQUIRED)
#find_package(CSparse REQUIRED)
#find_package(Eigen3 REQUIRED)

# 소스파일 지정
set(SOURCE_FILE exercise5.cpp)

# 헤더 디렉토리 지정
include_directories(
	using_g2o
#	${G2O_INCLUDE_DIR}
#	${CMAKE_CURRENT_SOURCE_DIR}/../../extraLib/g2o 
#	${CSPARSE_INCLUDE_DIR}
#	${EIGEN3_INCLUDE_DIR}
	)
	
# 하위 디렉토리 지정
add_subdirectory(using_g2o)

# 실행파일 생성
add_executable(test ${SOURCE_FILE})

# 라이브러리 링크 설정
target_link_libraries(test
	using_g2o
#	${G2O_LIBRARIES}
#	${G2O_CORE_LIBRARY}
#	${G2O_STUFF_LIBRARY}
#	${CSPARSE_LIBRARY}
#	cholmod
#	g2o_types_slam2d
#	g2o_types_slam3d
#	g2o_solver_csparse
#	g2o_csparse_extension
	)
