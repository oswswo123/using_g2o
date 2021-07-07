# 프로젝트 이름 설정
project(exercise)
cmake_minimum_required(VERSION 3.16)

# c++ 컴파일러 지정
set(CMAKE_CXX_COMPILER g++)
# 컴파일 타입 지정
set(CMAKE_BUILD_TYPE Debug)

# g2o의 CMAKE_MODULE 경로 추가
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/g2o/cmake_modules)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../extraLib/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)

# 소스파일 지정
set(SOURCE_FILE exercise1.cpp)
# 헤더 디렉토리 지정
include_directories(${G2O_INCLUDE_DIR})
#include_directories(${G2O_INCLUDE_DIR} lib/g2o)

# 실행파일 생성
add_executable(test ${SOURCE_FILE})

# 라이브러리 링크 설정
target_link_libraries(test
	${G2O_CORE_LIBRARY}
	${G2O_STUFF_LIBRARY}
	g2o_types_slam3d)
