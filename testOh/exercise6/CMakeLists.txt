# 프로젝트 이름 설정
project(exercise)
cmake_minimum_required(VERSION 3.16)

# c++ 컴파일러 지정
set(CMAKE_CXX_COMPILER g++)
# 컴파일 타입 지정
set(CMAKE_BUILD_TYPE Debug)

# 소스파일 지정
set(SOURCE_FILE exercise6.cpp)

# 하위 디렉토리 지정
add_subdirectory(
	${CMAKE_CURRENT_SOURCE_DIR}/../../src/using_g2o	# CMakeLists가 위치한 source dir
	${CMAKE_CURRENT_SOURCE_DIR}/build/using_g2o		# build 결과물을 배치할 binary dir
	)

# 실행파일 생성
add_executable(test ${SOURCE_FILE})

# 헤더 파일
target_include_directories(test PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../../src/using_g2o	# using_g2o lib
	)
	
# 라이브러리 링크 설정
target_link_libraries(test
	using_g2o
	)
