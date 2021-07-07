set(using_g2o_file
	create_LS.cpp
	create_LS.h
	create_BS.cpp
	create_BS.h
	create_OA.cpp
	create_OA.h
	create_optimizer.cpp
	create_optimizer.h
	using_optimizer.cpp
	using_optimizer.h
	)
	
add_library(using_g2o ${using_g2o_file})

# g2o의 CMAKE_MODULE 경로 추가
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../extraLib/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)

find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(Eigen3 REQUIRED)

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

# 헤더 파일 링크 설정
target_include_directories(using_g2o PUBLIC
	${CSPARSE_INCLUDE_DIR}
	${G2O_INCLUDE_DIR}
	${CSPARSE_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
	)

# 라이브러리 링크 설정
target_link_libraries(using_g2o
	${G2O_CORE_LIBRARY}
	${G2O_STUFF_LIBRARY}
	${CSPARSE_LIBRARY}
	cholmod
	g2o_csparse_extension
	g2o_types_slam2d
	g2o_types_slam3d
	g2o_solver_csparse
	${G2O_LIBRARIES}
	)

