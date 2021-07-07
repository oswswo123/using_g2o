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

#find_package(G2O REQUIRED)

