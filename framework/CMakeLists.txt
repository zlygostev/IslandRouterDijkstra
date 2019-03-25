add_library(framework
	maps.cpp
	include/maps.h
	include/path.h
	include/map_types.h
	include/prioritized_queue.h

	)

target_include_directories(framework PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE include)


target_compile_features(framework
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)
