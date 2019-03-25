add_library(simulation
	time_prediction.cpp
	include/time_prediction.h)

add_dependencies(simulation framework)

target_include_directories(simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../framework/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE include)

target_compile_features(simulation
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)
