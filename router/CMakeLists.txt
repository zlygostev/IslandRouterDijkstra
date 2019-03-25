#add_library(router
#	router.cpp
#	include/router.h)

#add_dependencies(router framework)

#target_include_directories(router PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../framework/include>
#    $<INSTALL_INTERFACE:include>
#    PRIVATE include)

#target_compile_features(router
#    PUBLIC cxx_lambdas cxx_auto_type
#    PRIVATE cxx_lambdas cxx_auto_type)
