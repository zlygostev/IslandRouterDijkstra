cmake_minimum_required(VERSION 3.8)
project(Bachelor)

add_subdirectory(framework)

add_subdirectory(simulation)

add_subdirectory(model)

add_subdirectory(router)

add_subdirectory(visualizer)



add_executable(Bachelor main.cpp)
target_link_libraries(Bachelor visualizer framework simulation)
target_include_directories(Bachelor PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/model/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/router/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE include)

add_custom_command(
    TARGET Bachelor
    POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        $<TARGET_FILE_DIR:Bachelor>/assets)





