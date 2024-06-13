cmake_minimum_required(VERSION 3.8)

project(tf2_conversions)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

ament_target_dependencies(${PROJECT_NAME} INTERFACE geometry_msgs)

target_include_directories(${PROJECT_NAME}
    INTERFACE 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${EIGEN3_INCLUDE_DIRS}
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
)

ament_export_dependencies(Eigen3 geometry_msgs)
ament_export_targets(${PROJECT_NAME}_export)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_package()

function(add_test base_name)
	add_executable(${base_name} test/${base_name}.cpp)
    ament_target_dependencies(${base_name}  geometry_msgs)
    target_include_directories(${base_name}
        PUBLIC 
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
        ${EIGEN3_INCLUDE_DIRS}
    )
    install(
        TARGETS ${base_name}
        DESTINATION lib/test/${PROJECT_NAME}
    )
endfunction()

add_test(msg2eigen)
add_test(eigen2msg)