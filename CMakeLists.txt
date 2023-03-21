cmake_minimum_required(VERSION 3.8)
project(logi_g29)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

FetchContent_Declare(
        libackermann
        GIT_REPOSITORY https://github.com/iscumd/libackermann.git
)
FetchContent_MakeAvailable(libackermann)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(logi-g29 src/logi-g29.cpp src/logi_g29_node.cpp)
target_include_directories(logi-g29 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(logi-g29 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(logi-g29 libackermann)

# Make deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        ackermann_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        joy
        )

ament_target_dependencies(
        logi-g29
        ${dependencies}
)

install(TARGETS logi-g29
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    #TODO add
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
