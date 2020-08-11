cmake_minimum_required(VERSION 2.8.3)
project(stopwatch)

add_definitions(-Wall -Wextra -Wmaybe-uninitialized -Wno-unused-result -Wno-unused-parameter -Werror)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    std_msgs
    message_generation
    utils_node
)

execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions(-DCOMMIT="${GIT_COMMIT_HASH}")

add_service_files(
    FILES
    saveRecordsService.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}_lib
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_LIBS}
)


add_library(${PROJECT_NAME}_lib
    src/clock.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}
    src/stopwatch.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_lib
)
