cmake_minimum_required(VERSION 2.8.3)
project(grizzly_motor_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  grizzly_motor_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grizzly_motor_driver
  CATKIN_DEPENDS diagnostic_updater grizzly_motor_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/driver.cpp
  src/interface.cpp
  src/node.cpp
  src/register.cpp
  src/registers.cpp
  src/diagnostic_updater.cpp
)

target_link_libraries(grizzly_motor_driver
  ${catkin_LIBRARIES}
)

add_executable(test_node src/main.cpp)

target_link_libraries(test_node
  grizzly_motor_driver
  ${catkin_LIBRARIES}
)

# Install
install(TARGETS grizzly_motor_driver test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  file(GLOB LINT_SRCS
    src/*.cpp
    include/*.h)
  roslint_cpp(${LINT_SRCS})
  roslint_add_test()
endif()
