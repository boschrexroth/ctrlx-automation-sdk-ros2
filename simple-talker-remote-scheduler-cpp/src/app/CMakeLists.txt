cmake_minimum_required(VERSION 3.8)
project(app)
set(TARGET_PROJECT_NAME talker)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CTRLX_DL REQUIRED ctrlx-datalayer)
pkg_check_modules(CTRLX_SCHEDULER REQUIRED ctrlx-scheduler)

#
# Set link directories
#
MESSAGE( STATUS "Libraries directory: ${LIBRARY_DIR}")
link_directories(
  ${LIBRARY_DIR}
  ${LIBRARY_DEP_DIR}
  )

SET ( PRIVATE_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CTRLX_DL_INCLUDE_DIRECTORIES}
  ${CTRLX_SCHEDULER_INCLUDE_DIRECTORIES}
  )

# Set target link libraries


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(talker 
    src/scheduler_member_function.cpp
    src/callable_factory.cpp
    src/callable.cpp
)

ament_target_dependencies(talker rclcpp std_msgs)


#
# Set target include directories
#
target_include_directories ( ${TARGET_PROJECT_NAME}
  PUBLIC ${PUBLIC_INCLUDE_DIRS}
  PUBLIC ${LIBRARY_INCLUDES}
  PRIVATE ${PRIVATE_INCLUDE_DIRS}
)
#
target_link_libraries(${TARGET_PROJECT_NAME} -Wl,--no-undefined)
target_link_libraries(${TARGET_PROJECT_NAME}
  ${CTRLX_DL_LIBRARIES}
  ${CTRLX_SCHEDULER_LIBRARIES}
  pthread
  ssl
  crypto
)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
