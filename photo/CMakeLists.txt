cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(photo)
# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

################
# DEPENDENCIES #
################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(photo_msgs REQUIRED)


find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
find_package(PkgConfig REQUIRED)
pkg_check_modules(gphoto2 REQUIRED libgphoto2)

set(DEPS
  photo_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${gphoto2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
   src/libphoto2/photo_reporter.cpp
   src/libphoto2/photo_camera_list.cpp
   src/libphoto2/photo_camera.cpp
   src/libphoto2/photo_image.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ${DEPS}
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_lib BEFORE PRIVATE
  INTERFACE_INCLUDE_DIRECTORIES>)


add_executable(photo_node src/photo_node/photo_node.cpp)
add_executable(test_photo src/test_photo/test_photo.cpp)



target_link_libraries(photo_node
  ${PROJECT_NAME}_lib
  ${gphoto2_LIBRARIES}
)

target_link_libraries(test_photo
    ${PROJECT_NAME}_lib
    ${gphoto2_LIBRARIES}
)

###########
# INSTALL #
###########
# Install includes.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS photo_node test_photo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include) #  equivalent to catkin_package(INCLUDE ...).
ament_export_libraries(${PROJECT_NAME}_lib) #  equivalent to catkin_package(LIBRARIES ...).
ament_export_dependencies(${DEPS})

ament_package()
