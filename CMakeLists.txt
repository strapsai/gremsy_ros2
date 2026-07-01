cmake_minimum_required(VERSION 3.8)

project (gremsy_ros2)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(VIO 1)

if (MB1)
set(MB1 $ENV{MB1})
message("-- MB1: ${MB1}")
add_definitions(-DMB1=${MB1})
endif(MB1)

if (VIO)
set(VIO $ENV{VIO})
message("-- VIO: ${VIO}")
add_definitions(-DVIO=${VIO})
endif(VIO)

if (ZIO)
set(ZIO $ENV{ZIO})
message("-- ZIO: ${ZIO}")
add_definitions(-DZIO=${ZIO})
endif(ZIO)

if (ORUSL)
set(ORUSL $ENV{ORUSL})
message("-- ORUSL: ${ORUSL}")
add_definitions(-DORUSL=${ORUSL})
endif(ORUSL)

find_package(PkgConfig REQUIRED)
# GStreamer (RTSP video for the UIs) + GTKmm (UI toolkit)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GST REQUIRED
                    gstreamer-1.0
                    gstreamer-video-1.0
                    gstreamer-app-1.0
                    )
# GTKmm is only needed for the GUI executables (ui_demo, ui_demo_ros2). Keep it
# optional so the onboard bridge (spirit_driver) always builds even where the
# GUI toolkit is absent (e.g. a headless build container).
pkg_check_modules(GTKMM QUIET gtkmm-3.0)

# PayloadSDK (the Gremsy UDP library). Only spirit_driver actually opens a
# connection; the UIs link it for the shared enums/param table.
add_subdirectory(libs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

pkg_check_modules(JSONCPP jsoncpp)
find_package(CURL REQUIRED)

set(PAYLOADSDKLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(${PAYLOADSDKLIB_DIR}
                    ${PAYLOADSDKLIB_DIR}/third-party/mavlink/include
                    ${PAYLOADSDKLIB_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDKLIB_DIR}/gSDK/src
                    ${GST_INCLUDE_DIRS}
                    ${CURL_INCLUDE_DIR}
                    )

# --------------------------------------------------------------------------
# 1) spirit_driver : onboard ROS2 <-> Gremsy PayloadSDK bridge
# --------------------------------------------------------------------------
add_executable(spirit_driver src/spirit_driver.cpp)
ament_target_dependencies(spirit_driver
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_geometry_msgs drone_msgs)
target_link_libraries(spirit_driver
                    PayloadSDK
                    ${CMAKE_THREAD_LIBS_INIT}
                    ${GST_LIBRARIES}
                    ${JSONCPP_LIBRARIES}
                    ${CURL_LIBRARIES}
                    stdc++fs
                    )

install(TARGETS spirit_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --------------------------------------------------------------------------
# GUI executables (only when GTKmm is available)
# --------------------------------------------------------------------------
if(NOT GTKMM_FOUND)
  message(WARNING "gtkmm-3.0 not found: skipping ui_demo and ui_demo_ros2 (spirit_driver still builds).")
else()
# Shared GTK widgets (backend-agnostic: emit command signals, receive updates)
set(UI_WIDGET_SOURCES
    tests/ui_demo/ui_gtk/MainWindow.cpp
    tests/ui_demo/ui_gtk/PayloadControlTab/PayloadSettingsTab.cpp
    )
set(UI_WIDGET_INCLUDES
    ${GTKMM_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/ui_demo/ui_gtk
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/ui_demo/ui_gtk/PayloadControlTab
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/payload-define
    )

# --------------------------------------------------------------------------
# 2) ui_demo : original GTK app, talks to the Gremsy directly over UDP
# --------------------------------------------------------------------------
add_executable(ui_demo tests/ui_demo/main.cpp ${UI_WIDGET_SOURCES})
target_include_directories(ui_demo PRIVATE
    ${UI_WIDGET_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/ui_demo)
target_link_libraries(ui_demo PayloadSDK pthread ${GTKMM_LIBRARIES} ${GST_LIBRARIES})

# --------------------------------------------------------------------------
# 3) ui_demo_ros2 : same GTK app, but every control/telemetry goes over ROS2.
#    Needs only a running spirit_driver on the drone; never touches the Gremsy.
# --------------------------------------------------------------------------
add_executable(ui_demo_ros2 tests/ui_demo_ros2/main.cpp ${UI_WIDGET_SOURCES})
target_include_directories(ui_demo_ros2 PRIVATE
    ${UI_WIDGET_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/ui_demo_ros2)
ament_target_dependencies(ui_demo_ros2 rclcpp std_msgs geometry_msgs drone_msgs)
target_link_libraries(ui_demo_ros2 PayloadSDK pthread ${GTKMM_LIBRARIES} ${GST_LIBRARIES})

install(TARGETS ui_demo ui_demo_ros2
        DESTINATION lib/${PROJECT_NAME})
endif()  # GTKMM_FOUND

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
