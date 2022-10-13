CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(rviz_plugins)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(multi_map_server REQUIRED)

###########
## Build ##
###########
# todo eric just use qt5 for ros2
#message(">>> rviz_QT_VERSION: ${rviz_QT_VERSION}")
#if (rviz_QT_VERSION VERSION_LESS "5")
if (0)
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
    qt4_wrap_cpp(MOC_FILES
            src/goal_tool.h
            src/probmap_display.h
            src/aerialmap_display.h
            src/multi_probmap_display.h
            )
else ()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    #    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    find_package(Qt5 REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
    qt5_wrap_cpp(MOC_FILES
            src/goal_tool.h
            src/probmap_display.h
            src/aerialmap_display.h
            src/multi_probmap_display.h
            )
endif ()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
        src/pose_tool.cpp
        src/goal_tool.cpp
        src/probmap_display.cpp
        src/aerialmap_display.cpp
        src/multi_probmap_display.cpp
        ${MOC_FILES}
        )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros rviz_common rviz_default_plugins rviz_rendering multi_map_server)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
ament_export_libraries(${PROJECT_NAME})

install(FILES
        plugin_description.xml
        DESTINATION share)

install(DIRECTORY media/
        DESTINATION share/media)

install(DIRECTORY icons/
        DESTINATION share/icons)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

ament_package()