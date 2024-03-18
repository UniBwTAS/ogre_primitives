cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

project(ogre_primitives)

find_package(catkin REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    DEPENDS OGRE_OV OGRE OGRE-Overlay)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OGRE_OV_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE-Overlay_INCLUDE_DIRS}
    include
)

set(SRC_FILES
    src/wire_frame_bounding_box.cpp
    src/solid_bounding_box.cpp
    src/frustum2d.cpp
    src/map_tile.cpp
    src/cone2d.cpp)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OGRE_OV_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${OGRE-Overlay_LIBRARIES})

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})