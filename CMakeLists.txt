cmake_minimum_required(VERSION 3.10)

project(MeshViewer)

# Find CGAL
find_package(CGAL REQUIRED COMPONENTS Core Qt5)

# Find Qt5
find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets REQUIRED)

# Find QGLViewer
find_library(QGLVIEWER_LIBRARY
    NAMES QGLViewer-qt5 QGLViewer
    PATHS /usr/local/lib /usr/lib /opt/local/lib
)

# Find Eigen
find_package(Eigen3 REQUIRED)

find_path(QGLVIEWER_INCLUDE_DIR
    NAMES QGLViewer/qglviewer.h
    PATHS /usr/local/include /usr/include /opt/local/include
)

if (QGLVIEWER_LIBRARY AND QGLVIEWER_INCLUDE_DIR)
    message(STATUS "Found QGLViewer: ${QGLVIEWER_LIBRARY}")
    message(STATUS "QGLViewer include path: ${QGLVIEWER_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "QGLViewer not found! Install it or specify the correct path.")
endif()

# Include directories
include_directories(${QGLVIEWER_INCLUDE_DIR} ${CGAL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Create executable
add_executable(mesh_viewer main.cpp)

# Link libraries
target_link_libraries(mesh_viewer
    ${QGLVIEWER_LIBRARY}
    CGAL::CGAL CGAL::CGAL_Qt5
    Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets
)

# Ensure we use C++17 or newer
set_property(TARGET mesh_viewer PROPERTY CXX_STANDARD 17)
set_property(TARGET mesh_viewer PROPERTY CXX_STANDARD_REQUIRED ON)

message(STATUS "CGAL version: ${CGAL_VERSION}")

target_compile_definitions(mesh_viewer PRIVATE CGAL_USE_BASIC_VIEWER CGAL_EIGEN3_ENABLED)

set(CMAKE_BUILD_TYPE Debug)

