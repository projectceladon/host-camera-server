cmake_minimum_required (VERSION 3.6)
project (vhal-client VERSION 0.1 DESCRIPTION "VHAL Client library written in C++17 for Touch, Joystick, GPS, Audio, Camera and Sensor, " LANGUAGES CXX)

option(BUILD_EXAMPLES "Build host_camera_service?" ON)

message(STATUS "Project name: ${PROJECT_NAME}")


set (TARGET_OS "linux")
if(NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif()
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat -Wformat-security -Werror=format-security \
                     -D_FORTIFY_SOURCE=2 -fstack-protector-strong -fPIE -fPIC -Wall")
set (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set (LINK_FLAGS "-pie -Wl,-z,relro -Wl,-z,now -Wl,-z,noexecstack")
set (CMAKE_EXE_LINKER_FLAGS "${LINK_FLAGS}")
set (CMAKE_SHARED_LINKER_FLAGS "${LINK_FLAGS}")

include_directories (
  "${CMAKE_CURRENT_SOURCE_DIR}/include/libvhal"
)

include(GNUInstallDirs)

set (vhal_version_major 0)
set (vhal_version_minor 1)

link_directories ("${LIBRARY_OUTPUT_PATH}")

add_subdirectory (source)
add_subdirectory (include/libvhal)
if (BUILD_EXAMPLES)
  add_subdirectory (host_camera_service)
endif()

#Add pkg-config file
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake" ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install( FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}/pkgconfig )
# TODO: Add tests
