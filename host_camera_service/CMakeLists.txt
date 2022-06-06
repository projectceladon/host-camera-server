
find_package(PkgConfig REQUIRED)

pkg_check_modules(PKG_FFMPEG REQUIRED
    libavdevice
    libavfilter
    libavformat
    libavcodec
    libswresample
    libswscale
    libavutil
    IMPORTED_TARGET
)

add_library(FFmpeg INTERFACE)

target_link_libraries(FFmpeg
    INTERFACE
    PkgConfig::PKG_FFMPEG
)

add_executable (stream host_camera_server.cc)

find_package(Threads REQUIRED)

target_link_libraries(stream
    PRIVATE
    FFmpeg
    Threads::Threads
    ${PROJECT_NAME}
    ${CMAKE_DL_LIBS}
)

install(TARGETS stream
        DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
