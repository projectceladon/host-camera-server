add_executable (camera_socket_client camera_socket_client.cc)
target_link_libraries (camera_socket_client LINK_PUBLIC ${PROJECT_NAME})

add_executable (camera_client camera_client.cc)
target_link_libraries (camera_client LINK_PUBLIC ${PROJECT_NAME})


find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

INCLUDE_DIRECTORIES(PRIVATE  ${SWSCALE_INCLUDE_DIR} ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR} ${AVDEVICE_INCLUDE_DIR})
ADD_EXECUTABLE(stream host_camera_server.cc)
TARGET_LINK_LIBRARIES(stream  LINK_PUBLIC ${PROJECT_NAME} ${SWSCALE_LIBRARY} ${ARCHIVE_LIBS} ${LINK_FLAGS} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY})

install(TARGETS camera_socket_client
                camera_client
                stream
        DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
