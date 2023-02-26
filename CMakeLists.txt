cmake_minimum_required(VERSION 3.15)
project(libvideo-streaming VERSION 1.0)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# video-streaming
set(LIB_SOURCES
                src/vstrm_clock_delta.c
                src/vstrm_dbg.c
                src/vstrm_event.c
                src/vstrm_frame.c
                src/vstrm_h264_sei_streaming.c
                src/vstrm_receiver.c
                src/vstrm_rtp_h264_rx.c
                src/vstrm_rtp_h264_tx.c
                src/vstrm_sender.c
                src/vstrm_session_metadata.c
                src/vstrm_video_stats.c
)

add_library(video-streaming ${LIB_SOURCES})
set_target_properties(video-streaming PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(video-streaming PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(video-streaming PRIVATE "VSTRM_API_EXPORTS")
target_compile_definitions(video-streaming PRIVATE "_GNU_SOURCE")
target_compile_options(video-streaming PRIVATE "-fvisibility=hidden")
target_compile_options(video-streaming PRIVATE "-std=gnu99")

if(WIN32)
  target_link_libraries(video-streaming ws2_32)
endif()

target_link_libraries(video-streaming
                                futils
                                h264
                                pomp
                                rtp
                                transport-packet
                                ulog
                                video-metadata
)
