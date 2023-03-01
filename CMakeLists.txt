cmake_minimum_required(VERSION 3.15)
project(video-streaming VERSION 1.0)

if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

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

add_library(${PROJECT_NAME} ${LIB_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "VSTRM_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")

if(WIN32)
    target_link_libraries(${PROJECT_NAME} 
                          PRIVATE 
                          ws2_32
                          futils
                          h264
                          pomp
                          rtp
                          transport-packet
                          ulog
                          video-metadata
        )
else()

target_link_libraries(${PROJECT_NAME}
                                PRIVATE
                                futils
                                h264
                                pomp
                                rtp
                                transport-packet
                                ulog
                                video-metadata
)

endif()

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    PUBLIC_HEADER DESTINATION include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
