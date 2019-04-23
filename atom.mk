
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libvideo-streaming
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video streaming library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DVSTRM_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/vstrm_clock_delta.c \
	src/vstrm_dbg.c \
	src/vstrm_frame.c \
	src/vstrm_h264_sei_streaming.c \
	src/vstrm_receiver.c \
	src/vstrm_rtp_h264_rx.c \
	src/vstrm_rtp_h264_tx.c \
	src/vstrm_sender.c \
	src/vstrm_session_metadata.c \
	src/vstrm_video_stats.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libpomp \
	librtp \
	libulog \
	libvideo-metadata

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := vstrm-test
LOCAL_DESCRIPTION := Video streaming library test program
LOCAL_CATEGORY_PATH := libs/streaming
LOCAL_SRC_FILES := \
	tests/vstrm_test.c \
	tests/vstrm_test_receiver.c \
	tests/vstrm_test_sender.c \
	tests/vstrm_test_socket.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libpomp \
	libulog \
	libvideo-buffers \
	libvideo-buffers-generic \
	libvideo-streaming

include $(BUILD_EXECUTABLE)
