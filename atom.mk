
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libvideo-streaming
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video streaming library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DVSTRM_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	src/vstrm_clock_delta.c \
	src/vstrm_dbg.c \
	src/vstrm_event.c \
	src/vstrm_frame.c \
	src/vstrm_frame_from_mbuf.c \
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
	libmedia-buffers \
	libpomp \
	librtp \
	libtransport-packet \
	libulog \
	libvideo-defs \
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
	tools/vstrm_test.c \
	tools/vstrm_test_receiver.c \
	tools/vstrm_test_sender.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libtransport-packet \
	libtransport-socket \
	libulog \
	libvideo-defs \
	libvideo-streaming

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)


ifdef TARGET_TEST

include $(CLEAR_VARS)
LOCAL_MODULE := tst-libvideo-streaming
LOCAL_CFLAGS += -DTARGET_TEST -D_GNU_SOURCE
# No longer depend on the libvideo-streaming module (see below), so its
# LOCAL_EXPORT_C_INCLUDES (include/) must be added here explicitly too.
LOCAL_C_INCLUDES := $(LOCAL_PATH)/src $(LOCAL_PATH)/include
# libvideo-streaming is built as a shared library with -fvisibility=hidden,
# exporting only its VSTRM_API-tagged public functions; the internal
# functions this suite exercises (vstrm_rtp_h264_{rx,tx}_*,
# vstrm_clock_delta_*, vstrm_event_{read,write}, vstrm_video_stats_{read,write},
# vstrm_session_metadata_*) are compiled but never exported, so linking
# against the library for them fails. Compile the library's own sources
# directly into this whitebox test binary instead of linking against it.
LOCAL_SRC_FILES := \
	src/vstrm_clock_delta.c \
	src/vstrm_dbg.c \
	src/vstrm_event.c \
	src/vstrm_frame.c \
	src/vstrm_frame_from_mbuf.c \
	src/vstrm_h264_sei_streaming.c \
	src/vstrm_receiver.c \
	src/vstrm_rtp_h264_rx.c \
	src/vstrm_rtp_h264_tx.c \
	src/vstrm_sender.c \
	src/vstrm_session_metadata.c \
	src/vstrm_video_stats.c \
	tests/vstrm_test_frame.c \
	tests/vstrm_test_dbg.c \
	tests/vstrm_test_internal.c \
	tests/vstrm_test_tx.c \
	tests/vstrm_test_rx.c \
	tests/vstrm_test_concealment.c \
	tests/vstrm_test_sender.c \
	tests/vstrm_test_receiver.c \
	tests/vstrm_test.c
LOCAL_LIBRARIES := \
	libcunit \
	libfutils \
	libh264 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	librtp \
	libtransport-packet \
	libulog \
	libvideo-defs \
	libvideo-metadata

include $(BUILD_EXECUTABLE)

endif
