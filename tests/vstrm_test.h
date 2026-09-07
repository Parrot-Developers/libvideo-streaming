/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#ifndef _VSTRM_TEST_H_
#define _VSTRM_TEST_H_

#include <transport-packet/tpkt.h>
#include <video-streaming/vstrm.h>

/* Internal headers (LOCAL_C_INCLUDES reaches src/); vstrm_priv.h itself
 * pulls in vstrm_rtp_h264.h, vstrm_clock_delta.h, vstrm_event.h,
 * vstrm_rtcp_app.h, vstrm_video_stats_priv.h, <h264/h264.h>, <rtp/rtp.h>,
 * <futils/futils.h> and <libpomp.h> */
#include "vstrm_priv.h"

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define FN(_name) (char *)_name

#define VSTRM_TEST_MOCK_MAX_CAPTURED 32


extern CU_TestInfo g_vstrm_test_frame[];
extern CU_TestInfo g_vstrm_test_dbg[];
extern CU_TestInfo g_vstrm_test_internal[];
extern CU_TestInfo g_vstrm_test_tx[];
extern CU_TestInfo g_vstrm_test_rx[];
extern CU_TestInfo g_vstrm_test_concealment[];
extern CU_TestInfo g_vstrm_test_sender[];
extern CU_TestInfo g_vstrm_test_receiver[];


/* Wraps a struct h264_ctx used purely to build minimal but syntactically
 * real H.264 NAL units (SPS/PPS/slice/SEI) via libh264's own writer API --
 * the same mechanism src/vstrm_rtp_h264_rx.c itself uses to generate
 * concealment slices. */
struct h264_fixture {
	struct h264_ctx *ctx;
	struct h264_sps sps;
	struct h264_pps pps;
};


int h264_fixture_new(struct h264_fixture *fx,
		     uint32_t mb_width,
		     uint32_t mb_height,
		     int entropy_coding_mode_flag);
void h264_fixture_clear(struct h264_fixture *fx);

/* Builds a real SPS NALU (no start code) into buf, returns its length */
size_t
h264_fixture_build_sps(struct h264_fixture *fx, uint8_t *buf, size_t cap);

/* Builds a real PPS NALU (no start code) into buf, returns its length */
size_t
h264_fixture_build_pps(struct h264_fixture *fx, uint8_t *buf, size_t cap);

/* Builds a real slice NALU (grey-I if is_idr, else skipped-P) covering
 * [mb_start, mb_start + mb_count) into buf, returns its length */
size_t h264_fixture_build_slice(struct h264_fixture *fx,
				bool is_idr,
				uint32_t frame_num,
				uint32_t mb_start,
				uint32_t mb_count,
				uint8_t *buf,
				size_t cap);

/* Builds a "Parrot Streaming" v2 user-data-unregistered SEI NALU giving an
 * explicit slice_mb_count hint, into buf, returns its length */
size_t h264_fixture_build_sei_v2_hint(struct h264_fixture *fx,
				      uint16_t slice_mb_count,
				      uint8_t *buf,
				      size_t cap);

/* Builds a "Parrot Streaming" v4 user-data-unregistered SEI NALU giving both
 * a normal and a recovery-point slice_mb_count hint, into buf, returns its
 * length */
size_t h264_fixture_build_sei_v4_hint(struct h264_fixture *fx,
				      uint16_t slice_mb_count,
				      uint16_t slice_mb_count_recovery_point,
				      uint8_t *buf,
				      size_t cap);

/* Builds a real recovery-point SEI NALU into buf, returns its length */
size_t h264_fixture_build_sei_recovery_point(struct h264_fixture *fx,
					     uint8_t *buf,
					     size_t cap);

/* Same as h264_fixture_build_slice(), but lets the caller additionally set
 * the reference-picture-list-modification (rplm) and decoded-reference-
 * picture-marking (drpm) slice header fields -- either pointer may be NULL
 * to leave the corresponding fields zeroed (no rplm/no adaptive marking) */
size_t h264_fixture_build_slice_ext(struct h264_fixture *fx,
				    bool is_idr,
				    uint32_t frame_num,
				    uint32_t mb_start,
				    uint32_t mb_count,
				    const struct h264_rplm *rplm,
				    const struct h264_drpm *drpm,
				    uint8_t *buf,
				    size_t cap);


/* Frame fixtures */
struct vstrm_frame *make_tagged_frame(uint64_t ntp_ts);
struct vstrm_frame_nalu make_nalu(const uint8_t *data,
				  size_t len,
				  uint32_t priority,
				  uint32_t importance);


/* RTP packet fixtures: builds a full raw RTP packet (header + optional
 * payload) as bytes, following the manual header-flags + pomp_buffer_write +
 * rtp_pkt_finalize_header sequence used by src/vstrm_rtp_h264_tx.c /
 * src/vstrm_sender.c. Returns the number of bytes written into buf. */
size_t build_rtp_packet(uint8_t payload_type,
			uint16_t seqnum,
			uint32_t timestamp,
			uint32_t ssrc,
			bool marker,
			const uint8_t *payload,
			size_t payload_len,
			uint8_t *buf,
			size_t cap);

/* Wraps raw RTP bytes into a tpkt_packet (copies the data) */
struct tpkt_packet *make_tpkt_from_bytes(const uint8_t *buf, size_t len);

/* Parses raw RTP bytes into a struct rtp_pkt (via rtp_pkt_read()), for
 * driving vstrm_rtp_h264_rx_process_packet() directly. Sets the returned
 * pkt's rtp_timestamp from its (unwrapped) header timestamp -- fine for
 * short tests that never cross a 32-bit wraparound. Caller must
 * rtp_pkt_destroy() the result. */
struct rtp_pkt *make_rtp_pkt_from_raw(const uint8_t *buf, size_t len);

/* Builds a struct vstrm_timestamp with every field set to the same value,
 * good enough for tests that don't care about clock-skew computations */
struct vstrm_timestamp make_timestamp(uint64_t ts);


/* Mock cbs.send_ctrl capture context, shared shape for both
 * vstrm_sender_cbs.send_ctrl and vstrm_receiver_cbs.send_ctrl (same
 * signature modulo the first parameter's type, which is opaque here). */
struct mock_ctrl_ctx {
	struct tpkt_packet *captured[VSTRM_TEST_MOCK_MAX_CAPTURED];
	int count;
	int fail_res; /* 0 = never fail */
};

void mock_ctrl_ctx_reset(struct mock_ctrl_ctx *ctx);
void mock_ctrl_ctx_clear(struct mock_ctrl_ctx *ctx);
int mock_sender_send_ctrl_cb(struct vstrm_sender *stream,
			     struct tpkt_packet *pkt,
			     void *userdata);
int mock_receiver_send_ctrl_cb(struct vstrm_receiver *stream,
			       struct tpkt_packet *pkt,
			       void *userdata);


#define VSTRM_TEST_MOCK_MAX_FRAMES 16


/* Mock vstrm_rtp_h264_rx_cbs capture context: shared by vstrm_test_rx.c
 * and vstrm_test_concealment.c, which both drive the rx layer directly
 * (bypassing vstrm_receiver / rtp_jitter). Captured frames are ref'd so
 * they survive past the callback for post-hoc assertions; the test must
 * call mock_rx_ctx_clear() to drop those extra refs. */
struct mock_rx_ctx {
	struct vstrm_frame *frames[VSTRM_TEST_MOCK_MAX_FRAMES];
	int frame_count;
	struct vstrm_codec_info codec_info;
	int codec_info_changed_count;
};

void mock_rx_ctx_reset(struct mock_rx_ctx *ctx);
void mock_rx_ctx_clear(struct mock_rx_ctx *ctx);
void mock_rx_recv_frame_cb(struct vstrm_rtp_h264_rx *rtp_h264_rx,
			   struct vstrm_frame *frame,
			   void *userdata);
void mock_rx_codec_info_changed_cb(struct vstrm_rtp_h264_rx *rtp_h264_rx,
				   const struct vstrm_codec_info *info,
				   void *userdata);


/* Feeds a real SPS then a real PPS (built via the h264_fixture, as two
 * separate single-NALU RTP packets, non-marker) into rx -- required before
 * any slice NALU if the test wants au_complete() to actually output a frame
 * via recv_frame instead of silently dropping it (vstrm_rtp_h264_rx_.c's
 * au_complete() drops any frame while !sps.valid || !pps.valid). Advances
 * *seq by 2. Returns 0 on success, negative errno otherwise. */
int prime_sps_pps_packets(struct vstrm_rtp_h264_rx *rx,
			  struct h264_fixture *fx,
			  uint16_t *seq,
			  uint32_t ts);


/* Debug-file helpers: vstrm_dbg_create_file() (src/vstrm_dbg.c) names files
 * with an unpredictable timestamp/pid/pointer prefix, so tests locate them
 * by suffix instead (e.g. "_sender_stream.bin"). Returns the matching
 * file's size in bytes, or -1 if no matching file is found in `dir`. */
long find_dbg_file_size(const char *dir, const char *suffix);

/* Removes every regular file directly inside `dir`, then rmdir()s it.
 * Best-effort cleanup for a mkdtemp()'d test directory. */
void remove_dbg_dir(const char *dir);


#endif /* !_VSTRM_TEST_H_ */
