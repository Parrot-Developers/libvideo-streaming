/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"


static int
new_rx(struct vstrm_rtp_h264_rx **rx, struct mock_rx_ctx *ctx, uint32_t flags)
{
	struct vstrm_rtp_h264_rx_cfg cfg = {0};
	struct vstrm_rtp_h264_rx_cbs cbs = {0};

	mock_rx_ctx_reset(ctx);
	cfg.flags = flags;
	cbs.userdata = ctx;
	cbs.recv_frame = &mock_rx_recv_frame_cb;
	cbs.codec_info_changed = &mock_rx_codec_info_changed_cb;
	return vstrm_rtp_h264_rx_new(&cfg, &cbs, rx);
}


static struct rtp_pkt *pkt_from_payload(uint16_t seq,
					uint32_t ts,
					bool marker,
					const uint8_t *payload,
					size_t payload_len)
{
	uint8_t raw[512];
	size_t rawlen = build_rtp_packet(VSTRM_RTP_H264_PAYLOAD_TYPE,
					 seq,
					 ts,
					 0x1234,
					 marker,
					 payload,
					 payload_len,
					 raw,
					 sizeof(raw));
	return make_rtp_pkt_from_raw(raw, rawlen);
}


static int send_sei_v2_hint(struct vstrm_rtp_h264_rx *rx,
			    struct h264_fixture *fx,
			    uint16_t *seq,
			    uint32_t ts,
			    uint16_t slice_mb_count)
{
	uint8_t sei[64];
	size_t sei_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(ts);
	int res;

	sei_len = h264_fixture_build_sei_v2_hint(
		fx, slice_mb_count, sei, sizeof(sei));
	if (sei_len == 0)
		return -EIO;
	pkt = pkt_from_payload((*seq)++, ts, false, sei, sei_len);
	if (pkt == NULL)
		return -ENOMEM;
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	rtp_pkt_destroy(pkt);
	return res;
}


/* Builds a fresh rx primed with SPS/PPS, then feeds two real slices with a
 * gap in between (gap=1 on the second slice's packet -- the seam identified
 * during architecture research: no need for a real dropped packet or
 * rtp_jitter, vstrm_rtp_h264_rx_process_packet() takes gap directly). A
 * "Parrot Streaming" v2 SEI hint (slice_mb_count=1) is sent right after
 * priming so handle_missing_slices() takes the SEI-hint branch instead of
 * needing a CAVLC bitstream reparse of slice1's raw data. With slice1's
 * first_mb_in_slice=0 and the hint=1, the gap covers mb [1, slice2_mb_start).
 */
static void setup_slice_gap(struct vstrm_rtp_h264_rx *rx,
			    struct h264_fixture *fx,
			    uint16_t *seq,
			    bool slice1_is_idr,
			    bool slice2_is_idr,
			    uint32_t slice2_mb_start,
			    uint8_t *slice1,
			    size_t *slice1_len,
			    uint8_t *slice2,
			    size_t *slice2_len)
{
	int res;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = prime_sps_pps_packets(rx, fx, seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = send_sei_v2_hint(rx, fx, seq, 1000, 1 /* slice1 covers 1 mb */);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	*slice1_len = h264_fixture_build_slice(
		fx, slice1_is_idr, 0, 0, 1, slice1, 64);
	CU_ASSERT_TRUE_FATAL(*slice1_len > 0);
	pkt = pkt_from_payload((*seq)++, 1000, false, slice1, *slice1_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	*slice2_len = h264_fixture_build_slice(
		fx, slice2_is_idr, 0, slice2_mb_start, 1, slice2, 64);
	CU_ASSERT_TRUE_FATAL(*slice2_len > 0);
	pkt = pkt_from_payload((*seq)++, 1000, true, slice2, *slice2_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(
		rx, pkt, 1 /* gap */, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);
}


static void test_concealment_slice_gap_generates_grey_i_slice(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice1[64], slice2[64];
	size_t slice1_len, slice2_len;

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* slice1 IDR -> the whole frame is considered IDR, so the gap must
	 * be concealed with a grey I slice, not a skipped P slice, even
	 * though slice2 itself is a regular (non-IDR) slice */
	setup_slice_gap(rx,
			&fx,
			&seq,
			true,
			false,
			2,
			slice1,
			&slice1_len,
			slice2,
			&slice2_len);

	/* [SPS, PPS, SEI-hint, slice1, SYNTHETIC, slice2] -- the SEI hint
	 * packet is itself a real NALU appended into the frame, not just a
	 * side-channel signal, so it counts too */
	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 6);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[4].cdata[0] & 0x1f,
			H264_NALU_TYPE_SLICE_IDR);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 1);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_slice_gap_generates_skipped_p_slice(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice1[64], slice2[64];
	size_t slice1_len, slice2_len;

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Neither slice is IDR -> the gap must be concealed with a skipped
	 * P slice */
	setup_slice_gap(rx,
			&fx,
			&seq,
			false,
			false,
			2,
			slice1,
			&slice1_len,
			slice2,
			&slice2_len);

	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 6);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[4].cdata[0] & 0x1f,
			H264_NALU_TYPE_SLICE);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 1);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_disabled_marks_frame_incomplete(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice1[64], slice2[64];
	size_t slice1_len, slice2_len;

	res = new_rx(&rx, &ctx, 0 /* concealment disabled */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	setup_slice_gap(rx,
			&fx,
			&seq,
			true,
			false,
			2,
			slice1,
			&slice1_len,
			slice2,
			&slice2_len);

	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	/* No synthetic slice is added: [SPS, PPS, SEI-hint, slice1, slice2] */
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 5);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 1);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_cabac_without_sei_hint_skips_synthesis(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice1[64], slice2[64];
	size_t slice1_len, slice2_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 1 /* CABAC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* No SEI hint sent this time */

	slice1_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 1, slice1, sizeof(slice1));
	CU_ASSERT_TRUE_FATAL(slice1_len > 0);
	pkt = pkt_from_payload(seq++, 1000, false, slice1, slice1_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	slice2_len = h264_fixture_build_slice(
		&fx, false, 0, 2, 1, slice2, sizeof(slice2));
	CU_ASSERT_TRUE_FATAL(slice2_len > 0);
	pkt = pkt_from_payload(seq++, 1000, true, slice2, slice2_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(
		rx, pkt, 1 /* gap */, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	/* CABAC without a SEI hint: synthesis is skipped entirely (a real,
	 * currently-permanent limitation), frame is just marked incomplete;
	 * note info.error is NOT set here, unlike the "disabled" case above
	 * -- handle_missing_slices() exits before ever reaching that line */
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 4);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 0);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_mb_ordering_mismatch_marks_error(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice1[64], slice2[64];
	size_t slice1_len, slice2_len;

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* SEI hint says slice1 covers 3 mb (mb_start becomes 0+3=3), but
	 * slice2 claims to start at mb 1: 3 >= 1 is a mismatch */
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = send_sei_v2_hint(rx, &fx, &seq, 1000, 3);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	{
		struct rtp_pkt *pkt;
		struct vstrm_timestamp timestamp = make_timestamp(1000);

		slice1_len = h264_fixture_build_slice(
			&fx, true, 0, 0, 1, slice1, sizeof(slice1));
		CU_ASSERT_TRUE_FATAL(slice1_len > 0);
		pkt = pkt_from_payload(seq++, 1000, false, slice1, slice1_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);

		slice2_len = h264_fixture_build_slice(
			&fx, false, 0, 1, 1, slice2, sizeof(slice2));
		CU_ASSERT_TRUE_FATAL(slice2_len > 0);
		pkt = pkt_from_payload(seq++, 1000, true, slice2, slice2_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(
			rx, pkt, 1 /* gap */, &timestamp);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}

	/* [SPS, PPS, SEI-hint, slice1, slice2] -- mismatch aborts before any
	 * synthetic NALU is generated */
	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 5);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 1);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_frame_level_missing_frames(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	struct rtp_pkt *pkt;

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_FRAME);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Frame A: frame_num=0 (the very first AU: check_missing_frames()
	 * never runs for it, both because au.first is still true and because
	 * frame_num==0 is defined as "can't tell how many were missed") */
	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(1000);
		pkt = pkt_from_payload(seq++, 1000, true, slice, slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL(ctx.frame_count, 1);

	/* Frame B: frame_num=3 -> 2 frames (frame_num 1 and 2) missing,
	 * synthesized as skipped-P frames before Frame B itself completes */
	slice_len = h264_fixture_build_slice(
		&fx, false, 3, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(2000);
		pkt = pkt_from_payload(seq++, 2000, true, slice, slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}

	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 4);
	CU_ASSERT_EQUAL(ctx.frames[1]->info.gen_concealment, 1);
	CU_ASSERT_EQUAL(ctx.frames[1]->info.ref, 1);
	CU_ASSERT_EQUAL(ctx.frames[1]->info.error, 1);
	CU_ASSERT_EQUAL(ctx.frames[1]->info.complete, 1);
	CU_ASSERT_EQUAL(ctx.frames[2]->info.gen_concealment, 1);
	CU_ASSERT_PTR_NOT_NULL(ctx.frames[3]);

	/* Cap check: frame_num jumps from 3 to 43 (39 missing), synthesis
	 * must stop at exactly MAX_FRAME_CONCEALMENT=30 */
	slice_len = h264_fixture_build_slice(
		&fx, false, 43, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(3000);
		pkt = pkt_from_payload(seq++, 3000, true, slice, slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	/* +30 synthetic frames + Frame C itself */
	CU_ASSERT_EQUAL(ctx.frame_count, 4 + 30 + 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_grey_idr_before_first_real_idr(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp ts = make_timestamp(1000);

	res = new_rx(&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_GREY_IDR_FRAME);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* First real slice ever is NOT IDR -> a synthetic grey IDR frame is
	 * generated first (au.first is still true at this point) */
	slice_len = h264_fixture_build_slice(
		&fx, false, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	pkt = pkt_from_payload(seq++, 1000, true, slice, slice_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 2);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.gen_grey_idr, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 3);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[2].cdata[0] & 0x1f,
			H264_NALU_TYPE_SLICE_IDR);
	CU_ASSERT_EQUAL(ctx.frames[1]->info.gen_grey_idr, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[1]->nalu_count, 3);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_sei_v4_hint_used_for_gap_sizing(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t sei[64], slice1[64], slice2[64];
	size_t sei_len, slice1_len, slice2_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* "Parrot Streaming" v4 hint (no recovery point in play): slice1
	 * covers 1 mb. Exercises sei_user_data_unregistered_cb()'s v4 branch
	 * (src/vstrm_rtp_h264_rx.c:1051-1058) and handle_missing_slices()'s
	 * info_v4.valid branch (:1943-1951), used in preference to a CAVLC
	 * bitstream reparse -- same shape as the v2-hint test above, but
	 * v4 is the format actually used in production */
	sei_len = h264_fixture_build_sei_v4_hint(&fx, 1, 0, sei, sizeof(sei));
	CU_ASSERT_TRUE_FATAL(sei_len > 0);
	pkt = pkt_from_payload(seq++, 1000, false, sei, sei_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	slice1_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 1, slice1, sizeof(slice1));
	CU_ASSERT_TRUE_FATAL(slice1_len > 0);
	pkt = pkt_from_payload(seq++, 1000, false, slice1, slice1_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* slice2 starts at mb 2 -> gap is exactly [1, 2), matching the v4
	 * hint's slice_mb_count=1 */
	slice2_len = h264_fixture_build_slice(
		&fx, false, 0, 2, 1, slice2, sizeof(slice2));
	CU_ASSERT_TRUE_FATAL(slice2_len > 0);
	pkt = pkt_from_payload(seq++, 1000, true, slice2, slice2_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(
		rx, pkt, 1 /* gap */, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* [SPS, PPS, SEI-v4-hint, slice1, SYNTHETIC, slice2] */
	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 6);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[4].cdata[0] & 0x1f,
			H264_NALU_TYPE_SLICE_IDR);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 1);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_concealment_sei_v4_recovery_point_uses_recovery_count(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t sei[64], rec[16], slice1[64], slice2[64];
	size_t sei_len, rec_len, slice1_len, slice2_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(
		&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* 3 mb wide so mb 0, 1 and 2 all exist */
	res = h264_fixture_new(&fx, 3, 1, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Deliberately wrong slice_mb_count=5 (would make mb_start=0+5=5,
	 * a bogus mismatch against slice2's mb_start=2 below) vs. the
	 * correct slice_mb_count_recovery_point=1. The recovery-point value
	 * is only used if current_frame->recovery_point is set
	 * (src/vstrm_rtp_h264_rx.c:1946-1951), itself only set via a real
	 * recovery-point SEI (sei_recovery_point_cb, :1006-1019) landing on
	 * this frame before the gap is processed. */
	sei_len = h264_fixture_build_sei_v4_hint(&fx, 5, 1, sei, sizeof(sei));
	CU_ASSERT_TRUE_FATAL(sei_len > 0);
	pkt = pkt_from_payload(seq++, 1000, false, sei, sei_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	rec_len = h264_fixture_build_sei_recovery_point(&fx, rec, sizeof(rec));
	CU_ASSERT_TRUE_FATAL(rec_len > 0);
	pkt = pkt_from_payload(seq++, 1000, false, rec, rec_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	slice1_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 1, slice1, sizeof(slice1));
	CU_ASSERT_TRUE_FATAL(slice1_len > 0);
	pkt = pkt_from_payload(seq++, 1000, false, slice1, slice1_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	slice2_len = h264_fixture_build_slice(
		&fx, false, 0, 2, 1, slice2, sizeof(slice2));
	CU_ASSERT_TRUE_FATAL(slice2_len > 0);
	pkt = pkt_from_payload(seq++, 1000, true, slice2, slice2_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(
		rx, pkt, 1 /* gap */, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* If the recovery-point count (1) was used: gap=[1,2), synthesis
	 * succeeds -> [SPS, PPS, SEI-v4-hint, recovery-pt SEI, slice1,
	 * SYNTHETIC, slice2] and complete=1. If the wrong count (5) had been
	 * used instead, mb_start=5 >= mb_end=2 would abort as a mismatch,
	 * leaving complete=0 and no synthetic NALU -- this is the behavior
	 * this test pins down. */
	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 7);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 1);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.error, 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


CU_TestInfo g_vstrm_test_concealment[] = {
	{FN("concealment-slice-gap-generates-grey-i-slice"),
	 &test_concealment_slice_gap_generates_grey_i_slice},
	{FN("concealment-slice-gap-generates-skipped-p-slice"),
	 &test_concealment_slice_gap_generates_skipped_p_slice},
	{FN("concealment-disabled-marks-frame-incomplete"),
	 &test_concealment_disabled_marks_frame_incomplete},
	{FN("concealment-cabac-without-sei-hint-skips-synthesis"),
	 &test_concealment_cabac_without_sei_hint_skips_synthesis},
	{FN("concealment-mb-ordering-mismatch-marks-error"),
	 &test_concealment_mb_ordering_mismatch_marks_error},
	{FN("concealment-frame-level-missing-frames"),
	 &test_concealment_frame_level_missing_frames},
	{FN("concealment-grey-idr-before-first-real-idr"),
	 &test_concealment_grey_idr_before_first_real_idr},
	{FN("concealment-sei-v4-hint-used-for-gap-sizing"),
	 &test_concealment_sei_v4_hint_used_for_gap_sizing},
	{FN("concealment-sei-v4-recovery-point-uses-recovery-count"),
	 &test_concealment_sei_v4_recovery_point_uses_recovery_count},

	CU_TEST_INFO_NULL,
};
