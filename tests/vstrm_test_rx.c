/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"

#include <video-metadata/vmeta_frame_proto.h>


static void fill_nalu_bytes(uint8_t *buf, size_t len, uint8_t nal_type)
{
	buf[0] = (uint8_t)((3 << 5) | (nal_type & 0x1f));
	for (size_t i = 1; i < len; i++)
		buf[i] = (uint8_t)(i & 0xff);
}


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


static struct rtp_pkt *pkt_from_payload(uint8_t payload_type,
					uint16_t seq,
					uint32_t ts,
					bool marker,
					const uint8_t *payload,
					size_t payload_len)
{
	uint8_t raw[512];
	size_t rawlen = build_rtp_packet(payload_type,
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


/* Builds a full RTP packet with a proto-metadata header extension (the
 * "VSTRM_METADATA_PROTO_HEADER_LEN"-prefixed format written by
 * vstrm_rtp_h264_tx_add_proto_metadata() / read by process_extheader() in
 * src/vstrm_rtp_h264_rx.c: 2-byte id, 2-byte length-in-32bit-words, 2-byte
 * packed last_pack/current_pack/padding, 2-byte offset, then the content
 * bytes plus zero-padding out to a 4-byte boundary), followed by a plain
 * H264 payload. process_extheader()'s errors are silently ignored by its
 * caller, so the content bytes never need to be real protobuf -- this
 * tests the has_pack/set_pack/has_all_packs fragment-tracking framing,
 * not vmeta_frame_read2()'s protobuf decoding. */
static size_t build_proto_ext_rtp_packet(uint16_t seqnum,
					 uint32_t timestamp,
					 bool marker,
					 uint8_t nb_packs,
					 uint8_t current_pack,
					 uint16_t offset_val,
					 const uint8_t *content,
					 size_t content_len,
					 const uint8_t *payload,
					 size_t payload_len,
					 uint8_t *buf,
					 size_t cap)
{
	struct rtp_pkt *pkt = NULL;
	size_t pos;
	const void *cdata = NULL;
	size_t len = 0;
	int res;
	uint16_t id_be, len_words_be, offset_be, last_cur_pad;
	uint8_t padding;
	uint16_t data_after_prefix;
	uint16_t len_words;

	res = rtp_pkt_new(&pkt);
	if (res < 0)
		return 0;
	pkt->raw.buf = pomp_buffer_new(RTP_PKT_HEADER_SIZE);
	if (pkt->raw.buf == NULL) {
		rtp_pkt_destroy(pkt);
		return 0;
	}

	RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, VERSION, RTP_PKT_VERSION);
	RTP_PKT_HEADER_FLAGS_SET(
		pkt->header.flags, PAYLOAD_TYPE, VSTRM_RTP_H264_PAYLOAD_TYPE);
	RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, EXTENSION, 1);
	if (marker)
		RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, MARKER, 1);
	pkt->header.seqnum = seqnum;
	pkt->header.timestamp = timestamp;
	pkt->header.ssrc = 0x1234;

	pos = RTP_PKT_HEADER_SIZE;

	padding = (uint8_t)(4 - (content_len % 4));
	if (padding == 4)
		padding = 0;

	data_after_prefix = (uint16_t)(4 + content_len + padding);
	len_words = data_after_prefix / 4;
	id_be = htons(VMETA_FRAME_PROTO_RTP_EXT_ID);
	len_words_be = htons(len_words);
	last_cur_pad = vstrm_rtp_h264_meta_header_pack(
		(uint8_t)(nb_packs - 1), current_pack, padding);
	offset_be = htons(offset_val);

	pomp_buffer_write(pkt->raw.buf, &pos, &id_be, sizeof(id_be));
	pomp_buffer_write(
		pkt->raw.buf, &pos, &len_words_be, sizeof(len_words_be));
	pomp_buffer_write(
		pkt->raw.buf, &pos, &last_cur_pad, sizeof(last_cur_pad));
	pomp_buffer_write(pkt->raw.buf, &pos, &offset_be, sizeof(offset_be));
	pomp_buffer_write(pkt->raw.buf, &pos, content, content_len);
	if (padding > 0) {
		uint8_t zero[4] = {0};
		pomp_buffer_write(pkt->raw.buf, &pos, zero, padding);
	}

	pkt->payload.off = pos;
	if (payload_len > 0) {
		pomp_buffer_write(pkt->raw.buf, &pos, payload, payload_len);
	}
	pkt->payload.len = pos - pkt->payload.off;

	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &pkt->raw.len, NULL);
	pkt->raw.cdata = cdata;

	res = rtp_pkt_finalize_header(pkt);
	if (res < 0) {
		rtp_pkt_destroy(pkt);
		return 0;
	}

	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
	if (len > cap)
		len = cap;
	memcpy(buf, cdata, len);

	rtp_pkt_destroy(pkt);
	return len;
}


static void test_rx_proto_metadata_extheader_fragments(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	uint8_t content0[4] = {0xAA, 0xBB, 0xCC, 0xDD};
	uint8_t content1[4] = {0x11, 0x22, 0x33, 0x44};
	uint8_t sei[4];
	uint8_t raw[128];
	size_t rawlen;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	fill_nalu_bytes(sei, sizeof(sei), H264_NALU_TYPE_SEI);

	/* Fragment 0/2: has_pack(0) is false, set_pack(0) marks it,
	 * has_all_packs(last_pack=1) is false (pack 1 still missing) */
	rawlen = build_proto_ext_rtp_packet(1,
					    1000,
					    false,
					    2,
					    0,
					    0,
					    content0,
					    sizeof(content0),
					    sei,
					    sizeof(sei),
					    raw,
					    sizeof(raw));
	CU_ASSERT_TRUE_FATAL(rawlen > 0);
	pkt = make_rtp_pkt_from_raw(raw, rawlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* Re-sending the same fragment exercises has_pack()'s early-return
	 * ("already have it") branch */
	pkt = make_rtp_pkt_from_raw(raw, rawlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* Fragment 1/2 (the last one): completes has_all_packs(1) */
	rawlen = build_proto_ext_rtp_packet(2,
					    1000,
					    true,
					    2,
					    1,
					    sizeof(content0),
					    content1,
					    sizeof(content1),
					    sei,
					    sizeof(sei),
					    raw,
					    sizeof(raw));
	CU_ASSERT_TRUE_FATAL(rawlen > 0);
	pkt = make_rtp_pkt_from_raw(raw, rawlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	mock_rx_ctx_clear(&ctx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_proto_metadata_fragments_above_64_packs(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	uint8_t content[4];
	uint8_t sei[4];
	uint8_t raw[128];
	size_t rawlen;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);
	/* 65 packs (ids 0..64) is the smallest count that reaches pack_id=64,
	 * crossing from the pack_bf_low-only range into pack_bf_high
	 * (src/vstrm_rtp_h264_rx.c:2408-2456, both has_pack()'s "pack_id<128"
	 * branch and has_all_packs()'s "more than one [64-bit word]" branch
	 * were previously uncovered) */
	const uint8_t nb_packs = 65;

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	fill_nalu_bytes(sei, sizeof(sei), H264_NALU_TYPE_SEI);

	for (uint8_t i = 0; i < nb_packs; i++) {
		memset(content, (int)i, sizeof(content));
		rawlen = build_proto_ext_rtp_packet(
			(uint16_t)(i + 1),
			1000,
			(i == nb_packs - 1),
			nb_packs,
			i,
			(uint16_t)(i * sizeof(content)),
			content,
			sizeof(content),
			sei,
			sizeof(sei),
			raw,
			sizeof(raw));
		CU_ASSERT_TRUE_FATAL(rawlen > 0);
		pkt = make_rtp_pkt_from_raw(raw, rawlen);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}

	mock_rx_ctx_clear(&ctx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_set_codec_info_with_valid_sps_pps(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	struct vstrm_codec_info info;
	const struct vstrm_codec_info *got = NULL;

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	memset(&info, 0, sizeof(info));
	info.codec = VSTRM_CODEC_VIDEO_H264;
	info.h264.spslen = (uint32_t)h264_fixture_build_sps(
		&fx, info.h264.sps, sizeof(info.h264.sps));
	info.h264.ppslen = (uint32_t)h264_fixture_build_pps(
		&fx, info.h264.pps, sizeof(info.h264.pps));
	CU_ASSERT_TRUE_FATAL(info.h264.spslen > 0 && info.h264.ppslen > 0);

	/* Unlike the receiver-level set_codec_info test (which deliberately
	 * used a mismatched ssrc to bypass this), a real, parseable SPS/PPS
	 * here successfully drives sps_received()/pps_received(), the same
	 * functions the normal depayload path uses, firing
	 * codec_info_changed with the real derived width/height */
	res = vstrm_rtp_h264_rx_set_codec_info(rx, &info);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(ctx.codec_info_changed_count, 1);
	CU_ASSERT_EQUAL(ctx.codec_info.h264.width, 32);
	CU_ASSERT_EQUAL(ctx.codec_info.h264.height, 32);

	/* vstrm_codec_info_cmp() also compares width/height, which weren't
	 * known when `info` was first built above -- fill them in with the
	 * derived values just confirmed via codec_info_changed */
	info.h264.width = 32;
	info.h264.height = 32;

	res = vstrm_rtp_h264_rx_get_codec_info(rx, &got);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(got);
	CU_ASSERT_TRUE(vstrm_codec_info_cmp(got, &info));

	res = vstrm_rtp_h264_rx_set_codec_info(NULL, &info);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_rtp_h264_rx_set_codec_info(rx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_rtp_h264_rx_get_codec_info(rx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_rtp_h264_rx_get_codec_info(NULL, &got);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_stap_b_deaggregation(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	uint8_t sei[4], sei2[6];
	uint8_t stap[64];
	size_t pos = 0;
	uint16_t len_be;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Both aggregated members are SEI (not SPS/PPS): nalu_complete()'s
	 * "always really parse this NALU" set is {SLICE*, SPS, PPS, and the
	 * default case}; SEI is real-parsed only *conditionally* (only once
	 * sps.valid && pps.valid), which is false here since none were ever
	 * primed -- so these garbage-content SEIs are never actually run
	 * through h264_reader_parse_nalu(). Using a real SPS type here
	 * instead (as in the STAP-A test, which *does* prime real SPS/PPS
	 * first) would hit the unconditional SPS parse path and fail on
	 * this garbage content. */
	fill_nalu_bytes(sei, sizeof(sei), H264_NALU_TYPE_SEI);
	fill_nalu_bytes(sei2, sizeof(sei2), H264_NALU_TYPE_SEI);

	/* STAP-B is STAP-A plus a 2-byte DON right after the indicator byte;
	 * process_aggregation() itself ignores the DON value (UNUSED param),
	 * so deaggregation behaves identically to STAP-A past that offset */
	stap[pos++] = VSTRM_RTP_H264_NALU_TYPE_STAP_B;
	stap[pos++] = 0x00; /* DON hi */
	stap[pos++] = 0x00; /* DON lo */
	len_be = htons((uint16_t)sizeof(sei));
	memcpy(stap + pos, &len_be, 2);
	pos += 2;
	memcpy(stap + pos, sei, sizeof(sei));
	pos += sizeof(sei);
	len_be = htons((uint16_t)sizeof(sei2));
	memcpy(stap + pos, &len_be, 2);
	pos += 2;
	memcpy(stap + pos, sei2, sizeof(sei2));
	pos += sizeof(sei2);

	pkt = pkt_from_payload(
		VSTRM_RTP_H264_PAYLOAD_TYPE, 1, 1000, true, stap, pos);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* No SPS/PPS were primed, so au_complete() drops the frame silently
	 * (see the "waiting for sps/pps" gate) -- this test is only about
	 * the STAP-B depayload dispatch itself, not full frame output */
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	/* Malformed STAP-B (too short for even the DON field) is rejected */
	{
		uint8_t bad[2] = {VSTRM_RTP_H264_NALU_TYPE_STAP_B, 0};
		pkt = pkt_from_payload(
			VSTRM_RTP_H264_PAYLOAD_TYPE, 2, 1000, true, bad, 2);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
		CU_ASSERT_EQUAL(res, -EIO);
		rtp_pkt_destroy(pkt);
	}

	mock_rx_ctx_clear(&ctx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_fu_b_reassembly(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	uint8_t nalu[16];
	uint8_t frag[20];
	size_t frag_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* FU-B is FU-A plus a 2-byte DON right after fu_ind/fu_hdr; a single
	 * fragment can carry both the start and end bits at once */
	fill_nalu_bytes(nalu, sizeof(nalu), H264_NALU_TYPE_SLICE);
	frag[0] = (nalu[0] & 0xe0) | VSTRM_RTP_H264_NALU_TYPE_FU_B;
	frag[1] = (uint8_t)(0x80 | 0x40 | (nalu[0] & 0x1f)); /* start+end */
	frag[2] = 0x00; /* DON hi */
	frag[3] = 0x00; /* DON lo */
	memcpy(frag + 4, nalu + 1, sizeof(nalu) - 1);
	frag_len = 4 + (sizeof(nalu) - 1);

	pkt = pkt_from_payload(
		VSTRM_RTP_H264_PAYLOAD_TYPE, 1, 1000, true, frag, frag_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* Same "no sps/pps -> frame silently dropped" caveat as STAP-B */
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	/* Malformed FU-B (too short for fu_ind+fu_hdr+DON) is rejected --
	 * bad[0]'s low 5 bits must actually encode FU_B (29) for the
	 * dispatch switch to route here at all; a plain {0,0,0} instead
	 * decodes as nalu_type 0, falling into the default (single-NALU)
	 * case and succeeding trivially instead of being rejected */
	{
		uint8_t bad[3] = {VSTRM_RTP_H264_NALU_TYPE_FU_B, 0, 0};
		pkt = pkt_from_payload(
			VSTRM_RTP_H264_PAYLOAD_TYPE, 2, 1000, true, bad, 3);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
		CU_ASSERT_EQUAL(res, -EIO);
		rtp_pkt_destroy(pkt);
	}

	mock_rx_ctx_clear(&ctx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_single_nalu_passthrough(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[32];
	size_t slice_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);

	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       true,
			       slice,
			       slice_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 3);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[2].len, slice_len);
	CU_ASSERT_EQUAL(memcmp(ctx.frames[0]->nalus[2].cdata, slice, slice_len),
			0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_fu_a_reassembly(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	size_t split;
	uint8_t fu_ind, fu_hdr1, fu_hdr2;
	uint8_t frag1[66], frag2[66];
	size_t frag1_len, frag2_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len >= 4);

	split = 1 + (slice_len - 1) / 2;
	fu_ind = (slice[0] & 0xe0) | VSTRM_RTP_H264_NALU_TYPE_FU_A;
	fu_hdr1 = (uint8_t)(0x80 | (slice[0] & 0x1f)); /* start */
	fu_hdr2 = (uint8_t)(0x40 | (slice[0] & 0x1f)); /* end */

	frag1[0] = fu_ind;
	frag1[1] = fu_hdr1;
	memcpy(frag1 + 2, slice + 1, split - 1);
	frag1_len = 2 + (split - 1);

	frag2[0] = fu_ind;
	frag2[1] = fu_hdr2;
	memcpy(frag2 + 2, slice + split, slice_len - split);
	frag2_len = 2 + (slice_len - split);

	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       false,
			       frag1,
			       frag1_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       true,
			       frag2,
			       frag2_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 3);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[2].len, slice_len);
	CU_ASSERT_EQUAL(memcmp(ctx.frames[0]->nalus[2].cdata, slice, slice_len),
			0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_fu_a_orphaned_and_interrupted_fragment(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	uint8_t fu_ind, fu_hdr_start;
	uint8_t frag[66];
	size_t frag_len;
	uint8_t sei[64];
	size_t sei_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Orphaned continuation fragment: no preceding start fragment */
	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len >= 4);
	fu_ind = (slice[0] & 0xe0) | VSTRM_RTP_H264_NALU_TYPE_FU_A;
	frag[0] = fu_ind;
	frag[1] = (uint8_t)(0x40 | (slice[0] & 0x1f)); /* end, no start */
	memcpy(frag + 2, slice + 1, slice_len - 1);
	frag_len = 2 + (slice_len - 1);
	pkt = pkt_from_payload(
		VSTRM_RTP_H264_PAYLOAD_TYPE, seq++, 1000, true, frag, frag_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, -EIO);
	rtp_pkt_destroy(pkt);
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	/* Start a fragment, then interrupt it with an unrelated single-NALU
	 * packet (a type change, not a continuation): the pending fragment
	 * must be silently aborted, not corrupt the next real NALU */
	fu_hdr_start = (uint8_t)(0x80 | (slice[0] & 0x1f));
	frag[0] = fu_ind;
	frag[1] = fu_hdr_start;
	memcpy(frag + 2, slice + 1, slice_len - 1);
	frag_len = 2 + (slice_len - 1);
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       false,
			       frag,
			       frag_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* Since SPS/PPS are already valid at this point, nalu_complete()
	 * genuinely parses any non-slice NALU's RBSP (not just slices) --
	 * a garbage payload would fail that parse and never reach
	 * au_add_nalu()/au_complete() at all, so this must be a real,
	 * parseable SEI, not arbitrary bytes */
	sei_len = h264_fixture_build_sei_v2_hint(&fx, 1, sei, sizeof(sei));
	CU_ASSERT_TRUE_FATAL(sei_len > 0);
	pkt = pkt_from_payload(
		VSTRM_RTP_H264_PAYLOAD_TYPE, seq++, 1000, true, sei, sei_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	/* Frame completes with [SPS, PPS, SEI] -- the aborted fragment never
	 * became a NALU */
	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 3);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[2].len, sei_len);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_stap_a_deaggregation(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t sps[32], pps[32];
	size_t spslen, ppslen;
	uint8_t stap[128];
	size_t pos = 0;
	uint16_t len_be;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);
	uint8_t slice[32];
	size_t slice_len;

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	spslen = h264_fixture_build_sps(&fx, sps, sizeof(sps));
	ppslen = h264_fixture_build_pps(&fx, pps, sizeof(pps));
	CU_ASSERT_TRUE_FATAL(spslen > 0 && ppslen > 0);

	stap[pos++] = VSTRM_RTP_H264_NALU_TYPE_STAP_A;
	len_be = htons((uint16_t)spslen);
	memcpy(stap + pos, &len_be, 2);
	pos += 2;
	memcpy(stap + pos, sps, spslen);
	pos += spslen;
	len_be = htons((uint16_t)ppslen);
	memcpy(stap + pos, &len_be, 2);
	pos += 2;
	memcpy(stap + pos, pps, ppslen);
	pos += ppslen;

	pkt = pkt_from_payload(
		VSTRM_RTP_H264_PAYLOAD_TYPE, seq++, 1000, false, stap, pos);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.codec_info_changed_count, 1);
	CU_ASSERT_EQUAL(ctx.codec_info.h264.width, 32);
	CU_ASSERT_EQUAL(ctx.codec_info.h264.height, 32);

	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       true,
			       slice,
			       slice_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.frame_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 3);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[0].len, spslen);
	CU_ASSERT_EQUAL(memcmp(ctx.frames[0]->nalus[0].cdata, sps, spslen), 0);
	CU_ASSERT_EQUAL(ctx.frames[0]->nalus[1].len, ppslen);
	CU_ASSERT_EQUAL(memcmp(ctx.frames[0]->nalus[1].cdata, pps, ppslen), 0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_mtap_rejected(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	uint8_t payload[8] = {0};
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	payload[0] = VSTRM_RTP_H264_NALU_TYPE_MTAP16;
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       1,
			       1000,
			       true,
			       payload,
			       sizeof(payload));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, -EIO);
	rtp_pkt_destroy(pkt);

	payload[0] = VSTRM_RTP_H264_NALU_TYPE_MTAP24;
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       2,
			       1000,
			       true,
			       payload,
			       sizeof(payload));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, -EIO);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	mock_rx_ctx_clear(&ctx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_bad_payload_type_rejected(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	uint8_t sei[4];
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	fill_nalu_bytes(sei, sizeof(sei), H264_NALU_TYPE_SEI);
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE + 1,
			       1,
			       1000,
			       true,
			       sei,
			       sizeof(sei));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, -EIO);
	rtp_pkt_destroy(pkt);
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	mock_rx_ctx_clear(&ctx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_implicit_eof_on_timestamp_change(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice1[32];
	size_t slice1_len;
	uint8_t slice2[32];
	size_t slice2_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp ts1 = make_timestamp(1000);
	struct vstrm_timestamp ts2 = make_timestamp(2000);

	res = new_rx(&rx, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* First slice of frame 1, covering only half the macroblocks, with
	 * NO marker bit (the rest of the frame is never sent) */
	slice1_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 2, slice1, sizeof(slice1));
	CU_ASSERT_TRUE_FATAL(slice1_len > 0);
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       false,
			       slice1,
			       slice1_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts1);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	/* A NALU with a different RTP timestamp arrives without ever seeing
	 * a marker bit for frame 1 -- implicit end-of-frame must complete
	 * (and mark incomplete) frame 1 before processing this new NALU */
	slice2_len = h264_fixture_build_slice(
		&fx, false, 1, 0, 4, slice2, sizeof(slice2));
	CU_ASSERT_TRUE_FATAL(slice2_len > 0);
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       2000,
			       true,
			       slice2,
			       slice2_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts2);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL(ctx.frame_count, 2);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[0]->nalu_count, 3);
	CU_ASSERT_EQUAL(ctx.frames[0]->info.complete, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.frames[1]->nalu_count, 1);
	CU_ASSERT_EQUAL(ctx.frames[1]->nalus[0].len, slice2_len);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_full_mb_status_reparses_own_slice(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[32];
	size_t slice_len;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(1000);

	res = new_rx(&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Whole 2x2 (4 mb) frame in a single IDR (grey-I) slice, with
	 * CHECK_FLAG(self->cfg.flags, H264_FULL_MB_STATUS) enabled
	 * (src/vstrm_rtp_h264_rx.c:1864-1887): instead of the normal delayed
	 * previous-slice update, the slice's own data is immediately
	 * reparsed via H264_READER_FLAGS_SLICE_DATA, driving
	 * slice_data_mb_cb() for each real macroblock */
	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
			       seq++,
			       1000,
			       true,
			       slice,
			       slice_len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	CU_ASSERT_EQUAL(res, 0);
	rtp_pkt_destroy(pkt);

	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ctx.frames[0]->info.mb_status);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[0]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_VALID_ISLICE);
	}

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_rplm_short_term_idc0_and_idc1(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	struct rtp_pkt *pkt;
	struct h264_rplm rplm;

	res = new_rx(&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Frame 0: IDR reference frame, all mb VALID_ISLICE */
	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(1000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       1000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}

	/* Frame 1: non-IDR skipped-P, rplm idc=0 (subtract),
	 * abs_diff_pic_num_minus1=0 -> slice_cb()'s switch case 0
	 * (src/vstrm_rtp_h264_rx.c:764-778) */
	memset(&rplm, 0, sizeof(rplm));
	rplm.ref_pic_list_modification_flag_l0 = 1;
	rplm.pic_num_l0[0].modification_of_pic_nums_idc = 0;
	rplm.pic_num_l0[0].abs_diff_pic_num_minus1 = 0;
	rplm.pic_num_l0[1].modification_of_pic_nums_idc = 3; /* terminator */
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 1, 0, 4, &rplm, NULL, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(2000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       2000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 2);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[1]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_VALID_PSLICE);
	}
	CU_ASSERT_EQUAL(ctx.frames[1]->info.error, 0);

	/* Frame 2: non-IDR skipped-P, rplm idc=1 (add),
	 * abs_diff_pic_num_minus1=0 -> slice_cb()'s switch case 1
	 * (:779-793); same delta magnitude as above (0), only the sign
	 * differs, still resolves to an available short-term reference */
	memset(&rplm, 0, sizeof(rplm));
	rplm.ref_pic_list_modification_flag_l0 = 1;
	rplm.pic_num_l0[0].modification_of_pic_nums_idc = 1;
	rplm.pic_num_l0[0].abs_diff_pic_num_minus1 = 0;
	rplm.pic_num_l0[1].modification_of_pic_nums_idc = 3;
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 2, 0, 4, &rplm, NULL, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(3000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       3000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 3);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[2]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_VALID_PSLICE);
	}
	CU_ASSERT_EQUAL(ctx.frames[2]->info.error, 0);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_ltr_lifecycle_set_get_reset(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	struct rtp_pkt *pkt;
	struct h264_rplm rplm;
	struct h264_drpm drpm;

	res = new_rx(&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Frame A: IDR, marks itself as long-term reference pic_num=0 ->
	 * au_output()'s ref_long_set_and_copy(self, idxA, 0, [I,I,I,I])
	 * (src/vstrm_rtp_h264_rx.c:375-390) */
	memset(&drpm, 0, sizeof(drpm));
	drpm.long_term_reference_flag = 1;
	slice_len = h264_fixture_build_slice_ext(
		&fx, true, 0, 0, 4, NULL, &drpm, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(1000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       1000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 1);

	/* Frame B: non-IDR skipped-P, rplm idc=2 referencing
	 * long_term_pic_num=0 -> frame_update()'s ref_long_get()
	 * (:393-410) succeeds, and with H264_FULL_MB_STATUS its own
	 * reparse (slice_data_mb_cb -> frame_set_mb_status) checks
	 * frame->ref_mb_status against frame A's all-VALID_ISLICE copy */
	memset(&rplm, 0, sizeof(rplm));
	rplm.ref_pic_list_modification_flag_l0 = 1;
	rplm.pic_num_l0[0].modification_of_pic_nums_idc = 2;
	rplm.pic_num_l0[0].long_term_pic_num = 0;
	rplm.pic_num_l0[1].modification_of_pic_nums_idc = 3;
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 1, 0, 4, &rplm, NULL, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(2000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       2000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 2);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[1]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_VALID_PSLICE);
	}
	CU_ASSERT_EQUAL(ctx.frames[1]->info.error, 0);

	/* Frame D: non-IDR, drpm MMCO=2 (long-term reset) targeting the
	 * same long_term_pic_num=0 -> slice_cb()'s DRPM switch case 2
	 * (:833-845) sets ltr_reset/ltr_reset_pic_num, consumed by
	 * au_output()'s ref_long_reset() (:364-372) */
	memset(&drpm, 0, sizeof(drpm));
	drpm.adaptive_ref_pic_marking_mode_flag = 1;
	drpm.mm[0].memory_management_control_operation = 2;
	drpm.mm[0].long_term_pic_num = 0;
	drpm.mm[1].memory_management_control_operation = 0; /* terminator */
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 2, 0, 4, NULL, &drpm, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(3000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       3000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 3);

	/* Frame E: identical rplm idc=2 request as Frame B -> ref_long_get()
	 * now returns -ENOENT (reset by Frame D), so frame_set_mb_status()
	 * falls back to error propagation for every macroblock -- this is
	 * what proves ref_long_reset() actually took effect */
	memset(&rplm, 0, sizeof(rplm));
	rplm.ref_pic_list_modification_flag_l0 = 1;
	rplm.pic_num_l0[0].modification_of_pic_nums_idc = 2;
	rplm.pic_num_l0[0].long_term_pic_num = 0;
	rplm.pic_num_l0[1].modification_of_pic_nums_idc = 3;
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 3, 0, 4, &rplm, NULL, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(4000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       4000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 4);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[3]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION);
	}
	CU_ASSERT_EQUAL(ctx.frames[3]->info.error, 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_rx_ltr_set_via_mmco6(void)
{
	int res;
	struct vstrm_rtp_h264_rx *rx = NULL;
	struct mock_rx_ctx ctx;
	struct h264_fixture fx;
	uint16_t seq = 1;
	uint8_t slice[64];
	size_t slice_len;
	struct rtp_pkt *pkt;
	struct h264_rplm rplm;
	struct h264_drpm drpm;

	res = new_rx(&rx, &ctx, VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0 /* CAVLC */);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = prime_sps_pps_packets(rx, &fx, &seq, 1000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Frame 0: plain IDR, no drpm/rplm. Needed as a clean short-term
	 * reference for Frame A below -- with H264_FULL_MB_STATUS, a non-IDR
	 * frame's own mb_status is only set to VALID_PSLICE if
	 * frame_set_mb_status() finds a resolvable *own* reference
	 * (ref_short_get_at_delta()/ref_long_get(), src/vstrm_rtp_h264_rx.c:
	 * 434-458, 588-605); with none available (as a bare Frame A would
	 * have, being the very first frame ever) it gets marked
	 * ERROR_PROPAGATION instead, which would then poison Frame B's
	 * "get" below for the wrong reason */
	slice_len = h264_fixture_build_slice(
		&fx, true, 0, 0, 4, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(1000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       1000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 1);

	/* Frame A: non-IDR, drpm MMCO=6 marks itself as long-term reference
	 * with long_term_frame_idx=5 -> slice_cb()'s DRPM switch case 6
	 * (:870-873, previously uncovered) sets ltr/ltr_pic_num, consumed by
	 * au_output()'s ref_long_set_and_copy(self, idxA, 5, [P,P,P,P])
	 * (src/vstrm_rtp_h264_rx.c:375-390) -- unlike
	 * test_rx_ltr_lifecycle_set_get_reset() which sets the LTR via the
	 * IDR long_term_reference_flag path (always ltr_pic_num=0), MMCO=6
	 * is the only way a non-IDR frame can mark itself LTR, and it
	 * carries an arbitrary long_term_frame_idx. It resolves its own
	 * reference against Frame 0 (delta=0, the default), so its own
	 * mb_status ends up VALID_PSLICE, not ERROR_PROPAGATION */
	memset(&drpm, 0, sizeof(drpm));
	drpm.adaptive_ref_pic_marking_mode_flag = 1;
	drpm.mm[0].memory_management_control_operation = 6;
	drpm.mm[0].long_term_frame_idx = 5;
	drpm.mm[1].memory_management_control_operation = 0; /* terminator */
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 1, 0, 4, NULL, &drpm, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(2000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       2000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 2);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[1]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_VALID_PSLICE);
	}
	CU_ASSERT_EQUAL(ctx.frames[1]->info.error, 0);

	/* Frame B: rplm idc=2 referencing long_term_pic_num=5 (the exact
	 * value MMCO=6 stored) -> ref_long_get() succeeds */
	memset(&rplm, 0, sizeof(rplm));
	rplm.ref_pic_list_modification_flag_l0 = 1;
	rplm.pic_num_l0[0].modification_of_pic_nums_idc = 2;
	rplm.pic_num_l0[0].long_term_pic_num = 5;
	rplm.pic_num_l0[1].modification_of_pic_nums_idc = 3;
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 2, 0, 4, &rplm, NULL, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(3000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       3000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 3);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[2]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_VALID_PSLICE);
	}
	CU_ASSERT_EQUAL(ctx.frames[2]->info.error, 0);

	/* Frame C: rplm idc=2 referencing long_term_pic_num=99 (a value MMCO=6
	 * never stored) -> ref_long_get() returns -ENOENT, proving the
	 * pic_num actually came from long_term_frame_idx=5 and not some
	 * hardcoded/default value */
	memset(&rplm, 0, sizeof(rplm));
	rplm.ref_pic_list_modification_flag_l0 = 1;
	rplm.pic_num_l0[0].modification_of_pic_nums_idc = 2;
	rplm.pic_num_l0[0].long_term_pic_num = 99;
	rplm.pic_num_l0[1].modification_of_pic_nums_idc = 3;
	slice_len = h264_fixture_build_slice_ext(
		&fx, false, 3, 0, 4, &rplm, NULL, slice, sizeof(slice));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);
	{
		struct vstrm_timestamp ts = make_timestamp(4000);
		pkt = pkt_from_payload(VSTRM_RTP_H264_PAYLOAD_TYPE,
				       seq++,
				       4000,
				       true,
				       slice,
				       slice_len);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &ts);
		CU_ASSERT_EQUAL(res, 0);
		rtp_pkt_destroy(pkt);
	}
	CU_ASSERT_EQUAL_FATAL(ctx.frame_count, 4);
	for (uint32_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ctx.frames[3]->info.mb_status[i],
				VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION);
	}
	CU_ASSERT_EQUAL(ctx.frames[3]->info.error, 1);

	mock_rx_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_rtp_h264_rx_destroy(rx);
	CU_ASSERT_EQUAL(res, 0);
}


CU_TestInfo g_vstrm_test_rx[] = {
	{FN("rx-single-nalu-passthrough"), &test_rx_single_nalu_passthrough},
	{FN("rx-fu-a-reassembly"), &test_rx_fu_a_reassembly},
	{FN("rx-fu-a-orphaned-and-interrupted-fragment"),
	 &test_rx_fu_a_orphaned_and_interrupted_fragment},
	{FN("rx-stap-a-deaggregation"), &test_rx_stap_a_deaggregation},
	{FN("rx-mtap-rejected"), &test_rx_mtap_rejected},
	{FN("rx-bad-payload-type-rejected"),
	 &test_rx_bad_payload_type_rejected},
	{FN("rx-implicit-eof-on-timestamp-change"),
	 &test_rx_implicit_eof_on_timestamp_change},
	{FN("rx-proto-metadata-extheader-fragments"),
	 &test_rx_proto_metadata_extheader_fragments},
	{FN("rx-proto-metadata-fragments-above-64-packs"),
	 &test_rx_proto_metadata_fragments_above_64_packs},
	{FN("rx-set-codec-info-with-valid-sps-pps"),
	 &test_rx_set_codec_info_with_valid_sps_pps},
	{FN("rx-stap-b-deaggregation"), &test_rx_stap_b_deaggregation},
	{FN("rx-fu-b-reassembly"), &test_rx_fu_b_reassembly},
	{FN("rx-full-mb-status-reparses-own-slice"),
	 &test_rx_full_mb_status_reparses_own_slice},
	{FN("rx-rplm-short-term-idc0-and-idc1"),
	 &test_rx_rplm_short_term_idc0_and_idc1},
	{FN("rx-ltr-lifecycle-set-get-reset"),
	 &test_rx_ltr_lifecycle_set_get_reset},
	{FN("rx-ltr-set-via-mmco6"), &test_rx_ltr_set_via_mmco6},

	CU_TEST_INFO_NULL,
};
