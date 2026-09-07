/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"


/*
 * Shared fixtures
 */

int h264_fixture_new(struct h264_fixture *fx,
		     uint32_t mb_width,
		     uint32_t mb_height,
		     int entropy_coding_mode_flag)
{
	int res;

	memset(fx, 0, sizeof(*fx));

	res = h264_ctx_new(&fx->ctx);
	if (res < 0)
		return res;

	fx->sps.profile_idc = H264_PROFILE_BASELINE;
	fx->sps.chroma_format_idc = 1; /* 4:2:0 */
	fx->sps.log2_max_frame_num_minus4 = 4;
	fx->sps.pic_order_cnt_type = 2;
	fx->sps.max_num_ref_frames = 1;
	fx->sps.pic_width_in_mbs_minus1 = mb_width - 1;
	fx->sps.pic_height_in_map_units_minus1 = mb_height - 1;
	fx->sps.frame_mbs_only_flag = 1;

	fx->pps.pic_parameter_set_id = 0;
	fx->pps.seq_parameter_set_id = 0;
	fx->pps.entropy_coding_mode_flag = entropy_coding_mode_flag;

	res = h264_ctx_set_sps(fx->ctx, &fx->sps);
	if (res < 0)
		goto error;
	res = h264_ctx_set_pps(fx->ctx, &fx->pps);
	if (res < 0)
		goto error;

	return 0;

error:
	h264_ctx_destroy(fx->ctx);
	fx->ctx = NULL;
	return res;
}


void h264_fixture_clear(struct h264_fixture *fx)
{
	if (fx->ctx != NULL)
		h264_ctx_destroy(fx->ctx);
	memset(fx, 0, sizeof(*fx));
}


size_t h264_fixture_build_sps(struct h264_fixture *fx, uint8_t *buf, size_t cap)
{
	struct h264_nalu_header nh = {.nal_ref_idc = 3,
				      .nal_unit_type = H264_NALU_TYPE_SPS};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


size_t h264_fixture_build_pps(struct h264_fixture *fx, uint8_t *buf, size_t cap)
{
	struct h264_nalu_header nh = {.nal_ref_idc = 3,
				      .nal_unit_type = H264_NALU_TYPE_PPS};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


size_t h264_fixture_build_slice(struct h264_fixture *fx,
				bool is_idr,
				uint32_t frame_num,
				uint32_t mb_start,
				uint32_t mb_count,
				uint8_t *buf,
				size_t cap)
{
	struct h264_nalu_header nh = {0};
	struct h264_slice_header sh = {0};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	nh.nal_ref_idc = 3;
	nh.nal_unit_type =
		is_idr ? H264_NALU_TYPE_SLICE_IDR : H264_NALU_TYPE_SLICE;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	sh.first_mb_in_slice = mb_start;
	sh.slice_type = is_idr ? H264_SLICE_TYPE_I : H264_SLICE_TYPE_P;
	sh.pic_parameter_set_id = fx->pps.pic_parameter_set_id;
	sh.frame_num = frame_num;
	sh.disable_deblocking_filter_idc = 2;
	res = h264_ctx_set_slice_header(fx->ctx, &sh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = is_idr ? h264_write_grey_i_slice(&bs, fx->ctx, mb_count)
		     : h264_write_skipped_p_slice(&bs, fx->ctx, mb_count);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


size_t h264_fixture_build_sei_v2_hint(struct h264_fixture *fx,
				      uint16_t slice_mb_count,
				      uint8_t *buf,
				      size_t cap)
{
	struct vstrm_h264_sei_streaming_v2 v2 = {
		.slice_count = 1, .slice_mb_count = slice_mb_count};
	uint8_t uuid[16];
	uint8_t payload[32];
	size_t paylen = sizeof(payload);
	struct h264_sei sei = {0};
	struct h264_nalu_header nh = {.nal_ref_idc = 0,
				      .nal_unit_type = H264_NALU_TYPE_SEI};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	res = vstrm_h264_sei_streaming_v2_write(&v2, uuid, payload, &paylen);
	if (res < 0)
		return 0;

	sei.type = H264_SEI_TYPE_USER_DATA_UNREGISTERED;
	memcpy(sei.user_data_unregistered.uuid, uuid, 16);
	sei.user_data_unregistered.buf = payload;
	sei.user_data_unregistered.len = paylen;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;
	res = h264_ctx_add_sei(fx->ctx, &sei);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


size_t h264_fixture_build_sei_v4_hint(struct h264_fixture *fx,
				      uint16_t slice_mb_count,
				      uint16_t slice_mb_count_recovery_point,
				      uint8_t *buf,
				      size_t cap)
{
	struct vstrm_h264_sei_streaming_v4 v4 = {
		.slice_mb_count = slice_mb_count,
		.slice_mb_count_recovery_point = slice_mb_count_recovery_point};
	uint8_t uuid[16];
	uint8_t payload[32];
	size_t paylen = sizeof(payload);
	struct h264_sei sei = {0};
	struct h264_nalu_header nh = {.nal_ref_idc = 0,
				      .nal_unit_type = H264_NALU_TYPE_SEI};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	res = vstrm_h264_sei_streaming_v4_write(&v4, uuid, payload, &paylen);
	if (res < 0)
		return 0;

	sei.type = H264_SEI_TYPE_USER_DATA_UNREGISTERED;
	memcpy(sei.user_data_unregistered.uuid, uuid, 16);
	sei.user_data_unregistered.buf = payload;
	sei.user_data_unregistered.len = paylen;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;
	res = h264_ctx_add_sei(fx->ctx, &sei);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


size_t h264_fixture_build_sei_recovery_point(struct h264_fixture *fx,
					     uint8_t *buf,
					     size_t cap)
{
	struct h264_sei sei = {0};
	struct h264_nalu_header nh = {.nal_ref_idc = 0,
				      .nal_unit_type = H264_NALU_TYPE_SEI};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	sei.type = H264_SEI_TYPE_RECOVERY_POINT;
	sei.recovery_point.recovery_frame_cnt = 0;
	sei.recovery_point.exact_match_flag = 1;
	sei.recovery_point.broken_link_flag = 0;
	sei.recovery_point.changing_slice_group_idc = 0;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;
	res = h264_ctx_add_sei(fx->ctx, &sei);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


size_t h264_fixture_build_slice_ext(struct h264_fixture *fx,
				    bool is_idr,
				    uint32_t frame_num,
				    uint32_t mb_start,
				    uint32_t mb_count,
				    const struct h264_rplm *rplm,
				    const struct h264_drpm *drpm,
				    uint8_t *buf,
				    size_t cap)
{
	struct h264_nalu_header nh = {0};
	struct h264_slice_header sh = {0};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	nh.nal_ref_idc = 3;
	nh.nal_unit_type =
		is_idr ? H264_NALU_TYPE_SLICE_IDR : H264_NALU_TYPE_SLICE;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	sh.first_mb_in_slice = mb_start;
	sh.slice_type = is_idr ? H264_SLICE_TYPE_I : H264_SLICE_TYPE_P;
	sh.pic_parameter_set_id = fx->pps.pic_parameter_set_id;
	sh.frame_num = frame_num;
	sh.disable_deblocking_filter_idc = 2;
	if (rplm != NULL)
		sh.rplm = *rplm;
	if (drpm != NULL)
		sh.drpm = *drpm;
	res = h264_ctx_set_slice_header(fx->ctx, &sh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = is_idr ? h264_write_grey_i_slice(&bs, fx->ctx, mb_count)
		     : h264_write_skipped_p_slice(&bs, fx->ctx, mb_count);
	if (res == 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


static int s_dispose_calls;


static void frame_dispose_cb(struct vstrm_frame *base)
{
	UNUSED(base);
	s_dispose_calls++;
}


static const struct vstrm_frame_ops s_frame_ops = {
	.dispose = &frame_dispose_cb,
};


struct vstrm_frame *make_tagged_frame(uint64_t ntp_ts)
{
	struct vstrm_frame *frame = NULL;
	int res = vstrm_frame_new(&s_frame_ops, 0, &frame);
	if (res < 0)
		return NULL;
	frame->timestamps.ntp = ntp_ts;
	return frame;
}


struct vstrm_frame_nalu make_nalu(const uint8_t *data,
				  size_t len,
				  uint32_t priority,
				  uint32_t importance)
{
	struct vstrm_frame_nalu nalu = {0};
	nalu.cdata = data;
	nalu.len = len;
	nalu.priority = priority;
	nalu.importance = importance;
	return nalu;
}


size_t build_rtp_packet(uint8_t payload_type,
			uint16_t seqnum,
			uint32_t timestamp,
			uint32_t ssrc,
			bool marker,
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

	res = rtp_pkt_new(&pkt);
	if (res < 0)
		return 0;

	pkt->raw.buf = pomp_buffer_new(RTP_PKT_HEADER_SIZE);
	if (pkt->raw.buf == NULL) {
		rtp_pkt_destroy(pkt);
		return 0;
	}

	RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, VERSION, RTP_PKT_VERSION);
	RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, PAYLOAD_TYPE, payload_type);
	if (marker)
		RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, MARKER, 1);
	pkt->header.seqnum = seqnum;
	pkt->header.timestamp = timestamp;
	pkt->header.ssrc = ssrc;

	pos = RTP_PKT_HEADER_SIZE;
	pkt->payload.off = pos;
	if (payload_len > 0) {
		res = pomp_buffer_write(
			pkt->raw.buf, &pos, payload, payload_len);
		if (res < 0) {
			rtp_pkt_destroy(pkt);
			return 0;
		}
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


struct tpkt_packet *make_tpkt_from_bytes(const uint8_t *buf, size_t len)
{
	struct tpkt_packet *pkt = NULL;
	int res = tpkt_new_with_data(buf, len, &pkt);
	if (res < 0)
		return NULL;
	return pkt;
}


struct rtp_pkt *make_rtp_pkt_from_raw(const uint8_t *buf, size_t len)
{
	struct pomp_buffer *pbuf;
	struct rtp_pkt *pkt = NULL;
	int res;

	pbuf = pomp_buffer_new_with_data(buf, len);
	if (pbuf == NULL)
		return NULL;

	res = rtp_pkt_new(&pkt);
	if (res < 0) {
		pomp_buffer_unref(pbuf);
		return NULL;
	}

	res = rtp_pkt_read(pbuf, pkt);
	pomp_buffer_unref(pbuf);
	if (res < 0) {
		rtp_pkt_destroy(pkt);
		return NULL;
	}

	pkt->rtp_timestamp = pkt->header.timestamp;
	return pkt;
}


struct vstrm_timestamp make_timestamp(uint64_t ts)
{
	struct vstrm_timestamp timestamp;
	memset(&timestamp, 0, sizeof(timestamp));
	timestamp.input = ts;
	timestamp.rtp = ts;
	timestamp.ntp = ts;
	timestamp.ntp_unskewed = ts;
	timestamp.ntp_local = ts;
	timestamp.ntp_raw = ts;
	timestamp.ntp_raw_unskewed = ts;
	return timestamp;
}


void mock_rx_ctx_reset(struct mock_rx_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}


void mock_rx_ctx_clear(struct mock_rx_ctx *ctx)
{
	int i;
	int stored = ctx->frame_count < VSTRM_TEST_MOCK_MAX_FRAMES
			     ? ctx->frame_count
			     : VSTRM_TEST_MOCK_MAX_FRAMES;
	for (i = 0; i < stored; i++) {
		if (ctx->frames[i] != NULL)
			vstrm_frame_unref(ctx->frames[i]);
	}
	memset(ctx, 0, sizeof(*ctx));
}


void mock_rx_recv_frame_cb(struct vstrm_rtp_h264_rx *rtp_h264_rx,
			   struct vstrm_frame *frame,
			   void *userdata)
{
	struct mock_rx_ctx *ctx = userdata;
	UNUSED(rtp_h264_rx);
	/* frame_count is a true, uncapped call counter -- some tests expect
	 * more calls than VSTRM_TEST_MOCK_MAX_FRAMES can store, so storage
	 * is capped independently of the count itself */
	if (ctx->frame_count < VSTRM_TEST_MOCK_MAX_FRAMES) {
		vstrm_frame_ref(frame);
		ctx->frames[ctx->frame_count] = frame;
	}
	ctx->frame_count++;
}


void mock_rx_codec_info_changed_cb(struct vstrm_rtp_h264_rx *rtp_h264_rx,
				   const struct vstrm_codec_info *info,
				   void *userdata)
{
	struct mock_rx_ctx *ctx = userdata;
	UNUSED(rtp_h264_rx);
	ctx->codec_info = *info;
	ctx->codec_info_changed_count++;
}


int prime_sps_pps_packets(struct vstrm_rtp_h264_rx *rx,
			  struct h264_fixture *fx,
			  uint16_t *seq,
			  uint32_t ts)
{
	uint8_t nalu[64];
	uint8_t raw[128];
	size_t nlen, rawlen;
	struct rtp_pkt *pkt;
	struct vstrm_timestamp timestamp = make_timestamp(ts);
	int res;

	nlen = h264_fixture_build_sps(fx, nalu, sizeof(nalu));
	if (nlen == 0)
		return -EIO;
	rawlen = build_rtp_packet(VSTRM_RTP_H264_PAYLOAD_TYPE,
				  (*seq)++,
				  ts,
				  0x1234,
				  false,
				  nalu,
				  nlen,
				  raw,
				  sizeof(raw));
	pkt = make_rtp_pkt_from_raw(raw, rawlen);
	if (pkt == NULL)
		return -ENOMEM;
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	rtp_pkt_destroy(pkt);
	if (res < 0)
		return res;

	nlen = h264_fixture_build_pps(fx, nalu, sizeof(nalu));
	if (nlen == 0)
		return -EIO;
	rawlen = build_rtp_packet(VSTRM_RTP_H264_PAYLOAD_TYPE,
				  (*seq)++,
				  ts,
				  0x1234,
				  false,
				  nalu,
				  nlen,
				  raw,
				  sizeof(raw));
	pkt = make_rtp_pkt_from_raw(raw, rawlen);
	if (pkt == NULL)
		return -ENOMEM;
	res = vstrm_rtp_h264_rx_process_packet(rx, pkt, 0, &timestamp);
	rtp_pkt_destroy(pkt);
	return res;
}


void mock_ctrl_ctx_reset(struct mock_ctrl_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}


void mock_ctrl_ctx_clear(struct mock_ctrl_ctx *ctx)
{
	int i;
	for (i = 0; i < ctx->count; i++) {
		if (ctx->captured[i] != NULL)
			tpkt_unref(ctx->captured[i]);
	}
	memset(ctx, 0, sizeof(*ctx));
}


int mock_sender_send_ctrl_cb(struct vstrm_sender *stream,
			     struct tpkt_packet *pkt,
			     void *userdata)
{
	struct mock_ctrl_ctx *ctx = userdata;
	UNUSED(stream);
	if (ctx->fail_res != 0)
		return ctx->fail_res;
	if (ctx->count < VSTRM_TEST_MOCK_MAX_CAPTURED) {
		tpkt_ref(pkt);
		ctx->captured[ctx->count++] = pkt;
	}
	return 0;
}


int mock_receiver_send_ctrl_cb(struct vstrm_receiver *stream,
			       struct tpkt_packet *pkt,
			       void *userdata)
{
	struct mock_ctrl_ctx *ctx = userdata;
	UNUSED(stream);
	if (ctx->fail_res != 0)
		return ctx->fail_res;
	if (ctx->count < VSTRM_TEST_MOCK_MAX_CAPTURED) {
		tpkt_ref(pkt);
		ctx->captured[ctx->count++] = pkt;
	}
	return 0;
}


/*
 * Tests
 */

static void test_frame_new_add_nalu_refcount(void)
{
	int res;
	struct vstrm_frame *frame;
	uint8_t d1[3] = {0x67, 0xaa, 0xbb};
	uint8_t d2[2] = {0x66, 0xcc};
	struct vstrm_frame_nalu n1 = make_nalu(d1, sizeof(d1), 2, 3);
	struct vstrm_frame_nalu n2 = make_nalu(d2, sizeof(d2), 0, 1);

	s_dispose_calls = 0;
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	CU_ASSERT_EQUAL(frame->refcount, 1);

	res = vstrm_frame_add_nalu(frame, &n1);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n2);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(frame->nalu_count, 2);
	CU_ASSERT_PTR_EQUAL(frame->nalus[0].cdata, d1);
	CU_ASSERT_EQUAL(frame->nalus[0].len, sizeof(d1));
	CU_ASSERT_EQUAL(frame->nalus[0].priority, 2);
	CU_ASSERT_EQUAL(frame->nalus[0].importance, 3);
	CU_ASSERT_PTR_EQUAL(frame->nalus[1].cdata, d2);

	/* Invalid NALUs rejected */
	struct vstrm_frame_nalu bad = make_nalu(NULL, 0, 0, 0);
	res = vstrm_frame_add_nalu(frame, &bad);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* Shared frame: add_nalu forbidden */
	vstrm_frame_ref(frame);
	CU_ASSERT_EQUAL(frame->refcount, 2);
	res = vstrm_frame_add_nalu(frame, &n1);
	CU_ASSERT_EQUAL(res, -EPERM);
	CU_ASSERT_EQUAL(frame->nalu_count, 2);

	vstrm_frame_unref(frame);
	CU_ASSERT_EQUAL(s_dispose_calls, 0);
	vstrm_frame_unref(frame);
	CU_ASSERT_EQUAL(s_dispose_calls, 1);
}


static void test_frame_get_size_and_copy_flags(void)
{
	int res;
	struct vstrm_frame *frame;
	uint8_t d_sps[3] = {0x67, 0xaa, 0xbb};
	uint8_t d_sei[2] = {0x66, 0xcc};
	uint8_t d_slice[4] = {0x61, 0x01, 0x02, 0x03};
	struct vstrm_frame_nalu n_sps = make_nalu(d_sps, sizeof(d_sps), 0, 0);
	struct vstrm_frame_nalu n_sei = make_nalu(d_sei, sizeof(d_sei), 0, 0);
	struct vstrm_frame_nalu n_slice =
		make_nalu(d_slice, sizeof(d_slice), 0, 0);
	size_t size;
	uint8_t buf[64];
	uint8_t expected[64];
	size_t epos;
	static const struct {
		uint32_t flags;
	} combos[] = {
		{0},
		{VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE},
		{VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE},
		{VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS},
		{VSTRM_FRAME_COPY_FLAGS_FILTER_SEI},
		{VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS |
		 VSTRM_FRAME_COPY_FLAGS_FILTER_SEI},
		{VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE |
		 VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS},
	};
	const struct vstrm_frame_nalu *nalus[] = {&n_sps, &n_sei, &n_slice};

	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	res = vstrm_frame_add_nalu(frame, &n_sps);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n_sei);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n_slice);
	CU_ASSERT_EQUAL(res, 0);

	for (size_t c = 0; c < sizeof(combos) / sizeof(combos[0]); c++) {
		uint32_t flags = combos[c].flags;

		/* Build the expected output by mirroring the filter/insert
		 * logic directly against the known NALU list */
		epos = 0;
		for (size_t i = 0; i < 3; i++) {
			const struct vstrm_frame_nalu *nalu = nalus[i];
			uint8_t type = nalu->cdata[0] & 0x1f;
			if ((flags & VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS) &&
			    (type == H264_NALU_TYPE_SPS ||
			     type == H264_NALU_TYPE_PPS))
				continue;
			if ((flags & VSTRM_FRAME_COPY_FLAGS_FILTER_SEI) &&
			    type == H264_NALU_TYPE_SEI)
				continue;
			if (flags &
			    VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE) {
				expected[epos++] = 0x00;
				expected[epos++] = 0x00;
				expected[epos++] = 0x00;
				expected[epos++] = 0x01;
			} else if (flags &
				   VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE) {
				expected[epos++] = (nalu->len >> 24) & 0xff;
				expected[epos++] = (nalu->len >> 16) & 0xff;
				expected[epos++] = (nalu->len >> 8) & 0xff;
				expected[epos++] = nalu->len & 0xff;
			}
			memcpy(expected + epos, nalu->cdata, nalu->len);
			epos += nalu->len;
		}

		res = vstrm_frame_get_size(frame, &size, flags);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(size, epos);

		memset(buf, 0, sizeof(buf));
		res = vstrm_frame_copy(frame, buf, size, flags);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(memcmp(buf, expected, epos), 0);
	}

	/* Buffer too small */
	res = vstrm_frame_get_size(frame, &size, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_copy(frame, buf, size - 1, 0);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	vstrm_frame_unref(frame);
}


static void test_codec_info_cmp_and_str_helpers(void)
{
	struct vstrm_codec_info i1, i2;

	memset(&i1, 0, sizeof(i1));
	i1.codec = VSTRM_CODEC_VIDEO_H264;
	i1.h264.width = 640;
	i1.h264.height = 480;
	memcpy(i1.h264.sps, "SPS", 3);
	i1.h264.spslen = 3;
	memcpy(i1.h264.pps, "PPS", 3);
	i1.h264.ppslen = 3;

	i2 = i1;
	CU_ASSERT_TRUE(vstrm_codec_info_cmp(&i1, &i2));

	i2 = i1;
	i2.h264.width = 1280;
	CU_ASSERT_FALSE(vstrm_codec_info_cmp(&i1, &i2));

	i2 = i1;
	i2.h264.spslen = 4;
	CU_ASSERT_FALSE(vstrm_codec_info_cmp(&i1, &i2));

	i2 = i1;
	memcpy(i2.h264.pps, "XXX", 3);
	CU_ASSERT_FALSE(vstrm_codec_info_cmp(&i1, &i2));

	CU_ASSERT_STRING_EQUAL(vstrm_codec_str(VSTRM_CODEC_UNKNOWN), "UNKNOWN");
	CU_ASSERT_STRING_EQUAL(vstrm_codec_str(VSTRM_CODEC_VIDEO_H264), "H264");
	CU_ASSERT_STRING_EQUAL(vstrm_codec_str((enum vstrm_codec)999),
			       "UNKNOWN");

	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str(VSTRM_FRAME_MB_STATUS_UNKNOWN),
		"UNKNOWN");
	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str(VSTRM_FRAME_MB_STATUS_VALID_ISLICE),
		"VALID_ISLICE");
	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str(VSTRM_FRAME_MB_STATUS_VALID_PSLICE),
		"VALID_PSLICE");
	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str(
			VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_PSLICE),
		"MISSING_CONCEALED_PSLICE");
	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str(VSTRM_FRAME_MB_STATUS_MISSING),
		"MISSING");
	CU_ASSERT_STRING_EQUAL(vstrm_frame_mb_status_str(
				       VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION),
			       "ERROR_PROPAGATION");
	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str(
			VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_ISLICE),
		"MISSING_CONCEALED_ISLICE");
	CU_ASSERT_STRING_EQUAL(
		vstrm_frame_mb_status_str((enum vstrm_frame_mb_status)999),
		"UNKNOWN");
}


long find_dbg_file_size(const char *dir, const char *suffix)
{
	DIR *d;
	struct dirent *ent;
	size_t suffix_len = strlen(suffix);
	long size = -1;

	d = opendir(dir);
	if (d == NULL)
		return -1;

	while ((ent = readdir(d)) != NULL) {
		size_t name_len = strlen(ent->d_name);
		char path[512];
		struct stat st;

		if (name_len < suffix_len)
			continue;
		if (strcmp(ent->d_name + name_len - suffix_len, suffix) != 0)
			continue;

		snprintf(path, sizeof(path), "%s/%s", dir, ent->d_name);
		if (stat(path, &st) == 0)
			size = (long)st.st_size;
		break;
	}

	closedir(d);
	return size;
}


void remove_dbg_dir(const char *dir)
{
	DIR *d;
	struct dirent *ent;

	d = opendir(dir);
	if (d == NULL)
		return;

	while ((ent = readdir(d)) != NULL) {
		char path[512];

		if (strcmp(ent->d_name, ".") == 0 ||
		    strcmp(ent->d_name, "..") == 0)
			continue;
		snprintf(path, sizeof(path), "%s/%s", dir, ent->d_name);
		unlink(path);
	}

	closedir(d);
	rmdir(dir);
}


CU_TestInfo g_vstrm_test_frame[] = {
	{FN("frame-new-add-nalu-refcount"), &test_frame_new_add_nalu_refcount},
	{FN("frame-get-size-and-copy-flags"),
	 &test_frame_get_size_and_copy_flags},
	{FN("codec-info-cmp-and-str-helpers"),
	 &test_codec_info_cmp_and_str_helpers},

	CU_TEST_INFO_NULL,
};
