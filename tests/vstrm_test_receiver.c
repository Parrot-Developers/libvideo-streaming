/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"


/* struct vstrm_receiver_cbs has a single shared userdata pointer for every
 * callback (send_ctrl, recv_frame, codec_info_changed, ...), but each mock
 * callback wants its own context type (mock_ctrl_ctx vs mock_rx_ctx) --
 * bundle both into one struct passed as the single userdata. */
struct receiver_events_ctx {
	int event_count;
	enum vstrm_event last_event;
	int goodbye_count;
	char goodbye_reason[64];
	int session_metadata_peer_changed_count;
};


struct receiver_test_ctx {
	struct mock_rx_ctx rx;
	struct mock_ctrl_ctx ctrl;
	struct receiver_events_ctx events;
};


static void receiver_test_ctx_reset(struct receiver_test_ctx *ctx)
{
	mock_rx_ctx_reset(&ctx->rx);
	mock_ctrl_ctx_reset(&ctx->ctrl);
	memset(&ctx->events, 0, sizeof(ctx->events));
}


static void receiver_test_ctx_clear(struct receiver_test_ctx *ctx)
{
	mock_rx_ctx_clear(&ctx->rx);
	mock_ctrl_ctx_clear(&ctx->ctrl);
}


/* struct vstrm_receiver_cbs.recv_frame/codec_info_changed have a different
 * first-argument type than struct vstrm_rtp_h264_rx_cbs's, so they cannot
 * reuse mock_rx_recv_frame_cb/mock_rx_codec_info_changed_cb directly even
 * though the storage (struct mock_rx_ctx) is identical */
static void mock_receiver_recv_frame_cb(struct vstrm_receiver *stream,
					struct vstrm_frame *frame,
					void *userdata)
{
	struct receiver_test_ctx *combined = userdata;
	struct mock_rx_ctx *ctx = &combined->rx;
	UNUSED(stream);
	if (ctx->frame_count < VSTRM_TEST_MOCK_MAX_FRAMES) {
		vstrm_frame_ref(frame);
		ctx->frames[ctx->frame_count] = frame;
	}
	ctx->frame_count++;
}


static void
mock_receiver_codec_info_changed_cb(struct vstrm_receiver *stream,
				    const struct vstrm_codec_info *info,
				    void *userdata)
{
	struct receiver_test_ctx *combined = userdata;
	struct mock_rx_ctx *ctx = &combined->rx;
	UNUSED(stream);
	ctx->codec_info = *info;
	ctx->codec_info_changed_count++;
}


/* Local send_ctrl mock (cannot reuse the shared mock_receiver_send_ctrl_cb,
 * which expects userdata to be a bare struct mock_ctrl_ctx *) */
static int local_receiver_send_ctrl_cb(struct vstrm_receiver *stream,
				       struct tpkt_packet *pkt,
				       void *userdata)
{
	struct receiver_test_ctx *combined = userdata;
	struct mock_ctrl_ctx *ctx = &combined->ctrl;
	UNUSED(stream);
	if (ctx->fail_res != 0)
		return ctx->fail_res;
	if (ctx->count < VSTRM_TEST_MOCK_MAX_CAPTURED) {
		tpkt_ref(pkt);
		ctx->captured[ctx->count++] = pkt;
	}
	return 0;
}


static void mock_receiver_event_cb(struct vstrm_receiver *stream,
				   enum vstrm_event event,
				   void *userdata)
{
	struct receiver_test_ctx *combined = userdata;
	UNUSED(stream);
	combined->events.event_count++;
	combined->events.last_event = event;
}


static void mock_receiver_goodbye_cb(struct vstrm_receiver *stream,
				     const char *reason,
				     void *userdata)
{
	struct receiver_test_ctx *combined = userdata;
	UNUSED(stream);
	combined->events.goodbye_count++;
	if (reason != NULL) {
		strncpy(combined->events.goodbye_reason,
			reason,
			sizeof(combined->events.goodbye_reason) - 1);
	}
}


static void
mock_receiver_session_metadata_peer_changed_cb(struct vstrm_receiver *stream,
					       const struct vmeta_session *meta,
					       void *userdata)
{
	struct receiver_test_ctx *combined = userdata;
	UNUSED(stream);
	UNUSED(meta);
	combined->events.session_metadata_peer_changed_count++;
}


static int new_receiver(struct vstrm_receiver **receiver,
			struct receiver_test_ctx *ctx,
			uint32_t flags)
{
	struct vstrm_receiver_cfg cfg = {0};
	struct vstrm_receiver_cbs cbs = {0};

	receiver_test_ctx_reset(ctx);
	cfg.flags = flags;
	strncpy(cfg.self_meta.serial_number,
		"SN-RX-TEST",
		sizeof(cfg.self_meta.serial_number) - 1);
	cbs.send_ctrl = &local_receiver_send_ctrl_cb;
	cbs.recv_frame = &mock_receiver_recv_frame_cb;
	cbs.codec_info_changed = &mock_receiver_codec_info_changed_cb;
	cbs.event = &mock_receiver_event_cb;
	cbs.goodbye = &mock_receiver_goodbye_cb;
	cbs.session_metadata_peer_changed =
		&mock_receiver_session_metadata_peer_changed_cb;
	return vstrm_receiver_new(&cfg, &cbs, ctx, receiver);
}


static struct tpkt_packet *tpkt_from_h264_payload(uint16_t seq,
						  uint32_t ts,
						  uint32_t ssrc,
						  bool marker,
						  const uint8_t *payload,
						  size_t payload_len)
{
	uint8_t raw[512];
	size_t rawlen = build_rtp_packet(VSTRM_RTP_H264_PAYLOAD_TYPE,
					 seq,
					 ts,
					 ssrc,
					 marker,
					 payload,
					 payload_len,
					 raw,
					 sizeof(raw));
	return make_tpkt_from_bytes(raw, rawlen);
}


/* RTCP compound packet inspector, mirroring vstrm_test_sender.c's
 * rtcp_inspect -- used to verify the *content* of packets captured by
 * local_receiver_send_ctrl_cb without re-deriving librtp's wire format by
 * hand */
struct rx_rtcp_inspect {
	int sdes_item_count;
	char cname[64];
	int clock_delta_count;
	struct vstrm_clock_delta last_clock_delta;
};


static void rx_inspect_sdes_item_cb(uint32_t ssrc,
				    const struct rtcp_pkt_sdes_item *item,
				    void *userdata)
{
	struct rx_rtcp_inspect *insp = userdata;
	UNUSED(ssrc);
	insp->sdes_item_count++;
	if (item->type == RTCP_PKT_SDES_TYPE_CNAME) {
		size_t len = item->data_len < sizeof(insp->cname) - 1
				     ? item->data_len
				     : sizeof(insp->cname) - 1;
		memcpy(insp->cname, item->data, len);
		insp->cname[len] = '\0';
	}
}


static void rx_inspect_app_cb(const struct rtcp_pkt_app *app, void *userdata)
{
	struct rx_rtcp_inspect *insp = userdata;
	struct pomp_buffer *buf;
	size_t pos = 0;

	if (app->name != VSTRM_RTCP_APP_PACKET_NAME)
		return;
	if (app->subtype != VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA)
		return;

	buf = pomp_buffer_new_with_data(app->data, app->data_len);
	if (buf == NULL)
		return;
	if (vstrm_clock_delta_read(buf, &pos, &insp->last_clock_delta) == 0)
		insp->clock_delta_count++;
	pomp_buffer_unref(buf);
}


static void inspect_rx_rtcp(struct tpkt_packet *pkt,
			    struct rx_rtcp_inspect *insp)
{
	struct rtcp_pkt_read_cbs cbs = {0};
	const struct pomp_buffer *buf;

	memset(insp, 0, sizeof(*insp));
	cbs.sdes_item = &rx_inspect_sdes_item_cb;
	cbs.app = &rx_inspect_app_cb;

	buf = tpkt_get_buffer(pkt);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &cbs, insp), 0);
}


static void test_receiver_new_destroy_invalid_args(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct vstrm_receiver_cfg cfg = {0};
	struct vstrm_receiver_cbs cbs = {0};
	struct vstrm_receiver_cbs cbs_missing;

	receiver_test_ctx_reset(&ctx);
	cbs.send_ctrl = &local_receiver_send_ctrl_cb;
	cbs.recv_frame = &mock_receiver_recv_frame_cb;
	cbs.codec_info_changed = &mock_receiver_codec_info_changed_cb;

	res = vstrm_receiver_new(NULL, &cbs, &ctx, &receiver);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_new(&cfg, NULL, &ctx, &receiver);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_new(&cfg, &cbs, &ctx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	cbs_missing = cbs;
	cbs_missing.send_ctrl = NULL;
	res = vstrm_receiver_new(&cfg, &cbs_missing, &ctx, &receiver);
	CU_ASSERT_EQUAL(res, -EINVAL);

	cbs_missing = cbs;
	cbs_missing.codec_info_changed = NULL;
	res = vstrm_receiver_new(&cfg, &cbs_missing, &ctx, &receiver);
	CU_ASSERT_EQUAL(res, -EINVAL);

	cbs_missing = cbs;
	cbs_missing.recv_frame = NULL;
	res = vstrm_receiver_new(&cfg, &cbs_missing, &ctx, &receiver);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* cfg.loop == NULL is documented as optional: still succeeds */
	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(receiver);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_receiver_destroy(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_receiver_recv_data_basic(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct h264_fixture fx;
	uint8_t nalu[64];
	size_t nlen;
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* vstrm_receiver_recv_data() enqueues into an internal rtp_jitter
	 * before draining it via rtp_jitter_process(); in practice, for
	 * packets arriving already in sequence order (no reordering to wait
	 * for), the jitter buffer releases them promptly within the same
	 * call rather than holding them for the full configured delay --
	 * confirmed empirically (a real build run), so no artificial sleep
	 * is needed here. */
	nlen = h264_fixture_build_sps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_pps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(2, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_slice(&fx, true, 0, 0, 4, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(3, 1000, ssrc, true, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	CU_ASSERT_EQUAL(ctx.rx.codec_info_changed_count, 1);
	CU_ASSERT_EQUAL(ctx.rx.codec_info.h264.width, 32);
	CU_ASSERT_EQUAL(ctx.rx.codec_info.h264.height, 32);
	CU_ASSERT_EQUAL_FATAL(ctx.rx.frame_count, 1);
	CU_ASSERT_EQUAL(ctx.rx.frames[0]->nalu_count, 3);

	/* Now that a source (and its rtp_jitter) exists, this returns a
	 * real converted value instead of erroring out */
	CU_ASSERT_TRUE(vstrm_receiver_get_ntp_from_rtp_ts(receiver, 1000) > 0);

	res = vstrm_receiver_recv_data(receiver, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	receiver_test_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_clear_smoke(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Calling clear() before any packet was ever received, and calling
	 * it repeatedly, must not crash */
	res = vstrm_receiver_clear(receiver);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_receiver_clear(receiver);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_receiver_clear(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_set_video_stats_timestamp_gating(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct vstrm_video_stats stats = {0};

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* timestamp==0 is documented as "not available": silently ignored,
	 * not an error */
	stats.version = VSTRM_VIDEO_STATS_VERSION_2;
	stats.timestamp = 0;
	stats.v2.presentation_frame_count = 123;
	res = vstrm_receiver_set_video_stats(receiver, &stats);
	CU_ASSERT_EQUAL(res, 0);

	/* A nonzero timestamp is applied */
	stats.timestamp = 42;
	res = vstrm_receiver_set_video_stats(receiver, &stats);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_receiver_set_video_stats(NULL, &stats);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_set_video_stats(receiver, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_recv_ctrl_sdes_event_goodbye(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct pomp_buffer *buf;
	size_t pos;
	struct tpkt_packet *pkt;
	const struct vmeta_session *meta;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* recv_ctrl's SDES and BYE handling both need a real, already
	 * established data source: SDES via vmeta_session_is_valid() (any
	 * partially-filled session, e.g. serial_number only, is treated as
	 * "not valid" and silently discarded); BYE via an explicit check
	 * that bye->sources[0] == self->source.ssrc. A source only exists
	 * after at least one real RTP data packet has been received. */
	{
		uint8_t nalu[8] = {0};
		struct tpkt_packet *data_pkt;
		nalu[0] = (uint8_t)((3 << 5) | H264_NALU_TYPE_SEI);
		data_pkt = tpkt_from_h264_payload(
			1, 1000, 0xdeadbeef, false, nalu, sizeof(nalu));
		CU_ASSERT_PTR_NOT_NULL_FATAL(data_pkt);
		res = vstrm_receiver_recv_data(receiver, data_pkt);
		CU_ASSERT_EQUAL(res, 0);
		tpkt_unref(data_pkt);
	}

	/* SDES from the peer (sender) updates session_metadata_peer and
	 * fires the change callback -- vmeta_session_is_valid() requires
	 * friendly_name, maker=="Parrot" and model to all be set */
	{
		struct vmeta_session peer_meta = {0};
		strncpy(peer_meta.serial_number,
			"SN-PEER-1",
			sizeof(peer_meta.serial_number) - 1);
		strncpy(peer_meta.friendly_name,
			"PeerDrone",
			sizeof(peer_meta.friendly_name) - 1);
		strncpy(peer_meta.maker, "Parrot", sizeof(peer_meta.maker) - 1);
		strncpy(peer_meta.model, "AnafiX", sizeof(peer_meta.model) - 1);
		buf = pomp_buffer_new(512);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = vstrm_session_metadata_write_rtcp_sdes(
			buf, &pos, 0xdeadbeef, &peer_meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);

		res = vstrm_receiver_recv_ctrl(receiver, pkt);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(ctx.events.session_metadata_peer_changed_count,
				1);

		res = vstrm_receiver_get_session_metadata_peer(receiver, &meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		CU_ASSERT_STRING_EQUAL(meta->serial_number,
				       peer_meta.serial_number);

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
	}

	/* RTCP APP(EVENT) from the peer */
	{
		struct pomp_buffer *inner;
		size_t inner_pos = 0;
		const void *cdata;
		size_t len;
		struct rtcp_pkt_app app = {0};

		inner = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inner);
		res = vstrm_event_write(
			inner, &inner_pos, VSTRM_EVENT_RESOLUTION_CHANGE);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		pomp_buffer_get_cdata(inner, &cdata, &len, NULL);

		app.ssrc = 0xdeadbeef;
		app.name = VSTRM_RTCP_APP_PACKET_NAME;
		app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_EVENT;
		app.data = cdata;
		app.data_len = (uint32_t)len;

		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = rtcp_pkt_write_app(buf, &pos, &app);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);

		res = vstrm_receiver_recv_ctrl(receiver, pkt);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(ctx.events.event_count, 1);
		CU_ASSERT_EQUAL(ctx.events.last_event,
				VSTRM_EVENT_RESOLUTION_CHANGE);

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
		pomp_buffer_unref(inner);
	}

	/* RTCP APP(CLOCK_DELTA) from the peer: no ssrc gating, so an
	 * arbitrary (unmatched) sample is enough to exercise the whole
	 * vstrm_receiver_rtcp_app_clock_delta_cb() body, even though it
	 * gets rejected deep inside vstrm_clock_delta_process() (this
	 * receiver never sent its own probe, so expected_originate_ts is
	 * still 0 and won't match) */
	{
		struct vstrm_clock_delta cd = {
			.originate_ts = 1, .receive_ts = 2, .transmit_ts = 3};
		struct pomp_buffer *inner;
		size_t inner_pos = 0;
		const void *cdata;
		size_t len;
		struct rtcp_pkt_app app = {0};

		inner = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inner);
		res = vstrm_clock_delta_write(inner, &inner_pos, &cd);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		pomp_buffer_get_cdata(inner, &cdata, &len, NULL);

		app.ssrc = 0xdeadbeef;
		app.name = VSTRM_RTCP_APP_PACKET_NAME;
		app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA;
		app.data = cdata;
		app.data_len = (uint32_t)len;

		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = rtcp_pkt_write_app(buf, &pos, &app);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);

		res = vstrm_receiver_recv_ctrl(receiver, pkt);
		CU_ASSERT_EQUAL(res, 0);

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
		pomp_buffer_unref(inner);
	}

	/* RTCP BYE from the peer */
	{
		struct rtcp_pkt_bye bye = {0};
		bye.source_count = 1;
		bye.sources[0] = 0xdeadbeef;
		bye.reason = (const uint8_t *)"done";
		bye.reason_len = 4;

		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = rtcp_pkt_write_bye(buf, &pos, &bye);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);

		res = vstrm_receiver_recv_ctrl(receiver, pkt);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(ctx.events.goodbye_count, 1);
		CU_ASSERT_STRING_EQUAL(ctx.events.goodbye_reason, "done");

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
	}

	res = vstrm_receiver_recv_ctrl(receiver, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_getters_and_set_codec_info(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	uint32_t ssrc;
	struct vstrm_receiver_stats stats;
	const struct vmeta_session *meta;
	int64_t delta;
	struct vstrm_codec_info info;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_receiver_get_ssrc_self(receiver, &ssrc);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_receiver_get_ssrc_self(NULL, &ssrc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_get_ssrc_self(receiver, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vstrm_receiver_get_ssrc_peer(receiver, &ssrc);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_receiver_get_stats(receiver, &stats);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(stats.received_packet_count, 0);
	res = vstrm_receiver_get_stats(NULL, &stats);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_get_stats(receiver, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vstrm_receiver_get_session_metadata_self(receiver, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta);
	CU_ASSERT_STRING_EQUAL(meta->serial_number, "SN-RX-TEST");
	res = vstrm_receiver_get_session_metadata_self(receiver, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	{
		struct vmeta_session new_meta = {0};
		strncpy(new_meta.serial_number,
			"SN-RX-UPDATED",
			sizeof(new_meta.serial_number) - 1);
		res = vstrm_receiver_set_session_metadata_self(receiver,
							       &new_meta);
		CU_ASSERT_EQUAL(res, 0);
		res = vstrm_receiver_get_session_metadata_self(receiver, &meta);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_STRING_EQUAL(meta->serial_number, "SN-RX-UPDATED");
	}

	/* No clock delta handshake ever happened */
	res = vstrm_receiver_get_clock_delta(receiver, &delta, NULL);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	/* set_codec_info with an ssrc that deliberately does NOT match
	 * source.ssrc (0, since no RTP source exists yet in this test):
	 * this takes the early-return branch that just stores
	 * self->codec_info/codec_info_ssrc without forwarding to the rx
	 * layer, which would otherwise require real, parseable SPS/PPS
	 * bytes (info.h264.spslen/ppslen == 0 here is rejected there) */
	memset(&info, 0, sizeof(info));
	info.codec = VSTRM_CODEC_VIDEO_H264;
	info.h264.width = 640;
	info.h264.height = 480;
	res = vstrm_receiver_set_codec_info(receiver, &info, 1);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_receiver_set_codec_info(NULL, &info, 1);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_set_codec_info(receiver, NULL, 0);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* No source / no rtp_jitter yet: returns 0 rather than a real
	 * converted timestamp */
	CU_ASSERT_EQUAL(vstrm_receiver_get_ntp_from_rtp_ts(receiver, 1000), 0);
	CU_ASSERT_EQUAL(vstrm_receiver_get_ntp_from_rtp_ts(NULL, 1000), 0);

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_send_goodbye_and_recv_sender_report(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct h264_fixture fx;
	uint8_t nalu[64];
	size_t nlen;
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;
	char toolong[300];

	res = new_receiver(
		&receiver, &ctx, VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* vstrm_receiver_write_rtcp() (called synchronously by
	 * send_goodbye()) unconditionally calls rtp_jitter_get_info() on
	 * self->source.rtp_jitter, which only exists once a real data
	 * source has been established */
	nlen = h264_fixture_build_sps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* Receive a real RTCP sender report first (exercises
	 * vstrm_receiver_rtcp_sender_report_cb(), which has no ssrc gating
	 * and only touches self->source's own fields, never rtp_jitter) */
	{
		struct rtcp_pkt_sender_report sr = {0};
		struct pomp_buffer *buf;
		size_t pos = 0;
		struct tpkt_packet *ctrl_pkt;

		sr.ssrc = ssrc;
		sr.rtp_timestamp = 1000;
		sr.sender_packet_count = 1;
		sr.sender_byte_count = 100;

		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		res = rtcp_pkt_write_sender_report(buf, &pos, &sr);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &ctrl_pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);

		res = vstrm_receiver_recv_ctrl(receiver, ctrl_pkt);
		CU_ASSERT_EQUAL(res, 0);

		tpkt_unref(ctrl_pkt);
		pomp_buffer_unref(buf);
	}

	/* send_goodbye() -> write_rtcp() exercises write_rtcp_receiver_report
	 * (uses the last_sr_valid state just set above), write_rtcp_sdes
	 * (full, since full_sdes_send_ts==0 the first time), and, with
	 * ENABLE_RTCP_EXT, write_rtcp_clock_delta and write_rtcp_video_stats
	 * (both also gated on their *_send_ts==0 first-call special case) */
	res = vstrm_receiver_send_goodbye(receiver, "bye reason");
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.ctrl.count, 1);

	res = vstrm_receiver_send_goodbye(NULL, "x");
	CU_ASSERT_EQUAL(res, -EINVAL);

	memset(toolong, 'a', sizeof(toolong) - 1);
	toolong[sizeof(toolong) - 1] = '\0';
	res = vstrm_receiver_send_goodbye(receiver, toolong);
	CU_ASSERT_EQUAL(res, -EINVAL);

	receiver_test_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_rtcp_sender_report_ntp_regression(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct h264_fixture fx;
	uint8_t nalu[64];
	size_t nlen;
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;
	/* R1/N1, R2/N2 chosen so tsAnum=(R2-R1)=90000 (1s @ 90kHz) and
	 * tsAden=(N2-N1)=1000000us (1s) exactly, with N1/N2 landing on whole
	 * seconds so the NTP64 fixed-point round-trip is exact (no fraction
	 * rounding error) */
	const uint32_t r1 = 1000, r2 = 91000, r3 = 181000;
	const uint64_t n1_us = 5000000, n2_us = 6000000;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Establish the source with a throwaway single-NALU marker=true
	 * packet first (completing an empty/incomplete frame #0
	 * immediately), so the real AU built below starts fresh: crucially,
	 * self->current_timestamps (which frame->timestamps is copied from,
	 * map_timestamps(), src/vstrm_rtp_h264_rx.c:1305-1316) is only ever
	 * set from the *first* NALU of an AU (self->nalu.first,
	 * :2771-2772) -- so the NTP/RTP regression must already be primed
	 * (via the two sender reports below) *before* the SPS is sent,
	 * otherwise the frame's timestamp would be latched from the SPS's
	 * own (still tsAnum/tsAden == 0) computation instead */
	{
		uint8_t sei[8] = {0};
		sei[0] = (uint8_t)((3 << 5) | H264_NALU_TYPE_SEI);
		pkt = tpkt_from_h264_payload(
			1, 500, ssrc, true, sei, sizeof(sei));
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		res = vstrm_receiver_recv_data(receiver, pkt);
		CU_ASSERT_EQUAL(res, 0);
		tpkt_unref(pkt);
	}

	/* First sender report: self->source.last_sr_valid starts false, so
	 * this just takes the early "out:" path (src/vstrm_receiver.c:
	 * 134-135), recording last_sr/last_sr_valid -- no regression yet */
	{
		struct rtcp_pkt_sender_report sr = {0};
		struct pomp_buffer *buf;
		size_t pos = 0;
		struct tpkt_packet *ctrl_pkt;

		sr.ssrc = ssrc;
		sr.rtp_timestamp = r1;
		ntp_timestamp64_from_us(&sr.ntp_timestamp, n1_us);
		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		res = rtcp_pkt_write_sender_report(buf, &pos, &sr);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &ctrl_pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = vstrm_receiver_recv_ctrl(receiver, ctrl_pkt);
		CU_ASSERT_EQUAL(res, 0);
		tpkt_unref(ctrl_pkt);
		pomp_buffer_unref(buf);
	}

	/* Second sender report: last_sr_valid is now true -> runs the NTP/RTP
	 * linear regression (:137-161), setting self->source.tsAnum=90000,
	 * tsAden=1000000 */
	{
		struct rtcp_pkt_sender_report sr = {0};
		struct pomp_buffer *buf;
		size_t pos = 0;
		struct tpkt_packet *ctrl_pkt;

		sr.ssrc = ssrc;
		sr.rtp_timestamp = r2;
		ntp_timestamp64_from_us(&sr.ntp_timestamp, n2_us);
		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		res = rtcp_pkt_write_sender_report(buf, &pos, &sr);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &ctrl_pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = vstrm_receiver_recv_ctrl(receiver, ctrl_pkt);
		CU_ASSERT_EQUAL(res, 0);
		tpkt_unref(ctrl_pkt);
		pomp_buffer_unref(buf);
	}

	/* Now start the real AU: SPS is the *first* NALU of this AU, so it
	 * is the one whose regression-computed timestamp gets latched into
	 * self->current_timestamps (and thus into the eventual output
	 * frame). SPS/PPS/slice all share the same RTP timestamp (r3) --
	 * using a different one for SPS/PPS than for the slice would trigger
	 * the implicit-end-of-frame-on-timestamp-change path
	 * (src/vstrm_rtp_h264_rx.c:2756-2757) before the slice is even seen */
	nlen = h264_fixture_build_sps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(2, r3, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_pps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(3, r3, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* A real slice packet (rtp_timestamp=r3=r2+90000) drained from the
	 * jitter buffer runs the regression from
	 * vstrm_receiver_rtp_process_pkt_cb() (src/vstrm_receiver.c:669-696):
	 * timestamp.ntp = ((r3-r2)*tsAden + tsAnum/2)/tsAnum + n2_us
	 *              = (90000*1000000 + 45000)/90000 + 6000000
	 *              = 1000000 + 6000000 = 7000000
	 * but since SPS (above) was the AU's first NALU and already ran the
	 * same regression with the same result (r3 shared by all three
	 * packets), it is SPS's computed value that actually gets latched
	 * into self->current_timestamps / the output frame */
	nlen = h264_fixture_build_slice(&fx, true, 0, 0, 4, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(4, r3, ssrc, true, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* The throwaway SEI-only AU never reaches au_output()/recv_frame() at
	 * all (au_complete() drops it early: !sps.valid || !pps.valid,
	 * src/vstrm_rtp_h264_rx.c:1404-1411) -- only the real SPS/PPS/slice
	 * AU is observable here */
	CU_ASSERT_EQUAL_FATAL(ctx.rx.frame_count, 1);
	CU_ASSERT_EQUAL(ctx.rx.frames[0]->timestamps.ntp, 7000000);

	receiver_test_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_rtcp_timer_fires(void)
{
	int res;
	struct pomp_loop *loop;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct vstrm_receiver_cfg cfg = {0};
	struct vstrm_receiver_cbs cbs = {0};
	uint8_t nalu[8] = {0};
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	receiver_test_ctx_reset(&ctx);
	cfg.loop = loop;
	cfg.flags = VSTRM_RECEIVER_FLAGS_ENABLE_RTCP;
	strncpy(cfg.self_meta.serial_number,
		"SN-RX-TEST",
		sizeof(cfg.self_meta.serial_number) - 1);
	cbs.send_ctrl = &local_receiver_send_ctrl_cb;
	cbs.recv_frame = &mock_receiver_recv_frame_cb;
	cbs.codec_info_changed = &mock_receiver_codec_info_changed_cb;
	res = vstrm_receiver_new(&cfg, &cbs, &ctx, &receiver);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* The per-source RTCP timer (vstrm_receiver_rtcp_timer_cb) is only
	 * created once a source exists (inside vstrm_receiver_init_source(),
	 * gated on ENABLE_RTCP *and* cfg.loop != NULL) */
	nalu[0] = (uint8_t)((3 << 5) | H264_NALU_TYPE_SEI);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, sizeof(nalu));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	CU_ASSERT_EQUAL(ctx.ctrl.count, 0);

	/* The RTCP timer is periodic at 100ms; pump well past that */
	res = pomp_loop_wait_and_process(loop, 200);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(ctx.ctrl.count >= 1);

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
	pomp_loop_destroy(loop);
}


static void test_receiver_rtcp_sdes_compact_vs_full(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	uint8_t nalu[8] = {0};
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;
	struct rx_rtcp_inspect insp;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* new_receiver() only sets serial_number (-> CNAME); add a second
	 * field so full vs. compact SDES are actually distinguishable (a
	 * "full" SDES with only CNAME populated would be indistinguishable
	 * from "compact") */
	{
		struct vmeta_session meta = {0};
		strncpy(meta.serial_number,
			"SN-RX-TEST",
			sizeof(meta.serial_number) - 1);
		strncpy(meta.friendly_name,
			"TestReceiver",
			sizeof(meta.friendly_name) - 1);
		res = vstrm_receiver_set_session_metadata_self(receiver, &meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
	}

	/* vstrm_receiver_write_rtcp() (called synchronously by send_goodbye(),
	 * used here purely to force a tick without needing a real pomp_loop)
	 * unconditionally calls rtp_jitter_get_info() on
	 * self->source.rtp_jitter, which only exists once a real data source
	 * has been established */
	nalu[0] = (uint8_t)((3 << 5) | H264_NALU_TYPE_SEI);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, sizeof(nalu));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* First RTCP tick ever: self->source.full_sdes_send_ts==0
	 * unconditionally forces a full SDES (every populated
	 * session-metadata field, more than just CNAME) */
	res = vstrm_receiver_send_goodbye(receiver, NULL);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.ctrl.count, 1);
	inspect_rx_rtcp(ctx.ctrl.captured[0], &insp);
	CU_ASSERT_TRUE(insp.sdes_item_count > 1);
	CU_ASSERT_STRING_EQUAL(insp.cname, "SN-RX-TEST");

	/* Second tick, immediately after (well under the 2s full-SDES
	 * period): compact SDES, CNAME only
	 * (vstrm_receiver_write_rtcp_sdes(), src/vstrm_receiver.c:375-396) */
	res = vstrm_receiver_send_goodbye(receiver, NULL);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.ctrl.count, 2);
	inspect_rx_rtcp(ctx.ctrl.captured[1], &insp);
	CU_ASSERT_EQUAL(insp.sdes_item_count, 1);
	CU_ASSERT_STRING_EQUAL(insp.cname, "SN-RX-TEST");

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_get_clock_delta_full_round_trip(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	uint8_t nalu[8] = {0};
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;
	struct rx_rtcp_inspect insp;
	int64_t delta = 0;
	uint32_t precision = 0;
	uint64_t t1, t2, t3, t4;
	struct vstrm_clock_delta reply = {0};
	struct pomp_buffer *inner, *buf;
	size_t inner_pos = 0, pos = 0;
	const void *cdata;
	size_t len;
	struct rtcp_pkt_app app = {0};
	struct tpkt_packet *ctrl_pkt;

	res = new_receiver(
		&receiver, &ctx, VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	nalu[0] = (uint8_t)((3 << 5) | H264_NALU_TYPE_SEI);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, sizeof(nalu));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* self->source.clock_delta_send_ts starts at 0, so the first RTCP
	 * tick (forced via send_goodbye(), no real pomp_loop needed) always
	 * emits a CLOCK_DELTA APP probe (src/vstrm_receiver.c:594-602),
	 * which sets self->source.clock_delta_ctx.expected_originate_ts to
	 * its own transmit_ts (T1) -- capture T1 by parsing that packet */
	res = vstrm_receiver_send_goodbye(receiver, NULL);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.ctrl.count, 1);
	inspect_rx_rtcp(ctx.ctrl.captured[0], &insp);
	CU_ASSERT_EQUAL_FATAL(insp.clock_delta_count, 1);
	t1 = insp.last_clock_delta.transmit_ts;
	CU_ASSERT_TRUE_FATAL(t1 > 0);

	res = vstrm_receiver_get_clock_delta(receiver, &delta, NULL);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	/* Same construction as vstrm_test_sender.c's sender-side round trip:
	 * T1 reflected back as originate_ts, T2/T3 the peer's own
	 * receive/transmit times and T4 (this packet's own tpkt timestamp,
	 * i.e. self->source.rtcp_recv_ts) all well clear of
	 * CLOCK_DELTA_MIN_TS_DELTA (1000us) */
	t2 = t1 + 2000;
	t3 = t2 + 2000;
	t4 = t1 + 5000;
	reply.originate_ts = t1;
	reply.receive_ts = t2;
	reply.transmit_ts = t3;

	inner = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(inner);
	res = vstrm_clock_delta_write(inner, &inner_pos, &reply);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	pomp_buffer_get_cdata(inner, &cdata, &len, NULL);

	app.ssrc = ssrc;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA;
	app.data = cdata;
	app.data_len = (uint32_t)len;

	buf = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = rtcp_pkt_write_app(buf, &pos, &app);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &ctrl_pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_timestamp(ctrl_pkt, t4);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_receiver_recv_ctrl(receiver, ctrl_pkt);
	CU_ASSERT_EQUAL(res, 0);

	/* rt_delay = (t4-t1) - (t3-t2) = 3000us; clock_delta = 500 (relative
	 * to t1). This is the very first sample ever processed, so
	 * clock_delta_valid is set immediately (src/vstrm_clock_delta.c:
	 * 176-183) -- no need for a full window. */
	res = vstrm_receiver_get_clock_delta(receiver, &delta, &precision);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(delta, 500);
	CU_ASSERT_EQUAL(precision, 1500);

	res = vstrm_receiver_get_clock_delta(NULL, &delta, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_receiver_get_clock_delta(receiver, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	tpkt_unref(ctrl_pkt);
	pomp_buffer_unref(buf);
	pomp_buffer_unref(inner);
	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_clear_forces_resync_keeps_codec_info(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct h264_fixture fx;
	uint8_t nalu[64];
	size_t nlen;
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	nlen = h264_fixture_build_sps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_pps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(2, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_slice(&fx, true, 0, 0, 4, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(3, 1000, ssrc, true, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	CU_ASSERT_EQUAL_FATAL(ctx.rx.codec_info_changed_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.rx.frame_count, 1);

	/* vstrm_receiver_clear() just sets resync_needed=true
	 * (src/vstrm_receiver.c:914-921); the actual resync happens on the
	 * NEXT recv_data() call for this same ssrc, inside
	 * vstrm_receiver_update_seq()'s resync_needed branch (:755-759),
	 * which re-inits sequence tracking with keep_ps=true -- unlike a
	 * brand new ssrc (keep_ps=false), this preserves the already-known
	 * SPS/PPS */
	res = vstrm_receiver_clear(receiver);
	CU_ASSERT_EQUAL(res, 0);

	/* A second full-frame slice, same ssrc, deliberately at a completely
	 * unrelated sequence number (999, far from the previous seq=3) and
	 * with NO SPS/PPS resent -- the resync branch bypasses normal
	 * sequence validity checks entirely (it fires unconditionally,
	 * before the udelta/probation logic), and keep_ps=true means the
	 * slice still parses successfully off the already-known SPS/PPS */
	nlen = h264_fixture_build_slice(
		&fx, false, 1, 0, 4, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(999, 2000, ssrc, true, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* No new SPS/PPS was sent, so codec info must not "change" again;
	 * the new slice-only frame must still complete successfully */
	CU_ASSERT_EQUAL(ctx.rx.codec_info_changed_count, 1);
	CU_ASSERT_EQUAL_FATAL(ctx.rx.frame_count, 2);
	CU_ASSERT_EQUAL_FATAL(ctx.rx.frames[1]->nalu_count, 1);

	receiver_test_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_update_seq_large_jump_bad_seq_resync(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct receiver_test_ctx ctx;
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;
	uint8_t nalu[8] = {0};
	struct vstrm_receiver_stats stats;

	nalu[0] = (uint8_t)((3 << 5) | H264_NALU_TYPE_SEI);

	res = new_receiver(&receiver, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Packet 1 (seq=1): brand new ssrc -> vstrm_receiver_init_source(),
	 * never goes through vstrm_receiver_update_seq() at all; max_seq=1,
	 * bad_seq=RTP_SEQ_MOD+1 (never matches a real seq), received=0 */
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, sizeof(nalu));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);
	res = vstrm_receiver_get_stats(receiver, &stats);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(stats.received_packet_count, 0);

	/* Packet 2 (seq=30000): same ssrc, udelta=29999 falls in
	 * [MAX_DROPOUT=20000, RTP_SEQ_MOD-MAX_MISORDER=45536] -> "the
	 * sequence number made a very large jump" branch
	 * (src/vstrm_receiver.c:785-795); seq != bad_seq (still
	 * RTP_SEQ_MOD+1) -> rejected: bad_seq is armed to 30001, the packet
	 * does NOT count towards received_packet_count, max_seq stays at 1 */
	pkt = tpkt_from_h264_payload(
		30000, 1000, ssrc, false, nalu, sizeof(nalu));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);
	res = vstrm_receiver_get_stats(receiver, &stats);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(stats.received_packet_count, 0);

	/* Packet 3 (seq=30001 == the just-armed bad_seq): two sequential
	 * "bad" packets in a row -> treated as a legitimate resync (the peer
	 * probably restarted without telling us), vstrm_receiver_init_seq()
	 * is called and the packet DOES count this time */
	pkt = tpkt_from_h264_payload(
		30001, 1000, ssrc, false, nalu, sizeof(nalu));
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);
	res = vstrm_receiver_get_stats(receiver, &stats);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(stats.received_packet_count, 1);

	receiver_test_ctx_clear(&ctx);
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_receiver_dbg_files_created_and_written(void)
{
	int res;
	struct vstrm_receiver *receiver = NULL;
	struct vstrm_receiver_cfg cfg = {0};
	struct vstrm_receiver_cbs cbs = {0};
	struct receiver_test_ctx ctx;
	struct h264_fixture fx;
	uint8_t nalu[64];
	size_t nlen;
	struct tpkt_packet *pkt;
	const uint32_t ssrc = 0xAABBCCDD;
	char tmpdir[] = "/tmp/vstrm_receiver_dbg_XXXXXX";
	long size;
	struct rx_rtcp_inspect insp;
	uint64_t t1, t2, t3, t4;
	struct vstrm_clock_delta reply = {0};
	struct pomp_buffer *inner, *buf;
	size_t inner_pos = 0, pos = 0;
	const void *cdata;
	size_t len;
	struct rtcp_pkt_app app = {0};
	struct tpkt_packet *ctrl_pkt;

	CU_ASSERT_PTR_NOT_NULL_FATAL(mkdtemp(tmpdir));

	/* Unlike the sender, vstrm_receiver_create_dbg_files()
	 * (src/vstrm_receiver.c:805-835) is only called from
	 * vstrm_receiver_init_source(), i.e. once a real data source exists
	 * -- not at vstrm_receiver_new() time */
	receiver_test_ctx_reset(&ctx);
	cfg.flags = VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT;
	cfg.dbg_dir = tmpdir;
	cfg.dbg_flags = VSTRM_DBG_FLAG_RECEIVER_RTP_IN |
			VSTRM_DBG_FLAG_RECEIVER_RTP_JITTER |
			VSTRM_DBG_FLAG_RECEIVER_STREAM |
			VSTRM_DBG_FLAG_VIDEO_STATS | VSTRM_DBG_FLAG_CLOCK_DELTA;
	strncpy(cfg.self_meta.serial_number,
		"SN-RX-TEST",
		sizeof(cfg.self_meta.serial_number) - 1);
	cbs.send_ctrl = &local_receiver_send_ctrl_cb;
	cbs.recv_frame = &mock_receiver_recv_frame_cb;
	cbs.codec_info_changed = &mock_receiver_codec_info_changed_cb;
	res = vstrm_receiver_new(&cfg, &cbs, &ctx, &receiver);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_fixture_new(&fx, 2, 2, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Establishes the source (creating the dbg files) and drives
	 * RTP_IN (every recv_data), RTP_JITTER (every packet drained from
	 * the jitter buffer, vstrm_receiver_rtp_process_pkt_cb(),
	 * src/vstrm_receiver.c:693-694) and STREAM (codec_info_changed +
	 * recv_frame writes, :708-709/:722-723) */
	nlen = h264_fixture_build_sps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(1, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_pps(&fx, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(2, 1000, ssrc, false, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	nlen = h264_fixture_build_slice(&fx, true, 0, 0, 4, nalu, sizeof(nalu));
	CU_ASSERT_TRUE_FATAL(nlen > 0);
	pkt = tpkt_from_h264_payload(3, 1000, ssrc, true, nalu, nlen);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	res = vstrm_receiver_recv_data(receiver, pkt);
	CU_ASSERT_EQUAL(res, 0);
	tpkt_unref(pkt);

	/* First RTCP tick (forced via send_goodbye(), no real pomp_loop
	 * needed): VIDEO_STATS is written unconditionally by
	 * vstrm_receiver_write_rtcp_video_stats() (:480-489); CLOCK_DELTA's
	 * own dbg_csv is only written from inside vstrm_clock_delta_process()
	 * (src/vstrm_clock_delta.c), which only runs when a *reply* is
	 * received, not when this probe is sent -- so a full round trip is
	 * still needed below to populate that file */
	res = vstrm_receiver_send_goodbye(receiver, NULL);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.ctrl.count, 1);
	inspect_rx_rtcp(ctx.ctrl.captured[0], &insp);
	CU_ASSERT_EQUAL_FATAL(insp.clock_delta_count, 1);
	t1 = insp.last_clock_delta.transmit_ts;
	CU_ASSERT_TRUE_FATAL(t1 > 0);

	t2 = t1 + 2000;
	t3 = t2 + 2000;
	t4 = t1 + 5000;
	reply.originate_ts = t1;
	reply.receive_ts = t2;
	reply.transmit_ts = t3;

	inner = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(inner);
	res = vstrm_clock_delta_write(inner, &inner_pos, &reply);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	pomp_buffer_get_cdata(inner, &cdata, &len, NULL);

	app.ssrc = ssrc;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA;
	app.data = cdata;
	app.data_len = (uint32_t)len;

	buf = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = rtcp_pkt_write_app(buf, &pos, &app);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &ctrl_pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_timestamp(ctrl_pkt, t4);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_receiver_recv_ctrl(receiver, ctrl_pkt);
	CU_ASSERT_EQUAL(res, 0);

	tpkt_unref(ctrl_pkt);
	pomp_buffer_unref(buf);
	pomp_buffer_unref(inner);

	/* Files are only flushed to disk once fclose()'d by
	 * vstrm_receiver_close_dbg_files() (called from
	 * vstrm_receiver_destroy()), so the size checks must happen after
	 * destroy() returns */
	res = vstrm_receiver_destroy(receiver);
	CU_ASSERT_EQUAL(res, 0);

	size = find_dbg_file_size(tmpdir, "_receiver_rtp_in.bin");
	CU_ASSERT_TRUE(size > 0);
	size = find_dbg_file_size(tmpdir, "_receiver_rtp_jitter.bin");
	CU_ASSERT_TRUE(size > 0);
	size = find_dbg_file_size(tmpdir, "_receiver_stream.bin");
	CU_ASSERT_TRUE(size > 0);
	size = find_dbg_file_size(tmpdir, "_receiver_video_stats.dat");
	CU_ASSERT_TRUE(size > 0);
	size = find_dbg_file_size(tmpdir, "_receiver_clk_delta.dat");
	CU_ASSERT_TRUE(size > 0);

	receiver_test_ctx_clear(&ctx);
	h264_fixture_clear(&fx);
	remove_dbg_dir(tmpdir);
}


CU_TestInfo g_vstrm_test_receiver[] = {
	{FN("receiver-new-destroy-invalid-args"),
	 &test_receiver_new_destroy_invalid_args},
	{FN("receiver-recv-data-basic"), &test_receiver_recv_data_basic},
	{FN("receiver-clear-smoke"), &test_receiver_clear_smoke},
	{FN("receiver-set-video-stats-timestamp-gating"),
	 &test_receiver_set_video_stats_timestamp_gating},
	{FN("receiver-recv-ctrl-sdes-event-goodbye"),
	 &test_receiver_recv_ctrl_sdes_event_goodbye},
	{FN("receiver-getters-and-set-codec-info"),
	 &test_receiver_getters_and_set_codec_info},
	{FN("receiver-send-goodbye-and-recv-sender-report"),
	 &test_receiver_send_goodbye_and_recv_sender_report},
	{FN("receiver-rtcp-sender-report-ntp-regression"),
	 &test_receiver_rtcp_sender_report_ntp_regression},
	{FN("receiver-rtcp-timer-fires"), &test_receiver_rtcp_timer_fires},
	{FN("receiver-rtcp-sdes-compact-vs-full"),
	 &test_receiver_rtcp_sdes_compact_vs_full},
	{FN("receiver-get-clock-delta-full-round-trip"),
	 &test_receiver_get_clock_delta_full_round_trip},
	{FN("receiver-clear-forces-resync-keeps-codec-info"),
	 &test_receiver_clear_forces_resync_keeps_codec_info},
	{FN("receiver-update-seq-large-jump-bad-seq-resync"),
	 &test_receiver_update_seq_large_jump_bad_seq_resync},
	{FN("receiver-dbg-files-created-and-written"),
	 &test_receiver_dbg_files_created_and_written},

	CU_TEST_INFO_NULL,
};
