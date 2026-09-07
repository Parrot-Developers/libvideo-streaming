/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <video-defs/vdefs.h>


static void fill_nalu_bytes(uint8_t *buf, size_t len, uint8_t nal_type)
{
	buf[0] = (uint8_t)((3 << 5) | (nal_type & 0x1f));
	for (size_t i = 1; i < len; i++)
		buf[i] = (uint8_t)(i & 0xff);
}


/* Pool backing every mbuf_coded_video_frame built by this file's tests.
 * Intentionally never destroyed: it is a module-level fixture that just
 * lives for the duration of the test process, consistent with how other
 * such fixtures in this test binary behave (there is no whole-file CUnit
 * teardown to hook it into). */
static struct mbuf_pool *s_mbuf_pool;


static struct mbuf_pool *get_mbuf_pool(void)
{
	if (s_mbuf_pool == NULL) {
		int res = mbuf_pool_new(mbuf_mem_generic_impl,
					4096,
					0,
					MBUF_POOL_LOW_MEM_GROW,
					0,
					"vstrm-test-sender",
					&s_mbuf_pool);
		CU_ASSERT_EQUAL_FATAL(res, 0);
	}
	return s_mbuf_pool;
}


/* Builds an empty, not-yet-finalized mbuf_coded_video_frame tagged with
 * ntp_ts as a microsecond timestamp (timescale=1000000), matching how
 * make_tagged_frame()'s frame->timestamps.ntp is used elsewhere in this
 * file */
static struct mbuf_coded_video_frame *make_tagged_mbuf_frame(uint64_t ntp_ts)
{
	struct vdef_coded_frame info = {0};
	struct mbuf_coded_video_frame *frame = NULL;
	int res;

	info.format = vdef_h264_raw_nalu;
	info.type = VDEF_CODED_FRAME_TYPE_CODED;
	info.info.timestamp = ntp_ts;
	info.info.timescale = 1000000;

	res = mbuf_coded_video_frame_new(&info, &frame);
	if (res < 0)
		return NULL;
	return frame;
}


/* Copies data into a fresh pool buffer and adds it as one NAL unit */
static int add_mbuf_nalu(struct mbuf_coded_video_frame *frame,
			 const uint8_t *data,
			 size_t len,
			 uint32_t priority,
			 uint32_t importance)
{
	struct mbuf_mem *mem = NULL;
	void *mem_data;
	size_t cap;
	struct vdef_nalu nalu = {0};
	int res;

	res = mbuf_pool_get(get_mbuf_pool(), &mem);
	if (res < 0)
		return res;
	res = mbuf_mem_get_data(mem, &mem_data, &cap);
	if (res < 0) {
		mbuf_mem_unref(mem);
		return res;
	}
	CU_ASSERT_TRUE_FATAL(len <= cap);
	memcpy(mem_data, data, len);

	nalu.size = len;
	nalu.priority = priority;
	nalu.importance = importance;
	nalu.h264.type = H264_NALU_TYPE_SLICE;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	mbuf_mem_unref(mem);
	return res;
}


static int new_sender(struct vstrm_sender **sender,
		      struct mock_ctrl_ctx *ctx,
		      uint32_t flags)
{
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};

	mock_ctrl_ctx_reset(ctx);
	cfg.flags = flags;
	cfg.dyn.target_packet_size = 1000;
	strncpy(cfg.self_meta.serial_number,
		"SN-TEST-1",
		sizeof(cfg.self_meta.serial_number) - 1);
	strncpy(cfg.self_meta.friendly_name,
		"TestSender",
		sizeof(cfg.self_meta.friendly_name) - 1);
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	return vstrm_sender_new(&cfg, &cbs, ctx, sender);
}


/* Sends one small frame so that write_rtcp() (which is a no-op until at
 * least one RTP packet has been sent) becomes active */
static void unlock_rtcp(struct vstrm_sender *sender)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct tpkt_list *list = NULL;

	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);
}


/* RTCP compound packet inspector, used to verify the *content* of packets
 * captured by mock_sender_send_ctrl_cb without re-deriving librtp's wire
 * format by hand */
struct rtcp_inspect {
	int sdes_item_count;
	char cname[64];
	int event_count;
	enum vstrm_event last_event;
	int bye_count;
	char bye_reason[64];
	int receiver_report_count;
	uint32_t last_rtd;
	int video_stats_count;
	struct vstrm_video_stats last_video_stats;
	int clock_delta_count;
	struct vstrm_clock_delta last_clock_delta;
	int sender_report_count;
	uint32_t last_sr_rtp_timestamp;
};


static void inspect_sdes_item_cb(uint32_t ssrc,
				 const struct rtcp_pkt_sdes_item *item,
				 void *userdata)
{
	struct rtcp_inspect *insp = userdata;
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


static void inspect_bye_cb(const struct rtcp_pkt_bye *bye, void *userdata)
{
	struct rtcp_inspect *insp = userdata;
	insp->bye_count++;
	if (bye->reason != NULL) {
		size_t len = bye->reason_len < sizeof(insp->bye_reason) - 1
				     ? bye->reason_len
				     : sizeof(insp->bye_reason) - 1;
		memcpy(insp->bye_reason, bye->reason, len);
		insp->bye_reason[len] = '\0';
	}
}


static void
inspect_receiver_report_cb(const struct rtcp_pkt_receiver_report *rr,
			   void *userdata)
{
	struct rtcp_inspect *insp = userdata;
	UNUSED(rr);
	insp->receiver_report_count++;
}


static void inspect_sender_report_cb(const struct rtcp_pkt_sender_report *sr,
				     void *userdata)
{
	struct rtcp_inspect *insp = userdata;
	insp->sender_report_count++;
	insp->last_sr_rtp_timestamp = sr->rtp_timestamp;
}


static void inspect_app_cb(const struct rtcp_pkt_app *app, void *userdata)
{
	struct rtcp_inspect *insp = userdata;
	struct pomp_buffer *buf;
	size_t pos = 0;

	if (app->name != VSTRM_RTCP_APP_PACKET_NAME)
		return;

	if (app->subtype == VSTRM_RTCP_APP_PACKET_SUBTYPE_EVENT) {
		enum vstrm_event event = VSTRM_EVENT_NONE;
		buf = pomp_buffer_new_with_data(app->data, app->data_len);
		if (buf == NULL)
			return;
		if (vstrm_event_read(buf, &pos, &event) == 0) {
			insp->event_count++;
			insp->last_event = event;
		}
		pomp_buffer_unref(buf);
	} else if (app->subtype == VSTRM_RTCP_APP_PACKET_SUBTYPE_VIDEO_STATS) {
		struct vstrm_video_stats_dyn dyn = {0};
		buf = pomp_buffer_new_with_data(app->data, app->data_len);
		if (buf == NULL)
			return;
		if (vstrm_video_stats_read(
			    buf, &pos, &insp->last_video_stats, &dyn) == 0) {
			insp->video_stats_count++;
		}
		vstrm_video_stats_dyn_clear(&dyn);
		pomp_buffer_unref(buf);
	} else if (app->subtype == VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA) {
		buf = pomp_buffer_new_with_data(app->data, app->data_len);
		if (buf == NULL)
			return;
		if (vstrm_clock_delta_read(
			    buf, &pos, &insp->last_clock_delta) == 0) {
			insp->clock_delta_count++;
		}
		pomp_buffer_unref(buf);
	}
}


static void inspect_rtcp(struct tpkt_packet *pkt, struct rtcp_inspect *insp)
{
	struct rtcp_pkt_read_cbs cbs = {0};
	const struct pomp_buffer *buf;

	memset(insp, 0, sizeof(*insp));
	cbs.sdes_item = &inspect_sdes_item_cb;
	cbs.bye = &inspect_bye_cb;
	cbs.receiver_report = &inspect_receiver_report_cb;
	cbs.sender_report = &inspect_sender_report_cb;
	cbs.app = &inspect_app_cb;

	buf = tpkt_get_buffer(pkt);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &cbs, insp), 0);
}


static void test_sender_new_destroy_invalid_args(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct vstrm_sender_cbs cbs_no_send = {0};

	mock_ctrl_ctx_reset(&ctx);
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;

	res = vstrm_sender_new(NULL, &cbs, &ctx, &sender);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_new(&cfg, NULL, &ctx, &sender);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_new(&cfg, &cbs_no_send, &ctx, &sender);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_new(&cfg, &cbs, &ctx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(sender);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_sender_destroy(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_sender_send_frame_produces_expected_rtp(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct mbuf_coded_video_frame *frame;
	uint8_t data[50];
	struct tpkt_list *list = NULL;
	struct tpkt_packet *tpkt;
	const void *cdata;
	size_t len;
	struct pomp_buffer *pbuf;
	struct rtp_pkt *rtp_pkt;

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(list);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), 1);

	tpkt = tpkt_list_first(list);
	CU_ASSERT_PTR_NOT_NULL_FATAL(tpkt);
	pbuf = tpkt_get_buffer(tpkt);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pbuf);

	res = rtp_pkt_new(&rtp_pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = rtp_pkt_read(pbuf, rtp_pkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(
		RTP_PKT_HEADER_FLAGS_GET(rtp_pkt->header.flags, PAYLOAD_TYPE),
		VSTRM_RTP_H264_PAYLOAD_TYPE);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(rtp_pkt->header.flags, MARKER),
			1);
	CU_ASSERT_EQUAL(rtp_pkt->payload.len, sizeof(data));
	tpkt_get_cdata(tpkt, &cdata, &len, NULL);
	CU_ASSERT_EQUAL(memcmp((const uint8_t *)cdata + rtp_pkt->payload.off,
			       data,
			       sizeof(data)),
			0);
	rtp_pkt_destroy(rtp_pkt);

	res = vstrm_sender_send_frame(NULL, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_send_frame(sender, NULL, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL(res, -EINVAL);

	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_rtcp_sdes_compact_vs_full(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct rtcp_inspect insp;

	res = new_sender(&sender, &ctx, VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* write_rtcp() is a no-op until at least one RTP packet was sent */
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(ctx.count, 0);

	unlock_rtcp(sender);

	/* First RTCP tick ever: full_sdes_send_ts==0 unconditionally forces
	 * a full SDES (every populated session-metadata field, more than
	 * just CNAME) */
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 1);
	inspect_rtcp(ctx.captured[0], &insp);
	CU_ASSERT_TRUE(insp.sdes_item_count > 1);
	CU_ASSERT_STRING_EQUAL(insp.cname, "SN-TEST-1");

	/* Second RTCP tick, immediately after (well under the 2s full-SDES
	 * period): compact SDES, CNAME only */
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RESOLUTION_CHANGE);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 2);
	inspect_rtcp(ctx.captured[1], &insp);
	CU_ASSERT_EQUAL(insp.sdes_item_count, 1);
	CU_ASSERT_STRING_EQUAL(insp.cname, "SN-TEST-1");

	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_send_event_and_goodbye(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct rtcp_inspect insp;
	char toolong[300];

	res = new_sender(&sender, &ctx, VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	unlock_rtcp(sender);

	res = vstrm_sender_send_event(sender, VSTRM_EVENT_PHOTO_TRIGGER);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 1);
	inspect_rtcp(ctx.captured[0], &insp);
	CU_ASSERT_EQUAL(insp.event_count, 1);
	CU_ASSERT_EQUAL(insp.last_event, VSTRM_EVENT_PHOTO_TRIGGER);
	CU_ASSERT_EQUAL(insp.bye_count, 0);

	res = vstrm_sender_send_goodbye(sender, "bye reason");
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 2);
	inspect_rtcp(ctx.captured[1], &insp);
	CU_ASSERT_EQUAL(insp.bye_count, 1);
	CU_ASSERT_STRING_EQUAL(insp.bye_reason, "bye reason");

	res = vstrm_sender_send_goodbye(sender, NULL);
	CU_ASSERT_EQUAL(res, 0);

	memset(toolong, 'a', sizeof(toolong) - 1);
	toolong[sizeof(toolong) - 1] = '\0';
	res = vstrm_sender_send_goodbye(sender, toolong);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


struct sender_events_ctx {
	int rr_count;
	uint32_t last_rtd;
	int video_stats_count;
	uint32_t last_total_frame_count;
	int goodbye_count;
	char last_goodbye_reason[64];
	int session_metadata_peer_changed_count;
	struct vmeta_session last_peer_meta;
};


static void rr_event_cb(struct vstrm_sender *stream,
			const struct rtcp_pkt_receiver_report *rr,
			uint32_t rtd,
			void *userdata)
{
	struct sender_events_ctx *ev = userdata;
	UNUSED(stream);
	UNUSED(rr);
	ev->rr_count++;
	ev->last_rtd = rtd;
}


static void vs_event_cb(struct vstrm_sender *stream,
			const struct vstrm_video_stats *vs,
			const struct vstrm_video_stats_dyn *vsd,
			void *userdata)
{
	struct sender_events_ctx *ev = userdata;
	UNUSED(stream);
	UNUSED(vsd);
	ev->video_stats_count++;
	ev->last_total_frame_count = vs->v2.total_frame_count;
}


static void goodbye_event_cb(struct vstrm_sender *stream,
			     const char *reason,
			     void *userdata)
{
	struct sender_events_ctx *ev = userdata;
	UNUSED(stream);
	ev->goodbye_count++;
	if (reason != NULL) {
		strncpy(ev->last_goodbye_reason,
			reason,
			sizeof(ev->last_goodbye_reason) - 1);
	}
}


static void session_metadata_peer_changed_cb(struct vstrm_sender *stream,
					     const struct vmeta_session *meta,
					     void *userdata)
{
	struct sender_events_ctx *ev = userdata;
	UNUSED(stream);
	ev->session_metadata_peer_changed_count++;
	ev->last_peer_meta = *meta;
}


static void
test_sender_recv_ctrl_receiver_report_and_video_stats_callbacks(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct sender_events_ctx events;
	struct pomp_buffer *buf;
	size_t pos = 0;
	struct rtcp_pkt_receiver_report rr = {0};
	struct tpkt_packet *pkt;

	memset(&events, 0, sizeof(events));
	mock_ctrl_ctx_reset(&ctx);
	cfg.dyn.target_packet_size = 1000;
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	cbs.receiver_report = &rr_event_cb;
	cbs.video_stats = &vs_event_cb;
	cbs.goodbye = &goodbye_event_cb;
	res = vstrm_sender_new(&cfg, &cbs, &events, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Receiver report with report_count=0 sidesteps the LSR/DLSR
	 * round-trip-delay arithmetic entirely (rtd stays UINT32_MAX) --
	 * this test is about the recv_ctrl -> rtcp_pkt_read -> callback
	 * wiring, not about re-deriving RFC 3550 SS6.4.1 by hand */
	rr.ssrc = 0xAABBCCDD;
	rr.report_count = 0;
	buf = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = rtcp_pkt_write_receiver_report(buf, &pos, &rr);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(events.rr_count, 1);
	CU_ASSERT_EQUAL(events.last_rtd, UINT32_MAX);
	tpkt_unref(pkt);
	pomp_buffer_unref(buf);

	/* RTCP APP(VIDEO_STATS): a v2 video-stats struct with
	 * mb_status_class_count=mb_status_zone_count=0 needs no dynamic
	 * arrays at all, keeping the fixture minimal */
	{
		struct vstrm_video_stats vs = {0};
		struct vstrm_video_stats_dyn dyn = {0};
		struct pomp_buffer *inner;
		size_t inner_pos = 0;
		const void *cdata;
		size_t len;
		struct rtcp_pkt_app app = {0};

		vs.version = VSTRM_VIDEO_STATS_VERSION_2;
		vs.v2.total_frame_count = 42;

		inner = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inner);
		res = vstrm_video_stats_write(inner, &inner_pos, &vs, &dyn);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		pomp_buffer_get_cdata(inner, &cdata, &len, NULL);

		app.ssrc = 0xAABBCCDD;
		app.name = VSTRM_RTCP_APP_PACKET_NAME;
		app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_VIDEO_STATS;
		app.data = cdata;
		app.data_len = (uint32_t)len;

		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = rtcp_pkt_write_app(buf, &pos, &app);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = vstrm_sender_recv_ctrl(sender, pkt);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(events.video_stats_count, 1);
		CU_ASSERT_EQUAL(events.last_total_frame_count, 42);

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
		pomp_buffer_unref(inner);
	}

	/* RTCP APP(CLOCK_DELTA): no ssrc gating at all, so a single arbitrary
	 * (unmatched) sample is enough to exercise the callback -- it will
	 * be rejected deep inside vstrm_clock_delta_process() (originate_ts
	 * doesn't match any expected value), but that still runs the whole
	 * vstrm_sender_rtcp_app_clock_delta_cb() body */
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

		app.ssrc = 0xAABBCCDD;
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
		res = vstrm_sender_recv_ctrl(sender, pkt);
		CU_ASSERT_EQUAL(res, 0);

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
		pomp_buffer_unref(inner);
	}

	/* RTCP BYE: gated on bye->sources[0] == self->peer_ssrc, which the
	 * earlier receiver-report feed above already set to 0xAABBCCDD */
	{
		struct rtcp_pkt_bye bye = {0};
		bye.source_count = 1;
		bye.sources[0] = 0xAABBCCDD;
		bye.reason = (const uint8_t *)"peer done";
		bye.reason_len = 9;

		buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = rtcp_pkt_write_bye(buf, &pos, &bye);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(buf, &pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = vstrm_sender_recv_ctrl(sender, pkt);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(events.goodbye_count, 1);
		CU_ASSERT_STRING_EQUAL(events.last_goodbye_reason, "peer done");

		tpkt_unref(pkt);
		pomp_buffer_unref(buf);
	}

	res = vstrm_sender_recv_ctrl(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_recv_ctrl_receiver_report_rtt_computation(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct sender_events_ctx events;
	struct pomp_buffer *buf;
	size_t pos = 0;
	struct rtcp_pkt_receiver_report rr = {0};
	struct tpkt_packet *pkt;

	memset(&events, 0, sizeof(events));
	cfg.dyn.target_packet_size = 1000;
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	cbs.receiver_report = &rr_event_cb;
	res = vstrm_sender_new(&cfg, &cbs, &events, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Round-trip delay per RFC 3550 SS6.4.1
	 * (vstrm_sender_rtcp_receiver_report_cb(), src/vstrm_sender.c:88-119):
	 * LSR = 10.0s exactly, this packet's own tpkt timestamp (i.e.
	 * self->rtcp_recv_ts) = 10.25s -> diff=250000us; DLSR=8192 raw
	 * 1/65536s units = 125000us -> rtd = diff - dlsr = 125000us exactly.
	 * All values are chosen as exact powers of 2/65536 so the 32-bit NTP
	 * fixed-point conversions round-trip with zero error. */
	rr.ssrc = 0xAABBCCDD;
	rr.report_count = 1;
	rr.reports[0].ssrc = 0x11223344;
	rr.reports[0].lsr.seconds = 10;
	rr.reports[0].lsr.fraction = 0;
	rr.reports[0].dlsr = 8192;
	buf = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = rtcp_pkt_write_receiver_report(buf, &pos, &rr);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_timestamp(pkt, 10250000);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(events.rr_count, 1);
	CU_ASSERT_EQUAL(events.last_rtd, 125000);

	tpkt_unref(pkt);
	pomp_buffer_unref(buf);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_get_clock_delta_full_round_trip(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct rtcp_inspect insp;
	int64_t delta = 0;
	uint32_t precision = 0;
	uint64_t t1, t2, t3, t4;
	struct vstrm_clock_delta reply = {0};
	struct pomp_buffer *inner, *buf;
	size_t inner_pos = 0, pos = 0;
	const void *cdata;
	size_t len;
	struct rtcp_pkt_app app = {0};
	struct tpkt_packet *pkt;

	res = new_sender(&sender, &ctx, VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	unlock_rtcp(sender);

	/* self->clock_delta_send_ts starts at 0, so the very first RTCP tick
	 * always emits a CLOCK_DELTA APP probe (src/vstrm_sender.c:521-530),
	 * which sets self->clock_delta_ctx.expected_originate_ts to its own
	 * transmit_ts (T1). Capture T1 by parsing that outgoing packet. */
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 1);
	inspect_rtcp(ctx.captured[0], &insp);
	CU_ASSERT_EQUAL_FATAL(insp.clock_delta_count, 1);
	t1 = insp.last_clock_delta.transmit_ts;
	CU_ASSERT_TRUE_FATAL(t1 > 0);

	/* Before any reply: clock delta is not yet available
	 * (vstrm_sender_get_clock_delta(), src/vstrm_sender.c:1149-1150) */
	res = vstrm_sender_get_clock_delta(sender, &delta, NULL);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	/* Craft the peer's reply: T1 reflected back as originate_ts, T2/T3
	 * the peer's own receive/transmit times, well clear of
	 * CLOCK_DELTA_MIN_TS_DELTA (1000us); T4 (this packet's own tpkt
	 * timestamp, i.e. self->rtcp_recv_ts) likewise well clear of T1 */
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

	app.ssrc = 0xAABBCCDD;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA;
	app.data = cdata;
	app.data_len = (uint32_t)len;

	buf = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = rtcp_pkt_write_app(buf, &pos, &app);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_timestamp(pkt, t4);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);

	/* rt_delay = (t4-t1) - (t3-t2) = 5000-2000 = 3000us; clock_delta =
	 * (t2+t3-t1-t4+1)/2 = 500 (relative to t1). This is the very first
	 * sample ever processed, so clock_delta_valid is set immediately
	 * (src/vstrm_clock_delta.c:176-183) -- no need for a full window. */
	res = vstrm_sender_get_clock_delta(sender, &delta, &precision);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(delta, 500);
	CU_ASSERT_EQUAL(precision, 1500);

	res = vstrm_sender_get_clock_delta(NULL, &delta, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_get_clock_delta(sender, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	tpkt_unref(pkt);
	pomp_buffer_unref(buf);
	pomp_buffer_unref(inner);
	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_recv_ctrl_session_metadata_peer_changed(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct sender_events_ctx events;
	struct pomp_buffer *buf;
	size_t pos;
	struct tpkt_packet *pkt;
	struct vmeta_session peer_meta = {0};

	memset(&events, 0, sizeof(events));
	cfg.dyn.target_packet_size = 1000;
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	cbs.session_metadata_peer_changed = &session_metadata_peer_changed_cb;
	res = vstrm_sender_new(&cfg, &cbs, &events, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* First SDES ever: peer metadata goes from all-empty to populated ->
	 * vmeta_session_cmp() (1 means equal) sees a difference, callback
	 * fires (src/vstrm_sender.c:999-1005) */
	strncpy(peer_meta.serial_number,
		"SN-PEER-1",
		sizeof(peer_meta.serial_number) - 1);
	pos = 0;
	buf = pomp_buffer_new(512);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = vstrm_session_metadata_write_rtcp_sdes(
		buf, &pos, 0xdeadbeef, &peer_meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(events.session_metadata_peer_changed_count, 1);
	CU_ASSERT_STRING_EQUAL(events.last_peer_meta.serial_number,
			       "SN-PEER-1");
	tpkt_unref(pkt);
	pomp_buffer_unref(buf);

	/* Same metadata again -> no change, callback must NOT fire again */
	pos = 0;
	buf = pomp_buffer_new(512);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = vstrm_session_metadata_write_rtcp_sdes(
		buf, &pos, 0xdeadbeef, &peer_meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(events.session_metadata_peer_changed_count, 1);
	tpkt_unref(pkt);
	pomp_buffer_unref(buf);

	/* Different metadata -> callback fires again */
	strncpy(peer_meta.serial_number,
		"SN-PEER-2",
		sizeof(peer_meta.serial_number) - 1);
	pos = 0;
	buf = pomp_buffer_new(512);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = vstrm_session_metadata_write_rtcp_sdes(
		buf, &pos, 0xdeadbeef, &peer_meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(events.session_metadata_peer_changed_count, 2);
	CU_ASSERT_STRING_EQUAL(events.last_peer_meta.serial_number,
			       "SN-PEER-2");
	tpkt_unref(pkt);
	pomp_buffer_unref(buf);

	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_dbg_files_created_and_written(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct mbuf_coded_video_frame *frame;
	uint8_t data[20];
	struct tpkt_list *list = NULL;
	char tmpdir[] = "/tmp/vstrm_sender_dbg_XXXXXX";
	long size;
	struct rtcp_inspect insp;
	uint64_t t1, t2, t3, t4;
	struct vstrm_clock_delta reply = {0};
	struct pomp_buffer *inner, *buf;
	size_t inner_pos = 0, pos = 0;
	const void *cdata;
	size_t len;
	struct rtcp_pkt_app app = {0};
	struct tpkt_packet *pkt;

	CU_ASSERT_PTR_NOT_NULL_FATAL(mkdtemp(tmpdir));

	/* vstrm_sender_create_dbg_files() (src/vstrm_sender.c:599-629) opens
	 * one real file per set flag, using cfg.dbg_dir directly (bypassing
	 * the VSTRM_DBG_DIR/VSTRM_DBG_FLAGS env vars, which the test driver
	 * unsets for hermetic runs) */
	mock_ctrl_ctx_reset(&ctx);
	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT;
	cfg.dyn.target_packet_size = 1000;
	cfg.dbg_dir = tmpdir;
	cfg.dbg_flags = VSTRM_DBG_FLAG_SENDER_STREAM |
			VSTRM_DBG_FLAG_SENDER_RTP_PAYLOAD |
			VSTRM_DBG_FLAG_SENDER_RTP_OUT |
			VSTRM_DBG_FLAG_VIDEO_STATS | VSTRM_DBG_FLAG_CLOCK_DELTA;
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	res = vstrm_sender_new(&cfg, &cbs, &ctx, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);

	/* First RTCP tick (forced via send_event(), unlocked by the frame
	 * just sent above): emits a CLOCK_DELTA APP probe, but
	 * VSTRM_DBG_FLAG_CLOCK_DELTA's dbg_csv is only written from inside
	 * vstrm_clock_delta_process() (src/vstrm_clock_delta.c), which only
	 * runs when a *reply* is received -- so a full round trip is needed
	 * below to actually populate that file */
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 1);
	inspect_rtcp(ctx.captured[0], &insp);
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

	app.ssrc = 0xAABBCCDD;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA;
	app.data = cdata;
	app.data_len = (uint32_t)len;

	buf = pomp_buffer_new(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = rtcp_pkt_write_app(buf, &pos, &app);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_set_timestamp(pkt, t4);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);

	tpkt_unref(pkt);
	pomp_buffer_unref(buf);
	pomp_buffer_unref(inner);

	/* RTCP APP(VIDEO_STATS): vstrm_sender_rtcp_app_video_stats_cb()
	 * unconditionally writes the CSV header + one row
	 * (src/vstrm_sender.c:214-225) whenever a valid VIDEO_STATS APP is
	 * received, regardless of the callback being set */
	{
		struct vstrm_video_stats vs = {0};
		struct vstrm_video_stats_dyn dyn = {0};
		struct pomp_buffer *vs_inner;
		size_t vs_inner_pos = 0;
		const void *vs_cdata;
		size_t vs_len;
		struct rtcp_pkt_app vs_app = {0};
		struct tpkt_packet *vs_pkt;
		struct pomp_buffer *vs_buf;
		size_t vs_pos = 0;

		vs.version = VSTRM_VIDEO_STATS_VERSION_2;
		vs.v2.total_frame_count = 1;

		vs_inner = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(vs_inner);
		res = vstrm_video_stats_write(
			vs_inner, &vs_inner_pos, &vs, &dyn);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		pomp_buffer_get_cdata(vs_inner, &vs_cdata, &vs_len, NULL);

		vs_app.ssrc = 0xAABBCCDD;
		vs_app.name = VSTRM_RTCP_APP_PACKET_NAME;
		vs_app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_VIDEO_STATS;
		vs_app.data = vs_cdata;
		vs_app.data_len = (uint32_t)vs_len;

		vs_buf = pomp_buffer_new(0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(vs_buf);
		res = rtcp_pkt_write_app(vs_buf, &vs_pos, &vs_app);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = tpkt_new_from_buffer(vs_buf, &vs_pkt);
		CU_ASSERT_EQUAL_FATAL(res, 0);

		res = vstrm_sender_recv_ctrl(sender, vs_pkt);
		CU_ASSERT_EQUAL(res, 0);

		tpkt_unref(vs_pkt);
		pomp_buffer_unref(vs_buf);
		pomp_buffer_unref(vs_inner);
	}

	/* Files are only flushed to disk once fclose()'d by
	 * vstrm_sender_close_dbg_files() (called from vstrm_sender_destroy()),
	 * so the size checks below must happen after destroy() returns */
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);

	size = find_dbg_file_size(tmpdir, "_sender_stream.bin");
	CU_ASSERT_TRUE(size > 0);
	size = find_dbg_file_size(tmpdir, "_sender_rtp_payload.bin");
	CU_ASSERT_TRUE(size > 0);

	/* self->dbg.rtp_out is created (grep-confirmed at
	 * src/vstrm_sender.c:615-616) and closed (:642-644), but nothing in
	 * vstrm_sender.c ever writes to it -- a real, currently-permanent
	 * gap (this file is always empty, unlike its "stream"/"rtp_payload"
	 * siblings). Documented here rather than asserted as populated. */
	size = find_dbg_file_size(tmpdir, "_sender_rtp_out.bin");
	CU_ASSERT_EQUAL(size, 0);

	size = find_dbg_file_size(tmpdir, "_sender_video_stats.dat");
	CU_ASSERT_TRUE(size > 0);
	size = find_dbg_file_size(tmpdir, "_sender_clk_delta.dat");
	CU_ASSERT_TRUE(size > 0);

	mock_ctrl_ctx_clear(&ctx);
	remove_dbg_dir(tmpdir);
}


static void test_sender_write_rtcp_sender_report_negative_diff(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct tpkt_list *list = NULL;
	struct rtcp_inspect insp;
	struct timespec ts;
	uint64_t before, after;
	uint64_t frame_ntp;
	uint32_t last_rtp_timestamp;
	uint32_t expected_lo, expected_hi;

	/* A fixed huge constant (e.g. "~317 years") cannot safely stand in
	 * for "always larger than any real cur_timestamp": rtp_timestamp_
	 * from_us() overflows uint64_t past ~6.5 years' worth of
	 * microseconds at VSTRM_RTP_H264_CLK_RATE, while a long-lived CI
	 * runner's CLOCK_MONOTONIC (which tracks host uptime, not wall
	 * clock) can easily exceed 116 days -- exactly the magnitude of the
	 * previous constant here, which made cur_timestamp overtake it and
	 * silently flip production to the *positive*-diff branch instead of
	 * the negative one this test means to exercise, and underflowed the
	 * (frame_ntp - before) subtraction below into garbage.
	 *
	 * Deriving frame_ntp from the actual current monotonic time plus a
	 * one-hour margin sidesteps both problems at once: it is guaranteed
	 * to stay ahead of "before"/"after" (captured only milliseconds
	 * later) regardless of how long the host has been up, while staying
	 * nowhere near the overflow ceiling for any realistic uptime */
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &frame_ntp);
	frame_ntp += UINT64_C(3600000000); /* + 1 hour */

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	last_rtp_timestamp = (uint32_t)rtp_timestamp_from_us(
		frame_ntp, VSTRM_RTP_H264_CLK_RATE);

	frame = make_tagged_mbuf_frame(frame_ntp);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &before);

	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);

	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, 0);

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &after);

	CU_ASSERT_EQUAL_FATAL(ctx.count, 1);
	inspect_rtcp(ctx.captured[0], &insp);
	CU_ASSERT_EQUAL_FATAL(insp.sender_report_count, 1);

	/* Expected sr.rtp_timestamp = last_rtp_timestamp -
	 * rtp_timestamp_from_us(frame_ntp - cur_timestamp, CLK_RATE), for
	 * whatever cur_timestamp the real write_rtcp() call used, somewhere
	 * in [before, after]; as cur_timestamp increases, (frame_ntp -
	 * cur_timestamp) decreases, so the expected value (in exact, non-
	 * wrapped arithmetic) increases, i.e. expected_lo <= expected_hi
	 * always holds true BEFORE the uint32_t cast below.
	 *
	 * The cast itself can still wrap (last_rtp_timestamp is already a
	 * huge, wrapped value given frame_ntp's magnitude, tracking real
	 * uptime), and if a slow/loaded CI runner stretches the real gap
	 * between "before" and "after" enough, the wrap boundary can fall
	 * *inside* that gap --
	 * at which point comparing the wrapped bounds directly with
	 * >=/<= (or picking "lo"/"hi" via a plain > on the wrapped values)
	 * breaks even though the produced value is correct. Comparing
	 * wraparound-safe signed deltas instead (same trick as
	 * rtp_diff_seqnum()) sidesteps the issue entirely: it only requires
	 * the true (unwrapped) delta to stay under 2^31, which trivially
	 * holds for a gap of at most a few seconds. */
	expected_lo = (uint32_t)(
		(int64_t)last_rtp_timestamp -
		(int64_t)rtp_timestamp_from_us(frame_ntp - before,
					       VSTRM_RTP_H264_CLK_RATE));
	expected_hi =
		(uint32_t)((int64_t)last_rtp_timestamp -
			   (int64_t)rtp_timestamp_from_us(
				   frame_ntp - after, VSTRM_RTP_H264_CLK_RATE));
	CU_ASSERT_TRUE((int32_t)(insp.last_sr_rtp_timestamp - expected_lo) >=
		       0);
	CU_ASSERT_TRUE((int32_t)(expected_hi - insp.last_sr_rtp_timestamp) >=
		       0);

	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_write_rtcp_ctrl_send_netdown_suppressed(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	unlock_rtcp(sender);

	/* First -ENETDOWN from cbs.send_ctrl: logged, ctrl_netdown_logged is
	 * set (src/vstrm_sender.c:552-563); the error still propagates all
	 * the way back to the caller of send_event() */
	ctx.fail_res = -ENETDOWN;
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, -ENETDOWN);
	CU_ASSERT_EQUAL(ctx.count, 0);

	/* Second consecutive -ENETDOWN: same code path, but this time
	 * ctrl_netdown_logged is already true (the "logged only once"
	 * sub-branch) -- not separately observable from here, but this
	 * still exercises that specific line */
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, -ENETDOWN);
	CU_ASSERT_EQUAL(ctx.count, 0);

	/* A subsequent successful send resets ctrl_netdown_logged and is
	 * captured normally */
	ctx.fail_res = 0;
	res = vstrm_sender_send_event(sender, VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(ctx.count, 1);

	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_send_frame_ancillary_data_as_tpkt_userdata(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct tpkt_list *list = NULL;
	struct tpkt_packet *tpkt;
	const uint8_t ancillary[6] = {1, 2, 3, 4, 5, 6};
	void *got;

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	/* ancillary_data/ancillary_data_len (passed directly to
	 * vstrm_sender_send_frame() below, NOT a field on the frame) is what
	 * the function copies (calloc+memcpy) into each produced
	 * rtp_pkt->userdata, then attaches to the output tpkt via
	 * tpkt_set_user_data(..., tpkt_user_data_release_cb, ...)
	 * (src/vstrm_sender.c:807-820, :877-886) -- the ancillary buffer
	 * itself does not need to outlive this call, since it is copied */
	res = vstrm_sender_send_frame(
		sender, frame, NULL, ancillary, sizeof(ancillary), &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(tpkt_list_get_count(list), 1);

	tpkt = tpkt_list_first(list);
	CU_ASSERT_PTR_NOT_NULL_FATAL(tpkt);
	got = tpkt_get_user_data(tpkt);
	CU_ASSERT_PTR_NOT_NULL_FATAL(got);
	CU_ASSERT_PTR_NOT_EQUAL(got, ancillary);
	CU_ASSERT_EQUAL(memcmp(got, ancillary, sizeof(ancillary)), 0);

	/* Dropping the list's (and thus the tpkt's) last reference must
	 * invoke tpkt_user_data_release_cb(), which frees the copied
	 * buffer -- not directly observable here, but must not crash/leak
	 * (verified on a real build via ASan) */
	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_send_frame_out_timestamp_latency_selection(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct tpkt_list *list = NULL;
	struct tpkt_packet *tpkt;
	struct timespec ts;
	uint64_t before, after;
	uint64_t exp_ts;

	mock_ctrl_ctx_reset(&ctx);
	cfg.dyn.target_packet_size = 1000;
	/* Importance 0: total latency (100ms) is much smaller than network
	 * latency (UINT32_MAX ms, ~136 years) -> out_timestamp1 (frame TS +
	 * total latency) must win: rtp_pkt->out_timestamp==out_timestamp1
	 * (src/vstrm_sender.c:822-846), a value fully deterministic from the
	 * frame's own NTP timestamp (no wall-clock dependency) */
	cfg.dyn.max_total_latency_ms[0] = 100;
	cfg.dyn.max_network_latency_ms[0] = UINT32_MAX;
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	res = vstrm_sender_new(&cfg, &cbs, &ctx, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(tpkt_list_get_count(list), 1);
	tpkt = tpkt_list_first(list);
	CU_ASSERT_PTR_NOT_NULL_FATAL(tpkt);
	CU_ASSERT_EQUAL(tpkt_get_expiration_timestamp(tpkt),
			UINT64_C(1000000) + UINT64_C(100) * 1000);

	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);

	/* Importance 0, second sender: total latency is now huge (frame TS
	 * is an astronomically large ~317000 years, since max_total_latency_ms
	 * alone is capped at UINT32_MAX ms, only ~49.7 days -- not
	 * necessarily larger than a long-uptime test machine's own
	 * CLOCK_MONOTONIC value, so the frame TS itself provides the real
	 * margin here) vs. a small 100ms network latency -> out_timestamp2
	 * (input TS + network latency) must win this time; input TS is real
	 * wall-clock time (cur_timestamp at send_frame() time), so only a
	 * bounded-range check is possible */
	mock_ctrl_ctx_reset(&ctx);
	cfg.dyn.max_total_latency_ms[0] = UINT32_MAX;
	cfg.dyn.max_network_latency_ms[0] = 100;
	res = vstrm_sender_new(&cfg, &cbs, &ctx, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = make_tagged_mbuf_frame(UINT64_C(10000000000000000));
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &before);
	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &after);

	CU_ASSERT_EQUAL_FATAL(tpkt_list_get_count(list), 1);
	tpkt = tpkt_list_first(list);
	CU_ASSERT_PTR_NOT_NULL_FATAL(tpkt);
	exp_ts = tpkt_get_expiration_timestamp(tpkt);
	CU_ASSERT_TRUE(exp_ts >= before + UINT64_C(100) * 1000);
	CU_ASSERT_TRUE(exp_ts <= after + UINT64_C(100) * 1000);

	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);
	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_frame_from_mbuf_coded_video_frame_large_timestamp(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct vstrm_frame *vframe = NULL;
	/* Pins vstrm_frame_new_from_mbuf_coded_video_frame()'s exact output
	 * for a huge timestamp, deterministically (regression test for a
	 * uint64_t overflow in that conversion) */
	uint64_t huge_ts = UINT64_C(10000000000000000);

	frame = make_tagged_mbuf_frame(huge_ts);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_frame_new_from_mbuf_coded_video_frame(
		frame, NULL, &vframe);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vframe);
	CU_ASSERT_EQUAL(vframe->timestamps.ntp, huge_ts);

	vstrm_frame_unref(vframe);
	mbuf_coded_video_frame_unref(frame);
}


static void test_sender_priority_not_propagated_to_tpkt(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct mbuf_coded_video_frame *frame;
	uint8_t data1[20], data2[20];
	struct tpkt_list *list = NULL;
	struct tpkt_packet *tpkt;

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data1, sizeof(data1), H264_NALU_TYPE_SLICE);
	fill_nalu_bytes(data2, sizeof(data2), H264_NALU_TYPE_SLICE);
	/* distinct priorities: NALU priority 0 (highest) vs 5 (lower) */
	res = add_mbuf_nalu(frame, data1, sizeof(data1), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = add_mbuf_nalu(frame, data2, sizeof(data2), 5, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(tpkt_list_get_count(list), 2);

	/* Regression test for a confirmed bug: vstrm_sender.c never calls
	 * tpkt_set_priority() on the outgoing packets, so every packet
	 * keeps tpkt's default priority (0) regardless of the differing
	 * NALU priorities above. If this is ever fixed, this assertion
	 * should be updated deliberately rather than silently drifting. */
	for (tpkt = tpkt_list_first(list); tpkt != NULL;
	     tpkt = tpkt_list_next(list, tpkt)) {
		res = tpkt_get_priority(tpkt);
		CU_ASSERT_EQUAL(res, 0);
	}

	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_getters_and_cfg_dyn(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct tpkt_list *list = NULL;
	struct vstrm_sender_stats stats;
	uint32_t ssrc;
	const struct vmeta_session *meta;
	struct vstrm_sender_cfg_dyn cfg_dyn;
	uint16_t seq;
	uint32_t rtpts;
	int64_t delta;

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Before any frame is sent, next_seqnum/rtp timestamp are still at
	 * their initial values, and clock delta is never available at all
	 * without VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT */
	res = vstrm_sender_get_clock_delta(sender, &delta, NULL);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	res = vstrm_sender_get_ssrc_self(sender, &ssrc);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_sender_get_ssrc_self(NULL, &ssrc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_get_ssrc_self(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* No receiver report ever came in: peer SSRC getter still succeeds
	 * (it just returns whatever the never-updated field holds) */
	res = vstrm_sender_get_ssrc_peer(sender, &ssrc);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_sender_get_session_metadata_self(sender, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta);
	CU_ASSERT_STRING_EQUAL(meta->serial_number, "SN-TEST-1");
	res = vstrm_sender_get_session_metadata_self(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	{
		struct vmeta_session new_meta = {0};
		strncpy(new_meta.serial_number,
			"SN-UPDATED",
			sizeof(new_meta.serial_number) - 1);
		res = vstrm_sender_set_session_metadata_self(sender, &new_meta);
		CU_ASSERT_EQUAL(res, 0);
		res = vstrm_sender_get_session_metadata_self(sender, &meta);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_STRING_EQUAL(meta->serial_number, "SN-UPDATED");
	}

	res = vstrm_sender_get_cfg_dyn(sender, &cfg_dyn);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(cfg_dyn.target_packet_size, 1000);
	res = vstrm_sender_get_cfg_dyn(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	cfg_dyn.target_packet_size = 500;
	res = vstrm_sender_set_cfg_dyn(sender, &cfg_dyn);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_sender_get_cfg_dyn(sender, &cfg_dyn);
	CU_ASSERT_EQUAL(cfg_dyn.target_packet_size, 500);
	res = vstrm_sender_set_cfg_dyn(NULL, &cfg_dyn);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_set_cfg_dyn(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vstrm_sender_get_next_frame_params(sender, 1000000, &seq, &rtpts);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(seq, 0);

	/* Send a frame, then check stats and that next_frame_params advanced */
	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);

	res = vstrm_sender_get_stats(sender, &stats);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(stats.total_packet_count, 1);
	CU_ASSERT_TRUE(stats.total_byte_count > 0);
	res = vstrm_sender_get_stats(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_get_stats(NULL, &stats);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vstrm_sender_get_next_frame_params(sender, 2000000, &seq, &rtpts);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(seq, 1);
	res = vstrm_sender_get_next_frame_params(NULL, 2000000, &seq, &rtpts);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_session_metadata_peer_from_recv_ctrl(void)
{
	int res;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct pomp_buffer *buf;
	size_t pos = 0;
	struct tpkt_packet *pkt;
	const struct vmeta_session *meta;
	struct vmeta_session peer_meta = {0};

	res = new_sender(&sender, &ctx, 0);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_sender_get_session_metadata_peer(sender, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta);
	CU_ASSERT_STRING_EQUAL(meta->serial_number, "");

	strncpy(peer_meta.serial_number,
		"SN-PEER-1",
		sizeof(peer_meta.serial_number) - 1);
	buf = pomp_buffer_new(512);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = vstrm_session_metadata_write_rtcp_sdes(
		buf, &pos, 0xdeadbeef, &peer_meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = tpkt_new_from_buffer(buf, &pkt);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vstrm_sender_recv_ctrl(sender, pkt);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_sender_get_session_metadata_peer(sender, &meta);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_STRING_EQUAL(meta->serial_number, peer_meta.serial_number);
	res = vstrm_sender_get_session_metadata_peer(sender, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_sender_get_session_metadata_peer(NULL, &meta);
	CU_ASSERT_EQUAL(res, -EINVAL);

	tpkt_unref(pkt);
	pomp_buffer_unref(buf);
	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_sender_rtcp_timer_fires(void)
{
	int res;
	struct pomp_loop *loop;
	struct vstrm_sender *sender = NULL;
	struct mock_ctrl_ctx ctx;
	struct vstrm_sender_cfg cfg = {0};
	struct vstrm_sender_cbs cbs = {0};
	struct mbuf_coded_video_frame *frame;
	uint8_t data[10];
	struct tpkt_list *list = NULL;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	mock_ctrl_ctx_reset(&ctx);
	cfg.loop = loop;
	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTCP;
	cfg.dyn.target_packet_size = 1000;
	cbs.send_ctrl = &mock_sender_send_ctrl_cb;
	res = vstrm_sender_new(&cfg, &cbs, &ctx, &sender);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* write_rtcp() (including the periodic timer's own call to it) is a
	 * no-op until at least one RTP packet has been sent */
	frame = make_tagged_mbuf_frame(1000000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	res = add_mbuf_nalu(frame, data, sizeof(data), 0, 0);
	CU_ASSERT_EQUAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_sender_send_frame(sender, frame, NULL, NULL, 0, &list);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	tpkt_list_destroy(list);
	mbuf_coded_video_frame_unref(frame);

	CU_ASSERT_EQUAL(ctx.count, 0);

	/* The RTCP timer is periodic at 100ms; pump well past that */
	res = pomp_loop_wait_and_process(loop, 200);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(ctx.count >= 1);

	mock_ctrl_ctx_clear(&ctx);
	res = vstrm_sender_destroy(sender);
	CU_ASSERT_EQUAL(res, 0);
	pomp_loop_destroy(loop);
}


CU_TestInfo g_vstrm_test_sender[] = {
	{FN("sender-new-destroy-invalid-args"),
	 &test_sender_new_destroy_invalid_args},
	{FN("sender-send-frame-produces-expected-rtp"),
	 &test_sender_send_frame_produces_expected_rtp},
	{FN("sender-rtcp-sdes-compact-vs-full"),
	 &test_sender_rtcp_sdes_compact_vs_full},
	{FN("sender-send-event-and-goodbye"),
	 &test_sender_send_event_and_goodbye},
	{FN("sender-recv-ctrl-receiver-report-and-video-stats-callbacks"),
	 &test_sender_recv_ctrl_receiver_report_and_video_stats_callbacks},
	{FN("sender-recv-ctrl-receiver-report-rtt-computation"),
	 &test_sender_recv_ctrl_receiver_report_rtt_computation},
	{FN("sender-get-clock-delta-full-round-trip"),
	 &test_sender_get_clock_delta_full_round_trip},
	{FN("sender-recv-ctrl-session-metadata-peer-changed"),
	 &test_sender_recv_ctrl_session_metadata_peer_changed},
	{FN("sender-dbg-files-created-and-written"),
	 &test_sender_dbg_files_created_and_written},
	{FN("sender-write-rtcp-sender-report-negative-diff"),
	 &test_sender_write_rtcp_sender_report_negative_diff},
	{FN("sender-write-rtcp-ctrl-send-netdown-suppressed"),
	 &test_sender_write_rtcp_ctrl_send_netdown_suppressed},
	{FN("sender-send-frame-ancillary-data-as-tpkt-userdata"),
	 &test_sender_send_frame_ancillary_data_as_tpkt_userdata},
	{FN("sender-send-frame-out-timestamp-latency-selection"),
	 &test_sender_send_frame_out_timestamp_latency_selection},
	{FN("frame-from-mbuf-coded-video-frame-large-timestamp"),
	 &test_frame_from_mbuf_coded_video_frame_large_timestamp},
	{FN("sender-priority-not-propagated-to-tpkt"),
	 &test_sender_priority_not_propagated_to_tpkt},
	{FN("sender-getters-and-cfg-dyn"), &test_sender_getters_and_cfg_dyn},
	{FN("sender-session-metadata-peer-from-recv-ctrl"),
	 &test_sender_session_metadata_peer_from_recv_ctrl},
	{FN("sender-rtcp-timer-fires"), &test_sender_rtcp_timer_fires},

	CU_TEST_INFO_NULL,
};
