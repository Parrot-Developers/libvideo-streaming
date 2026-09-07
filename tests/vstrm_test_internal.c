/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"

#include <ctype.h>

#include <video-metadata/vmeta.h>


static void test_clock_delta_read_write_roundtrip(void)
{
	struct pomp_buffer *buf;
	size_t pos = 0;
	int res;
	struct vstrm_clock_delta in = {
		.originate_ts = 1000, .receive_ts = 2000, .transmit_ts = 3000};
	struct vstrm_clock_delta out;

	memset(&out, 0, sizeof(out));

	buf = pomp_buffer_new(32);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = vstrm_clock_delta_write(buf, &pos, &in);
	CU_ASSERT_EQUAL(res, 0);

	pos = 0;
	res = vstrm_clock_delta_read(buf, &pos, &out);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(out.originate_ts, in.originate_ts);
	CU_ASSERT_EQUAL(out.receive_ts, in.receive_ts);
	CU_ASSERT_EQUAL(out.transmit_ts, in.transmit_ts);

	/* NULL args */
	res = vstrm_clock_delta_write(NULL, &pos, &in);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_clock_delta_read(buf, &pos, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_clock_delta_process_single_and_windowed(void)
{
	struct vstrm_clock_delta_ctx ctx;
	struct vstrm_clock_delta delta;
	int res;
	uint64_t receive_ts;

	memset(&ctx, 0, sizeof(ctx));
	vstrm_clock_delta_init(&ctx);

	/* (a) mismatched originate_ts: ctx.expected_originate_ts is 0 right
	 * after init, and a non-zero originate_ts never matches it */
	delta.originate_ts = 12345;
	delta.receive_ts = 20000;
	delta.transmit_ts = 21000;
	res = vstrm_clock_delta_process(&ctx, &delta, 100000);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(ctx.clock_delta_valid, 0);
	CU_ASSERT_EQUAL(ctx.window_size, 0);

	/* (b) rt_delay <= 0: force it by making the peer's own round trip
	 * (transmit_ts - receive_ts) exceed our round trip
	 * (receive_ts - originate_ts) */
	ctx.expected_originate_ts = 1000;
	delta.originate_ts = 1000;
	delta.receive_ts = 2000;
	delta.transmit_ts = 500000; /* peer RTT (498000) >> our RTT (9000) */
	res = vstrm_clock_delta_process(&ctx, &delta, 10000);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(ctx.clock_delta_valid, 0);
	CU_ASSERT_EQUAL(ctx.window_size, 0);
	/* the token is still consumed even on rt_delay<=0 rejection */
	CU_ASSERT_EQUAL(ctx.expected_originate_ts, 0);

	/* (c) fill a full CLOCK_DELTA_WINDOW_SIZE (10) window: each round i
	 * has rt_delay=5000, clock_delta=100*i, except round 3 which has
	 * rt_delay=2000 (the minimum) and clock_delta=777 (a marker value).
	 * Using originate_ts=T, peer_receive_ts=T+a, peer_transmit_ts=
	 * T+a+b, receive_ts=T+a+b+c gives rt_delay=a+c and
	 * clock_delta=(a-c+1)/2 regardless of b (as long as b>=1000); so for
	 * a target (rt_delay, clock_delta) pair, a=(rt_delay+2*clock_delta)/2
	 * and c=rt_delay-a. */
	for (int i = 0; i < CLOCK_DELTA_WINDOW_SIZE; i++) {
		int64_t rt_delay = (i == 3) ? 2000 : 5000;
		int64_t clock_delta = (i == 3) ? 777 : 100 * i;
		int64_t a = (rt_delay + 2 * clock_delta) / 2;
		int64_t c = rt_delay - a;
		/* Keep all 10 rounds well within CLOCK_DELTA_WINDOW_TIMEOUT
		 * (5s) of the first round's receive_ts, or the window would
		 * close early on the timeout condition instead of the
		 * intended window_pos>=CLOCK_DELTA_WINDOW_SIZE count */
		uint64_t t = 1000000ULL + (uint64_t)i * 10000ULL;
		uint64_t b = 100000;

		ctx.expected_originate_ts = t;
		delta.originate_ts = t;
		delta.receive_ts = t + (uint64_t)a;
		delta.transmit_ts = t + (uint64_t)a + b;
		receive_ts = t + (uint64_t)a + b + (uint64_t)c;

		res = vstrm_clock_delta_process(&ctx, &delta, receive_ts);
		CU_ASSERT_EQUAL(res, 0);
	}
	CU_ASSERT_EQUAL(ctx.clock_delta_valid, 1);
	CU_ASSERT_EQUAL(ctx.window_size, CLOCK_DELTA_WINDOW_SIZE);
	CU_ASSERT_EQUAL(ctx.window_pos, 0);
	/* window completion overwrites the averages with the min-RTD
	 * sample's exact values, regardless of any blending seen so far */
	CU_ASSERT_EQUAL(ctx.rt_delay_min_avg, 2000);
	CU_ASSERT_EQUAL(ctx.clock_delta_avg, 777);

	/* (d) one more round replacing window slot 0 (old rt_delay=5000,
	 * clock_delta=0) with a new minimum (rt_delay=1000, clock_delta=999)
	 * exercises the sliding-window EWMA (alpha=1/32):
	 * rt_delay_min_avg: 2000 + (1000 - 2000 + 16) / 32 = 2000 + (-30) =
	 * 1970 clock_delta_avg:  777 + (999 - 777 + 16) / 32 = 777 + 7 = 784 */
	{
		int64_t rt_delay = 1000;
		int64_t clock_delta = 999;
		int64_t a = (rt_delay + 2 * clock_delta) / 2;
		int64_t c = rt_delay - a;
		uint64_t t = 50000000;
		uint64_t b = 100000;

		ctx.expected_originate_ts = t;
		delta.originate_ts = t;
		delta.receive_ts = t + (uint64_t)a;
		delta.transmit_ts = t + (uint64_t)a + b;
		receive_ts = t + (uint64_t)a + b + (uint64_t)c;

		res = vstrm_clock_delta_process(&ctx, &delta, receive_ts);
		CU_ASSERT_EQUAL(res, 0);
	}
	CU_ASSERT_EQUAL(ctx.rt_delay_min_avg, 1970);
	CU_ASSERT_EQUAL(ctx.clock_delta_avg, 784);

	/* NULL args / zero receive_ts */
	res = vstrm_clock_delta_process(NULL, &delta, receive_ts);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_clock_delta_process(&ctx, &delta, 0);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_event_read_write_roundtrip(void)
{
	struct pomp_buffer *buf;
	size_t pos;
	int res;
	enum vstrm_event ev;
	static const enum vstrm_event all[] = {
		VSTRM_EVENT_NONE,
		VSTRM_EVENT_RECONFIGURE,
		VSTRM_EVENT_RESOLUTION_CHANGE,
		VSTRM_EVENT_PHOTO_TRIGGER,
		VSTRM_EVENT_FRAMERATE_CHANGE,
	};

	for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
		buf = pomp_buffer_new(8);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		pos = 0;
		res = vstrm_event_write(buf, &pos, all[i]);
		CU_ASSERT_EQUAL(res, 0);
		pos = 0;
		res = vstrm_event_read(buf, &pos, &ev);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(ev, all[i]);
		pomp_buffer_unref(buf);
	}

	/* VSTRM_EVENT_MAX itself is not a valid value: write() only checks
	 * the wire range (0..UINT8_MAX), but read() rejects event>=MAX */
	buf = pomp_buffer_new(8);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	pos = 0;
	res = vstrm_event_write(buf, &pos, VSTRM_EVENT_MAX);
	CU_ASSERT_EQUAL(res, 0);
	pos = 0;
	res = vstrm_event_read(buf, &pos, &ev);
	CU_ASSERT_EQUAL(res, -EPROTO);
	pomp_buffer_unref(buf);

	CU_ASSERT_EQUAL(vstrm_event_from_str("RECONFIGURE"),
			VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(vstrm_event_from_str("reconfigure"),
			VSTRM_EVENT_RECONFIGURE);
	CU_ASSERT_EQUAL(vstrm_event_from_str("bogus"), VSTRM_EVENT_NONE);
	CU_ASSERT_STRING_EQUAL(vstrm_event_to_str(VSTRM_EVENT_RECONFIGURE),
			       "RECONFIGURE");
	CU_ASSERT_STRING_EQUAL(vstrm_event_to_str(VSTRM_EVENT_NONE), "NONE");
	CU_ASSERT_STRING_EQUAL(vstrm_event_to_str((enum vstrm_event)999),
			       "NONE");
}


/* Data-driven: every enum vstrm_event value (except the VSTRM_EVENT_MAX
 * sentinel, not a real event) paired with its exact wire string, looping
 * over the table to assert both directions of the conversion instead of
 * hand-picking one value like test_event_read_write_roundtrip() does */
static void test_event_to_str_from_str(void)
{
	static const struct {
		enum vstrm_event event;
		const char *str;
	} table[] = {
		{VSTRM_EVENT_NONE, "NONE"},
		{VSTRM_EVENT_RECONFIGURE, "RECONFIGURE"},
		{VSTRM_EVENT_RESOLUTION_CHANGE, "RESOLUTION_CHANGE"},
		{VSTRM_EVENT_PHOTO_TRIGGER, "PHOTO_TRIGGER"},
		{VSTRM_EVENT_FRAMERATE_CHANGE, "FRAMERATE_CHANGE"},
	};
	size_t i, j;
	char lower[32];

	for (i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
		/* to_str */
		CU_ASSERT_STRING_EQUAL(vstrm_event_to_str(table[i].event),
				       table[i].str);

		/* from_str, exact case */
		CU_ASSERT_EQUAL(vstrm_event_from_str(table[i].str),
				table[i].event);

		/* from_str is documented case-insensitive */
		CU_ASSERT_TRUE_FATAL(strlen(table[i].str) < sizeof(lower));
		for (j = 0; table[i].str[j] != '\0'; j++)
			lower[j] =
				(char)tolower((unsigned char)table[i].str[j]);
		lower[j] = '\0';
		CU_ASSERT_EQUAL(vstrm_event_from_str(lower), table[i].event);
	}
}


static void test_video_stats_read_write_roundtrip(void)
{
	int res;
	struct vstrm_video_stats meta, out_meta;
	struct vstrm_video_stats_dyn dyn, out_dyn;
	struct pomp_buffer *buf;
	size_t pos;

	memset(&meta, 0, sizeof(meta));
	meta.version = VSTRM_VIDEO_STATS_VERSION_2;
	meta.timestamp = 123456789;
	meta.v2.total_frame_count = 100;
	meta.v2.output_frame_count = 95;
	meta.v2.errored_output_frame_count = 2;
	meta.v2.missed_frame_count = 5;
	meta.v2.discarded_frame_count = 1;
	meta.v2.errored_second_count = 3;
	meta.v2.presentation_frame_count = 90;
	meta.mb_status_class_count = VSTRM_H264_MB_STATUS_CLASS_COUNT;
	meta.mb_status_zone_count = VSTRM_H264_MB_STATUS_ZONE_COUNT;

	res = vstrm_video_stats_dyn_init(
		&dyn, meta.mb_status_class_count, meta.mb_status_zone_count);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	for (uint32_t i = 0; i < meta.mb_status_zone_count; i++)
		dyn.errored_second_count_by_zone[i] = i + 1;
	for (uint32_t j = 0; j < meta.mb_status_class_count; j++) {
		for (uint32_t i = 0; i < meta.mb_status_zone_count; i++) {
			uint32_t k = j * meta.mb_status_zone_count + i;
			dyn.macroblock_status[k] = k + 1;
		}
	}

	buf = pomp_buffer_new(1024);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	pos = 0;
	res = vstrm_video_stats_write(buf, &pos, &meta, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&out_meta, 0, sizeof(out_meta));
	memset(&out_dyn, 0, sizeof(out_dyn));
	pos = 0;
	res = vstrm_video_stats_read(buf, &pos, &out_meta, &out_dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(out_meta.version, meta.version);
	CU_ASSERT_EQUAL(out_meta.timestamp, meta.timestamp);
	CU_ASSERT_EQUAL(out_meta.v2.total_frame_count,
			meta.v2.total_frame_count);
	CU_ASSERT_EQUAL(out_meta.v2.output_frame_count,
			meta.v2.output_frame_count);
	CU_ASSERT_EQUAL(out_meta.v2.missed_frame_count,
			meta.v2.missed_frame_count);
	CU_ASSERT_EQUAL(out_meta.v2.presentation_frame_count,
			meta.v2.presentation_frame_count);
	for (uint32_t i = 0; i < meta.mb_status_zone_count; i++) {
		CU_ASSERT_EQUAL(out_dyn.errored_second_count_by_zone[i],
				dyn.errored_second_count_by_zone[i]);
	}
	for (uint32_t k = 0;
	     k < meta.mb_status_class_count * meta.mb_status_zone_count;
	     k++) {
		CU_ASSERT_EQUAL(out_dyn.macroblock_status[k],
				dyn.macroblock_status[k]);
	}

	/* mismatched class/zone counts between meta and dyn are rejected */
	{
		struct vstrm_video_stats_dyn bad_dyn;
		memset(&bad_dyn, 0, sizeof(bad_dyn));
		res = vstrm_video_stats_dyn_init(&bad_dyn,
						 meta.mb_status_class_count + 1,
						 meta.mb_status_zone_count);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = vstrm_video_stats_write(buf, &pos, &meta, &bad_dyn);
		CU_ASSERT_EQUAL(res, -EINVAL);
		vstrm_video_stats_dyn_clear(&bad_dyn);
	}

	/* dyn increment helpers */
	{
		struct vstrm_video_stats_dyn inc_dyn;
		memset(&inc_dyn, 0, sizeof(inc_dyn));
		res = vstrm_video_stats_dyn_init(&inc_dyn, 7, 5);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		vstrm_video_stats_dyn_inc_mb_status_count(&inc_dyn, 2, 3);
		vstrm_video_stats_dyn_inc_mb_status_count(&inc_dyn, 2, 3);
		CU_ASSERT_EQUAL(inc_dyn.macroblock_status[2 * 5 + 3], 2);
		vstrm_video_stats_dyn_inc_errored_second_count_by_zone(&inc_dyn,
								       4);
		CU_ASSERT_EQUAL(inc_dyn.errored_second_count_by_zone[4], 1);
		vstrm_video_stats_dyn_clear(&inc_dyn);
	}

	vstrm_video_stats_dyn_clear(&dyn);
	vstrm_video_stats_dyn_clear(&out_dyn);
	pomp_buffer_unref(buf);
}


static void test_meta_header_pack_unpack_roundtrip(void)
{
	uint16_t packed;
	uint8_t last_pack, current_pack, padding;
	int res;

	packed = vstrm_rtp_h264_meta_header_pack(5, 3, 2);
	res = vstrm_rtp_h264_meta_header_unpack(
		packed, &last_pack, &current_pack, &padding);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(last_pack, 5);
	CU_ASSERT_EQUAL(current_pack, 3);
	CU_ASSERT_EQUAL(padding, 2);

	/* Boundary values: 6-bit fields max 0x3f, padding 2-bit max 0x3 */
	packed = vstrm_rtp_h264_meta_header_pack(0x3f, 0x3f, 0x3);
	res = vstrm_rtp_h264_meta_header_unpack(
		packed, &last_pack, &current_pack, &padding);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(last_pack, 0x3f);
	CU_ASSERT_EQUAL(current_pack, 0x3f);
	CU_ASSERT_EQUAL(padding, 0x3);

	res = vstrm_rtp_h264_meta_header_unpack(
		packed, NULL, &current_pack, &padding);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void sdes_item_read_cb(uint32_t ssrc,
			      const struct rtcp_pkt_sdes_item *item,
			      void *userdata)
{
	struct vmeta_session *out = userdata;
	UNUSED(ssrc);
	vstrm_session_metadata_read_rtcp_sdes(item, out);
}


static void test_session_metadata_sdes_roundtrip(void)
{
	int res;
	struct vmeta_session in_meta;
	struct vmeta_session out_meta;
	struct pomp_buffer *buf;
	size_t pos = 0;
	struct rtcp_pkt_read_cbs cbs = {0};

	memset(&in_meta, 0, sizeof(in_meta));
	strncpy(in_meta.serial_number,
		"SN-12345",
		sizeof(in_meta.serial_number) - 1);
	strncpy(in_meta.friendly_name,
		"MyDrone",
		sizeof(in_meta.friendly_name) - 1);
	strncpy(in_meta.software_version,
		"1.2.3",
		sizeof(in_meta.software_version) - 1);
	strncpy(in_meta.maker, "Parrot", sizeof(in_meta.maker) - 1);

	buf = pomp_buffer_new(1024);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = vstrm_session_metadata_write_rtcp_sdes(
		buf, &pos, 0x12345678, &in_meta);
	CU_ASSERT_EQUAL(res, 0);

	memset(&out_meta, 0, sizeof(out_meta));
	cbs.sdes_item = &sdes_item_read_cb;
	res = rtcp_pkt_read(buf, &cbs, &out_meta);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_STRING_EQUAL(out_meta.serial_number, in_meta.serial_number);
	CU_ASSERT_STRING_EQUAL(out_meta.friendly_name, in_meta.friendly_name);
	CU_ASSERT_STRING_EQUAL(out_meta.software_version,
			       in_meta.software_version);
	CU_ASSERT_STRING_EQUAL(out_meta.maker, in_meta.maker);

	pomp_buffer_unref(buf);
}


static void test_video_stats_v1_roundtrip_and_csv(void)
{
	int res;
	struct vstrm_video_stats meta, out_meta;
	struct vstrm_video_stats_dyn dyn, out_dyn, copy_dyn;
	struct pomp_buffer *buf;
	size_t pos;
	FILE *csv;

	memset(&meta, 0, sizeof(meta));
	meta.version = VSTRM_VIDEO_STATS_VERSION_1;
	meta.timestamp = 987654321;
	meta.v1.rssi = -42;
	meta.v1.total_frame_count = 10;
	meta.v1.output_frame_count = 9;
	meta.v1.errored_output_frame_count = 1;
	meta.v1.missed_frame_count = 2;
	meta.v1.discarded_frame_count = 1;
	meta.v1.errored_second_count = 3;
	meta.mb_status_class_count = 2;
	meta.mb_status_zone_count = 2;

	res = vstrm_video_stats_dyn_init(&dyn, 2, 2);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	for (uint32_t i = 0; i < 2; i++)
		dyn.errored_second_count_by_zone[i] = i + 1;
	for (uint32_t k = 0; k < 4; k++)
		dyn.macroblock_status[k] = k + 10;

	buf = pomp_buffer_new(1024);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	pos = 0;
	res = vstrm_video_stats_write(buf, &pos, &meta, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&out_meta, 0, sizeof(out_meta));
	memset(&out_dyn, 0, sizeof(out_dyn));
	pos = 0;
	res = vstrm_video_stats_read(buf, &pos, &out_meta, &out_dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(out_meta.version, meta.version);
	CU_ASSERT_EQUAL(out_meta.timestamp, meta.timestamp);
	CU_ASSERT_EQUAL(out_meta.v1.rssi, meta.v1.rssi);
	CU_ASSERT_EQUAL(out_meta.v1.total_frame_count,
			meta.v1.total_frame_count);
	CU_ASSERT_EQUAL(out_meta.v1.errored_second_count,
			meta.v1.errored_second_count);
	for (uint32_t i = 0; i < 2; i++) {
		CU_ASSERT_EQUAL(out_dyn.errored_second_count_by_zone[i],
				dyn.errored_second_count_by_zone[i]);
	}
	for (uint32_t k = 0; k < 4; k++) {
		CU_ASSERT_EQUAL(out_dyn.macroblock_status[k],
				dyn.macroblock_status[k]);
	}

	/* Unknown/bad version rejected on both write and read */
	{
		struct vstrm_video_stats bad = meta;
		bad.version = 99;
		pos = 0;
		res = vstrm_video_stats_write(buf, &pos, &bad, &dyn);
		CU_ASSERT_EQUAL(res, -EINVAL);
	}
	{
		struct pomp_buffer *badbuf = pomp_buffer_new(8);
		uint8_t bad_version = 99;
		size_t bpos = 0;
		struct vstrm_video_stats bad_out;
		struct vstrm_video_stats_dyn bad_dyn = {0};
		memset(&bad_out, 0, sizeof(bad_out));
		pomp_buffer_write(badbuf, &bpos, &bad_version, 1);
		bpos = 0;
		res = vstrm_video_stats_read(badbuf, &bpos, &bad_out, &bad_dyn);
		CU_ASSERT_EQUAL(res, -EIO);
		pomp_buffer_unref(badbuf);
	}

	/* dyn deep-copy */
	memset(&copy_dyn, 0, sizeof(copy_dyn));
	res = vstrm_video_stats_dyn_copy(&copy_dyn, &dyn);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	for (uint32_t k = 0; k < 4; k++) {
		CU_ASSERT_EQUAL(copy_dyn.macroblock_status[k],
				dyn.macroblock_status[k]);
	}
	res = vstrm_video_stats_dyn_copy(NULL, &dyn);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* CSV header/row writers, both versions, plus the csv==NULL no-op
	 * and the mismatched-count no-op. Note: out_meta/out_dyn above were
	 * read back from a v1-written buffer, so they are v1-versioned too
	 * -- a genuinely v2-versioned struct is needed here to actually
	 * reach video_stats_csv_{header,write}_v2() rather than re-hitting
	 * the v1 variants a second time. */
	csv = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(csv);
	vstrm_video_stats_csv_header(csv, VSTRM_VIDEO_STATS_VERSION_1, 2, 2);
	vstrm_video_stats_csv_write(csv, &meta, &dyn);
	{
		struct vstrm_video_stats v2_meta = {0};
		struct vstrm_video_stats_dyn v2_dyn = {0};
		v2_meta.version = VSTRM_VIDEO_STATS_VERSION_2;
		v2_meta.timestamp = 42;
		v2_meta.v2.total_frame_count = 7;
		vstrm_video_stats_csv_header(
			csv, VSTRM_VIDEO_STATS_VERSION_2, 0, 0);
		vstrm_video_stats_csv_write(csv, &v2_meta, &v2_dyn);
	}
	fflush(csv);
	CU_ASSERT_TRUE(ftell(csv) > 0);
	fclose(csv);

	vstrm_video_stats_csv_header(NULL, VSTRM_VIDEO_STATS_VERSION_1, 2, 2);
	vstrm_video_stats_csv_write(NULL, &meta, &dyn);

	vstrm_video_stats_dyn_clear(&dyn);
	vstrm_video_stats_dyn_clear(&out_dyn);
	vstrm_video_stats_dyn_clear(&copy_dyn);
	pomp_buffer_unref(buf);
}


static void test_h264_sei_streaming_v1_v4_roundtrip(void)
{
	struct vstrm_h264_sei_streaming_v1 v1_in = {0}, v1_out = {0};
	struct vstrm_h264_sei_streaming_v4 v4_in, v4_out;
	uint8_t uuid[16];
	uint8_t buf[64];
	size_t len;
	int res;

	/* v1: variable-length, 2 slices */
	v1_in.index_in_gop = 3;
	v1_in.slice_count = 2;
	v1_in.slice_mb_count[0] = 10;
	v1_in.slice_mb_count[1] = 20;
	len = sizeof(buf);
	res = vstrm_h264_sei_streaming_v1_write(&v1_in, uuid, buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(len, 2 + 2 * 2);
	CU_ASSERT_TRUE(vstrm_h264_sei_streaming_is_v1(uuid));
	CU_ASSERT_FALSE(vstrm_h264_sei_streaming_is_v2(uuid));
	CU_ASSERT_TRUE(vstrm_h264_is_sei_streaming(uuid));

	res = vstrm_h264_sei_streaming_v1_read(&v1_out, uuid, buf, len);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(v1_out.index_in_gop, v1_in.index_in_gop);
	CU_ASSERT_EQUAL(v1_out.slice_count, v1_in.slice_count);
	CU_ASSERT_EQUAL(v1_out.slice_mb_count[0], v1_in.slice_mb_count[0]);
	CU_ASSERT_EQUAL(v1_out.slice_mb_count[1], v1_in.slice_mb_count[1]);

	/* Wrong UUID is rejected */
	{
		uint8_t wrong_uuid[16];
		memset(wrong_uuid, 0xAA, sizeof(wrong_uuid));
		res = vstrm_h264_sei_streaming_v1_read(
			&v1_out, wrong_uuid, buf, len);
		CU_ASSERT_EQUAL(res, -EIO);
	}

	/* v4: fixed 4 bytes, two distinct fields */
	memset(&v4_in, 0, sizeof(v4_in));
	memset(&v4_out, 0, sizeof(v4_out));
	v4_in.slice_mb_count = 100;
	v4_in.slice_mb_count_recovery_point = 50;
	len = sizeof(buf);
	res = vstrm_h264_sei_streaming_v4_write(&v4_in, uuid, buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(len, 4);
	CU_ASSERT_TRUE(vstrm_h264_sei_streaming_is_v4(uuid));
	CU_ASSERT_FALSE(vstrm_h264_sei_streaming_is_v1(uuid));

	res = vstrm_h264_sei_streaming_v4_read(&v4_out, uuid, buf, len);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(v4_out.slice_mb_count, v4_in.slice_mb_count);
	CU_ASSERT_EQUAL(v4_out.slice_mb_count_recovery_point,
			v4_in.slice_mb_count_recovery_point);

	/* A UUID matching none of v1/v2/v4 */
	{
		uint8_t unknown_uuid[16];
		memset(unknown_uuid, 0, sizeof(unknown_uuid));
		CU_ASSERT_FALSE(vstrm_h264_is_sei_streaming(unknown_uuid));
	}
}


static void test_session_metadata_extra_fields(void)
{
	int res;
	struct vmeta_session in_meta;
	struct vmeta_session out_meta;
	struct pomp_buffer *buf;
	size_t pos = 0;

	memset(&in_meta, 0, sizeof(in_meta));
	strncpy(in_meta.serial_number,
		"SN-1",
		sizeof(in_meta.serial_number) - 1);
	strncpy(in_meta.model, "AnafiX", sizeof(in_meta.model) - 1);
	strncpy(in_meta.build_id, "build-42", sizeof(in_meta.build_id) - 1);
	in_meta.location.valid = 1;
	in_meta.location.latitude = 48.8;
	in_meta.location.longitude = 2.3;
	in_meta.location.altitude_wgs84ellipsoid = 100.0;
	in_meta.location.altitude_egm96amsl = 95.0;

	buf = pomp_buffer_new(1024);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	res = vstrm_session_metadata_write_rtcp_sdes(
		buf, &pos, 0x12345678, &in_meta);
	CU_ASSERT_EQUAL(res, 0);

	memset(&out_meta, 0, sizeof(out_meta));
	{
		struct rtcp_pkt_read_cbs cbs = {0};
		cbs.sdes_item = &sdes_item_read_cb;
		res = rtcp_pkt_read(buf, &cbs, &out_meta);
		CU_ASSERT_EQUAL(res, 0);
	}

	CU_ASSERT_STRING_EQUAL(out_meta.model, in_meta.model);
	CU_ASSERT_STRING_EQUAL(out_meta.build_id, in_meta.build_id);
	CU_ASSERT_EQUAL(out_meta.location.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(
		out_meta.location.latitude, in_meta.location.latitude, 0.001);
	CU_ASSERT_DOUBLE_EQUAL(
		out_meta.location.longitude, in_meta.location.longitude, 0.001);

	pomp_buffer_unref(buf);
}


static void test_session_metadata_read_email_phone_note(void)
{
	int res;
	struct vmeta_session meta;
	struct vmeta_session before;
	struct rtcp_pkt_sdes_item item = {0};

	/* RTCP_PKT_SDES_TYPE_EMAIL/PHONE/NOTE (src/vstrm_session_metadata.c:
	 * 211-239): vstrm_session_metadata_read_rtcp_sdes() has real switch
	 * cases for all three, but the vmeta_session_streaming_sdes_read()
	 * call they make has no matching case for any of them (no
	 * vmeta_session field maps to email/phone/note) -- so the call must
	 * not crash, but it also has no observable effect: `meta` must come
	 * out completely unchanged. This pins down that documented (current)
	 * behavior rather than assuming it. */
	memset(&meta, 0, sizeof(meta));
	strncpy(meta.serial_number, "SN-BASE", sizeof(meta.serial_number) - 1);
	before = meta;

	item.type = RTCP_PKT_SDES_TYPE_EMAIL;
	item.data = (const uint8_t *)"user@example.com";
	item.data_len = (uint8_t)strlen((const char *)item.data);
	res = vstrm_session_metadata_read_rtcp_sdes(&item, &meta);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(memcmp(&meta, &before, sizeof(meta)), 0);

	item.type = RTCP_PKT_SDES_TYPE_PHONE;
	item.data = (const uint8_t *)"+33123456789";
	item.data_len = (uint8_t)strlen((const char *)item.data);
	res = vstrm_session_metadata_read_rtcp_sdes(&item, &meta);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(memcmp(&meta, &before, sizeof(meta)), 0);

	item.type = RTCP_PKT_SDES_TYPE_NOTE;
	item.data = (const uint8_t *)"a note";
	item.data_len = (uint8_t)strlen((const char *)item.data);
	res = vstrm_session_metadata_read_rtcp_sdes(&item, &meta);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(memcmp(&meta, &before, sizeof(meta)), 0);
}


CU_TestInfo g_vstrm_test_internal[] = {
	{FN("clock-delta-read-write-roundtrip"),
	 &test_clock_delta_read_write_roundtrip},
	{FN("clock-delta-process-single-and-windowed"),
	 &test_clock_delta_process_single_and_windowed},
	{FN("event-read-write-roundtrip"), &test_event_read_write_roundtrip},
	{FN("event-to-str-from-str"), &test_event_to_str_from_str},
	{FN("video-stats-read-write-roundtrip"),
	 &test_video_stats_read_write_roundtrip},
	{FN("video-stats-v1-roundtrip-and-csv"),
	 &test_video_stats_v1_roundtrip_and_csv},
	{FN("h264-sei-streaming-v1-v4-roundtrip"),
	 &test_h264_sei_streaming_v1_v4_roundtrip},
	{FN("meta-header-pack-unpack-roundtrip"),
	 &test_meta_header_pack_unpack_roundtrip},
	{FN("session-metadata-sdes-roundtrip"),
	 &test_session_metadata_sdes_roundtrip},
	{FN("session-metadata-extra-fields"),
	 &test_session_metadata_extra_fields},
	{FN("session-metadata-read-email-phone-note"),
	 &test_session_metadata_read_email_phone_note},

	CU_TEST_INFO_NULL,
};
