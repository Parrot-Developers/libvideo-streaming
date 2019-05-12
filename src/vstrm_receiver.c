/**
 * Copyright (c) 2016 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vstrm_priv.h"

#define RTCP_PERIOD_MS 100
#define RTCP_FULL_SDES_PERIOD_US 2000000
#define RTCP_VIDEO_STATS_PERIOD_US 1000000

#define RTP_SEQ_MOD (1 << 16)
#define MAX_DROPOUT 20000
#define MAX_MISORDER 20000
#define MIN_SEQUENTIAL 0


struct vstrm_receiver {
	struct vstrm_receiver_cfg cfg;
	struct vstrm_receiver_cbs cbs;
	void *cbs_userdata;
	struct vmeta_session *session_metadata_self;
	uint32_t ssrc;
	struct vstrm_rtp_h264_rx *rtp_h264;

	/* Codec info provided out of band */
	struct vstrm_codec_info codec_info;
	uint32_t codec_info_ssrc;

	struct {
		uint32_t ssrc;

		/* Highest seq. number seen */
		uint16_t max_seq;

		/* Shifted count of seq. num cycles */
		uint32_t cycles;

		/* Base seq. number */
		uint32_t base_seq;

		/* Last 'bad' seq. number + 1 */
		uint32_t bad_seq;

		/* Seq. packets till source is valid */
		uint32_t probation;

		/* Packets received */
		uint32_t received;

		/* Packet expected at last interval */
		uint32_t expected_prior;

		/* Packet received at last interval */
		uint32_t received_prior;

		/* Bytes received */
		uint32_t received_bytes;

		struct vmeta_session session_metadata_peer;
		struct pomp_timer *rtcp_timer;
		struct rtcp_pkt_sender_report last_sr;
		int last_sr_valid;
		uint64_t last_sr_timestamp;
		struct rtp_jitter *rtp_jitter;

		uint64_t rtcp_recv_ts;
		uint64_t full_sdes_send_ts;
		uint64_t clock_delta_send_ts;
		uint64_t video_stats_send_ts;
		int64_t tsAnum;
		int64_t tsAden;
		uint32_t lastSrInterval;
		uint32_t srIntervalPacketCount;
		uint32_t srIntervalByteCount;

		struct vstrm_clock_delta last_clock_delta;
		uint64_t clock_delta_recv_ts;
		struct vstrm_clock_delta_ctx clock_delta_ctx;
	} source;

	struct {
		char *dir;
		FILE *rtp_in;
		FILE *rtp_jitter;
		FILE *stream;
		FILE *video_stats_csv;
		int video_stats_csv_header;
	} dbg;
};


static void
vstrm_receiver_rtcp_sender_report_cb(const struct rtcp_pkt_sender_report *sr,
				     void *userdata)
{
	struct vstrm_receiver *self = userdata;
	uint32_t last_rtp_timestamp = 0, rtp_timestamp = 0;
	uint64_t last_ntp_timestamp = 0, ntp_timestamp = 0;
	int64_t tsAnum = 0, tsAden = 0;

	if (sr->ssrc != self->source.ssrc)
		return;

	rtp_timestamp = sr->rtp_timestamp;
	ntp_timestamp64_to_us(&sr->ntp_timestamp, &ntp_timestamp);

	if (!self->source.last_sr_valid)
		goto out;

	last_rtp_timestamp = self->source.last_sr.rtp_timestamp;
	ntp_timestamp64_to_us(&self->source.last_sr.ntp_timestamp,
			      &last_ntp_timestamp);

	/* NTP to RTP linear regression:
	 * for samples m and n, RTPn = a * NTPn + b and RTPm = a * NTPm + b
	 * <=> RTPn - RTPm = a * (NTPn - NTPm)
	 * <=> NTPn = (RTPn - RTPm) / a + NTPm
	 * m sample is the last RTP/NTP pair received from a RTCP sender report
	 * n sample in the current RTP packet for which we want to derive the
	 * NTP timestamp
	 */
	tsAnum = (int32_t)(rtp_timestamp - last_rtp_timestamp);
	tsAden = (int64_t)(ntp_timestamp - last_ntp_timestamp);
	self->source.tsAnum = tsAnum;
	self->source.tsAden = tsAden;

	/* TODO: handle 32-bit wrap around? */
	self->source.lastSrInterval = ntp_timestamp - last_ntp_timestamp;
	self->source.srIntervalPacketCount =
		sr->sender_packet_count -
		self->source.last_sr.sender_packet_count;
	self->source.srIntervalByteCount =
		sr->sender_byte_count - self->source.last_sr.sender_byte_count;

out:
	self->source.last_sr = *sr;
	self->source.last_sr_valid = 1;
	self->source.last_sr_timestamp = self->source.rtcp_recv_ts;
}


static void
vstrm_receiver_rtcp_sdes_item_cb(uint32_t ssrc,
				 const struct rtcp_pkt_sdes_item *item,
				 void *userdata)
{
	struct vstrm_receiver *self = userdata;
	vstrm_session_metadata_read_rtcp_sdes(
		item, &self->source.session_metadata_peer);
}


static void vstrm_receiver_rtcp_bye_cb(const struct rtcp_pkt_bye *bye,
				       void *userdata)
{
	struct vstrm_receiver *self = userdata;
	char *reason = NULL;

	if (self->cbs.goodbye == NULL)
		return;
	if ((bye->source_count < 1) || (bye->sources[0] != self->source.ssrc))
		return;

	if (bye->reason_len > 0) {
		/* Copy and add terminating null char */
		reason = strndup((const char *)bye->reason, bye->reason_len);
		if (reason == NULL) {
			ULOG_ERRNO("strndup", ENOMEM);
			return;
		}
	}

	self->cbs.goodbye(self, reason, self->cbs_userdata);
	free(reason);
}


static void vstrm_receiver_rtcp_app_clock_delta_cb(struct vstrm_receiver *self,
						   struct pomp_buffer *buf)
{
	int res = 0;
	size_t pos = 0;

	/* Decode payload */
	res = vstrm_clock_delta_read(buf, &pos, &self->source.last_clock_delta);
	if (res < 0) {
		ULOG_ERRNO("vstrm_clock_delta_read", -res);
	} else {
		/* Remember last valid timestamp */
		self->source.clock_delta_recv_ts = self->source.rtcp_recv_ts;

		/* Process the clock delta */
		res = vstrm_clock_delta_process(
			&self->source.clock_delta_ctx,
			&self->source.last_clock_delta,
			self->source.clock_delta_recv_ts);
		if (res < 0)
			ULOG_ERRNO("vstrm_clock_delta_process", -res);
	}
}


static void vstrm_receiver_rtcp_app_event_cb(struct vstrm_receiver *self,
					     struct pomp_buffer *buf)
{
	int res = 0;
	size_t pos = 0;
	enum vstrm_event event;

	/* Decode payload */
	res = vstrm_event_read(buf, &pos, &event);
	if (res < 0) {
		ULOG_ERRNO("vstrm_event_read", -res);
		return;
	}

	if (self->cbs.event != NULL)
		self->cbs.event(self, event, self->cbs_userdata);
}


static void vstrm_receiver_rtcp_app_cb(const struct rtcp_pkt_app *app,
				       void *userdata)
{
	struct vstrm_receiver *self = userdata;
	struct pomp_buffer *buf = NULL;

	if (app->name == VSTRM_RTCP_APP_PACKET_NAME) {
		/* TODO: avoid copy */
		buf = pomp_buffer_new_with_data(app->data, app->data_len);
		if (buf == NULL)
			goto out;

		switch (app->subtype) {
		case VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA:
			vstrm_receiver_rtcp_app_clock_delta_cb(self, buf);
			break;
		case VSTRM_RTCP_APP_PACKET_SUBTYPE_EVENT:
			vstrm_receiver_rtcp_app_event_cb(self, buf);
			break;
		}
	}

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
}


static int
vstrm_receiver_write_rtcp_receiver_report(struct vstrm_receiver *self,
					  struct pomp_buffer *buf,
					  size_t *pos,
					  uint64_t cur_timestamp)
{
	int res = 0;
	struct rtcp_pkt_receiver_report rr;
	uint32_t ext_max_seq = 0;
	uint32_t expected = 0;
	int32_t lost = 0, lost_interval = 0;
	uint32_t expected_interval = 0;
	uint32_t received_interval = 0;
	uint32_t fraction = 0;
	uint32_t clk_rate = 0;
	uint32_t jitter_avg = 0;

	/* Extended highest sequence number */
	ext_max_seq = self->source.cycles + self->source.max_seq;

	/* Expected packet count */
	expected = ext_max_seq - self->source.base_seq + 1;

	/* Lost packet count */
	lost = expected - self->source.received;

	/* Clamp to 24 bits */
	if (lost >= 0x7fffff)
		lost = 0x7fffff;
	else if (lost <= -0x800000)
		lost = -0x800000;

	expected_interval = expected - self->source.expected_prior;
	received_interval = self->source.received - self->source.received_prior;
	lost_interval = expected_interval - received_interval;
	if (expected_interval != 0 && lost_interval > 0)
		fraction = (lost_interval << 8) / expected_interval;

	self->source.expected_prior = expected;
	self->source.received_prior = self->source.received;

	rtp_jitter_get_info(
		self->source.rtp_jitter, &clk_rate, &jitter_avg, NULL);
	jitter_avg = rtp_timestamp_from_us(jitter_avg, clk_rate);

	memset(&rr, 0, sizeof(rr));
	rr.ssrc = self->ssrc;
	rr.report_count = 1;
	rr.reports[0].ssrc = self->source.ssrc;
	rr.reports[0].fraction = fraction;
	rr.reports[0].lost = lost;
	rr.reports[0].ext_highest_seqnum = ext_max_seq;
	rr.reports[0].jitter = jitter_avg;

	if (self->source.last_sr_valid) {
		ntp_timestamp64_to_ntp_timestamp32(
			&self->source.last_sr.ntp_timestamp,
			&rr.reports[0].lsr);

		rr.reports[0].dlsr =
			((cur_timestamp - self->source.last_sr_timestamp)
			 << 16) /
			1000000;
	}

	/* Write in packet */
	res = rtcp_pkt_write_receiver_report(buf, pos, &rr);
	if (res < 0)
		ULOG_ERRNO("rtcp_pkt_write_receiver_report", -res);

	return res;
}


static int vstrm_receiver_write_rtcp_sdes(struct vstrm_receiver *self,
					  struct pomp_buffer *buf,
					  size_t *pos,
					  int full,
					  uint64_t cur_timestamp)
{
	int res = 0;
	struct rtcp_pkt_sdes sdes;
	struct rtcp_pkt_sdes_chunk chunk;
	struct rtcp_pkt_sdes_item item;

	if (full) {
		/* Write session metadata completely */
		res = vstrm_session_metadata_write_rtcp_sdes(
			buf, pos, self->ssrc, self->session_metadata_self);
	} else {
		/* Only write mandatory CNAME */
		memset(&sdes, 0, sizeof(sdes));
		memset(&chunk, 0, sizeof(chunk));
		memset(&item, 0, sizeof(item));
		sdes.chunk_count = 1;
		sdes.chunks = &chunk;
		chunk.ssrc = self->ssrc;
		chunk.item_count = 1;
		chunk.items = &item;
		item.type = RTCP_PKT_SDES_TYPE_CNAME;
		item.data = (const uint8_t *)
				    self->session_metadata_self->serial_number;
		item.data_len = strlen((const char *)item.data);
		res = rtcp_pkt_write_sdes(buf, pos, &sdes);
	}

	return res;
}


static int vstrm_receiver_write_rtcp_clock_delta(struct vstrm_receiver *self,
						 struct pomp_buffer *buf,
						 size_t *pos,
						 uint64_t cur_timestamp)
{
	int res = 0;
	struct vstrm_clock_delta cd;
	struct pomp_buffer *buf2 = NULL;
	size_t pos2 = 0;
	struct rtcp_pkt_app app;
	const void *cdata = NULL;
	size_t len = 0;

	/* Setup clock delta structure */
	memset(&cd, 0, sizeof(cd));
	cd.originate_ts = self->source.last_clock_delta.transmit_ts;
	cd.receive_ts = self->source.clock_delta_recv_ts;
	cd.transmit_ts = cur_timestamp;

	/* Serialize data in a temp buffer */
	buf2 = pomp_buffer_new(0);
	if (buf2 == NULL) {
		res = -ENOMEM;
		goto out;
	}
	res = vstrm_clock_delta_write(buf2, &pos2, &cd);
	if (res < 0) {
		ULOG_ERRNO("vstrm_clock_delta_write", -res);
		goto out;
	}
	pomp_buffer_get_cdata(buf2, &cdata, &len, NULL);

	/* Setup RTCP app structure */
	memset(&app, 0, sizeof(app));
	app.ssrc = self->ssrc;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA;
	app.data = cdata;
	app.data_len = len;

	/* Write in packet */
	res = rtcp_pkt_write_app(buf, pos, &app);
	if (res < 0) {
		ULOG_ERRNO("rtcp_pkt_write_app", -res);
		goto out;
	}

	self->source.clock_delta_ctx.expected_originate_ts = cur_timestamp;

out:
	if (buf2 != NULL)
		pomp_buffer_unref(buf2);
	return res;
}


static int vstrm_receiver_write_rtcp_video_stats(struct vstrm_receiver *self,
						 struct pomp_buffer *buf,
						 size_t *pos,
						 uint64_t cur_timestamp)
{
	int res = 0;
	const struct vstrm_video_stats *video_stats = NULL;
	const struct vstrm_video_stats_dyn *video_stats_dyn = NULL;
	struct pomp_buffer *buf2 = NULL;
	size_t pos2 = 0;
	struct rtcp_pkt_app app;
	const void *cdata = NULL;
	size_t len = 0;

	/* Get statistics from H.264 depayloader */
	res = vstrm_rtp_h264_rx_get_video_stats(
		self->rtp_h264, &video_stats, &video_stats_dyn);
	if (res < 0)
		goto out;

	/* Write to CSV file */
	if (!self->dbg.video_stats_csv_header) {
		vstrm_video_stats_csv_header(self->dbg.video_stats_csv,
					     video_stats->mb_status_class_count,
					     video_stats->mb_status_zone_count);
		self->dbg.video_stats_csv_header = 1;
	}
	vstrm_video_stats_csv_write(
		self->dbg.video_stats_csv, video_stats, video_stats_dyn);

	/* Serialize data in a temp buffer */
	buf2 = pomp_buffer_new(0);
	if (buf2 == NULL) {
		res = -ENOMEM;
		goto out;
	}
	res = vstrm_video_stats_write(
		buf2, &pos2, video_stats, video_stats_dyn);
	if (res < 0) {
		ULOG_ERRNO("vstrm_video_stats_write", -res);
		goto out;
	}
	pomp_buffer_get_cdata(buf2, &cdata, &len, NULL);

	/* Setup RTCP app structure */
	memset(&app, 0, sizeof(app));
	app.ssrc = self->ssrc;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_VIDEO_STATS;
	app.data = cdata;
	app.data_len = len;

	/* Write in packet */
	res = rtcp_pkt_write_app(buf, pos, &app);
	if (res < 0) {
		ULOG_ERRNO("rtcp_pkt_write_app", -res);
		goto out;
	}

out:
	if (buf2 != NULL)
		pomp_buffer_unref(buf2);
	return res;
}


static int vstrm_receiver_write_rtcp_goodbye(struct vstrm_receiver *self,
					     struct pomp_buffer *buf,
					     size_t *pos,
					     const char *reason)
{
	int res = 0;
	struct rtcp_pkt_bye bye;

	memset(&bye, 0, sizeof(bye));
	bye.source_count = 1;
	bye.sources[0] = self->ssrc;

	/* Reason */
	if ((reason) && (strlen(reason) > 0)) {
		bye.reason_len = strlen(reason);
		bye.reason = (const uint8_t *)reason;
	}

	/* Write in packet */
	res = rtcp_pkt_write_bye(buf, pos, &bye);
	if (res < 0)
		ULOG_ERRNO("rtcp_pkt_write_bye", -res);

	return res;
}


static int vstrm_receiver_write_rtcp(struct vstrm_receiver *self,
				     int bye,
				     const char *bye_reason)
{
	int res = 0;
	struct timespec cur_ts = {0, 0};
	uint64_t cur_timestamp = 0;
	struct pomp_buffer *buf = NULL;
	struct tpkt_packet *pkt = NULL;
	size_t pos = 0;
	int full_sdes;

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &cur_timestamp);

	buf = pomp_buffer_new(0);
	if (buf == NULL) {
		res = -ENOMEM;
		goto out;
	}

	/* Write receiver report */
	res = vstrm_receiver_write_rtcp_receiver_report(
		self, buf, &pos, cur_timestamp);
	if (res < 0)
		goto out;

	/* Write SDES */
	full_sdes = (cur_timestamp >=
		     self->source.full_sdes_send_ts + RTCP_FULL_SDES_PERIOD_US);
	res = vstrm_receiver_write_rtcp_sdes(
		self, buf, &pos, full_sdes, cur_timestamp);
	if (res < 0)
		goto out;
	if (full_sdes)
		self->source.full_sdes_send_ts = cur_timestamp;

	if (((self->cfg.flags & VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT) != 0) &&
	    (cur_timestamp >=
	     self->source.clock_delta_send_ts + RTCP_CLOCK_DELTA_PERIOD_US)) {
		res = vstrm_receiver_write_rtcp_clock_delta(
			self, buf, &pos, cur_timestamp);
		if (res < 0)
			goto out;
		self->source.clock_delta_send_ts = cur_timestamp;
	}

	if (((self->cfg.flags & VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT) != 0) &&
	    (cur_timestamp >=
	     self->source.video_stats_send_ts + RTCP_VIDEO_STATS_PERIOD_US)) {
		res = vstrm_receiver_write_rtcp_video_stats(
			self, buf, &pos, cur_timestamp);
		if (res < 0)
			goto out;
		self->source.video_stats_send_ts = cur_timestamp;
	}

	if (bye) {
		res = vstrm_receiver_write_rtcp_goodbye(
			self, buf, &pos, bye_reason);
		if (res < 0)
			goto out;
	}

	/* Send buffer (ignore EAGAIN and drop the packet) */
	res = tpkt_new_from_buffer(buf, &pkt);
	if (res < 0) {
		ULOG_ERRNO("tpkt_new_from_buffer", -res);
		goto out;
	}
	res = (*self->cbs.send_ctrl)(self, pkt, self->cbs_userdata);
	if (res < 0 && res != -EAGAIN)
		ULOG_ERRNO("cbs.send_ctrl", -res);

out:
	tpkt_unref(pkt);
	if (buf != NULL)
		pomp_buffer_unref(buf);
	return res;
}


static void vstrm_receiver_rtcp_timer_cb(struct pomp_timer *timer,
					 void *userdata)
{
	struct vstrm_receiver *self = userdata;
	vstrm_receiver_write_rtcp(self, 0, NULL);
}


static void vstrm_receiver_rtp_process_pkt_cb(struct rtp_jitter *jitter,
					      const struct rtp_pkt *pkt,
					      uint32_t gap,
					      void *userdata)
{
	struct vstrm_receiver *self = userdata;
	uint32_t clk_rate = 0;
	int64_t skew_avg = 0;
	struct vstrm_timestamp timestamp;
	uint64_t last_ntp_timestamp = 0;

	rtp_jitter_get_info(
		self->source.rtp_jitter, &clk_rate, NULL, &skew_avg);

	/* Setup timestamps */
	memset(&timestamp, 0, sizeof(timestamp));
	timestamp.input = pkt->in_timestamp;
	timestamp.rtp = pkt->rtp_timestamp;
	if (self->source.tsAnum == 0 || self->source.tsAden == 0) {
		timestamp.ntp = 0;
	} else {
		/* Linear regression to get the right NTP time from the RTP and
		 * previous sender report */
		ntp_timestamp64_to_us(&self->source.last_sr.ntp_timestamp,
				      &last_ntp_timestamp);
		timestamp.ntp = (uint64_t)(
			(((int64_t)pkt->rtp_timestamp -
			  (int64_t)self->source.last_sr.rtp_timestamp) *
				 self->source.tsAden +
			 self->source.tsAnum / 2) /
				self->source.tsAnum +
			last_ntp_timestamp);
	}
	timestamp.ntp_unskewed =
		(timestamp.ntp == 0) ? 0 : timestamp.ntp + skew_avg;
	timestamp.ntp_local =
		(self->source.clock_delta_ctx.clock_delta_valid == 0)
			? 0
			: timestamp.ntp -
				  self->source.clock_delta_ctx.clock_delta_avg;
	timestamp.ntp_raw = rtp_timestamp_to_us(timestamp.rtp, clk_rate);
	timestamp.ntp_raw_unskewed = timestamp.ntp_raw + skew_avg;

	if (self->dbg.rtp_jitter != NULL)
		vstrm_dbg_write_pomp_buf(self->dbg.rtp_jitter, pkt->raw.buf);

	vstrm_rtp_h264_rx_process_packet(self->rtp_h264, pkt, gap, &timestamp);
}


static void
vstrm_receiver_codec_info_changed_cb(struct vstrm_rtp_h264_rx *rtp_h264_rx,
				     const struct vstrm_codec_info *info,
				     void *userdata)
{
	struct vstrm_receiver *self = userdata;
	if (self->dbg.stream != NULL)
		vstrm_dbg_write_codec_info(self->dbg.stream, info);
	self->codec_info = *info;
	(*self->cbs.codec_info_changed)(self, info, self->cbs_userdata);
}


static void vstrm_receiver_recv_frame_cb(struct vstrm_rtp_h264_rx *rtp_h264_rx,
					 struct vstrm_frame *frame,
					 void *userdata)
{
	struct vstrm_receiver *self = userdata;
	const struct vstrm_codec_info *info = NULL;
	vstrm_rtp_h264_rx_get_codec_info(rtp_h264_rx, &info);
	if (self->dbg.stream != NULL)
		vstrm_dbg_write_frame(self->dbg.stream, info, frame);
	(*self->cbs.recv_frame)(self, frame, self->cbs_userdata);
}


static void vstrm_receiver_init_seq(struct vstrm_receiver *self, uint16_t seq)
{
	ULOGI("receiver: init_seq: seq=%d", seq);

	self->source.base_seq = seq;
	self->source.max_seq = seq;
	self->source.bad_seq = RTP_SEQ_MOD + 1;
	self->source.cycles = 0;
	self->source.received = 0;
	self->source.received_prior = 0;
	self->source.expected_prior = 0;

	/* Clear jitter buffer and depayloader */
	rtp_jitter_clear(self->source.rtp_jitter, seq);
	vstrm_rtp_h264_rx_clear(self->rtp_h264);
	if (self->source.ssrc == self->codec_info_ssrc) {
		vstrm_rtp_h264_rx_set_codec_info(self->rtp_h264,
						 &self->codec_info);
	}
}


static int vstrm_receiver_update_seq(struct vstrm_receiver *self, uint16_t seq)
{
	uint16_t udelta = seq - self->source.max_seq;

	/* Source is not valid until MIN_SEQUENTIAL packets with
	 * sequential sequence numbers have been received */
	if (self->source.probation > 0) {
		/* Packet is in sequence */
		if (seq == self->source.max_seq + 1) {
			self->source.probation--;
			self->source.max_seq = seq;
			if (self->source.probation == 0) {
				vstrm_receiver_init_seq(self, seq);
				self->source.received++;
				return 1;
			}
		} else {
			self->source.probation = MIN_SEQUENTIAL - 1;
			self->source.max_seq = seq;
		}
		return 0;
	} else if (udelta < MAX_DROPOUT) {
		/* In order, with permissible gap */
		if (seq < self->source.max_seq) {
			/* Sequence number wrapped - count another 64K cycle */
			self->source.cycles += RTP_SEQ_MOD;
		}
		self->source.max_seq = seq;
	} else if (udelta <= RTP_SEQ_MOD - MAX_MISORDER) {
		/* The sequence number made a very large jump */
		if (seq == self->source.bad_seq) {
			/* Two sequential packets -- assume that the other
			 * side restarted without telling us so just re-sync
			 * (i.e., pretend this was the first packet) */
			vstrm_receiver_init_seq(self, seq);
		} else {
			self->source.bad_seq = (seq + 1) & (RTP_SEQ_MOD - 1);
			return 0;
		}
	} else {
		/* Duplicate or reordered packet */
	}

	self->source.received++;
	return 1;
}


static void vstrm_receiver_create_dbg_files(struct vstrm_receiver *self)
{
	if (self->dbg.dir == NULL)
		return;

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_RECEIVER_RTP_IN) != 0) {
		self->dbg.rtp_in = vstrm_dbg_create_file(
			self->dbg.dir, self, "receiver_rtp_in.bin", "wb");
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_RECEIVER_RTP_JITTER) != 0) {
		self->dbg.rtp_jitter = vstrm_dbg_create_file(
			self->dbg.dir, self, "receiver_rtp_jitter.bin", "wb");
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_RECEIVER_STREAM) != 0) {
		self->dbg.stream = vstrm_dbg_create_file(
			self->dbg.dir, self, "receiver_stream.bin", "wb");
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_VIDEO_STATS) != 0) {
		self->dbg.video_stats_csv = vstrm_dbg_create_file(
			self->dbg.dir, self, "receiver_video_stats.dat", "w");
		self->dbg.video_stats_csv_header = 0;
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_CLOCK_DELTA) != 0) {
		self->source.clock_delta_ctx.dbg_csv = vstrm_dbg_create_file(
			self->dbg.dir, self, "receiver_clk_delta.dat", "w");
	}
}


static void vstrm_receiver_close_dbg_files(struct vstrm_receiver *self)
{
	if (self->dbg.rtp_in != NULL) {
		fclose(self->dbg.rtp_in);
		self->dbg.rtp_in = NULL;
	}
	if (self->dbg.rtp_jitter != NULL) {
		fclose(self->dbg.rtp_jitter);
		self->dbg.rtp_jitter = NULL;
	}
	if (self->dbg.stream != NULL) {
		fclose(self->dbg.stream);
		self->dbg.stream = NULL;
	}
	if (self->dbg.video_stats_csv != NULL) {
		fclose(self->dbg.video_stats_csv);
		self->dbg.video_stats_csv = NULL;
	}
	if (self->source.clock_delta_ctx.dbg_csv != NULL) {
		fclose(self->source.clock_delta_ctx.dbg_csv);
		self->source.clock_delta_ctx.dbg_csv = NULL;
	}
}


static const struct rtp_jitter_cbs jitter_cbs = {
	.process_pkt = &vstrm_receiver_rtp_process_pkt_cb,
};


static void vstrm_receiver_init_source(struct vstrm_receiver *self,
				       uint32_t ssrc,
				       uint16_t seq)
{
	int res = 0;
	struct rtp_jitter_cfg cfg;

	ULOGI("receiver: init_source: ssrc=0x%08x seq=%d", ssrc, seq);

	vstrm_receiver_close_dbg_files(self);
	if (self->source.rtcp_timer != NULL)
		pomp_timer_destroy(self->source.rtcp_timer);
	if (self->source.rtp_jitter != NULL)
		rtp_jitter_destroy(self->source.rtp_jitter);

	memset(&self->source, 0, sizeof(self->source));
	self->source.ssrc = ssrc;
	self->source.max_seq = seq - 1;
	/* TODO: make the probation configurable */
	self->source.probation = MIN_SEQUENTIAL;

	if (((self->cfg.flags & VSTRM_RECEIVER_FLAGS_ENABLE_RTCP) != 0) &&
	    (self->cfg.loop != NULL)) {
		self->source.rtcp_timer = pomp_timer_new(
			self->cfg.loop, &vstrm_receiver_rtcp_timer_cb, self);
		res = pomp_timer_set_periodic(self->source.rtcp_timer,
					      RTCP_PERIOD_MS,
					      RTCP_PERIOD_MS);
		if (res < 0)
			ULOG_ERRNO("pomp_timer_set_periodic", -res);
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.clk_rate = VSTRM_RTP_H264_CLK_RATE;
	/* TODO: make the delay configurable */
	cfg.delay = 30000;
	res = rtp_jitter_new(&cfg, &jitter_cbs, self, &self->source.rtp_jitter);
	if (res < 0)
		ULOG_ERRNO("rtp_jitter_new", -res);

	vstrm_receiver_create_dbg_files(self);
	vstrm_clock_delta_init(&self->source.clock_delta_ctx);
	vstrm_receiver_init_seq(self, seq);
}


int vstrm_receiver_new(const struct vstrm_receiver_cfg *cfg,
		       const struct vstrm_receiver_cbs *cbs,
		       void *userdata,
		       struct vstrm_receiver **ret_obj)
{
	int res = 0;
	struct vstrm_receiver *self = NULL;
	struct vstrm_rtp_h264_rx_cfg rtp_h264_cfg;
	struct vstrm_rtp_h264_rx_cbs rtp_h264_cbs;
	const char *env_dbg_dir = getenv("VSTRM_DBG_DIR");
	const char *env_dbg_flags = getenv("VSTRM_DBG_FLAGS");

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->send_ctrl == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->codec_info_changed == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->recv_frame == NULL, EINVAL);

	*ret_obj = NULL;

	/* Allocate and initialize structure */
	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->cfg = *cfg;
	self->cbs = *cbs;
	self->cbs_userdata = userdata;
	res = futils_random32(&(self->ssrc));
	if (res < 0) {
		ULOG_ERRNO("futils_random32", -res);
		goto error;
	}

	/* Override debug config with environment if any */
	if (env_dbg_dir != NULL)
		self->cfg.dbg_dir = env_dbg_dir;
	if (env_dbg_flags != NULL)
		self->cfg.dbg_flags = strtol(env_dbg_flags, NULL, 0);

	/* Copy debug directory string and update config pointer */
	if (self->cfg.dbg_dir != NULL) {
		self->dbg.dir = xstrdup(self->cfg.dbg_dir);
		self->cfg.dbg_dir = self->dbg.dir;
	}

	/* Session metadata */
	self->session_metadata_self = &self->cfg.self_meta;

	/* Create H.264 depayloader */
	memset(&rtp_h264_cfg, 0, sizeof(rtp_h264_cfg));
	rtp_h264_cfg.flags = cfg->flags;
	memset(&rtp_h264_cbs, 0, sizeof(rtp_h264_cbs));
	rtp_h264_cbs.userdata = self;
	rtp_h264_cbs.codec_info_changed = &vstrm_receiver_codec_info_changed_cb;
	rtp_h264_cbs.recv_frame = &vstrm_receiver_recv_frame_cb;
	res = vstrm_rtp_h264_rx_new(
		&rtp_h264_cfg, &rtp_h264_cbs, &self->rtp_h264);
	if (res < 0)
		goto error;

	*ret_obj = self;
	return 0;

	/* Cleanup in case of error */
error:
	vstrm_receiver_destroy(self);
	return res;
}


int vstrm_receiver_destroy(struct vstrm_receiver *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->rtp_h264 != NULL)
		vstrm_rtp_h264_rx_destroy(self->rtp_h264);
	if (self->source.rtcp_timer != NULL)
		pomp_timer_destroy(self->source.rtcp_timer);
	if (self->source.rtp_jitter != NULL)
		rtp_jitter_destroy(self->source.rtp_jitter);

	vstrm_receiver_close_dbg_files(self);
	free(self->dbg.dir);
	free(self);
	return 0;
}


int vstrm_receiver_recv_data(struct vstrm_receiver *self,
			     struct tpkt_packet *pkt)
{
	int res = 0;
	struct rtp_pkt *rtp_pkt = NULL;
	struct timespec cur_ts = {0, 0};
	uint64_t cur_timestamp = 0;
	struct pomp_buffer *buf;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	buf = tpkt_get_buffer(pkt);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &cur_timestamp);

	/* Read packet */
	res = rtp_pkt_new(&rtp_pkt);
	if (res < 0)
		goto out;
	res = rtp_pkt_read(buf, rtp_pkt);
	if (res < 0)
		goto out;

	rtp_pkt->in_timestamp = tpkt_get_timestamp(pkt);
	/* TODO: handle wrap */
	rtp_pkt->rtp_timestamp = rtp_pkt->header.timestamp;

	/* Initialize source (or reset it) */
	if (self->source.ssrc != rtp_pkt->header.ssrc) {
		vstrm_receiver_init_source(
			self, rtp_pkt->header.ssrc, rtp_pkt->header.seqnum);
	} else {
		res = vstrm_receiver_update_seq(self, rtp_pkt->header.seqnum);
		if (res == 1)
			self->source.received_bytes += rtp_pkt->raw.len;
	}

	if (self->dbg.rtp_in != NULL)
		vstrm_dbg_write_pomp_buf(self->dbg.rtp_in, buf);

	/* Notify raw packet received if needed */
	/* TODO: do it after setting up timestamp from jitter buffer, but before
	 * queuing it (and transferring ownership to jitter buffer) */
	if (self->cbs.recv_rtp_pkt != NULL)
		(*self->cbs.recv_rtp_pkt)(self, rtp_pkt, self->cbs_userdata);

	/* Add in jitter buffer */
	res = rtp_jitter_enqueue(self->source.rtp_jitter, rtp_pkt);
	if (res < 0)
		goto out;

	/* Jitter buffer has now ownership of packet */
	rtp_pkt = NULL;

	/* Try to process packets */
	if (!self->source.probation) {
		res = rtp_jitter_process(self->source.rtp_jitter,
					 cur_timestamp);
		if (res < 0)
			goto out;
	}

out:
	if (rtp_pkt != NULL)
		rtp_pkt_destroy(rtp_pkt);
	return res;
}


static const struct rtcp_pkt_read_cbs rtcp_cbs = {
	.sender_report = &vstrm_receiver_rtcp_sender_report_cb,
	.sdes_item = &vstrm_receiver_rtcp_sdes_item_cb,
	.bye = &vstrm_receiver_rtcp_bye_cb,
	.app = &vstrm_receiver_rtcp_app_cb,
};


int vstrm_receiver_recv_ctrl(struct vstrm_receiver *self,
			     struct tpkt_packet *pkt)
{
	int res = 0;
	struct vmeta_session old_session_metadata_peer;
	struct pomp_buffer *buf;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	buf = tpkt_get_buffer(pkt);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	/* TODO: check SSRC? what if RTCP is received before any RTP? */

	self->source.rtcp_recv_ts = tpkt_get_timestamp(pkt);

	/* Remember session metadata before parsing new one found in this
	 * RTCP packet */
	old_session_metadata_peer = self->source.session_metadata_peer;

	/* Read RTCP packet */
	res = rtcp_pkt_read(buf, &rtcp_cbs, self);
	if (res < 0)
		goto out;

	/* Check if peer session metadata has changed */
	if (self->cbs.session_metadata_peer_changed != NULL &&
	    memcmp(&old_session_metadata_peer,
		   &self->source.session_metadata_peer,
		   sizeof(old_session_metadata_peer)) != 0) {
		(*self->cbs.session_metadata_peer_changed)(
			self,
			&self->source.session_metadata_peer,
			self->cbs_userdata);
	}

out:
	return res;
}


int vstrm_receiver_send_goodbye(struct vstrm_receiver *self, const char *reason)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return vstrm_receiver_write_rtcp(self, 1, reason);
}


int vstrm_receiver_set_codec_info(struct vstrm_receiver *self,
				  const struct vstrm_codec_info *info,
				  uint32_t ssrc)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info == NULL, EINVAL);

	self->codec_info = *info;
	self->codec_info_ssrc = ssrc;
	if (self->source.ssrc == self->codec_info_ssrc) {
		return vstrm_rtp_h264_rx_set_codec_info(self->rtp_h264,
							&self->codec_info);
	} else {
		return 0;
	}
}


uint64_t vstrm_receiver_get_ntp_from_rtp_ts(struct vstrm_receiver *self,
					    uint32_t rtpts)
{
	uint32_t clk_rate = 0;
	int64_t skew_avg = 0;

	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(self->source.rtp_jitter == NULL, EINVAL, 0);

	rtp_jitter_get_info(
		self->source.rtp_jitter, &clk_rate, NULL, &skew_avg);

	/* TODO: this should be a real unskewed NTP timestamp */
	return rtp_timestamp_to_us(rtpts, clk_rate);
}


int vstrm_receiver_get_ssrc_self(struct vstrm_receiver *self, uint32_t *ssrc)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ssrc == NULL, EINVAL);

	*ssrc = self->ssrc;
	return 0;
}


int vstrm_receiver_get_ssrc_peer(struct vstrm_receiver *self, uint32_t *ssrc)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ssrc == NULL, EINVAL);

	*ssrc = self->source.ssrc;
	return 0;
}


int vstrm_receiver_get_stats(struct vstrm_receiver *self,
			     struct vstrm_receiver_stats *stats)
{
	uint32_t ext_max_seq = 0;
	uint32_t expected = 0;
	int32_t lost = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(stats == NULL, EINVAL);

	/* Compute the total lost packet count (same as RTCP
	 * receiver report contents) */
	ext_max_seq = self->source.cycles + self->source.max_seq;
	expected = ext_max_seq - self->source.base_seq + 1;
	lost = expected - self->source.received;

	stats->received_packet_count = self->source.received;
	stats->received_byte_count = self->source.received_bytes;
	stats->lost_packet_count = lost;

	return 0;
}


int vstrm_receiver_set_session_metadata_self(struct vstrm_receiver *self,
					     const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	*self->session_metadata_self = *meta;
	return 0;
}


int vstrm_receiver_get_session_metadata_self(struct vstrm_receiver *self,
					     const struct vmeta_session **meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	*meta = self->session_metadata_self;
	return 0;
}


int vstrm_receiver_get_session_metadata_peer(struct vstrm_receiver *self,
					     const struct vmeta_session **meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	*meta = &self->source.session_metadata_peer;
	return 0;
}
