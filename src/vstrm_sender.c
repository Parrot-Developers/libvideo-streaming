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


struct vstrm_sender {
	struct vstrm_sender_cfg cfg;
	struct vstrm_sender_cbs cbs;
	void *cbs_userdata;
	uint32_t ssrc;
	uint32_t peer_ssrc;
	struct vmeta_session *session_metadata_self;
	struct vmeta_session session_metadata_peer;

	struct vstrm_rtp_h264_tx *rtp_h264;
	struct pomp_timer *rtcp_timer;
	uint64_t rtcp_recv_ts;
	uint64_t full_sdes_send_ts;
	uint64_t clock_delta_send_ts;

	struct list_node packets;
	uint16_t next_seqnum;
	int monitoring_send_data_ready;
	int send_data_ready;

	struct vstrm_sender_stats stats;

	uint32_t last_rtp_timestamp;
	uint64_t last_ntp_timestamp;

	struct vstrm_clock_delta last_clock_delta;
	uint64_t clock_delta_recv_ts;
	struct vstrm_clock_delta_ctx clock_delta_ctx;

	struct vstrm_video_stats video_stats;
	struct vstrm_video_stats_dyn video_stats_dyn;

	bool invalid_rtd;
	uint64_t invalid_rtd_count;

	struct {
		char *dir;
		FILE *stream;
		FILE *rtp_payload;
		FILE *rtp_out;
		FILE *video_stats_csv;
		int video_stats_csv_header;
	} dbg;
};


static void
vstrm_sender_rtcp_receiver_report_cb(const struct rtcp_pkt_receiver_report *rr,
				     void *userdata)
{
	if (!rr || !userdata)
		return;
	struct vstrm_sender *self = userdata;
	self->peer_ssrc = rr->ssrc;

	uint32_t rtd_us = UINT32_MAX;
	if (rr->report_count > 0) {
		/* Compute the round-trip delay (see RFC3550 chap. 6.4.1) */
		struct ntp_timestamp32 a_32 = {0};
		ntp_timestamp32_from_us(&a_32, self->rtcp_recv_ts);
		struct ntp_timestamp64 a = {0};
		ntp_timestamp32_to_ntp_timestamp64(&a_32, &a);
		struct ntp_timestamp64 lsr = {0};
		ntp_timestamp32_to_ntp_timestamp64(&rr->reports[0].lsr, &lsr);
		int64_t diff = 0;
		ntp_timestamp64_diff_us(&a, &lsr, &diff);
		while (diff < 0) {
			a.seconds += (1 << 16);
			ntp_timestamp64_diff_us(&a, &lsr, &diff);
		}
		int64_t dlsr = ((uint64_t)rr->reports[0].dlsr * 1000000) >> 16;
		if (dlsr > diff) {
			if (!self->invalid_rtd) {
				self->invalid_rtd = true;
				ULOGE("invalid DLSR vs. time diff for RTD");
			}
			self->invalid_rtd_count++;
		} else {
			rtd_us = diff - dlsr;
			if (self->invalid_rtd) {
				self->invalid_rtd = false;
				ULOGE("RTD is now valid (%" PRIu64
				      " invalid RTD(s))",
				      self->invalid_rtd_count);
				self->invalid_rtd_count = 0;
			}
		}
	}

	if (self->cbs.receiver_report != NULL) {
		/* Notify callback */
		(*self->cbs.receiver_report)(
			self, rr, rtd_us, self->cbs_userdata);
	}
}


static void
vstrm_sender_rtcp_sdes_item_cb(uint32_t ssrc,
			       const struct rtcp_pkt_sdes_item *item,
			       void *userdata)
{
	struct vstrm_sender *self = userdata;
	self->peer_ssrc = ssrc;
	vstrm_session_metadata_read_rtcp_sdes(item,
					      &self->session_metadata_peer);
}


static void vstrm_sender_rtcp_bye_cb(const struct rtcp_pkt_bye *bye,
				     void *userdata)
{
	struct vstrm_sender *self = userdata;
	char *reason = NULL;

	if (self->cbs.goodbye == NULL)
		return;
	if ((bye->source_count < 1) || (bye->sources[0] != self->peer_ssrc))
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


static void vstrm_sender_rtcp_app_clock_delta_cb(struct vstrm_sender *self,
						 struct pomp_buffer *buf)
{
	int res = 0;
	size_t pos = 0;

	/* Decode payload */
	res = vstrm_clock_delta_read(buf, &pos, &self->last_clock_delta);
	if (res < 0) {
		ULOG_ERRNO("vstrm_clock_delta_read", -res);
	} else {
		/* Remember last valid timestamp */
		self->clock_delta_recv_ts = self->rtcp_recv_ts;

		/* Process the clock delta */
		res = vstrm_clock_delta_process(&self->clock_delta_ctx,
						&self->last_clock_delta,
						self->clock_delta_recv_ts);
		if (res < 0)
			ULOG_ERRNO("vstrm_clock_delta_process", -res);
	}
}


static void vstrm_sender_rtcp_app_video_stats_cb(struct vstrm_sender *self,
						 struct pomp_buffer *buf)
{
	int res = 0;
	size_t pos = 0;

	/* Decode payload */
	res = vstrm_video_stats_read(
		buf, &pos, &self->video_stats, &self->video_stats_dyn);
	if (res < 0) {
		ULOG_ERRNO("vstrm_video_stats_read", -res);
	} else {
		if (self->cbs.video_stats != NULL) {
			/* Notify callback */
			(*self->cbs.video_stats)(self,
						 &self->video_stats,
						 &self->video_stats_dyn,
						 self->cbs_userdata);
		}

		/* Write to CSV file */
		if (!self->dbg.video_stats_csv_header) {
			vstrm_video_stats_csv_header(
				self->dbg.video_stats_csv,
				self->video_stats.mb_status_class_count,
				self->video_stats.mb_status_zone_count);
			self->dbg.video_stats_csv_header = 1;
		}
		vstrm_video_stats_csv_write(self->dbg.video_stats_csv,
					    &self->video_stats,
					    &self->video_stats_dyn);
	}
}


static void vstrm_sender_rtcp_app_cb(const struct rtcp_pkt_app *app,
				     void *userdata)
{
	struct vstrm_sender *self = userdata;
	struct pomp_buffer *buf = NULL;

	if (app->name == VSTRM_RTCP_APP_PACKET_NAME) {
		/* TODO: avoid copy */
		buf = pomp_buffer_new_with_data(app->data, app->data_len);
		if (buf == NULL)
			goto out;

		switch (app->subtype) {
		case VSTRM_RTCP_APP_PACKET_SUBTYPE_CLOCK_DELTA:
			vstrm_sender_rtcp_app_clock_delta_cb(self, buf);
			break;
		case VSTRM_RTCP_APP_PACKET_SUBTYPE_VIDEO_STATS:
			vstrm_sender_rtcp_app_video_stats_cb(self, buf);
			break;
		}
	}

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
}


static int vstrm_sender_write_rtcp_sender_report(struct vstrm_sender *self,
						 struct pomp_buffer *buf,
						 size_t *pos,
						 uint64_t cur_timestamp)
{
	int res = 0;
	struct rtcp_pkt_sender_report sr;
	int64_t diff = 0;

	memset(&sr, 0, sizeof(sr));
	sr.ssrc = self->ssrc;

	/* Current NTP timestamp */
	ntp_timestamp64_from_us(&sr.ntp_timestamp, cur_timestamp);

	/* Get the RTP timestamp corresponding to current NTP timestamp */
	diff = cur_timestamp - self->last_ntp_timestamp;
	if (diff > 0)
		diff = rtp_timestamp_from_us(diff, VSTRM_RTP_H264_CLK_RATE);
	else
		diff = -rtp_timestamp_from_us(-diff, VSTRM_RTP_H264_CLK_RATE);
	sr.rtp_timestamp = self->last_rtp_timestamp + diff;

	/* Count of packets/bytes sent */
	sr.sender_packet_count = self->stats.total_packet_count;
	sr.sender_byte_count = self->stats.total_payload_byte_count;

	/* Write in packet */
	res = rtcp_pkt_write_sender_report(buf, pos, &sr);
	if (res < 0)
		ULOG_ERRNO("rtcp_pkt_write_sender_report", -res);

	return res;
}


static int vstrm_sender_write_rtcp_sdes(struct vstrm_sender *self,
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


static int vstrm_sender_write_rtcp_clock_delta(struct vstrm_sender *self,
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
	cd.originate_ts = self->last_clock_delta.transmit_ts;
	cd.receive_ts = self->clock_delta_recv_ts;
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

	self->clock_delta_ctx.expected_originate_ts = cur_timestamp;

out:
	if (buf2 != NULL)
		pomp_buffer_unref(buf2);
	return res;
}


static int vstrm_sender_write_rtcp_event(struct vstrm_sender *self,
					 struct pomp_buffer *buf,
					 size_t *pos,
					 enum vstrm_event event)
{
	int res = 0;
	struct pomp_buffer *buf2 = NULL;
	size_t pos2 = 0;
	struct rtcp_pkt_app app;
	const void *cdata = NULL;
	size_t len = 0;

	/* Serialize data in a temp buffer */
	buf2 = pomp_buffer_new(0);
	if (buf2 == NULL) {
		res = -ENOMEM;
		goto out;
	}
	res = vstrm_event_write(buf2, &pos2, event);
	if (res < 0) {
		ULOG_ERRNO("vstrm_event_write", -res);
		goto out;
	}
	pomp_buffer_get_cdata(buf2, &cdata, &len, NULL);

	/* Setup RTCP app structure */
	memset(&app, 0, sizeof(app));
	app.ssrc = self->ssrc;
	app.name = VSTRM_RTCP_APP_PACKET_NAME;
	app.subtype = VSTRM_RTCP_APP_PACKET_SUBTYPE_EVENT;
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


static int vstrm_sender_write_rtcp_goodbye(struct vstrm_sender *self,
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


static int vstrm_sender_write_rtcp(struct vstrm_sender *self,
				   int bye,
				   const char *bye_reason,
				   int send_event,
				   enum vstrm_event event)
{
	int res = 0;
	struct timespec cur_ts = {0, 0};
	uint64_t cur_timestamp = 0;
	struct pomp_buffer *buf = NULL;
	struct tpkt_packet *pkt = NULL;
	size_t pos = 0, len = 0;
	int full_sdes;

	/* Nothing to do if we did not send at least one RTP packet */
	if (self->last_rtp_timestamp == 0 || self->last_ntp_timestamp == 0)
		goto out;

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &cur_timestamp);

	buf = pomp_buffer_new(0);
	if (buf == NULL) {
		res = -ENOMEM;
		goto out;
	}

	/* Write sender report */
	res = vstrm_sender_write_rtcp_sender_report(
		self, buf, &pos, cur_timestamp);
	if (res < 0)
		goto out;

	/* Write SDES */
	full_sdes = (self->full_sdes_send_ts == 0) ||
		    (cur_timestamp >=
		     self->full_sdes_send_ts + RTCP_FULL_SDES_PERIOD_US);
	res = vstrm_sender_write_rtcp_sdes(
		self, buf, &pos, full_sdes, cur_timestamp);
	if (res < 0)
		goto out;
	if (full_sdes)
		self->full_sdes_send_ts = cur_timestamp;

	/* TODO: make the clock delta period configurable */
	if ((self->cfg.flags & VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT) != 0 &&
	    (cur_timestamp >=
	     self->clock_delta_send_ts + RTCP_CLOCK_DELTA_PERIOD_US)) {
		res = vstrm_sender_write_rtcp_clock_delta(
			self, buf, &pos, cur_timestamp);
		if (res < 0)
			goto out;
		self->clock_delta_send_ts = cur_timestamp;
	}

	if (((self->cfg.flags & VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT) != 0) &&
	    (send_event)) {
		res = vstrm_sender_write_rtcp_event(self, buf, &pos, event);
		if (res < 0)
			goto out;
	}

	if (bye) {
		res = vstrm_sender_write_rtcp_goodbye(
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
	else if (res == 0) {
		tpkt_get_cdata(pkt, NULL, &len, NULL);
		self->stats.total_control_sent_packet_count++;
		self->stats.total_control_sent_byte_count += len;
	}

out:
	tpkt_unref(pkt);
	if (buf != NULL)
		pomp_buffer_unref(buf);
	return res;
}


static void vstrm_sender_rtcp_timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct vstrm_sender *self = userdata;
	vstrm_sender_write_rtcp(self, 0, NULL, 0, VSTRM_EVENT_NONE);
}


static void vstrm_sender_monitor_send_data_ready(struct vstrm_sender *self,
						 int enable)
{
	int res = 0;
	res = (*self->cbs.monitor_send_data_ready)(
		self, enable, self->cbs_userdata);
	if (res < 0) {
		ULOG_ERRNO("cbs.monitor_send_data_ready", -res);
	} else {
		self->send_data_ready = !enable;
		self->monitoring_send_data_ready = enable;
	}
}


static void vstrm_sender_process_queue(struct vstrm_sender *self)
{
	int res = 0;
	struct rtp_pkt *rtp_pkt = NULL;
	struct rtp_pkt *tmp_pkt = NULL;
	struct tpkt_packet *pkt = NULL;
	struct timespec ts = {0, 0};
	uint64_t cur_timestamp = 0;
	uint64_t delta = 0;

	while (!list_is_empty(&self->packets)) {
		/* Get next packet */
		rtp_pkt = list_entry(
			list_first(&self->packets), struct rtp_pkt, node);
		if (rtp_pkt == NULL)
			break;
		res = tpkt_new_from_buffer(rtp_pkt->raw.buf, &pkt);
		if (res < 0) {
			ULOG_ERRNO("tpkt_new_from_buffer", -res);
			break;
		}

		/* Send packet, consider it handled if success or error
		 * that is not try again later (lower queue full) */
		res = (*self->cbs.send_data)(self, pkt, self->cbs_userdata);
		tpkt_unref(pkt);
		pkt = NULL;
		if (res == 0 || res != -EAGAIN) {
			if (res != 0) {
				ULOG_ERRNO("cbs.send_data", -res);
			} else if (self->dbg.rtp_out != NULL) {
				vstrm_dbg_write_pomp_buf(self->dbg.rtp_out,
							 rtp_pkt->raw.buf);
			}
			self->stats.total_packet_count++;
			self->stats.total_byte_count +=
				RTP_PKT_HEADER_SIZE + rtp_pkt->extheader.len +
				rtp_pkt->payload.len + rtp_pkt->padding.len;
			self->stats.total_header_byte_count +=
				RTP_PKT_HEADER_SIZE;
			self->stats.total_headerext_byte_count +=
				rtp_pkt->extheader.len;
			self->stats.total_payload_byte_count +=
				rtp_pkt->payload.len;
			self->stats.total_padding_byte_count +=
				rtp_pkt->padding.len;
			list_del(&rtp_pkt->node);
			rtp_pkt_destroy(rtp_pkt);
		} else {
			/* Queue full */
			break;
		}
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &cur_timestamp);

	/* Remove packets on timeout, but still account for them
	 * in sender reports */
	list_walk_entry_forward_safe(&self->packets, rtp_pkt, tmp_pkt, node)
	{
		if (cur_timestamp > rtp_pkt->out_timestamp &&
		    rtp_pkt->out_timestamp != 0) {
			delta = cur_timestamp - rtp_pkt->out_timestamp;
			ULOGD("drop packet: seqnum=%u size=%zu "
			      "importance=%" PRIu32 " (%ums late)",
			      rtp_pkt->header.seqnum,
			      rtp_pkt->raw.len,
			      rtp_pkt->importance,
			      (unsigned int)(delta / 1000));
			self->stats.total_packet_count++;
			self->stats.total_byte_count +=
				RTP_PKT_HEADER_SIZE + rtp_pkt->extheader.len +
				rtp_pkt->payload.len + rtp_pkt->padding.len;
			self->stats.total_header_byte_count +=
				RTP_PKT_HEADER_SIZE;
			self->stats.total_headerext_byte_count +=
				rtp_pkt->extheader.len;
			self->stats.total_payload_byte_count +=
				rtp_pkt->payload.len;
			self->stats.total_padding_byte_count +=
				rtp_pkt->padding.len;
			self->stats.dropped_packet_count++;
			self->stats.dropped_byte_count +=
				RTP_PKT_HEADER_SIZE + rtp_pkt->extheader.len +
				rtp_pkt->payload.len + rtp_pkt->padding.len;
			list_del(&rtp_pkt->node);
			rtp_pkt_destroy(rtp_pkt);
		}
	}

	/* Update monitoring */
	if (list_is_empty(&self->packets)) {
		if (self->monitoring_send_data_ready)
			vstrm_sender_monitor_send_data_ready(self, 0);
	} else {
		if (!self->monitoring_send_data_ready)
			vstrm_sender_monitor_send_data_ready(self, 1);
	}
}


static void vstrm_sender_create_dbg_files(struct vstrm_sender *self)
{
	if (self->dbg.dir == NULL)
		return;

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_SENDER_STREAM) != 0) {
		self->dbg.stream = vstrm_dbg_create_file(
			self->dbg.dir, self, "sender_stream.bin", "wb");
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_SENDER_RTP_PAYLOAD) != 0) {
		self->dbg.rtp_payload = vstrm_dbg_create_file(
			self->dbg.dir, self, "sender_rtp_payload.bin", "wb");
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_SENDER_RTP_OUT) != 0) {
		self->dbg.rtp_out = vstrm_dbg_create_file(
			self->dbg.dir, self, "sender_rtp_out.bin", "wb");
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_VIDEO_STATS) != 0) {
		self->dbg.video_stats_csv = vstrm_dbg_create_file(
			self->dbg.dir, self, "sender_video_stats.dat", "w");
		self->dbg.video_stats_csv_header = 0;
	}

	if ((self->cfg.dbg_flags & VSTRM_DBG_FLAG_CLOCK_DELTA) != 0) {
		self->clock_delta_ctx.dbg_csv = vstrm_dbg_create_file(
			self->dbg.dir, self, "sender_clk_delta.dat", "w");
	}
}


static void vstrm_sender_close_dbg_files(struct vstrm_sender *self)
{
	if (self->dbg.stream != NULL) {
		fclose(self->dbg.stream);
		self->dbg.stream = NULL;
	}
	if (self->dbg.rtp_payload != NULL) {
		fclose(self->dbg.rtp_payload);
		self->dbg.rtp_payload = NULL;
	}
	if (self->dbg.rtp_out != NULL) {
		fclose(self->dbg.rtp_out);
		self->dbg.rtp_out = NULL;
	}
	if (self->dbg.video_stats_csv != NULL) {
		fclose(self->dbg.video_stats_csv);
		self->dbg.video_stats_csv = NULL;
	}
	if (self->clock_delta_ctx.dbg_csv) {
		fclose(self->clock_delta_ctx.dbg_csv);
		self->clock_delta_ctx.dbg_csv = NULL;
	}
}


int vstrm_sender_new(const struct vstrm_sender_cfg *cfg,
		     const struct vstrm_sender_cbs *cbs,
		     void *userdata,
		     struct vstrm_sender **ret_obj)
{
	int res = 0;
	struct vstrm_sender *self = NULL;
	struct vstrm_rtp_h264_tx_cfg rtp_h264_cfg;
	const char *env_dbg_dir = getenv("VSTRM_DBG_DIR");
	const char *env_dbg_flags = getenv("VSTRM_DBG_FLAGS");

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->send_data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->send_ctrl == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->monitor_send_data_ready == NULL, EINVAL);

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
	list_init(&self->packets);
	self->send_data_ready = 1;

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
	vstrm_sender_create_dbg_files(self);

	/* Session metadata */
	self->session_metadata_self = &self->cfg.self_meta;

	/* Create H.264 payloader (unless it is a raw sender) */
	if ((self->cfg.flags & VSTRM_SENDER_FLAGS_RAW) == 0) {
		memset(&rtp_h264_cfg, 0, sizeof(rtp_h264_cfg));
		rtp_h264_cfg.flags = cfg->flags;
		rtp_h264_cfg.dyn.target_packet_size =
			cfg->dyn.target_packet_size;
		res = vstrm_rtp_h264_tx_new(&rtp_h264_cfg, &self->rtp_h264);
		if (res < 0)
			goto error;
	}

	/* Create RTCP timer */
	if ((self->cfg.flags & VSTRM_SENDER_FLAGS_ENABLE_RTCP) != 0) {
		self->rtcp_timer = pomp_timer_new(
			self->cfg.loop, &vstrm_sender_rtcp_timer_cb, self);
		res = pomp_timer_set_periodic(
			self->rtcp_timer, RTCP_PERIOD_MS, RTCP_PERIOD_MS);
		if (res < 0)
			ULOG_ERRNO("pomp_timer_set_periodic", -res);
	}

	vstrm_clock_delta_init(&self->clock_delta_ctx);

	*ret_obj = self;
	return 0;

	/* Cleanup in case of error */
error:
	vstrm_sender_destroy(self);
	return res;
}


int vstrm_sender_destroy(struct vstrm_sender *self)
{
	struct rtp_pkt *pkt = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	while (!list_is_empty(&self->packets)) {
		pkt = list_entry(
			list_first(&self->packets), struct rtp_pkt, node);
		list_del(&pkt->node);
		rtp_pkt_destroy(pkt);
	}

	if (self->rtp_h264 != NULL)
		vstrm_rtp_h264_tx_destroy(self->rtp_h264);
	if (self->rtcp_timer != NULL)
		pomp_timer_destroy(self->rtcp_timer);
	vstrm_video_stats_dyn_clear(&self->video_stats_dyn);

	vstrm_sender_close_dbg_files(self);
	free(self->dbg.dir);
	free(self);
	return 0;
}


int vstrm_sender_send_frame(struct vstrm_sender *self,
			    struct vstrm_frame *frame)
{
	int res = 0;
	struct list_node packets;
	struct rtp_pkt *pkt = NULL;
	struct timespec ts = {0, 0};
	uint64_t cur_timestamp = 0;
	uint32_t rtp_timestamp = 0;
	uint64_t ntp_timestamp = 0;
	uint64_t out_timestamp1 = 0, out_timestamp2 = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	/* Must not be a raw sender */
	ULOG_ERRNO_RETURN_ERR_IF(
		(self->cfg.flags & VSTRM_SENDER_FLAGS_RAW) != 0, EPERM);

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &cur_timestamp);
	rtp_timestamp = rtp_timestamp_from_us(frame->timestamps.ntp,
					      VSTRM_RTP_H264_CLK_RATE);
	ntp_timestamp = frame->timestamps.ntp;
	self->last_rtp_timestamp = rtp_timestamp;
	self->last_ntp_timestamp = ntp_timestamp;

	/* TODO: give a real codec info */
	if (self->dbg.stream != NULL)
		vstrm_dbg_write_frame(self->dbg.stream, NULL, frame);

	/* Create packets */
	list_init(&packets);
	res = vstrm_rtp_h264_tx_process_frame(self->rtp_h264, frame, &packets);
	if (res < 0)
		goto out;

	/* Queue packets */
	while (!list_is_empty(&packets)) {
		pkt = list_entry(list_first(&packets), struct rtp_pkt, node);
		list_del(&pkt->node);

		pkt->header.seqnum = self->next_seqnum;
		pkt->header.ssrc = self->ssrc;
		pkt->header.timestamp = rtp_timestamp;
		/* TODO: handle wrap */
		pkt->rtp_timestamp = rtp_timestamp;
		pkt->in_timestamp = cur_timestamp;
		/* The output timestamp is the earliest between
		 * the frame TS + the max total latency
		 * (if the max total latency is not null) and
		 * the input TS + the max network latency
		 * (if the max total latency is not null). */
		if (pkt->importance < VSTRM_FRAME_MAX_NALU_IMPORTANCE_LEVELS) {
			if (self->cfg.dyn
				    .max_total_latency_ms[pkt->importance] != 0)
				out_timestamp1 =
					frame->timestamps.ntp +
					self->cfg.dyn.max_total_latency_ms
							[pkt->importance] *
						1000;
			if (self->cfg.dyn
				    .max_network_latency_ms[pkt->importance] !=
			    0)
				out_timestamp2 =
					pkt->in_timestamp +
					self->cfg.dyn.max_network_latency_ms
							[pkt->importance] *
						1000;
		}
		pkt->out_timestamp = out_timestamp1;
		if ((out_timestamp1 == 0) || (out_timestamp2 < out_timestamp1))
			pkt->out_timestamp = out_timestamp2;

		/* Finalize header (should not fail, buffer already contains
		 * reserved room for it) */
		res = rtp_pkt_finalize_header(pkt);
		if (res < 0)
			ULOG_ERRNO("rtp_pkt_finalize_header", -res);
		list_add_after(list_last(&self->packets), &pkt->node);
		self->next_seqnum = (self->next_seqnum + 1) & 0xffff;

		if (self->dbg.rtp_payload != NULL) {
			vstrm_dbg_write_pomp_buf(self->dbg.rtp_payload,
						 pkt->raw.buf);
		}
	}

	/* Process now if possible */
	if (self->send_data_ready)
		vstrm_sender_process_queue(self);

out:
	return res;
}


int vstrm_sender_send_rtp_pkt(struct vstrm_sender *self, struct rtp_pkt *pkt)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!list_node_is_unref(&pkt->node), EINVAL);

	/* Must be a raw sender */
	ULOG_ERRNO_RETURN_ERR_IF(
		(self->cfg.flags & VSTRM_SENDER_FLAGS_RAW) == 0, EPERM);

	/* Add in list, process now if possible */
	/* TODO: update timeout */
	list_add_after(list_last(&self->packets), &pkt->node);
	if (self->send_data_ready)
		vstrm_sender_process_queue(self);

	return 0;
}


int vstrm_sender_send_event(struct vstrm_sender *self, enum vstrm_event event)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return vstrm_sender_write_rtcp(self, 0, NULL, 1, event);
}


int vstrm_sender_send_goodbye(struct vstrm_sender *self, const char *reason)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return vstrm_sender_write_rtcp(self, 1, reason, 0, VSTRM_EVENT_NONE);
}


static const struct rtcp_pkt_read_cbs rtcp_cbs = {
	.receiver_report = &vstrm_sender_rtcp_receiver_report_cb,
	.sdes_item = &vstrm_sender_rtcp_sdes_item_cb,
	.bye = &vstrm_sender_rtcp_bye_cb,
	.app = &vstrm_sender_rtcp_app_cb,
};


int vstrm_sender_recv_ctrl(struct vstrm_sender *self, struct tpkt_packet *pkt)
{
	int res = 0;
	struct vmeta_session old_session_metadata_peer;
	struct pomp_buffer *buf;
	size_t len = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	buf = tpkt_get_buffer(pkt);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	self->rtcp_recv_ts = tpkt_get_timestamp(pkt);
	tpkt_get_cdata(pkt, NULL, &len, NULL);
	self->stats.total_control_received_packet_count++;
	self->stats.total_control_received_byte_count += len;

	/* Remember session metadata before parsing new one found in this
	 * RTCP packet */
	old_session_metadata_peer = self->session_metadata_peer;

	/* Read RTCP packet */
	res = rtcp_pkt_read(buf, &rtcp_cbs, self);
	if (res < 0)
		goto out;

	/* Check if peer session metadata has changed */
	if (self->cbs.session_metadata_peer_changed != NULL &&
	    memcmp(&old_session_metadata_peer,
		   &self->session_metadata_peer,
		   sizeof(old_session_metadata_peer)) != 0) {
		(*self->cbs.session_metadata_peer_changed)(
			self, &self->session_metadata_peer, self->cbs_userdata);
	}

out:
	return res;
}


int vstrm_sender_notify_send_data_ready(struct vstrm_sender *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->send_data_ready = 1;
	vstrm_sender_process_queue(self);
	return 0;
}


int vstrm_sender_get_cfg_dyn(struct vstrm_sender *self,
			     struct vstrm_sender_cfg_dyn *cfg_dyn)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg_dyn == NULL, EINVAL);

	*cfg_dyn = self->cfg.dyn;
	return 0;
}


int vstrm_sender_set_cfg_dyn(struct vstrm_sender *self,
			     const struct vstrm_sender_cfg_dyn *cfg_dyn)
{
	int res = 0;
	struct vstrm_rtp_h264_tx_cfg_dyn rtp_h264_cfg_dyn;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg_dyn == NULL, EINVAL);

	self->cfg.dyn = *cfg_dyn;
	if (self->rtp_h264 != NULL) {
		rtp_h264_cfg_dyn.target_packet_size =
			cfg_dyn->target_packet_size;
		rtp_h264_cfg_dyn.packet_size_align = cfg_dyn->packet_size_align;
		res = vstrm_rtp_h264_tx_set_cfg_dyn(self->rtp_h264,
						    &rtp_h264_cfg_dyn);
	}
	return res;
}


int vstrm_sender_get_next_frame_params(struct vstrm_sender *self,
				       uint64_t timestamp,
				       uint16_t *seq,
				       uint32_t *rtpts)
{
	uint64_t rtp_timestamp;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(seq == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(rtpts == NULL, EINVAL);

	/* Must not be a raw sender */
	ULOG_ERRNO_RETURN_ERR_IF(
		(self->cfg.flags & VSTRM_SENDER_FLAGS_RAW) != 0, EPERM);

	rtp_timestamp =
		rtp_timestamp_from_us(timestamp, VSTRM_RTP_H264_CLK_RATE);

	*seq = self->next_seqnum;
	*rtpts = rtp_timestamp;

	return 0;
}


int vstrm_sender_get_ssrc_self(struct vstrm_sender *self, uint32_t *ssrc)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ssrc == NULL, EINVAL);

	*ssrc = self->ssrc;
	return 0;
}


int vstrm_sender_get_ssrc_peer(struct vstrm_sender *self, uint32_t *ssrc)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ssrc == NULL, EINVAL);

	*ssrc = self->peer_ssrc;
	return 0;
}


int vstrm_sender_get_stats(struct vstrm_sender *self,
			   struct vstrm_sender_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(stats == NULL, EINVAL);

	*stats = self->stats;
	return 0;
}


int vstrm_sender_set_session_metadata_self(struct vstrm_sender *self,
					   const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	if (memcmp(meta, self->session_metadata_self, sizeof(*meta)) == 0)
		return 0;

	*self->session_metadata_self = *meta;
	self->full_sdes_send_ts = 0;
	return 0;
}


int vstrm_sender_get_session_metadata_self(struct vstrm_sender *self,
					   const struct vmeta_session **meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	*meta = self->session_metadata_self;
	return 0;
}


int vstrm_sender_get_session_metadata_peer(struct vstrm_sender *self,
					   const struct vmeta_session **meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	*meta = &self->session_metadata_peer;
	return 0;
}
