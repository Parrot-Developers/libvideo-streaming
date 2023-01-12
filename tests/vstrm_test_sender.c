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

#include "vstrm_test.h"

#define ULOG_TAG vstrm_test_sender
#include <ulog.h>
ULOG_DECLARE_TAG(vstrm_test_sender);

#define DEFAULT_TARGET_PACKET_SIZE 1500
#define MAX_BITRATE 5000000
#define MAX_LATENCY_MS 200
#define MAX_NETWORK_LATENCY_CLASS_0 250
#define MAX_NETWORK_LATENCY_CLASS_1 200
#define MAX_NETWORK_LATENCY_CLASS_2 150
#define MAX_NETWORK_LATENCY_CLASS_3 100


static void unmap_file(struct vstrm_test_sender *self)
{
#ifdef _WIN32
	if (self->data != NULL)
		UnmapViewOfFile(self->data);
	self->data = NULL;
	if (self->map != INVALID_HANDLE_VALUE)
		CloseHandle(self->map);
	self->map = INVALID_HANDLE_VALUE;
	if (self->infile != INVALID_HANDLE_VALUE)
		CloseHandle(self->infile);
	self->infile = INVALID_HANDLE_VALUE;
#else
	if (self->fd >= 0) {
		if (self->data != NULL)
			munmap(self->data, self->data_len);
		self->data = NULL;
		close(self->fd);
		self->fd = -1;
	}
#endif
}


static int map_file(struct vstrm_test_sender *self)
{
	int res;

#ifdef _WIN32
	BOOL ret;
	LARGE_INTEGER filesize;

	self->infile = CreateFileA(self->input_file,
				   GENERIC_READ,
				   0,
				   NULL,
				   OPEN_EXISTING,
				   FILE_ATTRIBUTE_NORMAL,
				   NULL);
	if (self->infile == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileA('%s')", -res, self->input_file);
		goto error;
	}

	self->map = CreateFileMapping(
		self->infile, NULL, PAGE_READONLY, 0, 0, NULL);
	if (self->map == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileMapping('%s')", -res, self->input_file);
		goto error;
	}

	ret = GetFileSizeEx(self->infile, &filesize);
	if (ret == FALSE) {
		res = -EIO;
		ULOG_ERRNO("GetFileSizeEx('%s')", -res, self->input_file);
		goto error;
	}
	self->data_len = filesize.QuadPart;

	self->data = MapViewOfFile(self->map, FILE_MAP_READ, 0, 0, 0);
	if (self->data == NULL) {
		res = -EIO;
		ULOG_ERRNO("MapViewOfFile('%s')", -res, self->input_file);
		goto error;
	}
#else
	/* Try to open input file */
	self->fd = open(self->input_file, O_RDONLY);
	if (self->fd < 0) {
		res = -errno;
		ULOG_ERRNO("open('%s')", -res, self->input_file);
		goto error;
	}

	/* Get size and map it */
	self->data_len = lseek(self->fd, 0, SEEK_END);
	if (self->data_len == (size_t)-1) {
		res = -errno;
		ULOG_ERRNO("lseek", -res);
		goto error;
	}

	self->data =
		mmap(NULL, self->data_len, PROT_READ, MAP_PRIVATE, self->fd, 0);
	if (self->data == MAP_FAILED) {
		res = -errno;
		ULOG_ERRNO("mmap", -res);
		goto error;
	}
#endif

	return 0;

error:
	unmap_file(self);
	return res;
}


static void frame_dispose(struct vstrm_frame *frame)
{
	return;
}


static void socket_data_cb(int fd, uint32_t events, void *userdata)
{
	int res = 0;
	struct vstrm_test_sender *self = userdata;
	ssize_t readlen = 0;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if ((events & POMP_FD_EVENT_OUT) != 0) {
		/* Notify sender */
		res = vstrm_sender_notify_send_data_ready(self->sender);
		if (res < 0)
			ULOG_ERRNO("vstrm_sender_notify_send_data_ready", -res);
	}

	if ((events & POMP_FD_EVENT_IN) != 0) {
		do {
			/* Read data (and trash them...) */
			readlen = tskt_socket_read(self->data_sock,
						   self->rx_buf,
						   self->rx_buf_len,
						   NULL);
		} while (readlen > 0);
	}
}


static void socket_ctrl_cb(int fd, uint32_t events, void *userdata)
{
	int res;
	struct vstrm_test_sender *self = userdata;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct tpkt_packet *pkt = NULL;
	uint64_t ts = 0;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	do {
		/* Read data */
		readlen = tskt_socket_read(
			self->ctrl_sock, self->rx_buf, self->rx_buf_len, &ts);

		/* Something read ? */
		if (readlen > 0) {
			/* TODO: Avoid copy */
			buf = pomp_buffer_new_with_data(self->rx_buf, readlen);
			res = tpkt_new_from_buffer(buf, &pkt);
			pomp_buffer_unref(buf);
			buf = NULL;
			if (res < 0) {
				ULOG_ERRNO("tpkt_new_from_buffer", -res);
				tpkt_unref(pkt);
				break;
			}
			res = tpkt_set_timestamp(pkt, ts);
			if (res < 0) {
				ULOG_ERRNO("tpkt_set_timestamp", -res);
				tpkt_unref(pkt);
				break;
			}
			res = vstrm_sender_recv_ctrl(self->sender, pkt);
			tpkt_unref(pkt);
			pkt = NULL;
			if (res < 0)
				ULOG_ERRNO("vstrm_sender_recv_ctrl", -res);
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


static int send_data_cb(struct vstrm_sender *stream,
			struct tpkt_packet *pkt,
			bool marker,
			void *userdata)
{
	struct vstrm_test_sender *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	/* Write data */
	res = tskt_socket_write_pkt(self->data_sock, pkt);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_write_pkt", -res);

	return res;
}


static int send_ctrl_cb(struct vstrm_sender *stream,
			struct tpkt_packet *pkt,
			void *userdata)
{
	struct vstrm_test_sender *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	res = tskt_socket_write_pkt(self->ctrl_sock, pkt);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_write_pkt", -res);

	return res;
}


static int monitor_send_data_ready_cb(struct vstrm_sender *stream,
				      int enable,
				      void *userdata)
{
	struct vstrm_test_sender *self = userdata;
	uint32_t events = POMP_FD_EVENT_IN | (enable ? POMP_FD_EVENT_OUT : 0);

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	int fd = tskt_socket_get_fd(self->data_sock);
	if (fd < 0) {
		ULOG_ERRNO("tskt_socket_get_fd", -fd);
		return fd;
	}

	return pomp_loop_update(self->loop, fd, events);
}


static void session_metadata_peer_changed_cb(struct vstrm_sender *stream,
					     const struct vmeta_session *meta,
					     void *userdata)
{
	ULOGI("%s", __func__);
}


static void receiver_report_cb(struct vstrm_sender *stream,
			       const struct rtcp_pkt_receiver_report *rr,
			       uint32_t rtd,
			       void *userdata)
{
	ULOGI("%s", __func__);
}


static void video_stats_cb(struct vstrm_sender *stream,
			   const struct vstrm_video_stats *video_stats,
			   const struct vstrm_video_stats_dyn *video_stats_dyn,
			   void *userdata)
{
	ULOGI("%s", __func__);
}


static void
goodbye_cb(struct vstrm_sender *stream, const char *reason, void *userdata)
{
	ULOGI("%s", __func__);
}


/**
 * Computes the data socket tx size based on bitrate and maximum latency
 * This way the kernel will not bufferize too much and allow us to detect
 * congestion and do some preventive drops.
 * @max_bitrate: max expected bitrate in bits/s
 * @max_latency_ms: max desired latency in ms.
 */
static uint32_t get_socket_data_tx_size(uint32_t max_bitrate,
					uint32_t max_latency_ms)
{
	uint32_t total_size = max_bitrate * max_latency_ms / 1000 / 8;
	uint32_t min_size = max_bitrate * 50 / 1000 / 8;
	uint32_t size = total_size / 4;
	return size > min_size ? size : min_size;
}


static void au_end_cb(struct h264_ctx *ctx, void *userdata)
{
	int res;
	struct vstrm_test_sender *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);

	if (self->frame != NULL) {
		res = vstrm_sender_send_frame(self->sender, self->frame);
		if (res < 0)
			ULOG_ERRNO("vstrm_sender_send_frame", -res);
		vstrm_frame_unref(self->frame);
		self->frame = NULL;
		res = h264_reader_stop(self->reader);
		if (res < 0)
			ULOG_ERRNO("h264_reader_stop", -res);
	}
}


static void nalu_end_cb(struct h264_ctx *ctx,
			enum h264_nalu_type type,
			const uint8_t *buf,
			size_t len,
			const struct h264_nalu_header *nh,
			void *userdata)
{
	int res;
	struct vstrm_test_sender *self = userdata;
	struct vstrm_frame_nalu nalu;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);

	/* Save the SPS and PPS */
	if ((type == H264_NALU_TYPE_SPS) && (self->sps == NULL)) {
		self->sps_len = len;
		self->sps = malloc(self->sps_len);
		if (self->sps == NULL) {
			ULOG_ERRNO("malloc", ENOMEM);
			return;
		}
		memcpy(self->sps, buf, len);
		ULOGI("SPS found");
	} else if ((type == H264_NALU_TYPE_PPS) && (self->pps == NULL)) {
		self->pps_len = len;
		self->pps = malloc(self->pps_len);
		if (self->pps == NULL) {
			ULOG_ERRNO("malloc", ENOMEM);
			return;
		}
		memcpy(self->pps, buf, len);
		ULOGI("PPS found");
	}

	/* Get a new frame if needed */
	if (self->frame == NULL) {
		struct vstrm_frame_ops ops;
		ops.dispose = &frame_dispose;
		res = vstrm_frame_new(&ops, 0, &self->frame);
		if (res < 0) {
			ULOG_ERRNO("vstrm_frame_new", -res);
			return;
		}
		self->timestamp += self->frame_interval_us;
		self->frame->timestamps.ntp = self->timestamp;
	}

	/* Add the NALU to the frame */
	memset(&nalu, 0, sizeof(nalu));
	nalu.cdata = buf;
	nalu.len = len;
	res = vstrm_frame_add_nalu(self->frame, &nalu);
	if (res < 0) {
		ULOG_ERRNO("vstrm_frame_add_nalu", -res);
		return;
	}
}


static int finish(struct vstrm_test_sender *self)
{
	int res = 0;
	self->finished = 1;
	if (self->timer != NULL) {
		res = pomp_timer_clear(self->timer);
		if (res < 0)
			ULOG_ERRNO("pomp_timer_clear", -res);
	}
	if (self->finished_cb)
		self->finished_cb(self->userdata);
	return res;
}


static int au_parse(struct vstrm_test_sender *self)
{
	int res = 0;
	size_t off = 0;

	res = h264_reader_parse(self->reader,
				0,
				(uint8_t *)self->data + self->data_off,
				self->data_len - self->data_off,
				&off);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse", -res);
		return -res;
	}

	self->data_off += off;
	if (self->data_off >= self->data_len)
		finish(self);

	return res;
}


static void au_parse_timer(struct pomp_timer *timer, void *userdata)
{
	struct vstrm_test_sender *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	res = au_parse(self);
	if (res < 0)
		ULOG_ERRNO("au_parse", -res);
}


static void au_parse_idle(void *userdata)
{
	struct vstrm_test_sender *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	res = au_parse(self);
	if (res < 0) {
		ULOG_ERRNO("au_parse", -res);
		return;
	}

	if (!self->finished) {
		if (self->framerate != 0.) {
			res = pomp_timer_set_periodic(
				self->timer,
				(self->frame_interval_us + 500) / 1000,
				(self->frame_interval_us + 500) / 1000);
			if (res < 0)
				ULOG_ERRNO("pomp_timer_set_periodic", -res);
		} else {
			res = pomp_loop_idle_add(
				self->loop, &au_parse_idle, self);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_idle_add", -res);
		}
	}
}


static void *vstrm_test_sender_thread(void *ptr)
{
	struct vstrm_test_sender *self = ptr;

	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	while (!self->thread_should_stop)
		pomp_loop_wait_and_process(self->loop, -1);

	return NULL;
}


static const struct h264_ctx_cbs h264_cbs = {
	.au_end = &au_end_cb,
	.nalu_end = &nalu_end_cb,
};


static struct vstrm_sender_cbs vstrm_cbs = {
	.send_data = &send_data_cb,
	.send_ctrl = &send_ctrl_cb,
	.monitor_send_data_ready = &monitor_send_data_ready_cb,
	.session_metadata_peer_changed = &session_metadata_peer_changed_cb,
	.receiver_report = &receiver_report_cb,
	.video_stats = &video_stats_cb,
	.goodbye = &goodbye_cb,
};


int vstrm_test_sender_create(const char *file,
			     float framerate,
			     const char *local_addr,
			     uint16_t local_data_port,
			     uint16_t local_ctrl_port,
			     const char *remote_addr,
			     uint16_t remote_data_port,
			     uint16_t remote_ctrl_port,
			     void (*finished_cb)(void *userdata),
			     void *userdata,
			     struct vstrm_test_sender **ret_obj)
{
	int res = 0;
	struct vstrm_test_sender *self = NULL;
	struct vstrm_sender_cfg vstrm_cfg;
	uint32_t tx_size;

	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_data_port == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_ctrl_port == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	/* Context allocation */
	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}
#ifdef _WIN32
	self->infile = INVALID_HANDLE_VALUE;
	self->map = INVALID_HANDLE_VALUE;
#else
	self->fd = -1;
#endif
	self->finished_cb = finished_cb;
	self->userdata = userdata;
	self->input_file = file;

	/* Loop */
	self->loop = pomp_loop_new();
	if (self->loop == NULL) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		res = -ENOMEM;
		goto out;
	}

	/* Map the input file */
	res = map_file(self);
	if (res < 0)
		goto out;

	self->framerate = framerate;
	self->frame_interval_us =
		(framerate != 0.) ? 1000000. / framerate : 33333;

	/* Create the H.264 reader */
	res = h264_reader_new(&h264_cbs, self, &self->reader);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_new", -res);
		goto out;
	}

	/* Create the rx buffer */
	self->rx_buf_len = DEFAULT_RX_BUFFER_SIZE;
	self->rx_buf = malloc(self->rx_buf_len);
	if (self->rx_buf == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("malloc:rx_buf", -res);
		goto out;
	}

	/* Setup the data socket */
	res = tskt_socket_new(local_addr,
			      &local_data_port,
			      remote_addr,
			      remote_data_port,
			      NULL,
			      self->loop,
			      &socket_data_cb,
			      self,
			      &self->data_sock);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_new", -res);
		goto out;
	}
	ULOGI("RTP: address %s->%s port %u->%u",
	      local_addr,
	      remote_addr,
	      local_data_port,
	      remote_data_port);

	tx_size = get_socket_data_tx_size(MAX_BITRATE, MAX_LATENCY_MS);
	res = tskt_socket_set_rxbuf_size(self->data_sock, DEFAULT_RCVBUF_SIZE);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_rxbuf_size", -res);
		goto out;
	}
	res = tskt_socket_set_txbuf_size(self->data_sock, tx_size);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_txbuf_size", -res);
		goto out;
	}
	res = tskt_socket_set_class_selector(self->data_sock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_class_selector", -res);
		goto out;
	}

	/* Setup control socket */
	res = tskt_socket_new(local_addr,
			      &local_ctrl_port,
			      remote_addr,
			      remote_ctrl_port,
			      NULL,
			      self->loop,
			      &socket_ctrl_cb,
			      self,
			      &self->ctrl_sock);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_new", -res);
		goto out;
	}
	ULOGI("RTCP: address %s<->%s port %u<->%u",
	      local_addr,
	      remote_addr,
	      local_ctrl_port,
	      remote_ctrl_port);

	res = tskt_socket_set_rxbuf_size(self->ctrl_sock, DEFAULT_RCVBUF_SIZE);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_rxbuf_size", -res);
		goto out;
	}
	res = tskt_socket_set_txbuf_size(self->ctrl_sock, DEFAULT_SNDBUF_SIZE);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_txbuf_size", -res);
		goto out;
	}
	res = tskt_socket_set_class_selector(self->ctrl_sock,
					     IPTOS_PREC_FLASHOVERRIDE);
	if (res < 0) {
		ULOG_ERRNO("tskt_socket_set_class_selector", -res);
		goto out;
	}

	/* Sender configuration */
	memset(&vstrm_cfg, 0, sizeof(vstrm_cfg));
	vstrm_cfg.loop = self->loop;
	vstrm_cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT |
			  VSTRM_SENDER_FLAGS_ENABLE_RTCP |
			  VSTRM_SENDER_FLAGS_ENABLE_RTCP_EXT;
	vstrm_cfg.dyn.target_packet_size = DEFAULT_TARGET_PACKET_SIZE;
	vstrm_cfg.dyn.max_network_latency_ms[0] = MAX_NETWORK_LATENCY_CLASS_0;
	vstrm_cfg.dyn.max_network_latency_ms[1] = MAX_NETWORK_LATENCY_CLASS_1;
	vstrm_cfg.dyn.max_network_latency_ms[2] = MAX_NETWORK_LATENCY_CLASS_2;
	vstrm_cfg.dyn.max_network_latency_ms[3] = MAX_NETWORK_LATENCY_CLASS_3;

	/* Create sender */
	res = vstrm_sender_new(&vstrm_cfg, &vstrm_cbs, self, &self->sender);
	if (res < 0) {
		ULOG_ERRNO("vstrm_sender_new", -res);
		goto out;
	}

	/* Create timer */
	if (self->framerate != 0.) {
		self->timer = pomp_timer_new(self->loop, &au_parse_timer, self);
		if (self->timer == NULL) {
			ULOG_ERRNO("pomp_timer_new", ENOMEM);
			res = -ENOMEM;
			goto out;
		}
	}

	/* Create the sender thread */
	res = pthread_create(
		&self->thread, NULL, vstrm_test_sender_thread, (void *)self);
	if (res != 0) {
		ULOG_ERRNO("pthread_create", res);
		res = -res;
		goto out;
	} else {
		self->thread_launched = 1;
	}

	/* Start */
	res = pomp_loop_idle_add(self->loop, au_parse_idle, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		goto out;
	}

	ULOGI("started");

out:
	if (res < 0) {
		vstrm_test_sender_destroy(self);
		self = NULL;
	}

	*ret_obj = self;
	return res;
}


int vstrm_test_sender_join(struct vstrm_test_sender *self,
			   struct vstrm_sender_stats *stats)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (!self->thread_launched)
		return 0;

	self->thread_should_stop = 1;
	if (self->loop != NULL) {
		res = pomp_loop_wakeup(self->loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -res);
	}
	res = pthread_join(self->thread, NULL);
	if (res != 0)
		ULOG_ERRNO("pthread_join", res);
	self->thread_launched = 0;

	if (stats != NULL) {
		res = vstrm_sender_get_stats(self->sender, stats);
		if (res < 0)
			ULOG_ERRNO("vstrm_sender_get_stats", -res);
	}

	ULOGI("joined");
	return 0;
}


int vstrm_test_sender_destroy(struct vstrm_test_sender *self)
{
	int res;

	if (self == NULL)
		return 0;

	res = vstrm_test_sender_join(self, NULL);
	if (res < 0)
		ULOG_ERRNO("vstrm_test_sender_join", -res);
	if (self->sender != NULL) {
		res = vstrm_sender_destroy(self->sender);
		if (res < 0)
			ULOG_ERRNO("vstrm_sender_destroy", -res);
	}
	res = tskt_socket_destroy(self->data_sock);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_destroy", -res);
	res = tskt_socket_destroy(self->ctrl_sock);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_destroy", -res);
	if (self->reader != NULL) {
		res = h264_reader_destroy(self->reader);
		if (res < 0)
			ULOG_ERRNO("h264_reader_destroy", -res);
	}
	unmap_file(self);
	if (self->timer != NULL) {
		res = pomp_timer_destroy(self->timer);
		if (res < 0)
			ULOG_ERRNO("pomp_timer_destroy", -res);
	}
	if (self->loop != NULL) {
		res = pomp_loop_destroy(self->loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_destroy", -res);
	}
	if (self->frame != NULL)
		vstrm_frame_unref(self->frame);
	free(self->sps);
	free(self->pps);
	free(self->rx_buf);
	free(self);

	ULOGI("destroyed");
	return 0;
}
