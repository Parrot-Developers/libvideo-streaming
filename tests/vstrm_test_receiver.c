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

#define ULOG_TAG vstrm_test_receiver
#include <ulog.h>
ULOG_DECLARE_TAG(vstrm_test_receiver);


static void socket_data_cb(int fd, uint32_t events, void *userdata)
{
	int res;
	struct vstrm_test_receiver *self = userdata;
	ssize_t readlen = 0;
	struct pomp_buffer *buf = NULL;
	struct tpkt_packet *pkt = NULL;
	uint64_t ts = 0;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	do {
		/* Read data */
		readlen = tskt_socket_read(
			self->data_sock, self->rx_buf, self->rx_buf_len, &ts);

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
			res = vstrm_receiver_recv_data(self->receiver, pkt);
			tpkt_unref(pkt);
			pkt = NULL;
			if (res < 0)
				ULOG_ERRNO("vstrm_receiver_recv_data", -res);
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


static void socket_ctrl_cb(int fd, uint32_t events, void *userdata)
{
	int res;
	struct vstrm_test_receiver *self = userdata;
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
			res = vstrm_receiver_recv_ctrl(self->receiver, pkt);
			tpkt_unref(pkt);
			pkt = NULL;
			if (res < 0)
				ULOG_ERRNO("vstrm_receiver_recv_ctrl", -res);
		} else if (readlen == 0) {
			/* TODO: EOF */
		}
	} while (readlen > 0);
}


static int send_ctrl_cb(struct vstrm_receiver *stream,
			struct tpkt_packet *pkt,
			void *userdata)
{
	struct vstrm_test_receiver *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	res = tskt_socket_write_pkt(self->ctrl_sock, pkt);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_write_pkt", -res);

	return res;
}


static void codec_info_changed_cb(struct vstrm_receiver *stream,
				  const struct vstrm_codec_info *info,
				  void *userdata)
{
	ULOGI("%s", __func__);
}


static void recv_frame_cb(struct vstrm_receiver *stream,
			  struct vstrm_frame *frame,
			  void *userdata)
{
	struct vstrm_test_receiver *self = userdata;
	int res = 0;
	uint8_t *data = NULL;
	size_t size = 0, ret_size;

	ULOGD("%s", __func__);

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(frame == NULL, EINVAL);

	if (self->file != NULL) {
		res = vstrm_frame_get_size(
			frame,
			&size,
			VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE);
		if (res < 0) {
			ULOG_ERRNO("vstrm_frame_get_size", -res);
			return;
		}
		data = malloc(size);
		if (data == NULL) {
			ULOG_ERRNO("malloc", ENOMEM);
			return;
		}
		res = vstrm_frame_copy(
			frame,
			data,
			size,
			VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE);
		if (res < 0) {
			ULOG_ERRNO("vstrm_frame_copy", -res);
			free(data);
			return;
		}
		ret_size = fwrite(data, size, 1, self->file);
		if (ret_size < 1)
			ULOG_ERRNO("fwrite", EIO);
		free(data);
	}
}


static void session_metadata_peer_changed_cb(struct vstrm_receiver *stream,
					     const struct vmeta_session *meta,
					     void *userdata)
{
	ULOGI("%s", __func__);
}


static void
goodbye_cb(struct vstrm_receiver *stream, const char *reason, void *userdata)
{
	ULOGI("%s", __func__);
}


static void *vstrm_test_receiver_thread(void *ptr)
{
	struct vstrm_test_receiver *self = ptr;

	while (!self->thread_should_stop)
		pomp_loop_wait_and_process(self->loop, -1);

	return NULL;
}


static struct vstrm_receiver_cbs vstrm_cbs = {
	.send_ctrl = &send_ctrl_cb,
	.codec_info_changed = &codec_info_changed_cb,
	.recv_frame = &recv_frame_cb,
	.session_metadata_peer_changed = &session_metadata_peer_changed_cb,
	.goodbye = &goodbye_cb,
};


int vstrm_test_receiver_create(const char *local_addr,
			       uint16_t *local_data_port,
			       uint16_t *local_ctrl_port,
			       const char *remote_addr,
			       uint16_t remote_data_port,
			       uint16_t remote_ctrl_port,
			       const char *file,
			       void (*finished_cb)(void *userdata),
			       void *userdata,
			       struct vstrm_test_receiver **ret_obj)
{
	int res = 0;
	struct vstrm_test_receiver *self = NULL;
	struct vstrm_receiver_cfg vstrm_cfg;

	ULOG_ERRNO_RETURN_ERR_IF(local_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_data_port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_ctrl_port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	/* Context allocation */
	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}
	self->finished_cb = finished_cb;
	self->userdata = userdata;

	/* Output file */
	if (file != NULL) {
		self->file = fopen(file, "wb");
		if (self->file == NULL) {
			ULOG_ERRNO("fopen", errno);
			res = -errno;
			goto out;
		}
	}

	/* Loop */
	self->loop = pomp_loop_new();
	if (self->loop == NULL) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		res = -ENOMEM;
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
			      local_data_port,
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
	      *local_data_port,
	      remote_data_port);

	res = tskt_socket_set_txbuf_size(self->data_sock, DEFAULT_SNDBUF_SIZE);
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
			      local_ctrl_port,
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
	      *local_ctrl_port,
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

	/* Receiver configuration */
	memset(&vstrm_cfg, 0, sizeof(vstrm_cfg));
	vstrm_cfg.loop = self->loop;
	vstrm_cfg.flags = VSTRM_RECEIVER_FLAGS_H264_GEN_GREY_IDR_FRAME |
			  VSTRM_RECEIVER_FLAGS_H264_GEN_CONCEALMENT_SLICE |
			  VSTRM_RECEIVER_FLAGS_ENABLE_RTCP |
			  VSTRM_RECEIVER_FLAGS_ENABLE_RTCP_EXT;

	/* Create receiver */
	res = vstrm_receiver_new(&vstrm_cfg, &vstrm_cbs, self, &self->receiver);
	if (res < 0) {
		ULOG_ERRNO("vstrm_receiver_new", -res);
		goto out;
	}

	/* Create the receiver thread */
	res = pthread_create(
		&self->thread, NULL, vstrm_test_receiver_thread, (void *)self);
	if (res != 0) {
		ULOG_ERRNO("pthread_create", res);
		res = -res;
		goto out;
	} else {
		self->thread_launched = 1;
	}

	ULOGI("started");

out:
	if (res < 0) {
		vstrm_test_receiver_destroy(self);
		self = NULL;
	}

	*ret_obj = self;
	return res;
}


int vstrm_test_receiver_join(struct vstrm_test_receiver *self,
			     struct vstrm_receiver_stats *stats)
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
		res = vstrm_receiver_get_stats(self->receiver, stats);
		if (res < 0)
			ULOG_ERRNO("vstrm_receiver_get_stats", -res);
	}

	ULOGI("joined");
	return 0;
}


int vstrm_test_receiver_destroy(struct vstrm_test_receiver *self)
{
	int res;

	if (self == NULL)
		return 0;

	res = vstrm_test_receiver_join(self, NULL);
	if (res < 0)
		ULOG_ERRNO("vstrm_test_receiver_join", -res);
	if (self->receiver != NULL) {
		res = vstrm_receiver_destroy(self->receiver);
		if (res < 0)
			ULOG_ERRNO("vstrm_receiver_destroy", -res);
	}
	res = tskt_socket_destroy(self->data_sock);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_destroy", -res);
	res = tskt_socket_destroy(self->ctrl_sock);
	if (res < 0)
		ULOG_ERRNO("tskt_socket_destroy", -res);
	if (self->loop != NULL) {
		res = pomp_loop_destroy(self->loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_destroy", -res);
	}
	if (self->file != NULL) {
		res = fclose(self->file);
		if (res != 0)
			ULOG_ERRNO("fclose", -errno);
	}
	free(self->rx_buf);
	free(self);

	ULOGI("destroyed");
	return 0;
}
