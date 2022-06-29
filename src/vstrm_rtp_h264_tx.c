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

/**
 * RFC 6184: RTP Payload Format for H.264 Video
 */

#include "vstrm_priv.h"

/* clang-format off */
#define CHECK(_x) do { if ((res = (_x)) < 0) goto out; } while (0)
/* clang-format on */


/* keep MAX_LEN % 4 == 0, this avoids padding */
#define VSTRM_METADATA_FRAGMENT_MAX_LEN 300


struct vstrm_rtp_h264_tx {
	struct vstrm_rtp_h264_tx_cfg cfg;
	struct vstrm_frame *frame;
	struct list_node *packets;

	struct rtp_pkt *pkt;
	size_t pos;

	/* metadata informations */
	struct {
		uint8_t nb_packs;
		uint8_t current_pack;
	} metadata;
};


static int vstrm_rtp_h264_tx_add_legacy_metadata(struct vstrm_rtp_h264_tx *self)
{
	self->pkt->extheader.off = self->pos;
	struct vmeta_buffer buf;
	void *data = NULL;
	size_t capacity = 0;
	int res;

	/* Only add legacy metadata in first packet */
	if (!list_is_empty(self->packets))
		return 0;

	res = pomp_buffer_ensure_capacity(
		self->pkt->raw.buf, RTP_PKT_HEADER_SIZE + VMETA_FRAME_MAX_SIZE);
	if (res < 0) {
		ULOG_ERRNO("pomp_buffer_ensure_capacity", -res);
		goto out;
	}
	res = pomp_buffer_get_data(self->pkt->raw.buf, &data, NULL, &capacity);
	if (res < 0) {
		ULOG_ERRNO("pomp_buffer_get_data", -res);
		goto out;
	}
	vmeta_buffer_set_data(&buf, data, capacity, self->pos);
	res = vmeta_frame_write(&buf, self->frame->metadata);
	if (res < 0) {
		ULOG_ERRNO("vmeta_frame_write", -res);
		goto out;
	}
	self->pos = buf.pos;
	self->pkt->extheader.len = self->pos - self->pkt->extheader.off;

	RTP_PKT_HEADER_FLAGS_SET(self->pkt->header.flags, EXTENSION, 1);

	/* TODO: check overflow */
out:
	return res;
}


static int vstrm_rtp_h264_tx_add_proto_metadata(struct vstrm_rtp_h264_tx *self)
{
	self->pkt->extheader.off = self->pos;
	void *data = NULL;
	uint8_t *udata;
	size_t capacity = 0;
	int res;
	uint16_t id, len, offset;
	uint8_t padding;
	size_t packlen;
	off_t packoff;
	uint16_t last_cur_pad;
	const uint8_t *meta_content;
	size_t meta_len;
	res = vmeta_frame_proto_get_buffer(
		self->frame->metadata, &meta_content, &meta_len);
	if (res < 0) {
		ULOG_ERRNO("vmeta_frame_proto_get_buffer", -res);
		goto out_norelease;
	}

	if (list_is_empty(self->packets)) {
		/* Split into VSTRM_METADATA_FRAGMENT_MAX_LEN byte packets */
		self->metadata.nb_packs =
			(meta_len + VSTRM_METADATA_FRAGMENT_MAX_LEN - 1) /
			VSTRM_METADATA_FRAGMENT_MAX_LEN;
		if (self->metadata.nb_packs > 128) {
			res = -ENOBUFS;
			goto out;
		}
		self->metadata.current_pack = 0;
	}

	/* Stop here if all metadata fragments were sent */
	if (self->metadata.current_pack >= self->metadata.nb_packs) {
		res = 0;
		goto out;
	}

	packoff = self->metadata.current_pack * VSTRM_METADATA_FRAGMENT_MAX_LEN;
	if (self->metadata.current_pack == (self->metadata.nb_packs - 1)) {
		/* Last fragment */
		packlen = meta_len - packoff;
	} else {
		/* Intermediate fragment, take MAX_LEN of data */
		packlen = VSTRM_METADATA_FRAGMENT_MAX_LEN;
	}

	res = pomp_buffer_ensure_capacity(self->pkt->raw.buf,
					  RTP_PKT_HEADER_SIZE + packlen + 8);
	if (res < 0) {
		ULOG_ERRNO("pomp_buffer_ensure_capacity", -res);
		goto out;
	}
	res = pomp_buffer_get_data(self->pkt->raw.buf, &data, NULL, &capacity);
	if (res < 0) {
		ULOG_ERRNO("pomp_buffer_get_data", -res);
		goto out;
	}
	udata = data;
	/* Compute id + size + padding size */
	id = htons(VMETA_FRAME_PROTO_RTP_EXT_ID);
	len = htons(((packlen + 3) / 4) + 1);
	offset = htons(packoff);
	padding = 4 - (packlen % 4);
	if (padding == 4)
		padding = 0;
	last_cur_pad =
		vstrm_rtp_h264_meta_header_pack(self->metadata.nb_packs - 1,
						self->metadata.current_pack,
						padding);
	memcpy(&udata[self->pos], &id, sizeof(id));
	self->pos += sizeof(id);
	memcpy(&udata[self->pos], &len, sizeof(len));
	self->pos += sizeof(len);
	memcpy(&udata[self->pos], &last_cur_pad, sizeof(last_cur_pad));
	self->pos += sizeof(last_cur_pad);
	memcpy(&udata[self->pos], &offset, sizeof(offset));
	self->pos += sizeof(offset);
	memcpy(&udata[self->pos], meta_content + packoff, packlen);
	self->pos += packlen;
	if (padding > 0) {
		memset(&udata[self->pos], 0, padding);
		self->pos += padding;
	}
	self->pkt->extheader.len = self->pos - self->pkt->extheader.off;

	RTP_PKT_HEADER_FLAGS_SET(self->pkt->header.flags, EXTENSION, 1);

	self->metadata.current_pack++;
out:
	vmeta_frame_proto_release_buffer(self->frame->metadata, meta_content);
out_norelease:
	return res;
}


static int vstrm_rtp_h264_tx_begin_pkt(struct vstrm_rtp_h264_tx *self,
				       uint32_t priority,
				       uint32_t importance)
{
	int res = 0;

	/* Create new packet */
	res = rtp_pkt_new(&self->pkt);
	if (res < 0)
		goto error;

	self->pkt->raw.buf = pomp_buffer_new(RTP_PKT_HEADER_SIZE);
	if (self->pkt->raw.buf == NULL) {
		res = -ENOMEM;
		goto error;
	}

	RTP_PKT_HEADER_FLAGS_SET(
		self->pkt->header.flags, VERSION, RTP_PKT_VERSION);

	RTP_PKT_HEADER_FLAGS_SET(self->pkt->header.flags,
				 PAYLOAD_TYPE,
				 VSTRM_RTP_H264_PAYLOAD_TYPE);

	self->pkt->priority = priority;
	self->pkt->importance = importance;

	/* Skip RTP header */
	self->pos = RTP_PKT_HEADER_SIZE;

	/* If we have metadata, add in extended header */
	if ((self->cfg.flags & VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT) != 0 &&
	    self->frame->metadata != NULL &&
	    self->frame->metadata->type != VMETA_FRAME_TYPE_NONE) {

		if (self->frame->metadata->type != VMETA_FRAME_TYPE_PROTO)
			res = vstrm_rtp_h264_tx_add_legacy_metadata(self);
		else
			res = vstrm_rtp_h264_tx_add_proto_metadata(self);
		if (res < 0)
			goto error;
	}

	self->pkt->payload.off = self->pos;

	return 0;

	/* Cleanup in case of error */
error:
	if (self->pkt != NULL)
		rtp_pkt_destroy(self->pkt);
	self->pkt = NULL;
	return res;
}


static void vstrm_rtp_h264_tx_end_pkt(struct vstrm_rtp_h264_tx *self)
{
	/* Setup final payload length */
	self->pkt->payload.len = self->pos - self->pkt->payload.off;

	/* Setup data/len of buffer */
	pomp_buffer_get_cdata(self->pkt->raw.buf,
			      (const void **)&self->pkt->raw.cdata,
			      &self->pkt->raw.len,
			      NULL);

	/* Add in list (transfer ownership) */
	list_add_after(list_last(self->packets), &self->pkt->node);
	self->pkt = NULL;
	self->pos = 0;
}


static int vstrm_rtp_h264_tx_add_nalu(struct vstrm_rtp_h264_tx *self,
				      struct vstrm_frame_nalu *nalu)
{
	int res = 0;
	size_t max_size = self->cfg.dyn.target_packet_size;

	/* Can we put this nalu in current payload? */
	if (self->pkt != NULL && self->pos + 2 + nalu->len < max_size) {
		uint16_t len = htons(nalu->len);
		CHECK(pomp_buffer_write(
			self->pkt->raw.buf, &self->pos, &len, sizeof(len)));
		CHECK(pomp_buffer_write(self->pkt->raw.buf,
					&self->pos,
					nalu->cdata,
					nalu->len));
		if (nalu->priority < self->pkt->priority)
			self->pkt->priority = nalu->priority;
		if (nalu->importance < self->pkt->importance)
			self->pkt->importance = nalu->importance;
		return 0;
	}

	/* Finish current packet, start a new one */
	if (self->pkt != NULL)
		vstrm_rtp_h264_tx_end_pkt(self);
	CHECK(vstrm_rtp_h264_tx_begin_pkt(
		self, nalu->priority, nalu->importance));

	if (self->pos + nalu->len < 3 * max_size / 4 &&
	    self->pos + nalu->len + 3 < max_size) {
		/* Aggregation */
		uint16_t len = htons(nalu->len);
		uint8_t stap_ind = VSTRM_RTP_H264_NALU_TYPE_STAP_A;
		CHECK(pomp_buffer_write(self->pkt->raw.buf,
					&self->pos,
					&stap_ind,
					sizeof(stap_ind)));
		CHECK(pomp_buffer_write(
			self->pkt->raw.buf, &self->pos, &len, sizeof(len)));
		CHECK(pomp_buffer_write(self->pkt->raw.buf,
					&self->pos,
					nalu->cdata,
					nalu->len));
	} else if (self->pos + nalu->len <= max_size) {
		/* Single */
		CHECK(pomp_buffer_write(self->pkt->raw.buf,
					&self->pos,
					nalu->cdata,
					nalu->len));
		vstrm_rtp_h264_tx_end_pkt(self);
	} else {
		/* Fragmentation */
		size_t off = 1, len = 0;
		int start, end = 0;
		while ((!end) && (off < nalu->len)) {
			/* TODO: check overflow */
			len = max_size - self->pos - 2;
			if (len > nalu->len - off)
				len = nalu->len - off;
			uint8_t fu_ind = (nalu->cdata[0] & 0xe0) |
					 VSTRM_RTP_H264_NALU_TYPE_FU_A;
			start = (off == 1);
			end = (off + len == nalu->len);
			uint8_t fu_hdr = (start << 7) | (end << 6) |
					 (nalu->cdata[0] & 0x1f);
			CHECK(pomp_buffer_write(self->pkt->raw.buf,
						&self->pos,
						&fu_ind,
						sizeof(fu_ind)));
			CHECK(pomp_buffer_write(self->pkt->raw.buf,
						&self->pos,
						&fu_hdr,
						sizeof(fu_hdr)));
			CHECK(pomp_buffer_write(self->pkt->raw.buf,
						&self->pos,
						nalu->cdata + off,
						len));
			vstrm_rtp_h264_tx_end_pkt(self);
			if (!end)
				CHECK(vstrm_rtp_h264_tx_begin_pkt(
					self,
					nalu->priority,
					nalu->importance));
			off += len;
		}
	}

out:
	return res;
}


int vstrm_rtp_h264_tx_new(const struct vstrm_rtp_h264_tx_cfg *cfg,
			  struct vstrm_rtp_h264_tx **ret_obj)
{
	struct vstrm_rtp_h264_tx *self = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg == NULL, EINVAL);

	*ret_obj = NULL;

	/* Allocate and initialize structure */
	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->cfg = *cfg;

	*ret_obj = self;
	return 0;
}


int vstrm_rtp_h264_tx_destroy(struct vstrm_rtp_h264_tx *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->frame != NULL, EBUSY);
	ULOG_ERRNO_RETURN_ERR_IF(self->packets != NULL, EBUSY);
	ULOG_ERRNO_RETURN_ERR_IF(self->pkt != NULL, EBUSY);

	free(self);
	return 0;
}


int vstrm_rtp_h264_tx_process_frame(struct vstrm_rtp_h264_tx *self,
				    struct vstrm_frame *frame,
				    struct list_node *packets)
{
	int res = 0;
	struct vstrm_frame_nalu *nalu = NULL;
	struct list_node *node = NULL;
	struct rtp_pkt *pkt = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(packets == NULL, EINVAL);

	self->frame = frame;
	self->packets = packets;

	for (uint32_t i = 0; i < frame->nalu_count; i++) {
		nalu = &frame->nalus[i];
		res = vstrm_rtp_h264_tx_add_nalu(self, nalu);
		if (res < 0)
			goto error;
	}

	/* Finish last packet */
	if (self->pkt != NULL)
		vstrm_rtp_h264_tx_end_pkt(self);
	self->pkt = NULL;

	/* Set market bit of last packet */
	node = list_is_empty(packets) ? NULL : list_last(packets);
	if (node != NULL) {
		pkt = list_entry(node, struct rtp_pkt, node);
		RTP_PKT_HEADER_FLAGS_SET(pkt->header.flags, MARKER, 1);
	}

	self->frame = NULL;
	self->packets = NULL;
	return 0;

	/* Cleanup in case of error */
error:
	while (!list_is_empty(packets)) {
		pkt = list_entry(list_first(packets), struct rtp_pkt, node);
		list_del(&pkt->node);
		rtp_pkt_destroy(pkt);
	}
	if (self->pkt != NULL)
		rtp_pkt_destroy(self->pkt);
	self->pkt = NULL;

	self->frame = NULL;
	self->packets = NULL;
	return res;
}


int vstrm_rtp_h264_tx_set_cfg_dyn(
	struct vstrm_rtp_h264_tx *self,
	const struct vstrm_rtp_h264_tx_cfg_dyn *cfg_dyn)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg_dyn == NULL, EINVAL);

	self->cfg.dyn = *cfg_dyn;
	return 0;
}
