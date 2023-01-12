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

#define CHECK_FLAG(_flags, _f) (((_flags)&VSTRM_RECEIVER_FLAGS_##_f) != 0)

#define CHECK_ENV_FLAG(_flag)                                                  \
	((_flag) != NULL &&                                                    \
	 ((_flag)[0] == '1' || (_flag)[0] == 'y' || (_flag)[0] == 'Y'))


struct vstrm_rtp_h264_rx_frame {
	struct vstrm_frame *base;
};


struct vstrm_rtp_h264_rx_slice {
	int valid;
	struct h264_nalu_header nalu_header;
	struct h264_slice_header slice_header;
	const uint8_t *buf;
	size_t len;
	uint32_t mb_count;
	enum vstrm_frame_mb_status mb_status;
};


struct vstrm_rtp_h264_rx {
	struct vstrm_rtp_h264_rx_cfg cfg;
	struct vstrm_rtp_h264_rx_cbs cbs;
	uint8_t current_fu_type;
	int gap;
	int marker;
	struct h264_reader *reader;
	struct vstrm_video_stats video_stats;
	struct vstrm_video_stats_dyn video_stats_dyn;
	uint64_t err_sec_start_time;
	uint64_t *err_sec_start_time_by_zone;
	uint32_t max_frame_num;
	struct vstrm_codec_info codec_info;

	struct {
		struct vstrm_rtp_h264_rx_frame *frame;
		struct vmeta_frame *metadata;
		struct vstrm_frame_info info;
		struct vstrm_timestamp timestamp;
		struct vstrm_timestamp last_timestamp;
		uint64_t last_out_timestamp;
		uint32_t frame_num;
		uint32_t prev_frame_num;
		int first;
		int idr;
		int recovery_point;
	} au;

	struct {
		enum h264_nalu_type type;
		struct pomp_buffer *buf;
		int first;
		int last;
	} nalu;

	struct {
		int valid;
		struct h264_sps sps;
		struct h264_sps_derived sps_derived;
		uint8_t *buf;
		size_t len;
		size_t maxsize;
	} sps;

	struct {
		int valid;
		struct h264_pps pps;
		uint8_t *buf;
		size_t len;
		size_t maxsize;
	} pps;

	int update_mb_status;
	struct vstrm_rtp_h264_rx_slice slice;
	struct vstrm_rtp_h264_rx_slice slice_copy;
	struct vstrm_rtp_h264_rx_slice prev_slice;
	struct vstrm_rtp_h264_rx_slice tmp_slice;
	struct h264_slice_header tmp_slice_header;

	struct {
		int valid;
		struct vstrm_h264_sei_streaming_v1 sei;
	} info_v1;

	struct {
		int valid;
		struct vstrm_h264_sei_streaming_v2 sei;
	} info_v2;

	struct {
		int valid;
		struct vstrm_h264_sei_streaming_v4 sei;
	} info_v4;

	struct {
		uint64_t pack_bf_low;
		uint64_t pack_bf_high;
		uint8_t *buf;
		size_t len;
		size_t capacity;
	} metadata;
};


static void vstrm_rtp_h264_rx_frame_dispose(struct vstrm_frame *base)
{
	/* Release all buffers associated with NAL units */
	for (uint32_t i = 0; i < base->nalu_count; i++) {
		struct pomp_buffer *buf = base->nalus[i].userdata;
		pomp_buffer_unref(buf);
	}

	/* Release macroblock status */
	free(base->info.mb_status);
}


static int vstrm_rtp_h264_rx_frame_new(struct vstrm_rtp_h264_rx *rx,
				       struct vstrm_rtp_h264_rx_frame **ret_obj)
{
	int res = 0;
	struct vstrm_frame_ops ops;
	struct vstrm_frame *base = NULL;
	struct vstrm_rtp_h264_rx_frame *self = NULL;

	/* Create frame structure */
	memset(&ops, 0, sizeof(ops));
	ops.dispose = &vstrm_rtp_h264_rx_frame_dispose;
	res = vstrm_frame_new(&ops, sizeof(*self), &base);
	if (res < 0)
		return res;

	/* Link pointers */
	self = base->userdata;
	self->base = base;

	/* Alloc and init macroblock status to unknown */
	self->base->info.complete = 1;
	if (rx->au.info.mb_total != 0) {
		self->base->info.mb_width = rx->au.info.mb_width;
		self->base->info.mb_height = rx->au.info.mb_height;
		self->base->info.mb_total = rx->au.info.mb_total;
		self->base->info.mb_status = malloc(rx->au.info.mb_total);
		if (self->base->info.mb_status != NULL) {
			memset(self->base->info.mb_status,
			       VSTRM_FRAME_MB_STATUS_UNKNOWN,
			       rx->au.info.mb_total);
		}
	}

	*ret_obj = self;
	return 0;
}


static int
vstrm_rtp_h264_rx_frame_add_nalu(struct vstrm_rtp_h264_rx_frame *self,
				 struct pomp_buffer *buf)
{
	int res = 0;
	const void *cdata = NULL;
	size_t len = 0;
	struct vstrm_frame_nalu nalu;

	/* Get the data of the NALU */
	res = pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
	if (res < 0)
		goto out;

	/* Setup the NALU structure and add it in the frame */
	memset(&nalu, 0, sizeof(nalu));
	nalu.cdata = cdata;
	nalu.len = len;
	nalu.userdata = buf;
	res = vstrm_frame_add_nalu(self->base, &nalu);
	if (res < 0)
		goto out;

	/* Now that the NALU is in the frame, add a ref on the buffer */
	pomp_buffer_ref(buf);

out:
	return res;
}


static void vstrm_rtp_h264_rx_nalu_begin_cb(struct h264_ctx *ctx,
					    enum h264_nalu_type type,
					    const uint8_t *buf,
					    size_t len,
					    const struct h264_nalu_header *nh,
					    void *userdata)
{
}


static void vstrm_rtp_h264_rx_slice_cb(struct h264_ctx *ctx,
				       const uint8_t *buf,
				       size_t len,
				       const struct h264_slice_header *sh,
				       void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	struct h264_nalu_header nh;
	int res;

	/* Save header */
	self->slice.valid = 1;
	self->slice.slice_header = *sh;

	/* We can store pointers, we actually own the data stored in the
	 * NALU buffer */
	self->slice.buf = buf;
	self->slice.len = len;

	switch (H264_SLICE_TYPE(sh->slice_type)) {
	case H264_SLICE_TYPE_I:
	case H264_SLICE_TYPE_SI:
		self->slice.mb_status = VSTRM_FRAME_MB_STATUS_VALID_ISLICE;
		break;
	case H264_SLICE_TYPE_P:
	case H264_SLICE_TYPE_SP:
		self->slice.mb_status = VSTRM_FRAME_MB_STATUS_VALID_PSLICE;
		/* Detect the refresh zones with encodings not using I
		 * slice types: we use the NRI bits value 3 for refresh
		 * zones, other values are for non-refresh zones */
		res = h264_parse_nalu_header(buf, len, &nh);
		if (res < 0)
			ULOG_ERRNO("h264_parse_nalu_header", -res);
		else if (nh.nal_ref_idc == 3)
			self->slice.mb_status =
				VSTRM_FRAME_MB_STATUS_VALID_ISLICE;
		break;
	default:
		self->slice.mb_status = VSTRM_FRAME_MB_STATUS_UNKNOWN;
		break;
	}


	if (sh->rplm.ref_pic_list_modification_flag_l0) {
		for (int i = 0;
		     sh->rplm.pic_num_l0[i].modification_of_pic_nums_idc != 3;
		     i++) {
			self->au.info.uses_ltr =
				self->au.info.uses_ltr ||
				(sh->rplm.pic_num_l0[i]
					 .modification_of_pic_nums_idc == 2);
		}
	}
}


static void
vstrm_rtp_h264_rx_slice_data_end_cb(struct h264_ctx *ctx,
				    const struct h264_slice_header *sh,
				    uint32_t mb_count,
				    void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	self->slice.mb_count = mb_count;
}

static void vstrm_rtp_h264_rx_set_mb_status(struct vstrm_rtp_h264_rx *self,
					    uint32_t mb_start,
					    uint32_t mb_count,
					    enum vstrm_frame_mb_status status,
					    struct vstrm_frame_info *info);


static void
vstrm_rtp_h264_rx_slice_data_mb_cb(struct h264_ctx *ctx,
				   const struct h264_slice_header *sh,
				   uint32_t mb_addr,
				   enum h264_mb_type mb_type,
				   void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	enum vstrm_frame_mb_status status = VSTRM_FRAME_MB_STATUS_UNKNOWN;
	if (self->update_mb_status) {
		switch (mb_type) {
		case H264_MB_TYPE_I_NxN: /* NO BREAK */
		case H264_MB_TYPE_I_16x16: /* NO BREAK */
		case H264_MB_TYPE_I_PCM: /* NO BREAK */
		case H264_MB_TYPE_SI:
			status = VSTRM_FRAME_MB_STATUS_VALID_ISLICE;
			break;
		case H264_MB_TYPE_P_SKIP: /* NO BREAK */
		case H264_MB_TYPE_P_16x16: /* NO BREAK */
		case H264_MB_TYPE_P_16x8: /* NO BREAK */
		case H264_MB_TYPE_P_8x16: /* NO BREAK */
		case H264_MB_TYPE_P_8x8: /* NO BREAK */
		case H264_MB_TYPE_P_8x8ref0:
			status = VSTRM_FRAME_MB_STATUS_VALID_PSLICE;
			break;
		default:
			status = VSTRM_FRAME_MB_STATUS_UNKNOWN;
			break;
		}
		vstrm_rtp_h264_rx_set_mb_status(
			self, mb_addr, 1, status, &self->au.frame->base->info);
	}
}


static void vstrm_rtp_h264_rx_sps_cb(struct h264_ctx *ctx,
				     const uint8_t *buf,
				     size_t len,
				     const struct h264_sps *sps,
				     void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	void *newbuf = NULL;

	/* Resize internal buffer if needed */
	if (self->sps.maxsize < len) {
		newbuf = realloc(self->sps.buf, len);
		if (newbuf == NULL)
			return;
		self->sps.buf = newbuf;
		self->sps.maxsize = len;
	}

	/* Copy information */
	self->sps.valid = 1;
	self->sps.sps = *sps;
	memcpy(self->sps.buf, buf, len);
	self->sps.len = len;

	/* Get derived information */
	h264_get_sps_derived(sps, &self->sps.sps_derived);
}


static void vstrm_rtp_h264_rx_pps_cb(struct h264_ctx *ctx,
				     const uint8_t *buf,
				     size_t len,
				     const struct h264_pps *pps,
				     void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	void *newbuf = NULL;

	/* Resize internal buffer if needed */
	if (self->pps.maxsize < len) {
		newbuf = realloc(self->pps.buf, len);
		if (newbuf == NULL)
			return;
		self->pps.buf = newbuf;
		self->pps.maxsize = len;
	}

	/* Copy information */
	self->pps.valid = 1;
	self->pps.pps = *pps;
	memcpy(self->pps.buf, buf, len);
	self->pps.len = len;
}


static void vstrm_rtp_h264_rx_sei_recovery_point_cb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_recovery_point *sei,
	void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	self->au.recovery_point = 1;
}


static void vstrm_rtp_h264_rx_sei_user_data_unregistered_cb(
	struct h264_ctx *ctx,
	const uint8_t *buf,
	size_t len,
	const struct h264_sei_user_data_unregistered *sei,
	void *userdata)
{
	int res = 0;
	struct vstrm_rtp_h264_rx *self = userdata;

	/* Decode streaming SEI v1 or v2 */
	if (vstrm_h264_sei_streaming_is_v1(sei->uuid)) {
		res = vstrm_h264_sei_streaming_v1_read(
			&self->info_v1.sei, sei->uuid, sei->buf, sei->len);
		if (res < 0)
			ULOG_ERRNO("vstrm_h264_sei_streaming_v1_read", -res);
		else
			self->info_v1.valid = 1;
	} else if (vstrm_h264_sei_streaming_is_v2(sei->uuid)) {
		res = vstrm_h264_sei_streaming_v2_read(
			&self->info_v2.sei, sei->uuid, sei->buf, sei->len);
		if (res < 0)
			ULOG_ERRNO("vstrm_h264_sei_streaming_v2_read", -res);
		else
			self->info_v2.valid = 1;
	} else if (vstrm_h264_sei_streaming_is_v4(sei->uuid)) {
		res = vstrm_h264_sei_streaming_v4_read(
			&self->info_v4.sei, sei->uuid, sei->buf, sei->len);
		if (res < 0)
			ULOG_ERRNO("vstrm_h264_sei_streaming_v4_read", -res);
		else
			self->info_v4.valid = 1;
	}
}


static void vstrm_rtp_h264_rx_set_mb_status(struct vstrm_rtp_h264_rx *self,
					    uint32_t mb_start,
					    uint32_t mb_count,
					    enum vstrm_frame_mb_status status,
					    struct vstrm_frame_info *info)
{
	if (info->mb_status == NULL)
		return;

	uint8_t *mb_status = info->mb_status;

	/* Make sure range is valid */
	if (mb_start > info->mb_total || mb_start + mb_count > info->mb_total) {
		ULOGE("rtp_h264: invalid macroblock range: %u,%u (%ux%u)",
		      mb_start,
		      mb_count,
		      info->mb_width,
		      info->mb_height);
		return;
	}

	if (status == VSTRM_FRAME_MB_STATUS_VALID_PSLICE) {
		/* Need to check ref. status */
		for (uint32_t i = mb_start; i < mb_start + mb_count; i++) {
			switch (self->au.info.mb_status[i]) {
			case VSTRM_FRAME_MB_STATUS_VALID_ISLICE:
			case VSTRM_FRAME_MB_STATUS_VALID_PSLICE:
				/* OK, we can set the status */
				mb_status[i] = status;
				break;
			default:
				/* Error propagation */
				mb_status[i] =
					VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION;
				info->error = 1;
				break;
			}
		}
	} else {
		/* Set status directly */
		memset(mb_status + mb_start, status & 0xff, mb_count);
	}
}


static int
vstrm_rtp_h264_rx_gen_grey_i_slice(struct vstrm_rtp_h264_rx *self,
				   uint32_t nal_ref_idc,
				   const struct h264_slice_header *ref_sh,
				   uint32_t mb_start,
				   uint32_t mb_count,
				   struct vstrm_rtp_h264_rx_slice *new_slice,
				   struct pomp_buffer **buf)
{
	int res = 0;
	struct h264_ctx *ctx = h264_reader_get_ctx(self->reader);
	void *data = NULL;
	struct h264_nalu_header nh;
	struct h264_bitstream bs;

	/* Allocate output buffer, initialize bitstream */
	*buf = pomp_buffer_new_get_data(256 + mb_count, &data);
	if (*buf == NULL)
		return -ENOMEM;
	h264_bs_init(&bs, data, 256 + mb_count, 1);

	/* Only YUV 4:2:0 and YUV 4:2:2 are supported */
	if (self->sps.sps.chroma_format_idc != 1 &&
	    self->sps.sps.chroma_format_idc != 2) {
		res = -EIO;
		goto out;
	}

	/* Start NALU */
	res = h264_ctx_clear_nalu(ctx);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_clear_nalu", -res);
		goto out;
	}

	/* Setup NALU header */
	memset(&nh, 0, sizeof(nh));
	nh.nal_ref_idc = nal_ref_idc;
	nh.nal_unit_type = H264_NALU_TYPE_SLICE_IDR;
	res = h264_ctx_set_nalu_header(ctx, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_nalu_header", -res);
		goto out;
	}

	/* Setup slice header */
	memset(&self->tmp_slice_header, 0, sizeof(self->tmp_slice_header));
	if (ref_sh)
		self->tmp_slice_header = *ref_sh;
	self->tmp_slice_header.first_mb_in_slice = mb_start;
	self->tmp_slice_header.slice_type = H264_SLICE_TYPE_I;
	self->tmp_slice_header.redundant_pic_cnt = 0;
	self->tmp_slice_header.direct_spatial_mv_pred_flag = 0;
	self->tmp_slice_header.slice_qp_delta = 0;
	self->tmp_slice_header.disable_deblocking_filter_idc = 2;
	self->tmp_slice_header.slice_alpha_c0_offset_div2 = 0;
	self->tmp_slice_header.slice_beta_offset_div2 = 0;
	res = h264_ctx_set_slice_header(ctx, &self->tmp_slice_header);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_slice_header", -res);
		goto out;
	}

	/* Write slice */
	res = h264_write_grey_i_slice(&bs, ctx, mb_count);
	if (res < 0) {
		ULOG_ERRNO("h264_write_grey_i_slice", -res);
		goto out;
	}

	/* Finish pomp buffer */
	res = pomp_buffer_set_len(*buf, bs.off);
	if (res < 0) {
		ULOG_ERRNO("pomp_buffer_set_len", -res);
		goto out;
	}

	/* Fill slice information */
	memset(new_slice, 0, sizeof(*new_slice));
	new_slice->valid = 1;
	new_slice->nalu_header = nh;
	new_slice->slice_header = self->tmp_slice_header;
	new_slice->buf = bs.data;
	new_slice->len = bs.off;
	new_slice->mb_count = mb_count;
	new_slice->mb_status = VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED;

out:
	h264_bs_clear(&bs);
	if (res != 0 && *buf != NULL) {
		pomp_buffer_unref(*buf);
		*buf = NULL;
	}
	return res;
}


static int
vstrm_rtp_h264_rx_gen_grey_i_frame(struct vstrm_rtp_h264_rx *self,
				   struct vstrm_rtp_h264_rx_frame **frame)
{
	int res = 0;
	struct pomp_buffer *buf = NULL;

	/* Create frame */
	res = vstrm_rtp_h264_rx_frame_new(self, frame);
	if (res < 0)
		goto out;
	(*frame)->base->info.ref = 1;
	(*frame)->base->info.gen_grey_idr = 1;

	/* Insert SPS */
	buf = pomp_buffer_new_with_data(self->sps.buf, self->sps.len);
	if (buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	res = vstrm_rtp_h264_rx_frame_add_nalu(*frame, buf);
	if (res < 0)
		goto out;
	pomp_buffer_unref(buf);
	buf = NULL;

	/* Insert PPS */
	buf = pomp_buffer_new_with_data(self->pps.buf, self->pps.len);
	if (buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	res = vstrm_rtp_h264_rx_frame_add_nalu(*frame, buf);
	if (res < 0)
		goto out;
	pomp_buffer_unref(buf);
	buf = NULL;

	/* Insert grey I slice */
	memset(&self->tmp_slice, 0, sizeof(self->tmp_slice));
	res = vstrm_rtp_h264_rx_gen_grey_i_slice(self,
						 3,
						 NULL,
						 0,
						 self->au.info.mb_total,
						 &self->tmp_slice,
						 &buf);
	if (res < 0)
		goto out;
	res = vstrm_rtp_h264_rx_frame_add_nalu(*frame, buf);
	if (res < 0)
		goto out;
	pomp_buffer_unref(buf);
	buf = NULL;

	/* Initialize MB status */
	vstrm_rtp_h264_rx_set_mb_status(self,
					0,
					self->au.info.mb_total,
					VSTRM_FRAME_MB_STATUS_VALID_ISLICE,
					&(*frame)->base->info);

	/* Copy video statistics */
	(*frame)->base->video_stats = self->video_stats;
	vstrm_video_stats_dyn_copy(&(*frame)->base->video_stats_dyn,
				   &self->video_stats_dyn);

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
	if (res != 0 && *frame != NULL) {
		vstrm_frame_unref((*frame)->base);
		*frame = NULL;
	}
	return res;
}


static int
vstrm_rtp_h264_rx_gen_skipped_p_slice(struct vstrm_rtp_h264_rx *self,
				      uint32_t nal_ref_idc,
				      const struct h264_slice_header *ref_sh,
				      uint32_t mb_start,
				      uint32_t mb_count,
				      struct vstrm_rtp_h264_rx_slice *new_slice,
				      struct pomp_buffer **buf)
{
	int res = 0;
	struct h264_ctx *ctx = h264_reader_get_ctx(self->reader);
	void *data = NULL;
	struct h264_nalu_header nh;
	struct h264_bitstream bs;

	/* Allocate output buffer, initialize bitstream */
	*buf = pomp_buffer_new_get_data(256, &data);
	if (*buf == NULL)
		return -ENOMEM;
	h264_bs_init(&bs, data, 256, 1);

	/* Start NALU */
	res = h264_ctx_clear_nalu(ctx);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_clear_nalu", -res);
		goto out;
	}

	/* Setup NALU header */
	memset(&nh, 0, sizeof(nh));
	nh.nal_ref_idc = nal_ref_idc;
	nh.nal_unit_type = H264_NALU_TYPE_SLICE;
	res = h264_ctx_set_nalu_header(ctx, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_nalu_header", -res);
		goto out;
	}

	/* Setup slice header */
	memset(&self->tmp_slice_header, 0, sizeof(self->tmp_slice_header));
	if (ref_sh)
		self->tmp_slice_header = *ref_sh;
	self->tmp_slice_header.first_mb_in_slice = mb_start;
	self->tmp_slice_header.slice_type = H264_SLICE_TYPE_P;
	self->tmp_slice_header.redundant_pic_cnt = 0;
	self->tmp_slice_header.direct_spatial_mv_pred_flag = 0;
	self->tmp_slice_header.slice_qp_delta = 0;
	self->tmp_slice_header.disable_deblocking_filter_idc = 2;
	self->tmp_slice_header.slice_alpha_c0_offset_div2 = 0;
	self->tmp_slice_header.slice_beta_offset_div2 = 0;
	res = h264_ctx_set_slice_header(ctx, &self->tmp_slice_header);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_slice_header", -res);
		goto out;
	}

	/* Write slice */
	res = h264_write_skipped_p_slice(&bs, ctx, mb_count);
	if (res < 0) {
		ULOG_ERRNO("h264_write_skipped_p_slice", -res);
		goto out;
	}

	/* Finish pomp buffer */
	res = pomp_buffer_set_len(*buf, bs.off);
	if (res < 0) {
		ULOG_ERRNO("pomp_buffer_set_len", -res);
		goto out;
	}

	/* Fill slice information */
	memset(new_slice, 0, sizeof(*new_slice));
	new_slice->valid = 1;
	new_slice->nalu_header = nh;
	new_slice->slice_header = self->tmp_slice_header;
	new_slice->buf = bs.data;
	new_slice->len = bs.off;
	new_slice->mb_count = mb_count;
	new_slice->mb_status = VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED;

out:
	h264_bs_clear(&bs);
	if (res != 0 && *buf != NULL) {
		pomp_buffer_unref(*buf);
		*buf = NULL;
	}
	return res;
}


static void
vstrm_rtp_h264_rx_dump_mb_status(struct vstrm_rtp_h264_rx *self,
				 struct vstrm_rtp_h264_rx_frame *frame)
{
#if 0
	if (frame->base->info.mb_status == NULL)
		return;

	uint8_t *mb_status = frame->base->info.mb_status;
	ULOGI("==========");

	/* TODO: avoid runtime stack allocation */
	char row[self->au.info.mb_width + 1];
	for (uint32_t j = 0; j < self->au.info.mb_height; j++) {
		for (uint32_t i = 0; i < self->au.info.mb_width; i++) {
			switch (mb_status[j * self->au.info.mb_width + i]) {
			default: /* NO BREAK */
			case VSTRM_FRAME_MB_STATUS_UNKNOWN:
				row[i] = '?';
				break;
			case VSTRM_FRAME_MB_STATUS_VALID_ISLICE:
				row[i] = 'I';
				break;
			case VSTRM_FRAME_MB_STATUS_VALID_PSLICE:
				row[i] = 'P';
				break;
			case VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED:
				row[i] = '-';
				break;
			case VSTRM_FRAME_MB_STATUS_MISSING:
				row[i] = 'x';
				break;
			case VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION:
				row[i] = 'e';
				break;
			}
		}
		row[self->au.info.mb_width] = '\0';
		ULOGI("|%s|", row);
	}
#endif
}


static void
vstrm_rtp_h264_rx_update_err_sec_stats(struct vstrm_rtp_h264_rx *self,
				       struct vstrm_video_stats_dyn *dyn,
				       uint32_t zone)
{
	if (self->au.timestamp.ntp_raw >
	    self->err_sec_start_time + VSTRM_USECS_PER_SEC) {
		self->err_sec_start_time = self->au.timestamp.ntp_raw;
		self->video_stats.v2.errored_second_count++;
	}
	if (self->au.timestamp.ntp_raw >
	    self->err_sec_start_time_by_zone[zone] + VSTRM_USECS_PER_SEC) {
		self->err_sec_start_time_by_zone[zone] =
			self->au.timestamp.ntp_raw;
		vstrm_video_stats_dyn_inc_errored_second_count_by_zone(dyn,
								       zone);
	}
}


static void
vstrm_rtp_h264_rx_update_mb_status_stats(struct vstrm_rtp_h264_rx *self)
{
	struct vstrm_video_stats_dyn *dyn = &self->video_stats_dyn;
	for (uint32_t j = 0, k = 0; j < self->au.info.mb_height; j++) {
		for (uint32_t i = 0; i < self->au.info.mb_width; i++, k++) {
			uint32_t zone = j * dyn->mb_status_zone_count /
					self->au.info.mb_height;
			uint8_t status =
				self->au.frame->base->info.mb_status[k];
			vstrm_video_stats_dyn_inc_mb_status_count(
				dyn, status, zone);
			if (status != VSTRM_FRAME_MB_STATUS_VALID_ISLICE &&
			    status != VSTRM_FRAME_MB_STATUS_VALID_PSLICE) {
				vstrm_rtp_h264_rx_update_err_sec_stats(
					self, dyn, zone);
			}
		}
	}
}


static void vstrm_rtp_h264_rx_map_timestamps(const struct vstrm_timestamp *src,
					     uint64_t out_timestamp,
					     struct vstrm_frame_timestamps *dst)
{
	dst->ntp = src->ntp;
	dst->ntp_unskewed = src->ntp_unskewed;
	dst->ntp_raw = src->ntp_raw;
	dst->ntp_raw_unskewed = src->ntp_raw_unskewed;
	dst->local = src->ntp_local;
	dst->recv_start = src->input;
	dst->recv_end = out_timestamp;
}


static int vstrm_rtp_h264_rx_au_complete(struct vstrm_rtp_h264_rx *self)
{
	int ref = 0;
	struct timespec ts = {0, 0};
	uint64_t out_timestamp = 0;

	/* Wait for SPS/PPS */
	if (!self->sps.valid || !self->pps.valid) {
		/* TODO: only print once */
		ULOGD("rtp_h264: waiting for sps/pps: dropping frame");
		/* Do not count frames before sync in total/discarded/missed
		 * counters: before receiving the SPS/PPS the decoding cannot
		 * start and all frames are dropped; they are not to be
		 * counted in the video stats */
		goto out;
	}

	self->video_stats.v2.total_frame_count++;

	if (self->au.frame == NULL) {
		/* No data in frame */
		self->video_stats.v2.discarded_frame_count++;
		self->video_stats.v2.missed_frame_count++;
		goto out;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &out_timestamp);

	/* Copy timestamp and metadata */
	vstrm_rtp_h264_rx_map_timestamps(&self->au.timestamp,
					 out_timestamp,
					 &self->au.frame->base->timestamps);
	self->au.frame->base->metadata = self->au.metadata;
	if (self->au.frame->base->metadata)
		vmeta_frame_ref(self->au.frame->base->metadata);

	/* Update timing statistics */
	self->au.last_timestamp = self->au.timestamp;
	self->au.last_out_timestamp = out_timestamp;

	/* Update macroblock status of last slice
	 * (Not needed in FULL_MB_STATUS) */
	if (self->prev_slice.valid &&
	    !CHECK_FLAG(self->cfg.flags, H264_FULL_MB_STATUS)) {
		uint32_t mb_start =
			self->prev_slice.slice_header.first_mb_in_slice;
		uint32_t mb_end = self->au.info.mb_total;
		if (mb_start >= mb_end) {
			/* Mismatch in macroblock ordering */
			self->au.frame->base->info.error = 1;
			ULOGW("%s: mismatch in macroblock ordering: "
			      "mb_start=%u mb_end=%u",
			      __func__,
			      mb_start,
			      mb_end);
		} else {
			vstrm_rtp_h264_rx_set_mb_status(
				self,
				mb_start,
				mb_end - mb_start,
				self->prev_slice.mb_status,
				&self->au.frame->base->info);
		}
	}
	vstrm_rtp_h264_rx_update_mb_status_stats(self);
	vstrm_rtp_h264_rx_dump_mb_status(self, self->au.frame);

	/* Copy video statistics */
	self->au.frame->base->video_stats = self->video_stats;
	vstrm_video_stats_dyn_copy(&self->au.frame->base->video_stats_dyn,
				   &self->video_stats_dyn);

	/* If the frame is a reference, save the MB status */
	ref = self->au.frame->base->info.ref;
	if (ref) {
		memcpy(self->au.info.mb_status,
		       self->au.frame->base->info.mb_status,
		       self->au.info.mb_total);
	}

	/* Notify upper layer */
	self->video_stats.v2.output_frame_count++;
	if (self->au.frame->base->info.error)
		self->video_stats.v2.errored_output_frame_count++;
	if (!self->au.frame->base->info.complete &&
	    CHECK_FLAG(self->cfg.flags, H264_GEN_CONCEALMENT_SLICE)) {
		ULOGW("rtp_h264: incomplete frame");
	}
	self->au.frame->base->info.uses_ltr = self->au.info.uses_ltr;
	(*self->cbs.recv_frame)(self, self->au.frame->base, self->cbs.userdata);

out:
	self->au.info.uses_ltr = false;
	/* Frame no more needed */
	if (self->au.frame != NULL)
		vstrm_frame_unref(self->au.frame->base);
	self->au.frame = NULL;

	/* Reset for next frame */
	self->nalu.first = 1;
	self->nalu.last = 0;
	if (self->au.metadata) {
		vmeta_frame_unref(self->au.metadata);
		self->au.metadata = NULL;
	}
	self->metadata.pack_bf_low = UINT64_C(0);
	self->metadata.pack_bf_high = UINT64_C(0);
	self->metadata.len = 0;
	memset(&self->au.timestamp, 0, sizeof(self->au.timestamp));
	self->au.info.complete = 1;
	self->au.info.error = 0;
	if (ref)
		self->au.prev_frame_num = self->au.frame_num;
	self->au.first = 0;
	self->au.idr = 0;
	self->au.recovery_point = 0;
	self->au.info.ref = 0;
	memset(&self->prev_slice, 0, sizeof(self->prev_slice));
	return 0;
}


static int vstrm_rtp_h264_rx_au_cancel(struct vstrm_rtp_h264_rx *self)
{
	if (self->au.frame != NULL)
		vstrm_frame_unref(self->au.frame->base);
	self->au.frame = NULL;

	return vstrm_rtp_h264_rx_au_complete(self);
}


static void
vstrm_rtp_h264_rx_check_missing_frames(struct vstrm_rtp_h264_rx *self)
{
	uint32_t missing = 0;

	/* If mode isn't intra refresh, we ignore IDR frames */
	if (self->au.prev_frame_num > self->au.frame_num)
		return;

	/* Determine missing count (handle wrapping) */
	missing = (self->au.frame_num - self->au.prev_frame_num - 1 +
		   self->max_frame_num) %
		  self->max_frame_num;
	if (missing != 0) {
		ULOGD("rtp_h264: missing frames: %u", missing);
		vstrm_rtp_h264_rx_set_mb_status(self,
						0,
						self->au.info.mb_total,
						VSTRM_FRAME_MB_STATUS_MISSING,
						&self->au.frame->base->info);
		self->video_stats.v2.total_frame_count += missing;
		self->video_stats.v2.missed_frame_count += missing;
		vstrm_rtp_h264_rx_update_mb_status_stats(self);
	}
}


static int
vstrm_rtp_h264_rx_au_add_nalu(struct vstrm_rtp_h264_rx *self,
			      struct pomp_buffer *buf,
			      const struct vstrm_rtp_h264_rx_slice *cur_slice)
{
	int res = 0;

	/* Generate a grey I frame if needed */
	if (self->au.first && self->nalu.first &&
	    CHECK_FLAG(self->cfg.flags, H264_GEN_GREY_IDR_FRAME)) {
		struct vstrm_rtp_h264_rx_frame *grey_i_frame = NULL;
		res = vstrm_rtp_h264_rx_gen_grey_i_frame(self, &grey_i_frame);
		if (res == 0) {
			vstrm_rtp_h264_rx_dump_mb_status(self, grey_i_frame);

			/* Notify upper layer */
			ULOGI("rtp_h264: inserting grey I frame");
			(*self->cbs.recv_frame)(
				self, grey_i_frame->base, self->cbs.userdata);
			vstrm_frame_unref(grey_i_frame->base);
		}
	}

	/* Create frame structure if needed */
	if (self->au.frame == NULL) {
		res = vstrm_rtp_h264_rx_frame_new(self, &self->au.frame);
		if (res < 0)
			goto out;
	}

	/* Add NALU in frame, nothing more to do if not a slice */
	res = vstrm_rtp_h264_rx_frame_add_nalu(self->au.frame, buf);
	if (res < 0)
		goto out;
	self->nalu.first = 0;
	if (cur_slice == NULL)
		goto out;

	/* Setup frame number for first slice */
	if (!self->prev_slice.valid) {
		self->au.frame_num = cur_slice->slice_header.frame_num;
		self->au.frame->base->info.ref =
			cur_slice->nalu_header.nal_ref_idc != 0;

		/* Check for missing frames
		 * (only for ref. frames except for first one) */
		if (!self->au.first && self->au.frame->base->info.ref)
			vstrm_rtp_h264_rx_check_missing_frames(self);
	}

	/* Update the macroblock status of the previous slice. It is delayed
	 * to avoid parsing slice data to determine the number of macroblocks
	 * in the previous slice (we use start of current as hint). */
	if (CHECK_FLAG(self->cfg.flags, H264_FULL_MB_STATUS)) {
		/* In FULL_MB_STATUS, parse the complete slice to get the
		 * status of each macroblock. It is not done for concealed
		 * slices as we already know the status of each macroblock. */
		if (cur_slice->mb_status ==
		    VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED) {
			vstrm_rtp_h264_rx_set_mb_status(
				self,
				cur_slice->slice_header.first_mb_in_slice,
				cur_slice->mb_count,
				cur_slice->mb_status,
				&self->au.frame->base->info);
		} else {
			const void *cdata = NULL;
			size_t len = 0;
			pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
			self->update_mb_status = 1;
			h264_reader_parse_nalu(self->reader,
					       H264_READER_FLAGS_SLICE_DATA,
					       cdata,
					       len);
			self->update_mb_status = 0;
		}
	} else if (self->prev_slice.valid) {
		uint32_t mb_start =
			self->prev_slice.slice_header.first_mb_in_slice;
		uint32_t mb_end = cur_slice->slice_header.first_mb_in_slice;
		if (mb_start >= mb_end) {
			/* Mismatch in macroblock ordering */
			self->au.frame->base->info.error = 1;
			ULOGW("%s: mismatch in macroblock ordering: "
			      "mb_start=%u mb_end=%u",
			      __func__,
			      mb_start,
			      mb_end);
		} else {
			vstrm_rtp_h264_rx_set_mb_status(
				self,
				mb_start,
				mb_end - mb_start,
				self->prev_slice.mb_status,
				&self->au.frame->base->info);
		}
	}

	/* Update previous slice */
	self->prev_slice = *cur_slice;

out:
	return res;
}


static int vstrm_rtp_h264_rx_handle_missing_slices(
	struct vstrm_rtp_h264_rx *self,
	const struct vstrm_rtp_h264_rx_slice *cur_slice)
{
	int res = 0;
	struct h264_nalu_header nh;
	uint32_t nal_ref_idc = 0;
	const struct h264_slice_header *ref_sh = NULL;
	uint32_t mb_start = 0;
	uint32_t mb_end = 0;
	struct pomp_buffer *buf = NULL;
	memset(&nh, 0, sizeof(nh));

	/* If no valid SPS/PPS yet, nothing to do */
	if (!self->sps.valid || !self->pps.valid)
		goto out;

	/* If no slice in frame yet, nothing to do */
	if (!self->prev_slice.valid && cur_slice == NULL)
		goto out;

	/* Determine start of gap */
	if (self->prev_slice.valid) {
		/* Use information found in custom SEI */
		if (self->info_v2.valid) {
			self->slice = self->prev_slice;
			self->slice.mb_count = self->info_v2.sei.slice_mb_count;
		} else if (self->info_v4.valid) {
			self->slice = self->prev_slice;
			self->slice.mb_count =
				(self->au.recovery_point)
					? self->info_v4.sei
						  .slice_mb_count_recovery_point
					: self->info_v4.sei.slice_mb_count;
		} else if (!self->pps.pps.entropy_coding_mode_flag) {
			/* Reparse previous slice data to get number of
			 * macroblocks */
			res = h264_reader_parse_nalu(
				self->reader,
				H264_READER_FLAGS_SLICE_DATA,
				self->prev_slice.buf,
				self->prev_slice.len);
			if (res < 0) {
				ULOGW("rtp_h264: failed to parse nalu: %d(%s)",
				      res,
				      strerror(-res));
				goto out;
			}
		} else {
			/* Number of macroblocks in CABAC not supported */
			self->au.frame->base->info.complete = 0;
			goto out;
		}

		/* Get nal_ref_idc of previous slice */
		res = h264_parse_nalu_header(
			self->slice.buf, self->slice.len, &nh);
		if (res < 0)
			goto out;
		nal_ref_idc = nh.nal_ref_idc;

		/* Setup reference slice header and start of gap */
		ref_sh = &self->slice.slice_header;
		mb_start = self->slice.slice_header.first_mb_in_slice +
			   self->slice.mb_count;
		self->prev_slice = self->slice;
	} else {
		mb_start = 0;
	}

	/* Determine end of gap */
	if (cur_slice != NULL) {
		/* Get nal_ref_idc of current slice */
		res = h264_parse_nalu_header(
			cur_slice->buf, cur_slice->len, &nh);
		if (res < 0)
			goto out;
		nal_ref_idc = nh.nal_ref_idc;

		/* Setup reference slice header and end of gap */
		if (ref_sh == NULL)
			ref_sh = &cur_slice->slice_header;
		mb_end = cur_slice->slice_header.first_mb_in_slice;
	} else {
		/* Last slice of frame is missing, need to to determine total
		 * number of macroblocks */
		mb_end = self->au.info.mb_total;
	}

	/* If the gap was just before the first slice
	 * (we missed AUD, SPS, SPS, SEI...) */
	if (mb_start == 0 && mb_end == 0)
		goto out;

	/* Create frame structure if needed */
	if (self->au.frame == NULL) {
		res = vstrm_rtp_h264_rx_frame_new(self, &self->au.frame);
		if (res < 0)
			goto out;
	}

	self->au.frame->base->info.error = 1;
	if (mb_start >= mb_end) {
		/* Mismatch in macroblock ordering */
		self->au.frame->base->info.complete = 0;
		ULOGW("%s: mismatch in macroblock ordering: "
		      "mb_start=%u mb_end=%u",
		      __func__,
		      mb_start,
		      mb_end);
		goto out;
	}
	ULOGD("rtp_h264: missing slices mb_start=%u mb_end=%u mb_count=%u",
	      mb_start,
	      mb_end,
	      mb_end - mb_start);

	/* Generate NALU with skipped P slice */
	if (CHECK_FLAG(self->cfg.flags, H264_GEN_CONCEALMENT_SLICE)) {
		memset(&self->tmp_slice, 0, sizeof(self->tmp_slice));
		if (self->au.idr) {
			res = vstrm_rtp_h264_rx_gen_grey_i_slice(
				self,
				nal_ref_idc,
				ref_sh,
				mb_start,
				mb_end - mb_start,
				&self->tmp_slice,
				&buf);
			if (res < 0)
				goto out;
		} else {
			res = vstrm_rtp_h264_rx_gen_skipped_p_slice(
				self,
				nal_ref_idc,
				ref_sh,
				mb_start,
				mb_end - mb_start,
				&self->tmp_slice,
				&buf);
			if (res < 0)
				goto out;
		}

		/* Add in frame */
		res = vstrm_rtp_h264_rx_au_add_nalu(
			self, buf, &self->tmp_slice);
		if (res < 0)
			goto out;
	} else {
		/* The frame will be incomplete */
		self->au.frame->base->info.complete = 0;
		self->prev_slice.mb_status = VSTRM_FRAME_MB_STATUS_MISSING;
		vstrm_rtp_h264_rx_set_mb_status(self,
						mb_start,
						mb_end - mb_start,
						VSTRM_FRAME_MB_STATUS_MISSING,
						&self->au.frame->base->info);
	}

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
	return res;
}


static int vstrm_rtp_h264_rx_handle_missing_eof(struct vstrm_rtp_h264_rx *self)
{
	/* Add missing slices at the end and complete the frame */
	vstrm_rtp_h264_rx_handle_missing_slices(self, NULL);
	vstrm_rtp_h264_rx_au_complete(self);
	return 0;
}


static void vstrm_rtp_h264_rx_sps(struct vstrm_rtp_h264_rx *self)
{
	uint32_t mb_width = 0, mb_height = 0;

	/* Abort current frame if any */
	if (self->au.frame != NULL) {
		vstrm_frame_unref(self->au.frame->base);
		self->au.frame = NULL;
	}

	self->max_frame_num = self->sps.sps_derived.MaxFrameNum;

	/* Compute size */
	mb_width = self->sps.sps_derived.PicWidthInMbs;
	mb_height = self->sps.sps_derived.FrameHeightInMbs;

	/* Update macroblock status array */
	if (mb_width != self->au.info.mb_width ||
	    mb_height != self->au.info.mb_height) {
		self->au.info.mb_width = mb_width;
		self->au.info.mb_height = mb_height;
		self->au.info.mb_total = mb_width * mb_height;
		free(self->au.info.mb_status);
		self->au.info.mb_status = calloc(1, self->au.info.mb_total);
		vstrm_rtp_h264_rx_set_mb_status(self,
						0,
						self->au.info.mb_total,
						VSTRM_FRAME_MB_STATUS_UNKNOWN,
						&self->au.info);
	}
}


static void vstrm_rtp_h264_rx_pps(struct vstrm_rtp_h264_rx *self)
{
	struct vstrm_codec_info codec_info;

	/* Nothing to do if the SPS is not valid */
	if (!self->sps.valid)
		return;

	/* Setup new codec info */
	memset(&codec_info, 0, sizeof(codec_info));
	codec_info.codec = VSTRM_CODEC_VIDEO_H264;
	codec_info.h264.width = self->sps.sps_derived.Width;
	codec_info.h264.height = self->sps.sps_derived.Height;

	/* Copy SPS */
	if (self->sps.len > sizeof(codec_info.h264.sps)) {
		ULOGE("SPS is too big: %zu (%zu)",
		      self->sps.len,
		      sizeof(codec_info.h264.sps));
		return;
	}
	codec_info.h264.spslen = self->sps.len;
	memcpy(codec_info.h264.sps, self->sps.buf, self->sps.len);

	/* Copy PPS */
	if (self->pps.len > sizeof(codec_info.h264.pps)) {
		ULOGE("PPS is too big: %zu (%zu)",
		      self->pps.len,
		      sizeof(codec_info.h264.pps));
		return;
	}
	codec_info.h264.ppslen = self->pps.len;
	memcpy(codec_info.h264.pps, self->pps.buf, self->pps.len);

	/* Check if changed */
	if (memcmp(&codec_info, &self->codec_info, sizeof(codec_info))) {
		/* Reset extra info found in SEI with geometry */
		self->info_v1.valid = 0;
		self->info_v2.valid = 0;
		self->info_v4.valid = 0;
		self->codec_info = codec_info;
		(*self->cbs.codec_info_changed)(
			self, &self->codec_info, self->cbs.userdata);
	}
}


static int vstrm_rtp_h264_rx_nalu_complete(struct vstrm_rtp_h264_rx *self)
{
	int res = 0;
	const void *cdata = NULL;
	size_t len = 0;
	struct h264_nalu_header nh;
	int parse_nalu = 0, is_slice = 0, is_idr = 0;
	memset(&nh, 0, sizeof(nh));
	memset(&self->slice_copy, 0, sizeof(self->slice_copy));

	res = pomp_buffer_get_cdata(self->nalu.buf, &cdata, &len, NULL);
	if (res < 0)
		goto out;

	/* Parse NALU header directly do to a pre-filter */
	res = h264_parse_nalu_header(cdata, len, &nh);
	if (res < 0)
		goto out;
	self->nalu.type = nh.nal_unit_type;

	/* Don't bother parsing some NALU if we don't have SPS/PPS */
	switch (self->nalu.type) {
	case H264_NALU_TYPE_SLICE_IDR:
		is_idr = 1;
		/* fall through */
	case H264_NALU_TYPE_SLICE:
	case H264_NALU_TYPE_SLICE_DPA:
	case H264_NALU_TYPE_SLICE_DPB:
	case H264_NALU_TYPE_SLICE_DPC:
		parse_nalu = self->sps.valid && self->pps.valid;
		is_slice = 1;
		self->au.idr = (self->nalu.type == H264_NALU_TYPE_SLICE_IDR);
		break;

	case H264_NALU_TYPE_SEI:
		parse_nalu = self->sps.valid && self->pps.valid;
		is_slice = 0;
		break;

	case H264_NALU_TYPE_SPS:
	case H264_NALU_TYPE_PPS:
	default:
		parse_nalu = 1;
		is_slice = 0;
		break;
	}

	/* Parse NALU if needed */
	if (parse_nalu) {
		memset(&self->slice, 0, sizeof(self->slice));
		res = h264_reader_parse_nalu(self->reader, 0, cdata, len);
		if (res < 0) {
			ULOG_ERRNO("h264_reader_parse_nalu", -res);
			goto out;
		}

		/* Save slice info in current slice */
		if (is_slice) {
			self->slice.nalu_header = nh;
			self->slice_copy = self->slice;
		} else if (self->nalu.type == H264_NALU_TYPE_SPS) {
			vstrm_rtp_h264_rx_sps(self);
		} else if (self->nalu.type == H264_NALU_TYPE_PPS) {
			vstrm_rtp_h264_rx_pps(self);
		}
	}

	/* If there was a gap, handle missing slices */
	if (self->gap) {
		res = vstrm_rtp_h264_rx_handle_missing_slices(
			self,
			self->slice_copy.valid ? &self->slice_copy : NULL);
		if (res < 0)
			goto out;
		self->gap = 0;
	}

	/* Add NALU in current frame */
	res = vstrm_rtp_h264_rx_au_add_nalu(
		self,
		self->nalu.buf,
		self->slice_copy.valid ? &self->slice_copy : NULL);
	if (res < 0)
		goto out;

	/* If it was the last NALU, complete the frame */
	if (self->nalu.last) {
		res = vstrm_rtp_h264_rx_au_complete(self);
		if (res < 0)
			goto out;
	}

out:
	/* Reset for next NALU */
	self->nalu.type = H264_NALU_TYPE_UNKNOWN;
	pomp_buffer_unref(self->nalu.buf);
	self->nalu.buf = NULL;
	return res;
}


static int vstrm_rtp_h264_rx_nalu_append(struct vstrm_rtp_h264_rx *self,
					 const uint8_t *buf,
					 size_t len)
{
	/* If no buffer, create one with the data */
	if (self->nalu.buf == NULL) {
		self->nalu.buf = pomp_buffer_new_with_data(buf, len);
		return self->nalu.buf == NULL ? -ENOMEM : 0;
	}

	/* Append to current buffer */
	return pomp_buffer_append_data(self->nalu.buf, buf, len);
}


static int vstrm_rtp_h264_rx_process_aggregation(struct vstrm_rtp_h264_rx *self,
						 uint8_t stap_ind,
						 uint16_t don,
						 const uint8_t *payloadbuf,
						 size_t payloadlen)
{
	int res = 0;
	uint16_t nalulen = 0;

	while (payloadlen >= 2) {
		nalulen = (payloadbuf[0] << 8) | payloadbuf[1];
		payloadbuf += 2;
		payloadlen -= 2;

		if (payloadlen < nalulen) {
			res = -EIO;
			ULOGW("rtp_h264: bad payload length: %zu (%u)",
			      payloadlen,
			      nalulen);
			goto out;
		}

		if (self->marker && payloadlen - nalulen < 2)
			self->nalu.last = 1;

		/* Append NALU data, and complete it */
		res = vstrm_rtp_h264_rx_nalu_append(self, payloadbuf, nalulen);
		if (res < 0)
			goto out;
		res = vstrm_rtp_h264_rx_nalu_complete(self);
		if (res < 0)
			goto out;

		payloadbuf += nalulen;
		payloadlen -= nalulen;
	}

	if (payloadlen != 0)
		ULOGW("rtp_h264: extra bytes in payload: %zu", payloadlen);

out:
	return res;
}


static int vstrm_rtp_h264_rx_process_fragment(struct vstrm_rtp_h264_rx *self,
					      uint8_t fu_ind,
					      uint8_t fu_hdr,
					      uint16_t don,
					      const uint8_t *payloadbuf,
					      size_t payloadlen)
{
	int res = 0;
	int start = (fu_hdr & 0x80) != 0;
	int end = (fu_hdr & 0x40) != 0;
	uint8_t nalu_header = 0;

	if (start) {
		/* Abort current fragment if needed */
		if (self->current_fu_type != 0) {
			ULOGD("rtp_h264: abort current fu");
			if (self->nalu.buf != NULL)
				pomp_buffer_set_len(self->nalu.buf, 0);
		}

		/* Add NALU header with type */
		self->current_fu_type = fu_ind & 0x1f;
		nalu_header = (fu_ind & 0xe0) | (fu_hdr & 0x1f);
		res = vstrm_rtp_h264_rx_nalu_append(self, &nalu_header, 1);
		if (res < 0)
			goto out;
	} else if (self->current_fu_type == 0) {
		/* We missed the start of the fragment */
		res = -EIO;
		ULOGD("rtp_h264: skip fu");
		goto out;
	}

	/* Add NALU data */
	res = vstrm_rtp_h264_rx_nalu_append(self, payloadbuf, payloadlen);
	if (res < 0)
		goto out;

	/* Complete NALU if it is the last fragment */
	if (end) {
		self->current_fu_type = 0;
		self->nalu.last = self->marker;
		res = vstrm_rtp_h264_rx_nalu_complete(self);
		if (res < 0)
			goto out;
	}

out:
	return res;
}


static inline int has_pack(struct vstrm_rtp_h264_rx *self, uint8_t pack_id)
{
	/* Never process packs >= 128 */
	if (pack_id < 64)
		return !!(self->metadata.pack_bf_low &
			  (UINT64_C(1) << pack_id));
	else if (pack_id < 128)
		return !!(self->metadata.pack_bf_high &
			  (UINT64_C(1) << (pack_id - 64)));
	else
		return 1;
}


static inline void set_pack(struct vstrm_rtp_h264_rx *self, uint8_t pack_id)
{
	/* Never set packs >= 128 */
	if (pack_id < 64)
		self->metadata.pack_bf_low |= (UINT64_C(1) << pack_id);
	else if (pack_id < 128)
		self->metadata.pack_bf_high |= (UINT64_C(1) << (pack_id - 64));
}


static inline int has_all_packs(struct vstrm_rtp_h264_rx *self,
				uint8_t last_pack_id)
{
	/* We cannot have more than 128 packs */
	if (last_pack_id >= 128)
		return 0;

	/* Only one bitfield to check */
	if (last_pack_id < 64) {
		uint64_t mask =
			(last_pack_id == 63)
				? UINT64_MAX
				: ((UINT64_C(1) << (last_pack_id + 1)) - 1);
		return (self->metadata.pack_bf_low & mask) == mask;
	}

	/* More than one, the low one must be full ones */
	if (self->metadata.pack_bf_low != UINT64_MAX)
		return 0;
	uint64_t mask = (last_pack_id == 127)
				? UINT64_MAX
				: ((UINT64_C(1) << (last_pack_id - 63)) - 1);
	return (self->metadata.pack_bf_high & mask) == mask;
}


static int vstrm_rtp_h264_rx_process_extheader(struct vstrm_rtp_h264_rx *self,
					       uint16_t id,
					       const uint8_t *extheaderbuf,
					       size_t extheaderlen)
{
	int res = 0;
	struct vmeta_buffer buf;
	const char *mime_type;

	if (id == VMETA_FRAME_PROTO_RTP_EXT_ID) {
		if (extheaderlen < VSTRM_METADATA_PROTO_HEADER_LEN)
			return -EPROTO;

		uint16_t last_cur_pad;
		memcpy(&last_cur_pad, &extheaderbuf[4], sizeof(last_cur_pad));
		uint8_t last_pack, cur_pack, padding;
		vstrm_rtp_h264_meta_header_unpack(
			last_cur_pad, &last_pack, &cur_pack, &padding);
		uint16_t offset;
		memcpy(&offset, &extheaderbuf[6], sizeof(offset));
		offset = ntohs(offset);

		if (has_pack(self, cur_pack))
			goto out;
		set_pack(self, cur_pack);

		size_t len = extheaderlen - padding -
			     VSTRM_METADATA_PROTO_HEADER_LEN;

		if (self->metadata.capacity < offset + len) {
			uint8_t *tmp =
				realloc(self->metadata.buf, offset + len);
			if (!tmp) {
				res = -ENOMEM;
				ULOG_ERRNO("realloc", -res);
				goto out;
			}
			self->metadata.buf = tmp;
			self->metadata.capacity = offset + len;
		}
		memcpy(&self->metadata.buf[offset],
		       extheaderbuf + VSTRM_METADATA_PROTO_HEADER_LEN,
		       len);
		if (cur_pack == last_pack)
			self->metadata.len = offset + len;

		if (!has_all_packs(self, last_pack))
			goto out;

		vmeta_buffer_set_cdata(
			&buf, self->metadata.buf, self->metadata.len, 0);
		mime_type = VMETA_FRAME_PROTO_MIME_TYPE;

	} else {
		vmeta_buffer_set_cdata(&buf, extheaderbuf, extheaderlen, 0);
		mime_type = NULL;
	}

	/* Read metadata */
	if (self->au.metadata) {
		vmeta_frame_unref(self->au.metadata);
		self->au.metadata = NULL;
	}
	res = vmeta_frame_read2(
		&buf,
		mime_type,
		!(CHECK_FLAG(self->cfg.flags,
			     DISABLE_VIDEO_METADATA_CONVERSION)),
		&self->au.metadata);
	if (res < 0) {
		ULOG_ERRNO("vmeta_frame_read2", -res);
		goto out;
	}

out:
	return res;
}


static const struct h264_ctx_cbs h264_cbs = {
	.nalu_begin = &vstrm_rtp_h264_rx_nalu_begin_cb,
	.slice = &vstrm_rtp_h264_rx_slice_cb,
	.slice_data_end = &vstrm_rtp_h264_rx_slice_data_end_cb,
	.slice_data_mb = &vstrm_rtp_h264_rx_slice_data_mb_cb,
	.sps = &vstrm_rtp_h264_rx_sps_cb,
	.pps = &vstrm_rtp_h264_rx_pps_cb,
	.sei_recovery_point = &vstrm_rtp_h264_rx_sei_recovery_point_cb,
	.sei_user_data_unregistered =
		&vstrm_rtp_h264_rx_sei_user_data_unregistered_cb,
};


int vstrm_rtp_h264_rx_new(const struct vstrm_rtp_h264_rx_cfg *cfg,
			  const struct vstrm_rtp_h264_rx_cbs *cbs,
			  struct vstrm_rtp_h264_rx **ret_obj)
{
	int res = 0;
	struct vstrm_rtp_h264_rx *self = NULL;
	const char *flag = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->codec_info_changed == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->recv_frame == NULL, EINVAL);

	*ret_obj = NULL;

	/* Allocate and initialize structure */
	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->cfg = *cfg;
	self->cbs = *cbs;
	self->codec_info.codec = VSTRM_CODEC_VIDEO_H264;
	self->au.first = 1;
	self->au.info.complete = 1;
	self->nalu.first = 1;

	self->metadata.buf = NULL;
	self->metadata.capacity = 0;

	/* Enable some flags via env. variable */
	flag = getenv("VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS");
	if (CHECK_ENV_FLAG(flag)) {
		ULOGI("enable H264_FULL_MB_STATUS via env");
		self->cfg.flags |= VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS;
	}

	/* Create the H.264 reader */
	res = h264_reader_new(&h264_cbs, self, &self->reader);
	if (res < 0)
		goto error;

	/* Setup video statistics */
	self->video_stats.version = VSTRM_VIDEO_STATS_VERSION_2;
	self->video_stats.mb_status_class_count =
		VSTRM_H264_MB_STATUS_CLASS_COUNT;
	self->video_stats.mb_status_zone_count =
		VSTRM_H264_MB_STATUS_ZONE_COUNT;
	res = vstrm_video_stats_dyn_init(
		&self->video_stats_dyn,
		self->video_stats.mb_status_class_count,
		self->video_stats.mb_status_zone_count);
	if (res < 0)
		goto error;
	self->err_sec_start_time_by_zone = calloc(
		self->video_stats.mb_status_zone_count, sizeof(uint64_t));
	if (self->err_sec_start_time_by_zone == NULL)
		goto error;

	*ret_obj = self;
	return 0;

	/* Cleanup in case of error */
error:
	vstrm_video_stats_dyn_clear(&self->video_stats_dyn);
	free(self->err_sec_start_time_by_zone);
	if (self->reader != NULL)
		h264_reader_destroy(self->reader);
	free(self);
	return res;
}


int vstrm_rtp_h264_rx_destroy(struct vstrm_rtp_h264_rx *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	h264_reader_destroy(self->reader);
	vstrm_video_stats_dyn_clear(&self->video_stats_dyn);
	free(self->err_sec_start_time_by_zone);
	if (self->au.metadata)
		vmeta_frame_unref(self->au.metadata);
	if (self->au.frame != NULL)
		vstrm_frame_unref(self->au.frame->base);
	free(self->au.info.mb_status);
	if (self->nalu.buf != NULL)
		pomp_buffer_unref(self->nalu.buf);
	free(self->metadata.buf);
	free(self->sps.buf);
	free(self->pps.buf);
	free(self);
	return 0;
}


int vstrm_rtp_h264_rx_clear(struct vstrm_rtp_h264_rx *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	self->current_fu_type = 0;
	self->gap = 0;

	memset(&self->codec_info, 0, sizeof(self->codec_info));
	self->codec_info.codec = VSTRM_CODEC_VIDEO_H264;
	self->sps.valid = 0;
	self->pps.valid = 0;

	if (self->au.metadata) {
		vmeta_frame_unref(self->au.metadata);
		self->au.metadata = NULL;
	}
	self->metadata.pack_bf_low = UINT64_C(0);
	self->metadata.pack_bf_high = UINT64_C(0);
	self->metadata.len = 0;

	free(self->au.info.mb_status);
	memset(&self->au.info, 0, sizeof(self->au.info));

	memset(&self->au.timestamp, 0, sizeof(self->au.timestamp));
	memset(&self->au.last_timestamp, 0, sizeof(self->au.last_timestamp));

	self->au.last_out_timestamp = 0;
	self->au.frame_num = 0;
	self->au.prev_frame_num = 0;
	self->au.first = 1;
	self->au.info.complete = 1;

	self->nalu.first = 1;
	self->nalu.last = 0;

	self->slice.valid = 0;
	self->prev_slice.valid = 0;

	self->info_v1.valid = 0;
	self->info_v2.valid = 0;
	self->info_v4.valid = 0;

	return 0;
}


int vstrm_rtp_h264_rx_process_packet(struct vstrm_rtp_h264_rx *self,
				     const struct rtp_pkt *pkt,
				     uint32_t gap,
				     const struct vstrm_timestamp *timestamp)
{
	int res = 0;
	uint8_t nalu_type = 0;
	const uint8_t *payloadbuf = NULL;
	size_t payloadlen = 0;
	uint16_t payloadtype = 0;
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(timestamp == NULL, EINVAL);

	payloadtype = RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, PAYLOAD_TYPE);
	if (payloadtype != VSTRM_RTP_H264_PAYLOAD_TYPE) {
		res = -EIO;
		ULOGW("rtp_h264: bad payload type: %u (%u)",
		      payloadtype,
		      VSTRM_RTP_H264_PAYLOAD_TYPE);
		goto out;
	}

	payloadbuf = pkt->raw.cdata + pkt->payload.off;
	payloadlen = pkt->payload.len;
	if (payloadlen < 1) {
		res = -EIO;
		ULOGW("rtp_h264: bad payload length: %zu (%u)", payloadlen, 1);
		goto out;
	}
	nalu_type = payloadbuf[0] & 0x1f;

	/* Do not clear gap if not handled yet (fragmented NALU for example) */
	if (!self->gap && gap != 0)
		self->gap = 1;

	/* Do we have a pending fragment? */
	if (self->current_fu_type != 0 &&
	    (gap != 0 || self->current_fu_type != nalu_type)) {
		/* Abort current fragment */
		ULOGD("rtp_h264: abort current fu");
		if (self->nalu.buf != NULL)
			pomp_buffer_set_len(self->nalu.buf, 0);
		self->current_fu_type = 0;
		self->gap = 1;
	}

	if (!self->nalu.first && self->au.timestamp.rtp != 0 &&
	    self->au.timestamp.rtp != pkt->rtp_timestamp) {
		ULOGD("rtp_h264: missing end of frame");
		self->gap = 1;
		vstrm_rtp_h264_rx_handle_missing_eof(self);
	}

	/* Process header extensions (ignore errors) */
	if (pkt->extheader.len != 0) {
		vstrm_rtp_h264_rx_process_extheader(self,
						    pkt->extheader.id,
						    pkt->raw.cdata +
							    pkt->extheader.off,
						    pkt->extheader.len);
	}

	if (self->nalu.first)
		self->au.timestamp = *timestamp;

	self->marker = RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, MARKER);

	switch (nalu_type) {
	case VSTRM_RTP_H264_NALU_TYPE_STAP_A:
		res = vstrm_rtp_h264_rx_process_aggregation(
			self, payloadbuf[0], 0, payloadbuf + 1, payloadlen - 1);
		break;

	case VSTRM_RTP_H264_NALU_TYPE_STAP_B:
		if (payloadlen < 3) {
			res = -EIO;
			ULOGW("rtp_h264: bad payload length: %zu (%u)",
			      payloadlen,
			      3);
			goto out;
		}
		res = vstrm_rtp_h264_rx_process_aggregation(
			self,
			payloadbuf[0],
			(payloadbuf[1] << 8) | payloadbuf[2],
			payloadbuf + 3,
			payloadlen - 3);
		break;

	case VSTRM_RTP_H264_NALU_TYPE_MTAP16: /* NO BREAK */
	case VSTRM_RTP_H264_NALU_TYPE_MTAP24:
		res = -EIO;
		ULOGW("rtp_h264: MTAP not supported");
		goto out;

	case VSTRM_RTP_H264_NALU_TYPE_FU_A:
		if (payloadlen < 2) {
			res = -EIO;
			ULOGW("rtp_h264: bad payload length: %zu (%u)",
			      payloadlen,
			      2);
			goto out;
		}
		res = vstrm_rtp_h264_rx_process_fragment(self,
							 payloadbuf[0],
							 payloadbuf[1],
							 0,
							 payloadbuf + 2,
							 payloadlen - 2);
		break;

	case VSTRM_RTP_H264_NALU_TYPE_FU_B:
		if (payloadlen < 4) {
			res = -EIO;
			ULOGW("rtp_h264: bad payload length: %zu (%u)",
			      payloadlen,
			      4);
			goto out;
		}
		res = vstrm_rtp_h264_rx_process_fragment(self,
							 payloadbuf[0],
							 payloadbuf[1],
							 (payloadbuf[2] << 8) |
								 payloadbuf[3],
							 payloadbuf + 4,
							 payloadlen - 4);
		break;

	default:
		self->nalu.last = self->marker;
		res = vstrm_rtp_h264_rx_nalu_append(
			self, payloadbuf, payloadlen);
		if (res < 0)
			goto out;
		res = vstrm_rtp_h264_rx_nalu_complete(self);
		break;
	}

out:
	return res;
}


int vstrm_rtp_h264_rx_get_video_stats(
	struct vstrm_rtp_h264_rx *self,
	const struct vstrm_video_stats **video_stats,
	const struct vstrm_video_stats_dyn **video_stats_dyn)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(video_stats == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(video_stats_dyn == NULL, EINVAL);
	*video_stats = &self->video_stats;
	*video_stats_dyn = &self->video_stats_dyn;
	return 0;
}


int vstrm_rtp_h264_rx_set_video_stats(
	struct vstrm_rtp_h264_rx *self,
	const struct vstrm_video_stats *video_stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(video_stats == NULL, EINVAL);

	if (self->video_stats.version != VSTRM_VIDEO_STATS_VERSION_2)
		return -EPROTO;

	/* The timestamp must be set by the caller to a monotonic timestamp
	 * on the sender's clock, e.g. a frame capture timestamp */
	if (video_stats->timestamp == 0)
		return 0;

	self->video_stats.timestamp = video_stats->timestamp;
	self->video_stats.v2.presentation_frame_count =
		video_stats->v2.presentation_frame_count;
	self->video_stats.v2.presentation_timestamp_delta_integral =
		video_stats->v2.presentation_timestamp_delta_integral;
	self->video_stats.v2.presentation_timestamp_delta_integral_sq =
		video_stats->v2.presentation_timestamp_delta_integral_sq;
	self->video_stats.v2.presentation_timing_error_integral =
		video_stats->v2.presentation_timing_error_integral;
	self->video_stats.v2.presentation_timing_error_integral_sq =
		video_stats->v2.presentation_timing_error_integral_sq;
	self->video_stats.v2.presentation_estimated_latency_integral =
		video_stats->v2.presentation_estimated_latency_integral;
	self->video_stats.v2.presentation_estimated_latency_integral_sq =
		video_stats->v2.presentation_estimated_latency_integral_sq;
	self->video_stats.v2.player_latency_integral =
		video_stats->v2.player_latency_integral;
	self->video_stats.v2.player_latency_integral_sq =
		video_stats->v2.player_latency_integral_sq;
	self->video_stats.v2.estimated_latency_precision_integral =
		video_stats->v2.estimated_latency_precision_integral;

	return 0;
}


int vstrm_rtp_h264_rx_get_codec_info(struct vstrm_rtp_h264_rx *self,
				     const struct vstrm_codec_info **info)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info == NULL, EINVAL);
	*info = &self->codec_info;
	return 0;
}


int vstrm_rtp_h264_rx_set_codec_info(struct vstrm_rtp_h264_rx *self,
				     const struct vstrm_codec_info *info)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info->codec != VSTRM_CODEC_VIDEO_H264, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info->h264.spslen == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info->h264.ppslen == 0, EINVAL);

	res = h264_reader_parse_nalu(
		self->reader, 0, info->h264.sps, info->h264.spslen);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse_nalu:SPS", -res);
		return res;
	}

	res = h264_reader_parse_nalu(
		self->reader, 0, info->h264.pps, info->h264.ppslen);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse_nalu:PPS", -res);
		return res;
	}

	vstrm_rtp_h264_rx_sps(self);
	vstrm_rtp_h264_rx_pps(self);
	return 0;
}
