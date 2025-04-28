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

/* Maximum number of P-SKIP concealment frames to generate. */
#define MAX_FRAME_CONCEALMENT (30)

/* Maximum number of reference frames kept for MB status. */
#define MAX_REF_FRAMES (4)

/* Debugging */
/* #define DEBUG_MB_STATUS_DUMP */
/* #define DEBUG_RPLM_DRPM */
/* #define DEBUG_REF_FRAMES_DOT */


struct vstrm_rtp_h264_rx_frame {
	struct vstrm_frame *base;
	unsigned int index;
	bool recovery_point;
	bool idr;
	bool ltr;
	unsigned int ltr_pic_num;
	bool ltr_reset;
	unsigned int ltr_reset_pic_num;
	bool ref_set;
	bool ref_frame_available;
	unsigned int ref_frame_index;
	const uint8_t *ref_mb_status;
	unsigned int used_ltr_pic_num;
	int layer;
};


struct vstrm_rtp_h264_rx_slice {
	bool valid;
	struct h264_nalu_header nalu_header;
	struct h264_slice_header slice_header;
	const uint8_t *buf;
	size_t len;
	uint32_t mb_count;
	enum vstrm_frame_mb_status mb_status;
	bool ltr;
	unsigned int ltr_pic_num;
	bool ltr_reset;
	unsigned int ltr_reset_pic_num;
	bool uses_ltr;
	unsigned int used_ltr_pic_num;
	int ref_frame_delta;
};


struct vstrm_rtp_h264_rx_ref_element {
	bool available;
	unsigned int frame_index;
	unsigned int ltr_pic_num;
	uint8_t *mb_status;
};


struct vstrm_rtp_h264_rx {
	struct vstrm_rtp_h264_rx_cfg cfg;
	struct vstrm_rtp_h264_rx_cbs cbs;
	uint8_t current_fu_type;
	bool gap;
	bool marker;
	struct h264_reader *reader;
	struct vstrm_video_stats video_stats;
	struct vstrm_video_stats_dyn video_stats_dyn;
	uint64_t err_sec_start_time;
	uint64_t *err_sec_start_time_by_zone;
	uint32_t max_frame_num;
	uint32_t max_pic_order_cnt_lsb;
	struct vstrm_codec_info codec_info;
	struct vstrm_rtp_h264_rx_frame *current_frame;
	struct vstrm_timestamp current_timestamps;
	struct vstrm_timestamp last_timestamps;

	bool max_num_ref_logged;
	bool max_num_rplm_logged;

	struct {
		/* Short-term references (note: avoiding the 'short' keyword) */
		struct vstrm_rtp_h264_rx_ref_element shrt[MAX_REF_FRAMES];
		int shrt_index;

		/* Long-term reference (note: avoiding the 'long' keyword) */
		struct vstrm_rtp_h264_rx_ref_element lng;
	} ref;

	struct {
		bool first;
		unsigned int index;
		uint32_t frame_num;
		uint32_t prev_frame_num;
		uint32_t pic_order_count_lsb;
		uint32_t prev_pic_order_count_lsb;
	} au;

	struct {
		enum h264_nalu_type type;
		struct pomp_buffer *buf;
		bool first;
		bool last;
		bool recovery_point;
		bool idr;
	} nalu;

	struct {
		bool valid;
		struct h264_sps sps;
		struct h264_sps_derived sps_derived;
		uint8_t *buf;
		size_t len;
		size_t maxsize;
		uint32_t mb_width;
		uint32_t mb_height;
		uint32_t mb_total;
	} sps;

	struct {
		bool valid;
		struct h264_pps pps;
		uint8_t *buf;
		size_t len;
		size_t maxsize;
	} pps;

	bool update_mb_status;
	struct vstrm_rtp_h264_rx_slice slice;
	struct vstrm_rtp_h264_rx_slice slice_copy;
	struct vstrm_rtp_h264_rx_slice prev_slice;
	struct vstrm_rtp_h264_rx_slice tmp_slice;
	struct h264_slice_header tmp_slice_header;

	struct {
		bool valid;
		struct vstrm_h264_sei_streaming_v1 sei;
	} info_v1;

	struct {
		bool valid;
		struct vstrm_h264_sei_streaming_v2 sei;
	} info_v2;

	struct {
		bool valid;
		struct vstrm_h264_sei_streaming_v4 sei;
	} info_v4;

	struct {
		uint64_t pack_bf_low;
		uint64_t pack_bf_high;
		uint8_t *buf;
		size_t len;
		size_t capacity;
		struct vmeta_frame *meta;
	} metadata;

#ifdef DEBUG_REF_FRAMES_DOT
	FILE *dot_file;
#endif
};


static void dot_clear(struct vstrm_rtp_h264_rx *self)
{
#ifdef DEBUG_REF_FRAMES_DOT
	if (self->dot_file == NULL)
		return;

	fprintf(self->dot_file, "}");
	fclose(self->dot_file);
	self->dot_file = NULL;
#endif
}


static void dot_init(struct vstrm_rtp_h264_rx *self)
{
#ifdef DEBUG_REF_FRAMES_DOT
	if (self->dot_file != NULL)
		dot_clear(self);

	self->dot_file = fopen("ref.dot", "w");
	if (self->dot_file == NULL) {
		int err = -errno;
		ULOG_ERRNO("fopen", -err);
		return;
	}

	fprintf(self->dot_file, "digraph {\n");
	fprintf(self->dot_file, "\tnode [style=filled]\n");
	fprintf(self->dot_file, "\t\"UNDEF\" [fillcolor=firebrick1]\n");
#endif
}


static void dot_add_frame(struct vstrm_rtp_h264_rx *self,
			  struct vstrm_rtp_h264_rx_frame *frame)
{
#ifdef DEBUG_REF_FRAMES_DOT
	if (self->dot_file == NULL)
		return;

	if (frame->idr) {
		fprintf(self->dot_file,
			"\t\"#%u\" [fillcolor=%s]\n",
			frame->index,
			frame->base->info.gen_grey_idr ? "darkgray" : "orchid");
	} else {
		fprintf(self->dot_file,
			"\t\"#%u\" [colorscheme=%s fillcolor=%d]\n",
			frame->index,
			frame->base->info.gen_concealment ? "reds5"
			: frame->base->info.error	  ? "blues5"
							  : "greens5",
			frame->recovery_point ? 5 : 4 - frame->layer);
	}
	if (frame->ref_frame_available) {
		fprintf(self->dot_file,
			"\t\"#%u\" -> \"#%u\"\n",
			frame->index,
			frame->ref_frame_index);
	} else if (!frame->idr) {
		fprintf(self->dot_file,
			"\t\"#%u\" -> \"UNDEF\"\n",
			frame->index);
	}
#endif
}


static void ref_clear(struct vstrm_rtp_h264_rx *self)
{
	for (unsigned int i = 0; i < MAX_REF_FRAMES; i++) {
		free(self->ref.shrt[i].mb_status);
		self->ref.shrt[i].mb_status = NULL;
		self->ref.shrt[i].frame_index = 0;
		self->ref.shrt[i].ltr_pic_num = 0;
		self->ref.shrt[i].available = false;
	}
	self->ref.shrt_index = 0;

	free(self->ref.lng.mb_status);
	self->ref.lng.mb_status = NULL;
	self->ref.lng.available = false;
	self->ref.lng.ltr_pic_num = 0;
}


static int ref_init(struct vstrm_rtp_h264_rx *self)
{
	ref_clear(self);

	for (unsigned int i = 0; i < MAX_REF_FRAMES; i++) {
		self->ref.shrt[i].mb_status = calloc(1, self->sps.mb_total);
		if (self->ref.shrt[i].mb_status == NULL) {
			int err = -ENOMEM;
			ULOG_ERRNO("calloc", -err);
			return err;
		}
	}

	self->ref.lng.mb_status = calloc(1, self->sps.mb_total);
	if (self->ref.lng.mb_status == NULL) {
		int err = -ENOMEM;
		ULOG_ERRNO("calloc", -err);
		return err;
	}

	return 0;
}


static void ref_short_inc_and_copy(struct vstrm_rtp_h264_rx *self,
				   unsigned int frame_index,
				   const uint8_t *mb_status)
{
	if (mb_status == NULL)
		return;

	self->ref.shrt_index++;
	if (self->ref.shrt_index >= MAX_REF_FRAMES)
		self->ref.shrt_index = 0;

	if (self->ref.shrt[self->ref.shrt_index].mb_status == NULL)
		return;

	memcpy(self->ref.shrt[self->ref.shrt_index].mb_status,
	       mb_status,
	       self->sps.mb_total);
	self->ref.shrt[self->ref.shrt_index].available = true;
	self->ref.shrt[self->ref.shrt_index].frame_index = frame_index;
}


static int ref_short_get_at_delta(struct vstrm_rtp_h264_rx *self,
				  int delta,
				  unsigned int *frame_index,
				  const uint8_t **mb_status)
{
	if ((delta >= MAX_REF_FRAMES) || (delta <= -MAX_REF_FRAMES))
		return -EINVAL;

	int idx = (self->ref.shrt_index + MAX_REF_FRAMES + delta) %
		  MAX_REF_FRAMES;

	if (self->ref.shrt[idx].mb_status == NULL)
		return -ENOMEM;
	if (!self->ref.shrt[idx].available)
		return -ENOENT;

	if (frame_index)
		*frame_index = self->ref.shrt[idx].frame_index;
	if (mb_status)
		*mb_status = self->ref.shrt[idx].mb_status;

	return 0;
}


static void ref_long_reset(struct vstrm_rtp_h264_rx *self,
			   unsigned int ltr_pic_num)
{
	if (self->ref.lng.ltr_pic_num != ltr_pic_num)
		return;
	self->ref.lng.available = false;
	self->ref.lng.frame_index = 0;
	self->ref.lng.ltr_pic_num = 0;
}


static void ref_long_set_and_copy(struct vstrm_rtp_h264_rx *self,
				  unsigned int frame_index,
				  unsigned int ltr_pic_num,
				  const uint8_t *mb_status)
{
	if (mb_status == NULL)
		return;

	if (self->ref.lng.mb_status == NULL)
		return;

	memcpy(self->ref.lng.mb_status, mb_status, self->sps.mb_total);
	self->ref.lng.available = true;
	self->ref.lng.frame_index = frame_index;
	self->ref.lng.ltr_pic_num = ltr_pic_num;
}


static int ref_long_get(struct vstrm_rtp_h264_rx *self,
			unsigned int ltr_pic_num,
			unsigned int *frame_index,
			const uint8_t **mb_status)
{
	if (self->ref.lng.mb_status == NULL)
		return -ENOMEM;
	if (!self->ref.lng.available ||
	    self->ref.lng.ltr_pic_num != ltr_pic_num)
		return -ENOENT;

	if (frame_index)
		*frame_index = self->ref.lng.frame_index;
	if (mb_status)
		*mb_status = self->ref.lng.mb_status;

	return 0;
}


static void frame_set_mb_status(struct vstrm_rtp_h264_rx_frame *frame,
				uint32_t mb_start,
				uint32_t mb_count,
				enum vstrm_frame_mb_status status)
{
	if (frame->base->info.mb_status == NULL)
		return;

	uint8_t *mb_status = frame->base->info.mb_status;

	/* Make sure range is valid */
	if (mb_start > frame->base->info.mb_total ||
	    mb_start + mb_count > frame->base->info.mb_total) {
		ULOGE("rtp_h264: invalid macroblock range: %u,%u (%ux%u)",
		      mb_start,
		      mb_count,
		      frame->base->info.mb_width,
		      frame->base->info.mb_height);
		return;
	}

	if (status == VSTRM_FRAME_MB_STATUS_VALID_PSLICE) {
		enum vstrm_frame_mb_status err_prop =
			VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION;
		/* We need to check the reference frame status */
		if (frame->ref_mb_status == NULL) {
			/* No reference available -> error propagation */
			memset(mb_status + mb_start, err_prop & 0xff, mb_count);
			frame->base->info.error = 1;
		} else {
			for (uint32_t i = mb_start; i < mb_start + mb_count;
			     i++) {
				switch (frame->ref_mb_status[i]) {
				case VSTRM_FRAME_MB_STATUS_VALID_ISLICE:
				case VSTRM_FRAME_MB_STATUS_VALID_PSLICE:
					/* OK, we can set the status */
					mb_status[i] = status;
					break;
				default:
					/* Error propagation */
					mb_status[i] = err_prop;
					frame->base->info.error = 1;
					break;
				}
			}
		}
	} else {
		/* Set status directly */
		memset(mb_status + mb_start, status & 0xff, mb_count);
		frame->base->info.error |=
			(status != VSTRM_FRAME_MB_STATUS_VALID_ISLICE &&
			 status != VSTRM_FRAME_MB_STATUS_VALID_PSLICE);
	}
}


static void frame_dump_mb_status(struct vstrm_rtp_h264_rx_frame *frame)
{
#ifdef DEBUG_MB_STATUS_DUMP
	if (frame->base->info.mb_status == NULL)
		return;

	uint8_t *mb_status = frame->base->info.mb_status;
	ULOGI("===== Frame #%u =====", frame->index);

	/* TODO: avoid runtime stack allocation */
	char row[frame->base->info.mb_width + 1];
	for (uint32_t j = 0; j < frame->base->info.mb_height; j++) {
		for (uint32_t i = 0; i < frame->base->info.mb_width; i++) {
			switch (mb_status[j * frame->base->info.mb_width + i]) {
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
			case VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_PSLICE:
				row[i] = '-';
				break;
			case VSTRM_FRAME_MB_STATUS_MISSING:
				row[i] = 'x';
				break;
			case VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION:
				row[i] = 'e';
				break;
			case VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_ISLICE:
				row[i] = '#';
				break;
			}
		}
		row[frame->base->info.mb_width] = '\0';
		ULOGI("|%s|", row);
	}
#endif
}


static void frame_dispose(struct vstrm_frame *base)
{
	/* Release all buffers associated with NAL units */
	for (uint32_t i = 0; i < base->nalu_count; i++) {
		struct pomp_buffer *buf = base->nalus[i].userdata;
		pomp_buffer_unref(buf);
	}

	/* Release macroblock status */
	free(base->info.mb_status);
}


static int frame_new(struct vstrm_rtp_h264_rx *rx,
		     unsigned int index,
		     struct vstrm_rtp_h264_rx_frame **ret_obj)
{
	int res = 0;
	struct vstrm_frame_ops ops;
	struct vstrm_frame *base = NULL;
	struct vstrm_rtp_h264_rx_frame *self = NULL;

	/* Create frame structure */
	memset(&ops, 0, sizeof(ops));
	ops.dispose = &frame_dispose;
	res = vstrm_frame_new(&ops, sizeof(*self), &base);
	if (res < 0)
		return res;

	/* Link pointers */
	self = base->userdata;
	self->base = base;

	self->index = index;
	self->base->info.complete = 1;

	/* Alloc and init macroblock status to unknown */
	if (rx->sps.mb_total != 0) {
		self->base->info.mb_width = rx->sps.mb_width;
		self->base->info.mb_height = rx->sps.mb_height;
		self->base->info.mb_total = rx->sps.mb_total;
		self->base->info.mb_status = malloc(rx->sps.mb_total);
		if (self->base->info.mb_status != NULL) {
			memset(self->base->info.mb_status,
			       VSTRM_FRAME_MB_STATUS_UNKNOWN,
			       rx->sps.mb_total);
		}
	}

	*ret_obj = self;
	return 0;
}


static void frame_update(struct vstrm_rtp_h264_rx *self,
			 struct vstrm_rtp_h264_rx_frame *frame,
			 const struct vstrm_rtp_h264_rx_slice *cur_slice)
{
	if (cur_slice == NULL)
		return;

	frame->ltr = cur_slice->ltr;
	frame->ltr_pic_num = cur_slice->ltr_pic_num;
	frame->ltr_reset = cur_slice->ltr_reset;
	frame->ltr_reset_pic_num = cur_slice->ltr_reset_pic_num;
	frame->base->info.uses_ltr = cur_slice->uses_ltr;
	frame->used_ltr_pic_num = cur_slice->used_ltr_pic_num;
	frame->base->info.ref = (cur_slice->nalu_header.nal_ref_idc != 0);
	if (!frame->ref_set) {
		if (frame->idr) {
			frame->layer = 0;
		} else if (frame->base->info.uses_ltr) {
			int err = ref_long_get(self,
					       frame->used_ltr_pic_num,
					       &frame->ref_frame_index,
					       &frame->ref_mb_status);
			if (err < 0 && err != -ENOENT)
				ULOG_ERRNO("ref_long_get", -err);
			frame->ref_frame_available = (err == 0);
			frame->layer = (frame->ltr) ? 0 : 1;
		} else {
			int err = ref_short_get_at_delta(
				self,
				cur_slice->ref_frame_delta,
				&frame->ref_frame_index,
				&frame->ref_mb_status);
			if (err < 0 && err != -ENOENT)
				ULOG_ERRNO("ref_long_get", -err);
			frame->ref_frame_available = (err == 0);
			frame->layer = (cur_slice->ref_frame_delta < 0) ? 0 : 1;
		}
		frame->ref_set = true;
#ifdef DEBUG_RPLM_DRPM
		ULOGI("frame #%u idr=%d ltr=%d(%u) ltr_reset=%d(%u) "
		      "uses_ltr=%d(%u) ref_frame_index=%u ref=%d",
		      frame->index,
		      frame->idr,
		      frame->ltr,
		      frame->ltr_pic_num,
		      frame->ltr_reset,
		      frame->ltr_reset_pic_num,
		      frame->base->info.uses_ltr,
		      frame->used_ltr_pic_num,
		      frame->ref_frame_index,
		      frame->base->info.ref);
#endif
	}
}


static int frame_add_nalu(struct vstrm_rtp_h264_rx_frame *frame,
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
	res = vstrm_frame_add_nalu(frame->base, &nalu);
	if (res < 0)
		goto out;

	/* Now that the NALU is in the frame, add a ref on the buffer */
	pomp_buffer_ref(buf);

out:
	return res;
}


static void nalu_begin_cb(struct h264_ctx *ctx,
			  enum h264_nalu_type type,
			  const uint8_t *buf,
			  size_t len,
			  const struct h264_nalu_header *nh,
			  void *userdata)
{
}


static void slice_cb(struct h264_ctx *ctx,
		     const uint8_t *buf,
		     size_t len,
		     const struct h264_slice_header *sh,
		     void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	struct h264_nalu_header nh = {0};
	int err;

	/* Save header */
	self->slice.valid = true;
	self->slice.slice_header = *sh;

	/* We can store pointers, we actually own the data stored in the
	 * NALU buffer */
	self->slice.buf = buf;
	self->slice.len = len;

	err = h264_parse_nalu_header(buf, len, &nh);
	if (err < 0)
		ULOG_ERRNO("h264_parse_nalu_header", -err);

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
		if (nh.nal_ref_idc == 3) {
			self->slice.mb_status =
				VSTRM_FRAME_MB_STATUS_VALID_ISLICE;
		}
		break;
	default:
		self->slice.mb_status = VSTRM_FRAME_MB_STATUS_UNKNOWN;
		break;
	}

	if (nh.nal_unit_type == H264_NALU_TYPE_SLICE_IDR) {
		if (sh->drpm.long_term_reference_flag) {
			self->slice.ltr = true;
			self->slice.ltr_pic_num = 0;
		}
#ifdef DEBUG_RPLM_DRPM
		ULOGI("IDR no_output_of_prior_pics_flag=%d "
		      "long_term_reference_flag=%d",
		      sh->drpm.no_output_of_prior_pics_flag,
		      sh->drpm.long_term_reference_flag);
#endif
	} else {
		int num_ref =
			self->pps.pps.num_ref_idx_l0_default_active_minus1 + 1;
		if (sh->num_ref_idx_active_override_flag)
			num_ref = sh->num_ref_idx_l0_active_minus1 + 1;
		if (num_ref > 1 && !self->max_num_ref_logged) {
			ULOGW("unsupported number of reference "
			      "pictures: %d > 1 (logged only once)",
			      num_ref);
			self->max_num_ref_logged = true;
		}
#ifdef DEBUG_RPLM_DRPM
		ULOGI("num_ref=%d ref_pic_list_modification_flag_l0=%d "
		      "adaptive_ref_pic_marking_mode_flag=%d",
		      num_ref,
		      sh->rplm.ref_pic_list_modification_flag_l0,
		      sh->drpm.adaptive_ref_pic_marking_mode_flag);
#endif
	}
	if (sh->rplm.ref_pic_list_modification_flag_l0) {
		for (int i = 0;
		     sh->rplm.pic_num_l0[i].modification_of_pic_nums_idc != 3;
		     i++) {
			if (i > 0) {
				if (!self->max_num_rplm_logged) {
					ULOGW("unsupported number of RPLM "
					      "entries: %d > 1 "
					      "(logged only once)",
					      i + 1);
					self->max_num_rplm_logged = true;
				}
				break;
			}
			switch (sh->rplm.pic_num_l0[i]
					.modification_of_pic_nums_idc) {
			case 0:
				self->slice.ref_frame_delta =
					-(int)sh->rplm.pic_num_l0[i]
						 .abs_diff_pic_num_minus1;
#ifdef DEBUG_RPLM_DRPM
				ULOGI("rplm_l0 [%d] "
				      "modification_of_pic_nums_idc=%u "
				      "abs_diff_pic_num_minus1=%u",
				      i,
				      sh->rplm.pic_num_l0[i]
					      .modification_of_pic_nums_idc,
				      sh->rplm.pic_num_l0[i]
					      .abs_diff_pic_num_minus1);
#endif
				break;
			case 1:
				self->slice.ref_frame_delta =
					(int)sh->rplm.pic_num_l0[i]
						.abs_diff_pic_num_minus1;
#ifdef DEBUG_RPLM_DRPM
				ULOGI("rplm_l0 [%d] "
				      "modification_of_pic_nums_idc=%u "
				      "abs_diff_pic_num_minus1=%u",
				      i,
				      sh->rplm.pic_num_l0[i]
					      .modification_of_pic_nums_idc,
				      sh->rplm.pic_num_l0[i]
					      .abs_diff_pic_num_minus1);
#endif
				break;
			case 2:
				self->slice.uses_ltr = true;
				self->slice.used_ltr_pic_num =
					sh->rplm.pic_num_l0[i]
						.long_term_pic_num;
#ifdef DEBUG_RPLM_DRPM
				ULOGI("rplm_l0 [%d] "
				      "modification_of_pic_nums_idc=%u "
				      "long_term_pic_num=%u",
				      i,
				      sh->rplm.pic_num_l0[i]
					      .modification_of_pic_nums_idc,
				      sh->rplm.pic_num_l0[i].long_term_pic_num);
#endif
				break;
			default:
				break;
			}
		}
	}
	if (sh->drpm.adaptive_ref_pic_marking_mode_flag) {
		for (int i = 0;
		     sh->drpm.mm[i].memory_management_control_operation != 0;
		     i++) {
			uint32_t op =
				sh->drpm.mm[i]
					.memory_management_control_operation;
			switch (op) {
			case 1:
#ifdef DEBUG_RPLM_DRPM
				ULOGI("drpm [%d] "
				      "memory_management_control_operation=%u "
				      "difference_of_pic_nums_minus1=%u",
				      i,
				      op,
				      sh->drpm.mm[i]
					      .difference_of_pic_nums_minus1);
#endif
				break;
			case 2:
				self->slice.ltr_reset = true;
				self->slice.ltr_reset_pic_num =
					sh->drpm.mm[i].long_term_pic_num;
#ifdef DEBUG_RPLM_DRPM
				ULOGI("drpm [%d] "
				      "memory_management_control_operation=%u "
				      "long_term_pic_num=%u",
				      i,
				      op,
				      sh->drpm.mm[i].long_term_pic_num);
#endif
				break;
			case 3:
#ifdef DEBUG_RPLM_DRPM
				ULOGI("drpm [%d] "
				      "memory_management_control_operation=%u "
				      "difference_of_pic_nums_minus1=%u "
				      "long_term_frame_idx=%u",
				      i,
				      op,
				      sh->drpm.mm[i]
					      .difference_of_pic_nums_minus1,
				      sh->drpm.mm[i].long_term_frame_idx);
#endif
				break;
			case 4:
#ifdef DEBUG_RPLM_DRPM
				ULOGI("drpm [%d] "
				      "memory_management_control_operation=%u "
				      "max_long_term_frame_idx_plus1=%u",
				      i,
				      op,
				      sh->drpm.mm[i]
					      .max_long_term_frame_idx_plus1);
#endif
				break;
			case 6:
				self->slice.ltr = true;
				self->slice.ltr_pic_num =
					sh->drpm.mm[i].long_term_frame_idx;
#ifdef DEBUG_RPLM_DRPM
				ULOGI("drpm [%d] "
				      "memory_management_control_operation=%u "
				      "long_term_frame_idx=%u",
				      i,
				      op,
				      sh->drpm.mm[i].long_term_frame_idx);
#endif
				break;
			case 5:
			default:
#ifdef DEBUG_RPLM_DRPM
				ULOGI("drpm [%d] "
				      "memory_management_control_operation=%u",
				      i,
				      op);
#endif
				break;
			}
		}
	}
}


static void slice_data_end_cb(struct h264_ctx *ctx,
			      const struct h264_slice_header *sh,
			      uint32_t mb_count,
			      void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	self->slice.mb_count = mb_count;
}


static void slice_data_mb_cb(struct h264_ctx *ctx,
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
		frame_set_mb_status(self->current_frame, mb_addr, 1, status);
	}
}


static void sps_cb(struct h264_ctx *ctx,
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
	self->sps.valid = true;
	self->sps.sps = *sps;
	memcpy(self->sps.buf, buf, len);
	self->sps.len = len;

	/* Get derived information */
	h264_get_sps_derived(sps, &self->sps.sps_derived);
}


static void pps_cb(struct h264_ctx *ctx,
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
	self->pps.valid = true;
	self->pps.pps = *pps;
	memcpy(self->pps.buf, buf, len);
	self->pps.len = len;
}


static void sei_recovery_point_cb(struct h264_ctx *ctx,
				  const uint8_t *buf,
				  size_t len,
				  const struct h264_sei_recovery_point *sei,
				  void *userdata)
{
	struct vstrm_rtp_h264_rx *self = userdata;
	self->nalu.recovery_point = true;
}


static void
sei_user_data_unregistered_cb(struct h264_ctx *ctx,
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
			self->info_v1.valid = true;
	} else if (vstrm_h264_sei_streaming_is_v2(sei->uuid)) {
		res = vstrm_h264_sei_streaming_v2_read(
			&self->info_v2.sei, sei->uuid, sei->buf, sei->len);
		if (res < 0)
			ULOG_ERRNO("vstrm_h264_sei_streaming_v2_read", -res);
		else
			self->info_v2.valid = true;
	} else if (vstrm_h264_sei_streaming_is_v4(sei->uuid)) {
		res = vstrm_h264_sei_streaming_v4_read(
			&self->info_v4.sei, sei->uuid, sei->buf, sei->len);
		if (res < 0)
			ULOG_ERRNO("vstrm_h264_sei_streaming_v4_read", -res);
		else
			self->info_v4.valid = true;
	}
}


static const struct h264_ctx_cbs h264_cbs = {
	.nalu_begin = &nalu_begin_cb,
	.slice = &slice_cb,
	.slice_data_end = &slice_data_end_cb,
	.slice_data_mb = &slice_data_mb_cb,
	.sps = &sps_cb,
	.pps = &pps_cb,
	.sei_recovery_point = &sei_recovery_point_cb,
	.sei_user_data_unregistered = &sei_user_data_unregistered_cb,
};


static int generate_grey_i_slice(struct vstrm_rtp_h264_rx *self,
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
	new_slice->valid = true;
	new_slice->nalu_header = nh;
	new_slice->slice_header = self->tmp_slice_header;
	new_slice->buf = bs.data;
	new_slice->len = bs.off;
	new_slice->mb_count = mb_count;
	new_slice->mb_status = VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_ISLICE;

out:
	h264_bs_clear(&bs);
	if (res != 0 && *buf != NULL) {
		pomp_buffer_unref(*buf);
		*buf = NULL;
	}
	return res;
}


static int generate_skipped_p_slice(struct vstrm_rtp_h264_rx *self,
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
	new_slice->valid = true;
	new_slice->nalu_header = nh;
	new_slice->slice_header = self->tmp_slice_header;
	new_slice->buf = bs.data;
	new_slice->len = bs.off;
	new_slice->mb_count = mb_count;
	new_slice->mb_status = VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_PSLICE;

out:
	h264_bs_clear(&bs);
	if (res != 0 && *buf != NULL) {
		pomp_buffer_unref(*buf);
		*buf = NULL;
	}
	return res;
}


static void update_err_sec_stats(struct vstrm_rtp_h264_rx *self,
				 uint64_t frame_ts,
				 struct vstrm_video_stats_dyn *dyn,
				 uint32_t zone)
{
	if ((self->err_sec_start_time == UINT64_MAX) ||
	    (frame_ts > self->err_sec_start_time + VSTRM_USECS_PER_SEC)) {
		self->err_sec_start_time = frame_ts;
		self->video_stats.v2.errored_second_count++;
	}
	if ((self->err_sec_start_time_by_zone[zone] == UINT64_MAX) ||
	    (frame_ts >
	     self->err_sec_start_time_by_zone[zone] + VSTRM_USECS_PER_SEC)) {
		self->err_sec_start_time_by_zone[zone] = frame_ts;
		vstrm_video_stats_dyn_inc_errored_second_count_by_zone(dyn,
								       zone);
	}
}


static void update_mb_status_stats(struct vstrm_rtp_h264_rx *self,
				   struct vstrm_rtp_h264_rx_frame *frame)
{
	struct vstrm_video_stats_dyn *dyn = &self->video_stats_dyn;
	for (uint32_t j = 0, k = 0; j < self->sps.mb_height; j++) {
		for (uint32_t i = 0; i < self->sps.mb_width; i++, k++) {
			uint32_t zone = j * dyn->mb_status_zone_count /
					self->sps.mb_height;
			uint8_t status =
				(frame) ? frame->base->info.mb_status[k]
					: VSTRM_FRAME_MB_STATUS_UNKNOWN;
			vstrm_video_stats_dyn_inc_mb_status_count(
				dyn, status, zone);
			if (status != VSTRM_FRAME_MB_STATUS_VALID_ISLICE &&
			    status != VSTRM_FRAME_MB_STATUS_VALID_PSLICE) {
				update_err_sec_stats(
					self,
					(frame) ? frame->base->timestamps
							  .ntp_raw
						: self->last_timestamps.ntp_raw,
					dyn,
					zone);
			}
		}
	}
}


static void map_timestamps(const struct vstrm_timestamp *src,
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


static inline uint64_t
interpolate_ts(uint64_t start_ts, uint64_t end_ts, size_t i, size_t count)
{
	return end_ts - (count - i) * (end_ts - start_ts) / (count + 1);
}


static int interpolate_timestamps(const struct vstrm_timestamp *start,
				  const struct vstrm_timestamp *end,
				  size_t i,
				  size_t count,
				  struct vstrm_frame_timestamps *dst)
{
	if (start == NULL || end == NULL || dst == NULL)
		return -EINVAL;
	if (count == 0 || i >= count)
		return -EINVAL;

	dst->ntp = interpolate_ts(start->ntp, end->ntp, i, count);
	dst->ntp_unskewed = interpolate_ts(
		start->ntp_unskewed, end->ntp_unskewed, i, count);
	dst->ntp_raw = interpolate_ts(start->ntp_raw, end->ntp_raw, i, count);
	dst->ntp_raw_unskewed = interpolate_ts(
		start->ntp_raw_unskewed, end->ntp_raw_unskewed, i, count);
	dst->local = interpolate_ts(start->ntp_local, end->ntp_local, i, count);
	dst->recv_start = interpolate_ts(start->input, end->input, i, count);
	dst->recv_end = dst->recv_start;

	return 0;
}


static void au_output(struct vstrm_rtp_h264_rx *self,
		      struct vstrm_rtp_h264_rx_frame *frame)
{
	/* Reference frames management */
	if (frame->ltr_reset)
		ref_long_reset(self, frame->ltr_reset_pic_num);
	if (frame->base->info.ref) {
		if (frame->ltr) {
			ref_long_set_and_copy(self,
					      frame->index,
					      frame->ltr_pic_num,
					      frame->base->info.mb_status);
		}
		ref_short_inc_and_copy(
			self, frame->index, frame->base->info.mb_status);
	} else {
		/* Assume layer == 2 for non-ref frames (we have no way to
		 * know the total layer count on the receiver side) */
		frame->layer = 2;
	}
	dot_add_frame(self, frame);

	/* Update and copy video statistics */
	update_mb_status_stats(self, frame);
	frame_dump_mb_status(frame);
	self->video_stats.v2.total_frame_count++;
	self->video_stats.v2.output_frame_count++;
	if (frame->base->info.error)
		self->video_stats.v2.errored_output_frame_count++;
	frame->base->video_stats = self->video_stats;
	/* Note: this should be a frame capture timestamp on the sender's
	 * monotonic clock, but we don't have this information here */
	frame->base->video_stats.timestamp = 0;
	vstrm_video_stats_dyn_copy(&frame->base->video_stats_dyn,
				   &self->video_stats_dyn);

	if (!frame->base->info.complete &&
	    CHECK_FLAG(self->cfg.flags, H264_GEN_CONCEALMENT_SLICE)) {
		ULOGW("rtp_h264: incomplete frame");
	}

	/* Notify upper layer */
	(*self->cbs.recv_frame)(self, frame->base, self->cbs.userdata);
}


static int au_complete(struct vstrm_rtp_h264_rx *self)
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


	if (self->current_frame == NULL) {
		/* No data in frame */
		self->video_stats.v2.total_frame_count++;
		self->video_stats.v2.discarded_frame_count++;
		self->video_stats.v2.missed_frame_count++;
		goto out;
	}
	ref = self->current_frame->base->info.ref;

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &out_timestamp);

	/* Copy timestamp and metadata */
	map_timestamps(&self->current_timestamps,
		       out_timestamp,
		       &self->current_frame->base->timestamps);
	self->current_frame->base->metadata = self->metadata.meta;
	if (self->current_frame->base->metadata)
		vmeta_frame_ref(self->current_frame->base->metadata);

	/* Update timing statistics */
	self->last_timestamps = self->current_timestamps;

	/* Update macroblock status of last slice
	 * (Not needed in FULL_MB_STATUS) */
	if (self->prev_slice.valid &&
	    !CHECK_FLAG(self->cfg.flags, H264_FULL_MB_STATUS)) {
		uint32_t mb_start =
			self->prev_slice.slice_header.first_mb_in_slice;
		uint32_t mb_end = self->sps.mb_total;
		if (mb_start >= mb_end) {
			/* Mismatch in macroblock ordering */
			self->current_frame->base->info.error = 1;
			ULOGW("%s: mismatch in macroblock ordering: "
			      "mb_start=%u mb_end=%u",
			      __func__,
			      mb_start,
			      mb_end);
		} else {
			frame_set_mb_status(self->current_frame,
					    mb_start,
					    mb_end - mb_start,
					    self->prev_slice.mb_status);
		}
	}

	au_output(self, self->current_frame);

out:
	/* Frame no more needed */
	if (self->current_frame != NULL)
		vstrm_frame_unref(self->current_frame->base);
	self->current_frame = NULL;

	/* Reset for next frame */
	self->nalu.first = true;
	self->nalu.last = false;
	self->nalu.recovery_point = false;
	self->nalu.idr = false;
	if (self->metadata.meta) {
		vmeta_frame_unref(self->metadata.meta);
		self->metadata.meta = NULL;
	}
	self->metadata.pack_bf_low = UINT64_C(0);
	self->metadata.pack_bf_high = UINT64_C(0);
	self->metadata.len = 0;
	memset(&self->current_timestamps, 0, sizeof(self->current_timestamps));
	if (ref)
		self->au.prev_frame_num = self->au.frame_num;
	self->au.prev_pic_order_count_lsb = self->au.pic_order_count_lsb;
	self->au.first = false;
	self->au.index++;
	memset(&self->prev_slice, 0, sizeof(self->prev_slice));
	return 0;
}


static int generate_grey_i_frame(struct vstrm_rtp_h264_rx *self,
				 unsigned int frame_index,
				 struct vstrm_rtp_h264_rx_frame **frame)
{
	int res = 0;
	struct pomp_buffer *buf = NULL, *slice_buf = NULL;

	ULOGI("generating grey IDR frame");

	/* Generate grey I slice */
	memset(&self->tmp_slice, 0, sizeof(self->tmp_slice));
	res = generate_grey_i_slice(self,
				    3,
				    NULL,
				    0,
				    self->sps.mb_total,
				    &self->tmp_slice,
				    &slice_buf);
	if (res < 0)
		goto out;

	/* Create frame */
	res = frame_new(self, frame_index, frame);
	if (res < 0)
		goto out;
	(*frame)->idr = true;
	(*frame)->base->info.complete = 1;
	(*frame)->base->info.ref = 1;
	(*frame)->base->info.error = 1;
	(*frame)->base->info.gen_grey_idr = 1;
	frame_update(self, *frame, &self->tmp_slice);

	/* Insert SPS */
	buf = pomp_buffer_new_with_data(self->sps.buf, self->sps.len);
	if (buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	res = frame_add_nalu(*frame, buf);
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
	res = frame_add_nalu(*frame, buf);
	if (res < 0)
		goto out;
	pomp_buffer_unref(buf);
	buf = NULL;

	/* Insert grey I slice */
	res = frame_add_nalu(*frame, slice_buf);
	if (res < 0)
		goto out;
	pomp_buffer_unref(slice_buf);
	slice_buf = NULL;

	/* Initialize MB status */
	frame_set_mb_status(*frame,
			    0,
			    self->sps.mb_total,
			    VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_ISLICE);

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
	if (slice_buf != NULL)
		pomp_buffer_unref(slice_buf);
	if (res != 0 && *frame != NULL) {
		vstrm_frame_unref((*frame)->base);
		*frame = NULL;
	}
	return res;
}


static int generate_skipped_p_frame(struct vstrm_rtp_h264_rx *self,
				    unsigned int frame_index,
				    int frame_num,
				    int pic_order_count_lsb,
				    struct vstrm_rtp_h264_rx_frame **frame)
{
	int res = 0;
	struct pomp_buffer *buf = NULL;
	struct h264_slice_header ref_sh = {0};

	/* Generate P-skip slice */
	memset(&self->tmp_slice, 0, sizeof(self->tmp_slice));
	memset(&ref_sh, 0, sizeof(ref_sh));
	ref_sh.frame_num = frame_num;
	ref_sh.pic_order_cnt_lsb = pic_order_count_lsb;
	res = generate_skipped_p_slice(self,
				       3,
				       &ref_sh,
				       0,
				       self->sps.mb_total,
				       &self->tmp_slice,
				       &buf);
	if (res < 0)
		goto out;

	/* Create frame */
	res = frame_new(self, frame_index, frame);
	if (res < 0)
		goto out;
	(*frame)->base->info.complete = 1;
	(*frame)->base->info.ref = 1;
	(*frame)->base->info.error = 1;
	(*frame)->base->info.gen_concealment = 1;
	frame_update(self, *frame, &self->tmp_slice);

	/* Insert P-skip slice */
	res = frame_add_nalu(*frame, buf);
	if (res < 0)
		goto out;
	pomp_buffer_unref(buf);
	buf = NULL;

	/* Initialize MB status */
	frame_set_mb_status(*frame,
			    0,
			    self->sps.mb_total,
			    VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_PSLICE);

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
	if (res != 0 && *frame != NULL) {
		vstrm_frame_unref((*frame)->base);
		*frame = NULL;
	}
	return res;
}


static void check_missing_frames(struct vstrm_rtp_h264_rx *self)
{
	int err;
	uint32_t missing = 0;
	uint32_t missing_poc = 0;
	uint32_t count = 0;
	bool conceal = CHECK_FLAG(self->cfg.flags, H264_GEN_CONCEALMENT_FRAME);
	bool conceal_poc = (self->sps.sps.pic_order_cnt_type == 0 &&
			    self->max_pic_order_cnt_lsb > 0);

	/* If frame_num is zero, stop here as it is not possible to predict how
	 * many frames have been missed */
	if (self->au.frame_num == 0)
		return;

	/* If frame num equals previous one, no frame is missing */
	if (self->au.frame_num == self->au.prev_frame_num)
		return;

	/* If frame_num is smaller than previous, frame_num has looped.
	 * Only frames from zero to (frame_num - 1) can be concealed. */
	if (self->au.frame_num < self->au.prev_frame_num) {
		/* Set previous frame_num to (max_frame_num - 1) to conceal
		 * frames from zero to (frame_num - 1) */
		self->au.prev_frame_num = (self->max_frame_num - 1);
	}
	if (conceal_poc &&
	    self->au.pic_order_count_lsb < self->au.prev_pic_order_count_lsb) {
		self->au.prev_pic_order_count_lsb =
			(self->max_pic_order_cnt_lsb - 1);
	}

	/* Determine missing frame_num and POC (handle wrapping) */
	missing = (self->au.frame_num - self->au.prev_frame_num - 1 +
		   self->max_frame_num) %
		  self->max_frame_num;
	if (missing == 0)
		return;

	if (conceal_poc) {
		missing_poc = (self->au.pic_order_count_lsb -
			       self->au.prev_pic_order_count_lsb - 1 +
			       self->max_pic_order_cnt_lsb) %
			      self->max_pic_order_cnt_lsb;
	}

	count = MIN(missing, (unsigned int)MAX_FRAME_CONCEALMENT);

	ULOGD("rtp_h264: missing frames: %u (poc: %u)"
	      " (concealed: %u) (cur:%u, prev:%u, max:%u)"
	      " (poc: cur: %u, prev: %u, max: %u)",
	      missing,
	      missing_poc,
	      conceal ? count : 0,
	      self->au.frame_num,
	      self->au.prev_frame_num,
	      self->max_frame_num,
	      self->au.pic_order_count_lsb,
	      self->au.prev_pic_order_count_lsb,
	      self->max_pic_order_cnt_lsb);

	if (conceal_poc && missing_poc < missing) {
		ULOGE("mising frames > missing poc!");
		return;
	}

	uint32_t frame_num = 0;
	uint32_t pic_order_cnt_lsb = 0;
	for (unsigned int i = 0; i < count; i++) {
		frame_num = (self->au.prev_frame_num + (i + 1)) %
			    self->max_frame_num;
		if (conceal_poc) {
			float closest =
				((float)missing_poc / (float)missing) * (i + 1);
			pic_order_cnt_lsb = (self->au.prev_pic_order_count_lsb +
					     (uint32_t)closest) %
					    self->max_pic_order_cnt_lsb;
		}
		if (conceal) {
			/* Generate NALU with skipped P slice */
			struct vstrm_rtp_h264_rx_frame *p_frame = NULL;
			err = generate_skipped_p_frame(self,
						       self->au.index,
						       frame_num,
						       pic_order_cnt_lsb,
						       &p_frame);
			if (err < 0) {
				ULOG_ERRNO("generate_skipped_p_frame", -err);
				continue;
			}
			/* Set frame timestamps */
			err = interpolate_timestamps(
				&self->last_timestamps,
				&self->current_timestamps,
				i,
				count,
				&p_frame->base->timestamps);
			if (err < 0) {
				ULOG_ERRNO(
					"vstrm_rtp_h264_rx"
					"_interpolate_timestamps",
					-err);
				continue;
			}
			au_output(self, p_frame);
			vstrm_frame_unref(p_frame->base);
			self->au.first = false;
			self->au.index++;
			if (self->current_frame != NULL)
				self->current_frame->index = self->au.index;
		} else if (self->current_frame != NULL) {
			frame_set_mb_status(self->current_frame,
					    0,
					    self->sps.mb_total,
					    VSTRM_FRAME_MB_STATUS_MISSING);
			self->video_stats.v2.total_frame_count++;
			self->video_stats.v2.missed_frame_count++;
			update_mb_status_stats(self, NULL);
		}
	}
	/* Update previous frame_num / POC */
	self->au.prev_frame_num = frame_num;
	self->au.prev_pic_order_count_lsb = pic_order_cnt_lsb;
}


static int au_add_nalu(struct vstrm_rtp_h264_rx *self,
		       struct pomp_buffer *buf,
		       const struct vstrm_rtp_h264_rx_slice *cur_slice,
		       bool update_frame)
{
	int res = 0;

	/* Generate a grey I frame if needed */
	if (self->au.first && self->sps.valid && self->pps.valid &&
	    cur_slice != NULL && !self->nalu.idr &&
	    CHECK_FLAG(self->cfg.flags, H264_GEN_GREY_IDR_FRAME)) {
		struct vstrm_rtp_h264_rx_frame *grey_i_frame = NULL;
		res = generate_grey_i_frame(
			self, self->au.index, &grey_i_frame);
		if (res == 0) {
			struct timespec ts = {0, 0};
			uint64_t out_timestamp = 0;
			time_get_monotonic(&ts);
			time_timespec_to_us(&ts, &out_timestamp);
			struct vstrm_timestamp tmp_ts =
				self->current_timestamps;
			if (tmp_ts.input > 0)
				tmp_ts.input--;
			if (tmp_ts.rtp > 0)
				tmp_ts.rtp--;
			if (tmp_ts.ntp > 0)
				tmp_ts.ntp--;
			if (tmp_ts.ntp_unskewed > 0)
				tmp_ts.ntp_unskewed--;
			if (tmp_ts.ntp_local > 0)
				tmp_ts.ntp_local--;
			if (tmp_ts.ntp_raw > 0)
				tmp_ts.ntp_raw--;
			if (tmp_ts.ntp_raw_unskewed > 0)
				tmp_ts.ntp_raw_unskewed--;
			map_timestamps(&tmp_ts,
				       out_timestamp,
				       &grey_i_frame->base->timestamps);
			au_output(self, grey_i_frame);
			vstrm_frame_unref(grey_i_frame->base);
			self->au.first = false;
			self->au.index++;
			if (self->current_frame != NULL)
				self->current_frame->index = self->au.index;
		}
	}

	/* Setup frame number for first slice */
	if (cur_slice != NULL && !self->prev_slice.valid) {
		self->au.frame_num = cur_slice->slice_header.frame_num;
		self->au.pic_order_count_lsb =
			cur_slice->slice_header.pic_order_cnt_lsb;
		if (!self->au.first)
			check_missing_frames(self);
		/* A picture including a memory_management_control_operation
		 * equal to 5 shall have frame_num constraints as described
		 * above and, after the decoding of the current picture and the
		 * processing of the memory management control operations, the
		 * picture shall be inferred to have had frame_num equal to 0
		 * for all subsequent use in the decoding process, except as
		 * specified in clause 7.4.1.2.4. */
		if (cur_slice->slice_header.drpm
			    .adaptive_ref_pic_marking_mode_flag) {
			for (int i = 0;
			     cur_slice->slice_header.drpm.mm[i]
				     .memory_management_control_operation != 0;
			     i++) {
				const struct h264_drpm_item *mm =
					&cur_slice->slice_header.drpm.mm[i];
				if (mm->memory_management_control_operation ==
				    5) {
					self->au.frame_num = 0;
					break;
				}
			}
		}
	}

	/* Create frame structure if needed */
	if (self->current_frame == NULL) {
		res = frame_new(self, self->au.index, &self->current_frame);
		if (res < 0)
			goto out;
	}

	if (self->nalu.recovery_point)
		self->current_frame->recovery_point = true;
	if (self->nalu.idr)
		self->current_frame->idr = true;
	if (update_frame)
		frame_update(self, self->current_frame, cur_slice);

	/* Add NALU in frame, nothing more to do if not a slice */
	res = frame_add_nalu(self->current_frame, buf);
	if (res < 0)
		goto out;
	self->nalu.first = false;
	if (cur_slice == NULL)
		goto out;

	/* Update the macroblock status of the previous slice. It is delayed
	 * to avoid parsing slice data to determine the number of macroblocks
	 * in the previous slice (we use start of current as hint). */
	if (CHECK_FLAG(self->cfg.flags, H264_FULL_MB_STATUS)) {
		/* In FULL_MB_STATUS, parse the complete slice to get the
		 * status of each macroblock. It is not done for concealed
		 * slices as we already know the status of each macroblock. */
		if ((cur_slice->mb_status ==
		     VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_PSLICE) ||
		    (cur_slice->mb_status ==
		     VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED_ISLICE)) {
			frame_set_mb_status(
				self->current_frame,
				cur_slice->slice_header.first_mb_in_slice,
				cur_slice->mb_count,
				cur_slice->mb_status);
		} else {
			const void *cdata = NULL;
			size_t len = 0;
			pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
			self->update_mb_status = true;
			h264_reader_parse_nalu(self->reader,
					       H264_READER_FLAGS_SLICE_DATA,
					       cdata,
					       len);
			self->update_mb_status = false;
		}
	} else if (self->prev_slice.valid) {
		uint32_t mb_start =
			self->prev_slice.slice_header.first_mb_in_slice;
		uint32_t mb_end = cur_slice->slice_header.first_mb_in_slice;
		if (mb_start >= mb_end) {
			/* Mismatch in macroblock ordering */
			self->current_frame->base->info.error = 1;
			ULOGW("%s: mismatch in macroblock ordering: "
			      "mb_start=%u mb_end=%u",
			      __func__,
			      mb_start,
			      mb_end);
		} else {
			frame_set_mb_status(self->current_frame,
					    mb_start,
					    mb_end - mb_start,
					    self->prev_slice.mb_status);
		}
	}

	/* Update previous slice */
	self->prev_slice = *cur_slice;

out:
	return res;
}


static int
handle_missing_slices(struct vstrm_rtp_h264_rx *self,
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
	if (self->current_frame != NULL && self->prev_slice.valid) {
		/* Use information found in custom SEI */
		if (self->info_v2.valid) {
			self->slice = self->prev_slice;
			self->slice.mb_count = self->info_v2.sei.slice_mb_count;
		} else if (self->info_v4.valid) {
			self->slice = self->prev_slice;
			self->slice.mb_count =
				(self->current_frame->recovery_point &&
				 (self->info_v4.sei
					  .slice_mb_count_recovery_point != 0))
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
			self->current_frame->base->info.complete = 0;
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
		mb_end = self->sps.mb_total;
	}

	/* If the gap was just before the first slice
	 * (we missed AUD, SPS, SPS, SEI...) */
	if (mb_start == 0 && mb_end == 0)
		goto out;

	/* Create frame structure if needed */
	if (self->current_frame == NULL) {
		res = frame_new(self, self->au.index, &self->current_frame);
		if (res < 0)
			goto out;
	}

	if (self->nalu.recovery_point)
		self->current_frame->recovery_point = true;
	if (self->nalu.idr)
		self->current_frame->idr = true;
	frame_update(self,
		     self->current_frame,
		     cur_slice ? cur_slice : &self->prev_slice);

	self->current_frame->base->info.error = 1;
	if (mb_start >= mb_end) {
		/* Mismatch in macroblock ordering */
		self->current_frame->base->info.complete = 0;
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
		if (self->current_frame->idr) {
			res = generate_grey_i_slice(self,
						    nal_ref_idc,
						    ref_sh,
						    mb_start,
						    mb_end - mb_start,
						    &self->tmp_slice,
						    &buf);
			if (res < 0)
				goto out;
		} else {
			res = generate_skipped_p_slice(self,
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
		res = au_add_nalu(self, buf, &self->tmp_slice, false);
		if (res < 0)
			goto out;
	} else {
		/* The frame will be incomplete */
		self->current_frame->base->info.complete = 0;
		self->prev_slice.mb_status = VSTRM_FRAME_MB_STATUS_MISSING;
		frame_set_mb_status(self->current_frame,
				    mb_start,
				    mb_end - mb_start,
				    VSTRM_FRAME_MB_STATUS_MISSING);
	}

out:
	if (buf != NULL)
		pomp_buffer_unref(buf);
	return res;
}


static int handle_missing_eof(struct vstrm_rtp_h264_rx *self)
{
	/* Add missing slices at the end and complete the frame */
	handle_missing_slices(self, NULL);
	au_complete(self);
	return 0;
}


static void sps_received(struct vstrm_rtp_h264_rx *self)
{
	uint32_t mb_width = 0, mb_height = 0;

	/* Abort current frame if any */
	if (self->current_frame != NULL) {
		vstrm_frame_unref(self->current_frame->base);
		self->current_frame = NULL;
	}

	self->max_frame_num = self->sps.sps_derived.MaxFrameNum;
	self->max_pic_order_cnt_lsb = self->sps.sps_derived.MaxPicOrderCntLsb;

	/* Compute size */
	mb_width = self->sps.sps_derived.PicWidthInMbs;
	mb_height = self->sps.sps_derived.FrameHeightInMbs;

	/* Update macroblock status array */
	if (mb_width != self->sps.mb_width ||
	    mb_height != self->sps.mb_height) {
		self->sps.mb_width = mb_width;
		self->sps.mb_height = mb_height;
		self->sps.mb_total = mb_width * mb_height;
		int err = ref_init(self);
		if (err < 0)
			ULOG_ERRNO("ref_init", -err);
		dot_init(self);
	}
}


static void pps_received(struct vstrm_rtp_h264_rx *self)
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
		/* TODO: merge this with part of vstrm_rtp_h264_rx_clear? */
		self->au.frame_num = 0;
		self->au.prev_frame_num = 0;
		self->au.pic_order_count_lsb = 0;
		self->au.prev_pic_order_count_lsb = 0;
		self->au.first = true;

		self->nalu.first = true;
		self->nalu.last = false;
		self->nalu.recovery_point = false;
		self->nalu.idr = false;

		self->slice.valid = false;
		self->prev_slice.valid = false;
		/* Reset extra info found in SEI with geometry */
		self->info_v1.valid = false;
		self->info_v2.valid = false;
		self->info_v4.valid = false;
		self->codec_info = codec_info;
		(*self->cbs.codec_info_changed)(
			self, &self->codec_info, self->cbs.userdata);
	}
}


static int nalu_complete(struct vstrm_rtp_h264_rx *self)
{
	int res = 0;
	const void *cdata = NULL;
	size_t len = 0;
	struct h264_nalu_header nh;
	int parse_nalu = 0, is_slice = 0;
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
	case H264_NALU_TYPE_SLICE:
	case H264_NALU_TYPE_SLICE_DPA:
	case H264_NALU_TYPE_SLICE_DPB:
	case H264_NALU_TYPE_SLICE_DPC:
		parse_nalu = self->sps.valid && self->pps.valid;
		is_slice = 1;
		self->nalu.idr = (self->nalu.type == H264_NALU_TYPE_SLICE_IDR);
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
			sps_received(self);
		} else if (self->nalu.type == H264_NALU_TYPE_PPS) {
			pps_received(self);
		}
	}

	/* If there was a gap, handle missing slices */
	if (self->gap) {
		res = handle_missing_slices(
			self,
			self->slice_copy.valid ? &self->slice_copy : NULL);
		if (res < 0)
			goto out;
		self->gap = false;
	}

	/* Add NALU in current frame */
	res = au_add_nalu(self,
			  self->nalu.buf,
			  self->slice_copy.valid ? &self->slice_copy : NULL,
			  true);
	if (res < 0)
		goto out;

	/* If it was the last NALU, complete the frame */
	if (self->nalu.last) {
		res = au_complete(self);
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


static int
nalu_append(struct vstrm_rtp_h264_rx *self, const uint8_t *buf, size_t len)
{
	/* If no buffer, create one with the data */
	if (self->nalu.buf == NULL) {
		self->nalu.buf = pomp_buffer_new_with_data(buf, len);
		return self->nalu.buf == NULL ? -ENOMEM : 0;
	}

	/* Append to current buffer */
	return pomp_buffer_append_data(self->nalu.buf, buf, len);
}


static int process_aggregation(struct vstrm_rtp_h264_rx *self,
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
			self->nalu.last = true;

		/* Append NALU data, and complete it */
		res = nalu_append(self, payloadbuf, nalulen);
		if (res < 0)
			goto out;
		res = nalu_complete(self);
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


static int process_fragment(struct vstrm_rtp_h264_rx *self,
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
		res = nalu_append(self, &nalu_header, 1);
		if (res < 0)
			goto out;
	} else if (self->current_fu_type == 0) {
		/* We missed the start of the fragment */
		res = -EIO;
		ULOGD("rtp_h264: skip fu");
		goto out;
	}

	/* Add NALU data */
	res = nalu_append(self, payloadbuf, payloadlen);
	if (res < 0)
		goto out;

	/* Complete NALU if it is the last fragment */
	if (end) {
		self->current_fu_type = 0;
		self->nalu.last = self->marker;
		res = nalu_complete(self);
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


static int process_extheader(struct vstrm_rtp_h264_rx *self,
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
	if (self->metadata.meta) {
		vmeta_frame_unref(self->metadata.meta);
		self->metadata.meta = NULL;
	}
	res = vmeta_frame_read2(
		&buf,
		mime_type,
		!(CHECK_FLAG(self->cfg.flags,
			     DISABLE_VIDEO_METADATA_CONVERSION)),
		&self->metadata.meta);
	if (res == -ENODATA) {
		/* Empty sample */
		res = 0;
	} else if (res < 0) {
		ULOG_ERRNO("vmeta_frame_read2", -res);
		goto out;
	}

out:
	return res;
}


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
	self->au.first = true;
	self->nalu.first = true;

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
	self->err_sec_start_time = UINT64_MAX;
	self->err_sec_start_time_by_zone = calloc(
		self->video_stats.mb_status_zone_count, sizeof(uint64_t));
	if (self->err_sec_start_time_by_zone == NULL)
		goto error;
	for (uint32_t k = 0; k < self->video_stats.mb_status_zone_count; k++)
		self->err_sec_start_time_by_zone[k] = UINT64_MAX;

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
	if (self->current_frame != NULL)
		vstrm_frame_unref(self->current_frame->base);
	ref_clear(self);
	dot_clear(self);
	if (self->nalu.buf != NULL)
		pomp_buffer_unref(self->nalu.buf);
	if (self->metadata.meta)
		vmeta_frame_unref(self->metadata.meta);
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
	self->gap = false;
	self->max_num_ref_logged = false;
	self->max_num_rplm_logged = false;

	memset(&self->codec_info, 0, sizeof(self->codec_info));
	self->codec_info.codec = VSTRM_CODEC_VIDEO_H264;
	self->sps.valid = false;
	self->sps.mb_width = 0;
	self->sps.mb_height = 0;
	self->sps.mb_total = 0;
	self->pps.valid = false;

	if (self->metadata.meta) {
		vmeta_frame_unref(self->metadata.meta);
		self->metadata.meta = NULL;
	}
	self->metadata.pack_bf_low = UINT64_C(0);
	self->metadata.pack_bf_high = UINT64_C(0);
	self->metadata.len = 0;

	ref_clear(self);
	dot_clear(self);

	memset(&self->current_timestamps, 0, sizeof(self->current_timestamps));
	memset(&self->last_timestamps, 0, sizeof(self->last_timestamps));

	self->au.index = 0;
	self->au.frame_num = 0;
	self->au.prev_frame_num = 0;
	self->au.pic_order_count_lsb = 0;
	self->au.prev_pic_order_count_lsb = 0;
	self->au.first = true;

	self->nalu.first = true;
	self->nalu.last = false;
	self->nalu.recovery_point = false;
	self->nalu.idr = false;

	self->slice.valid = false;
	self->prev_slice.valid = false;

	self->info_v1.valid = false;
	self->info_v2.valid = false;
	self->info_v4.valid = false;

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
		self->gap = true;

	/* Do we have a pending fragment? */
	if (self->current_fu_type != 0 &&
	    (gap != 0 || self->current_fu_type != nalu_type)) {
		/* Abort current fragment */
		ULOGD("rtp_h264: abort current fu");
		if (self->nalu.buf != NULL)
			pomp_buffer_set_len(self->nalu.buf, 0);
		self->current_fu_type = 0;
		self->gap = true;
	}

	if (!self->nalu.first && self->current_timestamps.rtp != 0 &&
	    self->current_timestamps.rtp != pkt->rtp_timestamp) {
		ULOGD("rtp_h264: missing end of frame");
		self->gap = true;
		handle_missing_eof(self);
	}

	/* Process header extensions (ignore errors) */
	if (pkt->extheader.len != 0) {
		process_extheader(self,
				  pkt->extheader.id,
				  pkt->raw.cdata + pkt->extheader.off,
				  pkt->extheader.len);
	}

	if (self->nalu.first)
		self->current_timestamps = *timestamp;

	self->marker = RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, MARKER);

	switch (nalu_type) {
	case VSTRM_RTP_H264_NALU_TYPE_STAP_A:
		res = process_aggregation(
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
		res = process_aggregation(self,
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
		res = process_fragment(self,
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
		res = process_fragment(self,
				       payloadbuf[0],
				       payloadbuf[1],
				       (payloadbuf[2] << 8) | payloadbuf[3],
				       payloadbuf + 4,
				       payloadlen - 4);
		break;

	default:
		self->nalu.last = self->marker;
		res = nalu_append(self, payloadbuf, payloadlen);
		if (res < 0)
			goto out;
		res = nalu_complete(self);
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

	sps_received(self);
	pps_received(self);
	return 0;
}
