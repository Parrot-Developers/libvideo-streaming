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

#ifndef _VSTRM_RTP_H264_H_
#define _VSTRM_RTP_H264_H_


/** 5.1.  RTP Header Usage */
#define VSTRM_RTP_H264_PAYLOAD_TYPE 96
#define VSTRM_RTP_H264_CLK_RATE 90000

/** 5.7.1 Single-time aggregation packet */
#define VSTRM_RTP_H264_NALU_TYPE_STAP_A 24
#define VSTRM_RTP_H264_NALU_TYPE_STAP_B 25

/** 5.7.2 Multi-time aggregation packet */
#define VSTRM_RTP_H264_NALU_TYPE_MTAP16 26
#define VSTRM_RTP_H264_NALU_TYPE_MTAP24 27

/** 5.8 Fragmentation unit */
#define VSTRM_RTP_H264_NALU_TYPE_FU_A 28
#define VSTRM_RTP_H264_NALU_TYPE_FU_B 29


struct vstrm_rtp_h264_rx;
struct vstrm_rtp_h264_tx;


struct vstrm_rtp_h264_rx_cfg {
	uint32_t flags;
};


struct vstrm_rtp_h264_rx_cbs {
	void *userdata;

	void (*codec_info_changed)(struct vstrm_rtp_h264_rx *rtp_h264_rx,
				   const struct vstrm_codec_info *info,
				   void *userdata);

	void (*recv_frame)(struct vstrm_rtp_h264_rx *rtp_h264_rx,
			   struct vstrm_frame *frame,
			   void *userdata);
};


struct vstrm_rtp_h264_tx_cfg_dyn {
	uint32_t target_packet_size;
};


struct vstrm_rtp_h264_tx_cfg {
	uint32_t flags;
	struct vstrm_rtp_h264_tx_cfg_dyn dyn;
};


int vstrm_rtp_h264_rx_new(const struct vstrm_rtp_h264_rx_cfg *cfg,
			  const struct vstrm_rtp_h264_rx_cbs *cbs,
			  struct vstrm_rtp_h264_rx **ret_obj);


int vstrm_rtp_h264_rx_destroy(struct vstrm_rtp_h264_rx *self);


int vstrm_rtp_h264_rx_clear(struct vstrm_rtp_h264_rx *self);


int vstrm_rtp_h264_rx_process_packet(struct vstrm_rtp_h264_rx *self,
				     const struct rtp_pkt *pkt,
				     uint32_t gap,
				     const struct vstrm_timestamp *timestamp);


int vstrm_rtp_h264_rx_get_video_stats(
	struct vstrm_rtp_h264_rx *self,
	const struct vstrm_video_stats **video_stats,
	const struct vstrm_video_stats_dyn **video_stats_dyn);


int vstrm_rtp_h264_rx_get_codec_info(struct vstrm_rtp_h264_rx *self,
				     const struct vstrm_codec_info **info);


int vstrm_rtp_h264_rx_set_codec_info(struct vstrm_rtp_h264_rx *self,
				     const struct vstrm_codec_info *info);


int vstrm_rtp_h264_rx_generate_grey_i_frame(struct vstrm_rtp_h264_rx *self,
					    struct vstrm_frame **ret_frame);


int vstrm_rtp_h264_tx_new(const struct vstrm_rtp_h264_tx_cfg *cfg,
			  struct vstrm_rtp_h264_tx **ret_obj);


int vstrm_rtp_h264_tx_destroy(struct vstrm_rtp_h264_tx *self);


int vstrm_rtp_h264_tx_process_frame(struct vstrm_rtp_h264_tx *self,
				    struct vstrm_frame *frame,
				    struct list_node *packets);


int vstrm_rtp_h264_tx_set_cfg_dyn(
	struct vstrm_rtp_h264_tx *self,
	const struct vstrm_rtp_h264_tx_cfg_dyn *cfg_dyn);


#endif /* !_VSTRM_RTP_H264_H_ */
