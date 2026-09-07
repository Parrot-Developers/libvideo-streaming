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

#include <media-buffers/mbuf_coded_video_frame.h>
#include <video-defs/vdefs.h>


static void frame_from_mbuf_dispose(struct vstrm_frame *vframe)
{
	struct mbuf_coded_video_frame *frame = vframe->userdata;

	for (uint32_t i = 0; i < vframe->nalu_count; i++) {
		intptr_t start_code_size = (intptr_t)vframe->nalus[i].userdata;
		int res = mbuf_coded_video_frame_release_nalu(
			frame, i, vframe->nalus[i].cdata - start_code_size);
		if (res < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_release_nalu", -res);
	}

	mbuf_coded_video_frame_unref(frame);
}


int vstrm_frame_new_from_mbuf_coded_video_frame(
	struct mbuf_coded_video_frame *frame,
	struct vmeta_frame *metadata,
	struct vstrm_frame **ret_obj)
{
	int res;
	struct vstrm_frame *vframe = NULL;
	struct vdef_coded_frame frame_info;
	unsigned int nalu_count = 0;
	const struct vstrm_frame_ops ops = {
		.dispose = &frame_from_mbuf_dispose,
	};

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &frame_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	res = mbuf_coded_video_frame_get_nalu_count(frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -res);
		return res;
	}
	nalu_count = res;

	mbuf_coded_video_frame_ref(frame);

	res = vstrm_frame_new(&ops, 0, &vframe);
	if (res < 0) {
		ULOG_ERRNO("vstrm_frame_new", -res);
		mbuf_coded_video_frame_unref(frame);
		return res;
	}
	vframe->userdata = frame;
	if (metadata != NULL) {
		vmeta_frame_ref(metadata);
		vframe->metadata = metadata;
	}

	if (frame_info.info.timescale != 0) {
		/* Split before scaling to avoid a uint64_t overflow on
		 * large timestamps */
		uint64_t ts = frame_info.info.timestamp;
		uint64_t tb = frame_info.info.timescale;
		vframe->timestamps.ntp = (ts / tb) * VSTRM_USECS_PER_SEC +
					 ((ts % tb) * VSTRM_USECS_PER_SEC) / tb;
	}

	for (unsigned int i = 0; i < nalu_count; i++) {
		const void *data;
		struct vdef_nalu nalu;
		int start_code_size;

		res = mbuf_coded_video_frame_get_nalu(frame, i, &data, &nalu);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_get_nalu", -res);
			goto error;
		}

		switch (frame_info.format.data_format) {
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			start_code_size = 0;
			break;
		case VDEF_CODED_DATA_FORMAT_AVCC:
			start_code_size = 4;
			break;
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
			res = h264_get_start_code_length(data, nalu.size);
			if (res < 0) {
				ULOG_ERRNO("h264_get_start_code_length", -res);
				mbuf_coded_video_frame_release_nalu(
					frame, i, data);
				goto error;
			}
			start_code_size = res;
			break;
		default:
			res = -ENOSYS;
			ULOG_ERRNO("unsupported data format", -res);
			mbuf_coded_video_frame_release_nalu(frame, i, data);
			goto error;
		}

		res = vstrm_frame_add_nalu(
			vframe,
			&(struct vstrm_frame_nalu){
				.cdata =
					(const uint8_t *)data + start_code_size,
				.len = nalu.size - start_code_size,
				.priority = nalu.priority,
				.importance = nalu.importance,
				.userdata = (void *)(intptr_t)start_code_size,
			});
		if (res < 0) {
			ULOG_ERRNO("vstrm_frame_add_nalu", -res);
			mbuf_coded_video_frame_release_nalu(frame, i, data);
			goto error;
		}
	}

	*ret_obj = vframe;
	return 0;

error:
	vstrm_frame_unref(vframe);
	return res;
}
