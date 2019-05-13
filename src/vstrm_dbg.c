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

ULOG_DECLARE_TAG(vstrm);


FILE *vstrm_dbg_create_file(const char *dir,
			    void *ctx,
			    const char *name,
			    const char *mode)
{
	FILE *file = NULL;
	uint64_t epoch_sec = 0;
	int32_t utc_offset_sec = 0;
	struct tm tm;
	char path[260] = "";

	time_local_get(&epoch_sec, &utc_offset_sec);
	time_local_to_tm(epoch_sec, utc_offset_sec, &tm);

	snprintf(path,
		 sizeof(path),
		 "%s/vstrm_%04d%02d%02d_%02d%02d%02d_%d_%p_%s",
		 dir,
		 tm.tm_year + 1900,
		 tm.tm_mon + 1,
		 tm.tm_mday,
		 tm.tm_hour,
		 tm.tm_min,
		 tm.tm_sec,
		 getpid(),
		 ctx,
		 name);
	file = fopen(path, mode);
	if (file == NULL) {
		ULOGW("failed to create debug file '%s': err=%d(%s)",
		      path,
		      errno,
		      strerror(errno));
	}
	return file;
}


static void vstrm_dbg_write_h264_nalu(FILE *file, const void *buf, size_t count)
{
	fwrite("\x00\x00\x00\x01", 1, 4, file);
	fwrite(buf, 1, count, file);
}


void vstrm_dbg_write_raw(FILE *file, const void *buf, size_t count)
{
	/* Write size as 4 byte big endian */
	uint8_t count_buf[4] = {
		(count >> 24) & 0xff,
		(count >> 16) & 0xff,
		(count >> 8) & 0xff,
		count & 0xff,
	};

	fwrite(count_buf, 1, 4, file);
	fwrite(buf, 1, count, file);
}


void vstrm_dbg_write_pomp_buf(FILE *file, struct pomp_buffer *buf)
{
	const void *cdata = NULL;
	size_t len = 0;
	pomp_buffer_get_cdata(buf, &cdata, &len, NULL);
	vstrm_dbg_write_raw(file, cdata, len);
}


void vstrm_dbg_write_codec_info(FILE *file, const struct vstrm_codec_info *info)
{
	if (info->codec == VSTRM_CODEC_VIDEO_H264) {
		vstrm_dbg_write_h264_nalu(
			file, info->h264.sps, info->h264.spslen);
		vstrm_dbg_write_h264_nalu(
			file, info->h264.pps, info->h264.ppslen);
	}
}


void vstrm_dbg_write_frame(FILE *file,
			   const struct vstrm_codec_info *info,
			   struct vstrm_frame *frame)
{
	size_t i = 0;
	if (info == NULL || info->codec == VSTRM_CODEC_VIDEO_H264) {
		for (i = 0; i < frame->nalu_count; i++) {
			vstrm_dbg_write_h264_nalu(file,
						  frame->nalus[i].cdata,
						  frame->nalus[i].len);
		}
	}
}
