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


static void encode_uuid(uint8_t uuid[16],
			uint32_t v0,
			uint32_t v1,
			uint32_t v2,
			uint32_t v3)
{
	uuid[0] = (v0 >> 24) & 0xff;
	uuid[1] = (v0 >> 16) & 0xff;
	uuid[2] = (v0 >> 8) & 0xff;
	uuid[3] = v0 & 0xff;
	uuid[4] = (v1 >> 24) & 0xff;
	uuid[5] = (v1 >> 16) & 0xff;
	uuid[6] = (v1 >> 8) & 0xff;
	uuid[7] = v1 & 0xff;
	uuid[8] = (v2 >> 24) & 0xff;
	uuid[9] = (v2 >> 16) & 0xff;
	uuid[10] = (v2 >> 8) & 0xff;
	uuid[11] = v2 & 0xff;
	uuid[12] = (v3 >> 24) & 0xff;
	uuid[13] = (v3 >> 16) & 0xff;
	uuid[14] = (v3 >> 8) & 0xff;
	uuid[15] = v3 & 0xff;
}


static int check_uuid(const uint8_t uuid[16],
		      uint32_t v0,
		      uint32_t v1,
		      uint32_t v2,
		      uint32_t v3)
{
	return (uuid[0] == ((v0 >> 24) & 0xff)) &&
	       (uuid[1] == ((v0 >> 16) & 0xff)) &&
	       (uuid[2] == ((v0 >> 8) & 0xff)) && (uuid[3] == (v0 & 0xff)) &&
	       (uuid[4] == ((v1 >> 24) & 0xff)) &&
	       (uuid[5] == ((v1 >> 16) & 0xff)) &&
	       (uuid[6] == ((v1 >> 8) & 0xff)) && (uuid[7] == (v1 & 0xff)) &&
	       (uuid[8] == ((v2 >> 24) & 0xff)) &&
	       (uuid[9] == ((v2 >> 16) & 0xff)) &&
	       (uuid[10] == ((v2 >> 8) & 0xff)) && (uuid[11] == (v2 & 0xff)) &&
	       (uuid[12] == ((v3 >> 24) & 0xff)) &&
	       (uuid[13] == ((v3 >> 16) & 0xff)) &&
	       (uuid[14] == ((v3 >> 8) & 0xff)) && (uuid[15] == (v3 & 0xff));
}


int vstrm_h264_sei_streaming_is_v1(const uint8_t uuid[16])
{
	ULOG_ERRNO_RETURN_ERR_IF(uuid == NULL, EINVAL);
	return check_uuid(uuid,
			  VSTRM_H264_SEI_STREAMING_V1_UUID_0,
			  VSTRM_H264_SEI_STREAMING_V1_UUID_1,
			  VSTRM_H264_SEI_STREAMING_V1_UUID_2,
			  VSTRM_H264_SEI_STREAMING_V1_UUID_3);
}


ssize_t vstrm_h264_sei_streaming_v1_get_size(
	const struct vstrm_h264_sei_streaming_v1 *sei)
{
	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	return 2 + sei->slice_count * 2;
}


int vstrm_h264_sei_streaming_v1_write(
	const struct vstrm_h264_sei_streaming_v1 *sei,
	uint8_t uuid[16],
	uint8_t *buf,
	size_t *len)
{
	uint32_t i = 0;

	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(uuid == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == NULL, EINVAL);

	ssize_t res = vstrm_h264_sei_streaming_v1_get_size(sei);
	if (res < 0) {
		ULOG_ERRNO("vstrm_h264_sei_streaming_v1_get_size", (int)-res);
		return res;
	}
	size_t sz = res;

	if (*len < sz)
		return -EAGAIN;

	encode_uuid(uuid,
		    VSTRM_H264_SEI_STREAMING_V1_UUID_0,
		    VSTRM_H264_SEI_STREAMING_V1_UUID_1,
		    VSTRM_H264_SEI_STREAMING_V1_UUID_2,
		    VSTRM_H264_SEI_STREAMING_V1_UUID_3);

	buf[0] = sei->index_in_gop;
	buf[1] = sei->slice_count;
	for (i = 0; i < sei->slice_count; i++) {
		buf[2 * i + 2] = (sei->slice_mb_count[i] >> 8) & 0xff;
		buf[2 * i + 3] = sei->slice_mb_count[i] & 0xff;
	}

	*len = sz;
	return 0;
}


int vstrm_h264_sei_streaming_v1_read(struct vstrm_h264_sei_streaming_v1 *sei,
				     const uint8_t uuid[16],
				     const uint8_t *buf,
				     size_t len)
{
	uint32_t i = 0;

	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(uuid == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	if (!vstrm_h264_sei_streaming_is_v1(uuid))
		return -EIO;

	if (len < 2)
		return -EIO;
	sei->index_in_gop = buf[0];
	sei->slice_count = buf[1];

	ssize_t res = vstrm_h264_sei_streaming_v1_get_size(sei);
	if (res < 0) {
		ULOG_ERRNO("vstrm_h264_sei_streaming_v1_get_size", (int)-res);
		return res;
	}
	size_t sz = res;

	if (len < sz)
		return -EIO;
	for (i = 0; i < sei->slice_count; i++)
		sei->slice_mb_count[i] = (buf[2 * i + 2] << 8) | buf[2 * i + 3];

	return 0;
}


int vstrm_h264_sei_streaming_is_v2(const uint8_t uuid[16])
{
	ULOG_ERRNO_RETURN_ERR_IF(uuid == NULL, EINVAL);

	return check_uuid(uuid,
			  VSTRM_H264_SEI_STREAMING_V2_UUID_0,
			  VSTRM_H264_SEI_STREAMING_V2_UUID_1,
			  VSTRM_H264_SEI_STREAMING_V2_UUID_2,
			  VSTRM_H264_SEI_STREAMING_V2_UUID_3);
}


ssize_t vstrm_h264_sei_streaming_v2_get_size(
	const struct vstrm_h264_sei_streaming_v2 *sei)
{
	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	return 4;
}


int vstrm_h264_sei_streaming_v2_write(
	const struct vstrm_h264_sei_streaming_v2 *sei,
	uint8_t uuid[16],
	uint8_t *buf,
	size_t *len)
{
	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(uuid == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == NULL, EINVAL);

	ssize_t res = vstrm_h264_sei_streaming_v2_get_size(sei);
	if (res < 0) {
		ULOG_ERRNO("vstrm_h264_sei_streaming_v2_get_size", (int)-res);
		return res;
	}
	size_t sz = res;

	if (*len < sz)
		return -EAGAIN;

	encode_uuid(uuid,
		    VSTRM_H264_SEI_STREAMING_V2_UUID_0,
		    VSTRM_H264_SEI_STREAMING_V2_UUID_1,
		    VSTRM_H264_SEI_STREAMING_V2_UUID_2,
		    VSTRM_H264_SEI_STREAMING_V2_UUID_3);

	buf[0] = (sei->slice_count >> 8) & 0xff;
	buf[1] = sei->slice_count & 0xff;
	buf[2] = (sei->slice_mb_count >> 8) & 0xff;
	buf[3] = sei->slice_mb_count & 0xff;

	*len = sz;
	return 0;
}


int vstrm_h264_sei_streaming_v2_read(struct vstrm_h264_sei_streaming_v2 *sei,
				     const uint8_t uuid[16],
				     const uint8_t *buf,
				     size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(uuid == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	if (!vstrm_h264_sei_streaming_is_v2(uuid))
		return -EIO;

	if (len < 4)
		return -EIO;
	sei->slice_count = (buf[0] << 8) | buf[1];
	sei->slice_mb_count = (buf[2] << 8) | buf[3];

	return 0;
}
