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


int vstrm_frame_new(const struct vstrm_frame_ops *ops,
		    size_t extra_size,
		    struct vstrm_frame **ret_obj)
{
	struct vstrm_frame *self = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ops == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ops->dispose == NULL, EINVAL);

	*ret_obj = NULL;

	/* Allocate structure + extra size */
	self = calloc(1, sizeof(*self) + extra_size);
	if (self == NULL)
		return -ENOMEM;
	self->ops = *ops;
	self->refcount = 1;
	list_node_unref(&self->node);

	/* If extra size was given, store it in userdata */
	if (extra_size > 0)
		self->userdata = (void *)(((uintptr_t)self) + sizeof(*self));

	*ret_obj = self;
	return 0;
}


void vstrm_frame_ref(struct vstrm_frame *self)
{
	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
#if defined(__GNUC__)
	__sync_add_and_fetch(&self->refcount, 1);
#elif defined(_WIN32)
	/* codecheck_ignore[SPACING,VOLATILE] */
	InterlockedIncrement((long volatile *)&self->refcount);
#else
#	error No atomic increment function found on this platform
#endif
}


void vstrm_frame_unref(struct vstrm_frame *self)
{
	uint32_t res = 0;
	if (self == NULL)
		return;
#if defined(__GNUC__)
	res = __sync_sub_and_fetch(&self->refcount, 1);
#elif defined(_WIN32)
	/* codecheck_ignore[SPACING,VOLATILE] */
	res = (uint32_t)InterlockedDecrement((long volatile *)&self->refcount);
#else
#	error No atomic decrement function found on this platform
#endif

	/* Free resource when ref count reaches 0 */
	if (res == 0) {
		if (list_node_is_ref(&self->node))
			ULOGW("frame %p is still in a list", self);

		(*self->ops.dispose)(self);
		vstrm_video_stats_dyn_clear(&self->video_stats_dyn);
		free(self->nalus);
		free(self);
	}
}


int vstrm_frame_add_nalu(struct vstrm_frame *self,
			 const struct vstrm_frame_nalu *nalu)
{
	struct vstrm_frame_nalu *newnalus;
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->refcount > 1, EPERM);
	ULOG_ERRNO_RETURN_ERR_IF(nalu == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(nalu->cdata == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(nalu->len == 0, EINVAL);

	/* Grow array if needed */
	if (self->nalu_count + 1 > self->max_nalu_count) {
		newnalus = realloc(self->nalus,
				   (self->max_nalu_count + 16) *
					   sizeof(struct vstrm_frame_nalu));
		if (newnalus == NULL)
			return -ENOMEM;
		self->nalus = newnalus;
		self->max_nalu_count = self->max_nalu_count + 16;
	}

	/* Simply copy buffer pointer in array */
	self->nalus[self->nalu_count] = *nalu;
	self->nalu_count++;
	return 0;
}


int vstrm_frame_get_size(struct vstrm_frame *self, size_t *size, uint32_t flags)
{
	const struct vstrm_frame_nalu *nalu = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == NULL, EINVAL);

	*size = 0;
	for (uint32_t i = 0; i < self->nalu_count; i++) {
		nalu = &self->nalus[i];
		if ((flags & VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS) &&
		    (((nalu->cdata[0] & 0x1F) == H264_NALU_TYPE_SPS) ||
		     ((nalu->cdata[0] & 0x1F) == H264_NALU_TYPE_PPS)))
			continue;
		if ((flags & VSTRM_FRAME_COPY_FLAGS_FILTER_SEI) &&
		    ((nalu->cdata[0] & 0x1F) == H264_NALU_TYPE_SEI))
			continue;
		if (flags & VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE)
			*size += 4;
		else if (flags & VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE)
			*size += 4;
		*size += nalu->len;
	}
	return 0;
}


int vstrm_frame_copy(struct vstrm_frame *self,
		     uint8_t *buf,
		     size_t len,
		     uint32_t flags)
{
	size_t pos = 0;
	const struct vstrm_frame_nalu *nalu = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	for (uint32_t i = 0; i < self->nalu_count; i++) {
		nalu = &self->nalus[i];
		if ((flags & VSTRM_FRAME_COPY_FLAGS_FILTER_SPS_PPS) &&
		    (((nalu->cdata[0] & 0x1F) == H264_NALU_TYPE_SPS) ||
		     ((nalu->cdata[0] & 0x1F) == H264_NALU_TYPE_PPS)))
			continue;
		if ((flags & VSTRM_FRAME_COPY_FLAGS_FILTER_SEI) &&
		    ((nalu->cdata[0] & 0x1F) == H264_NALU_TYPE_SEI))
			continue;
		if (flags & VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_START_CODE) {
			if (pos + 4 > len)
				return -EAGAIN;
			buf[pos++] = 0x00;
			buf[pos++] = 0x00;
			buf[pos++] = 0x00;
			buf[pos++] = 0x01;
		} else if (flags & VSTRM_FRAME_COPY_FLAGS_INSERT_NALU_SIZE) {
			if (pos + 4 > len)
				return -EAGAIN;
			buf[pos++] = (nalu->len >> 24) & 0xff;
			buf[pos++] = (nalu->len >> 16) & 0xff;
			buf[pos++] = (nalu->len >> 8) & 0xff;
			buf[pos++] = nalu->len & 0xff;
		}
		if (pos + nalu->len > len)
			return -EAGAIN;
		memcpy(buf + pos, nalu->cdata, nalu->len);
		pos += nalu->len;
	}
	return 0;
}


const char *vstrm_codec_str(enum vstrm_codec val)
{
	switch (val) {
	default: /* NO BREAK */
	case VSTRM_CODEC_UNKNOWN:
		return "UNKNOWN";
	case VSTRM_CODEC_VIDEO_H264:
		return "H264";
	}
}


const char *vstrm_frame_mb_status_str(enum vstrm_frame_mb_status val)
{
	switch (val) {
	case VSTRM_FRAME_MB_STATUS_UNKNOWN:
		return "UNKNOWN";
	case VSTRM_FRAME_MB_STATUS_VALID_ISLICE:
		return "VALID_ISLICE";
	case VSTRM_FRAME_MB_STATUS_VALID_PSLICE:
		return "VALID_PSLICE";
	case VSTRM_FRAME_MB_STATUS_MISSING_CONCEALED:
		return "MISSING_CONCEALED";
	case VSTRM_FRAME_MB_STATUS_MISSING:
		return "MISSING";
	case VSTRM_FRAME_MB_STATUS_ERROR_PROPAGATION:
		return "ERROR_PROPAGATION";
	default:
		return "UNKNOWN";
	}
}
