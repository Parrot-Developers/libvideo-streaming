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

/* clang-format off */
#define CHECK(_x) do { if ((res = (_x)) < 0) goto out; } while (0)
/* clang-format on */

FUTILS_STATIC_ASSERT(VSTRM_EVENT_MAX <= UINT8_MAX,
		     "VSTRM_EVENT_MAX value exceeds UINT8_MAX");


int vstrm_event_write(struct pomp_buffer *buf,
		      size_t *pos,
		      enum vstrm_event event)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);

	struct vstrm_event_msg msg = {
		.version = VSTRM_EVENT_MSG_VERSION,
		.event = event,
	};

	CHECK(vstrm_write_u8(buf, pos, msg.version));
	CHECK(vstrm_write_u8(buf, pos, msg.event));

out:
	return res;
}


int vstrm_event_read(struct pomp_buffer *buf,
		     size_t *pos,
		     enum vstrm_event *event)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(event == NULL, EINVAL);

	struct vstrm_event_msg msg = {0};

	CHECK(vstrm_read_u8(buf, pos, &msg.version));
	CHECK(vstrm_read_u8(buf, pos, &msg.event));

out:
	if (msg.version != VSTRM_EVENT_MSG_VERSION)
		res = -EPROTO;
	if (msg.event >= VSTRM_EVENT_MAX)
		res = -EPROTO;
	if (res == 0)
		*event = msg.event;
	return res;
}


enum vstrm_event vstrm_event_from_str(const char *str)
{
	if (strcasecmp(str, "RECONFIGURE") == 0)
		return VSTRM_EVENT_RECONFIGURE;
	else if (strcasecmp(str, "RESOLUTION_CHANGE") == 0)
		return VSTRM_EVENT_RESOLUTION_CHANGE;
	else if (strcasecmp(str, "PHOTO_TRIGGER") == 0)
		return VSTRM_EVENT_PHOTO_TRIGGER;
	else if (strcasecmp(str, "FRAMERATE_CHANGE") == 0)
		return VSTRM_EVENT_FRAMERATE_CHANGE;
	else
		return VSTRM_EVENT_NONE;
}


const char *vstrm_event_to_str(enum vstrm_event event)
{
	switch (event) {
	case VSTRM_EVENT_RECONFIGURE:
		return "RECONFIGURE";
	case VSTRM_EVENT_RESOLUTION_CHANGE:
		return "RESOLUTION_CHANGE";
	case VSTRM_EVENT_PHOTO_TRIGGER:
		return "PHOTO_TRIGGER";
	case VSTRM_EVENT_FRAMERATE_CHANGE:
		return "FRAMERATE_CHANGE";
	case VSTRM_EVENT_NONE:
	default:
		return "NONE";
	}
}
