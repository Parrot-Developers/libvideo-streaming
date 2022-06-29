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

#ifndef _VSTRM_PRIV_H_
#define _VSTRM_PRIV_H_

#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#define ULOG_TAG vstrm
#include <ulog.h>

#include <futils/futils.h>
#include <h264/h264.h>
#include <libpomp.h>
#include <rtp/rtp.h>

#include "video-metadata/vmeta.h"
#include "video-streaming/vstrm.h"

struct vstrm_timestamp;

#include "vstrm_clock_delta.h"
#include "vstrm_event.h"
#include "vstrm_rtcp_app.h"
#include "vstrm_rtp_h264.h"
#include "vstrm_video_stats_priv.h"

#define VSTRM_H264_MB_STATUS_CLASS_COUNT 6
#define VSTRM_H264_MB_STATUS_ZONE_COUNT 5

#define VSTRM_USECS_PER_SEC 1000000

#define VSTRM_METADATA_PROTO_HEADER_LEN 8


struct vstrm_timestamp {
	/* Reception timestamp */
	uint64_t input;

	/* RTP timestamp from the RTP packet header (without wrap) */
	uint64_t rtp;

	/* NTP timestamp computed from RTP timestamp with RTCP info:
	 * NTP = (RTP - b) / a */
	uint64_t ntp;

	/* Similar to ntp but with clock skew*/
	uint64_t ntp_unskewed;

	/* Similar to ntp but with clock delta from RTCP extension */
	uint64_t ntp_local;

	/* NTP timestamp computed from RTP timestamp:
	 * NTP = RTP * 1000000 / clkrate */
	uint64_t ntp_raw;

	/* Similar to ntp_raw but with clock skew */
	uint64_t ntp_raw_unskewed;
};


static inline char *xstrdup(const char *s)
{
	return s == NULL ? NULL : strdup(s);
}


static inline int xstrcmp(const char *s1, const char *s2)
{
	if (s1 == NULL && s2 == NULL)
		return 0;
	else if (s1 == NULL)
		return -1;
	else if (s2 == NULL)
		return 1;
	return strcmp(s1, s2);
}


int vstrm_session_metadata_write_rtcp_sdes(struct pomp_buffer *buf,
					   size_t *pos,
					   uint32_t ssrc,
					   const struct vmeta_session *meta);


int vstrm_session_metadata_read_rtcp_sdes(const struct rtcp_pkt_sdes_item *item,
					  struct vmeta_session *meta);


FILE *vstrm_dbg_create_file(const char *dir,
			    void *ctx,
			    const char *name,
			    const char *mode);


void vstrm_dbg_write_raw(FILE *file, const void *buf, size_t count);


void vstrm_dbg_write_pomp_buf(FILE *file, struct pomp_buffer *buf);


void vstrm_dbg_write_codec_info(FILE *file,
				const struct vstrm_codec_info *info);


void vstrm_dbg_write_frame(FILE *file,
			   const struct vstrm_codec_info *info,
			   struct vstrm_frame *frame);


static inline int
vstrm_write_u8(struct pomp_buffer *buf, size_t *pos, uint8_t v)
{
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int vstrm_write_i8(struct pomp_buffer *buf, size_t *pos, int8_t v)
{
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
vstrm_write_u16(struct pomp_buffer *buf, size_t *pos, uint16_t v)
{
	v = htons(v);
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
vstrm_write_i16(struct pomp_buffer *buf, size_t *pos, int16_t v)
{
	v = htons(v);
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
vstrm_write_u32(struct pomp_buffer *buf, size_t *pos, uint32_t v)
{
	v = htonl(v);
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
vstrm_write_i32(struct pomp_buffer *buf, size_t *pos, int32_t v)
{
	v = htonl(v);
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
vstrm_write_u64(struct pomp_buffer *buf, size_t *pos, uint64_t v)
{
	uint32_t _v[2];
	_v[0] = htonl((v >> 32) & 0xffffffff);
	_v[1] = htonl(v & 0xffffffff);
	return pomp_buffer_write(buf, pos, _v, sizeof(_v));
}


static inline int
vstrm_write_i64(struct pomp_buffer *buf, size_t *pos, int64_t v)
{
	uint32_t _v[2];
	_v[0] = htonl((v >> 32) & 0xffffffff);
	_v[1] = htonl(v & 0xffffffff);
	return pomp_buffer_write(buf, pos, _v, sizeof(_v));
}


static inline int
vstrm_read_u8(struct pomp_buffer *buf, size_t *pos, uint8_t *v)
{
	return pomp_buffer_read(buf, pos, v, sizeof(*v));
}


static inline int vstrm_read_i8(struct pomp_buffer *buf, size_t *pos, int8_t *v)
{
	return pomp_buffer_read(buf, pos, v, sizeof(*v));
}


static inline int
vstrm_read_u16(struct pomp_buffer *buf, size_t *pos, uint16_t *v)
{
	int res = 0;
	res = pomp_buffer_read(buf, pos, v, sizeof(*v));
	if (res == 0)
		*v = ntohs(*v);
	return res;
}


static inline int
vstrm_read_i16(struct pomp_buffer *buf, size_t *pos, int16_t *v)
{
	int res = 0;
	res = pomp_buffer_read(buf, pos, v, sizeof(*v));
	if (res == 0)
		*v = ntohs(*v);
	return res;
}


static inline int
vstrm_read_u32(struct pomp_buffer *buf, size_t *pos, uint32_t *v)
{
	int res = 0;
	res = pomp_buffer_read(buf, pos, v, sizeof(*v));
	if (res == 0)
		*v = ntohl(*v);
	return res;
}


static inline int
vstrm_read_i32(struct pomp_buffer *buf, size_t *pos, int32_t *v)
{
	int res = 0;
	res = pomp_buffer_read(buf, pos, v, sizeof(*v));
	if (res == 0)
		*v = ntohl(*v);
	return res;
}


static inline int
vstrm_read_u64(struct pomp_buffer *buf, size_t *pos, uint64_t *v)
{
	int res = 0;
	uint32_t _v[2];
	res = pomp_buffer_read(buf, pos, _v, sizeof(_v));
	if (res == 0)
		*v = ((uint64_t)ntohl(_v[0]) << 32) | ntohl(_v[1]);
	return res;
}


static inline int
vstrm_read_i64(struct pomp_buffer *buf, size_t *pos, int64_t *v)
{
	int res = 0;
	uint32_t _v[2];
	res = pomp_buffer_read(buf, pos, _v, sizeof(_v));
	if (res == 0)
		*v = ((int64_t)ntohl(_v[0]) << 32) | ntohl(_v[1]);
	return res;
}


#if defined(_WIN32)
static inline char *strndup(const char *s, size_t n)
{
	char *res;
	size_t len = 0;
	while ((s[len] != '\0') && (len < n))
		len++;
	res = malloc(len + 1);
	if (res == NULL)
		return NULL;
	memcpy(res, s, len);
	res[len] = '\0';
	return res;
}
#endif /* _WIN32 */


#endif /* !_VSTRM_PRIV_H_ */
