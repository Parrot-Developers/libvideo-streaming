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


#define VSTRM_SDES_ITEMS_MAX_COUNT 30


#define ADD_ITEM(_type, _data)                                                 \
	{                                                                      \
		if (chunk->item_count < VSTRM_SDES_ITEMS_MAX_COUNT) {          \
			struct rtcp_pkt_sdes_item *item =                      \
				&items[chunk->item_count];                     \
			item->type = (_type);                                  \
			item->data = (const uint8_t *)strdup(_data);           \
			item->data_len = strlen(_data);                        \
			chunk->item_count++;                                   \
		} else {                                                       \
			ULOGW("max SDES items count has been reached");        \
		}                                                              \
	}


#define ADD_PRIV_ITEM(_prefix, _value)                                         \
	{                                                                      \
		if (chunk->item_count < VSTRM_SDES_ITEMS_MAX_COUNT) {          \
			struct rtcp_pkt_sdes_item *item =                      \
				&items[chunk->item_count];                     \
			item->type = RTCP_PKT_SDES_TYPE_PRIV;                  \
			item->priv.prefix = (const uint8_t *)strdup(_prefix);  \
			item->priv.prefix_len = strlen(_prefix);               \
			item->priv.value = (const uint8_t *)strdup(_value);    \
			item->priv.value_len = strlen(_value);                 \
			chunk->item_count++;                                   \
		} else {                                                       \
			ULOGW("max SDES items count has been reached");        \
		}                                                              \
	}


static void vstrm_session_metadata_write_cb(enum vmeta_stream_sdes_type type,
					    const char *value,
					    const char *prefix,
					    void *userdata)
{
	struct rtcp_pkt_sdes *sdes = (struct rtcp_pkt_sdes *)userdata;
	if (sdes == NULL)
		return;

	struct rtcp_pkt_sdes_chunk *chunk =
		(struct rtcp_pkt_sdes_chunk *)sdes->chunks;
	struct rtcp_pkt_sdes_item *items =
		(struct rtcp_pkt_sdes_item *)chunk->items;

	switch (type) {
	case VMETA_STRM_SDES_TYPE_CNAME:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_CNAME, value);
		break;

	case VMETA_STRM_SDES_TYPE_NAME:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_NAME, value);
		break;

	case VMETA_STRM_SDES_TYPE_EMAIL:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_EMAIL, value);
		break;

	case VMETA_STRM_SDES_TYPE_PHONE:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_PHONE, value);
		break;

	case VMETA_STRM_SDES_TYPE_LOC:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_LOC, value);
		break;

	case VMETA_STRM_SDES_TYPE_TOOL:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_TOOL, value);
		break;

	case VMETA_STRM_SDES_TYPE_NOTE:
		ADD_ITEM(RTCP_PKT_SDES_TYPE_NOTE, value);
		break;

	case VMETA_STRM_SDES_TYPE_PRIV:
		ADD_PRIV_ITEM(prefix, value);
		break;

	default:
		break;
	}
}


int vstrm_session_metadata_write_rtcp_sdes(struct pomp_buffer *buf,
					   size_t *pos,
					   uint32_t ssrc,
					   const struct vmeta_session *meta)
{
	int res = 0;
	unsigned int i;
	struct rtcp_pkt_sdes sdes;
	struct rtcp_pkt_sdes_chunk chunk;
	struct rtcp_pkt_sdes_item *items = NULL;
	memset(&sdes, 0, sizeof(sdes));
	memset(&chunk, 0, sizeof(chunk));

	items = calloc(VSTRM_SDES_ITEMS_MAX_COUNT, sizeof(*items));
	if (items == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		return res;
	}

	sdes.chunk_count = 1;
	sdes.chunks = &chunk;
	chunk.ssrc = ssrc;
	chunk.items = items;

	res = vmeta_session_streaming_sdes_write(
		meta, vstrm_session_metadata_write_cb, &sdes);
	if (res < 0)
		ULOG_ERRNO("vmeta_session_streaming_sdes_write", -res);

	res = rtcp_pkt_write_sdes(buf, pos, &sdes);
	if (res < 0)
		ULOG_ERRNO("rtcp_pkt_write_sdes", -res);

	for (i = 0; i < chunk.item_count; i++) {
		if (chunk.items[i].type == RTCP_PKT_SDES_TYPE_PRIV) {
			free((void *)chunk.items[i].priv.prefix);
			free((void *)chunk.items[i].priv.value);
		} else {
			free((void *)chunk.items[i].data);
		}
	}

	free(items);
	return res;
}


#define COPY_ITEM(_field)                                                      \
	{                                                                      \
		strncpy(_field,                                                \
			(const char *)item->data,                              \
			item->data_len < sizeof(_field) ? item->data_len       \
							: sizeof(_field));     \
		_field[sizeof(_field) - 1] = '\0';                             \
	}


#define COPY_PRIV_ITEM(_prefix, _value)                                        \
	{                                                                      \
		strncpy(_prefix,                                               \
			(const char *)item->priv.prefix,                       \
			item->priv.prefix_len < sizeof(_prefix)                \
				? item->priv.prefix_len                        \
				: sizeof(_prefix));                            \
		_prefix[sizeof(_prefix) - 1] = '\0';                           \
		strncpy(_value,                                                \
			(const char *)item->priv.value,                        \
			item->priv.value_len < sizeof(_value)                  \
				? item->priv.value_len                         \
				: sizeof(_value));                             \
		_value[sizeof(_value) - 1] = '\0';                             \
	}


int vstrm_session_metadata_read_rtcp_sdes(const struct rtcp_pkt_sdes_item *item,
					  struct vmeta_session *meta)
{
	char prefix[64] = "";
	char value[64] = "";

	switch (item->type) {
	case RTCP_PKT_SDES_TYPE_CNAME:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_CNAME, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_NAME:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_NAME, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_EMAIL:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_EMAIL, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_PHONE:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_PHONE, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_LOC:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_LOC, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_TOOL:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_TOOL, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_NOTE:
		COPY_ITEM(value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_NOTE, value, NULL, meta);
		break;

	case RTCP_PKT_SDES_TYPE_PRIV:
		COPY_PRIV_ITEM(prefix, value);
		vmeta_session_streaming_sdes_read(
			VMETA_STRM_SDES_TYPE_PRIV, value, prefix, meta);
		break;

	default:
		break;
	}

	return 0;
}
