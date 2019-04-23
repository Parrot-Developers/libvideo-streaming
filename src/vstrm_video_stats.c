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
/* codecheck_ignore[COMPLEX_MACRO] */
#define CHECK(_x) if ((res = (_x)) < 0) goto out
/* clang-format on */


int vstrm_video_stats_write(struct pomp_buffer *buf,
			    size_t *pos,
			    const struct vstrm_video_stats *meta,
			    const struct vstrm_video_stats_dyn *dyn)
{
	int res = 0;
	uint8_t reserved1 = 0;
	uint8_t reserved2 = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dyn == NULL, EINVAL);

	if (meta->version != VSTRM_VIDEO_STATS_VERSION) {
		ULOGE("video_stats: bad version: %u (%u)",
		      meta->version,
		      VSTRM_VIDEO_STATS_VERSION);
		res = -EIO;
		goto out;
	}

	CHECK(vstrm_write_u8(buf, pos, meta->version));
	CHECK(vstrm_write_i8(buf, pos, meta->rssi));
	CHECK(vstrm_write_u8(buf, pos, reserved1));
	CHECK(vstrm_write_u8(buf, pos, reserved2));
	CHECK(vstrm_write_u64(buf, pos, meta->timestamp));
	CHECK(vstrm_write_u32(buf, pos, meta->total_frame_count));
	CHECK(vstrm_write_u32(buf, pos, meta->output_frame_count));
	CHECK(vstrm_write_u32(buf, pos, meta->errored_output_frame_count));
	CHECK(vstrm_write_u32(buf, pos, meta->missed_frame_count));
	CHECK(vstrm_write_u32(buf, pos, meta->discarded_frame_count));
	CHECK(vstrm_write_u64(buf, pos, meta->timestamp_delta_integral));
	CHECK(vstrm_write_u64(buf, pos, meta->timestamp_delta_integral_sq));
	CHECK(vstrm_write_u64(buf, pos, meta->timing_error_integral));
	CHECK(vstrm_write_u64(buf, pos, meta->timing_error_integral_sq));
	CHECK(vstrm_write_u64(buf, pos, meta->estimated_latency_integral));
	CHECK(vstrm_write_u64(buf, pos, meta->estimated_latency_integral_sq));
	CHECK(vstrm_write_u32(buf, pos, meta->errored_second_count));
	CHECK(vstrm_write_u32(buf, pos, meta->mb_status_class_count));
	CHECK(vstrm_write_u32(buf, pos, meta->mb_status_zone_count));

	/* Check that fields match */
	if (meta->mb_status_class_count != dyn->mb_status_class_count) {
		ULOGE("video_stats: mb_status_class_count mismatch: %u (%u)",
		      dyn->mb_status_class_count,
		      meta->mb_status_class_count);
		res = -EINVAL;
		goto out;
	}
	if (meta->mb_status_zone_count != dyn->mb_status_zone_count) {
		ULOGE("video_stats: mb_status_zone_count mismatch: %u (%u)",
		      dyn->mb_status_zone_count,
		      meta->mb_status_zone_count);
		res = -EINVAL;
		goto out;
	}

	/* 1 dimension array */
	for (uint32_t i = 0; i < meta->mb_status_zone_count; i++) {
		CHECK(vstrm_write_u32(
			buf, pos, dyn->errored_second_count_by_zone[i]));
	}

	/* 2 dimensions array */
	for (uint32_t j = 0; j < meta->mb_status_class_count; j++) {
		for (uint32_t i = 0; i < meta->mb_status_zone_count; i++) {
			uint32_t k = j * meta->mb_status_zone_count + i;
			CHECK(vstrm_write_u32(
				buf, pos, dyn->macroblock_status[k]));
		}
	}

out:
	return 0;
}


int vstrm_video_stats_read(struct pomp_buffer *buf,
			   size_t *pos,
			   struct vstrm_video_stats *meta,
			   struct vstrm_video_stats_dyn *dyn)
{
	int res = 0;
	uint8_t reserved1 = 0;
	uint8_t reserved2 = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dyn == NULL, EINVAL);

	/* Read and check version */
	CHECK(vstrm_read_u8(buf, pos, &meta->version));
	if (meta->version != VSTRM_VIDEO_STATS_VERSION) {
		ULOGE("video_stats: bad version: %u (%u)",
		      meta->version,
		      VSTRM_VIDEO_STATS_VERSION);
		res = -EIO;
		goto out;
	}

	CHECK(vstrm_read_i8(buf, pos, &meta->rssi));
	CHECK(vstrm_read_u8(buf, pos, &reserved1));
	CHECK(vstrm_read_u8(buf, pos, &reserved2));
	CHECK(vstrm_read_u64(buf, pos, &meta->timestamp));
	CHECK(vstrm_read_u32(buf, pos, &meta->total_frame_count));
	CHECK(vstrm_read_u32(buf, pos, &meta->output_frame_count));
	CHECK(vstrm_read_u32(buf, pos, &meta->errored_output_frame_count));
	CHECK(vstrm_read_u32(buf, pos, &meta->missed_frame_count));
	CHECK(vstrm_read_u32(buf, pos, &meta->discarded_frame_count));
	CHECK(vstrm_read_u64(buf, pos, &meta->timestamp_delta_integral));
	CHECK(vstrm_read_u64(buf, pos, &meta->timestamp_delta_integral_sq));
	CHECK(vstrm_read_u64(buf, pos, &meta->timing_error_integral));
	CHECK(vstrm_read_u64(buf, pos, &meta->timing_error_integral_sq));
	CHECK(vstrm_read_u64(buf, pos, &meta->estimated_latency_integral));
	CHECK(vstrm_read_u64(buf, pos, &meta->estimated_latency_integral_sq));
	CHECK(vstrm_read_u32(buf, pos, &meta->errored_second_count));
	CHECK(vstrm_read_u32(buf, pos, &meta->mb_status_class_count));
	CHECK(vstrm_read_u32(buf, pos, &meta->mb_status_zone_count));

	/* Initialize dynamic structure if needed */
	if (dyn->mb_status_class_count == 0 && dyn->mb_status_zone_count == 0) {
		res = vstrm_video_stats_dyn_init(dyn,
						 meta->mb_status_class_count,
						 meta->mb_status_zone_count);
		if (res < 0)
			goto out;
	}

	/* Check that fields match */
	/* TODO: automatically resize if needed ? */
	if (meta->mb_status_class_count != dyn->mb_status_class_count) {
		ULOGE("video_stats: mb_status_class_count mismatch: %u (%u)",
		      dyn->mb_status_class_count,
		      meta->mb_status_class_count);
		res = -EINVAL;
		goto out;
	}
	if (meta->mb_status_zone_count != dyn->mb_status_zone_count) {
		ULOGE("video_stats: mb_status_zone_count mismatch: %u (%u)",
		      dyn->mb_status_zone_count,
		      meta->mb_status_zone_count);
		res = -EINVAL;
		goto out;
	}

	/* 1 dimension array */
	for (uint32_t i = 0; i < meta->mb_status_zone_count; i++) {
		CHECK(vstrm_read_u32(
			buf, pos, &dyn->errored_second_count_by_zone[i]));
	}

	/* 2 dimensions array */
	for (uint32_t j = 0; j < meta->mb_status_class_count; j++) {
		for (uint32_t i = 0; i < meta->mb_status_zone_count; i++) {
			uint32_t k = j * meta->mb_status_zone_count + i;
			CHECK(vstrm_read_u32(
				buf, pos, &dyn->macroblock_status[k]));
		}
	}

out:
	return 0;
}


int vstrm_video_stats_dyn_init(struct vstrm_video_stats_dyn *dyn,
			       uint32_t mb_status_class_count,
			       uint32_t mb_status_zone_count)
{
	ULOG_ERRNO_RETURN_ERR_IF(dyn == NULL, EINVAL);

	memset(dyn, 0, sizeof(*dyn));
	dyn->mb_status_class_count = mb_status_class_count;
	dyn->mb_status_zone_count = mb_status_zone_count;

	dyn->errored_second_count_by_zone =
		calloc(dyn->mb_status_zone_count, sizeof(uint32_t));
	if (dyn->errored_second_count_by_zone == NULL)
		goto error;

	dyn->macroblock_status =
		calloc(dyn->mb_status_class_count * dyn->mb_status_zone_count,
		       sizeof(uint32_t));
	if (dyn->macroblock_status == NULL)
		goto error;

	return 0;

	/* Cleanup in case of error */
error:
	free(dyn->errored_second_count_by_zone);
	free(dyn->macroblock_status);
	memset(dyn, 0, sizeof(*dyn));
	return -ENOMEM;
}


int vstrm_video_stats_dyn_clear(struct vstrm_video_stats_dyn *dyn)
{
	ULOG_ERRNO_RETURN_ERR_IF(dyn == NULL, EINVAL);
	free(dyn->errored_second_count_by_zone);
	free(dyn->macroblock_status);
	memset(dyn, 0, sizeof(*dyn));
	return 0;
}


VSTRM_API
int vstrm_video_stats_dyn_copy(struct vstrm_video_stats_dyn *dst,
			       const struct vstrm_video_stats_dyn *src)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(dst == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(src == NULL, EINVAL);

	/* Allocate destination internal arrays */
	res = vstrm_video_stats_dyn_init(
		dst, src->mb_status_class_count, src->mb_status_zone_count);
	if (res < 0)
		return res;

	/* Copy internal arrays */
	memcpy(dst->errored_second_count_by_zone,
	       src->errored_second_count_by_zone,
	       src->mb_status_zone_count * sizeof(uint32_t));
	memcpy(dst->macroblock_status,
	       src->macroblock_status,
	       src->mb_status_class_count * src->mb_status_zone_count *
		       sizeof(uint32_t));
	return 0;
}


void vstrm_video_stats_csv_header(FILE *csv,
				  uint32_t mb_status_class_count,
				  uint32_t mb_status_zone_count)
{
	uint32_t i, j;

	if (csv == NULL)
		return;

	fprintf(csv,
		"timestamp rssi totalFrameCount "
		"outputFrameCount erroredOutputFrameCount "
		"discardedFrameCount missedFrameCount "
		"timestampDeltaIntegral timestampDeltaIntegralSq "
		"timingErrorIntegral timingErrorIntegralSq "
		"estimatedLatencyIntegral estimatedLatencyIntegralSq "
		"erroredSecondCount");
	for (i = 0; i < mb_status_zone_count; i++)
		fprintf(csv, " erroredSecondCountByZone[%d]", i);
	for (j = 0; j < mb_status_class_count; j++) {
		for (i = 0; i < mb_status_zone_count; i++)
			fprintf(csv, " macroblockStatus[%d][%d]", j, i);
	}
	fprintf(csv, "\n");
}


void vstrm_video_stats_csv_write(FILE *csv,
				 const struct vstrm_video_stats *meta,
				 const struct vstrm_video_stats_dyn *dyn)
{
	uint32_t i, j;

	if (csv == NULL)
		return;

	ULOG_ERRNO_RETURN_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(dyn == NULL, EINVAL);

	if (meta->version != VSTRM_VIDEO_STATS_VERSION) {
		ULOGE("video_stats: bad version: %u (%u)",
		      meta->version,
		      VSTRM_VIDEO_STATS_VERSION);
		return;
	}

	/* Check that fields match */
	if (meta->mb_status_class_count != dyn->mb_status_class_count) {
		ULOGE("video_stats: mb_status_class_count mismatch: %u (%u)",
		      dyn->mb_status_class_count,
		      meta->mb_status_class_count);
		return;
	}
	if (meta->mb_status_zone_count != dyn->mb_status_zone_count) {
		ULOGE("video_stats: mb_status_zone_count mismatch: %u (%u)",
		      dyn->mb_status_zone_count,
		      meta->mb_status_zone_count);
		return;
	}

	fprintf(csv,
		"%" PRIu64 " %i %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32
		" %" PRIu32 " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64
		" %" PRIu64 " %" PRIu64 " %" PRIu32,
		meta->timestamp,
		meta->rssi,
		meta->total_frame_count,
		meta->output_frame_count,
		meta->errored_output_frame_count,
		meta->discarded_frame_count,
		meta->missed_frame_count,
		meta->timestamp_delta_integral,
		meta->timestamp_delta_integral_sq,
		meta->timing_error_integral,
		meta->timing_error_integral_sq,
		meta->estimated_latency_integral,
		meta->estimated_latency_integral_sq,
		meta->errored_second_count);
	for (i = 0; i < meta->mb_status_zone_count; i++)
		fprintf(csv, " %" PRIu32, dyn->errored_second_count_by_zone[i]);
	for (j = 0; j < meta->mb_status_class_count; j++) {
		for (i = 0; i < meta->mb_status_zone_count; i++) {
			uint32_t k = j * meta->mb_status_zone_count + i;
			fprintf(csv, " %" PRIu32, dyn->macroblock_status[k]);
		}
	}
	fprintf(csv, "\n");
}
