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

#ifndef _VSTRM_VIDEO_STATS_PRIV_H_
#define _VSTRM_VIDEO_STATS_PRIV_H_


int vstrm_video_stats_write(struct pomp_buffer *buf,
			    size_t *pos,
			    const struct vstrm_video_stats *meta,
			    const struct vstrm_video_stats_dyn *dyn);


int vstrm_video_stats_read(struct pomp_buffer *buf,
			   size_t *pos,
			   struct vstrm_video_stats *meta,
			   struct vstrm_video_stats_dyn *dyn);


void vstrm_video_stats_csv_header(FILE *csv,
				  uint32_t mb_status_class_count,
				  uint32_t mb_status_zone_count);


void vstrm_video_stats_csv_write(FILE *csv,
				 const struct vstrm_video_stats *meta,
				 const struct vstrm_video_stats_dyn *dyn);


static inline void
vstrm_video_stats_dyn_inc_mb_status_count(struct vstrm_video_stats_dyn *dyn,
					  uint32_t mb_class,
					  uint32_t mb_zone)
{
	dyn->macroblock_status[mb_class * dyn->mb_status_zone_count +
			       mb_zone]++;
}


static inline void vstrm_video_stats_dyn_inc_errored_second_count_by_zone(
	struct vstrm_video_stats_dyn *dyn,
	uint32_t mb_zone)
{
	dyn->errored_second_count_by_zone[mb_zone]++;
}


#endif /* !_VSTRM_VIDEO_STATS_PRIV_H_ */
