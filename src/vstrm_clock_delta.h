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

#ifndef _VSTRM_CLOCK_DELTA_H_
#define _VSTRM_CLOCK_DELTA_H_


#define RTCP_CLOCK_DELTA_PERIOD_US 500000
#define CLOCK_DELTA_MIN_TS_DELTA 1000
#define CLOCK_DELTA_WINDOW_SIZE 10
#define CLOCK_DELTA_WINDOW_TIMEOUT                                             \
	(RTCP_CLOCK_DELTA_PERIOD_US * CLOCK_DELTA_WINDOW_SIZE)
#define CLOCK_DELTA_MAX_RTDELAY 500000
#define CLOCK_DELTA_AVG_ALPHA 32


struct vstrm_clock_delta {
	uint64_t originate_ts;
	uint64_t receive_ts;
	uint64_t transmit_ts;
};


struct vstrm_clock_delta_ctx {
	uint64_t expected_originate_ts;
	int64_t clock_delta_avg;
	int clock_delta_valid;
	int64_t rt_delay_avg;
	int64_t rt_delay_min_avg;
	int64_t clock_delta_window[CLOCK_DELTA_WINDOW_SIZE];
	int64_t rt_delay_window[CLOCK_DELTA_WINDOW_SIZE];
	int window_size;
	int window_pos;
	uint64_t window_start_ts;
	int64_t window_min_rt_delay;
	int64_t window_clock_delta_of_min_rtd;
	FILE *dbg_csv;
};


int vstrm_clock_delta_write(struct pomp_buffer *buf,
			    size_t *pos,
			    const struct vstrm_clock_delta *meta);


int vstrm_clock_delta_read(struct pomp_buffer *buf,
			   size_t *pos,
			   struct vstrm_clock_delta *meta);


void vstrm_clock_delta_init(struct vstrm_clock_delta_ctx *ctx);


int vstrm_clock_delta_process(struct vstrm_clock_delta_ctx *ctx,
			      const struct vstrm_clock_delta *delta,
			      uint64_t receive_ts);


#endif /* !_VSTRM_CLOCK_DELTA_H_ */
