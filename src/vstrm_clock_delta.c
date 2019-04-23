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


int vstrm_clock_delta_write(struct pomp_buffer *buf,
			    size_t *pos,
			    const struct vstrm_clock_delta *delta)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);

	CHECK(vstrm_write_u64(buf, pos, delta->originate_ts));
	CHECK(vstrm_write_u64(buf, pos, delta->receive_ts));
	CHECK(vstrm_write_u64(buf, pos, delta->transmit_ts));

out:
	return res;
}


int vstrm_clock_delta_read(struct pomp_buffer *buf,
			   size_t *pos,
			   struct vstrm_clock_delta *delta)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);

	CHECK(vstrm_read_u64(buf, pos, &delta->originate_ts));
	CHECK(vstrm_read_u64(buf, pos, &delta->receive_ts));
	CHECK(vstrm_read_u64(buf, pos, &delta->transmit_ts));

out:
	return res;
}


void vstrm_clock_delta_init(struct vstrm_clock_delta_ctx *ctx)
{
	ctx->expected_originate_ts = 0;
	ctx->clock_delta_avg = 0;
	ctx->rt_delay_avg = 0;
	ctx->rt_delay_min_avg = 0;
	memset(ctx->clock_delta_window, 0, sizeof(ctx->clock_delta_window));
	memset(ctx->rt_delay_window, 0, sizeof(ctx->rt_delay_window));
	ctx->window_size = 0;
	ctx->window_pos = 0;
	ctx->window_start_ts = 0;
	ctx->window_min_rt_delay = INT64_MAX;
	ctx->window_clock_delta_of_min_rtd = 0;

	if (ctx->dbg_csv)
		fprintf(ctx->dbg_csv,
			"timestamp clock_delta rt_delay rt_delay_avg "
			"clock_delta_avg rt_delay_min_avg\n");
}


/**
 * Process the clock delta values; for more documentation, see
 * https://smet.parrot.biz/projects/drone-streaming/wiki/Clock_Delta
 */
int vstrm_clock_delta_process(struct vstrm_clock_delta_ctx *ctx,
			      const struct vstrm_clock_delta *delta,
			      uint64_t receive_ts)
{
	uint64_t originate_ts, peer_receive_ts, peer_transmit_ts;
	int64_t rt_delay, clock_delta;

	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(receive_ts == 0, EINVAL);

	/* Check the consistency of values */
	if (delta->originate_ts == 0) {
		ULOGD("clock_delta: null originate timestamp");
		return 0;
	}
	originate_ts = delta->originate_ts;
	if (delta->receive_ts == 0) {
		ULOGD("clock_delta: null peer receive timestamp");
		return 0;
	}
	peer_receive_ts = delta->receive_ts;
	if (delta->transmit_ts == 0) {
		ULOGD("clock_delta: null peer transmit timestamp");
		return 0;
	}
	peer_transmit_ts = delta->transmit_ts;
	if (originate_ts != ctx->expected_originate_ts) {
		ULOGD("clock_delta: unexpected originate timestamp "
		      "(%" PRIu64 " vs. %" PRIu64 ")",
		      originate_ts,
		      ctx->expected_originate_ts);
		return 0;
	}
	if (peer_transmit_ts < peer_receive_ts + CLOCK_DELTA_MIN_TS_DELTA) {
		ULOGD("clock_delta: peer transmit timestamp "
		      "too close to peer receive timestamp");
		return 0;
	}
	if (receive_ts < originate_ts + CLOCK_DELTA_MIN_TS_DELTA) {
		ULOGD("clock_delta: originate timestamp "
		      "too close to receive timestamp");
		return 0;
	}

	/* Local computation */
	rt_delay = ((int64_t)receive_ts - (int64_t)originate_ts) -
		   ((int64_t)peer_transmit_ts - (int64_t)peer_receive_ts);
	clock_delta = ((int64_t)peer_receive_ts + (int64_t)peer_transmit_ts -
		       (int64_t)originate_ts - (int64_t)receive_ts + 1) /
		      2;

	if (rt_delay <= 0) {
		ULOGD("clock_delta: invalid round trip delay (%" PRIi64 ")",
		      rt_delay);
		goto end;
	}

	/* Average RTD */
	if (ctx->rt_delay_avg == 0) {
		/* Initial value */
		ctx->rt_delay_avg = rt_delay;
	} else {
		/* Sliding average
		 * alpha = 1 / CLOCK_DELTA_AVG_ALPHA */
		ctx->rt_delay_avg =
			ctx->rt_delay_avg + (rt_delay - ctx->rt_delay_avg +
					     CLOCK_DELTA_AVG_ALPHA / 2) /
						    CLOCK_DELTA_AVG_ALPHA;
	}

	/* Average clock delta and min RTD */
	if (ctx->window_size == 0) {
		/* Initialize the window */
		ctx->clock_delta_window[ctx->window_pos] = clock_delta;
		ctx->rt_delay_window[ctx->window_pos] = rt_delay;
		if (ctx->window_pos == 0) {
			/* First value in window */
			ctx->window_start_ts = receive_ts;
			ctx->window_min_rt_delay = rt_delay;
			ctx->window_clock_delta_of_min_rtd = clock_delta;
			ctx->rt_delay_min_avg = rt_delay;
			ctx->clock_delta_avg = clock_delta;
			ctx->clock_delta_valid = 1;
		} else if (rt_delay < ctx->window_min_rt_delay) {
			/* New min found */
			ctx->window_min_rt_delay = rt_delay;
			ctx->window_clock_delta_of_min_rtd = clock_delta;
		}
		ctx->window_pos++;

		if (ctx->window_pos >= CLOCK_DELTA_WINDOW_SIZE ||
		    receive_ts >=
			    ctx->window_start_ts + CLOCK_DELTA_WINDOW_TIMEOUT) {
			/* The window reached its minimum size */
			ctx->window_size = ctx->window_pos;
			ctx->window_pos = 0;
			ctx->rt_delay_min_avg = ctx->window_min_rt_delay;
			ctx->clock_delta_avg =
				ctx->window_clock_delta_of_min_rtd;
		} else {
			/* Initialization phase: alpha is 100% minus
			 * the filling percentage of the window */
			uint32_t perc_time =
				(receive_ts - ctx->window_start_ts) * 100 /
				CLOCK_DELTA_WINDOW_TIMEOUT;
			uint32_t perc_window =
				ctx->window_pos * 100 / CLOCK_DELTA_WINDOW_SIZE;
			uint32_t perc = perc_time > perc_window ? perc_time
								: perc_window;
			ctx->rt_delay_min_avg += (100 - perc) *
						 (ctx->window_min_rt_delay -
						  ctx->rt_delay_min_avg) /
						 100;
			ctx->clock_delta_avg +=
				(100 - perc) *
				(ctx->window_clock_delta_of_min_rtd -
				 ctx->clock_delta_avg) /
				100;
		}
	} else {
		/* Remember the old values and set the new ones */
		int64_t old_rt_delay = ctx->rt_delay_window[ctx->window_pos];
		ctx->clock_delta_window[ctx->window_pos] = clock_delta;
		ctx->rt_delay_window[ctx->window_pos] = rt_delay;

		if (rt_delay < ctx->window_min_rt_delay) {
			/* New min found */
			ctx->window_min_rt_delay = rt_delay;
			ctx->window_clock_delta_of_min_rtd = clock_delta;
		} else if (old_rt_delay == ctx->window_min_rt_delay) {
			/* We replace the current min value, find the new min */
			ctx->window_min_rt_delay = INT64_MAX;
			for (int i = 0; i < ctx->window_size; i++) {
				if (ctx->rt_delay_window[i] >
				    ctx->window_min_rt_delay) {
					/* Not interesting, go to next value */
					continue;
				}

				/* Save new min */
				ctx->window_min_rt_delay =
					ctx->rt_delay_window[i];
				ctx->window_clock_delta_of_min_rtd =
					ctx->clock_delta_window[i];

				if (ctx->rt_delay_window[i] == old_rt_delay) {
					/* We found the old min again, break */
					break;
				}

				/* Otherwise, continue searching */
			}
		}

		/* Update position and wrap if needed */
		ctx->window_pos++;
		if (ctx->window_pos >= ctx->window_size)
			ctx->window_pos = 0;

		if (ctx->window_min_rt_delay > CLOCK_DELTA_MAX_RTDELAY) {
			ULOGD("clock_delta: minimum round trip delay is "
			      "too big (%" PRIi64 ")",
			      ctx->window_min_rt_delay);
			goto end;
		}

		/* Average min RTD: sliding average
		 * alpha = 1 / CLOCK_DELTA_AVG_ALPHA */
		ctx->rt_delay_min_avg +=
			(ctx->window_min_rt_delay - ctx->rt_delay_min_avg +
			 CLOCK_DELTA_AVG_ALPHA / 2) /
			CLOCK_DELTA_AVG_ALPHA;

		if (ctx->window_min_rt_delay > ctx->rt_delay_min_avg * 2) {
			ULOGD("clock_delta: minimum round trip delay is "
			      "more than 2x the average RTD "
			      "(%" PRIi64 " vs %" PRIi64 ")",
			      ctx->window_min_rt_delay,
			      ctx->rt_delay_min_avg);
			goto end;
		}

		/* Average clock delta: sliding average
		 * alpha = 1 / CLOCK_DELTA_AVG_ALPHA */
		ctx->clock_delta_avg +=
			(ctx->window_clock_delta_of_min_rtd -
			 ctx->clock_delta_avg + CLOCK_DELTA_AVG_ALPHA / 2) /
			CLOCK_DELTA_AVG_ALPHA;
	}

end:
	ctx->expected_originate_ts = 0;

	if (ctx->dbg_csv)
		fprintf(ctx->dbg_csv,
			"%" PRIu64 " %" PRIi64 " %" PRIi64 " %" PRIi64
			" %" PRIi64 " %" PRIi64 "\n",
			receive_ts,
			clock_delta,
			rt_delay,
			ctx->rt_delay_avg,
			ctx->clock_delta_avg,
			ctx->rt_delay_min_avg);

	return 0;
}
