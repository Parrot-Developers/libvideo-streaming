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

#include "vstrm_test.h"

#define ULOG_TAG vstrm_test
#include <ulog.h>
ULOG_DECLARE_TAG(vstrm_test);


#define DEFAULT_ADDR "0.0.0.0"
#define DEFAULT_RECV_DATA_PORT 55004
#define DEFAULT_RECV_CTRL_PORT 55005


static struct vstrm_test *s_self;


static const char short_options[] = "hi:f:a:ro:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"send-file", required_argument, NULL, 'i'},
	{"send-framerate", required_argument, NULL, 'f'},
	{"send-addr", required_argument, NULL, 'a'},
	{"recv", no_argument, NULL, 'r'},
	{"recv-file", required_argument, NULL, 'o'},
	{0, 0, 0, 0},
};


/* Win32 stubs */
#ifdef _WIN32
static inline const char *strsignal(int signum)
{
	return "??";
}
#endif /* _WIN32 */


static void sig_handler(int signum)
{
	ULOGI("signal %d(%s) received", signum, strsignal(signum));

	if (s_self == NULL)
		return;

	s_self->thread_should_stop = 1;
	if (s_self->loop != NULL)
		pomp_loop_wakeup(s_self->loop);
}


static void sender_finished_cb(void *userdata)
{
	struct vstrm_test *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("sender finished");

	self->thread_should_stop = 1;
	if (self->loop != NULL)
		pomp_loop_wakeup(self->loop);
}


static void receiver_finished_cb(void *userdata)
{
	struct vstrm_test *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("receiver finished");

	self->thread_should_stop = 1;
	if (self->loop != NULL)
		pomp_loop_wakeup(self->loop);
}


static void welcome(char *prog_name)
{
	printf("\n%s - Video streaming library test program\n"
	       "Copyright (c) 2016 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	/* clang-format off */
	printf("Usage: %s [options]\n"
	       "Options:\n"
	       "  -h | --help                        "
		       "Print this message\n"
	       "  -i | --send-file <file_name>       "
		       "Sender H.264 (Annex B byte stream) "
		       "input file (.264/.h264)\n"
	       "  -f | --send-framerate <framerate>  "
		       "Sender video framerate (float, 0 means fastest)\n"
	       "  -a | --send-addr <address>  "
		       "Sender destination address (IPv4 \"x.y.z.t\")\n"
	       "  -r | --recv                        "
		       "Enable receiver\n"
	       "  -o | --recv-file <file_name>       "
		       "Receiver H.264 (Annex B byte stream) "
		       "output file (.264/.h264) - enables the receiver\n"
	       "\n",
	       prog_name);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int res = 0, status = EXIT_SUCCESS;
	int idx, c;
	struct vstrm_test *self = NULL;
	int receive = 0;
	char *send_file = NULL, *recv_file = NULL;
	float send_framerate = 0.;
	char *send_addr = DEFAULT_ADDR;
	char *recv_addr = DEFAULT_ADDR;
	uint16_t send_data_port = 0;
	uint16_t send_ctrl_port = 0;
	uint16_t recv_data_port = DEFAULT_RECV_DATA_PORT;
	uint16_t recv_ctrl_port = DEFAULT_RECV_CTRL_PORT;
	uint64_t start_time = 0, end_time = 0;
	struct vstrm_sender_stats sender_stats;
	struct vstrm_receiver_stats receiver_stats;
	struct timespec cur_ts = {0, 0};
	float time_s, sender_bitrate_mbits, receiver_bitrate_mbits;
	uint32_t sender_packet_rate, receiver_packet_rate;
	memset(&sender_stats, 0, sizeof(sender_stats));
	memset(&receiver_stats, 0, sizeof(receiver_stats));

	welcome(argv[0]);

	/* Context allocation */
	s_self = NULL;
	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_self = self;

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			status = EXIT_SUCCESS;
			goto out;

		case 'i':
			send_file = optarg;
			break;

		case 'f':
			send_framerate = atof(optarg);
			break;

		case 'a':
			recv_addr = optarg;
			break;

		case 'r':
			receive = 1;
			break;

		case 'o':
			recv_file = optarg;
			receive = 1;
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	/* Setup signal handlers */
	signal(SIGINT, &sig_handler);
	signal(SIGTERM, &sig_handler);
#ifndef _WIN32
	signal(SIGPIPE, SIG_IGN);
#endif

	/* Loop */
	self->loop = pomp_loop_new();
	if (self->loop == NULL) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Receiver */
	if (receive) {
		res = vstrm_test_receiver_create(DEFAULT_ADDR,
						 &recv_data_port,
						 &recv_ctrl_port,
						 DEFAULT_ADDR,
						 send_data_port,
						 send_ctrl_port,
						 recv_file,
						 receiver_finished_cb,
						 self,
						 &self->receiver);
		if (res < 0) {
			ULOG_ERRNO("vstrm_test_receiver_create", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	/* Sender */
	if (send_file != NULL) {
		res = vstrm_test_sender_create(send_file,
					       send_framerate,
					       send_addr,
					       0,
					       0,
					       recv_addr,
					       recv_data_port,
					       recv_ctrl_port,
					       sender_finished_cb,
					       self,
					       &self->sender);
		if (res < 0) {
			ULOG_ERRNO("vstrm_test_sender_create", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &start_time);

	while (!self->thread_should_stop)
		pomp_loop_wait_and_process(self->loop, -1);

	if (self->sender != NULL) {
		res = vstrm_test_sender_join(self->sender, &sender_stats);
		if (res < 0)
			ULOG_ERRNO("vstrm_test_sender_join", -res);
	}
	if (self->receiver != NULL) {
		res = vstrm_test_receiver_join(self->receiver, &receiver_stats);
		if (res < 0)
			ULOG_ERRNO("vstrm_test_receiver_join", -res);
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &end_time);

	time_s = (float)(end_time - start_time) / 1000000.;
	sender_packet_rate =
		(time_s)
			? (uint32_t)((uint64_t)sender_stats.total_packet_count *
				     1000000 / (end_time - start_time))
			: 0;
	sender_bitrate_mbits = (time_s != 0.)
				       ? (float)sender_stats.total_byte_count *
						 8. / time_s / 1000000.
				       : 0;
	receiver_packet_rate =
		(time_s) ? (uint32_t)((uint64_t)receiver_stats
					      .received_packet_count *
				      1000000 / (end_time - start_time))
			 : 0;
	receiver_bitrate_mbits =
		(time_s != 0.) ? (float)receiver_stats.received_byte_count *
					 8. / time_s / 1000000.
			       : 0;

	printf("\nOverall time:     %.2fs\n", time_s);
	printf("\nOverall sent:     %" PRIu32
	       " packets (%d pkt/s),\n"
	       "                  %" PRIu32 " bytes (%.1f Mbit/s)\n",
	       sender_stats.total_packet_count,
	       sender_packet_rate,
	       sender_stats.total_byte_count,
	       sender_bitrate_mbits);
	printf("\nOverall received: %" PRIu32
	       " packets (%d pkt/s),\n"
	       "                  %" PRIu32 " bytes (%.1f Mbit/s)\n",
	       receiver_stats.received_packet_count,
	       receiver_packet_rate,
	       receiver_stats.received_byte_count,
	       receiver_bitrate_mbits);
	printf("\n");

out:
	/* Cleanup and exit */
	if (self != NULL) {
		if (self->sender != NULL) {
			res = vstrm_test_sender_destroy(self->sender);
			if (res < 0)
				ULOG_ERRNO("vstrm_test_sender_destroy", -res);
		}
		if (self->receiver != NULL) {
			res = vstrm_test_receiver_destroy(self->receiver);
			if (res < 0)
				ULOG_ERRNO("vstrm_test_receiver_destroy", -res);
		}
		if (self->loop != NULL) {
			res = pomp_loop_destroy(self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_destroy", -res);
		}
		free(self);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Done!" : "Failed!");
	exit(status);
}
