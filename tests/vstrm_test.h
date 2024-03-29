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


#ifndef _VSTRM_TEST_H_
#define _VSTRM_TEST_H_

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <windows.h>
#	define IPTOS_PREC_INTERNETCONTROL 0xc0
#	define IPTOS_PREC_FLASHOVERRIDE 0x80
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <netinet/ip.h>
#	include <sys/mman.h>
#endif /* !_WIN32 */

#include <futils/futils.h>
#include <h264/h264.h>
#include <libpomp.h>
#include <transport-socket/tskt.h>
#include <video-streaming/vstrm.h>


#define DEFAULT_RCVBUF_SIZE 4096
#define DEFAULT_SNDBUF_SIZE 4096
#define DEFAULT_RX_BUFFER_SIZE 65536


struct vstrm_test_sender {
	pthread_t thread;
	int thread_launched;
	int thread_should_stop;
	struct pomp_loop *loop;
	struct pomp_timer *timer;
	const char *input_file;
#ifdef _WIN32
	HANDLE infile;
	HANDLE map;
#else
	int fd;
#endif
	void *data;
	size_t data_len;
	size_t data_off;
	int finished;
	uint8_t *sps;
	size_t sps_len;
	uint8_t *pps;
	size_t pps_len;
	struct h264_reader *reader;
	struct vstrm_sender *sender;
	struct tskt_socket *data_sock;
	struct tskt_socket *ctrl_sock;
	uint8_t *rx_buf;
	size_t rx_buf_len;
	struct vstrm_frame *frame;
	float framerate;
	uint32_t frame_interval_us;
	uint64_t timestamp;
	void (*finished_cb)(void *userdata);
	void *userdata;
};


struct vstrm_test_receiver {
	pthread_t thread;
	int thread_launched;
	int thread_should_stop;
	struct pomp_loop *loop;
	struct vstrm_receiver *receiver;
	struct tskt_socket *data_sock;
	struct tskt_socket *ctrl_sock;
	uint8_t *rx_buf;
	size_t rx_buf_len;
	FILE *file;
	void (*finished_cb)(void *userdata);
	void *userdata;
};


struct vstrm_test {
	struct pomp_loop *loop;
	int thread_should_stop;
	struct vstrm_test_sender *sender;
	struct vstrm_test_receiver *receiver;
};


int vstrm_test_sender_create(const char *file,
			     float framerate,
			     const char *local_addr,
			     uint16_t local_data_port,
			     uint16_t local_ctrl_port,
			     const char *remote_addr,
			     uint16_t remote_data_port,
			     uint16_t remote_ctrl_port,
			     void (*finished_cb)(void *userdata),
			     void *userdata,
			     struct vstrm_test_sender **ret_obj);


int vstrm_test_sender_join(struct vstrm_test_sender *self,
			   struct vstrm_sender_stats *stats);


int vstrm_test_sender_destroy(struct vstrm_test_sender *self);


int vstrm_test_receiver_create(const char *local_addr,
			       uint16_t *local_data_port,
			       uint16_t *local_ctrl_port,
			       const char *remote_addr,
			       uint16_t remote_data_port,
			       uint16_t remote_ctrl_port,
			       const char *file,
			       void (*finished_cb)(void *userdata),
			       void *userdata,
			       struct vstrm_test_receiver **ret_obj);


int vstrm_test_receiver_join(struct vstrm_test_receiver *self,
			     struct vstrm_receiver_stats *stats);


int vstrm_test_receiver_destroy(struct vstrm_test_receiver *self);


#endif /* _VSTRM_TEST_H_ */
