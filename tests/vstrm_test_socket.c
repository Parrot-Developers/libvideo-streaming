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

#define ULOG_TAG vstrm_test_socket
#include <ulog.h>
ULOG_DECLARE_TAG(vstrm_test_socket);


int vstrm_test_socket_setup(struct vstrm_test_socket *sock,
			    const char *local_addr,
			    uint16_t *local_port,
			    const char *remote_addr,
			    uint16_t remote_port,
			    struct pomp_loop *loop,
			    pomp_fd_event_cb_t fd_cb,
			    void *userdata)
{
	int res = 0;
	struct sockaddr_in addr;
	socklen_t addrlen = sizeof(addr);
	char dbg_local_addr[16];
	char dbg_remote_addr[16];
	const char *res1;

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(remote_addr == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fd_cb == NULL, EINVAL);

	/* Create socket */
	sock->fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock->fd < 0) {
		res = -errno;
		ULOG_ERRNO("socket", -res);
		goto error;
	}

	/* Setup flags */
	res = fd_set_close_on_exec(sock->fd);
	if (res < 0) {
		ULOG_ERRNO("fd_set_close_on_exec(%d)", -res, sock->fd);
		goto error;
	}
	res = fd_add_flags(sock->fd, O_NONBLOCK);
	if (res < 0) {
		ULOG_ERRNO("fd_add_flags(%d)", -res, sock->fd);
		goto error;
	}

	/* Setup local and remote addresses */
	memset(&sock->local_addr, 0, sizeof(sock->local_addr));
	sock->local_addr.sin_family = AF_INET;
	res = inet_pton(AF_INET, local_addr, &sock->local_addr.sin_addr);
	if (res <= 0) {
		ULOG_ERRNO("inet_pton", -res);
		goto error;
	}
	sock->local_addr.sin_port = htons(*local_port);
	memset(&sock->remote_addr, 0, sizeof(sock->remote_addr));
	sock->remote_addr.sin_family = AF_INET;
	res = inet_pton(AF_INET, remote_addr, &sock->remote_addr.sin_addr);
	if (res <= 0) {
		ULOG_ERRNO("inet_pton", -res);
		goto error;
	}
	sock->remote_addr.sin_port = htons(remote_port);

	/* Bind to local address */
retry_bind:
	if (bind(sock->fd,
		 (const struct sockaddr *)&sock->local_addr,
		 sizeof(sock->local_addr)) < 0) {
		res = -errno;
		if ((res == -EADDRINUSE) && (sock->local_addr.sin_port != 0)) {
			sock->local_addr.sin_port = 0;
			goto retry_bind;
		}
		ULOG_ERRNO("bind(%d)", -res, sock->fd);
		goto error;
	}

	/* Get the real bound address and port */
	res = getsockname(sock->fd, (struct sockaddr *)&addr, &addrlen);
	if (res < 0) {
		ULOG_ERRNO("getsockname(%d)", -res, sock->fd);
		goto error;
	}
	sock->local_addr = addr;
	*local_port = ntohs(addr.sin_port);

	/* Log the addresses and ports for debugging */
	dbg_local_addr[0] = '\0';
	dbg_remote_addr[0] = '\0';
	res1 = inet_ntop(AF_INET,
			 &sock->local_addr.sin_addr,
			 dbg_local_addr,
			 sizeof(dbg_local_addr));
	if (res1 == NULL)
		ULOG_ERRNO("inet_ntop", -errno);
	res1 = inet_ntop(AF_INET,
			 &sock->remote_addr.sin_addr,
			 dbg_remote_addr,
			 sizeof(dbg_remote_addr));
	if (res1 == NULL)
		ULOG_ERRNO("inet_ntop", -errno);
	ULOGI("fd=%d local %s:%d remote %s:%d",
	      sock->fd,
	      dbg_local_addr,
	      ntohs(sock->local_addr.sin_port),
	      dbg_remote_addr,
	      ntohs(sock->remote_addr.sin_port));

	/* Create the rx buffer */
	sock->rxbufsize = DEFAULT_RX_BUFFER_SIZE;
	sock->rxbuf = malloc(sock->rxbufsize);
	if (sock->rxbuf == NULL) {
		res = -ENOMEM;
		goto error;
	}

	/* Start monitoring for input */
	res = pomp_loop_add(loop, sock->fd, POMP_FD_EVENT_IN, fd_cb, userdata);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_add(%d)", -res, sock->fd);
		goto error;
	}

	/* Success */
	return 0;

	/* Cleanup in case of error */
error:
	if (sock->fd >= 0) {
		if (pomp_loop_has_fd(loop, sock->fd))
			pomp_loop_remove(loop, sock->fd);
		close(sock->fd);
		sock->fd = -1;
	}
	free(sock->rxbuf);
	sock->rxbuf = NULL;
	return res;
}


void vstrm_test_socket_cleanup(struct vstrm_test_socket *sock,
			       struct pomp_loop *loop)
{
	int res = 0;

	ULOG_ERRNO_RETURN_IF(sock == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(loop == NULL, EINVAL);

	if (sock->fd >= 0) {
		if (pomp_loop_has_fd(loop, sock->fd)) {
			res = pomp_loop_remove(loop, sock->fd);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_remove", -res);
		}
		close(sock->fd);
	}

	free(sock->rxbuf);
	memset(sock, 0, sizeof(*sock));
	sock->fd = -1;
}


int vstrm_test_socket_set_rx_size(struct vstrm_test_socket *sock, size_t size)
{
	int res = 0;
	int _size = size;

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);

	if (setsockopt(sock->fd, SOL_SOCKET, SO_RCVBUF, &_size, sizeof(_size)) <
	    0) {
		res = -errno;
		ULOG_ERRNO("setsockopt:SO_RCVBUF(%d)", -res, sock->fd);
		return res;
	}

	return 0;
}


int vstrm_test_socket_set_tx_size(struct vstrm_test_socket *sock, size_t size)
{
	int res = 0;
	int _size = size;

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);

	if (setsockopt(sock->fd, SOL_SOCKET, SO_SNDBUF, &_size, sizeof(_size)) <
	    0) {
		res = -errno;
		ULOG_ERRNO("setsockopt:SO_SNDBUF(%d)", -res, sock->fd);
	}

	return res;
}


int vstrm_test_socket_set_class(struct vstrm_test_socket *sock, int cls)
{
	int res = 0;
	int _cls = cls;

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);

	if (setsockopt(sock->fd, IPPROTO_IP, IP_TOS, &_cls, sizeof(_cls)) < 0) {
		res = -errno;
		ULOG_ERRNO("setsockopt:IP_TOS(%d)", -res, sock->fd);
	}

	return res;
}


ssize_t vstrm_test_socket_read(struct vstrm_test_socket *sock)
{
	ssize_t readlen = 0;
	struct sockaddr_in remote_addr;
	socklen_t remote_addr_len = sizeof(remote_addr);
	memset(&remote_addr, 0, sizeof(remote_addr));

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);

	/* Read data, ignoring interrupts */
	do {
		readlen = recvfrom(sock->fd,
				   sock->rxbuf,
				   sock->rxbufsize,
				   0,
				   (struct sockaddr *)&remote_addr,
				   &remote_addr_len);
	} while (readlen < 0 && errno == EINTR);

	if (readlen < 0) {
		readlen = -errno;
		if (errno != EAGAIN)
			ULOG_ERRNO("recvfrom(%d)", errno, sock->fd);
	}

	if ((readlen >= 0) && (sock->remote_addr.sin_port == 0)) {
		char dbg_local_addr[16];
		char dbg_remote_addr[16];
		const char *res1;

		sock->remote_addr = remote_addr;

		/* Log the addresses and ports for debugging */
		dbg_local_addr[0] = '\0';
		dbg_remote_addr[0] = '\0';
		res1 = inet_ntop(AF_INET,
				 &sock->local_addr.sin_addr,
				 dbg_local_addr,
				 sizeof(dbg_local_addr));
		if (res1 == NULL)
			ULOG_ERRNO("inet_ntop", -errno);
		res1 = inet_ntop(AF_INET,
				 &sock->remote_addr.sin_addr,
				 dbg_remote_addr,
				 sizeof(dbg_remote_addr));
		if (res1 == NULL)
			ULOG_ERRNO("inet_ntop", -errno);
		ULOGI("fd=%d local %s:%d remote %s:%d",
		      sock->fd,
		      dbg_local_addr,
		      ntohs(sock->local_addr.sin_port),
		      dbg_remote_addr,
		      ntohs(sock->remote_addr.sin_port));
	}

	return readlen;
}


ssize_t vstrm_test_socket_write(struct vstrm_test_socket *sock,
				const void *buf,
				size_t len)
{
	ssize_t writelen = 0;

	ULOG_ERRNO_RETURN_ERR_IF(sock == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	/* Write ignoring interrupts */
	do {
		writelen = sendto(sock->fd,
				  buf,
				  len,
				  0,
				  (struct sockaddr *)&sock->remote_addr,
				  sizeof(sock->remote_addr));
	} while (writelen < 0 && errno == EINTR);

	if (writelen >= 0) {
		if ((size_t)writelen != len) {
			ULOGW("partial write on fd=%d (%u/%u)",
			      sock->fd,
			      (unsigned int)writelen,
			      (unsigned int)len);
		}
	} else {
		writelen = -errno;
		if (errno != EAGAIN)
			ULOG_ERRNO("sendto(%d)", errno, sock->fd);
	}

	return writelen;
}
