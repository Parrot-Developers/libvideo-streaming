#!/usr/bin/env python

##
# Copyright (c) 2016 Parrot Drones SAS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Parrot Drones SAS Company nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##

import sys
import struct

import socket
import dpkt

_SRC_ADDR = socket.inet_aton("192.168.1.1")
_DST_ADDR = socket.inet_aton("192.168.1.2")
_SRC_PORT = 5004
_DST_PORT = 55004

def main():
    infilepath = sys.argv[1]
    outfilepath = sys.argv[1] + ".pcap"
    print("Opening '%s" % infilepath)
    with open(infilepath, "rb") as fin:
        print("Creating '%s" % outfilepath)
        with open(outfilepath, "wb") as fout:
            writer = dpkt.pcap.Writer(fout)
            while True:
                # Read buffer size (4 bytes big endian), then buffer itself
                buf = fin.read(4)
                if len(buf) < 4:
                    break
                buflen = struct.unpack(">I", buf)[0]
                buf = fin.read(buflen)

                # UDP layer
                udp_pkt = dpkt.udp.UDP()
                udp_pkt.data = buf
                udp_pkt.ulen = dpkt.udp.UDP_HDR_LEN + len(buf)
                udp_pkt.dport = _DST_PORT
                udp_pkt.sport = _SRC_PORT

                # IP layer
                ip_pkt = dpkt.ip.IP()
                ip_pkt.data = udp_pkt
                ip_pkt.src = _SRC_ADDR
                ip_pkt.dst = _DST_ADDR
                ip_pkt.p = dpkt.ip.IP_PROTO_UDP
                ip_pkt.len = dpkt.ip.IP_HDR_LEN + udp_pkt.ulen

                # Ethernet layer
                eth_pkt = dpkt.ethernet.Ethernet()
                eth_pkt.data = ip_pkt

                writer.writepkt(eth_pkt)

if __name__ == "__main__":
    main()
