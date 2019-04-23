# libvideo-streaming - Video streaming library

libvideo-streaming is a C library that handles the payloading and de-payloading
of H.264 encoded video frames in an RTP stream.

## Features

### Sender

* H.264 encoded video payloading following RFC 6184, supporting:
  * single NAL unit packets
  * fragmentation packets (FU-A)
  * aggregation packets (STAP-A)
* serializing of Parrot video frame metadata as RTP header extensions
* serializing of Parrot video session metadata as RTCP SDES packets
* congestion control: packet drop on timeout with priority class
* statistics output:
  * RTCP receiver reports
  * Parrot receiver video stats
* no network implementation: the application must handle network
sending/receiving

### Receiver

* H.264 encoded video de-payloading following RFC 6184, supporting:
  * single NAL unit packets
  * fragmentation packets (FU-A)
  * aggregation packets (STAP-A)
* de-serializing of Parrot video frame metadata from RTP header extensions
* de-serializing of Parrot video session metadata from RTCP SDES packets
* pre-decoder error concealment: reconstructing missing slices as skipped
slices to ensure a syntactically complete H.264 bitstream for decoders
* statistics computing:
  * RTCP receiver reports
  * Parrot receiver video stats
* no network implementation: the application must handle network
sending/receiving

## Dependencies

The library depends on the following Alchemy modules:

* libfutils
* libh264
* libpomp
* librtp
* libulog
* libvideo-metadata

## Building

Building is activated by enabling _libvideo-streaming_ in the Alchemy build
configuration.

## Operation

### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation). All API functions must be called from the _pomp_loop_
thread. All callback functions are called from the _pomp_loop_ thread.

### Sender

The sender takes as input H.264 frames as a structure containing the NAL units
information. The RTP payloading is performed and the packets to send are output
through the _send_data_ callback function. The control packets (RTCP) to send
are output through the _send_ctrl_ callback function. Packets are handled as
_pomp_buffer_ objects.

### Receiver

The receiver takes as input data (RTP) and control (RTCP) packets. The RTP
de-payloading is performed and the reconstructed frames are output through the
_recv_frame_ callback function. The control packets (RTCP) to send are output
through the _send_ctrl_ callback function. Packets are handled as
_pomp_buffer_ objects.

## Testing

The library can be tested using the provided _vstrm-test_ command-line program
which sends an H.264 (annex B byte stream) file as an RTP/AVP stream and/or
receives an RTP/AVP stream.

To build the test program, enable _vstrm-test_ in the Alchemy build
configuration.

For a list of available options, run

    $ vstrm-test -h
