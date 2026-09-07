/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"

#include <video-metadata/vmeta_frame_proto.h>


static void fill_nalu_bytes(uint8_t *buf, size_t len, uint8_t nal_type)
{
	buf[0] = (uint8_t)((3 << 5) | (nal_type & 0x1f));
	for (size_t i = 1; i < len; i++)
		buf[i] = (uint8_t)(i & 0xff);
}


static struct rtp_pkt *nth_pkt(struct list_node *packets, int n)
{
	struct rtp_pkt *pkt;
	int i = 0;
	list_walk_entry_forward(packets, pkt, node)
	{
		if (i == n)
			return pkt;
		i++;
	}
	return NULL;
}


static void free_packets(struct list_node *packets)
{
	struct rtp_pkt *pkt, *tmp;
	list_walk_entry_forward_safe(packets, pkt, tmp, node)
	{
		list_del(&pkt->node);
		rtp_pkt_destroy(pkt);
	}
}


static void test_tx_single_nalu_packetization(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct vstrm_rtp_h264_tx_stats stats;
	struct list_node packets;
	struct vstrm_frame *frame;
	uint8_t data[100];
	struct vstrm_frame_nalu nalu;
	struct rtp_pkt *pkt;

	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(list_length(&packets), 1);

	pkt = nth_pkt(&packets, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(pkt->payload.len, sizeof(data));
	CU_ASSERT_EQUAL(
		RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, PAYLOAD_TYPE),
		VSTRM_RTP_H264_PAYLOAD_TYPE);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, MARKER), 1);

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(stats.single_nalu_packet_count, 1);
	CU_ASSERT_EQUAL(stats.stap_packet_count, 0);
	CU_ASSERT_EQUAL(stats.fu_packet_count, 0);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_fu_a_fragmentation(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct vstrm_rtp_h264_tx_stats stats;
	struct list_node packets;
	struct vstrm_frame *frame;
	uint8_t data[250];
	struct vstrm_frame_nalu nalu;
	struct rtp_pkt *pkt;
	const void *cdata;
	size_t len;
	uint8_t reassembled[250];
	size_t rpos = 0;

	/* target_packet_size=100, nalu len=250: expect 3 FU-A fragments
	 * (86 + 86 + 77 bytes of the 249-byte payload after the 1-byte
	 * original NALU header, as derived by hand from the fragmentation
	 * loop in vstrm_rtp_h264_tx_add_nalu()) */
	cfg.dyn.target_packet_size = 100;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(list_length(&packets), 3);

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(stats.fu_packet_count, 3);
	CU_ASSERT_EQUAL(stats.single_nalu_packet_count, 0);
	CU_ASSERT_EQUAL(stats.stap_packet_count, 0);

	for (int i = 0; i < 3; i++) {
		uint8_t fu_ind, fu_hdr;
		bool start, end;

		pkt = nth_pkt(&packets, i);
		CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
		pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
		CU_ASSERT_TRUE(len >= pkt->payload.off + pkt->payload.len);

		fu_ind = ((const uint8_t *)cdata)[pkt->payload.off];
		fu_hdr = ((const uint8_t *)cdata)[pkt->payload.off + 1];
		start = (fu_hdr & 0x80) != 0;
		end = (fu_hdr & 0x40) != 0;

		CU_ASSERT_EQUAL(fu_ind & 0x1f, VSTRM_RTP_H264_NALU_TYPE_FU_A);
		CU_ASSERT_EQUAL(fu_hdr & 0x1f, H264_NALU_TYPE_SLICE);
		CU_ASSERT_EQUAL(start, i == 0);
		CU_ASSERT_EQUAL(end, i == 2);
		CU_ASSERT_EQUAL(
			RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, MARKER),
			i == 2 ? 1 : 0);

		memcpy(reassembled + 1 + rpos,
		       (const uint8_t *)cdata + pkt->payload.off + 2,
		       pkt->payload.len - 2);
		rpos += pkt->payload.len - 2;
	}
	reassembled[0] = data[0];
	CU_ASSERT_EQUAL(memcmp(reassembled, data, sizeof(data)), 0);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_stap_a_aggregation_and_lone_stap(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct vstrm_rtp_h264_tx_stats stats;
	struct list_node packets;
	struct vstrm_frame *frame;
	uint8_t sei[5], sps[6];
	struct vstrm_frame_nalu n_sei, n_sps;
	struct rtp_pkt *pkt;
	const void *cdata;
	size_t len;

	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Two aggregable NALUs -> one STAP-A packet with both members */
	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(sei, sizeof(sei), H264_NALU_TYPE_SEI);
	fill_nalu_bytes(sps, sizeof(sps), H264_NALU_TYPE_SPS);
	n_sei = make_nalu(sei, sizeof(sei), 0, 0);
	n_sps = make_nalu(sps, sizeof(sps), 0, 0);
	res = vstrm_frame_add_nalu(frame, &n_sei);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n_sps);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(list_length(&packets), 1);

	pkt = nth_pkt(&packets, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
	{
		const uint8_t *p = (const uint8_t *)cdata + pkt->payload.off;
		size_t expected_len = 1 + 2 + sizeof(sei) + 2 + sizeof(sps);
		CU_ASSERT_EQUAL(pkt->payload.len, expected_len);
		CU_ASSERT_EQUAL(p[0], VSTRM_RTP_H264_NALU_TYPE_STAP_A);
		CU_ASSERT_EQUAL((p[1] << 8) | p[2], (int)sizeof(sei));
		CU_ASSERT_EQUAL(memcmp(p + 3, sei, sizeof(sei)), 0);
		CU_ASSERT_EQUAL((p[3 + sizeof(sei)] << 8) |
					p[3 + sizeof(sei) + 1],
				(int)sizeof(sps));
		CU_ASSERT_EQUAL(
			memcmp(p + 3 + sizeof(sei) + 2, sps, sizeof(sps)), 0);
	}

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(stats.stap_packet_count, 1);
	CU_ASSERT_EQUAL(stats.single_nalu_packet_count, 0);

	free_packets(&packets);
	vstrm_frame_unref(frame);

	/* A lone aggregable NALU (last of the frame) is still sent as a
	 * genuine STAP-A packet containing a single NALU member -- an
	 * RFC 6184 Sec.5.7.1 deviation ("aggregation of at least two NAL
	 * units") that is easy to trigger and worth pinning down */
	list_init(&packets);
	frame = make_tagged_frame(2000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	n_sei = make_nalu(sei, sizeof(sei), 0, 0);
	res = vstrm_frame_add_nalu(frame, &n_sei);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(list_length(&packets), 1);

	pkt = nth_pkt(&packets, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
	CU_ASSERT_EQUAL(((const uint8_t *)cdata + pkt->payload.off)[0],
			VSTRM_RTP_H264_NALU_TYPE_STAP_A);
	CU_ASSERT_EQUAL(pkt->payload.len, 1 + 2 + sizeof(sei));

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(stats.stap_packet_count, 2);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_slice_closes_open_stap(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct vstrm_rtp_h264_tx_stats stats;
	struct list_node packets;
	struct vstrm_frame *frame;
	uint8_t sei[5], slice[20], sei2[5];
	struct vstrm_frame_nalu n_sei, n_slice, n_sei2;
	struct rtp_pkt *pkt;
	const void *cdata;
	size_t len;

	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(sei, sizeof(sei), H264_NALU_TYPE_SEI);
	fill_nalu_bytes(slice, sizeof(slice), H264_NALU_TYPE_SLICE);
	fill_nalu_bytes(sei2, sizeof(sei2), H264_NALU_TYPE_SEI);
	n_sei = make_nalu(sei, sizeof(sei), 0, 0);
	n_slice = make_nalu(slice, sizeof(slice), 0, 0);
	n_sei2 = make_nalu(sei2, sizeof(sei2), 0, 0);
	res = vstrm_frame_add_nalu(frame, &n_sei);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n_slice);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n_sei2);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	/* packet 1: STAP-A[sei, slice] (the slice both joins the open
	 * STAP-A *and* closes it); packet 2: STAP-A[sei2] (a fresh,
	 * independent lone STAP-A -- proof the first one was truly closed) */
	CU_ASSERT_EQUAL(list_length(&packets), 2);

	pkt = nth_pkt(&packets, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
	{
		const uint8_t *p = (const uint8_t *)cdata + pkt->payload.off;
		size_t expected_len = 1 + 2 + sizeof(sei) + 2 + sizeof(slice);
		CU_ASSERT_EQUAL(pkt->payload.len, expected_len);
		CU_ASSERT_EQUAL(p[0], VSTRM_RTP_H264_NALU_TYPE_STAP_A);
		CU_ASSERT_EQUAL(memcmp(p + 3, sei, sizeof(sei)), 0);
		CU_ASSERT_EQUAL(
			memcmp(p + 3 + sizeof(sei) + 2, slice, sizeof(slice)),
			0);
	}

	pkt = nth_pkt(&packets, 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
	CU_ASSERT_EQUAL(((const uint8_t *)cdata + pkt->payload.off)[0],
			VSTRM_RTP_H264_NALU_TYPE_STAP_A);
	CU_ASSERT_EQUAL(pkt->payload.len, 1 + 2 + sizeof(sei2));

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(stats.stap_packet_count, 2);
	CU_ASSERT_EQUAL(stats.single_nalu_packet_count, 0);
	CU_ASSERT_EQUAL(stats.fu_packet_count, 0);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_packet_size_align_padding(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct list_node packets;
	struct vstrm_frame *frame;
	uint8_t data[10];
	struct vstrm_frame_nalu nalu;
	struct rtp_pkt *pkt;
	const void *cdata;
	size_t len;

	cfg.dyn.target_packet_size = 1000;
	cfg.dyn.packet_size_align = 4;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(list_length(&packets), 1);

	pkt = nth_pkt(&packets, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	/* payload.len=10, 10%4=2 -> 2 padding bytes: [0x00, 0x02] */
	CU_ASSERT_EQUAL(pkt->payload.len, sizeof(data));
	CU_ASSERT_EQUAL(pkt->padding.len, 2);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, PADDING),
			1);
	pomp_buffer_get_cdata(pkt->raw.buf, &cdata, &len, NULL);
	CU_ASSERT_EQUAL(len, RTP_PKT_HEADER_SIZE + sizeof(data) + 2);
	CU_ASSERT_EQUAL(((const uint8_t *)cdata)[pkt->padding.off], 0x00);
	CU_ASSERT_EQUAL(((const uint8_t *)cdata)[pkt->padding.off + 1], 0x02);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_set_cfg_dyn_updates_live(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct vstrm_rtp_h264_tx_cfg_dyn cfg_dyn;
	struct vstrm_rtp_h264_tx_stats stats;
	struct list_node packets;
	struct vstrm_frame *frame;
	uint8_t small[50];
	uint8_t big[200];
	struct vstrm_frame_nalu nalu;

	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(small, sizeof(small), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(small, sizeof(small), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(list_length(&packets), 1);
	free_packets(&packets);
	vstrm_frame_unref(frame);

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(stats.single_nalu_packet_count, 1);
	CU_ASSERT_EQUAL(stats.fu_packet_count, 0);

	/* Shrink target_packet_size well below the next frame's NALU size */
	cfg_dyn.target_packet_size = 60;
	cfg_dyn.packet_size_align = 0;
	res = vstrm_rtp_h264_tx_set_cfg_dyn(tx, &cfg_dyn);
	CU_ASSERT_EQUAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(2000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	fill_nalu_bytes(big, sizeof(big), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(big, sizeof(big), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(list_length(&packets) > 1);
	free_packets(&packets);
	vstrm_frame_unref(frame);

	res = vstrm_rtp_h264_tx_get_stats(tx, &stats);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(stats.fu_packet_count > 0);

	res = vstrm_rtp_h264_tx_set_cfg_dyn(tx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vstrm_rtp_h264_tx_set_cfg_dyn(NULL, &cfg_dyn);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_proto_metadata_header_extension(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct list_node packets;
	struct vstrm_frame *frame;
	struct vmeta_frame *meta = NULL;
	Vmeta__TimedMetadata *proto_meta = NULL;
	Vmeta__CameraMetadata *camera;
	uint8_t data[20];
	struct vstrm_frame_nalu nalu;
	struct rtp_pkt *pkt;

	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT;
	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* A PROTO vmeta_frame with at least one non-default scalar field set
	 * packs to a non-empty buffer, which is what actually drives
	 * vstrm_rtp_h264_tx_add_proto_metadata() to write a real header
	 * extension (an all-default message packs to 0 bytes) */
	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = vmeta_frame_proto_get_unpacked_rw(meta, &proto_meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	camera = vmeta_frame_proto_get_camera(proto_meta);
	CU_ASSERT_PTR_NOT_NULL_FATAL(camera);
	camera->hfov = 1.0f;
	res = vmeta_frame_proto_release_unpacked_rw(meta, proto_meta);
	CU_ASSERT_EQUAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	/* Transfers ownership: vstrm_frame_unref() below will unref this
	 * metadata automatically once the frame's own refcount reaches 0,
	 * so it must NOT also be released separately here */
	frame->metadata = meta;

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(list_length(&packets), 1);

	pkt = list_entry(list_first(&packets), struct rtp_pkt, node);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION),
			1);
	CU_ASSERT_TRUE(pkt->extheader.len > 0);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_process_frame_metadata_error_cleans_up(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct list_node packets;
	struct vstrm_frame *frame;
	struct vmeta_frame *meta = NULL;
	Vmeta__TimedMetadata *proto_meta = NULL;
	Vmeta__CameraMetadata *camera;
	uint8_t data[20];
	struct vstrm_frame_nalu nalu;

	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT;
	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = vmeta_frame_proto_get_unpacked_rw(meta, &proto_meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	camera = vmeta_frame_proto_get_camera(proto_meta);
	CU_ASSERT_PTR_NOT_NULL_FATAL(camera);
	camera->hfov = 1.0f;
	/* Deliberately do NOT release the RW lock here (no
	 * vmeta_frame_proto_release_unpacked_rw()): this keeps
	 * meta->proto->w_lock set, which makes vmeta_frame_proto_get_buffer()
	 * return -EBUSY (libvideo-metadata/src/vmeta_frame_proto.c:426-429).
	 * vstrm_rtp_h264_tx_add_proto_metadata() propagates that error up
	 * through vstrm_rtp_h264_tx_begin_pkt() and
	 * vstrm_rtp_h264_tx_add_nalu(), reaching the "goto error" cleanup
	 * path in vstrm_rtp_h264_tx_process_frame()
	 * (src/vstrm_rtp_h264_tx.c:468-469, :490-503) -- this is the only
	 * deterministic (no OOM injection needed) way to reach that path */

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	/* Transfers ownership: vstrm_frame_unref() below will unref this
	 * metadata automatically once the frame's own refcount reaches 0 */
	frame->metadata = meta;

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, -EBUSY);
	/* The error-path cleanup must leave no partially-built packets
	 * behind, and self->pkt must already have been destroyed by
	 * vstrm_rtp_h264_tx_begin_pkt()'s own error handling */
	CU_ASSERT_TRUE(list_is_empty(&packets));

	res = vmeta_frame_proto_release_unpacked_rw(meta, proto_meta);
	CU_ASSERT_EQUAL(res, 0);

	/* self must be left in a clean, reusable state: a normal call right
	 * after the failed one must succeed */
	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_TRUE(list_length(&packets) >= 1);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_legacy_metadata_header_extension(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct list_node packets;
	struct vstrm_frame *frame;
	struct vmeta_frame *meta = NULL;
	uint8_t data[20];
	struct vstrm_frame_nalu nalu;
	struct rtp_pkt *pkt;
	struct vmeta_buffer rbuf;
	struct vmeta_frame_v1_streaming_basic out;

	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT;
	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Unlike PROTO, a V1_STREAMING_BASIC frame always writes a fixed,
	 * non-zero number of bytes (checked against
	 * VMETA_FRAME_V1_STREAMING_BASIC_SIZE in
	 * vmeta_frame_v1_streaming_basic_write()), so even all-default field
	 * values are enough to drive a real header extension */
	res = vmeta_frame_new(VMETA_FRAME_TYPE_V1_STREAMING_BASIC, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	meta->v1_strm_basic.camera_pan = 1.0f;
	meta->v1_strm_basic.battery_percentage = 42;

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	/* Transfers ownership: vstrm_frame_unref() below will unref this
	 * metadata automatically once the frame's own refcount reaches 0 */
	frame->metadata = meta;

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(list_length(&packets), 1);

	pkt = list_entry(list_first(&packets), struct rtp_pkt, node);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION),
			1);
	CU_ASSERT_TRUE(pkt->extheader.len > 0);

	/* Read the extension header back and check the content round-trips */
	vmeta_buffer_set_cdata(
		&rbuf, pkt->raw.cdata, pkt->raw.len, pkt->extheader.off);
	res = vmeta_frame_v1_streaming_basic_read(&rbuf, &out);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_DOUBLE_EQUAL(out.camera_pan, 1.0, 0.001);
	CU_ASSERT_EQUAL(out.battery_percentage, 42);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_legacy_metadata_only_added_to_first_fu_a_fragment(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct list_node packets;
	struct vstrm_frame *frame;
	struct vmeta_frame *meta = NULL;
	uint8_t data[250];
	struct vstrm_frame_nalu nalu;
	struct rtp_pkt *pkt;

	/* Same target_packet_size/nalu len as test_tx_fu_a_fragmentation(),
	 * but here the legacy metadata extension (28 bytes: 4-byte id/len +
	 * VMETA_FRAME_V1_STREAMING_BASIC_SIZE payload) eats into the FIRST
	 * packet's budget only, shrinking its first-fragment payload from
	 * (100 - 12 - 2) = 86 to (100 - 12 - 28 - 2) = 58 bytes -- so the
	 * 249 payload bytes (250-byte NALU minus its 1-byte header) now split
	 * as 58 + 86 + 86 + 19 = 4 fragments instead of 3 */
	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT;
	cfg.dyn.target_packet_size = 100;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vmeta_frame_new(VMETA_FRAME_TYPE_V1_STREAMING_BASIC, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	frame->metadata = meta;

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(list_length(&packets), 4);

	/* "Only add legacy metadata in first packet"
	 * (src/vstrm_rtp_h264_tx.c:67-69): the 2nd/3rd/4th FU-A fragments
	 * each call vstrm_rtp_h264_tx_begin_pkt() again, but self->packets is
	 * no longer empty by then, so add_legacy_metadata() must early-return
	 * 0 without touching the extension header */
	pkt = nth_pkt(&packets, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION),
			1);
	CU_ASSERT_TRUE(pkt->extheader.len > 0);

	pkt = nth_pkt(&packets, 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION),
			0);

	pkt = nth_pkt(&packets, 2);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION),
			0);

	pkt = nth_pkt(&packets, 3);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION),
			0);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_tx_process_frame_legacy_metadata_error_cleans_up(void)
{
	int res;
	struct vstrm_rtp_h264_tx *tx = NULL;
	struct vstrm_rtp_h264_tx_cfg cfg = {0};
	struct list_node packets;
	struct vstrm_frame *frame;
	struct vmeta_frame *meta = NULL;
	uint8_t data[20];
	struct vstrm_frame_nalu nalu;

	cfg.flags = VSTRM_SENDER_FLAGS_ENABLE_RTP_HEADER_EXT;
	cfg.dyn.target_packet_size = 1000;
	res = vstrm_rtp_h264_tx_new(&cfg, &tx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vmeta_frame_new(VMETA_FRAME_TYPE_V1_STREAMING_BASIC, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* struct vmeta_frame is a public, non-opaque struct: unlike the
	 * PROTO path (which has a real w_lock/-EBUSY mechanism to force a
	 * deterministic failure), legacy metadata has no such lock, so the
	 * equivalent deterministic (no OOM injection) trigger is corrupting
	 * ->type to a value outside the enum after a valid vmeta_frame_new()
	 * -- vmeta_frame_write()'s dispatch switch (src/vmeta_frame.c) has no
	 * case for it and falls to "default: res = -ENOSYS", which
	 * vstrm_rtp_h264_tx_add_legacy_metadata() propagates up through
	 * vstrm_rtp_h264_tx_begin_pkt() and vstrm_rtp_h264_tx_add_nalu(),
	 * reaching the same "goto error" cleanup path in
	 * vstrm_rtp_h264_tx_process_frame() as the PROTO case */
	meta->type = (enum vmeta_frame_type)9999;

	list_init(&packets);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	frame->metadata = meta;

	fill_nalu_bytes(data, sizeof(data), H264_NALU_TYPE_SLICE);
	nalu = make_nalu(data, sizeof(data), 0, 0);
	res = vstrm_frame_add_nalu(frame, &nalu);
	CU_ASSERT_EQUAL(res, 0);

	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL(res, -ENOSYS);
	CU_ASSERT_TRUE(list_is_empty(&packets));

	/* self must be left in a clean, reusable state: fix the type and
	 * confirm a subsequent call succeeds */
	meta->type = VMETA_FRAME_TYPE_V1_STREAMING_BASIC;
	res = vstrm_rtp_h264_tx_process_frame(tx, frame, &packets);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_TRUE(list_length(&packets) >= 1);

	free_packets(&packets);
	vstrm_frame_unref(frame);
	res = vstrm_rtp_h264_tx_destroy(tx);
	CU_ASSERT_EQUAL(res, 0);
}


CU_TestInfo g_vstrm_test_tx[] = {
	{FN("tx-single-nalu-packetization"),
	 &test_tx_single_nalu_packetization},
	{FN("tx-fu-a-fragmentation"), &test_tx_fu_a_fragmentation},
	{FN("tx-stap-a-aggregation-and-lone-stap"),
	 &test_tx_stap_a_aggregation_and_lone_stap},
	{FN("tx-slice-closes-open-stap"), &test_tx_slice_closes_open_stap},
	{FN("tx-packet-size-align-padding"),
	 &test_tx_packet_size_align_padding},
	{FN("tx-set-cfg-dyn-updates-live"), &test_tx_set_cfg_dyn_updates_live},
	{FN("tx-proto-metadata-header-extension"),
	 &test_tx_proto_metadata_header_extension},
	{FN("tx-process-frame-metadata-error-cleans-up"),
	 &test_tx_process_frame_metadata_error_cleans_up},
	{FN("tx-legacy-metadata-header-extension"),
	 &test_tx_legacy_metadata_header_extension},
	{FN("tx-legacy-metadata-only-added-to-first-fu-a-fragment"),
	 &test_tx_legacy_metadata_only_added_to_first_fu_a_fragment},
	{FN("tx-process-frame-legacy-metadata-error-cleans-up"),
	 &test_tx_process_frame_legacy_metadata_error_cleans_up},

	CU_TEST_INFO_NULL,
};
