/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"

#include <stdio.h>


static void fill_nalu_bytes(uint8_t *buf, size_t len, uint8_t nal_type)
{
	buf[0] = (uint8_t)((3 << 5) | (nal_type & 0x1f));
	for (size_t i = 1; i < len; i++)
		buf[i] = (uint8_t)(i & 0xff);
}


static void test_dbg_create_file(void)
{
	FILE *f;

	f = vstrm_dbg_create_file("/tmp", NULL, "vstrm_test_dbg", "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	fclose(f);

	/* A nonexistent directory makes the underlying fopen() fail */
	f = vstrm_dbg_create_file(
		"/nonexistent_dir_vstrm_test_dbg_xyz", NULL, "x", "wb");
	CU_ASSERT_PTR_NULL(f);
}


static void test_dbg_write_raw_and_pomp_buf(void)
{
	FILE *f;
	uint8_t data[5] = {1, 2, 3, 4, 5};
	uint8_t expected[9] = {0, 0, 0, 5, 1, 2, 3, 4, 5};
	uint8_t actual[9];
	size_t n;
	struct pomp_buffer *buf;

	/* vstrm_dbg_write_raw(): 4-byte big-endian length prefix + data */
	f = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	vstrm_dbg_write_raw(f, data, sizeof(data));
	fflush(f);
	fseek(f, 0, SEEK_SET);
	n = fread(actual, 1, sizeof(actual), f);
	CU_ASSERT_EQUAL(n, sizeof(expected));
	CU_ASSERT_EQUAL(memcmp(actual, expected, sizeof(expected)), 0);
	fclose(f);

	/* vstrm_dbg_write_pomp_buf(): same layout, sourced from a pomp_buffer
	 */
	buf = pomp_buffer_new_with_data(data, sizeof(data));
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	f = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	vstrm_dbg_write_pomp_buf(f, buf);
	fflush(f);
	fseek(f, 0, SEEK_SET);
	memset(actual, 0, sizeof(actual));
	n = fread(actual, 1, sizeof(actual), f);
	CU_ASSERT_EQUAL(n, sizeof(expected));
	CU_ASSERT_EQUAL(memcmp(actual, expected, sizeof(expected)), 0);
	fclose(f);
	pomp_buffer_unref(buf);
}


static void test_dbg_write_codec_info(void)
{
	FILE *f;
	struct vstrm_codec_info info;
	uint8_t expected[64];
	size_t epos = 0;
	uint8_t actual[64];
	size_t n;

	memset(&info, 0, sizeof(info));
	info.codec = VSTRM_CODEC_VIDEO_H264;
	fill_nalu_bytes(info.h264.sps, 6, H264_NALU_TYPE_SPS);
	info.h264.spslen = 6;
	fill_nalu_bytes(info.h264.pps, 4, H264_NALU_TYPE_PPS);
	info.h264.ppslen = 4;

	/* Two NALUs, each start-code (00 00 00 01) prefixed */
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x01;
	memcpy(expected + epos, info.h264.sps, info.h264.spslen);
	epos += info.h264.spslen;
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x01;
	memcpy(expected + epos, info.h264.pps, info.h264.ppslen);
	epos += info.h264.ppslen;

	f = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	vstrm_dbg_write_codec_info(f, &info);
	fflush(f);
	fseek(f, 0, SEEK_SET);
	n = fread(actual, 1, sizeof(actual), f);
	CU_ASSERT_EQUAL(n, epos);
	CU_ASSERT_EQUAL(memcmp(actual, expected, epos), 0);
	fclose(f);

	/* Non-H264 codec: nothing is written */
	info.codec = VSTRM_CODEC_UNKNOWN;
	f = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	vstrm_dbg_write_codec_info(f, &info);
	fflush(f);
	fseek(f, 0, SEEK_END);
	CU_ASSERT_EQUAL(ftell(f), 0);
	fclose(f);
}


static void test_dbg_write_frame(void)
{
	FILE *f;
	struct vstrm_frame *frame;
	uint8_t d1[4], d2[3];
	struct vstrm_frame_nalu n1, n2;
	int res;
	uint8_t expected[32];
	size_t epos = 0;
	uint8_t actual[32];
	size_t n;
	struct vstrm_codec_info info;

	fill_nalu_bytes(d1, sizeof(d1), H264_NALU_TYPE_SLICE);
	fill_nalu_bytes(d2, sizeof(d2), H264_NALU_TYPE_SEI);
	frame = make_tagged_frame(1000);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
	n1 = make_nalu(d1, sizeof(d1), 0, 0);
	n2 = make_nalu(d2, sizeof(d2), 0, 0);
	res = vstrm_frame_add_nalu(frame, &n1);
	CU_ASSERT_EQUAL(res, 0);
	res = vstrm_frame_add_nalu(frame, &n2);
	CU_ASSERT_EQUAL(res, 0);

	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x01;
	memcpy(expected + epos, d1, sizeof(d1));
	epos += sizeof(d1);
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x00;
	expected[epos++] = 0x01;
	memcpy(expected + epos, d2, sizeof(d2));
	epos += sizeof(d2);

	/* info == NULL is treated the same as an H264 codec_info */
	f = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	vstrm_dbg_write_frame(f, NULL, frame);
	fflush(f);
	fseek(f, 0, SEEK_SET);
	n = fread(actual, 1, sizeof(actual), f);
	CU_ASSERT_EQUAL(n, epos);
	CU_ASSERT_EQUAL(memcmp(actual, expected, epos), 0);
	fclose(f);

	/* A non-H264 codec_info suppresses all output */
	memset(&info, 0, sizeof(info));
	info.codec = VSTRM_CODEC_UNKNOWN;
	f = tmpfile();
	CU_ASSERT_PTR_NOT_NULL_FATAL(f);
	vstrm_dbg_write_frame(f, &info, frame);
	fflush(f);
	fseek(f, 0, SEEK_END);
	CU_ASSERT_EQUAL(ftell(f), 0);
	fclose(f);

	vstrm_frame_unref(frame);
}


CU_TestInfo g_vstrm_test_dbg[] = {
	{FN("dbg-create-file"), &test_dbg_create_file},
	{FN("dbg-write-raw-and-pomp-buf"), &test_dbg_write_raw_and_pomp_buf},
	{FN("dbg-write-codec-info"), &test_dbg_write_codec_info},
	{FN("dbg-write-frame"), &test_dbg_write_frame},

	CU_TEST_INFO_NULL,
};
