/**
 * Copyright (c) 2016 Parrot Drones SAS
 */

#include "vstrm_test.h"


static CU_SuiteInfo s_suites[] = {
	{FN("frame"), NULL, NULL, g_vstrm_test_frame},
	{FN("dbg"), NULL, NULL, g_vstrm_test_dbg},
	{FN("internal"), NULL, NULL, g_vstrm_test_internal},
	{FN("tx"), NULL, NULL, g_vstrm_test_tx},
	{FN("rx"), NULL, NULL, g_vstrm_test_rx},
	{FN("concealment"), NULL, NULL, g_vstrm_test_concealment},
	{FN("sender"), NULL, NULL, g_vstrm_test_sender},
	{FN("receiver"), NULL, NULL, g_vstrm_test_receiver},

	CU_SUITE_INFO_NULL,
};


static void run_automated()
{
	CU_automated_run_tests();
	CU_list_tests_to_file();
}


static void run_basic()
{
	CU_basic_set_mode(CU_BRM_VERBOSE);
	CU_basic_run_tests();
}


int main()
{
	const char *filename;

	/* Debug-dump env vars are read unconditionally by vstrm_sender_new()/
	 * vstrm_receiver_new() and would make a CI/dev environment with them
	 * set start writing files to disk; force them off for hermetic runs */
	unsetenv("VSTRM_DBG_DIR");
	unsetenv("VSTRM_DBG_FLAGS");
	unsetenv("VSTRM_RECEIVER_FLAGS_H264_FULL_MB_STATUS");

	CU_initialize_registry();
	CU_register_suites(s_suites);

	/* Set filename */
	filename = getenv("CUNIT_OUT_NAME");
	CU_set_output_filename(filename);

	/* Run tests */
	if (getenv("CUNIT_AUTOMATED") != NULL)
		run_automated();
	else
		run_basic();

	CU_cleanup_registry();
}
