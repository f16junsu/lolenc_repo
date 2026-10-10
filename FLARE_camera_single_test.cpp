/* One jb_0 HIGH pulse (1 us), one RAW_REQUIRED shot, one detection result.
 * Start the FLARE viewer with --shot-metadata and the PC verification tool
 * before running. See FLARE_camera_tests_README.txt for the full procedure.
 */
#define FLARE_CAMERA_SHOTS 1
#define FLARE_CAMERA_TEST_NAME "FLARE_camera_single_test"
#include "FLARE_camera_test_common.h"
int main() { return run_camera_test(); }
