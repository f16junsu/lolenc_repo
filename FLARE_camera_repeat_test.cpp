/* Ten shots, each with one jb_0 HIGH pulse (1 us), RAW_REQUIRED and detection.
 * Wait for a result/auto-ACK/peer-ready before each next shot; 1 s extra gap.
 * Start the FLARE viewer and PC verification tool before running.
 */
#define FLARE_CAMERA_SHOTS 10
#define FLARE_CAMERA_TEST_NAME "FLARE_camera_repeat_test"
#include "FLARE_camera_test_common.h"
int main() { return run_camera_test(); }
