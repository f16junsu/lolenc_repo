#ifndef FLARE_CAMERA_SETTINGS_H
#define FLARE_CAMERA_SETTINGS_H
#include <stdint.h>

// User-selected output: jb_0, HIGH for 1 us. jb_0 aliases AOM_MAIN.
// Connect this output to the camera trigger for this experiment.
#define FLARE_CAMERA_TTL jb_0
static const uint32_t CAMERA_PULSE_NS = 1000;
static const uint32_t CAMERA_SHOT_GAP_US = 1000000;
static const uint32_t CAMERA_RESULT_TIMEOUT_US = 3000000;

// A simple one-window bring-up configuration, independent of camera settings.
// Change these to the actual ion's top-left pixel coordinates and threshold.
// The complete camera frame must still be 512 x 512, binning 1 x 1.
static const uint16_t CAMERA_ROI_X = 100;
static const uint16_t CAMERA_ROI_Y = 200;
static const uint16_t CAMERA_ROI_WIDTH = 8;
static const uint16_t CAMERA_ROI_HEIGHT = 8;
static const int32_t CAMERA_THRESHOLD = 5000;
static const uint32_t CAMERA_CONFIDENCE_MARGIN = 1000;

#endif
