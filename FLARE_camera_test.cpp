/*
 * One camera shot: configure one 8x8 ROI, pulse TTL, read STATE_RESULT.
 * Camera: external rising-edge trigger, Camera Link enabled, 512x512/16-bit.
 * Run alone with the camera armed but not sending free-running frames.
 * This replaces the FLARE measurement config and leaves streaming stopped.
 * The selected TTL bank is initialized LOW; its other outputs stay LOW.
 * RAW is disabled: this reads classification results, not an image.
 *
 * Edit the TTL alias below to match the cable. Pulse width and ROI/threshold
 * are ArgInt parameters in the experiment GUI. Threshold is an example only.
 * Results are saved under datasets/rid<RID>_* in the usual HDF5 result file.
 * camera_test_summary: [pass, error_stage]
 *   stages: 1 local setup, 2 peer idle, 3 config, 4 peer ready,
 *           5 ACK/result timeout or bad reply, 6 invalid measurement, 7 dataset.
 * camera_test_result:
 *   [shot_id, token, result_code, error_flags, frame_valid, classifier_complete,
 *    config_current, ion_count, state_mask, valid_mask, low_confidence, saturation]
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xtime_l.h"

extern FLAREController flare_controller;
extern TTL jd_1;
static TTL& camera_trigger = jd_1;  // Change both occurrences if using another TTL.

ArgInt camera_pulse_us(10, 1, 10000, 1, "us");
ArgInt roi_x(252, 0, 504, 1, "pixel");
ArgInt roi_y(252, 0, 504, 1, "pixel");
ArgInt threshold(5000, 0, 10000000, 1, "count");

static DatasetFloatList summary("camera_test_summary", true);
static DatasetFloatList result_data("camera_test_result", true);

static uint32_t elapsed_us(XTime start) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(((now - start) * 1000000ULL) / COUNTS_PER_SECOND);
}

// delay() only moves the software cursor. auto_start() executes the queue.
static void begin_timeline() {
    init();
    flare_controller.reset_slot_tracking();
    delay(100 * us);
}

static bool direct_module(const LolencModule& module) {
    const uint64_t channel = module.virtual_table_channel;
    return channel < 64 && virtual_table->module_addr[channel] == module.addr &&
           !virtual_table->virtual_write[channel] && !virtual_table->table_size[channel];
}

static bool read_reply(FlareRecord& record, uint32_t timeout_us = 500000) {
    XTime start;
    XTime_GetTime(&start);
    while (elapsed_us(start) < timeout_us) {
        if (flare_controller.records_available()) {
            return flare_controller.read_record(&record) == FLARE_OK &&
                   !record.record_flags &&
                   record.entry_count == 1 + (record.payload_words + 1) / 2;
        }
        usleep(100);  // Wall-clock wait; delay() would not wait for a reply.
    }
    return false;
}

static bool peer_status(FlareRecord& record) {
    begin_timeline();
    const uint32_t sequence = flare_controller.read_status().last_tx_sequence;
    flare_controller.request_status();
    auto_start();
    const bool received = read_reply(record);
    auto_stop();
    return received && record.record_type == FLARE_MSG_STATUS_RESPONSE &&
           record.payload_words == 6 && record.sequence == sequence &&
           ((record.payload[0] >> 32) & 0xff) == 2;  // STATUS layout v2.
}

static int fail(unsigned stage) {
    auto_stop();
    summary.append_list(2, 0.0, (double)stage);
    xil_printf("FLARE_camera_test FAIL: stage %u\r\n", stage);
    return 1;
}

int main() {
    init();
    const int64_t pulse_us = camera_pulse_us.get_value();
    if (pulse_us < 1 || pulse_us > 10000 ||
        !direct_module(flare_controller) || !direct_module(camera_trigger)) return fail(1);
    const FlareStatus local = flare_controller.read_status();
    if (!local.channel_up || !local.lane_up || local.ack_pending || local.result_pending)
        return fail(1);
    flare_controller.flush_commands();
    flare_controller.clear_records();
    flare_controller.clear_sticky();
    camera_trigger.reset();
    camera_trigger.set_last_pulse(0);  // Initialize this bank's shared software mask.
    usleep(1000);

    // 1. Stop storage streaming, establish TTL LOW, then configure one ion.
    begin_timeline();
    camera_trigger.off();
    flare_controller.stream_stop();
    auto_start();
    usleep(10000);
    auto_stop();

    FlareRecord record = {};
    if (!peer_status(record)) return fail(2);
    const unsigned health = (unsigned)(record.payload[0] >> 56);
    if (!(health & 0x40) || (health & 0xbc) ||
        ((record.payload[0] >> 40) & 0xff) != 0) return fail(2);  // Linked and idle.
    uint32_t token = (uint32_t)record.payload[0] + 1U;
    uint32_t shot_id = (uint32_t)record.payload[1] + 1U;
    if (!token) ++token;
    if (!shot_id) ++shot_id;

    FlareConfig cfg = {};
    cfg.ion_count = 1;
    cfg.signal_width = cfg.signal_height = 8;
    cfg.background_width = cfg.background_height = 8;
    cfg.background_enable = 0;
    cfg.background_scale_q32 = 1U << 20;
    cfg.ion_x[0] = (uint16_t)roi_x.get_value();
    cfg.ion_y[0] = (uint16_t)roi_y.get_value();
    cfg.ion_threshold[0] = (int32_t)threshold.get_value();
    cfg.config_token32 = token;

    begin_timeline();
    const uint32_t commit_sequence = flare_controller.read_status().last_tx_sequence + 6U;
    if (flare_controller.configure(cfg) != FLARE_OK) return fail(3);
    auto_start();
    const bool configured = read_reply(record);
    auto_stop();
    FlareConfigStatus config_status = {};
    if (!configured || record.payload_words != 2 || record.sequence != commit_sequence ||
        FLAREController::decode_config_status(record, &config_status) != FLARE_OK ||
        config_status.request_sequence != commit_sequence || config_status.config_id ||
        config_status.status_code || config_status.error_flags ||
        config_status.config_version != (uint16_t)token) return fail(3);
    usleep(1000);  // Allow the classifier's configuration table to load.
    if (!peer_status(record) || (uint32_t)record.payload[0] != token ||
        (record.payload[0] >> 56) != 0x43 ||
        ((record.payload[0] >> 40) & 0xff) != 0) return fail(4);  // READY + CONFIG_VALID + LINK.

    // 2. Reserve one physical pulse and its ARM_TRIGGER at the same coarse tick.
    begin_timeline();
    const uint32_t arm_sequence = flare_controller.read_status().last_tx_sequence;
    flare_controller.arm_trigger(shot_id, FLARE_RAW_DISABLED);
    camera_trigger.on();  // No delay between ARM_TRIGGER and this edge.
    delay(pulse_us * us);
    camera_trigger.off();
    XTime start;
    XTime_GetTime(&start);
    auto_start();
    // Let the queued falling edge execute even if an early NACK arrives.
    usleep((unsigned)pulse_us + 200);

    // 3. Read both replies (either arrival order); the RTL sends RESULT_ACK.
    bool armed = false, received = false, acknowledged = false;
    FlareResult result = {};
    while (elapsed_us(start) < 3000000) {
        if (flare_controller.records_available()) {
            if (!read_reply(record) || record.payload_words != 3) return fail(5);
            if (record.record_type == FLARE_MSG_ARMED_ACK) {
                if (armed || record.sequence != arm_sequence || record.payload[0] != shot_id ||
                    (record.payload[1] & 0xffff) != 0 ||
                    ((record.payload[1] >> 32) & 0xffff) != (uint16_t)token) return fail(5);
                armed = true;
            } else if (record.record_type == FLARE_MSG_STATE_RESULT) {
                if (received || FLAREController::decode_result(record, &result) != FLARE_OK ||
                    result.shot_id != shot_id || result.config_token32 != token) return fail(5);
                received = true;
            } else {
                return fail(5);  // Includes NACK; do not silently discard it.
            }
        }
        const FlareStatus current = flare_controller.read_status();
        if (armed && received && !current.ack_pending && !current.result_pending &&
            current.last_acked_sequence == result.sequence) {
            acknowledged = true;
            break;
        }
        usleep(100);
    }
    auto_stop();

    // Save only after the time-sensitive exchange. append_list requires doubles.
    if (received) {
        if (result_data.append_list(12, (double)result.shot_id, (double)result.config_token32,
            (double)result.result_code, (double)result.measurement_error_flags,
            (double)result.frame_valid, (double)result.classifier_complete,
            (double)result.config_current, (double)result.ion_count,
            (double)result.state_mask, (double)result.valid_mask,
            (double)result.low_confidence_mask, (double)result.saturation_mask) != ERR_OKAY)
            return fail(7);
        xil_printf("shot %u: code %u errors 0x%x state 0x%x valid 0x%x\r\n",
            (unsigned)result.shot_id, (unsigned)result.result_code,
            (unsigned)result.measurement_error_flags, (unsigned)result.state_mask,
            (unsigned)result.valid_mask);
    }
    if (!armed || !received || !acknowledged) return fail(5);
    if (result.result_code || result.measurement_error_flags || !result.frame_valid ||
        !result.classifier_complete || !result.config_current || result.ion_count != 1 ||
        result.valid_mask != 1 || flare_controller.read_status().sticky_timestamp_error)
        return fail(6);
    if (summary.append_list(2, 1.0, 0.0) != ERR_OKAY) return fail(7);
    xil_printf("FLARE_camera_test PASS\r\n");
    return 0;  // The ELF runner handles the end-of-experiment handshake.
}
