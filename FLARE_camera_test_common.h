#ifndef FLARE_CAMERA_TEST_COMMON_H
#define FLARE_CAMERA_TEST_COMMON_H
/* LOLENC: delay() advances a software cursor, not wall time. Each exchange
 * stops/resets RTIO and queues events before auto_start(). No dataset IPC
 * runs while a shot is active. Return from main for ELF_Runner's handshake.
 * AUTO-ACK of Aurora RESULT is independent of the PC's Ethernet RAW ACK.
 * The board can prove detection reception, not that a PC saved the RAW.
 * Only FLARE_camera_verify.py may declare the combined experiment PASS.
 * Camera SDK exposure/gain/external-trigger/Camera Link are configured by
 * the operator. No free-running frames may arrive during this experiment.
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xtime_l.h"
#include "FLARE_camera_settings.h"

extern FLAREController flare_controller;
extern TTL FLARE_CAMERA_TTL;
static_assert(FLARE_CAMERA_SHOTS >= 1 && FLARE_CAMERA_SHOTS <= 10, "1..10 shots");
static_assert(CAMERA_PULSE_NS >= 8 && CAMERA_PULSE_NS % 8 == 0, "whole RTIO ticks");

enum CameraFailure {
    CAMERA_MAP=1, CAMERA_LINK=2, CAMERA_TIMEOUT=4, CAMERA_RECORD=8,
    CAMERA_PROTOCOL=16, CAMERA_LOCAL=32, CAMERA_CONFIG=64, CAMERA_PEER=128,
    CAMERA_ARM=256, CAMERA_RESULT=512, CAMERA_ACK=1024,
    CAMERA_TTL_ERROR=2048, CAMERA_HOST=4096
};
enum CameraCommand { CAMERA_STATUS, CAMERA_STOP_STREAM, CAMERA_CONFIGURE, CAMERA_SHOT };
struct CameraExchange {
    unsigned phase, kind, record_count;
    uint32_t failures, armed_us, result_us;
    bool armed, result_received;
    uint8_t raw_enabled;
    FlareRecord records[8];
    FlareStatus before, after;
    FlareResult detection;
};
static CameraExchange camera_exchanges[128];
static unsigned camera_exchange_count;
static uint32_t camera_failures;
static bool camera_publication_ok = true;
static DatasetFloatList camera_ds_summary("flare_camera_summary", true);
static DatasetFloatList camera_ds_settings("flare_camera_settings", true);
static DatasetFloatList camera_ds_results("flare_camera_results", true);
static DatasetFloatList camera_ds_records("flare_camera_records", true);
static DatasetFloatList camera_ds_exchanges("flare_camera_exchanges", true);
static DatasetFloatList camera_ds_ttl("flare_camera_ttl", true);

static uint32_t camera_elapsed_us(XTime start) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(((now - start) * 1000000ULL) / COUNTS_PER_SECOND);
}
static unsigned camera_delta(uint16_t after, uint16_t before) {
    return (uint16_t)(after - before);
}
static bool camera_local_bad(const FlareStatus& st) {
    return st.sticky_rx_overflow || st.sticky_tx_overflow || st.sticky_crc_error ||
        st.sticky_hard_error || st.sticky_soft_error || st.sticky_timestamp_error ||
        st.sticky_truncated || st.sticky_record_drop || st.sticky_tx_abort ||
        st.hard_error_count || st.soft_error_count || st.crc_error_count ||
        st.header_error_count || st.phy_overflow_count || st.tx_drop_count ||
        st.record_drop_count || st.tx_abort_count;
}
static bool camera_mapped(const LolencModule& mod) {
    const uint64_t ch = mod.virtual_table_channel;
    return ch < 64 && virtual_table->module_addr[ch] == mod.addr &&
        !virtual_table->virtual_write[ch] && !virtual_table->table_size[ch];
}
static void camera_publish(err_t err) {
    if (err != ERR_OKAY) camera_publication_ok = false;
}
static bool camera_status_shape(const FlareRecord& rec) {
    return rec.record_type == FLARE_MSG_STATUS_RESPONSE && rec.payload_words == 6 &&
        rec.entry_count == 4 && !rec.record_flags && ((rec.payload[0] >> 32) & 255) == 2;
}
static bool camera_idle(const FlareRecord& rec, bool require_ready, uint32_t token) {
    if (!camera_status_shape(rec)) return false;
    const unsigned health = (unsigned)(rec.payload[0] >> 56);
    // ready, configuration valid, Aurora up; no camera/shot/writer/result/fatal.
    if (!(health & 0x40) || (health & 0xbc) || ((rec.payload[0] >> 40) & 255)) return false;
    if (require_ready && ((health & 3) != 3 || (uint32_t)rec.payload[0] != token)) return false;
    return true;
}

static CameraExchange& camera_exchange(CameraCommand kind, const FlareConfig& cfg,
                                       uint32_t shot_id=0) {
    CameraExchange& ex = camera_exchanges[camera_exchange_count++];
    ex.phase = camera_exchange_count;
    ex.kind = kind;
    init();
    flare_controller.reset_slot_tracking();
    ex.before = ex.after = flare_controller.read_status();
    if (!ex.before.channel_up || !ex.before.lane_up) ex.failures |= CAMERA_LINK;
    if (camera_local_bad(ex.before)) ex.failures |= CAMERA_LOCAL;
    if (ex.before.record_level || ex.before.result_pending || ex.before.ack_pending)
        ex.failures |= CAMERA_PROTOCOL;
    if (ex.failures) { camera_failures |= ex.failures; return ex; }
    const unsigned wanted_tx = kind == CAMERA_CONFIGURE ? 7 : 1;
    const unsigned wanted_rx = kind == CAMERA_SHOT ? 2 : kind == CAMERA_STOP_STREAM ? 0 : 1;
    const uint32_t slots_before = flare_controller.slot_collision_count;
    if (kind == CAMERA_SHOT) {
        // TTL is an eight-bit group. Read actual output state, not the device
        // config's malloc'ed, uninitialized software shadow. Preserve bits 1..7.
        const uint64_t current = Xil_In64(FLARE_CAMERA_TTL.addr + 0x90) & 255;
        if (Xil_In64(FLARE_CAMERA_TTL.addr + 0x40) & 1) ex.failures |= CAMERA_TTL_ERROR;
        if (FLARE_CAMERA_TTL.get_channel() != 0) ex.failures |= CAMERA_TTL_ERROR;
        if (ex.failures) { camera_failures |= ex.failures; return ex; }
        FLARE_CAMERA_TTL.set_last_pulse(current);
        delay(100 * us);
        FLARE_CAMERA_TTL.off(); // Set a known LOW baseline before the rising edge.
        delay(100 * us);
        FLARE_CAMERA_TTL.on();
        flare_controller.arm_trigger(shot_id, FLARE_RAW_REQUIRED); // SAME tick as HIGH.
        delay(CAMERA_PULSE_NS);
        FLARE_CAMERA_TTL.off(); // Always queued before releasing the timeline.
    } else {
        delay(100 * us);
        if (kind == CAMERA_STATUS) flare_controller.request_status();
        else if (kind == CAMERA_STOP_STREAM) flare_controller.stream_stop();
        else if (flare_controller.configure(cfg) != FLARE_OK) ex.failures |= CAMERA_CONFIG;
    }
    if (ex.failures) { camera_failures |= ex.failures; return ex; }
    XTime start;
    XTime_GetTime(&start);
    auto_start();
    uint32_t last_record_us = 0;
    const uint32_t timeout_us = kind == CAMERA_SHOT ? CAMERA_RESULT_TIMEOUT_US : 500000;
    for (;;) {
        const uint32_t waited = camera_elapsed_us(start);
        const FlareStatus live = flare_controller.read_status();
        if (!live.channel_up || !live.lane_up) { ex.failures |= CAMERA_LINK; break; }
        if (camera_local_bad(live)) { ex.failures |= CAMERA_LOCAL; break; }
        if (flare_controller.records_available()) {
            if (ex.record_count == 8) { ex.failures |= CAMERA_PROTOCOL; break; }
            FlareRecord& rec = ex.records[ex.record_count++];
            if (flare_controller.read_record(&rec) != FLARE_OK) { ex.failures |= CAMERA_RECORD; break; }
            last_record_us = camera_elapsed_us(start);
            if (rec.record_flags) ex.failures |= CAMERA_RECORD;
            if (kind == CAMERA_STATUS) {
                if (!camera_status_shape(rec) || rec.sequence != ex.before.last_tx_sequence)
                    ex.failures |= CAMERA_PROTOCOL;
            } else if (kind == CAMERA_CONFIGURE) {
                FlareConfigStatus reply = {};
                const uint32_t commit_seq = ex.before.last_tx_sequence + 6U;
                if (rec.payload_words != 2 || rec.entry_count != 2 ||
                    FLAREController::decode_config_status(rec, &reply) != FLARE_OK ||
                    rec.sequence != commit_seq || reply.request_sequence != commit_seq ||
                    reply.config_id || reply.status_code || reply.error_flags ||
                    reply.config_version != (uint16_t)cfg.config_token32 || (rec.payload[1] >> 32))
                    ex.failures |= CAMERA_CONFIG;
            } else if (kind == CAMERA_SHOT) {
                if (rec.payload_words != 3 || rec.entry_count != 3) ex.failures |= CAMERA_RECORD;
                else if (rec.record_type == FLARE_MSG_ARMED_ACK) {
                    ex.armed_us = last_record_us;
                    ex.raw_enabled = (uint8_t)(rec.payload[1] >> 8);
                    if (ex.armed || rec.sequence != ex.before.last_tx_sequence ||
                        rec.payload[0] != shot_id || (rec.payload[1] & 0xffffffffULL) != 0x100 ||
                        ((rec.payload[1] >> 32) & 0xffff) != (uint16_t)cfg.config_token32 ||
                        (rec.payload[1] >> 48) || rec.payload[2]) ex.failures |= CAMERA_ARM;
                    else ex.armed = true;
                } else if (rec.record_type == FLARE_MSG_STATE_RESULT) {
                    ex.result_us = last_record_us;
                    if (ex.result_received) ex.failures |= CAMERA_PROTOCOL;
                    else {
                        ex.result_received = true; // Retain invalid results for diagnosis.
                        if (FLAREController::decode_result(rec, &ex.detection) != FLARE_OK ||
                            ex.detection.shot_id != shot_id || ex.detection.config_token32 != cfg.config_token32 ||
                            ex.detection.result_code || ex.detection.measurement_error_flags ||
                            !ex.detection.frame_valid || !ex.detection.classifier_complete ||
                            !ex.detection.config_current || ex.detection.ion_count != 1 ||
                            ex.detection.valid_mask != 1 || (ex.detection.state_mask & ~1U) ||
                            (ex.detection.low_confidence_mask & ~1U) || (ex.detection.saturation_mask & ~1U) ||
                            (rec.payload[2] >> 32)) ex.failures |= CAMERA_RESULT;
                    }
                } else ex.failures |= CAMERA_PROTOCOL;
            } else ex.failures |= CAMERA_PROTOCOL;
            if (ex.failures) break;
        } else if (kind == CAMERA_STOP_STREAM && waited >= 10000 &&
                   camera_delta(live.command_tx_count, ex.before.command_tx_count) == 1) break;
        else if (kind != CAMERA_SHOT && ex.record_count == wanted_rx && wanted_rx &&
                 waited - last_record_us >= 2000) break;
        else if (kind == CAMERA_SHOT && ex.armed && ex.result_received &&
                 waited - last_record_us >= 2000 && !live.ack_pending && !live.result_pending &&
                 camera_delta(live.ack_tx_count, ex.before.ack_tx_count) == 1 &&
                 live.last_acked_sequence == ex.detection.sequence) break;
        if (waited >= timeout_us || (kind == CAMERA_SHOT && !ex.armed && waited >= 500000)) {
            ex.failures |= CAMERA_TIMEOUT;
            break;
        }
        usleep(50);
    }
    // An early ARM rejection can precede the queued falling edge. Let that
    // edge execute before stopping RTIO; never leave the trigger HIGH on FAIL.
    while (kind == CAMERA_SHOT && camera_elapsed_us(start) < 1200) usleep(50);
    auto_stop();
    ex.after = flare_controller.read_status();
    if (camera_local_bad(ex.after)) ex.failures |= CAMERA_LOCAL;
    if (ex.record_count != wanted_rx || ex.after.record_level) ex.failures |= CAMERA_PROTOCOL;
    if (camera_delta(ex.after.command_tx_count, ex.before.command_tx_count) != wanted_tx ||
        camera_delta(ex.after.rx_message_count, ex.before.rx_message_count) != wanted_rx ||
        ex.after.duplicate_count != ex.before.duplicate_count ||
        flare_controller.slot_collision_count - slots_before != (kind == CAMERA_CONFIGURE ? 6U : 0U))
        ex.failures |= CAMERA_LOCAL;
    const unsigned wanted_ack = kind == CAMERA_SHOT ? 1 : 0;
    if (camera_delta(ex.after.ack_tx_count, ex.before.ack_tx_count) != wanted_ack ||
        camera_delta(ex.after.result_count, ex.before.result_count) != wanted_ack ||
        ex.after.last_tx_sequence != (uint32_t)(ex.before.last_tx_sequence + wanted_tx + wanted_ack) ||
        ex.after.ack_pending || ex.after.result_pending ||
        (kind == CAMERA_SHOT && ex.after.last_acked_sequence != ex.detection.sequence))
        ex.failures |= CAMERA_ACK;
    if (kind == CAMERA_SHOT && (Xil_In64(FLARE_CAMERA_TTL.addr + 0x90) & 1)) ex.failures |= CAMERA_TTL_ERROR;
    camera_failures |= ex.failures;
    return ex;
}

static void camera_save_exchange(const CameraExchange& ex) {
    camera_publish(camera_ds_exchanges.append_list(12,
        (double)ex.phase, (double)ex.kind, (double)ex.failures, (double)ex.record_count,
        (double)ex.armed_us, (double)ex.result_us,
        (double)camera_delta(ex.after.command_tx_count, ex.before.command_tx_count),
        (double)camera_delta(ex.after.rx_message_count, ex.before.rx_message_count),
        (double)camera_delta(ex.after.ack_tx_count, ex.before.ack_tx_count),
        (double)ex.after.tx_drop_count, (double)ex.after.record_drop_count, (double)ex.after.last_tx_sequence));
    for (unsigned i=0; i<ex.record_count; ++i) {
        const FlareRecord& rec = ex.records[i];
        camera_publish(camera_ds_records.append_list(18,
            (double)ex.phase, (double)rec.record_type, (double)rec.record_flags,
            (double)rec.payload_words, (double)rec.entry_count, (double)rec.sequence,
            (double)(uint32_t)rec.payload[0], (double)(uint32_t)(rec.payload[0] >> 32),
            (double)(uint32_t)rec.payload[1], (double)(uint32_t)(rec.payload[1] >> 32),
            (double)(uint32_t)rec.payload[2], (double)(uint32_t)(rec.payload[2] >> 32),
            (double)(uint32_t)rec.payload[3], (double)(uint32_t)(rec.payload[3] >> 32),
            (double)(uint32_t)rec.payload[4], (double)(uint32_t)(rec.payload[4] >> 32),
            (double)(uint32_t)rec.payload[5], (double)(uint32_t)(rec.payload[5] >> 32)));
    }
}

static int run_camera_test() {
    init();
    xil_printf(FLARE_CAMERA_TEST_NAME ": %d shots, jb_0 HIGH 1 us, RAW_REQUIRED\r\n", FLARE_CAMERA_SHOTS);
    if (!camera_mapped(flare_controller) || !camera_mapped(FLARE_CAMERA_TTL)) camera_failures |= CAMERA_MAP;
    FlareConfig cfg = {};
    cfg.ion_count=1; cfg.background_enable=0; cfg.background_scale_q32=1U << 20;
    cfg.signal_width=CAMERA_ROI_WIDTH; cfg.signal_height=CAMERA_ROI_HEIGHT;
    cfg.background_width=CAMERA_ROI_WIDTH; cfg.background_height=CAMERA_ROI_HEIGHT;
    cfg.confidence_margin32=CAMERA_CONFIDENCE_MARGIN;
    cfg.ion_x[0]=CAMERA_ROI_X; cfg.ion_y[0]=CAMERA_ROI_Y; cfg.ion_threshold[0]=CAMERA_THRESHOLD;
    uint32_t shot_id=0, baseline_shots=0, baseline_results=0;
    unsigned attempted=0, passed=0, failed_phase=0;
    unsigned shot_exchange_indices[FLARE_CAMERA_SHOTS] = {};
    if (!camera_failures) {
        flare_controller.flush_commands();
        // init() resets the time counter, not each module's command FIFO.
        // Discard TTL events left by an interrupted run before any auto_start.
        // AXI2FIFO reset bit 0 flushes commands; bit 2 would reset the outputs.
        Xil_Out128(FLARE_CAMERA_TTL.addr | 0x10, MAKE128CONST(0, 1));
        usleep(1000);
        flare_controller.clear_records(); flare_controller.clear_counters(); flare_controller.clear_sticky();
        flare_controller.slot_collision_count=0;
        usleep(1000);
        CameraExchange& initial = camera_exchange(CAMERA_STATUS, cfg);
        if (!camera_failures && !camera_idle(initial.records[0], false, 0)) camera_failures |= CAMERA_PEER;
        if (!camera_failures) camera_exchange(CAMERA_STOP_STREAM, cfg);
        if (!camera_failures) {
            CameraExchange& idle = camera_exchange(CAMERA_STATUS, cfg);
            if (!camera_failures && !camera_idle(idle.records[0], false, 0)) camera_failures |= CAMERA_PEER;
            if (!camera_failures) {
                cfg.config_token32=(uint32_t)idle.records[0].payload[0] + 1U;
                if (!cfg.config_token32) cfg.config_token32=1;
                shot_id=(uint32_t)idle.records[0].payload[1];
                baseline_shots=(uint32_t)idle.records[0].payload[2];
                baseline_results=(uint32_t)(idle.records[0].payload[2] >> 32);
                if (baseline_shots > 0xffffffffU - FLARE_CAMERA_SHOTS ||
                    baseline_results > 0xffffffffU - FLARE_CAMERA_SHOTS) camera_failures |= CAMERA_PEER;
            }
        }
        if (!camera_failures) camera_exchange(CAMERA_CONFIGURE, cfg);
        // The configuration loader can still be busy at CONFIG_STATUS arrival.
        for (unsigned retry=0; retry<10 && !camera_failures; ++retry) {
            CameraExchange& ready = camera_exchange(CAMERA_STATUS, cfg);
            if (camera_failures || camera_idle(ready.records[0], true, cfg.config_token32)) break;
            if (retry==9) camera_failures |= CAMERA_PEER;
            else usleep(10000);
        }
    }
    for (unsigned index=0; index<FLARE_CAMERA_SHOTS && !camera_failures; ++index) {
        if (++shot_id==0) ++shot_id;
        shot_exchange_indices[attempted++]=camera_exchange_count;
        CameraExchange& shot = camera_exchange(CAMERA_SHOT, cfg, shot_id);
        xil_printf("camera shot %u: result code %d errors 0x%x state 0x%x valid 0x%x low_conf 0x%x sat 0x%x mask 0x%x\r\n",
            (unsigned)shot_id, (int)shot.detection.result_code, (unsigned)shot.detection.measurement_error_flags,
            (unsigned)shot.detection.state_mask, (unsigned)shot.detection.valid_mask,
            (unsigned)shot.detection.low_confidence_mask, (unsigned)shot.detection.saturation_mask,
            (unsigned)shot.failures);
        if (!camera_failures && index && shot.detection.sequence ==
            camera_exchanges[shot_exchange_indices[index-1]].detection.sequence) camera_failures |= CAMERA_PROTOCOL;
        for (unsigned retry=0; retry<10 && !camera_failures; ++retry) {
            CameraExchange& post = camera_exchange(CAMERA_STATUS, cfg);
            if (!camera_failures && (
                post.records[0].payload[1] != shot_id ||
                (uint32_t)post.records[0].payload[2] != baseline_shots + index + 1U ||
                (uint32_t)(post.records[0].payload[2] >> 32) != baseline_results + index + 1U ||
                ((post.records[0].payload[0] >> 48) & 255))) camera_failures |= CAMERA_PEER;
            if (camera_failures || camera_idle(post.records[0], true, cfg.config_token32)) break;
            if (retry==9) camera_failures |= CAMERA_PEER;
            else usleep(10000);
        }
        if (!camera_failures) ++passed;
        if (index+1<FLARE_CAMERA_SHOTS && !camera_failures) usleep(CAMERA_SHOT_GAP_US);
    }
    auto_stop();
    if (camera_failures) failed_phase=camera_exchange_count;
    // Publish fixed-width doubles only after all RTIO/shot activity stopped.
    for (unsigned i=0; i<camera_exchange_count; ++i) camera_save_exchange(camera_exchanges[i]);
    camera_publish(camera_ds_settings.append_list(14,
        (double)cfg.config_token32, (double)cfg.ion_count, (double)cfg.background_enable,
        (double)cfg.background_scale_q32, (double)cfg.signal_width, (double)cfg.signal_height,
        (double)cfg.background_width, (double)cfg.background_height, (double)cfg.background_dx,
        (double)cfg.background_dy, (double)cfg.confidence_margin32,
        (double)cfg.ion_x[0], (double)cfg.ion_y[0], (double)cfg.ion_threshold[0]));
    camera_publish(camera_ds_ttl.append_list(5, (double)FLARE_CAMERA_TTL.addr,
        (double)FLARE_CAMERA_TTL.get_channel(), (double)CAMERA_PULSE_NS,
        (double)CAMERA_SHOT_GAP_US, (double)FLARE_RAW_REQUIRED));
    for (unsigned i=0; i<attempted; ++i) {
        const CameraExchange& ex=camera_exchanges[shot_exchange_indices[i]];
        const FlareResult& result=ex.detection;
        camera_publish(camera_ds_results.append_list(20,
            (double)(i+1), (double)result.shot_id, (double)result.config_token32,
            (double)result.sequence, (double)result.state_mask, (double)result.valid_mask,
            (double)result.low_confidence_mask, (double)result.saturation_mask,
            (double)result.measurement_error_flags, (double)result.result_code,
            (double)result.ion_count, (double)result.config_current,
            (double)result.classifier_complete, (double)result.frame_valid,
            (double)ex.armed, (double)ex.raw_enabled, (double)ex.armed_us, (double)ex.result_us,
            (double)camera_delta(ex.after.ack_tx_count, ex.before.ack_tx_count),
            (double)(!ex.failures && i<passed)));
    }
    if (!camera_publication_ok) camera_failures |= CAMERA_HOST;
    const bool board_ok=!camera_failures && passed==FLARE_CAMERA_SHOTS;
    camera_publish(camera_ds_summary.append_list(8, (double)board_ok, (double)camera_failures,
        (double)failed_phase, (double)FLARE_CAMERA_SHOTS, (double)attempted, (double)passed,
        (double)cfg.config_token32, (double)FLARE_RAW_REQUIRED));
    xil_printf(FLARE_CAMERA_TEST_NAME " DETECTION %s: %d/%d shots, mask 0x%x. PC RAW verification required.\r\n",
        board_ok && camera_publication_ok ? "PASS" : "FAIL", (int)passed, FLARE_CAMERA_SHOTS,
        (unsigned)camera_failures);
    return board_ok && camera_publication_ok ? 0 : 1;
}
#endif
