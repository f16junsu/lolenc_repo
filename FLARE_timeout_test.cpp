/*
 * FLARE_timeout_test.cpp -- two camera-free SOF-timeout shots and automatic ACK.
 * Run with the existing lolenc_repo configuration/device_config.
 * This replaces the peer measurement configuration with a one-ion test config,
 * stops the FLARE storage stream, and never touches TTL or sends STREAM_START.
 * Camera acquisition must be stopped/disconnected. RAW is disabled.
 * Successful completion leaves the test configuration active and stream stopped.
 * A valid FRAME_INVALID/MISSING_SOF result is the EXPECTED successful outcome.
 * This does not test pixels, camera triggering, classification, or DDR writes.
 *
 * Phases: 1 initial STATUS, 2 STREAM_STOP, 3 idle STATUS, 4 configure,
 * 5 ready STATUS, 6 ready HEARTBEAT,
 * 7 ARM/timeout/result/auto-ACK #1, 8 post-shot STATUS,
 * 9 CLEAR_ERROR(MISSING_SOF only), 10 ready STATUS,
 * 11 ARM/timeout/result/auto-ACK #2, 12 post-shot STATUS,
 * 13 CLEAR_ERROR(MISSING_SOF only), 14 ready STATUS.
 * Both shots use fresh IDs derived from the initial STATUS current_shot_id.
 * Stop on first failure, preserving records; no broad error clear or reset.
 * A failed exchange may leave a shot/result pending on the peer.
 *
 * LOLENC: init stops/resets RTIO, not the PHY. delay advances software time.
 * Queue at 100 us before auto_start, poll with XTime/usleep. ARMED_ACK deadline
 * 500 ms; result/auto-ACK deadline 3 s (current RTL SOF timeout default 1 s).
 * STATE_RESULT sequence is independent of ARM sequence; automatic ACK must
 * acknowledge the RESULT header sequence. It also consumes one TX sequence.
 * command_tx_count excludes auto-ACK; ack_tx_count counts it separately.
 * No manual RESULT_ACK: the default controller RTL acknowledges committed records.
 * No IPC while requests are active. Return from main for ELF_Runner handshake.
 * The existing dataset IPC has no timeout if the host disconnects.
 *
 * Expected result: code=1 FRAME_INVALID, flags=0x0002 MISSING_SOF,
 * masks=0, frame_valid=0, classifier_complete=0, config_current=0, ion_count=0.
 * shot_manager clears the result ion_count when frame/classification is invalid;
 * this field is not the configured ion count (the test configuration has one).
 * config_current=0 is expected without a classifier mask; it does not mean the
 * stored configuration vanished. Verify its token/config_valid in telemetry.
 * STATUS must show idle, no pending result/fatal, matching shot ID/token,
 * last_reason=2 SOF_TIMEOUT, and shot/result counts increasing once per shot.
 * CLEAR_ERROR clears the sticky cause, not historical counters/last_reason.
 * Its sticky clear is not directly readable in this telemetry layout; recovery
 * is checked by ready and, for the first shot, actual acceptance of a second ARM.
 *
 * Datasets (fixed-width doubles; payload words split lo32/hi32):
 * flare_timeout_summary [pass,mask,failed_phase,token,shot1,shot2,shots_passed,
 *   command_tx,ack_tx,rx,result_count,attempted_phases]
 * flare_timeout_shots [phase,shot_id,token,ARM_sequence,RESULT_sequence,
 *   armed_valid,result_valid,armed_us,result_received_us,ack_delta,result_delta,
 *   duplicate_delta,last_acked_sequence,pass]
 * flare_timeout_trials [phase,pass,mask,matches,first_reply_us,records,
 *   command_tx_delta,rx_delta,expected_command_tx,expected_rx,slot_delta,queue_error]
 * flare_timeout_settings [id,token,ion_count,background_enable,scale_q20,
 *   signal_w,signal_h,background_w,background_h,dx,dy,margin,x0,y0,threshold]
 * flare_timeout_status [phase,channel_up,lane_up,tx_ready,sticky_mask,command_tx,rx,
 *   hard,soft,crc,header,phy_overflow,tx_drop,record_drop,tx_abort,record_level,
 *   next_tx_sequence,last_msg_type,last_msg_sequence,last_nack,last_rx_error,slots]
 * flare_timeout_records [phase,type,flags,words,entries,sequence,W0lo,W0hi,...W5hi]
 * flare_timeout_env [channel,expected_base,firmware_base,virtual_write,queued_pages]
 * Expected normal totals: command_tx=20, auto_ack_tx=2, RX=13, results=2,
 * slot_adjustments=6, 14 phases. Timings include 100 us reservation/PS polling.
 *
 * mask: 01 map/queue, 02 link, 04 response timeout, 08 bad record,
 * 10 response shape/sequence/duplicate, 20 unexpected record,
 * 40 local counters/errors/slots, 80 host output, 100 local configure rejection,
 * 200 config response, 400 token/config_valid, 800 peer not idle/ready,
 * 1000 ARM rejection/identity, 2000 wrong result, 4000 missing/wrong auto-ACK,
 * 8000 post-shot status/counters. An unexpected timeout is mask 04; expected
 * SOF timeout is result flag 0002 and MUST NOT set the test failure mask.
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xtime_l.h"

extern FLAREController flare_controller;
static DatasetFloatList ds_summary("flare_timeout_summary", true);
static DatasetFloatList ds_trials("flare_timeout_trials", true);
static DatasetFloatList ds_status("flare_timeout_status", true);
static DatasetFloatList ds_records("flare_timeout_records", true);
static DatasetFloatList ds_env("flare_timeout_env", true);
static DatasetFloatList ds_settings("flare_timeout_settings", true);
static const uint32_t LINK_WAIT_US = 2000000;
static const uint32_t REPLY_TIMEOUT_US = 500000;
static const uint32_t QUIET_US = 2000;
static const uint32_t STOP_SETTLE_US = 10000;
static const unsigned MAX_RECORDS = 16;
enum Failure {
    BAD_ENV=0x01, LINK_DOWN=0x02, TIMEOUT=0x04, BAD_RECORD=0x08,
    BAD_REPLY=0x10, EXTRA_RECORD=0x20, LOCAL_ERROR=0x40, HOST_OUTPUT=0x80,
    LOCAL_CONFIG=0x100, CONFIG_REJECTED=0x200, TOKEN_MISMATCH=0x400,
    PEER_NOT_IDLE=0x800, BAD_ARM=0x1000, BAD_RESULT=0x2000, BAD_ACK=0x4000, BAD_RECOVERY=0x8000
};
static bool publication_ok = true;
static void check_publication(err_t result) {
    if (result != ERR_OKAY) {
        publication_ok = false;
        xil_printf("FLARE_timeout_test: dataset publication failed\r\n");
    }
}
struct Trial {
    unsigned phase;
    uint32_t failures;
    unsigned matches;
    uint32_t reply_us;
    unsigned count;
    unsigned expected_tx, expected_rx;
    uint32_t slots_before, slots_after;
    flare_err_t queue_error;
    FlareRecord records[MAX_RECORDS];
    FlareStatus before, after;
};
static uint32_t elapsed_us(XTime start) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(((now - start) * 1000000ULL) / COUNTS_PER_SECOND);
}

static bool link_up(const FlareStatus& st) {
    return st.channel_up && st.lane_up;
}

static uint32_t sticky_mask(const FlareStatus& st) {
    return ((uint32_t)st.sticky_rx_overflow << 5) |
           ((uint32_t)st.sticky_tx_overflow << 6) |
           ((uint32_t)st.sticky_crc_error << 7) |
           ((uint32_t)st.sticky_hard_error << 8) |
           ((uint32_t)st.sticky_soft_error << 9) |
           ((uint32_t)st.sticky_timestamp_error << 10) |
           ((uint32_t)st.sticky_truncated << 11) |
           ((uint32_t)st.sticky_record_drop << 12) |
           ((uint32_t)st.sticky_tx_abort << 13);
}

static bool local_error(const FlareStatus& st) {
    return sticky_mask(st) || st.hard_error_count || st.soft_error_count ||
           st.crc_error_count || st.header_error_count || st.phy_overflow_count ||
           st.tx_drop_count || st.record_drop_count || st.tx_abort_count;
}

static void save_status(unsigned phase, const FlareStatus& st, uint32_t slots) {
    check_publication(ds_status.append_list(22,
        (double)phase, (double)st.channel_up, (double)st.lane_up,
        (double)st.tx_ready, (double)sticky_mask(st),
        (double)st.command_tx_count, (double)st.rx_message_count,
        (double)st.hard_error_count, (double)st.soft_error_count,
        (double)st.crc_error_count, (double)st.header_error_count,
        (double)st.phy_overflow_count, (double)st.tx_drop_count,
        (double)st.record_drop_count, (double)st.tx_abort_count,
        (double)st.record_level, (double)st.last_tx_sequence,
        (double)st.last_msg_type, (double)st.last_msg_sequence,
        (double)st.last_nack_reason, (double)st.last_rx_error_reason,
        (double)slots));
    xil_printf("FLARE phase %d: link %d/%d tx %d rx %d sticky 0x%x "
               "hard %d soft %d crc %d header %d txdrop %d recdrop %d\r\n",
               (int)phase, (int)st.channel_up, (int)st.lane_up,
               (int)st.command_tx_count, (int)st.rx_message_count,
               (unsigned)sticky_mask(st), (int)st.hard_error_count,
               (int)st.soft_error_count, (int)st.crc_error_count,
               (int)st.header_error_count, (int)st.tx_drop_count,
               (int)st.record_drop_count);
}

static void save_record(unsigned phase, const FlareRecord& rec) {
    double words[12];
    for (unsigned i = 0; i < 6; ++i) {
        words[2*i] = (double)(uint32_t)rec.payload[i];
        words[2*i+1] = (double)(uint32_t)(rec.payload[i] >> 32);
    }
    check_publication(ds_records.append_list(18,
        (double)phase, (double)rec.record_type, (double)rec.record_flags,
        (double)rec.payload_words, (double)rec.entry_count, (double)rec.sequence,
        words[0], words[1], words[2], words[3], words[4], words[5],
        words[6], words[7], words[8], words[9], words[10], words[11]));
    xil_printf("RX phase %d: type 0x%x sequence 0x%x words %d flags 0x%x\r\n",
               (int)phase, (unsigned)rec.record_type, (unsigned)rec.sequence,
               (int)rec.payload_words, (unsigned)rec.record_flags);
    for (unsigned i = 0; i < rec.payload_words && i < 6; ++i) {
        xil_printf("  W%d = 0x%08x%08x\r\n", (int)i,
                   (unsigned)(rec.payload[i] >> 32), (unsigned)rec.payload[i]);
    }
    if (rec.record_type == FLARE_MSG_STATE_RESULT && rec.payload_words == 3) {
        FlareResult decoded = {};
        FLAREController::decode_result(rec, &decoded);
        xil_printf("  result: shot %u token 0x%x code %d errors 0x%x valid 0x%x frame %d classifier %d config_current %d ion_count %d\r\n",
            (unsigned)decoded.shot_id, (unsigned)decoded.config_token32,
            (int)decoded.result_code, (unsigned)decoded.measurement_error_flags,
            (unsigned)decoded.valid_mask, (int)decoded.frame_valid,
            (int)decoded.classifier_complete, (int)decoded.config_current, (int)decoded.ion_count);
    } else if (rec.record_type == FLARE_MSG_ARMED_ACK && rec.payload_words == 3) {
        xil_printf("  armed: shot %u status %d raw %d reason %d version 0x%x errors 0x%x\r\n",
            (unsigned)rec.payload[0], (int)(rec.payload[1] & 0xff),
            (int)((rec.payload[1] >> 8) & 0xff), (int)((rec.payload[1] >> 16) & 0xff),
            (unsigned)((rec.payload[1] >> 32) & 0xffff), (unsigned)rec.payload[2]);
    } else if (rec.record_type == FLARE_MSG_CONFIG_STATUS && rec.payload_words == 2) {
        FlareConfigStatus decoded = {};
        FLAREController::decode_config_status(rec, &decoded);
        xil_printf("  config: request_seq 0x%x id %d status %d version 0x%x errors 0x%x\r\n",
            (unsigned)decoded.request_sequence, (int)decoded.config_id,
            (int)decoded.status_code, (unsigned)decoded.config_version, (unsigned)decoded.error_flags);
    } else if (rec.record_type == FLARE_MSG_NACK && rec.payload_words >= 1) {
        xil_printf("  nack: request_seq 0x%x reason 0x%x\r\n",
            (unsigned)rec.payload[0], (unsigned)((rec.payload[0] >> 32) & 0xff));
    } else if (rec.record_type == FLARE_MSG_HEARTBEAT_RESPONSE) {
        xil_printf("  heartbeat: layout %d token 0x%x uptime_low16_ms %d health 0x%x\r\n",
                   (int)(rec.payload[0] >> 56), (unsigned)rec.payload[0],
                   (int)((rec.payload[0] >> 32) & 0xffff),
                   (unsigned)((rec.payload[0] >> 48) & 0xff));
    } else if (rec.record_type == FLARE_MSG_STATUS_RESPONSE) {
        xil_printf("  status: layout %d token 0x%x shot_state %d last_reason %d health 0x%x\r\n",
                   (int)((rec.payload[0] >> 32) & 0xff),
                   (unsigned)rec.payload[0], (int)((rec.payload[0] >> 40) & 0xff),
                   (int)((rec.payload[0] >> 48) & 0xff),
                   (unsigned)(rec.payload[0] >> 56));
    }
}


static DatasetFloatList ds_shots("flare_timeout_shots", true);
static uint32_t baseline_shots = 0, baseline_results = 0;
static uint32_t shot_ids[2] = {};

static bool passed(const Trial& trial) {
    return trial.failures == 0 && trial.matches == trial.expected_rx;
}

static uint8_t wanted_type(unsigned phase) {
    if (phase == 2 || phase == 9 || phase == 13) return 0;
    if (phase == 4) return FLARE_MSG_CONFIG_STATUS;
    if (phase == 6) return FLARE_MSG_HEARTBEAT_RESPONSE;
    return FLARE_MSG_STATUS_RESPONSE;
}

static void validate_reply(Trial& trial, const FlareRecord& rec,
                           const FlareConfig* cfg) {
    const uint8_t wanted = wanted_type(trial.phase);
    const bool config = wanted == FLARE_MSG_CONFIG_STATUS;
    const bool heartbeat = wanted == FLARE_MSG_HEARTBEAT_RESPONSE;
    const unsigned words = config ? 2 : heartbeat ? 1 : 6;
    const unsigned entries = config || heartbeat ? 2 : 4;
    const uint32_t sequence = trial.before.last_tx_sequence + trial.expected_tx - 1U;
    if (rec.record_flags || rec.payload_words != words || rec.entry_count != entries ||
        rec.sequence != sequence || trial.matches != 1) {
        trial.failures |= BAD_REPLY;
        return;
    }
    if (config) {
        FlareConfigStatus decoded = {};
        if (FLAREController::decode_config_status(rec, &decoded) != FLARE_OK ||
            decoded.request_sequence != sequence || (rec.payload[1] >> 32) != 0) {
            trial.failures |= BAD_REPLY;
        }
        if (decoded.config_id != 0 || decoded.status_code != 0 || decoded.error_flags ||
            decoded.config_version != (uint16_t)cfg->config_token32) {
            trial.failures |= CONFIG_REJECTED;
        }
        return;
    }
    const uint64_t word = rec.payload[0];
    const unsigned layout = heartbeat ? (unsigned)(word >> 56) : (unsigned)((word >> 32) & 0xff);
    const unsigned health = heartbeat ? (unsigned)((word >> 48) & 0xff) : (unsigned)(word >> 56);
    if (layout != 2) trial.failures |= BAD_REPLY;
    // Bit 0 READY is deliberately not a pass condition.
    const unsigned busy_mask = trial.phase == 1 ? 0xb8 : 0xbc;
    if (!(health & 0x40) || (health & busy_mask) ||
        (!heartbeat && ((word >> 40) & 0xff) != 0)) trial.failures |= PEER_NOT_IDLE;
    if (cfg && (!(health & 0x02) || (uint32_t)word != cfg->config_token32))
        trial.failures |= TOKEN_MISMATCH;
    if (trial.phase == 5 || trial.phase == 6 || trial.phase == 10 || trial.phase == 14) {
        if (!(health & 1)) trial.failures |= PEER_NOT_IDLE;
    }
    if (trial.phase == 8 || trial.phase == 10 || trial.phase == 12 || trial.phase == 14) {
        const unsigned completed = trial.phase >= 12 ? 2 : 1;
        if ((unsigned)((word >> 48) & 0xff) != 2 ||
            rec.payload[1] != shot_ids[completed - 1] ||
            (uint32_t)rec.payload[2] != baseline_shots + completed ||
            (uint32_t)(rec.payload[2] >> 32) != baseline_results + completed)
            trial.failures |= BAD_RECOVERY;
    }

}

static void run_trial(Trial& trial, const FlareConfig* cfg, const FlareStatus& previous) {
    const uint8_t wanted = wanted_type(trial.phase);
    trial.expected_tx = wanted == FLARE_MSG_CONFIG_STATUS ? 5 + 2 * cfg->ion_count : 1;
    trial.expected_rx = wanted ? 1 : 0;
    init();
    flare_controller.reset_slot_tracking();
    trial.before = trial.after = flare_controller.read_status();
    trial.slots_before = trial.slots_after = flare_controller.slot_collision_count;
    if (!link_up(trial.before)) trial.failures |= LINK_DOWN;
    if (local_error(trial.before)) trial.failures |= LOCAL_ERROR;
    if (trial.before.record_level || trial.before.ack_pending || trial.before.result_pending ||
        trial.before.command_tx_count != previous.command_tx_count ||
        trial.before.rx_message_count != previous.rx_message_count ||
        trial.before.ack_tx_count != previous.ack_tx_count ||
        trial.before.result_count != previous.result_count ||
        trial.before.duplicate_count != previous.duplicate_count ||
        trial.before.last_tx_sequence != previous.last_tx_sequence) trial.failures |= EXTRA_RECORD;
    if (trial.failures) return;

    delay(100 * us);
    if (!wanted) {
        if (trial.phase == 2) flare_controller.stream_stop();
        else flare_controller.clear_error(0x00000002U); // Only expected MISSING_SOF.
    }
    else if (wanted == FLARE_MSG_CONFIG_STATUS) trial.queue_error = flare_controller.configure(*cfg);
    else if (wanted == FLARE_MSG_HEARTBEAT_RESPONSE) flare_controller.heartbeat();
    else flare_controller.request_status();
    trial.slots_after = flare_controller.slot_collision_count;
    if (trial.queue_error != FLARE_OK) {
        trial.failures |= LOCAL_CONFIG;
        return;
    }
    XTime start;
    XTime_GetTime(&start);
    auto_start();
    for (;;) {
        const uint32_t waited = elapsed_us(start);
        if (waited >= REPLY_TIMEOUT_US) {
            if (trial.matches != trial.expected_rx ||
                flare_controller.read_status().command_tx_count != trial.before.command_tx_count + trial.expected_tx)
                trial.failures |= TIMEOUT;
            break;
        }
        const FlareStatus live = flare_controller.read_status();
        if (!link_up(live)) { trial.failures |= LINK_DOWN; break; }
        if (local_error(live)) { trial.failures |= LOCAL_ERROR; break; }
        if (flare_controller.records_available()) {
            if (trial.count == MAX_RECORDS) { trial.failures |= EXTRA_RECORD; break; }
            FlareRecord& rec = trial.records[trial.count];
            if (flare_controller.read_record(&rec) != FLARE_OK) {
                trial.failures |= BAD_RECORD;
                break;
            }
            ++trial.count;
            if (!wanted || rec.record_type != wanted) trial.failures |= EXTRA_RECORD;
            else {
                ++trial.matches;
                if (trial.matches == 1) trial.reply_us = elapsed_us(start);
                validate_reply(trial, rec, cfg);
            }
            if (trial.failures) break;
        } else if (wanted && trial.matches && waited - trial.reply_us >= QUIET_US) {
            break;
        } else if (!wanted && waited >= STOP_SETTLE_US &&
                   live.command_tx_count == trial.before.command_tx_count + 1) {
            break;
        }
        usleep(100);
    }
    auto_stop();
    trial.after = flare_controller.read_status();
    if (!link_up(trial.after)) trial.failures |= LINK_DOWN;
    if (trial.after.record_level) trial.failures |= EXTRA_RECORD;
    const uint32_t expected_slots = wanted == FLARE_MSG_CONFIG_STATUS ? trial.expected_tx - 1 : 0;
    if (local_error(trial.after) ||
        (int)trial.after.command_tx_count - (int)trial.before.command_tx_count != (int)trial.expected_tx ||
        (int)trial.after.rx_message_count - (int)trial.before.rx_message_count != (int)trial.expected_rx ||
        trial.after.ack_tx_count != trial.before.ack_tx_count ||
        trial.after.result_count != trial.before.result_count ||
        trial.after.duplicate_count != trial.before.duplicate_count ||
        trial.after.last_tx_sequence != (uint32_t)(trial.before.last_tx_sequence + trial.expected_tx) ||
        trial.slots_after - trial.slots_before != expected_slots) trial.failures |= LOCAL_ERROR;
}

static void save_trial(const Trial& trial) {
    check_publication(ds_trials.append_list(12,
        (double)trial.phase, (double)passed(trial), (double)trial.failures,
        (double)trial.matches, (double)trial.reply_us, (double)trial.count,
        (double)((int)trial.after.command_tx_count - (int)trial.before.command_tx_count),
        (double)((int)trial.after.rx_message_count - (int)trial.before.rx_message_count),
        (double)trial.expected_tx, (double)trial.expected_rx,
        (double)(trial.slots_after - trial.slots_before), (double)trial.queue_error));
    save_status(trial.phase, trial.after, trial.slots_after);
    for (unsigned i = 0; i < trial.count; ++i) save_record(trial.phase, trial.records[i]);
    xil_printf("FLARE timeout phase %d: %s mask 0x%x slots +%d local_config_error %d\r\n",
        (int)trial.phase, passed(trial) ? "PASS" : "FAIL", (unsigned)trial.failures,
        (int)(trial.slots_after - trial.slots_before), (int)trial.queue_error);
}

static uint32_t next_token(uint32_t previous) {
    do { ++previous; } while (!previous || previous == flare_controller.last_committed_token);
    return previous;
}

static FlareConfig make_config(uint32_t token, bool second) {
    FlareConfig cfg = {};
    cfg.ion_count = 1;
    cfg.background_enable = second ? 1 : 0;
    cfg.background_scale_q32 = 1u << 20;
    cfg.signal_width = cfg.signal_height = 8;
    cfg.background_width = cfg.background_height = 8;
    cfg.background_dx = second ? -16 : 0;
    cfg.background_dy = 16;
    cfg.confidence_margin32 = second ? 1000 : 500;
    cfg.ion_x[0] = second ? 116 : 100;
    cfg.ion_y[0] = 200;
    cfg.ion_threshold[0] = second ? 7000 : 5000;
    cfg.config_token32 = token;
    return cfg;
}

static void save_config(unsigned id, const FlareConfig& cfg) {
    check_publication(ds_settings.append_list(15, (double)id, (double)cfg.config_token32,
        (double)cfg.ion_count, (double)cfg.background_enable, (double)cfg.background_scale_q32,
        (double)cfg.signal_width, (double)cfg.signal_height,
        (double)cfg.background_width, (double)cfg.background_height,
        (double)cfg.background_dx, (double)cfg.background_dy, (double)cfg.confidence_margin32,
        (double)cfg.ion_x[0], (double)cfg.ion_y[0], (double)cfg.ion_threshold[0]));
}


struct Shot {
    uint32_t id, result_sequence, armed_us, result_us;
    bool armed, result;
};

static void run_shot(Trial& trial, Shot& shot, const FlareConfig& cfg, const FlareStatus& previous) {
    trial.expected_tx = 1;
    trial.expected_rx = 2;
    init();
    flare_controller.reset_slot_tracking();
    trial.before = trial.after = flare_controller.read_status();
    trial.slots_before = trial.slots_after = flare_controller.slot_collision_count;
    if (!link_up(trial.before)) trial.failures |= LINK_DOWN;
    if (local_error(trial.before)) trial.failures |= LOCAL_ERROR;
    if (trial.before.record_level || trial.before.ack_pending || trial.before.result_pending ||
        trial.before.command_tx_count != previous.command_tx_count ||
        trial.before.rx_message_count != previous.rx_message_count ||
        trial.before.ack_tx_count != previous.ack_tx_count ||
        trial.before.result_count != previous.result_count ||
        trial.before.duplicate_count != previous.duplicate_count ||
        trial.before.last_tx_sequence != previous.last_tx_sequence) trial.failures |= EXTRA_RECORD;
    if (trial.failures) return;
    delay(100 * us);
    flare_controller.arm_trigger(shot.id, FLARE_RAW_DISABLED);
    XTime start;
    XTime_GetTime(&start);
    auto_start();
    for (;;) {
        const uint32_t waited = elapsed_us(start);
        if ((!shot.armed && waited >= 500000) || waited >= 3000000) {
            if (!shot.armed || !shot.result) trial.failures |= TIMEOUT;
            if (shot.result) trial.failures |= BAD_ACK;
            break;
        }
        const FlareStatus live = flare_controller.read_status();
        if (!link_up(live)) { trial.failures |= LINK_DOWN; break; }
        if (local_error(live)) { trial.failures |= LOCAL_ERROR; break; }
        if (flare_controller.records_available()) {
            if (trial.count == MAX_RECORDS) { trial.failures |= EXTRA_RECORD; break; }
            FlareRecord& rec = trial.records[trial.count];
            if (flare_controller.read_record(&rec) != FLARE_OK) { trial.failures |= BAD_RECORD; break; }
            ++trial.count;
            if (rec.record_type != FLARE_MSG_ARMED_ACK && rec.record_type != FLARE_MSG_STATE_RESULT) {
                trial.failures |= EXTRA_RECORD;
                break;
            }
            if (rec.record_flags || rec.payload_words != 3 || rec.entry_count != 3) {
                trial.failures |= BAD_REPLY;
                break;
            }
            if (rec.record_type == FLARE_MSG_ARMED_ACK) {
                if (shot.armed || rec.sequence != trial.before.last_tx_sequence ||
                    rec.payload[0] != shot.id || (rec.payload[1] & 0xffff) != 0 ||
                    ((rec.payload[1] >> 24) & 0xff) != 0 ||
                    ((rec.payload[1] >> 32) & 0xffff) != (uint16_t)cfg.config_token32 ||
                    (rec.payload[1] >> 48) != 0 || rec.payload[2] != 0) {
                    trial.failures |= BAD_ARM;
                } else { shot.armed = true; shot.armed_us = elapsed_us(start); }
            } else {
                FlareResult result = {};
                // Keep arrival time/sequence even if payload validation fails.
                if (!shot.result) {
                    shot.result_us = elapsed_us(start);
                    shot.result_sequence = rec.sequence;
                }
                if (shot.result || FLAREController::decode_result(rec, &result) != FLARE_OK ||
                    result.shot_id != shot.id || result.config_token32 != cfg.config_token32 ||
                    result.result_code != 1 || result.measurement_error_flags != 0x0002 ||
                    result.state_mask || result.valid_mask || result.low_confidence_mask ||
                    result.saturation_mask || result.frame_valid || result.classifier_complete ||
                    result.config_current || result.ion_count != 0 || (rec.payload[2] >> 32) != 0) {
                    trial.failures |= BAD_RESULT;
                } else {
                    shot.result = true;
                    shot.result_us = elapsed_us(start);
                    shot.result_sequence = rec.sequence;
                }
            }
            trial.matches = (unsigned)shot.armed + (unsigned)shot.result;
            trial.reply_us = shot.armed_us;
            if (trial.failures) break;
        } else if (shot.armed && shot.result && waited - shot.result_us >= QUIET_US &&
                   live.ack_tx_count == trial.before.ack_tx_count + 1 && !live.ack_pending &&
                   !live.result_pending && live.last_acked_sequence == shot.result_sequence) {
            break;
        }
        usleep(100);
    }
    auto_stop();
    trial.after = flare_controller.read_status();
    trial.slots_after = flare_controller.slot_collision_count;
    if (!link_up(trial.after)) trial.failures |= LINK_DOWN;
    if (shot.result && (trial.after.ack_tx_count != trial.before.ack_tx_count + 1 ||
        trial.after.ack_pending || trial.after.last_acked_sequence != shot.result_sequence)) trial.failures |= BAD_ACK;
    if (trial.after.record_level || trial.after.result_pending) trial.failures |= EXTRA_RECORD;
    if (local_error(trial.after) ||
        trial.after.command_tx_count != trial.before.command_tx_count + 1 ||
        trial.after.rx_message_count != trial.before.rx_message_count + 2 ||
        trial.after.result_count != trial.before.result_count + 1 ||
        trial.after.duplicate_count != trial.before.duplicate_count ||
        trial.after.last_tx_sequence != (uint32_t)(trial.before.last_tx_sequence + 2U) ||
        trial.slots_after != trial.slots_before) trial.failures |= LOCAL_ERROR;
}

static void save_shot(const Trial& trial, const Shot& shot, uint32_t token) {
    check_publication(ds_shots.append_list(14,
        (double)trial.phase, (double)shot.id, (double)token,
        (double)trial.before.last_tx_sequence, (double)shot.result_sequence,
        (double)shot.armed, (double)shot.result, (double)shot.armed_us, (double)shot.result_us,
        (double)((int)trial.after.ack_tx_count - (int)trial.before.ack_tx_count),
        (double)((int)trial.after.result_count - (int)trial.before.result_count),
        (double)((int)trial.after.duplicate_count - (int)trial.before.duplicate_count),
        (double)trial.after.last_acked_sequence, (double)passed(trial)));
    xil_printf("Timeout shot %u: %s ARM %u us RESULT %u us auto_ACK +%d result_seq 0x%x\r\n",
        (unsigned)shot.id, passed(trial) ? "PASS" : "FAIL",
        (unsigned)shot.armed_us, (unsigned)shot.result_us,
        (int)trial.after.ack_tx_count - (int)trial.before.ack_tx_count, (unsigned)shot.result_sequence);
}
int main() {
    init();
    xil_printf("FLARE_timeout_test: 2 SOF-timeout shots, no TTL, RAW disabled\r\n");
    const uint64_t channel = flare_controller.virtual_table_channel;
    const uint64_t firmware_base = channel < 64 ? virtual_table->module_addr[channel] : 0;
    const uint64_t virtual_write = channel < 64 ? virtual_table->virtual_write[channel] : 0;
    const uint64_t queued_pages = channel < 64 ? virtual_table->table_size[channel] : 0;
    uint32_t failures = 0;
    if (channel >= 64 || firmware_base != flare_controller.addr || virtual_write || queued_pages)
        failures |= BAD_ENV;
    FlareStatus initial = flare_controller.read_status();
    if (!failures) {
        flare_controller.flush_commands();
        usleep(1000);
        XTime start;
        XTime_GetTime(&start);
        do {
            initial = flare_controller.read_status();
            if (link_up(initial)) break;
            usleep(1000);
        } while (elapsed_us(start) < LINK_WAIT_US);
        if (!link_up(initial)) failures |= LINK_DOWN;
    }
    if (!failures) {
        xil_printf("Clearing %d old FIFO entries; counters remain cumulative\r\n",
                   (int)flare_controller.records_available());
        flare_controller.clear_records();
        flare_controller.clear_counters();
        flare_controller.clear_sticky();
        flare_controller.slot_collision_count = 0;
        usleep(1000);
        initial = flare_controller.read_status();
    }

    static Trial trials[14] = {};
    Shot shots[2] = {};
    FlareConfig cfg = {};
    unsigned attempted = 0, failed_phase = 0;
    FlareStatus previous = initial;
    for (unsigned phase = 1; phase <= 14 && !failures; ++phase) {
        Trial& trial = trials[phase - 1];
        trial.phase = phase;
        if (phase == 7 || phase == 11) {
            const unsigned index = phase == 7 ? 0 : 1;
            shots[index].id = shot_ids[index];
            xil_printf("ARM shot %u: waiting for expected SOF timeout (up to 3 seconds)\r\n",
                       (unsigned)shots[index].id);
            run_shot(trial, shots[index], cfg, previous);
            if (index == 1 && shots[1].result && shots[0].result_sequence == shots[1].result_sequence)
                trial.failures |= BAD_RESULT;
        } else {
            run_trial(trial, phase >= 4 ? &cfg : 0, previous);
        }
        ++attempted;
        failures |= trial.failures;
        if (!passed(trial)) { if (!failures) failures |= BAD_REPLY; failed_phase = phase; }
        previous = trial.after;
        if (phase == 3 && !failures) {
            const FlareRecord& rec = trial.records[0];
            cfg = make_config(next_token((uint32_t)rec.payload[0]), false);
            baseline_shots = (uint32_t)rec.payload[2];
            baseline_results = (uint32_t)(rec.payload[2] >> 32);
            shot_ids[0] = (uint32_t)rec.payload[1] + 1U;
            if (!shot_ids[0]) ++shot_ids[0];
            shot_ids[1] = shot_ids[0] + 1U;
            if (!shot_ids[1]) ++shot_ids[1];
            // Counter saturation cannot prove exactly one new result; stop first.
            if (baseline_shots > 0xfffffffdU || baseline_results > 0xfffffffdU) failures |= BAD_RECOVERY;
            if (failures) failed_phase = phase;
        }
    }
    auto_stop();
    const FlareStatus final_status = flare_controller.read_status();
    if (!failures && (local_error(final_status) || !link_up(final_status) ||
        final_status.record_level || final_status.ack_pending || final_status.result_pending ||
        final_status.command_tx_count != 20 || final_status.rx_message_count != 13 ||
        final_status.ack_tx_count != 2 || final_status.result_count != 2 || final_status.duplicate_count ||
        final_status.last_tx_sequence != previous.last_tx_sequence ||
        flare_controller.slot_collision_count != 6)) failures |= LOCAL_ERROR;
    check_publication(ds_env.append_list(5, (double)channel, (double)flare_controller.addr,
        (double)firmware_base, (double)virtual_write, (double)queued_pages));
    save_status(0, initial, 0);
    if (cfg.config_token32) save_config(1, cfg);
    for (unsigned i = 0; i < attempted; ++i) save_trial(trials[i]);
    unsigned shots_passed = 0;
    if (attempted >= 7) { save_shot(trials[6], shots[0], cfg.config_token32); shots_passed += passed(trials[6]); }
    if (attempted >= 11) { save_shot(trials[10], shots[1], cfg.config_token32); shots_passed += passed(trials[10]); }
    if (!publication_ok) failures |= HOST_OUTPUT;
    bool ok = !failures && shots_passed == 2 && attempted == 14;
    check_publication(ds_summary.append_list(12, (double)ok, (double)failures, (double)failed_phase,
        (double)cfg.config_token32, (double)shot_ids[0], (double)shot_ids[1], (double)shots_passed,
        (double)final_status.command_tx_count, (double)final_status.ack_tx_count,
        (double)final_status.rx_message_count, (double)final_status.result_count, (double)attempted));
    if (!publication_ok) { failures |= HOST_OUTPUT; ok = false; }
    xil_printf("FLARE_timeout_test %s: mask 0x%x failed phase %d shots %d/2 cmd_tx %d auto_ack %d rx %d\r\n",
        ok ? "PASS" : "FAIL", (unsigned)failures, (int)failed_phase, (int)shots_passed,
        (int)final_status.command_tx_count, (int)final_status.ack_tx_count, (int)final_status.rx_message_count);
    return ok ? 0 : 1;
}
