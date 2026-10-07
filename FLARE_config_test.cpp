/*
 * FLARE_config_test.cpp -- apply configurations A/B without triggering a camera.
 * Run with the existing lolenc_repo/configuration.json and device_config.cpp.
 * This stops the global RTIO timeline and FLARE storage stream, clears local
 * FIFOs/counters once, and REPLACES the peer's active measurement configuration.
 * On success configuration B remains active; the stream remains stopped.
 * STREAM_STOP does not stop the camera itself. Run with no camera acquisition
 * or other command producer. No ARM_TRIGGER, TTL, STREAM_START, or PHY reset.
 *
 * Phases: 1 initial STATUS, 2 STREAM_STOP (no response), 3 idle STATUS,
 *         4 configure A, 5 STATUS A, 6 HEARTBEAT A,
 *         7 configure B, 8 STATUS B, 9 HEARTBEAT B.
 * Stop on the first failure. Phase 0 denotes startup/publication failure.
 * STATUS requires idle/no pending result/no fatal error. Phase 1 allows a
 * camera stream; phase 3 and later require it to be inactive after STREAM_STOP.
 * READY is reported, not required: it includes further measurement conditions.
 * Tokens are chosen after reading the peer, differ from its current token,
 * and are nonzero. A/B each configure one ion (7 commands per transaction).
 * CONFIG_STATUS must echo COMMIT's sequence in both header and payload,
 * report success/config_id=0/flags=0/version=token[15:0]. Subsequent telemetry
 * must show config_valid and the full token. This is not ROI/threshold readback.
 *
 * init() resets/stops RTIO; delay() changes only the software timestamp.
 * Queue at 100 us before auto_start(); timeout uses XTime/usleep wall time.
 * configure() uses next_slot(): 6 software slot adjustments per 7-command
 * transaction are EXPECTED. sticky_timestamp_error remains an actual failure.
 * Dataset IPC occurs only after all time-sensitive exchanges, with RTIO stopped.
 * Return from main(): ELF_Runner owns the host KILL handshake.
 *
 * HDF5 datasets (all values double; each payload word split lo32,hi32):
 * flare_config_summary [pass,mask,failed_phase,A_verified,B_verified,
 *   initial_token,A_token,B_token,tx,rx,attempted_phases,slot_adjustments]
 * flare_config_trials [phase,pass,mask,matches,first_reply_us,records,
 *   tx_delta,rx_delta,expected_tx,expected_rx,slot_adjustment_delta,queue_error]
 * flare_config_settings [A_or_B,token,ion_count,background_enable,scale_q20,
 *   signal_w,signal_h,background_w,background_h,dx,dy,margin,x0,y0,threshold]
 * flare_config_status [phase,channel_up,lane_up,tx_ready,sticky_mask,tx,rx,
 *   hard,soft,crc,header,phy_overflow,tx_drop,record_drop,tx_abort,record_level,
 *   next_tx_sequence,last_msg_type,last_msg_sequence,last_nack,last_rx_error,
 *   slot_adjustments]
 * flare_config_records [phase,type,flags,words,entries,sequence,W0lo,W0hi,...W5hi]
 * flare_config_env [channel,expected_base,firmware_base,virtual_write,queued_pages]
 * Normal run: 9 phases, TX=21, RX=8, slot_adjustments=12, mask=0.
 * first_reply_us includes the 100 us reservation and PS polling, not link latency.
 * Dataset IPC has no timeout in the existing BSP; a disconnected host can block.
 *
 * mask: 01 map/virtual queue, 02 link, 04 timeout, 08 malformed record,
 * 10 response shape/sequence/flags/duplicate, 20 unexpected extra record,
 * 40 local error/counter/slot mismatch, 80 dataset publication,
 * 100 local configure() rejection, 200 CONFIG_STATUS rejection/version mismatch,
 * 400 telemetry token/config_valid mismatch, 800 peer busy/fatal/not idle.
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xtime_l.h"

extern FLAREController flare_controller;
static DatasetFloatList ds_summary("flare_config_summary", true);
static DatasetFloatList ds_trials("flare_config_trials", true);
static DatasetFloatList ds_status("flare_config_status", true);
static DatasetFloatList ds_records("flare_config_records", true);
static DatasetFloatList ds_env("flare_config_env", true);
static DatasetFloatList ds_settings("flare_config_settings", true);
static const uint32_t LINK_WAIT_US = 2000000;
static const uint32_t REPLY_TIMEOUT_US = 500000;
static const uint32_t QUIET_US = 2000;
static const uint32_t STOP_SETTLE_US = 10000;
static const unsigned MAX_RECORDS = 16;
enum Failure {
    BAD_ENV=0x01, LINK_DOWN=0x02, TIMEOUT=0x04, BAD_RECORD=0x08,
    BAD_REPLY=0x10, EXTRA_RECORD=0x20, LOCAL_ERROR=0x40, HOST_OUTPUT=0x80,
    LOCAL_CONFIG=0x100, CONFIG_REJECTED=0x200, TOKEN_MISMATCH=0x400,
    PEER_NOT_IDLE=0x800
};
static bool publication_ok = true;
static void check_publication(err_t result) {
    if (result != ERR_OKAY) {
        publication_ok = false;
        xil_printf("FLARE_config_test: dataset publication failed\r\n");
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
    if (rec.record_type == FLARE_MSG_CONFIG_STATUS && rec.payload_words == 2) {
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


static bool passed(const Trial& trial) {
    return trial.failures == 0 && trial.matches == trial.expected_rx;
}

static uint8_t wanted_type(unsigned phase) {
    if (phase == 2) return 0;
    if (phase == 4 || phase == 7) return FLARE_MSG_CONFIG_STATUS;
    if (phase == 6 || phase == 9) return FLARE_MSG_HEARTBEAT_RESPONSE;
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
        trial.before.last_tx_sequence != previous.last_tx_sequence) trial.failures |= EXTRA_RECORD;
    if (trial.failures) return;

    delay(100 * us);
    if (!wanted) flare_controller.stream_stop();
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
    xil_printf("FLARE config phase %d: %s mask 0x%x slots +%d local_config_error %d\r\n",
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

int main() {
    init();
    xil_printf("FLARE_config_test: apply A/B, no camera trigger; B remains active\r\n");
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
    static Trial trials[9] = {};
    FlareConfig config_a = {}, config_b = {};
    uint32_t initial_token = 0;
    unsigned attempted = 0, failed_phase = 0;
    FlareStatus previous = initial;
    for (unsigned phase = 1; phase <= 9 && !failures; ++phase) {
        Trial& trial = trials[phase - 1];
        trial.phase = phase;
        const FlareConfig* cfg = phase >= 7 ? &config_b : phase >= 4 ? &config_a : 0;
        run_trial(trial, cfg, previous);
        ++attempted;
        failures |= trial.failures;
        if (!passed(trial)) { if (!failures) failures |= BAD_REPLY; failed_phase = phase; }
        previous = trial.after;
        if (phase == 1 && !failures) {
            initial_token = (uint32_t)trial.records[0].payload[0];
            config_a = make_config(next_token(initial_token), false);
            config_b = make_config(next_token(config_a.config_token32), true);
            // Avoid B reusing the peer's original token on uint32 wrap.
            if (config_b.config_token32 == initial_token)
                config_b.config_token32 = next_token(config_b.config_token32);
        }
    }
    auto_stop();
    const FlareStatus final_status = flare_controller.read_status();
    if (!failures && (local_error(final_status) || !link_up(final_status) ||
        final_status.record_level || final_status.command_tx_count != 21 ||
        final_status.rx_message_count != 8 || final_status.ack_tx_count != initial.ack_tx_count ||
        final_status.last_tx_sequence != previous.last_tx_sequence ||
        flare_controller.slot_collision_count != 12)) failures |= LOCAL_ERROR;

    check_publication(ds_env.append_list(5, (double)channel, (double)flare_controller.addr,
        (double)firmware_base, (double)virtual_write, (double)queued_pages));
    save_status(0, initial, 0);
    if (config_a.config_token32) { save_config(1, config_a); save_config(2, config_b); }
    for (unsigned i = 0; i < attempted; ++i) save_trial(trials[i]);
    if (!publication_ok) failures |= HOST_OUTPUT;
    const bool a_ok = attempted >= 6 && passed(trials[3]) && passed(trials[4]) && passed(trials[5]);
    const bool b_ok = attempted >= 9 && passed(trials[6]) && passed(trials[7]) && passed(trials[8]);
    bool ok = failures == 0 && a_ok && b_ok;
    check_publication(ds_summary.append_list(12, (double)ok, (double)failures, (double)failed_phase,
        (double)a_ok, (double)b_ok, (double)initial_token,
        (double)config_a.config_token32, (double)config_b.config_token32,
        (double)final_status.command_tx_count, (double)final_status.rx_message_count,
        (double)attempted, (double)flare_controller.slot_collision_count));
    if (!publication_ok) { failures |= HOST_OUTPUT; ok = false; }
    xil_printf("FLARE_config_test %s: mask 0x%x failed phase %d A 0x%x B 0x%x tx %d rx %d\r\n",
        ok ? "PASS" : "FAIL", (unsigned)failures, (int)failed_phase,
        (unsigned)config_a.config_token32, (unsigned)config_b.config_token32,
        (int)final_status.command_tx_count, (int)final_status.rx_message_count);
    return ok ? 0 : 1;
}
