/*
 * FLARE_repeat_test.cpp -- repeated HEARTBEAT + STATUS pairs, without a camera.
 * Run as a normal lolenc experiment using lolenc_repo/configuration.json.
 * device_config.cpp supplies `flare_controller`; do not create another device.
 *
 * LOLENC execution:
 *   init() stops/resets the shared RTIO counter; it is not a PHY reset.
 *   delay() advances the SOFTWARE timeline only. Queue one command before
 *   auto_start(), then use the PS timer/usleep for the real response timeout.
 *   Re-arm the timeline for every command so it cannot have a past stamp.
 *   Return from main(): ELF_Runner performs the host KILL handshake.
 * This is a standalone experiment: it stops the global RTIO timeline, flushes
 * the local FLARE command/record FIFOs, and clears its counters/sticky flags.
 * It sends no camera, configuration, stream-mode, or manual RESULT_ACK command.
 *
 * GUI arguments: repeat_pairs=1000 (1..10000), pair_gap_ms=10 (0..1000).
 * One pair sends TWO requests. Stop at the first failing exchange/idle check.
 * Error counters are cleared only at startup, never between requests. The
 * 10000-pair limit keeps expected counts (20000) below 16-bit saturation.
 * This tests repeated application exchanges at a modest rate, not line-rate BER.
 *
 * Results: results/<date>/<run>/<rid>-FLARE_repeat_test.h5, datasets/rid<rid>_*.
 * Use fixed-width DatasetFloatList::append_list rows, as in flare_link_diag.
 * UART xil_printf output is supplementary; REPORT and Dataset::set are avoided.
 * All variadic dataset values are doubles; 64-bit payloads are split into two
 * uint32 halves before conversion so their bits survive HDF5 storage exactly.
 *
 * flare_repeat_summary (1 x 18):
 *   [pass, failure_mask, requested_pairs, completed_pairs,
 *    heartbeat_attempts, heartbeat_passes, status_attempts, status_passes,
 *    failed_pair, failed_phase, command_tx_count, rx_message_count,
 *    heartbeat_min_us, heartbeat_mean_us, heartbeat_max_us,
 *    status_min_us, status_mean_us, status_max_us]
 *   failed_pair=0 means startup/publication failure or no failure; otherwise 1-based.
 *   failed_phase: 0=startup/publication, 1=heartbeat, 2=status, 3=idle/gap.
 *   Timings summarize PASSED exchanges; all three are 0 if none passed.
 *   They include the 100 us reservation and CPU polling, not just link latency.
 * flare_repeat_progress (rows x 7; every 100 pairs, and final):
 *   [completed_pairs, heartbeat_passes, status_passes, command_tx_count,
 *    rx_message_count, failure_mask, elapsed_ms]
 * flare_repeat_trials (rows x 9; first/last pair and first failing trial):
 *   [pair, phase, pass, failure_mask, matching_records, first_reply_us,
 *    captured_records, command_tx_delta, rx_message_delta]
 * flare_repeat_status (rows x 23; initial and final):
 *   [pair, phase, channel_up, lane_up, tx_ready, sticky_mask, command_tx_count,
 *    rx_message_count, hard_error_count, soft_error_count, crc_error_count,
 *    header_error_count, phy_overflow_count, tx_drop_count, record_drop_count,
 *    tx_abort_count, record_level, last_tx_sequence, last_msg_type,
 *    last_msg_sequence, last_nack_reason, last_rx_error_reason, slot_collisions]
 *   sticky_mask uses the hardware STATUS1 bit positions (5 through 13).
 * flare_repeat_records (rows x 19; saved trials only):
 *   [pair, phase, type, flags, payload_words, entry_count, sequence,
 *    W0_lo32, W0_hi32, ... , W5_lo32, W5_hi32]
 * flare_repeat_env (1 x 5):
 *   [device_channel, expected_base, firmware_base, virtual_write, queued_pages]
 * flare_repeat_settings (1 x 3): [repeat_pairs, pair_gap_ms, checkpoint_pairs]
 *
 * failure_mask bits (also printed in UART):
 *   0x01 firmware address map / virtual queue mismatch
 *   0x02 link unavailable or dropped during the test
 *   0x04 response timeout
 *   0x08 malformed PS record (stop reading: FIFO alignment is uncertain)
 *   0x10 wrong response shape/flags/layout/sequence, or duplicate response
 *   0x20 unexpected record or capture limit exceeded
 *   0x40 local error/sticky flag, or TX/RX count did not match the trial
 *   0x80 dataset publication returned an error (check host/UART logs)
 *   0x100 argument outside its supported range
 *
 * PASS establishes the requested number of protocol exchanges. Camera/configuration
 * readiness and peer health flags are reported, not required to be healthy.
 * HEARTBEAT is telemetry, not the ZCU102 PS watchdog kick. These two responses
 * echo the request's header sequence. Current RTL exposes its NEXT sequence as
 * the BSP's `last_tx_sequence`, so the pre-send snapshot is the expected echo.
 * The 500 ms timeout bounds the exchange only: the existing BSP dataset IPC
 * waits for the host without a timeout, so a disconnected host can block saving.
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xtime_l.h"

extern FLAREController flare_controller;

// Keep the literal ArgInt declarations: lolenc's GUI discovers them by regex.
ArgInt repeat_pairs(1000, 1, 10000, 1, "pairs");
ArgInt pair_gap_ms(10, 0, 1000, 1, "ms");

static DatasetFloatList ds_summary("flare_repeat_summary", true);
static DatasetFloatList ds_trials("flare_repeat_trials", true);
static DatasetFloatList ds_status("flare_repeat_status", true);
static DatasetFloatList ds_records("flare_repeat_records", true);
static DatasetFloatList ds_env("flare_repeat_env", true);
static DatasetFloatList ds_progress("flare_repeat_progress", true);
static DatasetFloatList ds_settings("flare_repeat_settings", true);

static const uint32_t LINK_WAIT_US = 2000000;
static const uint32_t REPLY_TIMEOUT_US = 500000;
static const uint32_t QUIET_US = 2000;
static const unsigned MAX_RECORDS = 16;
static const unsigned CHECKPOINT_PAIRS = 100;

enum Failure {
    BAD_ENV = 0x01, LINK_DOWN = 0x02, TIMEOUT = 0x04, BAD_RECORD = 0x08,
    BAD_REPLY = 0x10, EXTRA_RECORD = 0x20, LOCAL_ERROR = 0x40,
    HOST_OUTPUT = 0x80, BAD_ARGUMENT = 0x100
};

static bool publication_ok = true;

static void check_publication(err_t result) {
    if (result != ERR_OKAY) {
        publication_ok = false;
        xil_printf("FLARE_repeat_test: dataset publication failed; check host logs\r\n");
    }
}

struct Trial {
    unsigned phase;
    uint32_t failures;
    unsigned matches;
    uint32_t reply_us;
    unsigned count;
    FlareRecord records[MAX_RECORDS];
    FlareStatus before;
    FlareStatus after;
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

static void save_status(unsigned pair, unsigned phase, const FlareStatus& st) {
    check_publication(ds_status.append_list(23,
        (double)pair, (double)phase, (double)st.channel_up, (double)st.lane_up,
        (double)st.tx_ready, (double)sticky_mask(st),
        (double)st.command_tx_count, (double)st.rx_message_count,
        (double)st.hard_error_count, (double)st.soft_error_count,
        (double)st.crc_error_count, (double)st.header_error_count,
        (double)st.phy_overflow_count, (double)st.tx_drop_count,
        (double)st.record_drop_count, (double)st.tx_abort_count,
        (double)st.record_level, (double)st.last_tx_sequence,
        (double)st.last_msg_type, (double)st.last_msg_sequence,
        (double)st.last_nack_reason, (double)st.last_rx_error_reason,
        (double)flare_controller.slot_collision_count));
    xil_printf("FLARE phase %d: link %d/%d tx %d rx %d sticky 0x%x "
               "hard %d soft %d crc %d header %d txdrop %d recdrop %d\r\n",
               (int)phase, (int)st.channel_up, (int)st.lane_up,
               (int)st.command_tx_count, (int)st.rx_message_count,
               (unsigned)sticky_mask(st), (int)st.hard_error_count,
               (int)st.soft_error_count, (int)st.crc_error_count,
               (int)st.header_error_count, (int)st.tx_drop_count,
               (int)st.record_drop_count);
}

static void save_record(unsigned pair, unsigned phase, const FlareRecord& rec) {
    double words[12];
    for (unsigned i = 0; i < 6; ++i) {
        words[2*i] = (double)(uint32_t)rec.payload[i];
        words[2*i+1] = (double)(uint32_t)(rec.payload[i] >> 32);
    }
    check_publication(ds_records.append_list(19,
        (double)pair, (double)phase, (double)rec.record_type, (double)rec.record_flags,
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
    if (rec.record_type == FLARE_MSG_HEARTBEAT_RESPONSE) {
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

static void run_trial(Trial& trial) {
    const bool heartbeat = trial.phase == 1;
    const uint8_t wanted = heartbeat ? FLARE_MSG_HEARTBEAT_RESPONSE :
                                       FLARE_MSG_STATUS_RESPONSE;
    init();
    flare_controller.reset_slot_tracking();
    trial.before = flare_controller.read_status();
    if (!link_up(trial.before)) {
        trial.failures |= LINK_DOWN;
        trial.after = trial.before;
        return;
    }

    // Only one command is queued in this timeline. No host IPC while waiting.
    delay(100 * us);
    if (heartbeat) flare_controller.heartbeat();
    else           flare_controller.request_status();
    XTime start;
    XTime_GetTime(&start);
    auto_start();

    for (;;) {
        const uint32_t waited = elapsed_us(start);
        // Bound the loop even if the peer continuously sends unwanted packets.
        if (waited >= REPLY_TIMEOUT_US) {
            if (trial.matches == 0) trial.failures |= TIMEOUT;
            break;
        }
        FlareStatus live = flare_controller.read_status();
        if (!link_up(live)) {
            trial.failures |= LINK_DOWN;
            break;
        }
        if (flare_controller.records_available() != 0) {
            if (trial.count == MAX_RECORDS) {
                trial.failures |= EXTRA_RECORD;
                break;
            }
            FlareRecord& rec = trial.records[trial.count];
            const flare_err_t err = flare_controller.read_record(&rec);
            if (err != FLARE_OK) {
                trial.failures |= BAD_RECORD;
                break;
            }
            ++trial.count;
            if (rec.record_type != wanted) {
                trial.failures |= EXTRA_RECORD;
            } else {
                ++trial.matches;
                if (trial.matches == 1) trial.reply_us = elapsed_us(start);
                const unsigned layout = heartbeat ? (unsigned)(rec.payload[0] >> 56) :
                    (unsigned)((rec.payload[0] >> 32) & 0xff);
                if (rec.record_flags != 0 || layout != 2 || trial.matches != 1 ||
                    rec.payload_words != (heartbeat ? 1 : 6) ||
                    rec.entry_count != (heartbeat ? 2 : 4) ||
                    rec.sequence != trial.before.last_tx_sequence) {
                    trial.failures |= BAD_REPLY;
                }
            }
        } else if (trial.matches != 0 && waited - trial.reply_us >= QUIET_US) {
            break;
        }
        usleep(100);
    }
    auto_stop();
    trial.after = flare_controller.read_status();
    if (!link_up(trial.after)) trial.failures |= LINK_DOWN;
    if (local_error(trial.after) ||
        (int)trial.after.command_tx_count - (int)trial.before.command_tx_count != 1 ||
        (int)trial.after.rx_message_count - (int)trial.before.rx_message_count != 1 ||
        trial.after.ack_tx_count != trial.before.ack_tx_count ||
        trial.after.last_tx_sequence != (uint32_t)(trial.before.last_tx_sequence + 1U) ||
        flare_controller.slot_collision_count != 0) {
        trial.failures |= LOCAL_ERROR;
    }
}

static bool passed(const Trial& trial) {
    return trial.matches == 1 && trial.failures == 0;
}

static void save_trial(unsigned pair, const Trial& trial) {
    check_publication(ds_trials.append_list(9,
        (double)pair, (double)trial.phase, (double)passed(trial), (double)trial.failures,
        (double)trial.matches, (double)trial.reply_us, (double)trial.count,
        (double)((int)trial.after.command_tx_count - (int)trial.before.command_tx_count),
        (double)((int)trial.after.rx_message_count - (int)trial.before.rx_message_count)));
    for (unsigned i = 0; i < trial.count; ++i) save_record(pair, trial.phase, trial.records[i]);
    xil_printf("FLARE pair %d %s: %s mask 0x%x first_reply_us %d\r\n",
               (int)pair, trial.phase == 1 ? "HEARTBEAT" : "STATUS",
               passed(trial) ? "PASS" : "FAIL", (unsigned)trial.failures,
               (int)trial.reply_us);
}

struct Totals {
    unsigned attempts = 0;
    unsigned successes = 0;
    uint32_t minimum_us = 0;
    uint32_t maximum_us = 0;
    uint64_t sum_us = 0;

    void add(const Trial& trial) {
        ++attempts;
        if (!passed(trial)) return;
        if (successes == 0 || trial.reply_us < minimum_us) minimum_us = trial.reply_us;
        if (trial.reply_us > maximum_us) maximum_us = trial.reply_us;
        sum_us += trial.reply_us;
        ++successes;
    }
    double mean_us() const {
        return successes ? (double)sum_us / (double)successes : 0.0;
    }
};

// Check quiet intervals too: a delayed extra reply or link drop must not be
// silently absorbed into the next trial's baseline. Counters are never cleared
// here. Shorter transients than the polling interval require hardware counters.
static uint32_t check_idle(const FlareStatus& previous, FlareStatus& current) {
    current = flare_controller.read_status();
    uint32_t failures = 0;
    if (!link_up(current)) failures |= LINK_DOWN;
    if (local_error(current)) failures |= LOCAL_ERROR;
    if (current.command_tx_count != previous.command_tx_count ||
        current.rx_message_count != previous.rx_message_count ||
        current.ack_tx_count != previous.ack_tx_count ||
        current.last_tx_sequence != previous.last_tx_sequence ||
        current.record_level != 0) failures |= EXTRA_RECORD;
    return failures;
}

static uint32_t wait_gap(uint32_t gap_us, const FlareStatus& previous,
                         FlareStatus& current) {
    XTime start;
    XTime_GetTime(&start);
    for (;;) {
        const uint32_t failures = check_idle(previous, current);
        if (failures) return failures;
        const uint32_t waited = elapsed_us(start);
        if (waited >= gap_us) return 0;
        const uint32_t remaining = gap_us - waited;
        usleep(remaining < 1000 ? remaining : 1000);
    }
}

static void save_progress(unsigned completed, const Totals& heartbeat,
                          const Totals& status, const FlareStatus& st,
                          uint32_t failures, XTime start) {
    XTime now;
    XTime_GetTime(&now);
    // Use 64 bits: the largest allowed run can exceed 2^32 microseconds.
    const uint64_t elapsed_ms = ((now - start) * 1000ULL) / COUNTS_PER_SECOND;
    check_publication(ds_progress.append_list(7,
        (double)completed, (double)heartbeat.successes, (double)status.successes,
        (double)st.command_tx_count, (double)st.rx_message_count,
        (double)failures, (double)elapsed_ms));
    xil_printf("FLARE repeat: completed %d pairs, HB %d STATUS %d, tx %d rx %d, mask 0x%x\r\n",
               (int)completed, (int)heartbeat.successes, (int)status.successes,
               (int)st.command_tx_count, (int)st.rx_message_count, (unsigned)failures);
}

int main() {
    init();
    const int64_t requested = repeat_pairs.get_value();
    const int64_t gap_ms = pair_gap_ms.get_value();
    uint32_t failures = 0;
    if (requested < 1 || requested > 10000 || gap_ms < 0 || gap_ms > 1000) {
        failures |= BAD_ARGUMENT;
    }
    xil_printf("FLARE_repeat_test: %d HEARTBEAT/STATUS pairs, gap %d ms\r\n",
               (int)requested, (int)gap_ms);
    check_publication(ds_settings.append_list(3, (double)requested,
                      (double)gap_ms, (double)CHECKPOINT_PAIRS));

    const uint64_t channel = flare_controller.virtual_table_channel;
    const uint64_t firmware_base = channel < 64 ? virtual_table->module_addr[channel] : 0;
    const uint64_t virtual_write = channel < 64 ? virtual_table->virtual_write[channel] : 0;
    const uint64_t queued_pages = channel < 64 ? virtual_table->table_size[channel] : 0;
    check_publication(ds_env.append_list(5, (double)channel, (double)flare_controller.addr,
                      (double)firmware_base, (double)virtual_write, (double)queued_pages));
    if (channel >= 64 || firmware_base != flare_controller.addr || virtual_write || queued_pages) {
        failures |= BAD_ENV;
    }
    if (!publication_ok) failures |= HOST_OUTPUT;

    FlareStatus initial = flare_controller.read_status();
    if (failures == 0) {
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
    if (failures == 0) {
        xil_printf("Clearing %d old FIFO entries; counters stay cumulative for this run\r\n",
                   (int)flare_controller.records_available());
        flare_controller.clear_records();
        flare_controller.clear_counters();
        flare_controller.clear_sticky();
        usleep(1000);
        initial = flare_controller.read_status();
        if (local_error(initial) || initial.command_tx_count || initial.rx_message_count ||
            initial.ack_tx_count || initial.record_level) failures |= LOCAL_ERROR;
        if (!link_up(initial)) failures |= LINK_DOWN;
    }
    save_status(0, 0, initial);
    if (!publication_ok) failures |= HOST_OUTPUT;

    Totals hb_totals;
    Totals status_totals;
    Trial heartbeat = {};
    Trial status = {};
    unsigned completed = 0;
    unsigned failed_pair = 0;
    unsigned failed_phase = 0;
    unsigned last_trial_pair = 0;
    unsigned saved_trial_pair = 0;
    bool status_attempted = false;
    FlareStatus last = initial;
    XTime run_start;
    XTime_GetTime(&run_start);

    for (unsigned pair = 1; failures == 0 && pair <= (unsigned)requested; ++pair) {
        FlareStatus current;
        failures |= wait_gap(pair == 1 ? 0 : (uint32_t)gap_ms * 1000U, last, current);
        last = current;
        if (failures) { failed_pair = pair; failed_phase = 3; break; }

        heartbeat = {};
        status = {};
        heartbeat.phase = 1;
        status.phase = 2;
        last_trial_pair = pair;
        status_attempted = false;
        run_trial(heartbeat);
        hb_totals.add(heartbeat);
        last = heartbeat.after;
        failures |= heartbeat.failures;
        if (failures) { failed_pair = pair; failed_phase = 1; break; }

        failures |= check_idle(last, current);
        last = current;
        if (failures) { failed_pair = pair; failed_phase = 3; break; }
        run_trial(status);
        status_attempted = true;
        status_totals.add(status);
        last = status.after;
        failures |= status.failures;
        if (failures) { failed_pair = pair; failed_phase = 2; break; }
        ++completed;

        // Retain packet evidence from the first pair and the final attempted pair.
        // Dataset IPC happens only after auto_stop(), with no request outstanding.
        if (pair == 1) {
            save_trial(pair, heartbeat);
            save_trial(pair, status);
            saved_trial_pair = pair;
        }
        if (completed % CHECKPOINT_PAIRS == 0 && completed < (unsigned)requested) {
            save_progress(completed, hb_totals, status_totals, last, failures, run_start);
        }
        if (!publication_ok) { failures |= HOST_OUTPUT; break; }
    }
    auto_stop();
    if (failures == 0) {
        FlareStatus current;
        failures |= check_idle(last, current);
        last = current;
        if (failures) { failed_pair = completed; failed_phase = 3; }
        if (last.command_tx_count != completed * 2U ||
            last.rx_message_count != completed * 2U) failures |= LOCAL_ERROR;
    }
    if (last_trial_pair != 0 && last_trial_pair != saved_trial_pair) {
        save_trial(last_trial_pair, heartbeat);
        if (status_attempted) save_trial(last_trial_pair, status);
    }
    save_status(failed_pair ? failed_pair : completed, failed_phase, last);
    if (!publication_ok) failures |= HOST_OUTPUT;
    save_progress(completed, hb_totals, status_totals, last, failures, run_start);
    if (!publication_ok) failures |= HOST_OUTPUT;

    bool ok = failures == 0 && completed == (unsigned)requested &&
              hb_totals.successes == (unsigned)requested &&
              status_totals.successes == (unsigned)requested;
    check_publication(ds_summary.append_list(18,
        (double)ok, (double)failures, (double)requested, (double)completed,
        (double)hb_totals.attempts, (double)hb_totals.successes,
        (double)status_totals.attempts, (double)status_totals.successes,
        (double)failed_pair, (double)failed_phase,
        (double)last.command_tx_count, (double)last.rx_message_count,
        (double)hb_totals.minimum_us, hb_totals.mean_us(), (double)hb_totals.maximum_us,
        (double)status_totals.minimum_us, status_totals.mean_us(), (double)status_totals.maximum_us));
    if (!publication_ok) { failures |= HOST_OUTPUT; ok = false; }
    xil_printf("FLARE_repeat_test %s: %d/%d pairs, mask 0x%x, failed pair %d phase %d\r\n",
               ok ? "PASS" : "FAIL", (int)completed, (int)requested,
               (unsigned)failures, (int)failed_pair, (int)failed_phase);
    return ok ? 0 : 1; // ELF_Runner performs termination after main returns.
}
