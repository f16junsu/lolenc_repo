/*
 * Minimal external line-trigger test for the current RFSoC bitstream.
 * Input: ja_8 = InputController_7 (J4-connected EEM2TTL, channel 8).
 * Use a compatible logic-level trigger output, with the DIO channel set to I.
 * This test queues no TTL/RF/DAC outputs; it checks TimeController status only.
 *
 * Each shot arms once and waits for a NEW rising edge. A static HIGH is not
 * a trigger. At 60 Hz, the next edge normally arrives within about 16.7 ms.
 * No edge within 250 ms -> TIMEOUT; stop on the first failed shot.
 * Run again with the trigger source disabled to check the TIMEOUT path.
 *
 * Datasets (RID-prefixed by the existing host):
 *   line_trigger_shots:   [shot, result, wait_us, saw_waiting, raw_status]
 *   line_trigger_summary: [all_passed, passed, attempted, requested]
 * result: 1 = triggered, 0 = timeout, -1 = bad mode/status, -2 = stop failed.
 * wait_us is PS polling time from arming, NOT FPGA trigger latency or period.
 * Bit 0 of raw_status is DRAM calibration; the relevant mask is 0xE:
 *   0x2 stopped in line mode, 0x6 waiting, 0xA running.
 *
 * All host reporting happens after stopping and restoring TRIGGER_ALWAYS.
 * Dataset IPC itself can block if the existing host connection is lost.
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xtime_l.h"
#include "xil_printf.h"

extern TimeController tc_0;

static const unsigned SHOTS = 10;
static const uint32_t TIMEOUT_US = 250000;
static const uint32_t POLL_US = 20;
static const unsigned STATUS_MASK = 0xE;

struct ShotResult {
    int result;
    uint32_t wait_us;
    bool saw_waiting;
    unsigned status;
};

static uint32_t elapsed_us(XTime start) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(((now - start) * 1000000ULL) / COUNTS_PER_SECOND);
}

int main() {
    ShotResult records[SHOTS] = {};
    unsigned attempted = 0;
    unsigned passed = 0;

    init();
    set_trigger_mode(TRIGGER_LINE);
    usleep(10); // Allow the mode to cross into the RTIO clock domain.

    for (unsigned shot = 0; shot < SHOTS; ++shot) {
        // Reset the timestamp and disarm before accepting the next edge.
        init();
        usleep(10);
        ShotResult &record = records[attempted++];
        record.status = (unsigned)tc_0.read_status();
        record.result = -1;

        if ((record.status & STATUS_MASK) == 0x2) {
            XTime armed_at;
            XTime_GetTime(&armed_at);
            auto_start();
            for (;;) {
                record.status = (unsigned)tc_0.read_status();
                record.wait_us = elapsed_us(armed_at);
                const unsigned state = record.status & STATUS_MASK;
                if (state == 0xA) {
                    record.result = 1;
                    break;
                }
                if (state == 0x6) record.saw_waiting = true;
                // 0x2 is also allowed briefly while the arm write crosses clocks.
                if (state != 0x2 && state != 0x6) break;
                if (record.wait_us >= TIMEOUT_US) {
                    record.result = 0;
                    break;
                }
                usleep(POLL_US);
            }
        }

        auto_stop();
        usleep(10);
        if ((tc_0.read_status() & STATUS_MASK) != 0x2) record.result = -2;
        if (record.result != 1) break;
        ++passed;
        // Brief stopped interval, then rearm for another edge.
        usleep(1000);
    }

    init();
    set_trigger_mode(TRIGGER_ALWAYS);
    usleep(10);

    DatasetFloatList shots_dataset("line_trigger_shots", true);
    DatasetFloatList summary_dataset("line_trigger_summary", true);
    bool published = true;
    for (unsigned shot = 0; shot < attempted; ++shot) {
        const ShotResult &record = records[shot];
        const char *label = record.result == 1 ? "TRIGGERED" :
                            record.result == 0 ? "TIMEOUT" : "ERROR";
        xil_printf("[line trigger] shot %d: %s result=%d wait_us=%d waiting=%d status=0x%x\r\n",
                   (int)(shot + 1), label, record.result, (int)record.wait_us,
                   (int)record.saw_waiting, record.status);
        if (shots_dataset.append_list(5, (double)(shot + 1), (double)record.result,
                (double)record.wait_us, (double)record.saw_waiting,
                (double)record.status) != ERR_OKAY) published = false;
    }
    const bool all_passed = passed == SHOTS;
    if (summary_dataset.append_list(4, (double)all_passed, (double)passed,
            (double)attempted, (double)SHOTS) != ERR_OKAY) published = false;
    xil_printf("[line trigger] %s: %d/%d shots triggered; restored ALWAYS mode.\r\n",
               all_passed ? "PASS" : "FAIL", (int)passed, (int)SHOTS);
    if (!published) xil_printf("[line trigger] ERROR: dataset publication failed.\r\n");
    return all_passed && published ? 0 : 1;
}
