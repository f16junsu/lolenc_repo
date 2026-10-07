/*
 * flare_link_diag.cpp -- FLARE Aurora link diagnosis on the ZCU111 (2026-10-01).
 *
 * Read-only: no RTIO command is scheduled, nothing is transmitted on the link,
 * no module other than the FLARE cell is touched, and the PHY is not reset.
 *
 * HOW THE RESULTS LEAVE THE BOARD. The same way every experiment in this
 * repository sends data: DatasetFloatList::append_list, one row per call. The
 * lolenc master stores each call as one row, so every row of a dataset must
 * have the same number of values or the result file cannot be written. Do not
 * use report_int_value / report_int_array_value or Dataset*::set here: the
 * REPORT header TCP_Server builds has no '#' between name and length and the
 * master's parser raises on it, and the master's "set" branch raises too.
 *
 * Datasets, in results/<date>/<n>/<rid>-flare_link_diag.h5 under "datasets":
 *
 *   rid<rid>_flare_status      3 rows x 13
 *       [n, channel_up, lane_up, tx_ready, hard_err_cnt, soft_err_cnt,
 *        crc_err_cnt, header_err_cnt, rx_msg_cnt, phy_overflow_cnt,
 *        sticky_hard, sticky_soft, last_rx_error_reason]
 *       n = 0 just after the counters were cleared, n = 1 and 2 one second
 *       apart. The error counters count episodes and saturate at 65535.
 *
 *   rid<rid>_flare_sfp_scan    rows x 3
 *       [switch_addr, channel, byte0] for every I2C switch channel on which
 *       something answers at 0x50; the last row is [-1, -1, number_of_hits].
 *       byte0 == 3 is an SFP/SFP+/SFP28 module.
 *
 *   rid<rid>_flare_sfp         rows x 19, [kind, switch_addr, channel, ...]
 *       kind 1: identifier, nominal_bit_rate_x100MBd (A0h byte 12),
 *               wavelength_nm (bytes 60-61), ddm_type (byte 92),
 *               eth_10g_compliance (byte 3), then zeros
 *       kind 2: 16 ASCII codes, vendor name        (A0h bytes 20-35)
 *       kind 3: 16 ASCII codes, vendor part number (A0h bytes 40-55)
 *       kind 4: temp_raw (1/256 degC), vcc_raw (100 uV), txbias_raw (2 uA),
 *               txpower_raw (0.1 uW), rxpower_raw (0.1 uW), status byte 110
 *               (bit7 TX_DISABLE state, bit2 TX_FAULT, bit1 RX_LOS), zeros
 *       kind 9: a read of that module failed; next value is the step (1 = A0h,
 *               2 = A2h diagnostics)
 *
 *   rid<rid>_flare_diag_note   rows x 2, [code, detail]
 *       [0, 0]   the experiment reached its end
 *       [1, 0]   PS I2C1 could not be initialised
 *       [10, a]  the I2C switch at address a did not answer
 *
 * main() returns instead of calling end_experiment(): ELF_Runner sends the
 * KILL request itself once the user code has returned.
 */
#include "core.h"
#include "module.h"
#include "dataset.h"
#include "sleep.h"
#include "xiicps.h"
#include "xparameters.h"

extern FLAREController flare_controller;   // device_config.cpp

DatasetFloatList ds_status("flare_status", true);
DatasetFloatList ds_scan("flare_sfp_scan", true);
DatasetFloatList ds_sfp("flare_sfp", true);
DatasetFloatList ds_note("flare_diag_note", true);

static XIicPs iic;

/* append_list is variadic and reads every value as a double. */
static void put13(DatasetFloatList &d, const double *v) {
    d.append_list(13, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9],
                  v[10], v[11], v[12]);
}

static void put19(DatasetFloatList &d, const double *v) {
    d.append_list(19, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9],
                  v[10], v[11], v[12], v[13], v[14], v[15], v[16], v[17], v[18]);
}

static void put3(DatasetFloatList &d, double a, double b, double c) {
    d.append_list(3, a, b, c);
}

static void put2(DatasetFloatList &d, double a, double b) {
    d.append_list(2, a, b);
}

/* ------------------------------------------------------------------------ */
/* FLARE status                                                             */
/* ------------------------------------------------------------------------ */
static void report_flare(int n) {
    FlareStatus st = flare_controller.read_status();
    double v[13];
    v[0]  = (double)n;
    v[1]  = (double)st.channel_up;
    v[2]  = (double)st.lane_up;
    v[3]  = (double)st.tx_ready;
    v[4]  = (double)st.hard_error_count;
    v[5]  = (double)st.soft_error_count;
    v[6]  = (double)st.crc_error_count;
    v[7]  = (double)st.header_error_count;
    v[8]  = (double)st.rx_message_count;
    v[9]  = (double)st.phy_overflow_count;
    v[10] = (double)st.sticky_hard_error;
    v[11] = (double)st.sticky_soft_error;
    v[12] = (double)st.last_rx_error_reason;
    put13(ds_status, v);
    xil_printf("FLARE %d: channel_up %d lane_up %d hard %d soft %d\r\n", n,
               (int)st.channel_up, (int)st.lane_up,
               (int)st.hard_error_count, (int)st.soft_error_count);
}

/* ------------------------------------------------------------------------ */
/* PS I2C1, polled, the same calls TCP_Server uses for the clock chips      */
/* ------------------------------------------------------------------------ */
static bool bus_idle() {
    for (int i = 0; i < 5000; i++) {
        if (!XIicPs_BusIsBusy(&iic)) return true;
        usleep(10);
    }
    return false;
}

static bool i2c_send(u8 addr, u8 *buf, int n) {
    if (!bus_idle()) return false;
    if (XIicPs_MasterSendPolled(&iic, buf, n, addr) != XST_SUCCESS) return false;
    return bus_idle();
}

static bool i2c_recv(u8 addr, u8 *buf, int n) {
    if (!bus_idle()) return false;
    if (XIicPs_MasterRecvPolled(&iic, buf, n, addr) != XST_SUCCESS) return false;
    return bus_idle();
}

/* Set the device's address pointer, then read from it. */
static bool eeprom_read(u8 addr, u8 offset, u8 *buf, int n) {
    if (!i2c_send(addr, &offset, 1)) return false;
    return i2c_recv(addr, buf, n);
}

static bool switch_set(u8 sw, u8 value) {
    bool ok = i2c_send(sw, &value, 1);
    usleep(1000);
    return ok;
}

/* The switch channel is already selected. */
static void report_sfp(u8 sw, int ch) {
    double v[19];
    u8 a0[96];
    bool ok = true;
    for (int off = 0; off < 96 && ok; off += 16) {
        ok = eeprom_read(0x50, (u8)off, &a0[off], 16);
    }
    if (!ok) {
        for (int k = 0; k < 19; k++) v[k] = 0.0;
        v[0] = 9.0; v[1] = (double)sw; v[2] = (double)ch; v[3] = 1.0;
        put19(ds_sfp, v);
        return;
    }

    for (int k = 0; k < 19; k++) v[k] = 0.0;
    v[0] = 1.0; v[1] = (double)sw; v[2] = (double)ch;
    v[3] = (double)a0[0];
    v[4] = (double)a0[12];
    v[5] = (double)((a0[60] << 8) | a0[61]);
    v[6] = (double)a0[92];
    v[7] = (double)a0[3];
    put19(ds_sfp, v);

    v[0] = 2.0;
    for (int k = 0; k < 16; k++) v[3 + k] = (double)a0[20 + k];
    put19(ds_sfp, v);

    v[0] = 3.0;
    for (int k = 0; k < 16; k++) v[3 + k] = (double)a0[40 + k];
    put19(ds_sfp, v);

    xil_printf("SFP on switch 0x%x ch %d: %d nm, %d x100MBd\r\n", (int)sw, ch,
               (int)((a0[60] << 8) | a0[61]), (int)a0[12]);

    u8 a2[16];
    for (int k = 0; k < 19; k++) v[k] = 0.0;
    v[1] = (double)sw; v[2] = (double)ch;
    if (!eeprom_read(0x51, 96, a2, 16)) {
        v[0] = 9.0; v[3] = 2.0;
        put19(ds_sfp, v);
        return;
    }
    v[0] = 4.0;
    v[3] = (double)(int16_t)((a2[0] << 8) | a2[1]);
    v[4] = (double)((a2[2] << 8) | a2[3]);
    v[5] = (double)((a2[4] << 8) | a2[5]);
    v[6] = (double)((a2[6] << 8) | a2[7]);
    v[7] = (double)((a2[8] << 8) | a2[9]);
    v[8] = (double)a2[14];
    put19(ds_sfp, v);
    xil_printf("  tx %d rx %d (x0.1 uW), bias %d (x2 uA), status 0x%x\r\n",
               (int)v[6], (int)v[7], (int)v[5], (int)a2[14]);
}

static void sfp_diag() {
    XIicPs_Config *cfg = XIicPs_LookupConfig(XPAR_XIICPS_1_DEVICE_ID);
    if (cfg == NULL ||
        XIicPs_CfgInitialize(&iic, cfg, cfg->BaseAddress) != XST_SUCCESS ||
        XIicPs_SetSClk(&iic, 100000) != XST_SUCCESS) {
        put2(ds_note, 1.0, 0.0);
        return;
    }

    /* Two TCA9548A switches sit on this bus. Remember what they select now
     * (TCP_Server leaves 0x74 on the clock chips' bridge) and put it back. */
    const u8 sw[2] = {0x74, 0x75};
    u8 saved[2] = {0, 0};
    bool present[2];
    for (int m = 0; m < 2; m++) {
        present[m] = i2c_recv(sw[m], &saved[m], 1);
        if (!present[m]) put2(ds_note, 10.0, (double)sw[m]);
    }

    int hits = 0;
    for (int m = 0; m < 2; m++) {
        if (!present[m]) continue;
        if (present[1 - m]) switch_set(sw[1 - m], 0);
        for (int ch = 0; ch < 8; ch++) {
            if (!switch_set(sw[m], (u8)(1u << ch))) continue;
            u8 id = 0;
            if (!eeprom_read(0x50, 0, &id, 1)) continue;
            hits++;
            put3(ds_scan, (double)sw[m], (double)ch, (double)id);
            if (id == 0x03) report_sfp(sw[m], ch);
        }
        switch_set(sw[m], 0);
    }
    put3(ds_scan, -1.0, -1.0, (double)hits);

    for (int m = 0; m < 2; m++) {
        if (present[m]) switch_set(sw[m], saved[m]);
    }
}

int main() {
    init();
    xil_printf("FLARE link diagnosis\r\n");

    /* Start the counters from zero so growth over the next two seconds shows.
     * These two calls pulse bits of the FLARE control register only; unlike
     * reset_module() they reset nothing else. */
    flare_controller.clear_counters();
    flare_controller.clear_sticky();
    usleep(1000);
    report_flare(0);
    sleep(1);
    report_flare(1);
    sleep(1);
    report_flare(2);

    sfp_diag();

    put2(ds_note, 0.0, 0.0);
    xil_printf("FLARE link diagnosis done\r\n");
    return 0;
}
