//-----------------------------------------------------------------------------
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------
// Generic periodic LF read, simulate, and T5577 clone standalone mode.
//-----------------------------------------------------------------------------

#include "standalone.h"

#include "BigBuf.h"
#include "appmain.h"
#include "dbprint.h"
#include "fpga_apis.h"
#include "fpga_loader.h"
#include "lfdemod.h"
#include "lfops.h"
#include "lfsampling.h"
#include "pm3_cmd.h"
#include "protocols.h"
#include "proxmark3_arm.h"
#include "ticks_apis.h"
#include "util.h"

#include <string.h>

#define LF_AUTOREPLAY_MIN_PERIOD 32
#define LF_AUTOREPLAY_MAX_BITS 224
#define LF_AUTOREPLAY_MAX_ERRORS 4
#define LF_AUTOREPLAY_MAX_MISMATCH_PERCENT 1
#define LF_AUTOREPLAY_VERIFY_ATTEMPTS 2

typedef enum {
    LF_AUTOREPLAY_DIRECT,
    LF_AUTOREPLAY_MANCHESTER,
    LF_AUTOREPLAY_BIPHASE,
    LF_AUTOREPLAY_DIPHASE,
    LF_AUTOREPLAY_FSK,
    LF_AUTOREPLAY_PSK1,
} lf_autoreplay_modulation_t;

typedef struct {
    lf_autoreplay_modulation_t modulation;
    uint8_t invert;
} lf_autoreplay_hypothesis_t;

typedef struct {
    lf_autoreplay_modulation_t modulation;
    uint8_t clock;
    uint8_t invert;
    uint8_t fc_high;
    uint8_t fc_low;
    uint8_t carrier;
    uint8_t errors;
    uint16_t period;
    uint16_t bit_count;
    uint16_t mismatch_count;
    uint8_t bits[LF_AUTOREPLAY_MAX_BITS];
} lf_autoreplay_capture_t;

static const lf_autoreplay_hypothesis_t lf_autoreplay_hypotheses[] = {
    {LF_AUTOREPLAY_FSK, 0},
    {LF_AUTOREPLAY_MANCHESTER, 0},
    {LF_AUTOREPLAY_BIPHASE, 0},
    {LF_AUTOREPLAY_DIPHASE, 1},
    {LF_AUTOREPLAY_PSK1, 0},
    {LF_AUTOREPLAY_DIRECT, 0},
};

static uint16_t lf_autoreplay_gcd(uint16_t a, uint16_t b) {
    while (b != 0) {
        uint16_t remainder = a % b;
        a = b;
        b = remainder;
    }
    return a;
}

static uint16_t lf_autoreplay_clone_length(uint16_t period) {
    uint16_t length = (period / lf_autoreplay_gcd(period, 32)) * 32;
    return length <= LF_AUTOREPLAY_MAX_BITS ? length : 0;
}

static bool lf_autoreplay_clock_supported(int clock) {
    switch (clock) {
        case 8:
        case 16:
        case 32:
        case 40:
        case 50:
        case 64:
        case 100:
        case 128:
            return true;
        default:
            return false;
    }
}

static uint32_t lf_autoreplay_clock_config(uint8_t clock) {
    switch (clock) {
        case 8:
            return T55x7_BITRATE_RF_8;
        case 16:
            return T55x7_BITRATE_RF_16;
        case 32:
            return T55x7_BITRATE_RF_32;
        case 40:
            return T55x7_BITRATE_RF_40;
        case 50:
            return T55x7_BITRATE_RF_50;
        case 64:
            return T55x7_BITRATE_RF_64;
        case 100:
            return T55x7_BITRATE_RF_100;
        case 128:
            return T55x7_BITRATE_RF_128;
        default:
            return 0;
    }
}

static const char *lf_autoreplay_modulation_name(const lf_autoreplay_capture_t *capture) {
    switch (capture->modulation) {
        case LF_AUTOREPLAY_DIRECT:
            return "NRZ/direct";
        case LF_AUTOREPLAY_MANCHESTER:
            return "ASK/Manchester";
        case LF_AUTOREPLAY_BIPHASE:
            return "ASK/biphase";
        case LF_AUTOREPLAY_DIPHASE:
            return "ASK/diphase";
        case LF_AUTOREPLAY_FSK:
            if (capture->fc_high == 10 && capture->fc_low == 8) {
                return capture->invert ? "FSK2a" : "FSK2";
            }
            return capture->invert ? "FSK1" : "FSK1a";
        case LF_AUTOREPLAY_PSK1:
            return "PSK1";
        default:
            return "unknown";
    }
}

static bool lf_autoreplay_is_fsk_wave(const uint8_t *samples, size_t sample_count) {
    uint16_t field_clocks = countFC(samples, sample_count, true);
    uint8_t high = field_clocks >> 8;
    uint8_t low = field_clocks & 0xff;
    return (high == 10 && low == 8) || (high == 8 && low == 5);
}

static bool lf_autoreplay_extract_period(const uint8_t *bits, size_t size, lf_autoreplay_capture_t *capture) {
    uint32_t best_mismatches = 0xffffffffU;
    uint32_t best_valid = 0;
    uint16_t best_period = 0;
    uint8_t best_bits[LF_AUTOREPLAY_MAX_BITS] = {0};
    size_t max_start = MIN((size_t)31, size / 4);

    for (size_t start = 0; start <= max_start; start++) {
        for (uint16_t period = LF_AUTOREPLAY_MIN_PERIOD; period <= LF_AUTOREPLAY_MAX_BITS; period++) {
            WDT_HIT();
            if (lf_autoreplay_clone_length(period) == 0 || size <= start + period) {
                continue;
            }

            uint16_t zeros[LF_AUTOREPLAY_MAX_BITS] = {0};
            uint16_t ones[LF_AUTOREPLAY_MAX_BITS] = {0};
            uint32_t valid = 0;

            for (size_t i = start; i < size; i++) {
                uint8_t bit = bits[i];
                if (bit > 1) {
                    continue;
                }
                uint16_t phase = (i - start) % period;
                if (bit) {
                    ones[phase]++;
                } else {
                    zeros[phase]++;
                }
                valid++;
            }

            uint32_t minimum_valid = period + MAX((uint16_t)32, (uint16_t)(period / 3));
            if (valid < minimum_valid) {
                continue;
            }

            uint32_t mismatches = 0;
            bool covered = true;
            uint8_t period_bits[LF_AUTOREPLAY_MAX_BITS] = {0};
            for (uint16_t i = 0; i < period; i++) {
                if (zeros[i] + ones[i] == 0) {
                    covered = false;
                    break;
                }
                period_bits[i] = ones[i] > zeros[i];
                mismatches += MIN(ones[i], zeros[i]);
            }

            if (!covered || mismatches * 100 > valid * LF_AUTOREPLAY_MAX_MISMATCH_PERCENT) {
                continue;
            }

            if (best_period == 0 || mismatches * best_valid < best_mismatches * valid ||
                    (mismatches * best_valid == best_mismatches * valid && period < best_period)) {
                best_mismatches = mismatches;
                best_valid = valid;
                best_period = period;
                memcpy(best_bits, period_bits, period);
            }
        }
    }

    if (best_period == 0) {
        return false;
    }

    capture->period = best_period;
    capture->mismatch_count = best_mismatches;
    memcpy(capture->bits, best_bits, best_period);
    return true;
}

static uint16_t lf_autoreplay_rotation_mismatches(const uint8_t *first, const uint8_t *second,
                                                   uint16_t period) {
    uint16_t best = period;
    for (uint16_t rotation = 0; rotation < period; rotation++) {
        uint16_t mismatches = 0;
        for (uint16_t i = 0; i < period; i++) {
            if (first[i] != second[(i + rotation) % period]) {
                mismatches++;
            }
        }
        best = MIN(best, mismatches);
    }
    return best;
}

static bool lf_autoreplay_demodulate(const lf_autoreplay_hypothesis_t *hypothesis,
                                     lf_autoreplay_capture_t *capture) {
    uint32_t sampled_bits = SampleLF(false, 0, false, false);
    size_t size = sampled_bits / 8;
    uint8_t *samples = BigBuf_get_addr();
    int clock = 0;
    int start_index = 0;
    int invert = hypothesis->invert;
    int errors = 0;

    if (size < 1024 || getSignalProperties()->isnoise) {
        return false;
    }

    memset(capture, 0, sizeof(*capture));
    capture->modulation = hypothesis->modulation;
    capture->invert = hypothesis->invert;

    if (hypothesis->modulation == LF_AUTOREPLAY_FSK) {
        uint16_t field_clocks = countFC(samples, size, true);
        capture->fc_high = field_clocks >> 8;
        capture->fc_low = field_clocks & 0xff;
        if (!lf_autoreplay_is_fsk_wave(samples, size)) {
            return false;
        }
        clock = detectFSKClk(samples, size, capture->fc_high, capture->fc_low, &start_index);
        if (!lf_autoreplay_clock_supported(clock)) {
            return false;
        }
        size = fskdemod(samples, size, clock, capture->invert, capture->fc_high,
                        capture->fc_low, &start_index);
    } else {
        if (lf_autoreplay_is_fsk_wave(samples, size)) {
            return false;
        }

        switch (hypothesis->modulation) {
            case LF_AUTOREPLAY_MANCHESTER:
                errors = askdemod_ext(samples, &size, &clock, &invert,
                                      LF_AUTOREPLAY_MAX_ERRORS, 0, 1, &start_index);
                break;
            case LF_AUTOREPLAY_BIPHASE:
            case LF_AUTOREPLAY_DIPHASE: {
                errors = askdemod_ext(samples, &size, &clock, &invert,
                                      LF_AUTOREPLAY_MAX_ERRORS, 0, 0, &start_index);
                if (errors < 0 || errors > LF_AUTOREPLAY_MAX_ERRORS) {
                    return false;
                }
                int offset = 0;
                int biphase_errors = BiphaseRawDecode(samples, &size, &offset, hypothesis->invert);
                if (biphase_errors < 0) {
                    return false;
                }
                errors += biphase_errors;
                break;
            }
            case LF_AUTOREPLAY_PSK1: {
                uint8_t phase = 0;
                size_t first_shift = 0;
                capture->carrier = 0;
                clock = DetectPSKClock(samples, size, 0, &first_shift, &phase, &capture->carrier);
                if (capture->carrier != 2 && capture->carrier != 4 && capture->carrier != 8) {
                    return false;
                }
                errors = pskRawDemod_ext(samples, &size, &clock, &invert, &start_index);
                break;
            }
            case LF_AUTOREPLAY_DIRECT:
                errors = nrzRawDemod(samples, &size, &clock, &invert, &start_index);
                break;
            default:
                return false;
        }
    }

    if (errors < 0 || errors > LF_AUTOREPLAY_MAX_ERRORS || size < LF_AUTOREPLAY_MIN_PERIOD ||
            !lf_autoreplay_clock_supported(clock)) {
        return false;
    }

    capture->clock = clock;
    capture->errors = errors;
    return lf_autoreplay_extract_period(samples, size, capture);
}

static bool lf_autoreplay_same_config(const lf_autoreplay_capture_t *first,
                                      const lf_autoreplay_capture_t *second) {
    return first->modulation == second->modulation && first->clock == second->clock &&
           first->invert == second->invert && first->fc_high == second->fc_high &&
           first->fc_low == second->fc_low && first->carrier == second->carrier;
}

static bool lf_autoreplay_confirm(const lf_autoreplay_hypothesis_t *hypothesis,
                                  lf_autoreplay_capture_t *capture) {
    lf_autoreplay_capture_t confirmation;
    if (!lf_autoreplay_demodulate(hypothesis, &confirmation) ||
            !lf_autoreplay_same_config(capture, &confirmation) ||
            capture->period != confirmation.period) {
        return false;
    }

    uint16_t mismatches = lf_autoreplay_rotation_mismatches(capture->bits, confirmation.bits,
                                                            capture->period);
    if (mismatches * 100 > capture->period * LF_AUTOREPLAY_MAX_MISMATCH_PERCENT) {
        return false;
    }
    capture->mismatch_count += confirmation.mismatch_count + mismatches;
    capture->errors += confirmation.errors;
    return true;
}

static void lf_autoreplay_expand_for_t5577(lf_autoreplay_capture_t *capture) {
    uint16_t clone_length = lf_autoreplay_clone_length(capture->period);
    for (uint16_t i = capture->period; i < clone_length; i++) {
        capture->bits[i] = capture->bits[i % capture->period];
    }
    capture->bit_count = clone_length;
}

static bool lf_autoreplay_read(lf_autoreplay_capture_t *capture) {
    lf_autoreplay_capture_t best = {0};
    bool found = false;

    for (size_t i = 0; i < sizeof(lf_autoreplay_hypotheses) / sizeof(lf_autoreplay_hypotheses[0]); i++) {
        WDT_HIT();
        if (data_available()) {
            return false;
        }

        lf_autoreplay_capture_t candidate;
        if (!lf_autoreplay_demodulate(&lf_autoreplay_hypotheses[i], &candidate) ||
                !lf_autoreplay_confirm(&lf_autoreplay_hypotheses[i], &candidate)) {
            continue;
        }

        uint32_t candidate_score = candidate.errors * 1000 + candidate.mismatch_count;
        uint32_t best_score = best.errors * 1000 + best.mismatch_count;
        if (!found || candidate_score < best_score) {
            best = candidate;
            found = true;
        }
    }

    if (!found) {
        return false;
    }

    lf_autoreplay_expand_for_t5577(&best);
    *capture = best;
    return true;
}

static uint32_t lf_autoreplay_modulation_config(const lf_autoreplay_capture_t *capture) {
    switch (capture->modulation) {
        case LF_AUTOREPLAY_DIRECT:
            return T55x7_MODULATION_DIRECT;
        case LF_AUTOREPLAY_MANCHESTER:
            return T55x7_MODULATION_MANCHESTER;
        case LF_AUTOREPLAY_BIPHASE:
            return T55x7_MODULATION_BIPHASE;
        case LF_AUTOREPLAY_DIPHASE:
            return T55x7_MODULATION_DIPHASE;
        case LF_AUTOREPLAY_FSK:
            if (capture->fc_high == 10 && capture->fc_low == 8) {
                return capture->invert ? T55x7_MODULATION_FSK2a : T55x7_MODULATION_FSK2;
            }
            return capture->invert ? T55x7_MODULATION_FSK1 : T55x7_MODULATION_FSK1a;
        case LF_AUTOREPLAY_PSK1:
            return T55x7_MODULATION_PSK1;
        default:
            return 0;
    }
}

static uint32_t lf_autoreplay_psk_config(const lf_autoreplay_capture_t *capture) {
    if (capture->modulation != LF_AUTOREPLAY_PSK1) {
        return 0;
    }
    switch (capture->carrier) {
        case 4:
            return T55x7_PSKCF_RF_4;
        case 8:
            return T55x7_PSKCF_RF_8;
        default:
            return T55x7_PSKCF_RF_2;
    }
}

static void lf_autoreplay_append_fsk_wave(uint8_t *wave, size_t *wave_size, uint8_t field_clock,
                                          uint8_t bit_clock, int16_t *remainder) {
    uint8_t half_clock = field_clock / 2;
    uint8_t waves = (bit_clock + *remainder) / field_clock;
    for (uint8_t i = 0; i < waves; i++) {
        memset(wave + *wave_size, 0, field_clock - half_clock);
        memset(wave + *wave_size + field_clock - half_clock, 1, half_clock);
        *wave_size += field_clock;
    }
    *remainder = (bit_clock + *remainder) % field_clock;
    if (*remainder > half_clock) {
        memset(wave + *wave_size, 0, field_clock - half_clock);
        memset(wave + *wave_size + field_clock - half_clock, 1, half_clock);
        *wave_size += field_clock;
        *remainder -= field_clock;
    }
}

static bool lf_autoreplay_build_waveform(const lf_autoreplay_capture_t *capture, size_t *wave_size) {
    BigBuf_free();
    BigBuf_Clear_ext(false);
    uint8_t *wave = BigBuf_get_addr();
    size_t maximum = BigBuf_max_traceLen();
    size_t required = (size_t)capture->bit_count * capture->clock;
    if (required + 16 > maximum) {
        return false;
    }

    *wave_size = 0;
    uint8_t phase = 0;
    int16_t remainder = 0;

    for (uint16_t i = 0; i < capture->bit_count; i++) {
        uint8_t bit = capture->bits[i];
        switch (capture->modulation) {
            case LF_AUTOREPLAY_DIRECT:
                if (*wave_size + capture->clock > maximum) return false;
                memset(wave + *wave_size, bit, capture->clock);
                *wave_size += capture->clock;
                break;
            case LF_AUTOREPLAY_MANCHESTER: {
                uint8_t half = capture->clock / 2;
                if (*wave_size + capture->clock > maximum) return false;
                memset(wave + *wave_size, bit, half);
                memset(wave + *wave_size + half, bit ^ 1, capture->clock - half);
                *wave_size += capture->clock;
                break;
            }
            case LF_AUTOREPLAY_BIPHASE:
            case LF_AUTOREPLAY_DIPHASE: {
                uint8_t half = capture->clock / 2;
                uint8_t encoded = bit ^ capture->invert;
                if (*wave_size + capture->clock > maximum) return false;
                if (encoded) {
                    memset(wave + *wave_size, encoded ^ 1 ^ phase, half);
                    memset(wave + *wave_size + half, encoded ^ phase, capture->clock - half);
                } else {
                    memset(wave + *wave_size, phase, capture->clock);
                    phase ^= 1;
                }
                *wave_size += capture->clock;
                break;
            }
            case LF_AUTOREPLAY_FSK:
                lf_autoreplay_append_fsk_wave(wave, wave_size,
                                              bit ? capture->fc_high : capture->fc_low,
                                              capture->clock, &remainder);
                if (*wave_size > maximum) return false;
                break;
            case LF_AUTOREPLAY_PSK1: {
                uint8_t half = capture->carrier / 2;
                uint8_t generated = 0;
                if (bit != phase) {
                    if (*wave_size + capture->carrier > maximum) return false;
                    memset(wave + *wave_size, phase ^ 1, half);
                    memset(wave + *wave_size + half, phase, capture->carrier - half);
                    *wave_size += capture->carrier;
                    phase ^= 1;
                    generated += capture->carrier;
                }
                for (; generated < capture->clock; generated += capture->carrier) {
                    if (*wave_size + capture->carrier > maximum) return false;
                    memset(wave + *wave_size, phase, half);
                    memset(wave + *wave_size + half, phase ^ 1, capture->carrier - half);
                    *wave_size += capture->carrier;
                }
                break;
            }
            default:
                return false;
        }
    }
    return *wave_size > 0;
}

static bool lf_autoreplay_simulate(const lf_autoreplay_capture_t *capture) {
    size_t wave_size = 0;
    if (!lf_autoreplay_build_waveform(capture, &wave_size)) {
        return false;
    }
    SimulateTagLowFrequency(wave_size, 0, false);
    return true;
}

static uint32_t lf_autoreplay_pack_block(const lf_autoreplay_capture_t *capture, uint8_t block) {
    uint32_t value = 0;
    uint16_t offset = (block - 1) * 32;
    for (uint8_t i = 0; i < 32; i++) {
        value = (value << 1) | capture->bits[offset + i];
    }
    return value;
}

static void lf_autoreplay_write_block(uint8_t block, uint32_t value) {
    t55xx_write_block_t command = {
        .data = value,
        .pwd = 0,
        .blockno = block,
        .flags = 0,
    };
    T55xxWriteBlock((uint8_t *)&command, false);
}

static bool lf_autoreplay_demodulate_block(const lf_autoreplay_capture_t *capture,
                                           uint8_t *samples, size_t *size) {
    int clock = capture->clock;
    int start_index = 0;
    int invert = capture->invert;
    int errors = 0;

    switch (capture->modulation) {
        case LF_AUTOREPLAY_FSK:
            *size = fskdemod(samples, *size, clock, invert, capture->fc_high,
                             capture->fc_low, &start_index);
            break;
        case LF_AUTOREPLAY_MANCHESTER:
            errors = askdemod_ext(samples, size, &clock, &invert,
                                  LF_AUTOREPLAY_MAX_ERRORS, 0, 1, &start_index);
            break;
        case LF_AUTOREPLAY_BIPHASE:
        case LF_AUTOREPLAY_DIPHASE: {
            errors = askdemod_ext(samples, size, &clock, &invert,
                                  LF_AUTOREPLAY_MAX_ERRORS, 0, 0, &start_index);
            if (errors < 0 || errors > LF_AUTOREPLAY_MAX_ERRORS) {
                return false;
            }
            int offset = 0;
            int biphase_errors = BiphaseRawDecode(samples, size, &offset, capture->invert);
            if (biphase_errors < 0) {
                return false;
            }
            errors += biphase_errors;
            break;
        }
        case LF_AUTOREPLAY_PSK1:
            errors = pskRawDemod_ext(samples, size, &clock, &invert, &start_index);
            break;
        case LF_AUTOREPLAY_DIRECT:
            errors = nrzRawDemod(samples, size, &clock, &invert, &start_index);
            break;
        default:
            return false;
    }

    return errors >= 0 && errors <= LF_AUTOREPLAY_MAX_ERRORS && *size >= 32 &&
           clock == capture->clock;
}

static bool lf_autoreplay_contains_block(const uint8_t *bits, size_t size, uint32_t expected) {
    for (size_t offset = 0; offset + 32 <= size; offset++) {
        uint32_t value = 0;
        bool valid = true;
        for (uint8_t i = 0; i < 32; i++) {
            if (bits[offset + i] > 1) {
                valid = false;
                break;
            }
            value = (value << 1) | bits[offset + i];
        }
        if (valid && value == expected) {
            return true;
        }
    }
    return false;
}

static bool lf_autoreplay_verify_block(const lf_autoreplay_capture_t *capture,
                                       uint8_t block, uint32_t expected) {
    for (uint8_t attempt = 0; attempt < LF_AUTOREPLAY_VERIFY_ATTEMPTS; attempt++) {
        WDT_HIT();
        T55xxReadBlock(0, false, false, block, 0, 0, false);
        size_t size = getSampleCounter();
        uint8_t *samples = BigBuf_get_addr();
        if (size >= 1024 && lf_autoreplay_demodulate_block(capture, samples, &size) &&
                lf_autoreplay_contains_block(samples, size, expected)) {
            return true;
        }
    }
    Dbprintf("[-] page 0 block %u verify failed, expected %08x", block, expected);
    return false;
}

static bool lf_autoreplay_clone(const lf_autoreplay_capture_t *capture) {
    if (capture->bit_count == 0 || capture->bit_count > LF_AUTOREPLAY_MAX_BITS ||
            capture->bit_count % 32 != 0) {
        return false;
    }

    uint8_t blocks = capture->bit_count / 32;
    uint32_t configuration = lf_autoreplay_clock_config(capture->clock) |
                             lf_autoreplay_modulation_config(capture) |
                             lf_autoreplay_psk_config(capture) |
                             ((uint32_t)blocks << T55x7_MAXBLOCK_SHIFT);

    for (uint8_t block = blocks; block > 0; block--) {
        lf_autoreplay_write_block(block, lf_autoreplay_pack_block(capture, block));
    }
    lf_autoreplay_write_block(0, configuration);
    Dbprintf("[=] T5577 block 0: %08x", configuration);

    DbpString("[=] reading page 0 back for verification");
    for (uint8_t block = 1; block <= blocks; block++) {
        if (!lf_autoreplay_verify_block(capture, block,
                                        lf_autoreplay_pack_block(capture, block))) {
            return false;
        }
    }
    if (!lf_autoreplay_verify_block(capture, 0, configuration)) {
        return false;
    }
    return true;
}

void ModInfo(void) {
    DbpString("  LF periodic auto-demodulate/read/sim/clone");
}

void RunMod(void) {
    StandAloneMode();
    FpgaDownloadAndGo(FPGA_BITSTREAM_LF);
    DbpString(">> LF AutoReplay started <<");

    enum {
        STATE_READ,
        STATE_SIMULATE,
        STATE_CLONE,
    } state = STATE_READ;
    lf_autoreplay_capture_t capture = {0};

    for (;;) {
        WDT_HIT();
        if (data_available()) break;

        LEDsoff();
        if (state == STATE_READ) LED_A_ON();
        if (state == STATE_SIMULATE) LED_C_ON();
        if (state == STATE_CLONE) LED_D_ON();

        if (BUTTON_HELD(280) != BUTTON_HOLD) {
            continue;
        }
        WAIT_BUTTON_RELEASED();

        if (state == STATE_READ) {
            DbpString("[=] scanning LF modulation hypotheses");
            LED_B_ON();
            if (!lf_autoreplay_read(&capture)) {
                DbpString("[-] no independently confirmed periodic LF frame");
                SpinErr(LED_A, 100, 12);
                continue;
            }
            Dbprintf("[+] captured %s, RF/%u, period %u, stored %u bits",
                     lf_autoreplay_modulation_name(&capture), capture.clock,
                     capture.period, capture.bit_count);
            DbpString("[=] LED C: capture ready; press the button to simulate");
            SpinErr(LED_A | LED_B, 250, 2);
            state = STATE_SIMULATE;
        } else if (state == STATE_SIMULATE) {
            LED_B_ON();
            Dbprintf("[=] LEDs B+C: simulating %s RF/%u; press the button to stop",
                     lf_autoreplay_modulation_name(&capture),
                     capture.clock);
            if (!lf_autoreplay_simulate(&capture)) {
                DbpString("[-] capture does not fit the simulation buffer");
                SpinErr(LED_C, 100, 12);
                continue;
            }
            DbpString("[=] LED D: simulation stopped; press the button to clone");
            state = STATE_CLONE;
        } else {
            LED_B_ON();
            DbpString("[!] writing blocks to an unpassworded T5577");
            if (!lf_autoreplay_clone(&capture)) {
                DbpString("[-] clone write or verification failed; reposition the tag and retry");
                SpinErr(LED_D, 100, 12);
                continue;
            }
            DbpString("[+] clone write and page 0 verification complete");
            SpinErr(LED_A | LED_D, 250, 2);
            state = STATE_READ;
        }
    }

    FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
    SpinErr(LED_A | LED_B | LED_C | LED_D, 250, 5);
    LEDsoff();
    DbpString("[=] LF AutoReplay stopped");
}
