-- ---------------------------------------------------------------------------
-- Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
--
-- This program is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- See LICENSE.txt for the text of the license.
-- ---------------------------------------------------------------------------

local getopt = require('getopt')
local ansicolors = require('ansicolors')

author = 'Proxmark3 contributors'
version = 'v1.0.0'
desc = [[
Repeatedly captures an LF signal, auto-demodulates it, finds its periodic data,
and confirms the result against a second independent capture. A validated read
is saved as a timestamped .pm3 trace and as a 32-byte T5577 page 0 .bin image.

The binary contains blocks 0 through 7 in big-endian display order. Unused data
blocks are zero-filled. This script never writes to a transponder.
]]
usage = [[
script run lf_autoreplay [-n <pairs>] [-s <samples>] [-m <dB>] [-o <prefix>]
]]
arguments = [[
    -h            this help
    -n <pairs>    maximum capture pairs to try (default 0: until success)
    -s <samples>  samples per LF read (default 40000)
    -m <dB>       minimum demodulation margin (default 6)
    -o <prefix>   output filename prefix (default lf-autoreplay)
]]
example = [[
    script run lf_autoreplay
    script run lf_autoreplay -n 5 -o office-badge
]]

local MIN_PERIOD = 32
local MAX_BITS = 224
local MAX_ERRORS = 4
local MAX_MISMATCH_PERCENT = 1

local CLOCK_CONFIG = {
    [8] = 0x00000000,
    [16] = 0x00040000,
    [32] = 0x00080000,
    [40] = 0x000C0000,
    [50] = 0x00100000,
    [64] = 0x00140000,
    [100] = 0x00180000,
    [128] = 0x001C0000,
}

local MOD_DIRECT = 0x00000000
local MOD_PSK1 = 0x00001000
local MOD_FSK1 = 0x00004000
local MOD_FSK2 = 0x00005000
local MOD_FSK1A = 0x00006000
local MOD_FSK2A = 0x00007000
local MOD_MANCHESTER = 0x00008000
local MOD_BIPHASE = 0x00010000
local MOD_DIPHASE = 0x00018000
local PSK_RF_4 = 0x00000400
local PSK_RF_8 = 0x00000800
local MAXBLOCK_SHIFT_VALUE = 0x20

local function help()
    print(author)
    print(version)
    print(desc)
    print(ansicolors.cyan .. 'Usage' .. ansicolors.reset)
    print(usage)
    print(ansicolors.cyan .. 'Arguments' .. ansicolors.reset)
    print(arguments)
    print(ansicolors.cyan .. 'Examples' .. ansicolors.reset)
    print(example)
end

local function gcd(a, b)
    while b ~= 0 do
        a, b = b, a % b
    end
    return a
end

local function stored_length(period)
    local length = (period / gcd(period, 32)) * 32
    if length > MAX_BITS then
        return nil
    end
    return length
end

local function extract_period(bits)
    local size = #bits
    local max_start = math.min(31, math.floor(size / 4))
    local best = nil

    for start = 1, max_start + 1 do
        for period = MIN_PERIOD, MAX_BITS do
            if stored_length(period) ~= nil and size >= start + period then
                local zeros, ones = {}, {}
                for i = 1, period do
                    zeros[i], ones[i] = 0, 0
                end

                local valid = 0
                local limit = math.min(size, start - 1 + period * 8)
                for i = start, limit do
                    local bit = bits[i]
                    if bit == 0 or bit == 1 then
                        local phase = ((i - start) % period) + 1
                        if bit == 0 then
                            zeros[phase] = zeros[phase] + 1
                        else
                            ones[phase] = ones[phase] + 1
                        end
                        valid = valid + 1
                    end
                end

                local minimum_valid = period + math.max(32, math.floor(period / 3))
                if valid >= minimum_valid then
                    local mismatches = 0
                    local period_bits = {}
                    local covered = true
                    for i = 1, period do
                        if zeros[i] + ones[i] == 0 then
                            covered = false
                            break
                        end
                        period_bits[i] = (ones[i] > zeros[i]) and 1 or 0
                        mismatches = mismatches + math.min(zeros[i], ones[i])
                    end

                    if covered and mismatches * 100 <= valid * MAX_MISMATCH_PERCENT then
                        if best == nil or
                                mismatches * best.valid < best.mismatches * valid or
                                (mismatches * best.valid == best.mismatches * valid and period < best.period) then
                            best = {
                                bits = period_bits,
                                period = period,
                                mismatches = mismatches,
                                valid = valid,
                            }
                        end
                    end
                end
            end
        end
    end
    return best
end

local function rotation_mismatches(first, second)
    local period = #first
    local best, best_rotation = period, 0
    for rotation = 0, period - 1 do
        local mismatches = 0
        for i = 1, period do
            local j = ((i + rotation - 1) % period) + 1
            if first[i] ~= second[j] then
                mismatches = mismatches + 1
            end
        end
        if mismatches < best then
            best, best_rotation = mismatches, rotation
        end
    end
    return best, best_rotation
end

local function modulation_config(capture)
    local result = capture.result
    local modulation = result.modulation
    local encoding = result.encoding

    if modulation == 'NRZ' or (modulation == 'ASK' and encoding == 'raw') then
        return MOD_DIRECT
    end
    if modulation == 'ASK' and encoding == 'manchester' then
        return MOD_MANCHESTER
    end
    if modulation == 'ASK' and encoding == 'biphase' then
        return result.inverted and MOD_DIPHASE or MOD_BIPHASE
    end
    if modulation == 'FSK' then
        if result.fchigh == 10 and result.fclow == 8 then
            return result.inverted and MOD_FSK2A or MOD_FSK2
        end
        if result.fchigh == 8 and result.fclow == 5 then
            return result.inverted and MOD_FSK1 or MOD_FSK1A
        end
        return nil, ('unsupported FSK field clocks %d/%d'):format(result.fchigh, result.fclow)
    end
    if modulation == 'PSK' then
        return MOD_PSK1
    end
    return nil, ('unsupported modulation %s/%s'):format(tostring(modulation), tostring(encoding))
end

local function fixed_t5577_config(capture)
    local clock = CLOCK_CONFIG[capture.result.clock]
    if clock == nil then
        return nil, ('RF/%d cannot be represented by a T5577 normal-mode bitrate'):format(capture.result.clock)
    end

    local modulation, err = modulation_config(capture)
    if modulation == nil then
        return nil, err
    end

    local psk = 0
    if capture.result.modulation == 'PSK' then
        if capture.result.carrier == 4 then
            psk = PSK_RF_4
        elseif capture.result.carrier == 8 then
            psk = PSK_RF_8
        elseif capture.result.carrier ~= 2 then
            return nil, ('unsupported PSK carrier RF/%d'):format(capture.result.carrier)
        end
    end
    return clock + modulation + psk
end

local function bits_from_string(value)
    local bits, errors = {}, 0
    for i = 1, #value do
        bits[i] = value:byte(i)
        if bits[i] > 1 then
            errors = errors + 1
        end
    end
    return bits, errors
end

local function acquire(samples, margin)
    core.console('data clear')
    core.console(('lf read -s %d'):format(samples))

    local raw = core.get_graph_buffer()
    if type(raw) ~= 'table' or #raw < 1024 then
        return nil, 'LF read returned fewer than 1024 samples'
    end

    core.console(('data autodemod --thres %.2f'):format(margin))
    local result = core.get_autodemod()
    if type(result) ~= 'table' or type(result.bits) ~= 'string' then
        return nil, 'auto-demodulation produced no bits'
    end
    if result.rank ~= 1 then
        return nil, ('rank %d, not the winning hypothesis, was the first to demodulate'):format(result.rank)
    end
    if not result.promoted and result.margin < margin then
        return nil, ('demodulation margin %.2f dB is below %.2f dB'):format(result.margin, margin)
    end

    local bits, errors = bits_from_string(result.bits)
    if errors > MAX_ERRORS or result.errors > MAX_ERRORS then
        return nil, ('demodulation contains %d error markers'):format(math.max(errors, result.errors))
    end

    local periodic = extract_period(bits)
    if periodic == nil then
        return nil, 'no low-error period between 32 and 224 bits was found'
    end

    local capture = {
        raw = raw,
        result = result,
        bits = periodic.bits,
        period = periodic.period,
        mismatches = periodic.mismatches,
    }
    local config, err = fixed_t5577_config(capture)
    if config == nil then
        return nil, err
    end
    capture.fixed_config = config
    return capture
end

local function confirm(first, second)
    if first.fixed_config ~= second.fixed_config then
        return false, 'the modulation configuration changed between reads'
    end
    if first.period ~= second.period then
        return false, ('the detected period changed from %d to %d bits'):format(first.period, second.period)
    end

    local mismatches = rotation_mismatches(first.bits, second.bits)
    if mismatches * 100 > first.period * MAX_MISMATCH_PERCENT then
        return false, ('the two frames differ by %d of %d bits'):format(mismatches, first.period)
    end
    second.confirmation_mismatches = mismatches
    return true
end

local function page0_blocks(capture)
    local length = assert(stored_length(capture.period))
    local block_count = length / 32
    local expanded = {}
    for i = 1, length do
        expanded[i] = capture.bits[((i - 1) % capture.period) + 1]
    end

    local blocks = {}
    blocks[1] = capture.fixed_config + block_count * MAXBLOCK_SHIFT_VALUE
    for block = 1, block_count do
        local value = 0
        for bit = 1, 32 do
            value = value * 2 + expanded[(block - 1) * 32 + bit]
        end
        blocks[block + 1] = value
    end
    for i = block_count + 2, 8 do
        blocks[i] = 0
    end
    return blocks, length
end

local function file_exists(path)
    local file = io.open(path, 'rb')
    if file == nil then
        return false
    end
    file:close()
    return true
end

local function output_paths(prefix)
    local timestamp = os.date('%Y%m%d-%H%M%S')
    local base = prefix .. '-' .. timestamp
    local suffix = 0
    while file_exists(base .. '-page0.bin') or file_exists(base .. '-raw.pm3') do
        suffix = suffix + 1
        base = ('%s-%s-%03d'):format(prefix, timestamp, suffix)
    end
    return base .. '-page0.bin', base .. '-raw.pm3'
end

local function write_u32be(file, value)
    local b1 = math.floor(value / 0x1000000) % 0x100
    local b2 = math.floor(value / 0x10000) % 0x100
    local b3 = math.floor(value / 0x100) % 0x100
    local b4 = value % 0x100
    return file:write(string.char(b1, b2, b3, b4))
end

local function write_page0(path, blocks)
    local file, err = io.open(path, 'wb')
    if file == nil then
        return nil, err
    end
    for i = 1, 8 do
        local ok
        ok, err = write_u32be(file, blocks[i])
        if ok == nil then
            file:close()
            return nil, err
        end
    end
    local ok
    ok, err = file:close()
    if ok == nil then
        return nil, err
    end
    return true
end

local function write_pm3(path, samples)
    local file, err = io.open(path, 'w')
    if file == nil then
        return nil, err
    end

    local lines = {}
    for i = 1, #samples do
        lines[#lines + 1] = tostring(samples[i]) .. '\n'
        if #lines == 1024 then
            local ok
            ok, err = file:write(table.concat(lines))
            if ok == nil then
                file:close()
                return nil, err
            end
            lines = {}
        end
    end
    if #lines > 0 then
        local ok
        ok, err = file:write(table.concat(lines))
        if ok == nil then
            file:close()
            return nil, err
        end
    end
    local ok
    ok, err = file:close()
    if ok == nil then
        return nil, err
    end
    return true
end

local function main(args)
    local max_pairs = 0
    local samples = 40000
    local margin = 6
    local prefix = 'lf-autoreplay'

    for option, value in getopt.getopt(args, 'hn:s:m:o:') do
        if option == 'h' then
            return help()
        elseif option == 'n' then
            max_pairs = tonumber(value)
        elseif option == 's' then
            samples = tonumber(value)
        elseif option == 'm' then
            margin = tonumber(value)
        elseif option == 'o' then
            prefix = value
        end
    end

    if max_pairs == nil or max_pairs < 0 or max_pairs ~= math.floor(max_pairs) then
        return print('[!] -n must be a non-negative integer')
    end
    if samples == nil or samples < 4096 or samples > 40000 or samples ~= math.floor(samples) then
        return print('[!] -s must be an integer from 4096 through 40000')
    end
    if margin == nil or margin < 3 then
        return print('[!] -m must be at least 3 dB')
    end
    if prefix == nil or prefix == '' or prefix:find('[%c]') then
        return print('[!] -o must be a non-empty filename prefix without control characters')
    end

    print(('[=] Reading %d LF samples per capture; press Enter to stop'):format(samples))
    local pair = 0
    while max_pairs == 0 or pair < max_pairs do
        pair = pair + 1
        if core.kbd_enter_pressed() then
            print('[=] stopped by user')
            return
        end

        print(('[=] Capture pair %d, first read'):format(pair))
        local first, err = acquire(samples, margin)
        if first == nil then
            print('[!] ' .. err)
        else
            print(('[=] Candidate %s/%s RF/%d, period %d bits; confirming'):format(
                first.result.modulation, first.result.encoding, first.result.clock, first.period))

            local second
            second, err = acquire(samples, margin)
            if second == nil then
                print('[!] confirmation read: ' .. err)
            else
                local valid
                valid, err = confirm(first, second)
                if not valid then
                    print('[!] confirmation failed: ' .. err)
                else
                    local blocks, stored_bits = page0_blocks(second)
                    local bin_path, pm3_path = output_paths(prefix)

                    local ok
                    ok, err = write_pm3(pm3_path, second.raw)
                    if not ok then
                        return print(('[!] could not save %s: %s'):format(pm3_path, tostring(err)))
                    end
                    ok, err = write_page0(bin_path, blocks)
                    if not ok then
                        return print(('[!] saved %s, but could not save %s: %s'):format(
                            pm3_path, bin_path, tostring(err)))
                    end

                    print(('[+] Validated %s/%s RF/%d, period %d bits, stored as %d bits'):format(
                        second.result.modulation, second.result.encoding, second.result.clock,
                        second.period, stored_bits))
                    print(('[+] Decision margin %.2f dB, SNR %.2f dB, eye %.3f, cross-read errors %d'):format(
                        second.result.margin, second.result.snr, second.result.eye,
                        second.confirmation_mismatches))
                    for i = 1, 8 do
                        print(('    page 0 block %d: %08X'):format(i - 1, blocks[i]))
                    end
                    print('[+] Saved page 0 image: ' .. bin_path)
                    print('[+] Saved validated raw trace: ' .. pm3_path)
                    return
                end
            end
        end
        core.clearCommandBuffer()
    end
    print(('[!] no independently confirmed periodic LF frame after %d capture pairs'):format(pair))
end

main(args)
