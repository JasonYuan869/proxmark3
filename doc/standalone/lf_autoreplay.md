# LF_AUTOREPLAY — Periodic LF Read/Simulate/Clone

> **Frequency:** LF (125 kHz)
> **Hardware:** Generic Proxmark3
> **Storage:** One capture in volatile RAM

[Back to Standalone Modes Index](../../armsrc/Standalone/readme.md#individual-mode-documentation) | [Source Code](../../armsrc/Standalone/lf_autoreplay.c)

## What

AutoReplay is a protocol-independent counterpart to `LF_SAMYRUN`. It tries the
firmware's generic LF demodulators, identifies a repeating bitstream, stores its
modulation parameters and data in RAM, simulates it, and can write an equivalent
configuration and data image to an unpassworded T5577.

It supports the T5577 modulation and clock combinations that can be represented
without extended mode:

- ASK Manchester, biphase, and diphase
- NRZ/direct
- FSK1/FSK1a (`RF/8` and `RF/5` tones)
- FSK2/FSK2a (`RF/10` and `RF/8` tones)
- PSK1 with `RF/2`, `RF/4`, or `RF/8` carrier
- Bit clocks `RF/8`, `RF/16`, `RF/32`, `RF/40`, `RF/50`, `RF/64`, `RF/100`, and `RF/128`

The recovered periodic stream must expand to a whole number of 32-bit T5577
blocks without exceeding the seven data blocks (224 bits).

## Confidence checks

A demodulation is accepted only when all of the following hold:

1. The signal is not classified as noise.
2. Its modulation parameters map to a normal T5577 configuration.
3. The demodulated data contains a periodic frame of 32–224 bits with at most
   one percent disagreement between repetitions.
4. A new, independent LF acquisition produces the same modulation parameters
   and periodic frame, allowing for cyclic rotation.

This rejects many false positives, but it does not prove protocol-level
validity. AutoReplay does not validate a preamble, checksum, parity, or facility
code because it intentionally does not assume a tag protocol.

## Controls and LEDs

The button workflow follows SamyRun. LED B consistently means that an operation
is active, so the waiting and transmitting states are visually distinct:

| State | LED | Button behavior |
|-------|-----|-----------------|
| Ready to read | A | Hold for 280 ms to start capture |
| Capturing and analyzing | A+B | Wait for the two independent reads to finish |
| Capture ready | C | Hold for 280 ms to start simulation |
| Actively simulating | B+C | Press the button to stop simulation |
| Ready to clone | D | Hold for 280 ms to start the T5577 write |
| Actively cloning | B+D | Wait for the write and page 0 verification to finish |

An error flashes the current state's primary LED and leaves the mode in that
state. A USB command exits the standalone application.

## Important limitations

- The clone target must be an **unpassworded T5577**. The mode never performs a
  password check and never sends test mode.
- Sequence terminators, T5577 extended-mode bit rates, gaps, and non-T5577
  modulation schemes are not supported.
- PSK2/PSK3 are normalized to a replay-equivalent PSK1 stream when the generic
  PSK demodulator can recover it.
- Capture data is lost when the device resets or leaves standalone mode.
- Simulation is synthesized from the recovered bitstream; it is not a raw ADC
  waveform recording.
- Clone mode reads every written page 0 block back, retrying each read once. It
  reports success only when all data blocks and block 0 match. A failure leaves
  the mode ready to rewrite and verify the tag again.
- Test with tags and readers you are authorized to assess. Clone mode overwrites
  blocks 0–7 as needed on the presented T5577.

## Compilation

```text
make clean
make STANDALONE=LF_AUTOREPLAY -j
./pm3-flash-fullimage
```

Or set this in `Makefile.platform`:

```text
STANDALONE=LF_AUTOREPLAY
```

## CLI capture companion

The host-side `client/luascripts/lf_autoreplay.lua` script applies the same
independent-capture and periodicity checks through `data autodemod`:

```text
script run lf_autoreplay
```

It does not write a tag. On success it saves a timestamped native `.pm3` trace
from the confirmation read and a 32-byte `.bin` image containing T5577 page 0
blocks 0–7 in big-endian display order. Page 1 is intentionally omitted and
unused page 0 data blocks are zero-filled. Run `script run lf_autoreplay -h`
for capture-count, sample-count, confidence-margin, and filename options.
