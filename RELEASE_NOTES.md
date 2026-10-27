# FT8_Beacon v4.9 — Release notes

**EXPERIMENTAL — use at your own risk.** No warranty. Fit a proper **external low-pass filter (LPF) per band** before any antenna use (clone boards typically have none). Bench-test into a **dummy load**. Edit `CALLSIGN` / `LOCATOR` away from the fictional example **QQ0QQ** / **XX00** before transmitting.

## What’s in v4.9

- Docs: optional antenna note — 7 MHz EFHW (e.g. QRPGuys NoTune EFHW) covers 40/20/15/10 m without an ATU; LPF still required.
- Presently a **quad-band, quad-mode** GPS-timed CQ beacon: **FT8** (:00), **FT2** (:01), **FT4** (:02), **JTTY** (:03) on 40 / 20 / 15 / 10 m (240 s cycle).
- Standard FT8/FT4 type-1 CQ via `ft8_std.h` (PSK Reporter–friendly when enabled).
- FT2 uses the same packed symbols as FT4 with FT2 framing/timing for MSHV-style dials.
- JTTY: WSJT-X 3.3-family callsign action `CQ <call> CQ`, tones precomputed in `jtty_tones.h` (host tool `extras/encode_jtty.py`). On-air modulation in this release is hard 4-FSK to save flash.
- Optional GFSK shaping for FT8/FT4/FT2; random audio offsets 800–2400 Hz; band rotation; PPS or NMEA timing.

## Breaking / operator notes vs earlier private builds

- Shipped example identity is **QQ0QQ** / **XX00** (fictional). You must set your own call and grid and regenerate JTTY tones.
- Example callsign was updated from a previously used placeholder to fictional **QQ0QQ** / **XX00** (not for on-air use).
- Sketch-only package (no PC follower / companion tools).
- JTTY dials are preliminary (7090 / 14090 / 21090 / 28090 kHz pattern); verify locally.

## Not included

- PC-side followers, personal paths, or operator-specific tooling.
- Guarantees of on-air legality without your LPF, licence, and band checks.

## Docs

- README and USER_GUIDE now require a firmware backup before flash: keep the previous `.ino` or known-good hex, or dump the Nano (ATmega328P) flash with `avrdude`, because upload overwrites the image already on the board.
