# FT8_Beacon v4.9 — experimental multi-mode CQ beacon

**EXPERIMENTAL — use at your own risk.** No warranty of any kind. You are solely responsible for holding a valid amateur radio licence, staying in band, and fitting proper RF filtering before connecting an antenna.

Presently configured as a **quad-band, quad-mode** experimental CQ beacon (**FT8, FT4, FT2, and JTTY**) for common AliExpress / Banggood **WSPR beacon** clones (Arduino Nano ATmega328P, Si5351, GPS). Bands: 40 / 20 / 15 / 10 m.

Example identity in this package is the **fictional** callsign **QQ0QQ** and locator **XX00**.  
**Do not transmit with QQ0QQ/XX00 on air.** Edit `CALLSIGN` and `LOCATOR` at the top of the sketch to your licensed call and Maidenhead grid **before** you flash or key RF.

## Safety (read first)

1. **Amateur licence required** to transmit.
2. **External low-pass filter (LPF) required** for each band before any antenna use. These boards typically have **no onboard LPF**. Without a proper LPF you risk illegal harmonics and damaging other services.
3. Use a **dummy load** for all bench tests and first bring-up.
4. Stay inside your country’s band allocations and power limits.
5. This firmware is experimental; timing, dials (especially JTTY), and JTTY hard-FSK behaviour may change.

## Before you flash

1. Open `FT8_Beacon_v4_9.ino`.
2. Set `#define CALLSIGN` and `#define LOCATOR` to **your** call and 4-character grid.
3. If you changed the callsign, regenerate JTTY tones:
   ```bash
   python3 extras/encode_jtty.py YOURCALL
   ```
   That overwrites `jtty_tones.h` next to the sketch.
4. Never leave the example `QQ0QQ` / `XX00` in a radio that can radiate.

Full step-by-step flash and config notes: **[USER_GUIDE.md](USER_GUIDE.md)**.

## Schedule (`TX_INTERVAL_S` = 240 s)

| UTC minute mod 4 | Mode  | Start delay | Approx. length | Dial examples (40 / 20 / 15 / 10 m) |
|-----------------:|-------|------------:|---------------:|-------------------------------------|
| :00 | FT8  | +500 ms | 12.64 s | 7.074 / 14.074 / 21.074 / 28.074 MHz |
| :01 | FT2  | +476 ms | 2.52 s  | 7.052 / 14.084 / 21.144 / 28.184 MHz |
| :02 | FT4  | +500 ms | 5.04 s  | 7.0475 / 14.080 / 21.140 / 28.180 MHz |
| :03 | JTTY | +500 ms | 1.888 s | 7.090 / 14.090 / 21.090 / 28.090 MHz |

Audio offset (lowest tone): random **800–2400 Hz** on every mode.

- FT8 / FT2 / FT4 message: `CQ <CALLSIGN> <LOCATOR>` (standard type-1 when `FT8_STANDARD_MSG=1`).
- JTTY message: callsign action `CQ <CALLSIGN> CQ` (one frame, EOM=1), tones in `jtty_tones.h`.

Decode: **MSHV** for FT8/FT2/FT4; **WSJT-X 3.3 beta** for JTTY  
(https://github.com/WSJTX/wsjtx/releases/tag/v3.3.0-beta4).

## Hardware & libraries

- Board: Arduino **Nano (ATmega328P)** WSPR-beacon style clone (Si5351 + GPS).
- Libraries: Etherkit **Si5351**, Etherkit **JTEncode**, **TinyGPSPlus**.
- Serial monitor: **9600** baud (firmware status / banner).

## Quick flash (Arduino IDE)

1. **Back up the firmware already on the board before you upload.** Flashing this sketch overwrites whatever is on the Nano (ATmega328P). Keep a copy of the previous sketch (`.ino`) or a known-good `.hex` if you want to restore it. If you know how, dump the current flash with `avrdude` and save that file before you upload. If you skip the backup, the old image is gone.
2. Install Arduino IDE; select board **Arduino Nano** (ATmega328P).
3. Install the three libraries above (Library Manager).
4. Open `FT8_Beacon_v4_9.ino`; set callsign/locator; regenerate JTTY if needed.
5. Tools → Port → your port (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux — placeholder only).
6. Upload. If you get “not in sync”, try **Tools → Processor → ATmega328P (Old Bootloader)** (57600) vs the newer bootloader (115200).

## Config knobs

All at the top of the `.ino`: bands, `ROTATE_BANDS`, offsets, `TX_INTERVAL_S`, start delays, GFSK on/off, GPS/PPS, Si5351 calibration (`SI5351_CORR_PPB`), `CAL_MODE` (carrier into dummy load only).

## Package contents

- `FT8_Beacon_v4_9.ino` — main sketch  
- `ft8_std.h` — standard FT8/FT4 CQ encoder  
- `jtty_tones.h` — precomputed JTTY tones for the configured call  
- `extras/encode_jtty.py` — regenerate JTTY tones  
- `USER_GUIDE.md`, `RELEASE_NOTES.md`, `LICENSE`

Sketch-only release (no PC companion tools).

## Licence

See `LICENSE`. Example callsign **QQ0QQ** is fictional and for documentation only.
