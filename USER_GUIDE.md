# FT8_Beacon v4.9 — User Guide

**EXPERIMENTAL SOFTWARE — use at your own risk.** There is no warranty. You alone are responsible for licence compliance, staying in allocated spectrum, and RF filtering.

This release is presently set up as a **quad-band, quad-mode** beacon: **FT8, FT4, FT2, and JTTY** on 40 / 20 / 15 / 10 m (see schedule below).

## 0. Do this before anything else

1. Open `FT8_Beacon_v4_9.ino` in a text editor or the Arduino IDE.
2. Near the top, change:
   ```c
   #define CALLSIGN          "N0TOK"   // <- your licensed call
   #define LOCATOR           "XX00"    // <- your 4-char Maidenhead grid
   ```
3. If you changed `CALLSIGN`, regenerate JTTY tones (JTTY does not read the macro at run time — tones are baked into `jtty_tones.h`):
   ```bash
   python3 extras/encode_jtty.py YOURCALL
   ```
   Run that from the sketch folder (or pass `--out jtty_tones.h`). Confirm the header comment says `CQ YOURCALL CQ`.
4. **Never transmit** with the example identity **N0TOK / XX00**. It is fictional and shipped only so the sketch compiles out of the box.

FT8 / FT4 / FT2 already use the `CALLSIGN` / `LOCATOR` macros, so after you edit them the on-air CQ becomes `CQ <yourcall> <yourgrid>`.

## 1. Licence, filtering, and bench safety

- A valid **amateur radio licence** is required to put RF on the air.
- These clone boards **typically have no onboard low-pass filter**. Fit a **proper external LPF for each band** before connecting an antenna. Without it you can radiate strong harmonics.
- Use a **50 Ω dummy load** for all first tests, calibration (`CAL_MODE`), and until you are sure timing and frequency look sane on a receiver/SDR.
- Stay inside band edges and power limits for your licence class and country.
- This project is **experimental**; treat dials (especially JTTY), start delays, and JTTY hard-FSK as subject to change.

## 2. Hardware

Typical target: AliExpress / Banggood **WSPR beacon** clone based on:

- Arduino **Nano**, ATmega328P @ 16 MHz  
- **Si5351** clock generator (+ TCXO on many boards)  
- GPS module (often ATGM336H-class) for UTC timing  

Confirm your pinout matches the sketch defaults (GPS on D3/D4, LEDs on D8/D9/D10, PPS on D2 if used). Clones vary.

### Libraries (Arduino Library Manager)

| Library | Typical use |
|---------|-------------|
| Etherkit Si5351 | RF clock |
| Etherkit JTEncode | FT8 free-text path / shared tables |
| TinyGPSPlus | NMEA parse |

## 3. How to flash with the Arduino IDE

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. **Tools → Board → Arduino AVR Boards → Arduino Nano**.
3. **Tools → Processor → ATmega328P**.  
   - If upload fails with “not in sync” / stk500 errors, try **ATmega328P (Old Bootloader)** (57600 baud bootloader) vs the newer Nano bootloader (115200).
4. Install the three libraries above (**Sketch → Include Library → Manage Libraries…**).
5. **File → Open…** and select `FT8_Beacon_v4_9.ino` (folder name should match the `.ino` name).
6. Set **CALLSIGN** / **LOCATOR**; regenerate JTTY if the call changed (section 0).
7. Plug in the board over USB. **Tools → Port** → choose the port (examples only: `COM3` on Windows, `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux/macOS). Your port number will differ.
8. Click **Upload**. Wait for “Done uploading”.
9. Open **Tools → Serial Monitor** at **9600** baud. You should see a banner including your call/locator and mode list, then GPS lock messages. **No TX until GPS is stable** (by design).

## 4. On-air schedule

Cycle length **240 s** (`TX_INTERVAL_S`):

| Slot (UTC) | Mode | Notes |
|------------|------|--------|
| :00, :04, :08… | FT8 | Standard CQ when `FT8_STANDARD_MSG=1` |
| :01, :05, :09… | FT2 | MSHV / Decodium-style dials |
| :02, :06, :10… | FT4 | |
| :03, :07, :11… | JTTY | Short ~1.9 s frame; dials 7090 / 14090 / 21090 / 28090 kHz |

Verify:

- FT8 / FT2 / FT4 → **MSHV** (or WSJT-X for FT8/FT4).  
- JTTY → **WSJT-X 3.3 beta**: https://github.com/WSJTX/wsjtx/releases/tag/v3.3.0-beta4  

Expected example decode after you set your own call: `CQ YOURCALL YOURGRID` (FT*) and `CQ YOURCALL CQ` (JTTY).

## 5. Config knobs (top of sketch)

| Knob | Role |
|------|------|
| `CALLSIGN` / `LOCATOR` | Beacon identity (edit first) |
| `ENABLE_FT4` / `ENABLE_FT2` / `ENABLE_JTTY` | Mode on/off |
| `BANDS_HZ` / `FT4_BANDS_HZ` / `FT2_BANDS_HZ` / `JTTY_BANDS_HZ` | Dial frequencies |
| `ROTATE_BANDS` / `FIXED_BAND` | Band hopping |
| `OFFSET_*_HZ` | Audio offset window (keep 800–2400) |
| `TX_INTERVAL_S` / `*_START_DELAY_MS` / `TX_START_TRIM_MS` | Timing |
| `USE_GFSK` | Gaussian shaping for FT8/FT4/FT2 (JTTY stays hard 4-FSK in v4.9) |
| `USE_PPS` / `NMEA_LATENCY_MS` | GPS timing |
| `SI5351_CORR_PPB` / `CAL_MODE` | Frequency calibration (**dummy load only** in CAL_MODE) |

Serial status: **9600** baud.

## 6. Regenerating JTTY tones

```bash
python3 extras/encode_jtty.py YOURCALL
# optional:
python3 extras/encode_jtty.py --call YOURCALL --out jtty_tones.h
```

The script runs CRC-12 and TBCC self-checks, then writes `jtty_tones.h`. Example packaged tones are for **N0TOK** only.

## 7. If something goes wrong

- **No GPS lock / no TX:** check antenna for GPS, baud 9600, wiring RX/TX not swapped, wait for “stable” messages.  
- **Upload not in sync:** Old Bootloader vs new; correct USB port; close any other app holding the serial port.  
- **Wrong frequency:** measure with SDR into dummy load; adjust `SI5351_CORR_PPB` / confirm `SI5351_XTAL_HZ`.  
- **Decodes as free text only:** ensure `FT8_STANDARD_MSG` is `1` for spottable standard CQ.  
- **JTTY won’t decode:** confirm WSJT-X 3.3 beta, dial, offset in 800–2400, and that `jtty_tones.h` matches your call.

Remember: **external LPF per band before antenna**, and **dummy load on the bench**.
