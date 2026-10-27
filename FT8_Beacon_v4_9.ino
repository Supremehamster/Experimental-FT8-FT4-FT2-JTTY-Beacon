/*
 * FT8_Beacon_v4_9.ino - GPS-timed FT8 + FT4 + FT2 + JTTY CQ beacon (originally developed as G0TTV beacon)
 * for the IgrikXD "WSPR-beacon 2.0" clone (AliExpress/Banggood "WSPR BEACON", ATmega328P @16 MHz,
 * Si5351 + TCXO, 74ACT244 buffer).
 *
 * Based on v4.2. Schedule (TX_INTERVAL_S=240):
 *   sod%240==0   FT8   (CQ <CALLSIGN> <LOCATOR>)
 *   sod%240==60  FT2   MSHV/Decodium dials
 *   sod%240==120 FT4
 *   sod%240==180 JTTY  (callsign action "CQ <CALLSIGN> CQ", PROGMEM tones; WSJT-X 3.3 beta)
 * Optional GFSK for FT8/FT4/FT2 (BT as WSJT-X). JTTY is hard 4-FSK for flash (doc'd; GFSK same BT=2.0 family).
 * JTTY: 4-GFSK protocol @31.25 baud, 59 symbols / 1.888 s; async mode but TX start scheduled in :03 quarter.
 *
 * EXPERIMENTAL SOFTWARE — use at your own risk. No warranty. You are responsible for
 * licence compliance, band edges, and RF filtering.
 *
 * !! BEFORE TX: set CALLSIGN and LOCATOR to your licensed call and grid. Example QQ0QQ/XX00 is fictional —
 * !! never transmit with the example identity on air. Regenerate jtty_tones.h if you change the call.
 *
 * !! The board most likely has NO low-pass filter (upstream PCB 2.0 has none). Fit an external
 * !! LPF for each band before connecting an antenna. Test into a dummy load first.
 *
 * Libraries: Etherkit Si5351 (tested 2.4.0), Etherkit JTEncode (1.3.1), TinyGPSPlus (1.0.3).
 * JTTY tones precomputed (extras/encode_jtty.py); no WSJT-X Fortran on the AVR.
 * Board: Arduino Nano (ATmega328P). Serial monitor 9600 baud.
 */
#include <Wire.h>
#include <SoftwareSerial.h>
#include <si5351.h>
#include <JTEncode.h>
#include <TinyGPS++.h>
#include "jtty_tones.h"

// ============================ USER CONFIG ============================
#define CALLSIGN          "QQ0QQ"
#define LOCATOR           "XX00"
// 1 = standard FT8 CQ (type 1, decoders parse call+grid, PSK Reporter spots it) - recommended.
// 0 = JTEncode ft8_encode() free text "CQ " CALLSIGN " " LOCATOR (decodes as text only; no spots). FT4 needs 1.
#define FT8_STANDARD_MSG  1
#define ENABLE_FT4        1        // 1 = also send FT4 half way between FT8 TXs (TX_INTERVAL_S/2 later)
#define ENABLE_FT2        1        // v4.2: 1 = also send one FT2 per TX_INTERVAL_S, TX_INTERVAL_S/4 after the FT8
#define ENABLE_JTTY       1        // v4.9: 1 = JTTY (WSJT-X 3.3) at 3*TX_INTERVAL_S/4 (spare :03 slot)

// FT8 dial frequencies (Hz). TX RF = dial + audio offset.
const uint32_t BANDS_HZ[]       = {7074000UL, 14074000UL, 21074000UL, 28074000UL};
// FT4 dial frequencies (Hz), same band order as BANDS_HZ / BAND_NAMES.
const uint32_t FT4_BANDS_HZ[]   = {7047500UL, 14080000UL, 21140000UL, 28180000UL};
// FT2 dial frequencies (Hz), same band order. MSHV 2.76.7 defaults (src/config_band_all.h, all_bands_mods_frq[],
// FT2 column: 7.052.000 / 14.084.000 / 21.144.000 / 28.184.000), so MSHV's band buttons land on the beacon's dial.
const uint32_t FT2_BANDS_HZ[]   = {7052000UL, 14084000UL, 21144000UL, 28184000UL};
// JTTY preliminary dials (USB Hz). jttyaf.app/jtty §8; 15m/10m = same xxx090 pattern as 40/20.
const uint32_t JTTY_BANDS_HZ[]  = {7090000UL, 14090000UL, 21090000UL, 28090000UL};
const char *const BAND_NAMES[]  = {"40m", "20m", "15m", "10m"};
#define NUM_BANDS         (sizeof(BANDS_HZ) / sizeof(BANDS_HZ[0]))
#define ROTATE_BANDS      1        // 1 = 40->20->15->10, each mode keeps its own rotation; 0 = always FIXED_BAND
#define FIXED_BAND        1        // index into band tables when ROTATE_BANDS == 0
#define TX_INTERVAL_S     240      // seconds between FT8 TX starts; multiple of 60 with FT2/JTTY, divides 86400.
                                   // FT8 at sod%240==0; FT2 at ==60; FT4 at ==120; JTTY at ==180
// v4.1: random offset (lowest tone) limited to 800-2400 Hz inclusive on every band, FT8 and FT4
#define OFFSET_MIN_HZ     800      // FT8 random audio offset range per TX (lowest tone)
#define OFFSET_MAX_HZ     2400     // FT8 top tone = offset + 43.75 Hz, must stay < 2800 Hz
#define FT4_OFFSET_MIN_HZ 800      // FT4 random audio offset range per TX (lowest tone)
#define FT4_OFFSET_MAX_HZ 2400     // FT4 top tone = offset + 62.5 Hz; signal width 83.3 Hz (4 x 20.833) <= 2800 Hz
#define FT2_OFFSET_MIN_HZ 800      // FT2 random audio offset range per TX (lowest tone)
#define FT2_OFFSET_MAX_HZ 2400     // FT2 top tone = offset + 125 Hz; signal width 166.7 Hz (4 x 41.667) <= 2800 Hz
#define JTTY_OFFSET_MIN_HZ 800     // JTTY: offset = lowest tone f0; tones at f0+k*31.25 Hz, k=0..3
#define JTTY_OFFSET_MAX_HZ 2400    // top tone = offset + 93.75 Hz; BW ~127 Hz
#define TX_START_DELAY_MS 500      // FT8 nominal start: 0.5 s into the 15 s slot
#define FT4_START_DELAY_MS 500     // FT4 start into its 7.5 s slot. 500 = DT 0 in the WSJT-X FT4 decoder (it reports
                                   // DT = start - 0.5 s). WSJT-X's own FT4 TX starts at 300 ms (Modulator.cpp)
#define FT2_START_DELAY_MS 476     // FT2 start (first ramp symbol) into its 3.75 s slot. Same idea as FT4: chosen so the
                                   // receiving decoder shows DT 0. MSHV 2.76.7's FT2 decoder reports DT = (first sync
                                   // symbol) - 0.5 s, i.e. start + 24 ms ramp - 0.5 s (measured offline with MSHV's own
                                   // decoder, see README). MSHV's own FT2 TX starts its audio at 350 ms
                                   // (msplayerhv.cpp) + sound card latency; PC programs use ~100-150 ms. Ends at 2.996 s,
                                   // before MSHV starts its FT2 decode at 3.4 s into the slot (main_ms.cpp).
#define JTTY_START_DELAY_MS 500    // JTTY is async (no UTC slot), but we still start +0.5 s into the :03 quarter
                                   // so the short 1.888 s frame sits cleanly after the minute edge (same 0.5 s
                                   // habit as FT8/FT4). Documented choice — not a protocol requirement.
#define TX_START_TRIM_MS  0        // signed fine trim (ms) added to the TX start. If MSHV/WSJT-X (PC clock
                                   // known good) shows DT = +0.15 s, use -150; DT = -0.10 s -> +100. Applies to FT8, FT4, FT2, JTTY

// Modulation
#define USE_GFSK          1        // 1 = Gaussian-shaped tone changes (FT8: BT=2.0, 16 x 10 ms steps/symbol;
                                   //     FT4: BT=1.0, 8 x 6 ms steps/symbol; FT2: BT=1.0, 8 x 3 ms steps/symbol);
                                   // 0 = plain FSK (hard-keyed tones, one Si5351 write per symbol).
                                   // JTTY always hard 4-FSK in v4.9 (flash); protocol is 4-GFSK BT=2.0.

// Timing
#define USE_PPS           1        // use GPS 1PPS on PPS_PIN if pulses are seen (auto fallback to NMEA)
#define PPS_PIN           2        // D2/INT0: upstream 1.1 routes ATGM336H 1PPS here - UNVERIFIED on clone
#define GPS_STABLE_COUNT  10       // TX only after this many consecutive valid NMEA seconds, each exactly +1 s
#define NMEA_LATENCY_MS   0        // NMEA-only mode (no PPS): ms from true second edge to the START of the
                                   // NMEA burst. Tune from observed DT: NMEA_LATENCY_MS += DT*1000
#define NMEA_GAP_MS       20       // GPS line idle this long = idle gap between NMEA bursts (line not saturated)
#define NMEA_MAX_LAG_MS   900      // PPS mode: NMEA burst must start within this time after its PPS edge
#define GPS_SEND_PCAS     1        // 1 = at start-up send "$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02" so the ATGM336H
                                   // outputs only GGA+RMC (RAM only, NOT saved; lost at power-off). Its factory
                                   // output (GGA,GLL,GSA,GSV..,RMC,VTG,ZDA,TXT) can exceed what 9600 baud
                                   // carries, so NMEA falls >1 s behind PPS. Needs MCU GPS_TX_PIN -> GPS RXD.

// Si5351
#define SI5351_I2C_ADDR   0x60     // from upstream 1.1 firmware, unverified on clone
#define SI5351_XTAL_HZ    25000000UL  // UNKNOWN - CONFIRM TCXO marking (upstream BOM: 25 MHz TCXO)
#define SI5351_CORR_PPB   0L       // calibration, parts per billion. TX measured HIGH by e Hz at f MHz -> add +e*1000/f
#define SI5351_LOAD       SI5351_CRYSTAL_LOAD_8PF   // as upstream (irrelevant for a TCXO driving XA)
#define SI5351_DRIVE      SI5351_DRIVE_8MA          // as upstream fw 1.1

// Calibration: 1 = ignore GPS and output a steady carrier at CAL_FREQ_HZ forever (DUMMY LOAD ONLY),
// measure it with a counter/SDR and set SI5351_CORR_PPB. 0 = normal beacon.
#define CAL_MODE          0
#define CAL_FREQ_HZ       10000000UL

// Pins - all from upstream 1.1 firmware (also listed in the BG6JJI clone manual), unverified on clone PCB
#define GPS_RX_PIN        4        // MCU RX <- GPS TXD
#define GPS_TX_PIN        3        // MCU TX -> GPS RXD
#define GPS_BAUD          9600
#define TX_LED_PIN        8        // red
#define GPS_LED_PIN       9        // green
#define PWR_LED_PIN       10       // green
// =====================================================================
#if FT8_STANDARD_MSG
#include "ft8_std.h"
#endif

#if ENABLE_FT2 || ENABLE_JTTY
#define TX_SLOT_S     (TX_INTERVAL_S / 4)        // scheduler: FT8, FT2, FT4, JTTY, FT8 ...
#elif ENABLE_FT4
#define TX_SLOT_S     (TX_INTERVAL_S / 2)        // scheduler period: FT8, FT4, FT8, FT4 ...
#else
#define TX_SLOT_S     TX_INTERVAL_S
#endif
static_assert(TX_INTERVAL_S % 15 == 0 && 86400UL % TX_INTERVAL_S == 0, "TX_INTERVAL_S must be a multiple of 15 dividing 86400");
static_assert(TX_START_DELAY_MS + TX_START_TRIM_MS >= 0 && TX_START_DELAY_MS + TX_START_TRIM_MS <= 2500, "TX start out of range");
static_assert(OFFSET_MIN_HZ >= 100 && OFFSET_MAX_HZ + 44 <= 2800 && OFFSET_MIN_HZ <= OFFSET_MAX_HZ, "offset range");
static_assert(sizeof(FT4_BANDS_HZ) == sizeof(BANDS_HZ) && sizeof(BAND_NAMES) / sizeof(BAND_NAMES[0]) == NUM_BANDS, "band tables differ in length");
static_assert(FIXED_BAND < NUM_BANDS, "FIXED_BAND out of range");
#if ENABLE_FT4
static_assert(FT8_STANDARD_MSG, "FT4 needs FT8_STANDARD_MSG 1 (JTEncode 1.3.1 has no FT4 encoder)");
// FT4 slot start must also be a 15 s (hence 7.5 s) boundary, and leave >= 45 s per TX for TX + GPS re-sync
static_assert(TX_INTERVAL_S % 30 == 0 && TX_INTERVAL_S >= 90, "with ENABLE_FT4, TX_INTERVAL_S must be a multiple of 30, >= 90");
static_assert(FT4_START_DELAY_MS + TX_START_TRIM_MS >= 0 && FT4_START_DELAY_MS + TX_START_TRIM_MS + 5040 <= 7500,
              "FT4 TX (105 x 48 ms = 5.04 s) must start and end inside its 7.5 s slot");
// 4 tones x 20.833 Hz = 83.3 Hz occupied (lowest tone at offset, top tone at offset + 62.5 Hz)
static_assert(FT4_OFFSET_MIN_HZ >= 100 && FT4_OFFSET_MAX_HZ + 84 <= 2800 && FT4_OFFSET_MIN_HZ <= FT4_OFFSET_MAX_HZ, "FT4 offset range");
#endif
static_assert(sizeof(FT2_BANDS_HZ) == sizeof(BANDS_HZ), "FT2 band table differs in length");
static_assert(sizeof(JTTY_BANDS_HZ) == sizeof(BANDS_HZ), "JTTY band table differs in length");
#if ENABLE_FT2 || ENABLE_JTTY
static_assert(TX_INTERVAL_S % 60 == 0 && TX_INTERVAL_S >= 120, "with FT2/JTTY, TX_INTERVAL_S must be a multiple of 60, >= 120");
static_assert(TX_SLOT_S * 4 == TX_INTERVAL_S, "four quarter slots");
#endif
#if ENABLE_FT2
static_assert(FT8_STANDARD_MSG, "FT2 needs FT8_STANDARD_MSG 1 (FT2 uses the FT4 encoder in ft8_std.h)");
static_assert((TX_SLOT_S * 1000UL) % 3750UL == 0, "FT2 slot not on a 3.75 s boundary");
static_assert(FT2_START_DELAY_MS + TX_START_TRIM_MS >= 0 && FT2_START_DELAY_MS + TX_START_TRIM_MS + 2520 <= 3400,
              "FT2 TX (105 x 24 ms = 2.52 s) must start in its 3.75 s slot and end before MSHV decodes at 3.4 s");
static_assert(FT2_OFFSET_MIN_HZ >= 100 && FT2_OFFSET_MAX_HZ + 167 <= 2800 && FT2_OFFSET_MIN_HZ <= FT2_OFFSET_MAX_HZ, "FT2 offset range");
#endif
#if ENABLE_JTTY
// JTTY TX = 59 * 32 ms = 1.888 s; start +0.5 s; must finish inside 60 s quarter
static_assert(JTTY_START_DELAY_MS + TX_START_TRIM_MS >= 0 && JTTY_START_DELAY_MS + TX_START_TRIM_MS + 1888 <= 60000,
              "JTTY TX (1.888 s) must start and end inside its 60 s quarter");
static_assert(JTTY_OFFSET_MIN_HZ >= 100 && JTTY_OFFSET_MAX_HZ + 94 <= 2800 && JTTY_OFFSET_MIN_HZ <= JTTY_OFFSET_MAX_HZ, "JTTY offset range");
#endif

#define MODE_FT8      0
#define MODE_FT4      1
#define MODE_FT2      2
#define MODE_JTTY     3                          // JTTY (WSJT-X 3.3); serial token JTTY
#define MODE_NONE     0xFF                       // scheduler slot without a TX
#define FT8_SYMBOLS   79                         // 7 sync + 29 data + 7 sync + 29 data + 7 sync
#define FT4_SYMBOLS   105                        // ramp + 4 x 4 sync + 87 data + ramp
#define FT2_SYMBOLS   105                        // FT2 = the FT4 frame: ramp + 103 FT4 channel symbols + ramp
#define JTTY_SYMBOLS  JTTY_SYMBOL_COUNT          // 59 = 13 sync + 46 data
#define FT8_SYMBOL_US 160000UL
#define FT4_SYMBOL_US 48000UL                    // 576 samples at 12 kHz
#define FT2_SYMBOL_US 24000UL                    // 288 samples at 12 kHz
#define JTTY_SYMBOL_US 32000UL                   // 384 samples at 12 kHz = 32 ms exact
#if USE_GFSK
#define FT8_STEP_SHIFT 4                         // 16 sub-steps of 10 ms
#define FT4_STEP_SHIFT 3                         // 8 sub-steps of 6 ms (uses the second half of GFSK_W, see below)
#define FT2_STEP_SHIFT 3                         // 8 sub-steps of 3 ms (BT=1.0 like FT4: same GFSK_W[8..15] weights)
#else
#define FT8_STEP_SHIFT 0
#define FT4_STEP_SHIFT 0
#define FT2_STEP_SHIFT 0                         // hard-keyed: one write per 24 ms symbol
#endif
#define FT8_TONE_D256 1600                       // 6.25 Hz in 1/256 Hz units (exact)
#define FT4_TONE_D256 5333                       // 20.8333 Hz (12000/576) in 1/256 Hz; rounding -0.0013 Hz/tone
// FT2 tone spacing 12000/288 = 41.6667 Hz = 32000/3 in 1/256 Hz units. Not rounded: stepDev() multiplies by 32000
// and divides by 3 (x 32768 for the GFSK Q15 weights), so every tone/GFSK step is exact to 1/256 Hz (0.004 Hz).
#define FT2_TONE_NUM  32000UL
#define FT2_TONE_DEN  3UL
#define FT2_SPAN_D256 (3 * FT2_TONE_NUM / FT2_TONE_DEN)   // 3 x 41.6667 Hz = 125 Hz = 32000/256 exactly
// JTTY tone spacing 31.25 Hz = 8000/256 exactly. Hard FSK in v4.9 (no GFSK steps).
#define JTTY_TONE_D256 8000                      // 31.25 Hz in 1/256 Hz units
#define JTTY_SPAN_D256 (3UL * JTTY_TONE_D256)    // tone 0..3 span = 93.75 Hz
static_assert((FT8_SYMBOL_US >> FT8_STEP_SHIFT) * (1UL << FT8_STEP_SHIFT) == FT8_SYMBOL_US &&
              (FT4_SYMBOL_US >> FT4_STEP_SHIFT) * (1UL << FT4_STEP_SHIFT) == FT4_SYMBOL_US &&
              (FT2_SYMBOL_US >> FT2_STEP_SHIFT) * (1UL << FT2_STEP_SHIFT) == FT2_SYMBOL_US, "step length not exact");
static_assert(3UL * 32768UL * FT2_TONE_NUM < 4294967296ULL, "FT2 GFSK accumulator overflows 32 bits");
#if ENABLE_FT4 || ENABLE_FT2
#define MAX_SYMBOLS   FT4_SYMBOLS
#else
#define MAX_SYMBOLS   FT8_SYMBOLS
#endif

// GFSK weight (Q15) of the NEXT symbol's tone at sub-step s (centre of each step),
// from WSJT-X gfsk_pulse(); p(t)=0.5*(erf(k*BT*(t+0.5))-erf(k*BT*(t-0.5))), k=pi*sqrt(2/ln2), normalised.
// FT8 (BT=2.0, 16 steps): next weight = GFSK_W[s], previous weight = GFSK_W[15-s].
// FT4 and FT2 (BT=1.0, 8 steps of 1/8 symbol): next = GFSK_W[8+s], previous = GFSK_W[15-s]. Computed separately
// it is {0,0,0,0,16,301,2573,10439} = GFSK_W[8..15]: the weight depends on BT x (distance to the symbol
// boundary) - FT4 step s sits at BT*d = (s+0.5)/8 - 1, the same as FT8 step 8+s at 2*((8+s+0.5)/16 - 1) -
// and the pulse's far erf term is saturated (difference < 1e-10 of Q15). See extras/verification.txt.
const uint16_t GFSK_W[16] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 301, 2573, 10439};

Si5351 si5351(SI5351_I2C_ADDR);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
uint8_t tones[MAX_SYMBOLS];   // tones of the current TX (re-encoded for its mode just before each TX)
// current TX mode parameters (set by setMode())
uint8_t mMode, mSyms, mShift, mWOff; uint16_t mToneD256; uint32_t mStepUs;

// --- time keeping: UTC second-of-day refSod began at micros() == refUs ---
volatile uint32_t ppsUs = 0, ppsCount = 0;
// NMEA burst tracking: the GPS sends one burst of sentences per second, after that second's PPS edge.
// ATGM336H sends GGA first in each burst, so the '$' of GGA marks the burst start; each '$'
// snapshots the latest PPS edge. Idle gaps between bursts show the line is not saturated.
uint32_t lastCharUs = 0;
uint32_t sentUs = 0, sentPpsUs = 0, sentPpsCnt = 0;   // '$' of the sentence being parsed
bool burstFresh = false;   // an idle gap (new burst) was seen since the last reading
bool pendingTiming = false; // this second's reading had no timing yet (no GGA)
uint8_t noGapCount = 0;    // consecutive readings with no idle gap on the GPS line (= line saturated)
uint16_t ppsLagMs = 0;     // PPS edge -> start of NMEA burst, last reading
uint32_t refUs = 0, refSod = 0, lastSyncUs = 0, lastPpsCountUsed = 0, lastSecUs = 0;
#define SOD_NONE   -1      // no previous reading
#define SOD_RESUME -2      // previous reading was before a TX (GPS RX paused); next reading is checked against the old reference
int32_t lastSod = SOD_NONE, lastTxSlot = -1;
bool timeValid = false, ppsLocked = false;
bool gpsStable = false;    // TX allowed only when true
uint8_t stableCount = 0;   // consecutive valid +1 s NMEA readings
uint16_t txCount[4] = {0, 0, 0, 0}; // per mode (FT8, FT4, FT2, JTTY): each rotates independently

void ppsIsr() { ppsUs = micros(); ppsCount++; }

// Any GPS glitch: restart the stable-lock count; if we were locked, drop back to acquiring.
void gpsGlitch(const __FlashStringHelper *why) {
  if (stableCount || gpsStable) { Serial.print(F("GPS glitch (")); Serial.print(why); Serial.println(F(") - stable count reset")); }
  stableCount = 0;
  timeValid = false;
  if (gpsStable) { gpsStable = false; Serial.println(F("GPS lock LOST - TX disabled, re-acquiring")); }
}

// Called after every complete NMEA sentence. The first sentence carrying a new second is one
// "reading": it must have a fix (this sentence), valid time+date, and be exactly +1 s after the
// previous reading (sod arithmetic handles minute/hour/midnight rollover).
void onNmea(uint32_t nowUs) {
  if (!gps.time.isUpdated()) return;                         // sentence without time
  bool fixNow = gps.location.isUpdated() && gps.location.isValid();  // TinyGPS commits location only on fix
  gps.location.rawLat();                                     // clear 'updated' flag
  int32_t sod = (int32_t)gps.time.hour() * 3600L + gps.time.minute() * 60 + gps.time.second();
  if (sod == lastSod) {                                      // 2nd sentence (GGA/RMC) of same second
    bool gga = gps.satellites.isUpdated(); gps.satellites.value();   // always clear the GGA flag
#if USE_PPS
    // GGA came after RMC this second (other module order / RMC first): take the timing from it now
    if (pendingTiming && gga) {
      pendingTiming = false;
      noInterrupts(); uint32_t pCnt = ppsCount; interrupts();
      if (pCnt && sentPpsCnt && sentPpsCnt != lastPpsCountUsed && sentUs - sentPpsUs < NMEA_MAX_LAG_MS * 1000UL) {
        refUs = sentPpsUs; lastPpsCountUsed = sentPpsCnt; ppsLocked = true;
        refSod = sod; lastSyncUs = nowUs; timeValid = gpsStable;
      }
    }
#endif
    return;
  }
  pendingTiming = false;
  bool resume = (lastSod == SOD_RESUME);
  // date comes only from RMC; after a TX pause the last RMC is ~13 s old
  bool valid = fixNow && gps.time.isValid() && gps.date.isValid() && gps.date.age() < (resume ? 20000UL : 2500UL);
  bool consecutive = lastSod >= 0 && sod == (lastSod + 1) % 86400L;
  lastSod = sod;
  lastSecUs = nowUs;

  // Which PPS edge does this second label belong to? The edge that PRECEDED THE START of this
  // NMEA burst - not simply the latest edge: the old code took the latest edge, so when the time
  // sentence of second N arrived after edge N+1 (long/backlogged burst) the clock ran 1 s late.
  bool isGga = gps.satellites.isUpdated();                   // only GGA commits satellites
  gps.satellites.value();                                    // clear 'updated' flag
  bool fresh = burstFresh, ppsOk = false, ppsAlive = false, haveStart = false, lagBad = false;
  uint32_t startUs = 0, edgeUs = 0, edgeCnt = 0;
  burstFresh = false;
  // GGA is the first sentence of every ATGM336H burst: its '$' time is the burst start. If GGA was
  // lost (checksum/overflow) this second gives no timing; the previous PPS reference is kept.
  if (isGga) { startUs = sentUs; edgeUs = sentPpsUs; edgeCnt = sentPpsCnt; haveStart = true; }
  noGapCount = fresh ? 0 : (noGapCount < 255 ? noGapCount + 1 : 255);
#if USE_PPS
  noInterrupts(); uint32_t pUs = ppsUs, pCnt = ppsCount; interrupts();
  ppsAlive = pCnt && (nowUs - pUs) < 2000000UL;
  if (ppsAlive && haveStart && edgeCnt && edgeCnt != lastPpsCountUsed) {
    if (startUs - edgeUs < NMEA_MAX_LAG_MS * 1000UL) { ppsOk = true; ppsLagMs = (startUs - edgeUs) / 1000UL; }
    else lagBad = true;
  }
#endif
  if (!valid) { gpsGlitch(F("no valid fix/time/date")); return; }
  if (resume) {
    // first reading after a TX (GPS RX paused): must match the time predicted from the old reference
    if ((uint32_t)sod != (refSod + (nowUs - refUs) / 1000000UL) % 86400UL) { gpsGlitch(F("time jump across TX")); return; }
  } else if (!consecutive) {
    gpsGlitch(F("time step not +1 s")); return;
  } else if (lagBad) {
    gpsGlitch(F("NMEA >900 ms after PPS - GPS output too long for 9600 bd?")); return;
  } else if (ppsAlive && noGapCount >= 3) {
    // no idle time on the line for 3 s: the module is sending more than 9600 bd carries, so its
    // NMEA falls behind PPS by an unknown number of seconds - refuse rather than TX late
    gpsGlitch(F("GPS line saturated (no idle gap) - NMEA may lag PPS by >1 s")); return;
  } else if (stableCount < 255) {
    stableCount++;
  }
  if (!gpsStable) {
    Serial.print(F("GPS stable ")); Serial.print(stableCount); Serial.print('/'); Serial.print(GPS_STABLE_COUNT);
    if (ppsOk) { Serial.print(F(" (PPS->NMEA ")); Serial.print(ppsLagMs); Serial.print(F(" ms)")); }
    Serial.println();
    if (stableCount >= GPS_STABLE_COUNT) { gpsStable = true; Serial.println(F("GPS lock stable - TX enabled")); }
  }
  if (ppsOk) {
    refUs = edgeUs; lastPpsCountUsed = edgeCnt; ppsLocked = true;
  } else if (!ppsAlive && haveStart) {   // no PPS at all: NMEA-only timing from the burst start
    refUs = startUs - NMEA_LATENCY_MS * 1000UL - gps.time.centisecond() * 10000UL;
    ppsLocked = false;
  } else {
    pendingTiming = true;   // no GGA yet this second: a later GGA of the same second may supply timing
    return;                 // otherwise keep the previous reference
  }
  refSod = sod; lastSyncUs = nowUs; timeValid = gpsStable;
}

// ---------------- Si5351 fast tuning (PLL fixed at 800 MHz, MS0 fractional) ----------------
// set_freq() costs 3 I2C reads + 3 writes + 64-bit maths per call. For 10 ms (FT8) / 6 ms (FT4) / 3 ms (FT2)
// GFSK steps we precompute MS0 = a + b/c once per TX and then only change b (linearised, error
// < 0.003 Hz over the 44 Hz FT8 / 62.5 Hz FT4 / 125 Hz FT2 tone span: ~span^2/f0), writing the 8 MS0 registers
// in a single 400 kHz I2C burst.
#define PLL_HZ   800000000ULL   // library default PLLA (SI5351_PLL_FIXED), correction applied in PLL
#define MS_C     1048575UL      // max denominator -> finest steps
struct MsPlan { uint32_t a, b0, k12; } plan;
uint32_t lastB = 0xFFFFFFFF;

// span256 = lowest-to-top tone span in 1/256 Hz (FT8 7 x 6.25 = 43.75 Hz, FT4 3 x 20.833 = 62.5 Hz,
// FT2 3 x 41.667 = 125 Hz). Returns false if that span would cross an integer divider (then pick another offset).
// k12 * span256 < 2^32: k12 <= ~69200 at 7.0475 MHz, x 16000 (FT4) = 1.1e9, x 32000 (FT2) = 2.2e9.
bool planTx(uint32_t f0, uint32_t span256) {
  plan.a = PLL_HZ / f0;
  uint64_t rem = PLL_HZ - (uint64_t)plan.a * f0;
  plan.b0 = (uint32_t)((rem * MS_C + f0 / 2) / f0);
  plan.k12 = (uint32_t)(((uint64_t)MS_C * PLL_HZ * 4096ULL / f0) / f0);   // MS counts per Hz, Q12
  uint32_t maxDrop = (uint32_t)(((uint64_t)plan.k12 * span256) >> 20) + 2;
  return plan.b0 > maxDrop && plan.b0 < MS_C;
}

// d256 = deviation above f0 in 1/256 Hz
void writeMs0(uint32_t d256) {
  uint32_t b = plan.b0 - ((plan.k12 * d256 + (1UL << 19)) >> 20);
  if (b == lastB) return;
  lastB = b;
  uint32_t q = (128UL * b) / MS_C;
  uint32_t p1 = 128UL * plan.a + q - 512;
  uint32_t p2 = 128UL * b - MS_C * q;
  uint32_t p3 = MS_C;
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(42);                                   // MS0 parameters
  Wire.write((uint8_t)(p3 >> 8));  Wire.write((uint8_t)p3);
  Wire.write((uint8_t)((p1 >> 16) & 0x03));         // R0_DIV=1, DIVBY4 off
  Wire.write((uint8_t)(p1 >> 8));  Wire.write((uint8_t)p1);
  Wire.write((uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)));
  Wire.write((uint8_t)(p2 >> 8));  Wire.write((uint8_t)p2);
  Wire.endTransmission();
}

void setMode(uint8_t mode) {
  mMode = mode;
  if (mode == MODE_JTTY) {
    // JTTY hard FSK: one write per 32 ms symbol (GFSK BT=2.0 deferred to save flash)
    mSyms = JTTY_SYMBOLS; mShift = 0; mWOff = 0;
    mToneD256 = JTTY_TONE_D256; mStepUs = JTTY_SYMBOL_US;
    return;
  }
  bool ft4 = (mode == MODE_FT4), ft2 = (mode == MODE_FT2);
  mSyms     = ft2 ? FT2_SYMBOLS : (ft4 ? FT4_SYMBOLS : FT8_SYMBOLS);
  mShift    = ft2 ? FT2_STEP_SHIFT : (ft4 ? FT4_STEP_SHIFT : FT8_STEP_SHIFT);
  mWOff     = 16 - (1 << mShift);                 // FT8: GFSK_W[0..15]; FT4, FT2: GFSK_W[8..15]
  mToneD256 = ft4 ? FT4_TONE_D256 : FT8_TONE_D256;   // FT2 does not use it: exact FT2_TONE_NUM / FT2_TONE_DEN
  mStepUs   = (ft2 ? FT2_SYMBOL_US : (ft4 ? FT4_SYMBOL_US : FT8_SYMBOL_US)) >> mShift;   // 10000 / 6000 / 3000 us
}

// deviation (1/256 Hz) for global step n (0 .. mSyms << mShift - 1) of the current mode
uint32_t stepDev(uint16_t n) {
  uint8_t sym = n >> mShift;
  if (mMode == MODE_JTTY) return (uint32_t)tones[sym] * JTTY_TONE_D256;  // hard FSK, 31.25 Hz
#if USE_GFSK
  uint8_t s = n & ((1 << mShift) - 1);
  // first symbol: previous = itself; last symbol: next = itself (WSJT-X dummy symbols)
  uint8_t prev = tones[sym ? sym - 1 : 0], cur = tones[sym], next = tones[sym < mSyms - 1 ? sym + 1 : sym];
  uint32_t wn = pgm_read_word(&GFSK_W[mWOff + s]), wp = pgm_read_word(&GFSK_W[15 - s]);
  uint32_t acc = prev * wp + next * wn + cur * (32768UL - wp - wn);     // Q15 tone index
  if (mMode == MODE_FT2)                                               // acc * (32000/3) / 32768, rounded; acc <= 3*32768
    return (acc * FT2_TONE_NUM + FT2_TONE_DEN * 16384UL) / (FT2_TONE_DEN * 32768UL);
  return (acc * mToneD256 + 16384) >> 15;                              // acc <= 3*32768 (FT4) / 7*32768 (FT8)
#else
  if (mMode == MODE_FT2) return ((uint32_t)tones[sym] * FT2_TONE_NUM + FT2_TONE_DEN / 2) / FT2_TONE_DEN;
  return (uint32_t)tones[sym] * mToneD256;
#endif
}

// Fill tones[] for the given mode (deterministic; ~4 ms on the AVR, done before the TX wait).
bool encodeTones(uint8_t mode) {
#if ENABLE_JTTY
  if (mode == MODE_JTTY) {
    for (uint8_t i = 0; i < JTTY_SYMBOLS; i++) tones[i] = pgm_read_byte(&JTTY_TONES[i]);
    return true;
  }
#endif
#if FT8_STANDARD_MSG
  return ft8std::encodeCQ(CALLSIGN, LOCATOR, tones, mode != MODE_FT8);   // FT2 sends exactly the FT4 symbols
#else
  (void)mode;
  JTEncode enc;
  enc.ft8_encode("CQ " CALLSIGN " " LOCATOR, tones);   // free text, max 13 chars (FT8 only)
  return true;
#endif
}

void waitUntil(uint32_t tUs) { while ((int32_t)(micros() - tUs) < 0) {} }

const __FlashStringHelper *modeName(uint8_t mode) {
  if (mode == MODE_JTTY) return F("JTTY");
  if (mode == MODE_FT2)  return F("FT2");
  if (mode == MODE_FT4)  return F("FT4");
  return F("FT8");
}

// Transmit one FT8, FT4 or FT2 frame whose first symbol starts at micros() == startUs.
// slotEdgeUs = predicted micros() of the slot's PPS edge (for the timing report).
// Every step n is due at startUs + n * mStepUs (absolute, from the one start time), never by adding delays,
// so loop overheads do not accumulate; the start itself is referenced to the PPS edge (see loop()).
void transmit(uint8_t mode, uint8_t band, uint32_t startUs, uint32_t slotEdgeUs) {
  encodeTones(mode);                   // validated in setup()
  setMode(mode);
  uint32_t dial, span256;
  if (mode == MODE_JTTY)      { dial = JTTY_BANDS_HZ[band]; span256 = JTTY_SPAN_D256; }
  else if (mode == MODE_FT2)  { dial = FT2_BANDS_HZ[band];  span256 = FT2_SPAN_D256; }
  else if (mode == MODE_FT4)  { dial = FT4_BANDS_HZ[band];  span256 = 3UL * mToneD256; }
  else                        { dial = BANDS_HZ[band];      span256 = 7UL * mToneD256; }
  uint32_t offset, f0;
  do {   // re-draw if the tone span would cross an integer MS0 divider (planTx() == false)
    if (mode == MODE_JTTY) offset = random(JTTY_OFFSET_MIN_HZ, JTTY_OFFSET_MAX_HZ + 1);
    else if (mode == MODE_FT2) offset = random(FT2_OFFSET_MIN_HZ, FT2_OFFSET_MAX_HZ + 1);
    else if (mode == MODE_FT4) offset = random(FT4_OFFSET_MIN_HZ, FT4_OFFSET_MAX_HZ + 1);
    else offset = random(OFFSET_MIN_HZ, OFFSET_MAX_HZ + 1);
    f0 = dial + offset;
  } while (!planTx(f0, span256));

  Serial.print(F("TX ")); Serial.print(modeName(mode)); Serial.print(' '); Serial.print(BAND_NAMES[band]);
  Serial.print(F(" dial ")); Serial.print(dial);
  Serial.print(F(" offset ")); Serial.print(offset);
  Serial.print(F(" Hz RF ")); Serial.print(f0); Serial.print(F(" Hz"));
  Serial.println(ppsLocked ? F(" [PPS]") : F(" [NMEA]"));
  Serial.flush();

  si5351.set_freq((uint64_t)f0 * SI5351_FREQ_MULT, SI5351_CLK0);  // sets R div / frac mode; PLL stays 800 MHz
  si5351.output_enable(SI5351_CLK0, 0);
  lastB = 0xFFFFFFFF;
  writeMs0(stepDev(0));
  si5351.set_clock_pwr(SI5351_CLK0, 1);

  gpsSerial.stopListening();          // SoftwareSerial RX ISR would add ~1 ms jitter per char
  const uint16_t nSteps = (uint16_t)mSyms << mShift;   // FT8 1264 x 10 ms, FT4 840 x 6 ms, FT2 840 x 3 ms
  waitUntil(startUs);
  si5351.output_enable(SI5351_CLK0, 1);
  uint32_t rfOnUs = micros();
  noInterrupts(); uint32_t edgeUs = ppsUs; interrupts();   // PPS ISR keeps running during the wait
  digitalWrite(TX_LED_PIN, HIGH);
  uint32_t worstUs = 0;
  for (uint16_t n = 1; n < nSteps; n++) {
    uint32_t d = stepDev(n);
    uint32_t due = startUs + n * mStepUs;
    waitUntil(due);
    writeMs0(d);
    uint32_t took = micros() - due;
    if (took > worstUs) worstUs = took;
  }
  waitUntil(startUs + nSteps * mStepUs);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  digitalWrite(TX_LED_PIN, LOW);
  gpsSerial.listen();
  lastCharUs = micros();              // a burst already in progress is not a burst start
  timeValid = false;                  // force fresh GPS sync before the next TX
  lastSod = SOD_RESUME;               // GPS RX was paused ~13 s (FT4 ~5.5 s, FT2 ~3 s): next reading continues the lock
  lastSecUs = micros();
  Serial.print(F("TX done, worst step latency ")); Serial.print(worstUs); Serial.println(F(" us"));
  // Timing report. With PPS: last PPS edge before RF-on -> first symbol; expect (FT4_/FT2_)TX_START_DELAY_MS+TRIM.
  Serial.print(F("Slot PPS edge -> first symbol: "));
  if (ppsLocked && rfOnUs - edgeUs < 1000000UL) {
    Serial.print((long)((rfOnUs - edgeUs) / 1000UL)); Serial.print(F(" ms (edge vs predicted "));
    Serial.print((long)((int32_t)(edgeUs - slotEdgeUs) / 1000L)); Serial.println(F(" ms)"));
  } else {
    Serial.print((long)((int32_t)(rfOnUs - slotEdgeUs) / 1000L)); Serial.println(F(" ms (no PPS edge in last 1 s: vs reference estimate)"));
  }
}

// Mode of the TX starting at UTC second-of-day 'slot' (a multiple of TX_SLOT_S):
// FT8 at 0, FT2 at 1/4, FT4 at 1/2, JTTY at 3/4 (spare :03). Else MODE_NONE.
uint8_t txMode(uint32_t slot) {
  uint32_t r = slot % TX_INTERVAL_S;
  if (r == 0) return MODE_FT8;
#if ENABLE_FT4
  if (r == TX_INTERVAL_S / 2) return MODE_FT4;
#endif
#if ENABLE_FT2
  if (r == TX_INTERVAL_S / 4) return MODE_FT2;
#endif
#if ENABLE_JTTY
  if (r == 3UL * TX_INTERVAL_S / 4) return MODE_JTTY;
#endif
  return MODE_NONE;
}

void printStatus() {
  Serial.print(F("GPS sats=")); Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  if (!gpsStable) {
    Serial.print(F(" acquiring, stable ")); Serial.print(stableCount); Serial.print('/');
    Serial.print(GPS_STABLE_COUNT); Serial.println(F(" - no TX")); return;
  }
  if (!timeValid) { Serial.println(F(" locked, waiting for next NMEA second")); return; }
  uint32_t sod = (refSod + (micros() - refUs) / 1000000UL) % 86400UL;
  char buf[48];
  uint32_t toNext = TX_SLOT_S - (sod % TX_SLOT_S);
  uint8_t mode;
  while ((mode = txMode((sod + toNext) % 86400UL)) == MODE_NONE) toNext += TX_SLOT_S;   // skip the idle quarter
  uint8_t band = ROTATE_BANDS ? txCount[mode] % NUM_BANDS : FIXED_BAND;
  snprintf(buf, sizeof(buf), " UTC %02u:%02u:%02u %s next TX in %us ",
           (unsigned)(sod / 3600), (unsigned)(sod / 60 % 60), (unsigned)(sod % 60),
           ppsLocked ? "PPS" : "NMEA", (unsigned)toNext);
  Serial.print(buf); Serial.print(modeName(mode)); Serial.print(' '); Serial.println(BAND_NAMES[band]);
}

void setup() {
  pinMode(TX_LED_PIN, OUTPUT); pinMode(GPS_LED_PIN, OUTPUT); pinMode(PWR_LED_PIN, OUTPUT);
  digitalWrite(PWR_LED_PIN, HIGH);
  Serial.begin(9600);
#if ENABLE_FT4 && ENABLE_FT2 && ENABLE_JTTY
  Serial.println(F("FT8_Beacon v4.9 " CALLSIGN " " LOCATOR " FT8+FT4+FT2+JTTY - waiting for stable GPS lock before any TX"));
#elif ENABLE_FT4 && ENABLE_FT2
  Serial.println(F("FT8_Beacon v4.9 " CALLSIGN " " LOCATOR " FT8+FT4+FT2 - waiting for stable GPS lock before any TX"));
#elif ENABLE_FT2
  Serial.println(F("FT8_Beacon v4.9 " CALLSIGN " " LOCATOR " FT8+FT2 - waiting for stable GPS lock before any TX"));
#elif ENABLE_FT4
  Serial.println(F("FT8_Beacon v4.9 " CALLSIGN " " LOCATOR " FT8+FT4 - waiting for stable GPS lock before any TX"));
#else
  Serial.println(F("FT8_Beacon v4.9 " CALLSIGN " " LOCATOR " - waiting for stable GPS lock before any TX"));
#endif

  if (!si5351.init(SI5351_LOAD, SI5351_XTAL_HZ, SI5351_CORR_PPB)) {
    Serial.println(F("Si5351 not found - halted"));
    while (1) { digitalWrite(TX_LED_PIN, !digitalRead(TX_LED_PIN)); delay(250); }
  }
  Wire.setClock(400000);                       // init() calls Wire.begin() (100 kHz)
  si5351.set_clock_pwr(SI5351_CLK0, 0);        // keep driver off before first set_freq
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE);
  si5351.set_freq((uint64_t)BANDS_HZ[0] * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 0);

#if CAL_MODE
  si5351.set_freq((uint64_t)CAL_FREQ_HZ * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(TX_LED_PIN, HIGH);
  Serial.print(F("CAL_MODE: carrier on ")); Serial.print(CAL_FREQ_HZ);
  Serial.print(F(" Hz, correction ")); Serial.print(SI5351_CORR_PPB); Serial.println(F(" ppb"));
  while (1) {}
#endif
  // Encode once per mode here to validate CALLSIGN/LOCATOR and print the tones (compare with README).
  for (uint8_t mode = MODE_FT8; mode <= MODE_JTTY; mode++) {
    if ((mode == MODE_FT4 && !ENABLE_FT4) || (mode == MODE_FT2 && !ENABLE_FT2) || (mode == MODE_JTTY && !ENABLE_JTTY)) continue;
    if (!encodeTones(mode)) {
      Serial.println(F("Callsign/locator not encodable - halted"));
      while (1) {}
    }
    Serial.print(modeName(mode)); Serial.print(F(" tones: "));
    uint8_t nsym = (mode == MODE_FT8) ? FT8_SYMBOLS : (mode == MODE_JTTY) ? JTTY_SYMBOLS : FT4_SYMBOLS;
    for (uint8_t i = 0; i < nsym; i++) Serial.print(tones[i]);
    Serial.println();
  }

#if USE_PPS
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsIsr, RISING);
#endif
  gpsSerial.begin(GPS_BAUD);
#if GPS_SEND_PCAS
  gpsSerial.print(F("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n"));   // GGA+RMC only (RAM, not saved)
#endif
  randomSeed(analogRead(A7) ^ micros());
}

void loop() {
  static uint32_t lastStatusMs = 0;
  while (gpsSerial.available()) {
    uint32_t now = micros();
    if (now - lastCharUs > NMEA_GAP_MS * 1000UL) burstFresh = true;   // line was idle: new burst
    lastCharUs = now;
    char c = gpsSerial.read();
    if (c == '$') { sentUs = now; noInterrupts(); sentPpsUs = ppsUs; sentPpsCnt = ppsCount; interrupts(); }
    if (gps.encode(c)) onNmea(micros());
  }
  if ((stableCount || gpsStable) && micros() - lastSecUs > 2500000UL) gpsGlitch(F("NMEA timeout"));
  // reference too old? (a PPS-based one stays accurate for seconds, so ride out a lost GGA)
  if (timeValid && micros() - lastSyncUs > (ppsLocked ? 3500000UL : 1500000UL)) timeValid = false;
  // GPS LED: solid = stable lock (TX enabled); blinking = counting; off = no fix
  digitalWrite(GPS_LED_PIN, gpsStable ? HIGH : (stableCount ? (millis() >> 8) & 1 : LOW));

  if (gpsStable && timeValid) {
    uint32_t elapsed = (micros() - refUs) / 1000000UL;
    uint32_t sod = (refSod + elapsed) % 86400UL;
    // In the last second before a slot boundary: arm and transmit. Slots every TX_SLOT_S
    // (60 s with FT2/JTTY): FT8 :00.5, FT2 :01.476, FT4 :02.5, JTTY :03.5. Only one TX per slot.
    if (sod % TX_SLOT_S == TX_SLOT_S - 1) {
      int32_t slot = (int32_t)((sod + 1) % 86400UL);
      uint8_t mode = txMode((uint32_t)slot);
      if (slot != lastTxSlot && mode != MODE_NONE) {
        lastTxSlot = slot;
        // each mode rotates its own band index; normally FT2 and FT4 follow the FT8 on the same band
        uint8_t band = ROTATE_BANDS ? txCount[mode] % NUM_BANDS : FIXED_BAND;
        uint32_t slotEdgeUs = refUs + (elapsed + 1) * 1000000UL;
        int32_t delayMs;
        if (mode == MODE_JTTY) delayMs = JTTY_START_DELAY_MS;
        else if (mode == MODE_FT2) delayMs = FT2_START_DELAY_MS;
        else if (mode == MODE_FT4) delayMs = FT4_START_DELAY_MS;
        else delayMs = TX_START_DELAY_MS;
        delayMs += TX_START_TRIM_MS;
        uint32_t startUs = slotEdgeUs + delayMs * 1000L;
        transmit(mode, band, startUs, slotEdgeUs);
        txCount[mode]++;
      }
    }
  }
  if (millis() - lastStatusMs >= 5000) { lastStatusMs = millis(); printStatus(); }
}
