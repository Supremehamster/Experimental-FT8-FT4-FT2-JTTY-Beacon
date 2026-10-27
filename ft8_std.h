// ft8_std.h - FT8 and FT4 encoder for a standard "CQ <call> <grid4>" message (type i3=1).
//
// Why this file exists: Etherkit JTEncode's public ft8_encode() only packs FREE TEXT
// (type 0.0), and JTEncode 1.3.1 has no FT4 encoder at all. "CQ N0TOK XX00" fits in 13 chars,
// so free text decodes and shows the same words, but decoders do not treat it as a real CQ.
// WSJT-X/MSHV do not parse a callsign or grid from it, so PSK Reporter gets no spots. This file
// packs a proper standard message following the WSJT-X 77-bit protocol and uses the same LDPC
// generator matrix that JTEncode ships with (generator.h).
//
// FT8 and FT4 share everything up to the 174-bit codeword (77-bit packing, CRC14, LDPC(174,91)).
// FT4 differs only in: (1) the 77 message bits are XORed with the FT4 scrambling vector "rvec"
// BEFORE the CRC and parity are computed, (2) 2 bits per symbol (4-FSK, Gray 0,1,3,2),
// (3) frame R S4 D29 S4 D29 S4 D29 S4 R = 105 symbols (ramp symbols = tone 0, as ft8_lib).
// Output of both modes was checked bit-exact against kgoba/ft8_lib ft8_encode()/ft4_encode()
// (see README "Verification" and extras/verify_ft4_harness.cpp.txt).
#pragma once
#include <string.h>
#include <generator.h>   // from the Etherkit JTEncode library: generator_bits[83][12] in PROGMEM

namespace ft8std {

static int8_t idx(const char *set, char ch) {
  const char *p = strchr(set, ch);
  return (p && ch) ? (int8_t)(p - set) : -1;
}

// pack28 for a standard callsign (e.g. N0TOK, K1ABC, 9A1AA). Returns -1 if unsupported.
static int32_t packCall(const char *call) {
  char c6[7] = "      ";
  uint8_t n = strlen(call);
  if (n < 3 || n > 6) return -1;
  if (call[2] >= '0' && call[2] <= '9') memcpy(c6, call, n);                    // 3rd char digit, e.g. "KA1ABC", "9A1AA"
  else if (call[1] >= '0' && call[1] <= '9' && n <= 5) memcpy(c6 + 1, call, n); // e.g. "N0TOK" -> " N0TOK"
  else return -1;
  static const char A1[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static const char A2[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static const char A3[] = "0123456789";
  static const char A4[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  int8_t i1 = idx(A1, c6[0]), i2 = idx(A2, c6[1]), i3 = idx(A3, c6[2]);
  int8_t i4 = idx(A4, c6[3]), i5 = idx(A4, c6[4]), i6 = idx(A4, c6[5]);
  if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0 || i6 < 0) return -1;
  int32_t v = ((((int32_t)i1 * 36 + i2) * 10 + i3) * 27 + i4) * 27 + i5;
  v = v * 27 + i6;
  return 2063592L + 4194304L + v;           // NTOKENS + MAX22 + n
}

static int16_t packGrid4(const char *g) {
  if (strlen(g) != 4 || g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R' ||
      g[2] < '0' || g[2] > '9' || g[3] < '0' || g[3] > '9') return -1;
  return (int16_t)(((g[0] - 'A') * 18 + (g[1] - 'A')) * 100 + (g[2] - '0') * 10 + (g[3] - '0'));
}

static void putBits(uint8_t *buf, uint8_t &pos, uint32_t val, uint8_t nbits) {
  while (nbits--) {
    if ((val >> nbits) & 1) buf[pos >> 3] |= (uint8_t)(0x80 >> (pos & 7));
    pos++;
  }
}

static uint16_t crc14(const uint8_t *msg, uint8_t nbits) {   // poly 0x2757, as ft8_lib
  uint16_t r = 0;
  for (uint8_t i = 0; i < nbits; i++) {
    if ((i & 7) == 0) r ^= (uint16_t)msg[i >> 3] << 6;
    r = (r & 0x2000) ? (uint16_t)((r << 1) ^ 0x2757) : (uint16_t)(r << 1);
  }
  return r & 0x3FFF;
}

// FT4 scrambling vector rvec (WSJT-X genft4.f90 / ft8_lib kFT4_XOR_sequence), 77 bits MSB first.
static const uint8_t FT4_RVEC[10] PROGMEM = {0x4A, 0x5E, 0x89, 0xB4, 0xB0, 0x8A, 0x79, 0x55, 0xBE, 0x28};

// Bit 'bit' (0..173) of the LDPC(174,91) codeword: 0..90 systematic (77 msg + 14 CRC), 91..173 parity.
// One shared implementation for FT8 and FT4 (parity computed on the fly from JTEncode's generator).
static uint8_t cwBit(const uint8_t *a91, uint8_t bit) {
  if (bit < 91) return (a91[bit >> 3] >> (7 - (bit & 7))) & 1;
  uint8_t row = bit - 91, par = 0;
  for (uint8_t c = 0; c < 12; c++) {
    uint8_t x = a91[c] & pgm_read_byte(&generator_bits[row][c]);
    while (x) { par ^= 1; x &= (uint8_t)(x - 1); }
  }
  return par;
}

#define FT8STD_FT8_SYMBOLS 79
#define FT8STD_FT4_SYMBOLS 105

// Encode "CQ <call> <grid4>". ft4 == false: 79 FT8 tones (0..7). ft4 == true: 105 FT4 tones (0..3).
// Returns false if call/grid unsupported (tones untouched).
static bool encodeCQ(const char *call, const char *grid, uint8_t *tones, bool ft4 = false) {
  int32_t n28 = packCall(call);
  int16_t g15 = packGrid4(grid);
  if (n28 < 0 || g15 < 0) return false;

  uint8_t a91[12] = {0};
  uint8_t pos = 0;
  putBits(a91, pos, 2, 28);      // "CQ" token
  putBits(a91, pos, 0, 1);       // ipa
  putBits(a91, pos, n28, 28);    // callsign
  putBits(a91, pos, 0, 1);       // ipb
  putBits(a91, pos, 0, 1);       // ir (no R)
  putBits(a91, pos, g15, 15);    // grid
  putBits(a91, pos, 1, 3);       // i3 = 1 (standard message)
  if (ft4)                       // FT4 only: scramble the 77 bits before CRC + LDPC (rvec bits 77..79 are 0)
    for (uint8_t i = 0; i < 10; i++) a91[i] ^= pgm_read_byte(&FT4_RVEC[i]);
  uint16_t crc = crc14(a91, 82); // 77 bits + 5 zero bits
  a91[9] |= (uint8_t)(crc >> 11);
  a91[10] = (uint8_t)(crc >> 3);
  a91[11] = (uint8_t)(crc << 5);

  // Gray maps: FT8 {0,1,3,2,5,6,4,7}; FT4 {0,1,3,2} = the first 4 entries of the same table.
  static const uint8_t gray[8]   = {0, 1, 3, 2, 5, 6, 4, 7};
  uint8_t k, bps, nData;
  if (ft4) {
    // FT4 sync blocks {0,1,3,2},{1,0,2,3},{2,3,1,0},{3,2,0,1} = gray[i] XOR block number,
    // at symbols 1, 34, 67, 100; ramp symbols 0 and 104 = tone 0.
    tones[0] = tones[104] = 0;
    for (uint8_t b = 0; b < 4; b++)
      for (uint8_t i = 0; i < 4; i++) tones[1 + 33 * b + i] = gray[i] ^ b;
    k = 5; bps = 2; nData = 87;                 // 87 data symbols x 2 bits = 174
  } else {
    static const uint8_t costas[7] = {3, 1, 4, 0, 6, 5, 2};
    for (uint8_t i = 0; i < 7; i++) tones[i] = tones[36 + i] = tones[72 + i] = costas[i];
    k = 7; bps = 3; nData = 58;                 // 58 data symbols x 3 bits = 174
  }

  uint8_t bit = 0;
  for (uint8_t j = 0; j < nData; j++) {
    uint8_t v = 0;
    for (uint8_t b = 0; b < bps; b++) v = (uint8_t)((v << 1) | cwBit(a91, bit++));
    if (j == 29 || (ft4 && j == 58)) k += ft4 ? 4 : 7;   // skip the next sync block
    tones[k++] = gray[v];
  }
  return true;
}

} // namespace ft8std
