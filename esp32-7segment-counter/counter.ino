/*
 * 2-Digit 7-Segment LED Counter (0–30)
 * NodeMCU v3 — ESP8266 (ESP-12E / ESP8266MOD)
 * Display: MLN5241RK (2-digit, common cathode, red)
 *
 * Wiring:
 *   74HC595 SER   ← D7  (GPIO13, SPI MOSI)
 *   74HC595 SRCLK ← D5  (GPIO14, SPI CLK)
 *   74HC595 RCLK  ← D8  (GPIO15, SPI CS)  — has 10kΩ pull-down, starts LOW ✓
 *   74HC595 VCC   → 3.3V  |  GND → GND
 *   74HC595 OE    → GND   (output always enabled)
 *   74HC595 SRCLR → 3.3V  (clear disabled)
 *   74HC595 Q0–Q6 → 150Ω → Display seg a–g (both digits share)
 *
 *   D3 (GPIO0)  → 1kΩ → BC547 Q1 → COM1 pin 15 (TENS  digit)
 *   D4 (GPIO2)  → 1kΩ → BC547 Q2 → COM2 pin 6  (UNITS digit)
 *
 *   D1 (GPIO5)  → BTN_INC → GND  (INPUT_PULLUP)
 *   D2 (GPIO4)  → BTN_DEC → GND  (INPUT_PULLUP)
 *   D0 (GPIO16) → BTN_RST → GND  + 10kΩ from D0 to 3.3V  ← mandatory!
 *
 *   D6 (GPIO12) → 150Ω → Passive Buzzer (+) → GND
 *
 * BOOT NOTES:
 *   GPIO0 (D3) & GPIO2 (D4) have hardware pull-ups → transistors briefly ON
 *   at power-on until setup() clears them. This is harmless (a flicker).
 *   Do NOT hold the INC button (D3) while powering on — it enters flash mode.
 *
 * Arduino IDE board setting: "NodeMCU 1.0 (ESP-12E Module)"
 */

#include <SPI.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
// NodeMCU GPIO numbers (not D-label numbers)

#define PIN_LATCH    15   // D8 — 74HC595 RCLK
// SPI MOSI = GPIO13 (D7), SPI CLK = GPIO14 (D5) — set by SPI.begin()

#define DIGIT_TENS    0   // D3 — GPIO0  (tens  digit transistor)
#define DIGIT_UNITS   2   // D4 — GPIO2  (units digit transistor)

#define BTN_INC       5   // D1 — GPIO5  (INPUT_PULLUP)
#define BTN_DEC       4   // D2 — GPIO4  (INPUT_PULLUP)
#define BTN_RST      16   // D0 — GPIO16 (INPUT — no internal pull-up, needs 10kΩ external)

#define BUZZER_PIN   12   // D6 — GPIO12 (PWM capable)

// ── 7-segment encoding ────────────────────────────────────────────────────────
// 74HC595 Q0=a, Q1=b, Q2=c, Q3=d, Q4=e, Q5=f, Q6=g, Q7=NC
// Common cathode: bit HIGH = segment ON
//
//   _
//  |_|   a=top, b=top-right, c=bot-right, d=bottom
//  |_|   e=bot-left, f=top-left, g=middle

const uint8_t SEG_MAP[10] = {
  0x3F, // 0: a b c d e f  ·
  0x06, // 1: · b c · · ·  ·
  0x5B, // 2: a b · d e ·  g
  0x4F, // 3: a b c d · ·  g
  0x66, // 4: · b c · · f  g
  0x6D, // 5: a · c d · f  g
  0x7D, // 6: a · c d e f  g
  0x07, // 7: a b c · · ·  ·
  0x7F, // 8: a b c d e f  g
  0x6F, // 9: a b c d · f  g
};

// ── Counter ───────────────────────────────────────────────────────────────────

const int COUNT_MAX = 30;
const int COUNT_MIN =  0;
int counter = 0;

// ── Buzzer — non-blocking AC-remote two-phase beep ────────────────────────────
//
// Phase 0: 3800 Hz, 25 ms — sharp high-pitched attack
// Phase 1:    0 Hz,  5 ms — brief silence
// Phase 2: 2800 Hz, 60 ms — warm body tone
// Matches the "tick-beep" of Daikin/Mitsubishi/Panasonic AC remotes.

struct BeepPhase { uint32_t freq; uint32_t ms; };

const BeepPhase BEEP_SEQ[] = {
  {3800, 25},
  {   0,  5},
  {2800, 60},
};
const int BEEP_PHASES = sizeof(BEEP_SEQ) / sizeof(BEEP_SEQ[0]);

int      beepPhase   = -1;   // -1 = idle
uint32_t beepPhaseMs =  0;

void beepStart() {
  beepPhase   = 0;
  beepPhaseMs = millis();
  tone(BUZZER_PIN, BEEP_SEQ[0].freq);
}

void beepTick() {
  if (beepPhase < 0) return;
  if (millis() - beepPhaseMs < BEEP_SEQ[beepPhase].ms) return;

  beepPhase++;
  if (beepPhase >= BEEP_PHASES) {
    noTone(BUZZER_PIN);
    beepPhase = -1;
    return;
  }
  beepPhaseMs = millis();
  if (BEEP_SEQ[beepPhase].freq > 0)
    tone(BUZZER_PIN, BEEP_SEQ[beepPhase].freq);
  else
    noTone(BUZZER_PIN);
}

// ── Button debounce ───────────────────────────────────────────────────────────

const uint32_t DEBOUNCE_MS = 50;

struct Button {
  uint8_t  pin;
  bool     lastReading;
  bool     stableState;
  uint32_t lastChangeMs;
};

Button btnInc = {BTN_INC, HIGH, HIGH, 0};
Button btnDec = {BTN_DEC, HIGH, HIGH, 0};
Button btnRst = {BTN_RST, HIGH, HIGH, 0};

// Returns true exactly once per physical press (debounced falling edge).
bool checkPress(Button &btn) {
  bool     r   = digitalRead(btn.pin);
  uint32_t now = millis();
  if (r != btn.lastReading) { btn.lastChangeMs = now; btn.lastReading = r; }
  if (now - btn.lastChangeMs >= DEBOUNCE_MS && btn.stableState != r) {
    btn.stableState = r;
    return r == LOW;
  }
  return false;
}

// ── Display multiplexing via 74HC595 ─────────────────────────────────────────

const uint32_t MUX_US = 2000;   // 2 ms per digit → 250 Hz, no flicker
uint32_t lastMuxUs = 0;
bool     showTens  = true;

void shift595(uint8_t data) {
  digitalWrite(PIN_LATCH, LOW);
  SPI.transfer(data);
  digitalWrite(PIN_LATCH, HIGH);
}

// Call every loop iteration — switches active digit every MUX_US microseconds.
void updateDisplay() {
  if (micros() - lastMuxUs < MUX_US) return;
  lastMuxUs = micros();

  // Blank both digits first to prevent ghosting on transistor switch.
  digitalWrite(DIGIT_TENS,  LOW);
  digitalWrite(DIGIT_UNITS, LOW);
  shift595(0x00);

  if (showTens) {
    shift595(SEG_MAP[counter / 10]);
    digitalWrite(DIGIT_TENS, HIGH);
  } else {
    shift595(SEG_MAP[counter % 10]);
    digitalWrite(DIGIT_UNITS, HIGH);
  }
  showTens = !showTens;
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────

void setup() {
  // Hardware SPI: MOSI=D7(GPIO13), CLK=D5(GPIO14) assigned automatically
  SPI.begin();
  SPI.setFrequency(1000000);     // 1 MHz — well within 74HC595 max spec (25 MHz)
  SPI.setDataMode(SPI_MODE0);

  pinMode(PIN_LATCH, OUTPUT);
  shift595(0x00);                // clear 595 at start

  pinMode(DIGIT_TENS,  OUTPUT); digitalWrite(DIGIT_TENS,  LOW);
  pinMode(DIGIT_UNITS, OUTPUT); digitalWrite(DIGIT_UNITS, LOW);

  pinMode(BTN_INC, INPUT_PULLUP);  // GPIO5 — internal pull-up available
  pinMode(BTN_DEC, INPUT_PULLUP);  // GPIO4 — internal pull-up available
  pinMode(BTN_RST, INPUT);         // GPIO16 — NO internal pull-up, needs external 10kΩ

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
}

void loop() {
  updateDisplay();   // must run every iteration for flicker-free multiplexing
  beepTick();        // advances beep state machine without blocking

  if (checkPress(btnInc)) { if (counter < COUNT_MAX) counter++; beepStart(); }
  if (checkPress(btnDec)) { if (counter > COUNT_MIN) counter--; beepStart(); }
  if (checkPress(btnRst)) { counter = COUNT_MIN; beepStart(); }
}
