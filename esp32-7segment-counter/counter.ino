/*
 * 2-Digit 7-Segment LED Counter (0–30)
 * ESP32-WROOM-32 DevKitC (38-pin)
 * Display: MLN5241RK (2-digit, common cathode, red)
 *
 * Wiring:
 *   GPIO 13 → 150Ω → Seg a  ┐
 *   GPIO 14 → 150Ω → Seg b  │
 *   GPIO 16 → 150Ω → Seg c  │ both digits share these segment lines
 *   GPIO 17 → 150Ω → Seg d  │
 *   GPIO 18 → 150Ω → Seg e  │
 *   GPIO 19 → 150Ω → Seg f  │
 *   GPIO 21 → 150Ω → Seg g  ┘
 *
 *   GPIO 22 → 1kΩ → BC547 Q1 base; Q1 collector → COM1 pin 15 (TENS  digit)
 *   GPIO 23 → 1kΩ → BC547 Q2 base; Q2 collector → COM2 pin 6  (UNITS digit)
 *   Q1, Q2 emitters → GND
 *
 *   GPIO 25 → BTN_INC → GND  (INPUT_PULLUP)
 *   GPIO 26 → BTN_DEC → GND  (INPUT_PULLUP)
 *   GPIO 27 → BTN_RST → GND  (INPUT_PULLUP)
 *
 *   GPIO 4  → 150Ω → Passive Buzzer (+) → GND
 *   Power   : 3.3V + GND from DevKit board
 *
 * MLN5241RK pinout (common cathode, view from front):
 *   Seg a : pin 8  (or 17)    COM1 (tens,  left  digit): pin 15
 *   Seg b : pin 7  (or 16)    COM2 (units, right digit): pin 6
 *   Seg c : pin 4  (or 13)    DP (decimal): pins 3,12 — leave unconnected
 *   Seg d : pin 2  (or 11)
 *   Seg e : pin 1  (or 10)
 *   Seg f : pin 9  (or 18)
 *   Seg g : pin 5  (or 14)
 *   (Connect either pin of each mirrored pair — both sides are identical)
 *
 * Pins deliberately avoided (ESP32 strapping / flash pins):
 *   GPIO 0, 2, 5, 12, 15  — strapping pins (affect boot mode)
 *   GPIO 6–11              — internal flash (never use)
 *   GPIO 34–39             — input-only
 *
 * Arduino IDE board setting: "ESP32 Dev Module"
 */

// ── Segment pins (a → g) ──────────────────────────────────────────────────────

const uint8_t SEG_PINS[7] = {13, 14, 16, 17, 18, 19, 21};

// ── Digit select (HIGH = NPN transistor ON = digit cathode sunk to GND) ───────

const uint8_t DIGIT_TENS  = 22;
const uint8_t DIGIT_UNITS = 23;

// ── Button pins (INPUT_PULLUP — press connects to GND = LOW) ─────────────────

const uint8_t BTN_INC = 25;
const uint8_t BTN_DEC = 26;
const uint8_t BTN_RST = 27;

// ── Buzzer ────────────────────────────────────────────────────────────────────

const uint8_t BUZZER_PIN = 4;

// ── 7-segment encoding ────────────────────────────────────────────────────────
// Common cathode: bit HIGH = segment ON
// Bit position: 0=a, 1=b, 2=c, 3=d, 4=e, 5=f, 6=g
//
//     a
//   ┌───┐
// f │   │ b
//   ├───┤  ← g
// e │   │ c
//   └───┘
//     d

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
// Matches the "tick-beep" of Daikin / Mitsubishi / Panasonic AC remotes:
//   Phase 0: 3800 Hz, 25 ms — sharp high-pitched attack (the "tick")
//   Phase 1:    0 Hz,  5 ms — brief silence gap
//   Phase 2: 2800 Hz, 60 ms — warm body tone  (the "beep")
//
// Uses ESP32 LEDC hardware timer — completely non-blocking so display
// multiplexing continues uninterrupted during the beep.

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
  ledcWriteTone(BUZZER_PIN, BEEP_SEQ[0].freq);
}

void beepTick() {
  if (beepPhase < 0) return;
  if (millis() - beepPhaseMs < BEEP_SEQ[beepPhase].ms) return;

  beepPhase++;
  if (beepPhase >= BEEP_PHASES) {
    ledcWriteTone(BUZZER_PIN, 0);
    beepPhase = -1;
    return;
  }
  beepPhaseMs = millis();
  ledcWriteTone(BUZZER_PIN, BEEP_SEQ[beepPhase].freq);
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

// ── Display multiplexing ──────────────────────────────────────────────────────

const uint32_t MUX_US = 2000;   // 2 ms per digit → 250 Hz refresh, no flicker
uint32_t lastMuxUs = 0;
bool     showTens  = true;

void writeSegments(uint8_t digit) {
  uint8_t enc = SEG_MAP[digit];
  for (int i = 0; i < 7; i++) {
    digitalWrite(SEG_PINS[i], (enc >> i) & 1);
  }
}

void clearSegments() {
  for (int i = 0; i < 7; i++) digitalWrite(SEG_PINS[i], LOW);
}

// Call every loop iteration — switches active digit every MUX_US microseconds.
void updateDisplay() {
  if (micros() - lastMuxUs < MUX_US) return;
  lastMuxUs = micros();

  // Blank both digits before switching to prevent ghosting.
  digitalWrite(DIGIT_TENS,  LOW);
  digitalWrite(DIGIT_UNITS, LOW);
  clearSegments();

  if (showTens) {
    writeSegments(counter / 10);
    digitalWrite(DIGIT_TENS, HIGH);
  } else {
    writeSegments(counter % 10);
    digitalWrite(DIGIT_UNITS, HIGH);
  }
  showTens = !showTens;
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────

void setup() {
  for (int i = 0; i < 7; i++) {
    pinMode(SEG_PINS[i], OUTPUT);
    digitalWrite(SEG_PINS[i], LOW);
  }

  pinMode(DIGIT_TENS,  OUTPUT); digitalWrite(DIGIT_TENS,  LOW);
  pinMode(DIGIT_UNITS, OUTPUT); digitalWrite(DIGIT_UNITS, LOW);

  pinMode(BTN_INC, INPUT_PULLUP);
  pinMode(BTN_DEC, INPUT_PULLUP);
  pinMode(BTN_RST, INPUT_PULLUP);

  // Buzzer via ESP32 LEDC hardware PWM (core 3.x API)
  ledcAttach(BUZZER_PIN, 2800, 8);  // pin, initial freq, 8-bit resolution
  ledcWrite(BUZZER_PIN, 0);         // silent at start
}

void loop() {
  updateDisplay();   // must run every iteration for flicker-free multiplexing
  beepTick();        // advances beep state machine without blocking

  if (checkPress(btnInc)) { if (counter < COUNT_MAX) counter++; beepStart(); }
  if (checkPress(btnDec)) { if (counter > COUNT_MIN) counter--; beepStart(); }
  if (checkPress(btnRst)) { counter = COUNT_MIN; beepStart(); }
}
