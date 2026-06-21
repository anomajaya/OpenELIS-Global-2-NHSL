/*
 * 2-Digit 7-Segment LED Counter (0–30) for ESP32-WROOM-32
 * With AC-remote-style passive buzzer feedback
 *
 * Wiring:
 *   Segments a–g  : GPIO 13, 14, 16, 17, 18, 19, 21  (each via 150Ω resistor)
 *   Tens  digit   : GPIO 22  → 1kΩ → BC547 base  (collector → Display 1 COM → GND)
 *   Units digit   : GPIO 23  → 1kΩ → BC547 base  (collector → Display 2 COM → GND)
 *   INC  button   : GPIO 25  → GND  (uses internal pull-up)
 *   DEC  button   : GPIO 26  → GND  (uses internal pull-up)
 *   RST  button   : GPIO 27  → GND  (uses internal pull-up)
 *   Passive buzzer: GPIO 4   → Buzzer (+) ; Buzzer (–) → GND
 *   Power         : 3.3V + GND from ESP32 DevKit
 *
 * Display type: Common Cathode
 * Buzzer type : Passive (magnetic/piezo). Active buzzers will NOT work here
 *               because we need frequency control for the AC-remote tone.
 *
 * Arduino-ESP32 core compatibility:
 *   Core 3.x (current): uses ledcAttach(pin, freq, resolution)
 *   Core 2.x (legacy) : comment out the 3.x block, uncomment the 2.x block
 *                        in setup() and beepTick().
 */

// ── Pin definitions ───────────────────────────────────────────────────────────

const uint8_t SEG_PINS[7] = {13, 14, 16, 17, 18, 19, 21}; // a, b, c, d, e, f, g

const uint8_t DIGIT_TENS  = 22;
const uint8_t DIGIT_UNITS = 23;

const uint8_t BTN_INC    = 25;
const uint8_t BTN_DEC    = 26;
const uint8_t BTN_RST    = 27;

const uint8_t BUZZER_PIN = 4;

// ── 7-segment encoding ────────────────────────────────────────────────────────
// Common cathode: bit HIGH = segment ON
// Bit position: 6=g, 5=f, 4=e, 3=d, 2=c, 1=b, 0=a
//
//   _
//  |_|   a=top, b=top-right, c=bot-right, d=bottom
//  |_|   e=bot-left, f=top-left, g=middle
//
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

// ── Passive buzzer — non-blocking AC-remote beep ──────────────────────────────
//
// AC remote control sound profile:
//   Tone 1 (click) : 3800 Hz, 25 ms  — high-pitched attack
//   Tone 2 (body)  : 2800 Hz, 60 ms  — warm body tone
//   Gap between    :    5 ms silence
//
// This two-phase shape matches the characteristic "tick-beep" of most
// Daikin / Mitsubishi / Panasonic AC remote controls.

struct BeepPhase {
  uint32_t freq;        // Hz  (0 = silence)
  uint32_t durationMs;
};

const BeepPhase BEEP_SEQUENCE[] = {
  {3800, 25},  // phase 0: sharp attack
  {   0,  5},  // phase 1: brief silence
  {2800, 60},  // phase 2: warm body
};
const int BEEP_PHASES = sizeof(BEEP_SEQUENCE) / sizeof(BEEP_SEQUENCE[0]);

int      beepPhase      = -1;   // -1 = idle
uint32_t beepPhaseStart =  0;

// Start a fresh beep sequence (safe to call even if a beep is in progress).
void beepStart() {
  beepPhase      = 0;
  beepPhaseStart = millis();
  ledcWriteTone(BUZZER_PIN, BEEP_SEQUENCE[0].freq);  // ESP32 core 3.x
  // Core 2.x: ledcWriteTone(0, BEEP_SEQUENCE[0].freq);
}

// Advance the beep state machine — call every loop iteration.
void beepTick() {
  if (beepPhase < 0) return;

  if ((millis() - beepPhaseStart) >= BEEP_SEQUENCE[beepPhase].durationMs) {
    beepPhase++;
    if (beepPhase >= BEEP_PHASES) {
      ledcWriteTone(BUZZER_PIN, 0);  // ESP32 core 3.x
      // Core 2.x: ledcWriteTone(0, 0);
      beepPhase = -1;
      return;
    }
    beepPhaseStart = millis();
    ledcWriteTone(BUZZER_PIN, BEEP_SEQUENCE[beepPhase].freq);  // core 3.x
    // Core 2.x: ledcWriteTone(0, BEEP_SEQUENCE[beepPhase].freq);
  }
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

// Returns true exactly once per physical press (falling edge, debounced).
bool checkPress(Button &btn) {
  bool reading = digitalRead(btn.pin);
  uint32_t now = millis();

  if (reading != btn.lastReading) {
    btn.lastChangeMs = now;
    btn.lastReading  = reading;
  }

  if ((now - btn.lastChangeMs) >= DEBOUNCE_MS && btn.stableState != reading) {
    btn.stableState = reading;
    return (reading == LOW);
  }
  return false;
}

// ── Display multiplexing ──────────────────────────────────────────────────────

const uint32_t MUX_US = 2000;  // 2 ms per digit → 250 Hz refresh

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

void updateDisplay() {
  if ((micros() - lastMuxUs) < MUX_US) return;
  lastMuxUs = micros();

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

  // Buzzer — ESP32 core 3.x
  ledcAttach(BUZZER_PIN, 2800, 8);  // pin, initial freq, 8-bit resolution
  ledcWrite(BUZZER_PIN, 0);         // silent at start

  // Buzzer — ESP32 core 2.x (uncomment if you get a compile error above)
  // ledcSetup(0, 2800, 8);         // channel 0, freq, resolution
  // ledcAttachPin(BUZZER_PIN, 0);
  // ledcWrite(0, 0);
}

void loop() {
  updateDisplay();  // must run every iteration for flicker-free multiplexing
  beepTick();       // advances the beep state machine without blocking

  if (checkPress(btnInc)) {
    if (counter < COUNT_MAX) counter++;
    beepStart();
  }
  if (checkPress(btnDec)) {
    if (counter > COUNT_MIN) counter--;
    beepStart();
  }
  if (checkPress(btnRst)) {
    counter = COUNT_MIN;
    beepStart();
  }
}
