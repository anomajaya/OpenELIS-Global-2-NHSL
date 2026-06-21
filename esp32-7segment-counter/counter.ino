/*
 * 2-Digit 7-Segment LED Counter (0–30) for ESP32-WROOM-32
 *
 * Wiring:
 *   Segments a–g  : GPIO 13, 14, 16, 17, 18, 19, 21  (each via 150Ω resistor)
 *   Tens  digit   : GPIO 22  → 1kΩ → BC547 base  (collector → Display 1 COM → GND)
 *   Units digit   : GPIO 23  → 1kΩ → BC547 base  (collector → Display 2 COM → GND)
 *   INC  button   : GPIO 25  → GND  (uses internal pull-up)
 *   DEC  button   : GPIO 26  → GND  (uses internal pull-up)
 *   RST  button   : GPIO 27  → GND  (uses internal pull-up)
 *   Power         : 3.3V + GND from ESP32 DevKit
 *
 * Display type: Common Cathode
 * Logic: HIGH on digit pin turns transistor ON → sinks cathode → digit lights up
 */

// ── Pin definitions ───────────────────────────────────────────────────────────

// Segment pins in order: a, b, c, d, e, f, g
const uint8_t SEG_PINS[7] = {13, 14, 16, 17, 18, 19, 21};

const uint8_t DIGIT_TENS  = 22;  // NPN base for tens  digit (HIGH = ON)
const uint8_t DIGIT_UNITS = 23;  // NPN base for units digit (HIGH = ON)

const uint8_t BTN_INC = 25;      // Increment
const uint8_t BTN_DEC = 26;      // Decrement
const uint8_t BTN_RST = 27;      // Reset

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

// ── Counter limits ────────────────────────────────────────────────────────────

const int COUNT_MAX = 30;
const int COUNT_MIN =  0;

int counter = 0;

// ── Button debounce ───────────────────────────────────────────────────────────

const uint32_t DEBOUNCE_MS = 50;

struct Button {
  uint8_t  pin;
  bool     lastReading;   // raw GPIO reading
  bool     stableState;   // debounced state
  uint32_t lastChangeMs;  // timestamp of last raw change
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
    return (reading == LOW);  // fire only on press, not release
  }
  return false;
}

// ── Display multiplexing ──────────────────────────────────────────────────────

const uint32_t MUX_US = 2000;  // 2 ms per digit → 250 Hz refresh, no flicker

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

// Call as often as possible; switches active digit every MUX_US microseconds.
void updateDisplay() {
  if ((micros() - lastMuxUs) < MUX_US) return;
  lastMuxUs = micros();

  // Blank both digits first to prevent ghosting between transitions.
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
}

void loop() {
  updateDisplay();  // must run every loop iteration for smooth multiplexing

  if (checkPress(btnInc) && counter < COUNT_MAX) counter++;
  if (checkPress(btnDec) && counter > COUNT_MIN) counter--;
  if (checkPress(btnRst))                        counter = COUNT_MIN;
}
