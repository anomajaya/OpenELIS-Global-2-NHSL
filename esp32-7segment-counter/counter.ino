/*
 * 2-Digit 7-Segment Counter + 5V Fan Speed Controller + OTA updates
 * ESP32-WROOM-32 DevKitC (38-pin)
 * Display: 2-digit COMMON CATHODE, red (10-pin, 5 per side)
 *
 * Requires library: "IRremote" by Armin Joachimsmeyer, version 4.x
 * (WiFi / ArduinoOTA / ESPmDNS ship with the ESP32 board package — no install)
 *
 * ── OTA updates ──────────────────────────────────────────────────────────────
 *   1. Fill in WIFI_SSID / WIFI_PASS below (and change OTA_PASSWORD).
 *   2. Flash once over USB.
 *   3. From then on: Arduino IDE → Tools → Port → pick the network port
 *      "fan-controller at 192.168.x.x" and upload wirelessly.
 *   The device works fine with no WiFi — it retries every 30 s in the
 *   background and everything else runs normally.
 *
 * Display pinout (pin 1 = bottom-left, face toward you):
 *   Pin 1  (c)    → 150Ω → GPIO 21
 *   Pin 2  (DP)   — leave unconnected
 *   Pin 3  (e)    → 150Ω → GPIO 18
 *   Pin 4  (d)    → 150Ω → GPIO 17
 *   Pin 5  (g)    → 150Ω → GPIO 16
 *   Pin 6  (f)    → 150Ω → GPIO 14
 *   Pin 7  (DIG2) → BC547 Q2 collector  (UNITS digit)
 *   Pin 8  (DIG1) → BC547 Q1 collector  (TENS  digit)
 *   Pin 9  (b)    → 150Ω → GPIO 19
 *   Pin 10 (a)    → 150Ω → GPIO 13
 *
 * Digit select — BC547 NPN (HIGH = digit ON):
 *   GPIO 22 → 1kΩ → Q1 base; Q1 collector → Pin 8 DIG1; Q1 emitter → GND
 *   GPIO 23 → 1kΩ → Q2 base; Q2 collector → Pin 7 DIG2; Q2 emitter → GND
 *
 * Buttons (INPUT_PULLUP — press connects to GND):
 *   GPIO 25 → BTN_INC → GND
 *   GPIO 32 → BTN_DEC → GND
 *   GPIO 33 → BTN_RST → GND
 *
 * Buzzer:
 *   GPIO 4  → 150Ω → Passive Buzzer (+) → GND
 *
 * IR receiver — 1838 / VS1838B 38 kHz module:
 *   OUT/S  → GPIO 35   (input-only pin, ideal for this)
 *   VCC/+  → 3.3V
 *   GND/-  → GND rail
 *
 * Fan — 5V DC, PWM speed control (Q3 = 2N2222 / S8050 NPN):
 *   VIN (5V)  → Fan (+)
 *   Fan (−)   → Q3 collector;  Q3 emitter → GND
 *   GPIO 2    → 1kΩ → Q3 base
 *   1N4007 flyback diode ACROSS the fan: stripe (cathode) to VIN, other leg to Fan (−)
 *
 * Behaviour:
 *   - Value 0   → fan off.   Value 100 → full speed (display shows "FF").
 *   - Fan speed is proportional to the displayed value (PWM).
 *   - Leading zero is blanked: 0–9 show on the right digit only.
 *   - IR digits: each press shows immediately on the right digit. A second
 *     press within 3 s shifts the right digit to the left (e.g. 1 then 8 = 18).
 *     After a 3 s gap, the next press starts fresh on the right digit.
 *   - IR ▲/▼: +1 / −1 (0–99).
 *   - IR * : set to 0 (fan off).
 *   - IR # : step up 10 → 20 → ... → 100 (next multiple of 10, max 100).
 *   - Physical buttons: INC +1 (max 99), DEC −1 (min 0), RST → 0.
 */

#define DECODE_NEC          // restrict IRremote to NEC — saves RAM, faster
#include <IRremote.hpp>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ── WiFi / OTA settings — EDIT THESE ─────────────────────────────────────────

const char* WIFI_SSID    = "YOUR_WIFI_NAME";      // ← your WiFi network name
const char* WIFI_PASS    = "YOUR_WIFI_PASSWORD";  // ← your WiFi password
const char* OTA_HOSTNAME = "fan-controller";      // name shown in the IDE port list
const char* OTA_PASSWORD = "fan1234";             // ← change this! asked on upload

const uint32_t WIFI_RETRY_MS = 30000;  // retry WiFi every 30 s if not connected

// ── Structs first — Arduino IDE auto-generates prototypes before any code,
//    so structs used in function signatures must be declared at the top. ───────

struct BeepPhase {
  uint32_t freq;
  uint32_t ms;
};

struct Button {
  uint8_t  pin;
  bool     lastReading;
  bool     stableState;
  uint32_t lastChangeMs;
};

// ── Pin definitions ───────────────────────────────────────────────────────────

const uint8_t SEG_PINS[7]  = {13, 19, 21, 17, 18, 14, 16}; // a, b, c, d, e, f, g

const uint8_t DIGIT_TENS   = 22;
const uint8_t DIGIT_UNITS  = 23;

const uint8_t BTN_INC      = 25;
const uint8_t BTN_DEC      = 32;
const uint8_t BTN_RST      = 33;

const uint8_t BUZZER_PIN   = 4;
const uint8_t IR_RECV_PIN  = 35;
const uint8_t FAN_PIN      = 2;    // PWM → 1kΩ → Q3 base (LOW at boot = fan off)

// ── IR remote key codes (NEC command byte) ────────────────────────────────────
// Default map = the common 17-key kit remote (HX1838 kits, address 0x00).
// If your remote differs, press its keys and watch the Serial Monitor —
// unknown codes are printed as  "IR unknown cmd=0x??"  → edit the values here.

const uint8_t IR_CMD_DIGIT[10] = {
  0x19, // 0
  0x45, // 1
  0x46, // 2
  0x47, // 3
  0x44, // 4
  0x40, // 5
  0x43, // 6
  0x07, // 7
  0x15, // 8
  0x09, // 9
};
const uint8_t IR_CMD_UP   = 0x18;   // ▲ arrow
const uint8_t IR_CMD_DOWN = 0x52;   // ▼ arrow
const uint8_t IR_CMD_STAR = 0x16;   // *  → fan off (0)
const uint8_t IR_CMD_HASH = 0x0D;   // #  → next multiple of 10, up to 100

const uint32_t IR_SHIFT_MS = 3000;  // a digit within this window shifts the previous one left

// ── 7-segment encoding ────────────────────────────────────────────────────────
// Common cathode: bit 1 = segment ON (GPIO HIGH), bit 0 = segment OFF (GPIO LOW)
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

const uint8_t SEG_F = 0x71;  // letter F (a, f, g, e) — "FF" means value 100

// ── Counter / fan value ───────────────────────────────────────────────────────

const int COUNT_MAX = 99;    // limit for +1/−1 steps (buttons and IR arrows)
const int COUNT_MIN =  0;
const int FAN_MAX   = 100;   // reachable only via the # key
int counter = 0;             // 0–100; doubles as fan speed in percent

// ── Buzzer — non-blocking AC-remote two-phase beep ────────────────────────────

const BeepPhase BEEP_SEQ[] = {
  {3800, 25},  // sharp attack
  {   0,  5},  // brief silence
  {2800, 60},  // warm body tone
};
const int BEEP_PHASES = sizeof(BEEP_SEQ) / sizeof(BEEP_SEQ[0]);

int      beepPhase   = -1;
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

// ── Fan PWM ───────────────────────────────────────────────────────────────────
// 25 kHz PWM — above audible range, so the fan doesn't whine.
// Duty maps 0–100 % → 0–255. Small fans may not start below ~30 %.

void applyFan() {
  static int lastVal = -1;
  if (counter == lastVal) return;
  lastVal = counter;
  uint32_t duty = (uint32_t)counter * 255 / FAN_MAX;
  ledcWrite(FAN_PIN, duty);
}

// ── WiFi + OTA (non-blocking) ─────────────────────────────────────────────────
// The controller never waits for WiFi: it starts an attempt, keeps running,
// and finishes OTA setup whenever the connection succeeds.

bool     otaReady        = false;
uint32_t lastWifiTryMs   = 0;

void otaSetup() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    // Quiesce the hardware during the flash write
    digitalWrite(DIGIT_TENS,  LOW);
    digitalWrite(DIGIT_UNITS, LOW);
    ledcWriteTone(BUZZER_PIN, 0);
    ledcWrite(FAN_PIN, 0);
    Serial.println("OTA: update starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: done, rebooting");
  });
  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
    static int lastPct = -1;
    int pct = (int)(prog * 100UL / total);
    if (pct / 10 != lastPct / 10) { Serial.printf("OTA: %d%%\n", pct); lastPct = pct; }
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("OTA error %u — device keeps old firmware\n", err);
  });

  ArduinoOTA.begin();
  otaReady = true;
  Serial.printf("OTA ready — hostname '%s', IP %s\n",
                OTA_HOSTNAME, WiFi.localIP().toString().c_str());
}

void wifiTick() {
  if (otaReady) {
    ArduinoOTA.handle();
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    otaSetup();
    return;
  }
  if (millis() - lastWifiTryMs >= WIFI_RETRY_MS) {
    lastWifiTryMs = millis();
    Serial.printf("WiFi: connecting to '%s'...\n", WIFI_SSID);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ── Button debounce ───────────────────────────────────────────────────────────

const uint32_t DEBOUNCE_MS = 50;

Button btnInc = {BTN_INC, HIGH, HIGH, 0};
Button btnDec = {BTN_DEC, HIGH, HIGH, 0};
Button btnRst = {BTN_RST, HIGH, HIGH, 0};

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

// ── IR remote handling ────────────────────────────────────────────────────────

uint32_t lastDigitPressMs = 0;  // when the last digit key was pressed (0 = none)

int digitFromCmd(uint8_t cmd) {
  for (int d = 0; d < 10; d++) {
    if (IR_CMD_DIGIT[d] == cmd) return d;
  }
  return -1;
}

void handleIR() {
  if (!IrReceiver.decode()) return;
  uint8_t cmd     = IrReceiver.decodedIRData.command;
  bool    repeat  = IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT;
  IrReceiver.resume();
  if (repeat) return;                 // ignore NEC held-key repeat frames

  uint32_t now = millis();

  if (cmd == IR_CMD_UP) {
    lastDigitPressMs = 0;             // command keys end any digit-entry sequence
    if (counter < COUNT_MAX) counter++;
    beepStart();
    Serial.printf("IR UP → %d\n", counter);
    return;
  }

  if (cmd == IR_CMD_DOWN) {
    lastDigitPressMs = 0;
    if (counter > COUNT_MIN) counter--;
    beepStart();
    Serial.printf("IR DOWN → %d\n", counter);
    return;
  }

  if (cmd == IR_CMD_STAR) {           // * = fan off
    lastDigitPressMs = 0;
    counter = 0;
    beepStart();
    Serial.println("IR * → 0 (fan off)");
    return;
  }

  if (cmd == IR_CMD_HASH) {           // # = next multiple of 10, max 100
    lastDigitPressMs = 0;
    counter = min(FAN_MAX, (counter / 10 + 1) * 10);
    beepStart();
    if (counter >= FAN_MAX) Serial.println("IR # → FF (full speed)");
    else                    Serial.printf("IR # → %d\n", counter);
    return;
  }

  int d = digitFromCmd(cmd);
  if (d < 0) {
    Serial.printf("IR unknown cmd=0x%02X\n", cmd);   // use this to remap keys
    return;
  }

  // Digit entry: show immediately on the right digit. A press within 3 s of
  // the previous digit shifts that digit to the left (1 then 8 → 18). A first
  // digit of 0 stays 0, so 0 then 8 → 8. After 3 s of silence, start fresh.
  if (lastDigitPressMs != 0 && now - lastDigitPressMs <= IR_SHIFT_MS) {
    counter = (counter % 10) * 10 + d;
  } else {
    counter = d;
  }
  lastDigitPressMs = now;
  beepStart();
  Serial.printf("IR digit %d → %d\n", d, counter);
}

// ── Display multiplexing ──────────────────────────────────────────────────────

const uint32_t MUX_US = 2000;   // 2 ms per digit → 250 Hz refresh, no flicker
uint32_t lastMuxUs = 0;
bool     showTens  = true;

void writePattern(uint8_t enc) {
  for (int i = 0; i < 7; i++) {
    digitalWrite(SEG_PINS[i], (enc >> i) & 1);
  }
}

void clearSegments() {
  for (int i = 0; i < 7; i++) digitalWrite(SEG_PINS[i], LOW);
}

void updateDisplay() {
  if (micros() - lastMuxUs < MUX_US) return;
  lastMuxUs = micros();

  digitalWrite(DIGIT_TENS,  LOW);
  digitalWrite(DIGIT_UNITS, LOW);
  clearSegments();

  uint8_t encTens, encUnits;
  if (counter >= 100) {                // "FF" = full speed
    encTens  = SEG_F;
    encUnits = SEG_F;
  } else {
    // Blank the leading zero: 0–9 show on the right digit only
    encTens  = (counter < 10) ? 0x00 : SEG_MAP[counter / 10];
    encUnits = SEG_MAP[counter % 10];
  }

  if (showTens) {
    writePattern(encTens);
    digitalWrite(DIGIT_TENS, HIGH);
  } else {
    writePattern(encUnits);
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

  // Fan PWM: 25 kHz, 8-bit — start with fan off
  ledcAttach(FAN_PIN, 25000, 8);
  ledcWrite(FAN_PIN, 0);

  // Wait for pull-ups to fully settle before first read
  delay(100);

  // Re-initialize buttons from actual pin state to prevent false boot triggers
  {
    bool inc0 = digitalRead(BTN_INC);
    bool dec0 = digitalRead(BTN_DEC);
    bool rst0 = digitalRead(BTN_RST);
    btnInc = {BTN_INC, inc0, inc0, 0};
    btnDec = {BTN_DEC, dec0, dec0, 0};
    btnRst = {BTN_RST, rst0, rst0, 0};
  }

  ledcAttach(BUZZER_PIN, 2800, 8);
  ledcWrite(BUZZER_PIN, 0);

  Serial.begin(115200);
  Serial.println("Boot OK");
  Serial.printf("Pin states: INC=%d DEC=%d RST=%d\n",
                digitalRead(BTN_INC), digitalRead(BTN_DEC), digitalRead(BTN_RST));
  Serial.println("(1=idle  0=stuck-low or pressed)");

  // Start WiFi in the background — wifiTick() finishes OTA setup once connected
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // keeps OTA discovery responsive
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTryMs = millis();
  Serial.printf("WiFi: connecting to '%s' (non-blocking)...\n", WIFI_SSID);

  IrReceiver.begin(IR_RECV_PIN, DISABLE_LED_FEEDBACK);
  Serial.printf("IR receiver listening on GPIO %d\n", IR_RECV_PIN);
  Serial.printf("Fan PWM on GPIO %d (25 kHz)\n", FAN_PIN);

  // ── Segment scan: light each GPIO one at a time for 800 ms ──────────────
  // Enable both digit drivers so you can see which physical segment lights up
  digitalWrite(DIGIT_TENS,  HIGH);
  digitalWrite(DIGIT_UNITS, HIGH);
  const char* segNames[7] = {"a(top)","b(upper-R)","c(lower-R)","d(bottom)","e(lower-L)","f(upper-L)","g(middle)"};
  for (int i = 0; i < 7; i++) {
    Serial.printf("SEG %s  GPIO %d\n", segNames[i], SEG_PINS[i]);
    digitalWrite(SEG_PINS[i], HIGH);
    delay(800);
    digitalWrite(SEG_PINS[i], LOW);
    delay(200);
  }
  digitalWrite(DIGIT_TENS,  LOW);
  digitalWrite(DIGIT_UNITS, LOW);
  Serial.println("Scan done — counter starting");
}

void loop() {
  updateDisplay();
  beepTick();
  handleIR();
  applyFan();
  wifiTick();

  if (checkPress(btnInc)) { if (counter < COUNT_MAX) counter++; beepStart(); Serial.printf("INC → %d\n", counter); }
  if (checkPress(btnDec)) { if (counter > COUNT_MIN) counter--; beepStart(); Serial.printf("DEC → %d\n", counter); }
  if (checkPress(btnRst)) { counter = COUNT_MIN; beepStart(); Serial.println("RST → 0"); }

  // Print raw pin readings every 2 s — press each button and watch these values
  static uint32_t lastDiagMs = 0;
  if (millis() - lastDiagMs >= 2000) {
    lastDiagMs = millis();
    Serial.printf("RAW INC=%d DEC=%d RST=%d  cnt=%d\n",
                  digitalRead(BTN_INC), digitalRead(BTN_DEC), digitalRead(BTN_RST), counter);
  }
}
