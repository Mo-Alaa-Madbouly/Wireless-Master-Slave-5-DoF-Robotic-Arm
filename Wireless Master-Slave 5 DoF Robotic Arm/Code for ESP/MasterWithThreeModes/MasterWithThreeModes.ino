#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─── Pin Definitions ────────────────────────────────────────
const int POT_PINS[]    = {32, 33, 34, 35, 39};
const int NUM_JOINTS    = 5;

const int SHIFT_BTN     = 25;   // toggles mode 1 ↔ mode 2
const int RECORD_BTN    = 26;   // (mode 2) save current position
const int PERFORM_BTN   = 27;   // (mode 2) replay recorded positions

// ─── ESP-NOW ────────────────────────────────────────────────
uint8_t RECEIVER_MAC[] = {0x44, 0x1D, 0x64,  0xBD, 0x7E, 0xA8};

// ─── Constants ──────────────────────────────────────────────
const int   MAX_POSITIONS       = 4;      // max storable waypoints
const int   POT_RAW_MIN         = 683;    // ~1/6 of 4095 (0.55 V)
const int   POT_RAW_MAX         = 3413;   // ~5/6 of 4095 (2.75 V)
const int   INTERP_STEPS        = 50;     // smoothing steps between waypoints
const int   INTERP_STEP_DELAY   = 20;     // ms between each interpolation step
const int   HOLD_DELAY          = 1000;   // ms to hold at each waypoint
const int   REALTIME_DELAY      = 100;    // ms between real-time sends
const int   Min_angle           = 3;      // degree (angle)
// ─── Data Structures ────────────────────────────────────────

// Packet sent to slave via ESP-NOW.
typedef struct {
  uint8_t joint[5];   // angles 0–180° for joints 1–5
} ArmPosition;

/** All state for mode-2 recording/playback. */
typedef struct {
  ArmPosition waypoints[MAX_POSITIONS];  // recorded snapshots
  int         count;                     // number of waypoints saved so far
  bool        isPerforming;              // true while replaying
} RecordSession;

// ─── Globals ────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);   // adjust address if needed (0x3F)

esp_now_peer_info_t peerInfo;

RecordSession session = { .count = 0, .isPerforming = false };

enum OperatingMode { MODE_REALTIME = 1, MODE_RECORD = 2, MODE_ROS2 = 3 };
OperatingMode currentMode = MODE_REALTIME;

// Debounce timestamps (ms)
unsigned long lastShiftPress   = 0;
unsigned long lastRecordPress  = 0;
unsigned long lastPerformPress = 0;
const unsigned long DEBOUNCE_MS = 300;

// ─── ESP-NOW Callback ────────────────────────────────────────
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Uncomment for debug:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAILED");
}

// ─── Helper: Read potentiometers → ArmPosition ──────────────
/**
 * Reads all 5 potentiometers and maps raw ADC values to 0–180°.
 * @ return  ArmPosition struct with current joint angles.
 */
int readStableADC(int pin, int samples = 32) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100); // let ADC settle between reads
  }
  return sum / samples;
}

ArmPosition readMasterPosition() {
  ArmPosition pos;

  int raw = readStableADC(POT_PINS[0], 32); // 0 ~ 4095
  pos.joint[0] = (uint8_t) constrain(
    map(raw, 0, 4095, 0, 270), 0, 175
  );
  raw = readStableADC(POT_PINS[1], 32);
  pos.joint[1] = 180 - (uint8_t) constrain(
    (map(raw, 0, 4095, 0, 270) - 55), 0, 175
  );
  raw = readStableADC(POT_PINS[2], 32);
  pos.joint[2] = (uint8_t) constrain(
    (map(raw, 0, 4095, 0, 270) + 10), 0, 175
  );
  raw = readStableADC(POT_PINS[3], 32);
  pos.joint[3] = (uint8_t) constrain(
    map(raw, 0, 4095, 0, 270) - 80, 0, 175
  );
  raw = readStableADC(POT_PINS[4], 32);
  pos.joint[4] = (uint8_t) constrain(
    (map(raw, 0, 4095, 0, 270) - 150)*1.2, 0, 175
  );

  return pos;
}

// ─── Helper: Send ArmPosition via ESP-NOW ───────────────────
/**
 * Transmits an ArmPosition struct to the slave ESP32.
 * @ param pos  Position to send.
 */
void sendPosition(const ArmPosition &pos) {
  esp_now_send(RECEIVER_MAC, (uint8_t *)&pos, sizeof(ArmPosition));
}

// ─── Helper: Interpolate & send between two waypoints ───────
/**
 * Smoothly moves from `from` to `to` in INTERP_STEPS steps,
 * sending each intermediate position to the slave.
 *
 * @param from  Starting joint angles.
 * @param to    Target joint angles.
 */
void interpolateTo(const ArmPosition &from, const ArmPosition &to) {
  for (int step = 1; step <= INTERP_STEPS; step++) {
    ArmPosition mid;
    for (int j = 0; j < NUM_JOINTS; j++) {
      // Linear interpolation: from + (to - from) * (step / INTERP_STEPS)
      mid.joint[j] = (uint8_t)(
  from.joint[j] + (float)(to.joint[j] - from.joint[j]) * step / INTERP_STEPS
      );
    }
    sendPosition(mid);
    delay(INTERP_STEP_DELAY);
  }
}

// ─── LCD Helpers ────────────────────────────────────────────
/** Clear LCD and print two rows. Pass "" for an empty row. */
void lcdPrint(const char *row0, const char *row1) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(row0);
  lcd.setCursor(0, 1); lcd.print(row1);
}

/** Update LCD to reflect current mode and record session state. */
void updateLCD() {
  if (currentMode == MODE_REALTIME) {
    lcdPrint("Mode 1: RealTime", "Mirroring...    ");
  } else if (currentMode == MODE_RECORD) {
    // Mode 2 — show stage
    if (session.isPerforming) {
      lcdPrint("Mode 2: Perform ", "Playing back... ");
    } else {
      char row1[17];
      snprintf(row1, sizeof(row1), "Pts: %d/%d  REC",
               session.count, MAX_POSITIONS);
      lcdPrint("Mode 2: Record  ", row1);
    }
  } else {
    // Mode 3 — ROS2 / MoveIt
    lcdPrint("Mode 3: ROS2    ", "Waiting...      ");
  }
}

// ─── Button: SHIFT ──────────────────────────────────────────
/**
 * Toggles between MODE_REALTIME and MODE_RECORD.
 * Resets the recording session when entering Mode 2.
 */
void handleShiftButton() {
  unsigned long now = millis();
  if (digitalRead(SHIFT_BTN) == LOW && (now - lastShiftPress) > DEBOUNCE_MS) {
    lastShiftPress = now;

    if (currentMode == MODE_REALTIME) {
      // 1 → 2
      currentMode = MODE_RECORD;
      session.count       = 0;
      session.isPerforming = false;
      Serial.println("[SHIFT] → Mode 2: Record/Playback");
    } else if (currentMode == MODE_RECORD) {
      // 2 → 3
      currentMode = MODE_ROS2;
      session.isPerforming = false;
      Serial.println("[SHIFT] → Mode 3: ROS2/MoveIt");
      Serial.println("[ROS2] Ready. Send: j0,j1,j2,j3,j4<LF>");
    } else {
      // 3 → 1
      currentMode = MODE_REALTIME;
      Serial.println("[SHIFT] → Mode 1: Real-Time");
    }
    updateLCD();
  }
}

// ─── Button: RECORD (Mode 2, Stage A) ───────────────────────
/**
 * Saves the current master arm position as the next waypoint.
 * Ignored if the session is full or playback is running.
 */
void handleRecordButton() {
  unsigned long now = millis();
  if (digitalRead(RECORD_BTN) == LOW && (now - lastRecordPress) > DEBOUNCE_MS) {
    lastRecordPress = now;

    if (currentMode != MODE_RECORD || session.isPerforming) return;

    if (session.count >= MAX_POSITIONS) {
      Serial.println("[RECORD] Buffer full!");
      lcdPrint("Mode 2: Record  ", "Buffer FULL!    ");
      delay(1000);
      updateLCD();
      return;
    }

    // Snapshot current position
    session.waypoints[session.count] = readMasterPosition();
    Serial.printf("[RECORD] Saved position %d: J1=%d J2=%d J3=%d J4=%d J5=%d\n",
      session.count + 1,
      session.waypoints[session.count].joint[0],
      session.waypoints[session.count].joint[1],
      session.waypoints[session.count].joint[2],
      session.waypoints[session.count].joint[3],
      session.waypoints[session.count].joint[4]);
    session.count++;

    // Brief confirmation on LCD
    char msg[17];
    snprintf(msg, sizeof(msg), "Saved %d/%d!     ", session.count, MAX_POSITIONS);
    lcd.setCursor(0, 1);
    lcd.print(msg);
    delay(500);
    updateLCD();
  }
}

// ─── Button: PERFORM (Mode 2, Stage B) ──────────────────────
/**
 * Replays all recorded waypoints in sequence with smooth interpolation.
 * Requires at least 2 waypoints to interpolate.
 */
void handlePerformButton() {
  unsigned long now = millis();
  if (digitalRead(PERFORM_BTN) == LOW && (now - lastPerformPress) > DEBOUNCE_MS) {
    lastPerformPress = now;

    if (currentMode != MODE_RECORD || session.isPerforming) return;

    if (session.count < 2) {
      Serial.println("[PERFORM] Need at least 2 positions.");
      lcdPrint("Mode 2: Perform ", "Need >= 2 pts!  ");
      delay(1500);
      updateLCD();
      return;
    }

    // ── Begin playback ──
    session.isPerforming = true;
    updateLCD();
    Serial.printf("[PERFORM] Replaying %d positions...\n", session.count);

    // Move to the first waypoint from wherever the arm currently is
    ArmPosition current = readMasterPosition();
    interpolateTo(current, session.waypoints[0]);
    delay(HOLD_DELAY);

    // Traverse through all recorded waypoints
    for (int i = 0; i < session.count - 1; i++) {
      Serial.printf("[PERFORM] Moving: waypoint %d → %d\n", i + 1, i + 2);
      interpolateTo(session.waypoints[i], session.waypoints[i + 1]);
      delay(HOLD_DELAY);     // pause at each waypoint
    }

    // Hold at the final position
    sendPosition(session.waypoints[session.count - 1]);
    Serial.println("[PERFORM] Playback complete.");

    session.isPerforming = false;
    updateLCD();
  }
}

// ─── Mode 1: Real-Time Loop ──────────────────────────────────
/** Reads master arm and immediately forwards position to slave. */
void runRealTimeMode() {
  static ArmPosition lastSent = {};
  static bool initialized = false;

  ArmPosition pos = readMasterPosition();

  if (!initialized) {
    sendPosition(pos);
    lastSent = pos;
    initialized = true;
    delay(REALTIME_DELAY);
    return;
  }

  bool changed = false;
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (abs((int)pos.joint[i] - (int)lastSent.joint[i]) >= Min_angle) { // 2° threshold
      changed = true;
      break;
    }
  }

  if (changed) {
    sendPosition(pos);
    lastSent = pos;
    Serial.printf("[RT] J1=%d J2=%d J3=%d J4=%d J5=%d\n",
      pos.joint[0], pos.joint[1], pos.joint[2], pos.joint[3], pos.joint[4]);
  }
  delay(REALTIME_DELAY);
}

// ─── Mode 3: ROS2 / MoveIt Serial Loop ──────────────────────
/**
 * Listens for joint-angle packets arriving from a ROS2/MoveIt node
 * over USB Serial in the format:
 *
 *   "j0,j1,j2,j3,j4\n"   (5 integers, 0–180 each)
 *
 * On success  → forwards the position via ESP-NOW and replies "OK:j0,j1,j2,j3,j4\n"
 * On failure  → replies "ERR\n"
 *
 * The ROS2 serial_bridge node should open the same port at 115200 baud
 * and publish/subscribe on the /arm_joint_states topic.
 */
void runROS2Mode() {
  if (!Serial.available()) return;

  String data = Serial.readStringUntil('\n');
  data.trim();

  // ── Parse ──────────────────────────────────────────────────
  int j[5];
  int count = sscanf(data.c_str(), "%d,%d,%d,%d,%d",
                     &j[0], &j[1], &j[2], &j[3], &j[4]);

  if (count != 5) {
    Serial.println("ERR");
    return;
  }

  // ── Validate & build packet ────────────────────────────────
  ArmPosition pos;
  for (int i = 0; i < NUM_JOINTS; i++) {
    pos.joint[i] = (uint8_t) constrain(j[i], 0, 180);
  }

  // ── Forward via ESP-NOW to slave ───────────────────────────
  sendPosition(pos);

  // ── Acknowledge to ROS2 node ───────────────────────────────
  Serial.printf("OK:%d,%d,%d,%d,%d\n",
                pos.joint[0], pos.joint[1], pos.joint[2],
                pos.joint[3], pos.joint[4]);

  // ── Update LCD with live joint values ─────────────────────
  char row1[17];
  snprintf(row1, sizeof(row1), "%3d%3d%3d%3d%3d",
           pos.joint[0], pos.joint[1], pos.joint[2],
           pos.joint[3], pos.joint[4]);
  lcdPrint("Mode 3: ROS2    ", row1);
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // ── ADC ──
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // ── Buttons ──
  pinMode(SHIFT_BTN,   INPUT_PULLUP);
  pinMode(RECORD_BTN,  INPUT_PULLUP);
  pinMode(PERFORM_BTN, INPUT_PULLUP);

  // ── LCD ──
  lcd.init();
  lcd.backlight();
  lcdPrint("Robot Arm v2.0  ", "Initializing... ");

  // ── WiFi + ESP-NOW ──
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    lcdPrint("ESP-NOW FAILED  ", "Check MAC addr  ");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    lcdPrint("Peer add FAILED ", "Check receiver  ");
    while (true) delay(1000);
  }

  Serial.println("Master controller ready.");
  updateLCD();   // show Mode 1 on LCD
}

// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  // Always check the mode-shift button
  handleShiftButton();

  if (currentMode == MODE_REALTIME) {
    // ── Mode 1: stream master → slave continuously ──
    runRealTimeMode();

  } else if (currentMode == MODE_RECORD) {
    // ── Mode 2: handle record & perform buttons ──
    if (!session.isPerforming) {
      // Only poll buttons when not mid-playback
      handleRecordButton();
      handlePerformButton();
    }
    // No delay here — playback is blocking inside handlePerformButton

  } else {
    // ── Mode 3: receive from ROS2/MoveIt and forward via ESP-NOW ──
    runROS2Mode();
  }
}