#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── PCA9685 Setup ──────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // default I²C addr

// Servo PWM pulse range — tune these to match your servos
// Standard servos: ~500 µs (0°) to ~2400 µs (180°)
// At 50 Hz, one period = 20,000 µs → 4096 ticks
// tick = µs * 4096 / 20000
const int SERVO_MIN_TICK = 102;   // ≈ 500 µs  → 0°
const int SERVO_MAX_TICK = 492;   // ≈ 2400 µs → 180°
const int SERVO_FREQ_HZ  = 50;    // standard servo PWM frequency
const int NUM_JOINTS     = 5;     // must match master

// ─── ESP-NOW ────────────────────────────────────────────────
// Must match the master's ArmPosition struct exactly
typedef struct {
  uint8_t joint[5];   // angles 0–180° for joints 1–5
} ArmPosition;

// ─── State ──────────────────────────────────────────────────
volatile bool      newDataReady = false;
volatile ArmPosition pendingPos  = {};
ArmPosition        currentPos   = {};

// ─── Helper: angle (0–180°) → PCA9685 tick count ───────────
/**
 * Maps a joint angle to the correct PWM tick value for the PCA9685.
 * @param angle  Desired angle in degrees (0–180).
 * @return       Tick count to pass to setPWM().
 */
uint16_t angleToPwmTick(uint8_t angle) {
  return (uint16_t) map(angle, 0, 180, SERVO_MIN_TICK, SERVO_MAX_TICK);
}

// ─── Helper: move one servo channel ─────────────────────────
/**
 * Commands a single PCA9685 channel to the given angle.
 * @ param channel  PCA9685 output channel (0–4 for joints 1–5).
 * @ param angle    Target angle in degrees (0–180).
 */
void setServoAngle(uint8_t channel, uint8_t angle) {
  uint16_t tick = angleToPwmTick(constrain(angle, 0, 180));
  pwm.setPWM(channel, 0, tick);
}

// ─── Helper: apply a full ArmPosition to all servos ─────────
/**
 * Writes all 5 joint angles from an ArmPosition struct to the PCA9685.
 * @param pos  Position to apply.
 */
void applyPosition(const ArmPosition &pos) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    setServoAngle(i, pos.joint[i]);
  }
}

// ─── ESP-NOW Receive Callback ────────────────────────────────
/**
 * Called by the ESP-NOW stack on every received packet.
 * Copies the incoming data into pendingPos and sets the flag.
 * NOTE: Keep this ISR-like function fast — no Serial, no delays.
 *
 * @param mac   MAC address of the sender (master).
 * @param data  Raw received bytes.
 * @param len   Length of received data in bytes.
 */
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ArmPosition)) return;
  memcpy((void *)&pendingPos, data, sizeof(ArmPosition));
  newDataReady = true;
}
// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("Slave arm booting...");

  // ── I²C + PCA9685 ──
  Wire.begin(21, 22);           // SDA=21, SCL=22
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // calibrate to your board (typ. 25–27 MHz)
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  // Centre all servos at startup (90°)
  for (int i = 0; i < NUM_JOINTS; i++) {
    currentPos.joint[i] = 90;
  }
  applyPosition(currentPos);
  Serial.println("Servos centred at 90°.");

  // ── WiFi + ESP-NOW ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();            // ensure not associated to any AP

  // Print this board's MAC so you can paste it into the master
  Serial.print("Slave MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED — halting.");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("Slave ready — waiting for master packets.");
}

// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  // Check if the ESP-NOW callback deposited a new position
  if (newDataReady) {
    newDataReady = false;

    // Copy from volatile buffer under atomic-safe window
    ArmPosition pos;
    noInterrupts();
    memcpy(&pos, (const void *)&pendingPos, sizeof(ArmPosition));
    interrupts();

    // Apply to all servos
    applyPosition(pos);
    currentPos = pos;

    Serial.printf("[RX] J1=%d J2=%d J3=%d J4=%d J5=%d\n",
      pos.joint[0], pos.joint[1], pos.joint[2],
      pos.joint[3], pos.joint[4]);
  }

  // Nothing else to do — servo commands are fire-and-forget via I²C.
  // Add a small yield so the ESP32 WiFi/ESP-NOW stack can run.
  delay(10);
}