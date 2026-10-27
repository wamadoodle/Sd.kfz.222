#include <Arduino.h>
#include <ESP32Servo.h>
#include <NimBLEDevice.h>
#include <VL53L1X.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <string>

void setup();
void loop();

namespace Config {
constexpr uint8_t LEFT_IN1 = 26;
constexpr uint8_t LEFT_IN2 = 27;
constexpr uint8_t RIGHT_IN1 = 16;
constexpr uint8_t RIGHT_IN2 = 17;
constexpr uint8_t TURRET_SERVO_PIN = 18;
constexpr uint8_t LASER_PIN = 23;
constexpr uint8_t BUZZER_PIN = 19;
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

constexpr uint8_t BUZZER_PWM_CHANNEL = 4;
constexpr uint8_t MOTOR_DEADBAND = 8;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t LASER_MAX_ON_MS = 5000;
constexpr int TURRET_MIN_ANGLE = 0;
constexpr int TURRET_MAX_ANGLE = 180;
constexpr uint16_t SENSOR_TO_TIP_OFFSET_MM = 185;
constexpr uint16_t FAR_ZONE_CLEARANCE_MM = 1030;
constexpr uint16_t REACTIVE_TRIGGER_CLEARANCE_MM = 750;
constexpr uint16_t HARD_COLLISION_CLEARANCE_MM = 150;
constexpr uint16_t HARD_COLLISION_RELEASE_CLEARANCE_MM = 190;
constexpr uint16_t FAR_ZONE_RAW_MM = SENSOR_TO_TIP_OFFSET_MM + FAR_ZONE_CLEARANCE_MM;
constexpr uint16_t REACTIVE_TRIGGER_RAW_MM =
  SENSOR_TO_TIP_OFFSET_MM + REACTIVE_TRIGGER_CLEARANCE_MM;
constexpr uint16_t HARD_COLLISION_RAW_MM =
  SENSOR_TO_TIP_OFFSET_MM + HARD_COLLISION_CLEARANCE_MM;
constexpr uint16_t HARD_COLLISION_RELEASE_RAW_MM =
  SENSOR_TO_TIP_OFFSET_MM + HARD_COLLISION_RELEASE_CLEARANCE_MM;
constexpr uint32_t ROUTE_SCAN_SETTLE_MS = 250;
constexpr uint32_t ROUTE_SCAN_SAMPLE_TIMEOUT_MS = 750;
constexpr uint8_t ROUTE_SCAN_SAMPLE_COUNT = 3;
constexpr uint32_t TOF_FRESHNESS_MS = 200;
constexpr uint16_t ROUTE_MIN_CLEARANCE_RAW_MM = REACTIVE_TRIGGER_RAW_MM;
constexpr uint16_t OBSTACLE_ALARM_FREQUENCY_HZ = 2200;
constexpr uint32_t OBSTACLE_ALARM_DURATION_MS = 350;
constexpr uint32_t AUTO_TURN_MS_PER_DEG = 7;
constexpr uint32_t AUTO_ADVANCE_MS = 400;
constexpr uint32_t ROUTE_RETRY_INTERVAL_MS = 1000;
constexpr size_t MAX_UPLOADED_MELODY_NOTES = 256;
constexpr uint16_t MIN_UPLOADED_NOTE_DURATION_MS = 20;
constexpr uint16_t MAX_UPLOADED_NOTE_DURATION_MS = 5000;
constexpr uint16_t MAX_UPLOADED_NOTE_FREQUENCY_HZ = 4000;
constexpr char BLE_NAME[] = "SdKfz222";
}

constexpr int ROUTE_SCAN_ANGLES[] = {
    90, 75, 60, 45, 30, 15, 0, 15, 30, 45,
    60, 75, 90, 105, 120, 135, 150, 165, 180};
constexpr size_t ROUTE_SCAN_ANGLE_COUNT =
    sizeof(ROUTE_SCAN_ANGLES) / sizeof(ROUTE_SCAN_ANGLES[0]);

constexpr char BLE_SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char BLE_RX_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char BLE_TX_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

#include "PanzerliedMelody.h"

struct Command {
  char text[64];
};

enum class RouteState : uint8_t {
  Patrol,
  Scanning,
  RouteReady,
  Centering,
  Turning,
  TurnSettling,
  Advancing,
  WaitingForClear,
  NoRoute,
  Hold
};

class Motor {
public:
  Motor(uint8_t in1, uint8_t in2) : in1_(in1), in2_(in2) {}

  void begin() {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
  }

  void setDirection(int8_t direction) {
    if (direction == 0) {
      stop();
      return;
    }

    digitalWrite(in1_, direction > 0 ? HIGH : LOW);
    digitalWrite(in2_, direction > 0 ? LOW : HIGH);
  }

  void stop() {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
  }

private:
  uint8_t in1_;
  uint8_t in2_;
};

Motor leftMotor(Config::LEFT_IN1, Config::LEFT_IN2);
Motor rightMotor(Config::RIGHT_IN1, Config::RIGHT_IN2);
Servo turretServo;
VL53L1X tof;
NimBLECharacteristic *txCharacteristic = nullptr;
QueueHandle_t commandQueue = nullptr;

volatile uint16_t latestRangeMm = 0xFFFF;
volatile uint32_t latestRangeMs = 0;
volatile bool tofAvailable = false;
volatile bool bleConnected = false;
bool autonomousMode = false;
uint32_t lastCommandMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSerialRangeMs = 0;
bool motorsEnabledByCommand = false;
bool melodyPlaying = false;
bool melodyResting = false;
volatile bool obstaclePanEnabled = false;
bool turretPilotEnabled = false;
bool hardCollisionActive = false;
bool laserEnabled = false;
bool obstacleAlarmActive = false;
uint32_t obstacleAlarmStartedMs = 0;
uint32_t laserTurnedOnMs = 0;
RouteState routeState = RouteState::Patrol;
int turretPanAngle = 90;
uint32_t lastTurretPanStepMs = 0;
size_t routeScanAngleIndex = 0;
uint8_t routeScanValidSamples = 0;
uint32_t routeScanRangeSum = 0;
uint32_t lastRouteSampleMs = 0;
int bestRouteAngle = 90;
uint16_t bestRouteRangeMm = 0;
int routeTurnDirection = 0;
uint32_t routeActionStartedMs = 0;
uint32_t routeActionDurationMs = 0;
size_t melodyIndex = 0;
uint32_t noteStartedMs = 0;
Note uploadedMelody[Config::MAX_UPLOADED_MELODY_NOTES]{};
size_t uploadedMelodyLength = 0;
size_t uploadExpectedNotes = 0;
size_t uploadReceivedNotes = 0;
bool melodyUploadInProgress = false;
bool uploadedMelodyAvailable = false;
bool melodyUsingUploaded = false;

class ServerCallbacks final : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    bleConnected = true;
  }

  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    bleConnected = false;
    obstaclePanEnabled = false;
    NimBLEDevice::getAdvertising()->start();
  }
};

class RxCallbacks final : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
    const std::string value = characteristic->getValue();
    if (value.empty() || value.size() >= sizeof(Command) || commandQueue == nullptr) {
      return;
    }

    Command command{};
    size_t length = 0;
    for (size_t index = 0; index < value.size();) {
      const uint8_t first = static_cast<uint8_t>(value[index]);
      if (index + 2 < value.size() && first == 0xE2) {
        const uint8_t second = static_cast<uint8_t>(value[index + 1]);
        const uint8_t third = static_cast<uint8_t>(value[index + 2]);
        if ((second == 0x80 && (third == 0x90 || third == 0x91 ||
                                third == 0x92 || third == 0x93 || third == 0x94)) ||
            (second == 0x88 && third == 0x92)) {
          command.text[length++] = '-';
          index += 3;
          continue;
        }
      }
      command.text[length++] = value[index++];
    }
    while (length > 0 && (command.text[length - 1] == '\r' || command.text[length - 1] == '\n')) {
      command.text[--length] = '\0';
    }
    if (length > 0) {
      if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
        Serial.println("BLE command queue full; command dropped");
      }
    }
  }
};

void stopDrive() {
  leftMotor.stop();
  rightMotor.stop();
  motorsEnabledByCommand = false;
}

void notifyMidiUploadStatus(bool success, size_t received, size_t expected) {
  if (!bleConnected || txCharacteristic == nullptr) {
    return;
  }
  char message[40];
  if (success) {
    snprintf(message, sizeof(message), "MU,OK,%u",
             static_cast<unsigned>(received));
  } else {
    snprintf(message, sizeof(message), "MU,ERR,%u,%u",
             static_cast<unsigned>(received), static_cast<unsigned>(expected));
  }
  txCharacteristic->setValue(message);
  txCharacteristic->notify();
}

void setLaser(bool enabled) {
  if (enabled && !laserEnabled) {
    laserTurnedOnMs = millis();
  }
  laserEnabled = enabled;
  digitalWrite(Config::LASER_PIN, enabled ? HIGH : LOW);
}

size_t currentMelodyLength();
Note readCurrentMelodyNote(size_t index);

void triggerObstacleAlarm() {
  obstacleAlarmActive = true;
  obstacleAlarmStartedMs = millis();
  tone(Config::BUZZER_PIN, Config::OBSTACLE_ALARM_FREQUENCY_HZ);
}

void updateObstacleAlarm() {
  if (!obstacleAlarmActive ||
      static_cast<uint32_t>(millis() - obstacleAlarmStartedMs) <
          Config::OBSTACLE_ALARM_DURATION_MS) {
    return;
  }

  obstacleAlarmActive = false;
  const uint32_t alarmDuration = millis() - obstacleAlarmStartedMs;
  if (melodyPlaying) {
    noteStartedMs += alarmDuration;
    if (!melodyResting) {
      const Note note = readCurrentMelodyNote(melodyIndex);
      if (note.frequencyHz != 0) {
        tone(Config::BUZZER_PIN, note.frequencyHz);
      } else {
        noTone(Config::BUZZER_PIN);
      }
    } else {
      noTone(Config::BUZZER_PIN);
    }
  } else {
    noTone(Config::BUZZER_PIN);
  }
}

void startMelody() {
  melodyUsingUploaded = uploadedMelodyAvailable;
  if (currentMelodyLength() == 0) {
    melodyPlaying = false;
    return;
  }
  const Note firstNote = readCurrentMelodyNote(0);
  if (firstNote.durationMs == 0) {
    melodyPlaying = false;
    return;
  }
  melodyIndex = 0;
  melodyPlaying = true;
  melodyResting = false;
  noteStartedMs = millis();
  if (obstacleAlarmActive) {
    return;
  }
  if (firstNote.frequencyHz != 0) {
    tone(Config::BUZZER_PIN, firstNote.frequencyHz);
  } else {
    noTone(Config::BUZZER_PIN);
  }
}

void stopMelody() {
  melodyPlaying = false;
  melodyResting = false;
  if (!obstacleAlarmActive) {
    noTone(Config::BUZZER_PIN);
  }
}

size_t currentMelodyLength() {
  return melodyUsingUploaded ? uploadedMelodyLength : MELODY_LENGTH;
}

Note readCurrentMelodyNote(size_t index) {
  return melodyUsingUploaded ? uploadedMelody[index] : readMelodyNote(index);
}

void updateMelody() {
  if (!melodyPlaying || obstacleAlarmActive) {
    return;
  }
  const uint32_t now = millis();
  const Note currentNote = readCurrentMelodyNote(melodyIndex);
  if (melodyResting) {
    if (static_cast<uint32_t>(now - noteStartedMs) < currentNote.restMs) {
      return;
    }
  } else {
    if (static_cast<uint32_t>(now - noteStartedMs) < currentNote.durationMs) {
      return;
    }

    noTone(Config::BUZZER_PIN);
    if (currentNote.restMs != 0) {
      melodyResting = true;
      noteStartedMs = now;
      return;
    }
  }

  ++melodyIndex;
  if (melodyIndex >= currentMelodyLength()) {
    melodyIndex = 0;
  }

  const Note nextNote = readCurrentMelodyNote(melodyIndex);
  melodyResting = false;
  noteStartedMs = now;
  if (nextNote.frequencyHz != 0) {
    tone(Config::BUZZER_PIN, nextNote.frequencyHz);
  } else {
    noTone(Config::BUZZER_PIN);
  }
}

void startTofTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t pollPeriod = pdMS_TO_TICKS(50);
  for (;;) {
    const uint16_t range = tof.read();
    if (tof.timeoutOccurred()) {
      latestRangeMm = 0xFFFF;
    } else {
      latestRangeMm = range;
    }
    latestRangeMs = millis();
    vTaskDelayUntil(&lastWake, pollPeriod);
  }
}

void beginRouteScan(uint32_t now) {
  stopDrive();
  triggerObstacleAlarm();
  routeState = RouteState::Scanning;
  routeScanAngleIndex = 0;
  routeScanValidSamples = 0;
  routeScanRangeSum = 0;
  bestRouteRangeMm = 0;
  bestRouteAngle = 90;
  turretPanAngle = ROUTE_SCAN_ANGLES[routeScanAngleIndex];
  turretServo.write(turretPanAngle);
  lastTurretPanStepMs = now;
  lastRouteSampleMs = latestRangeMs;
  Serial.printf("Obstacle scan started at %d degrees\n", turretPanAngle);
}

void beginRouteAdvance(uint32_t now) {
  if (!autonomousMode &&
      (!bleConnected || static_cast<uint32_t>(now - lastCommandMs) >
                            Config::COMMAND_TIMEOUT_MS)) {
    stopDrive();
    routeState = RouteState::RouteReady;
    return;
  }

  leftMotor.setDirection(1);
  rightMotor.setDirection(1);
  motorsEnabledByCommand = true;
  routeState = RouteState::Advancing;
  routeActionStartedMs = now;
}

void beginRouteTurn(uint32_t now) {
  if (bestRouteRangeMm <= Config::ROUTE_MIN_CLEARANCE_RAW_MM) {
    stopDrive();
    routeState = RouteState::NoRoute;
    routeActionStartedMs = now;
    return;
  }

  if (!autonomousMode &&
      (!bleConnected || static_cast<uint32_t>(now - lastCommandMs) >
                            Config::COMMAND_TIMEOUT_MS)) {
    stopDrive();
    routeState = RouteState::WaitingForClear;
    routeActionStartedMs = now;
    return;
  }

  turretPanAngle = 90;
  turretServo.write(turretPanAngle);
  lastTurretPanStepMs = now;
  routeTurnDirection = bestRouteAngle > 90 ? 1 : (bestRouteAngle < 90 ? -1 : 0);
  routeActionStartedMs = now;
  routeState = RouteState::Centering;
}

void updateTurretPan() {
  const uint32_t now = millis();
  if ((!autonomousMode && (!obstaclePanEnabled || !bleConnected)) ||
      turretPilotEnabled) {
    if (routeState != RouteState::Patrol && routeState != RouteState::Hold) {
      stopDrive();
      routeState = RouteState::Patrol;
    }
  }

  if (!tofAvailable) {
    if (autonomousMode) {
      stopDrive();
    }
    return;
  }

  const uint16_t range = latestRangeMm;
  const bool rangeFresh =
      range != 0xFFFF &&
      static_cast<uint32_t>(now - latestRangeMs) <= Config::TOF_FRESHNESS_MS;
  if (rangeFresh) {
    if (range < Config::HARD_COLLISION_RAW_MM) {
      hardCollisionActive = true;
    } else if (hardCollisionActive &&
               range >= Config::HARD_COLLISION_RELEASE_RAW_MM) {
      hardCollisionActive = false;
    }

  }

  if (hardCollisionActive && routeState != RouteState::Turning) {
    stopDrive();
  }

  if (autonomousMode && turretPilotEnabled) {
    stopDrive();
    return;
  }

  const bool scanAllowed =
      autonomousMode || (obstaclePanEnabled && bleConnected);
  if (hardCollisionActive && scanAllowed &&
      (routeState == RouteState::Advancing ||
       routeState == RouteState::RouteReady)) {
    beginRouteScan(now);
    return;
  }

  switch (routeState) {
  case RouteState::Patrol:
    if (rangeFresh && range <= Config::REACTIVE_TRIGGER_RAW_MM && scanAllowed) {
      beginRouteScan(now);
      return;
    }
    if (autonomousMode) {
      if (!rangeFresh || hardCollisionActive) {
        stopDrive();
      } else {
        leftMotor.setDirection(1);
        rightMotor.setDirection(1);
        motorsEnabledByCommand = true;
      }
    }
    return;

  case RouteState::Scanning:
    stopDrive();
    if (static_cast<uint32_t>(now - lastTurretPanStepMs) <
        Config::ROUTE_SCAN_SETTLE_MS) {
      return;
    }
    if (rangeFresh && latestRangeMs != lastRouteSampleMs) {
      lastRouteSampleMs = latestRangeMs;
      routeScanRangeSum += range;
      ++routeScanValidSamples;
    }
    if (routeScanValidSamples < Config::ROUTE_SCAN_SAMPLE_COUNT &&
        static_cast<uint32_t>(now - lastTurretPanStepMs) <
            Config::ROUTE_SCAN_SETTLE_MS +
                Config::ROUTE_SCAN_SAMPLE_TIMEOUT_MS) {
      return;
    }
    if (routeScanValidSamples != 0) {
      const uint16_t averageRange = static_cast<uint16_t>(
          routeScanRangeSum / routeScanValidSamples);
      if (averageRange > bestRouteRangeMm) {
        bestRouteRangeMm = averageRange;
        bestRouteAngle = turretPanAngle;
      }
    }
    ++routeScanAngleIndex;
    if (routeScanAngleIndex >= ROUTE_SCAN_ANGLE_COUNT) {
      Serial.printf("Obstacle scan complete: best=%d degrees, range=%u mm\n",
                    bestRouteAngle, bestRouteRangeMm);
      routeState = RouteState::RouteReady;
      return;
    }
    turretPanAngle = ROUTE_SCAN_ANGLES[routeScanAngleIndex];
    turretServo.write(turretPanAngle);
    lastTurretPanStepMs = now;
    routeScanValidSamples = 0;
    routeScanRangeSum = 0;
    lastRouteSampleMs = latestRangeMs;
    return;

  case RouteState::RouteReady:
    beginRouteTurn(now);
    return;

  case RouteState::Centering:
    if (static_cast<uint32_t>(now - routeActionStartedMs) <
        Config::ROUTE_SCAN_SETTLE_MS) {
      return;
    }
    if (routeTurnDirection == 0) {
      routeState = RouteState::TurnSettling;
      routeActionStartedMs = now;
      return;
    }
    if (!autonomousMode &&
        (!bleConnected || static_cast<uint32_t>(now - lastCommandMs) >
                              Config::COMMAND_TIMEOUT_MS)) {
      stopDrive();
      routeState = RouteState::WaitingForClear;
      routeActionStartedMs = now;
      return;
    }
    {
      const int8_t turnDirection = routeTurnDirection > 0 ? 1 : -1;
      leftMotor.setDirection(turnDirection);
      rightMotor.setDirection(-turnDirection);
      motorsEnabledByCommand = true;
      routeActionDurationMs =
          static_cast<uint32_t>(abs(bestRouteAngle - 90)) *
          Config::AUTO_TURN_MS_PER_DEG;
      routeActionStartedMs = now;
      routeState = RouteState::Turning;
    }
    return;

  case RouteState::Turning:
    if (!bleConnected && !autonomousMode) {
      stopDrive();
      routeState = RouteState::Hold;
      return;
    }
    if (static_cast<uint32_t>(now - routeActionStartedMs) >=
        routeActionDurationMs) {
      stopDrive();
      routeState = RouteState::TurnSettling;
      routeActionStartedMs = now;
    }
    return;

  case RouteState::TurnSettling:
    if (!rangeFresh ||
        static_cast<int32_t>(latestRangeMs - routeActionStartedMs) <= 0) {
      if (static_cast<uint32_t>(now - routeActionStartedMs) >= 500) {
        routeState = RouteState::NoRoute;
        routeActionStartedMs = now;
      }
      return;
    }
    if (range <= Config::REACTIVE_TRIGGER_RAW_MM || hardCollisionActive) {
      beginRouteScan(now);
      return;
    }
    beginRouteAdvance(now);
    return;

  case RouteState::Advancing:
    if (!bleConnected && !autonomousMode) {
      stopDrive();
      routeState = RouteState::Hold;
      return;
    }
    if (rangeFresh && range <= Config::REACTIVE_TRIGGER_RAW_MM) {
      beginRouteScan(now);
      return;
    }
    if (static_cast<uint32_t>(now - routeActionStartedMs) >=
        Config::AUTO_ADVANCE_MS) {
      stopDrive();
      routeState = RouteState::Patrol;
    }
    return;

  case RouteState::WaitingForClear:
    if (rangeFresh && range > Config::FAR_ZONE_RAW_MM) {
      routeState = RouteState::Patrol;
      return;
    }
    if (static_cast<uint32_t>(now - routeActionStartedMs) >=
        Config::ROUTE_RETRY_INTERVAL_MS) {
      beginRouteScan(now);
    }
    return;

  case RouteState::NoRoute:
    if (rangeFresh && range > Config::FAR_ZONE_RAW_MM) {
      routeState = RouteState::Patrol;
      return;
    }
    if (static_cast<uint32_t>(now - routeActionStartedMs) >=
        Config::ROUTE_RETRY_INTERVAL_MS) {
      beginRouteScan(now);
    }
    return;

  case RouteState::Hold:
    stopDrive();
    return;
  }
}

void processCommand(const char *packet) {
  if (strcmp(packet, "S") == 0) {
    autonomousMode = false;
    turretPilotEnabled = false;
    stopDrive();
    setLaser(false);
    routeState = RouteState::Hold;
    lastCommandMs = millis();
    return;
  }

  int consumed = 0;
  int value = 0;

  if (strcmp(packet, "MODE,RC") == 0) {
    autonomousMode = false;
    turretPilotEnabled = false;
    stopDrive();
    routeState = RouteState::Patrol;
    lastCommandMs = millis();
    Serial.println("RC mode enabled");
    return;
  }
  if (strcmp(packet, "MODE,AUTO") == 0) {
    if (!tofAvailable) {
      Serial.println("Auto mode rejected: VL53L1X unavailable");
      return;
    }
    autonomousMode = true;
    turretPilotEnabled = false;
    stopDrive();
    turretPanAngle = 90;
    turretServo.write(turretPanAngle);
    routeState = RouteState::Patrol;
    lastCommandMs = millis();
    Serial.println("Auto mode enabled");
    return;
  }

  consumed = 0;
  if (sscanf(packet, "TP,%d%n", &value, &consumed) == 1 &&
      packet[consumed] == '\0' && (value == 0 || value == 1)) {
    if (!autonomousMode) {
      Serial.println("Turret pilot command ignored outside Auto mode");
      return;
    }
    turretPilotEnabled = value == 1;
    stopDrive();
    routeState = RouteState::Patrol;
    if (!turretPilotEnabled) {
      turretPanAngle = 90;
      turretServo.write(turretPanAngle);
    }
    lastCommandMs = millis();
    Serial.println(turretPilotEnabled ? "Turret pilot enabled"
                                      : "Automatic turret scan resumed");
    return;
  }

  if (sscanf(packet, "A,%d%n", &value, &consumed) == 1 &&
      packet[consumed] == '\0' && (value == 0 || value == 1)) {
    if (value == 1 && !tofAvailable) {
      Serial.println("Autonomous mode rejected: VL53L1X unavailable");
      return;
    }
    autonomousMode = value == 1;
    turretPilotEnabled = false;
    stopDrive();
    routeState = RouteState::Patrol;
    if (autonomousMode) {
      turretPanAngle = 90;
      turretServo.write(turretPanAngle);
    }
    lastCommandMs = millis();
    Serial.println(autonomousMode ? "Autonomous mode enabled" : "Manual mode enabled");
    return;
  }

  consumed = 0;
  if (sscanf(packet, "P,%d%n", &value, &consumed) == 1 &&
      packet[consumed] == '\0' && (value == 0 || value == 1)) {
    obstaclePanEnabled = value == 1;
    if (!obstaclePanEnabled && !autonomousMode &&
        routeState != RouteState::Patrol &&
        routeState != RouteState::Hold) {
      stopDrive();
      routeState = RouteState::Patrol;
    }
    lastCommandMs = millis();
    Serial.println(obstaclePanEnabled
                       ? "Obstacle-triggered turret pan enabled"
                       : "Obstacle-triggered turret pan disabled");
    return;
  }

  size_t uploadCount = 0;
  consumed = 0;
  if (sscanf(packet, "MU,BEGIN,%zu%n", &uploadCount, &consumed) == 1 &&
      packet[consumed] == '\0') {
    if (uploadCount == 0 || uploadCount > Config::MAX_UPLOADED_MELODY_NOTES) {
      Serial.printf("MIDI upload rejected: note count %u outside 1..%u\n",
                    static_cast<unsigned>(uploadCount),
                    static_cast<unsigned>(Config::MAX_UPLOADED_MELODY_NOTES));
      melodyUploadInProgress = false;
      notifyMidiUploadStatus(false, 0, uploadCount);
      return;
    }
    stopMelody();
    uploadedMelodyAvailable = false;
    melodyUploadInProgress = true;
    uploadExpectedNotes = uploadCount;
    uploadReceivedNotes = 0;
    Serial.printf("MIDI upload started: %u notes\n",
                  static_cast<unsigned>(uploadExpectedNotes));
    return;
  }

  size_t noteIndex = 0;
  int frequency = 0;
  int duration = 0;
  consumed = 0;
  if (sscanf(packet, "MU,N,%zu,%d,%d%n", &noteIndex, &frequency, &duration,
             &consumed) == 3 &&
      packet[consumed] == '\0') {
    if (!melodyUploadInProgress || noteIndex != uploadReceivedNotes ||
        noteIndex >= uploadExpectedNotes || frequency < 0 ||
        frequency > Config::MAX_UPLOADED_NOTE_FREQUENCY_HZ ||
        (frequency != 0 && frequency < 20) ||
        duration < Config::MIN_UPLOADED_NOTE_DURATION_MS ||
        duration > Config::MAX_UPLOADED_NOTE_DURATION_MS) {
      Serial.printf("MIDI note rejected at index %u\n",
                    static_cast<unsigned>(noteIndex));
      melodyUploadInProgress = false;
      notifyMidiUploadStatus(false, uploadReceivedNotes, uploadExpectedNotes);
      return;
    }
    uploadedMelody[noteIndex] = {
        static_cast<uint16_t>(frequency), static_cast<uint16_t>(duration), 0};
    ++uploadReceivedNotes;
    return;
  }

  if (strcmp(packet, "MU,END") == 0) {
    if (!melodyUploadInProgress || uploadReceivedNotes != uploadExpectedNotes) {
      Serial.printf("MIDI upload incomplete: received %u of %u notes\n",
                    static_cast<unsigned>(uploadReceivedNotes),
                    static_cast<unsigned>(uploadExpectedNotes));
      notifyMidiUploadStatus(false, uploadReceivedNotes, uploadExpectedNotes);
      melodyUploadInProgress = false;
      return;
    }
    uploadedMelodyLength = uploadExpectedNotes;
    uploadedMelodyAvailable = true;
    melodyUploadInProgress = false;
    melodyUsingUploaded = true;
    Serial.printf("MIDI upload complete: %u notes ready; send M,1 to play\n",
                  static_cast<unsigned>(uploadedMelodyLength));
    notifyMidiUploadStatus(true, uploadedMelodyLength, uploadedMelodyLength);
    return;
  }

  if (strcmp(packet, "MU,CANCEL") == 0) {
    melodyUploadInProgress = false;
    uploadExpectedNotes = 0;
    uploadReceivedNotes = 0;
    Serial.println("MIDI upload cancelled");
    return;
  }

  int throttle = 0;
  int steering = 0;
  consumed = 0;
  if (sscanf(packet, "J,%d,%d%n", &throttle, &steering, &consumed) == 2 &&
      packet[consumed] == '\0' && throttle >= -100 && throttle <= 100 &&
      steering >= -100 && steering <= 100) {
    if (autonomousMode) {
      autonomousMode = false;
      stopDrive();
      routeState = RouteState::Patrol;
    }
    if (routeState == RouteState::Hold) {
      routeState = RouteState::Patrol;
    } else if (routeState != RouteState::Patrol) {
      lastCommandMs = millis();
      return;
    }
    if (hardCollisionActive) {
      stopDrive();
      lastCommandMs = millis();
      return;
    }

    const int leftMix = constrain(throttle + steering, -100, 100);
    const int rightMix = constrain(throttle - steering, -100, 100);
    const int8_t leftDirection =
        abs(leftMix) <= Config::MOTOR_DEADBAND ? 0 : (leftMix > 0 ? 1 : -1);
    const int8_t rightDirection =
        abs(rightMix) <= Config::MOTOR_DEADBAND ? 0 : (rightMix > 0 ? 1 : -1);
    leftMotor.setDirection(leftDirection);
    rightMotor.setDirection(rightDirection);
    motorsEnabledByCommand = leftDirection != 0 || rightDirection != 0;
    lastCommandMs = millis();
    return;
  }

  int turretPan = 0;
  consumed = 0;
  if (sscanf(packet, "T,%d%n", &turretPan, &consumed) == 1 &&
      packet[consumed] == '\0' && turretPan >= -100 && turretPan <= 100) {
    if ((!autonomousMode || turretPilotEnabled) &&
        (routeState == RouteState::Patrol || routeState == RouteState::Hold)) {
      const int angle = map(turretPan, -100, 100, Config::TURRET_MIN_ANGLE,
                            Config::TURRET_MAX_ANGLE);
      turretServo.write(angle);
      turretPanAngle = angle;
      lastTurretPanStepMs = millis();
    }
    lastCommandMs = millis();
    return;
  }

  if (sscanf(packet, "L,%d%n", &value, &consumed) == 1 && packet[consumed] == '\0' &&
      (value == 0 || value == 1)) {
    setLaser(value == 1);
    lastCommandMs = millis();
    return;
  }
  consumed = 0;
  if (sscanf(packet, "M,%d%n", &value, &consumed) == 1 && packet[consumed] == '\0' &&
      (value == 0 || value == 1)) {
    if (value == 1) {
      startMelody();
    } else {
      stopMelody();
    }
    lastCommandMs = millis();
    return;
  }

  int left = 0;
  int right = 0;
  int turretAngle = 90;
  consumed = 0;
  if (sscanf(packet, "D,%d,%d,%d%n", &left, &right, &turretAngle, &consumed) != 3 ||
      packet[consumed] != '\0' || left < -255 || left > 255 || right < -255 || right > 255 ||
      turretAngle < Config::TURRET_MIN_ANGLE || turretAngle > Config::TURRET_MAX_ANGLE) {
    return;
  }
  if (autonomousMode) {
    autonomousMode = false;
    stopDrive();
    routeState = RouteState::Patrol;
  }
  if (routeState == RouteState::Hold) {
    routeState = RouteState::Patrol;
  } else if (routeState != RouteState::Patrol) {
    lastCommandMs = millis();
    return;
  }
  if (hardCollisionActive) {
    stopDrive();
    lastCommandMs = millis();
    return;
  }
  const int8_t leftDirection =
      abs(left) <= Config::MOTOR_DEADBAND ? 0 : (left > 0 ? 1 : -1);
  const int8_t rightDirection =
      abs(right) <= Config::MOTOR_DEADBAND ? 0 : (right > 0 ? 1 : -1);
  leftMotor.setDirection(leftDirection);
  rightMotor.setDirection(rightDirection);
  turretServo.write(turretAngle);
  turretPanAngle = turretAngle;
  lastTurretPanStepMs = millis();
  motorsEnabledByCommand = (abs(left) > Config::MOTOR_DEADBAND ||
                            abs(right) > Config::MOTOR_DEADBAND);
  lastCommandMs = millis();
}

void receiveCommands() {
  if (commandQueue == nullptr) {
    return;
  }
  Command command{};
  while (xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
    processCommand(command.text);
  }
}

void enforceCommandTimeout() {
  if (!bleConnected) {
    autonomousMode = false;
    turretPilotEnabled = false;
    stopDrive();
    setLaser(false);
    if (routeState != RouteState::Patrol && routeState != RouteState::Scanning &&
        routeState != RouteState::RouteReady &&
        routeState != RouteState::WaitingForClear && routeState != RouteState::NoRoute) {
      routeState = RouteState::Hold;
    }
    return;
  }
  const uint32_t now = millis();
  if (laserEnabled &&
      static_cast<uint32_t>(now - laserTurnedOnMs) >= Config::LASER_MAX_ON_MS) {
    setLaser(false);
    Serial.println("Laser automatically turned off after safety timeout");
  }
  if (static_cast<uint32_t>(now - lastCommandMs) > Config::COMMAND_TIMEOUT_MS) {
    if (motorsEnabledByCommand && !autonomousMode) {
      stopDrive();
    }
    if (!autonomousMode &&
        (routeState == RouteState::Turning || routeState == RouteState::Advancing)) {
      routeState = RouteState::Hold;
    }
  }
}

void sendTelemetry() {
  const uint32_t now = millis();
  if (!bleConnected || txCharacteristic == nullptr ||
      static_cast<uint32_t>(now - lastTelemetryMs) < Config::TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = now;

  char message[64];
  const uint16_t range = latestRangeMm;
  if (range == 0xFFFF) {
    snprintf(message, sizeof(message), "T,%lu,NA,%d", static_cast<unsigned long>(now),
             tofAvailable ? 1 : 0);
  } else {
    snprintf(message, sizeof(message), "T,%lu,%u,1", static_cast<unsigned long>(now), range);
  }
  txCharacteristic->setValue(message);
  txCharacteristic->notify();
}

void printTofStatus() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastSerialRangeMs) < 500) {
    return;
  }
  lastSerialRangeMs = now;

  if (!tofAvailable) {
    Serial.println("VL53L1X: not detected / initialization failed");
    return;
  }

  const uint16_t range = latestRangeMm;
  if (range == 0xFFFF) {
    Serial.println("VL53L1X: no valid reading (timeout)");
  } else {
    Serial.printf("VL53L1X distance: %u mm\n", range);
  }
}

void beginBle() {
  NimBLEDevice::init(Config::BLE_NAME);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  NimBLEService *service = server->createService(BLE_SERVICE_UUID);

  NimBLECharacteristic *rxCharacteristic = service->createCharacteristic(
      BLE_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxCharacteristic->setCallbacks(new RxCallbacks());
  txCharacteristic = service->createCharacteristic(BLE_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  if (!server->start()) {
    Serial.println("Failed to start BLE server");
    return;
  }

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->enableScanResponse(true);
  if (!advertising->setName(Config::BLE_NAME)) {
    Serial.println("Failed to add BLE device name to scan response");
  }
  advertising->start();
}

void setup() {
  Serial.begin(115200);
  setToneChannel(Config::BUZZER_PWM_CHANNEL);
  pinMode(Config::LASER_PIN, OUTPUT);
  setLaser(false);
  pinMode(Config::BUZZER_PIN, OUTPUT);
  leftMotor.begin();
  rightMotor.begin();
  stopDrive();

  turretServo.setPeriodHertz(50);
  const int servoChannel =
      turretServo.attach(Config::TURRET_SERVO_PIN, 500, 2500);
  Serial.printf("SG90 attach: %s (channel %d)\n",
                turretServo.attached() ? "OK" : "FAILED", servoChannel);
  turretPanAngle = 90;
  turretServo.write(turretPanAngle);
  lastTurretPanStepMs = millis();

  commandQueue = xQueueCreate(8, sizeof(Command));
  beginBle();

  Wire.begin(Config::I2C_SDA, Config::I2C_SCL);
  tof.setTimeout(30);
  tofAvailable = tof.init();
  Serial.println(tofAvailable ? "VL53L1X init: OK" : "VL53L1X init: FAILED");
  if (tofAvailable) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(50);
    xTaskCreatePinnedToCore(startTofTask, "tof-poll", 4096, nullptr, 1, nullptr, 0);
  }

  lastCommandMs = millis();
}

void loop() {
  receiveCommands();
  updateTurretPan();
  enforceCommandTimeout();
  updateObstacleAlarm();
  updateMelody();
  sendTelemetry();
  printTofStatus();
  yield();
}
