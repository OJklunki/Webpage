#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <ArduinoJson.h>
#include "Webpage.h"
#include <Adafruit_PWMServoDriver.h>
#include "esp_wpa2.h"
#include "wifipassword.h"


// Webserver and socket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

constexpr uint8_t SERVO_SDA = 32;
constexpr uint8_t SERVO_SCL = 33;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// tx pin for p2c and retriving variables
constexpr uint8_t PS2_TX_PIN = 34;
uint8_t buf[12];
uint8_t bufIndex = 0;
bool synced = false;
bool selectpressed = false;

// variables from p2 controller
bool triangle = false;
bool cross = false;
bool circle = false;
bool square = false;
bool l1 = false;
bool r1 = false;
bool l2 = false;
bool r2 = false;
bool btnselect = false;
bool btnstart = false;

bool btnselect_prev = false;

uint8_t lx = 0;
uint8_t ly = 0;
uint8_t rx_joy = 0;
uint8_t ry = 0;

// servo pins from hiwonder controller
constexpr u8_t SERVO_1_PIN = 19;
constexpr u8_t SERVO_2_PIN = 18;
constexpr u8_t SERVO_3_PIN = 5;
constexpr u8_t SERVO_4_PIN = 4;
constexpr u8_t SERVO_5_PIN = 0;
constexpr u8_t SERVO_6_PIN = 15;

constexpr u8_t LEDC_FREQ_HZ  = 50;
constexpr u8_t LEDC_RES_BITS = 16;

#define US_TO_DUTY(us) ((uint32_t)(us)*65535 / 20000)

constexpr u8_t SERVO_NUM = 6;

static const uint8_t SERVO_PINS[SERVO_NUM] = {
    SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN,
    SERVO_4_PIN, SERVO_5_PIN, SERVO_6_PIN
};

static const uint8_t SERVO_CHANNELS[SERVO_NUM] = {2, 3, 4, 5, 6, 7};

constexpr u16_t min_Duty_HI = 500;
constexpr u16_t max_Duty_HI = 2500;

unsigned long previousservo_ada[2] = {};

// Servo arm values
int baseroatatiion  = 90;
int tilt1           = 90;
int tilt2           = 90;
int tilt3           = 90;
int Rotationgripper = 90;
int Gripperclosing  = 90;

// Wifi name and password
constexpr const char* ssid     = "MyESP";
constexpr const char* password = "formy_self";

// buzzer pin
constexpr u8_t buzzerpin  = 27;
constexpr u8_t buzzerpin2 = 13;

// current position for servo
u16_t roationBC = 90;
u16_t gripperC  = 90;
u16_t roationGC = 90;
u16_t tilt1C    = 90;
u16_t tilt2C    = 90;
u16_t tilt3C    = 90;

// Gripper servo
bool Close             = false;
int  newpostiongripper = 0;
u8_t SERVO_DELAY       = 30;

// Buzzer flag and state
bool Buzzer      = false;
bool alarmbuzzer = false;

enum State { Firsttone, Lasttone, off };
State alarmstate = off;

// Serial read flags
bool tempread    = false;
bool postionread = false;
bool idread      = false;

// Timing
unsigned long previousserialmessage = 0;
unsigned long previousservo_hi[SERVO_NUM] = {};
unsigned long previousWsBroadcast   = 0;
unsigned long previousalarm         = 0;
unsigned long previoustest          = 0;
unsigned long previousFanSend       = 0;
unsigned long previousselect        = 0;

// interval
constexpr u16_t  WS_BROADCAST_INTERVAL = 200;
u16_t            buzzerDelay           = 500;
constexpr u16_t  FAN_SEND_INTERVAL     = 1000;

struct ToF_to_Arm {
  float DistanceA;
  float DistanceB;
  float ObjectSize;
  float Temperature_avg;
  float Humidity_avg;
};

struct Arm_to_ToF {
  bool  manualOverride;
  float fanspeed;
};

bool fanover_ride = false;
bool sensorDataValid = false;

ToF_to_Arm sensorData;
Arm_to_ToF fanData;

uint8_t tofAddress[] = {0xD4, 0xE9, 0xF4, 0xFB, 0x19, 0x88};
esp_now_peer_info_t peerInfo;


void p2c() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (!synced) {
      if      (bufIndex == 0 && b == 0x55) { buf[bufIndex++] = b; }
      else if (bufIndex == 1 && b == 0x55) { buf[bufIndex++] = b; synced = true; }
      else { bufIndex = 0; }
      continue;
    }
    buf[bufIndex++] = b;
    if (bufIndex == 12) {
      Serial.print("RAW: ");
      for (int i = 0; i < 12; i++) Serial.printf("%02X ", buf[i]);
      Serial.println();
      triangle  =  (buf[5] & 0xFF) & 1;
      circle    = ((buf[5] & 0xFF) >> 1) & 1;
      cross     = ((buf[5] & 0xFF) >> 2) & 1;
      square    = ((buf[5] & 0xFF) >> 3) & 1;
      l1        = ((buf[5] & 0xFF) >> 4) & 1;
      r1        = ((buf[5] & 0xFF) >> 5) & 1;
      l2        = ((buf[5] & 0xFF) >> 6) & 1;
      r2        = ((buf[5] & 0xFF) >> 7) & 1;
      btnselect =  (buf[6] & 0xFF) & 1;
      btnstart  = ((buf[6] & 0xFF) >> 1) & 1;
      lx     = buf[8];
      ly     = buf[9];
      rx_joy = buf[10];
      ry     = buf[11];
      bufIndex = 0;
      synced   = false;
    }
  }
}

void just_for_testing() {
  if (millis() - previoustest >= 1000) {
    previoustest = millis();
    Serial.printf(
        "BTN: tri=%d crs=%d cir=%d sqr=%d L1=%d L2=%d R1=%d R2=%d sel=%d sta=%d | "
        "LJ: x=%3d y=%3d  RJ: x=%3d y=%3d\n",
        triangle, cross, circle, square, l1, l2, r1, r2, btnselect, btnstart,
        lx, ly, rx_joy, ry);
  }
}

void servo_init() {
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ledcSetup(SERVO_CHANNELS[i], LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_PINS[i], SERVO_CHANNELS[i]);
  }
}

void buzzer_init() {
  ledcSetup(8, 2000, 8);
  ledcAttachPin(buzzerpin, 8);
  ledcSetup(9, 2000, 8);
  ledcAttachPin(buzzerpin2, 9);
  ledcWriteTone(8, 0);
  ledcWriteTone(9, 0);
}

int angleToPulse_sm(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 150, 550);
}

int angleToPulse_ti(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 150, 550);
}

int angleToPulse_hi(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, min_Duty_HI, max_Duty_HI);
}

void servo_move_ada(uint8_t channel, u16_t &currentPos, u16_t targetPos, uint8_t change) {
  if (currentPos == targetPos) return;
  if (millis() - previousservo_ada[channel] >= SERVO_DELAY) {
    previousservo_ada[channel] = millis();
    if (currentPos < targetPos) currentPos++;
    else                        currentPos--;
    if (change == 1) pwm.setPWM(channel, 0, angleToPulse_sm(currentPos));
    else             pwm.setPWM(channel, 0, currentPos);
  }
}

void servo_move_HI(uint8_t servo_id, uint16_t &currentpos, u16_t targetpos) {
  if (servo_id >= SERVO_NUM) {
    Serial.println("ERROR: servo_id out of range (1-6)");
    return;
  }
  if (targetpos > 180) {
    Serial.println("ERROR: The angle is to large (Max180\xC2\xB0)");
    return;
  }
  if (currentpos == targetpos) return;
  if (millis() - previousservo_hi[servo_id] >= SERVO_DELAY) {
    previousservo_hi[servo_id] = millis();
    if (currentpos < targetpos) currentpos++;
    else                        currentpos--;
    ledcWrite(SERVO_CHANNELS[servo_id], US_TO_DUTY(angleToPulse_hi(currentpos)));
  }
}

void closing() {
  if (!Close) return;
  if (gripperC == 0) { Close = false; return; }
  if (sensorData.DistanceA <= 18 && sensorData.DistanceB <= 18) {
    Close = false;
    Serial.println("Gripper closed to object");
    return;
  }
  Gripperclosing = 0;
}

// FIX: servo_id order now matches applyCommand joint order exactly:
//   servo_id 0 = joint 0 = base    (baseroatatiion)
//   servo_id 1 = joint 1 = tilt1   (tilt1)
//   servo_id 2 = joint 2 = tilt2   (tilt2)
//   servo_id 3 = joint 3 = tilt3   (tilt3)
//   servo_id 4 = joint 4 = twist   (Rotationgripper)
//   ada ch  0  = joint 5 = gripper (Gripperclosing)
void moveservos() {
  servo_move_HI(0, roationBC, baseroatatiion);
  servo_move_HI(1, tilt1C,    tilt1);
  servo_move_HI(2, tilt2C,    tilt2);
  servo_move_HI(3, tilt3C,    tilt3);
  servo_move_HI(4, roationGC, Rotationgripper);
  servo_move_ada(0, gripperC, Gripperclosing, 1);
}

constexpr uint8_t JOY_DEADZONE = 15;
constexpr uint8_t JOY_STEP     = 1;
constexpr uint8_t BTN_STEP     = 1;

void commands_for_p2c() {
  bool rising_edge = (btnselect && !btnselect_prev);
  if (rising_edge && millis() - previousselect >= 750) {
    previousselect = millis();
    selectpressed  = !selectpressed;
    Serial.printf("PS2 control %s\n", selectpressed ? "ENABLED" : "DISABLED");
  }
  btnselect_prev = btnselect;

  if (selectpressed) {
    if (abs((int)lx - 128) > JOY_DEADZONE) {
      int dir = (lx > 128) ? 1 : -1;
      baseroatatiion = constrain(baseroatatiion + dir * JOY_STEP, 0, 180);
    }
    if (abs((int)ly - 128) > JOY_DEADZONE) {
      int dir = (ly > 128) ? 1 : -1;
      tilt1 = constrain(tilt1 + dir * JOY_STEP, 0, 180);
    }
    if (abs((int)ry - 128) > JOY_DEADZONE) {
      int dir = (ry > 128) ? 1 : -1;
      tilt2 = constrain(tilt2 + dir * JOY_STEP, 0, 180);
    }
    if (abs((int)rx_joy - 128) > JOY_DEADZONE) {
      int dir = (rx_joy > 128) ? 1 : -1;
      tilt3 = constrain(tilt3 + dir * JOY_STEP, 0, 180);
    }
    if (l1) Rotationgripper = constrain(Rotationgripper - BTN_STEP, 0, 180);
    if (r1) Rotationgripper = constrain(Rotationgripper + BTN_STEP, 0, 180);
    if (l2) Gripperclosing  = constrain(Gripperclosing  - BTN_STEP, 0, 180);
    if (r2) Gripperclosing  = constrain(Gripperclosing  + BTN_STEP, 0, 180);

    if (triangle) { }
    if (circle)   { }
    if (cross)    { }
    if (square) {
      baseroatatiion  = 90;
      tilt1           = 0;
      tilt2           = 180;
      tilt3           = 180;
      Rotationgripper = 90;
      Gripperclosing  = 90;
    }
  }
}

void applyCommand(int joint, int value) {
  switch (joint) {
    case 0:
      baseroatatiion = value;
      Serial.printf("Base rotated: %d\xC2\xB0\n", value);
      break;
    case 1:
      tilt1 = value;
      Serial.printf("Tilt1: %d\xC2\xB0\n", value);
      break;
    case 2:
      tilt2 = value;
      Serial.printf("Tilt2: %d\xC2\xB0\n", value);
      break;
    case 3:
      tilt3 = value;
      Serial.printf("Tilt3: %d\xC2\xB0\n", value);
      break;
    case 4:
      Rotationgripper = value;
      Serial.printf("Gripper rotated: %d\xC2\xB0\n", value);
      break;
    case 5:
      Gripperclosing    = value;
      newpostiongripper = value;
      Serial.printf("Gripper closed: %d\xC2\xB0\n", value);
      break;
    default:
      Serial.println("Unknown joint in applyCommand");
      break;
  }
}

void broadcastPositions() {
  if (ws.count() == 0) return;
  StaticJsonDocument<192> doc;
  doc["type"]    = "positions";
  doc["base"]    = baseroatatiion;
  doc["tilt1"]   = tilt1;
  doc["tilt2"]   = tilt2;
  doc["tilt3"]   = tilt3;
  doc["twist"]   = Rotationgripper;
  doc["gripper"] = Gripperclosing;
  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
    broadcastPositions();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      StaticJsonDocument<64> doc;
      DeserializationError err = deserializeJson(doc, (char *)data);
      if (err) {
        Serial.print("JSON parse error: ");
        Serial.println(err.c_str());
        return;
      }
      int joint = doc["joint"];
      int value = doc["value"];
      applyCommand(joint, value);
      broadcastPositions();
    }
  } else if (type == WS_EVT_ERROR) {
    Serial.printf("WS error on client #%u\n", client->id());
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  sensorDataValid = true;
  if (millis() - previousserialmessage >= 2000) {
    previousserialmessage = millis();
    Serial.println("Incoming from ToF");
    Serial.printf(" A: %.1f mm  B: %.1f mm\n", sensorData.DistanceA, sensorData.DistanceB);
    if (sensorData.ObjectSize == 0) Serial.println("No object detected");
    else Serial.printf("Object size: %.1f mm\n", sensorData.ObjectSize);
    Serial.printf("Temp: %.1f C  Humidity: %.1f%%\n", sensorData.Temperature_avg, sensorData.Humidity_avg);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("Fan send failed");
}

void broadcast_2_webpage() {
  if (millis() - previousWsBroadcast >= WS_BROADCAST_INTERVAL) {
    previousWsBroadcast = millis();
    broadcastPositions();
  }
}

void alarm() {
  if (millis() - previousalarm >= buzzerDelay) {
    previousalarm = millis();
    switch (alarmstate) {
      case Firsttone:
        ledcWriteTone(8, 750);
        ledcWriteTone(9, 750);
        alarmstate  = Lasttone;
        buzzerDelay = 350;
        break;
      case Lasttone:
        ledcWriteTone(8, 625);
        ledcWriteTone(9, 625);
        alarmstate  = Firsttone;
        buzzerDelay = 500;
        break;
      default:
        Serial.println("Something wrong happened with alarm state");
        break;
    }
  }
}

String serialData = "";

void handleSerial() {
  if (!Serial.available()) return;
  serialData = Serial.readStringUntil('\n');
  serialData.trim();

  if (serialData.equalsIgnoreCase("alarm")) { alarmbuzzer = true; return; }
  if (serialData.equalsIgnoreCase("offalarm")) { alarmbuzzer = false; return; }
  if (serialData.equalsIgnoreCase("automatic fan")) { fanover_ride = false; return; }
  if (serialData.equalsIgnoreCase("close")) { Close = true; return; }

  int commaIndex = serialData.indexOf(',');
  String cmd = (commaIndex > 0) ? serialData.substring(0, commaIndex) : serialData;
  int param  = (commaIndex > 0) ? serialData.substring(commaIndex + 1).toInt() : 0;

  if (cmd.equalsIgnoreCase("fanspeed")) {
    if (param > 100) {
      Serial.println("Fanspeed % was too large (0-100)");
    } else if (param < 0) {
      Serial.println("Fanspeed % was too small (0-100)");
    } else {
      fanover_ride     = true;
      fanData.fanspeed = param;
      Serial.printf("Fanspeed %% set to: %.0f\n", fanData.fanspeed);
    }
    return;
  }

  if (commaIndex <= 0) { Serial.println("Use: joint,value"); return; }
  applyCommand(cmd.toInt(), param);
  broadcastPositions();
}

void send_fanspeed() {
  if (millis() - previousFanSend >= FAN_SEND_INTERVAL) {
    previousFanSend = millis();
    if (fanover_ride) {
      fanData.manualOverride = true;
      esp_now_send(tofAddress, (uint8_t *)&fanData, sizeof(fanData));
    } else if (sensorDataValid && sensorData.Temperature_avg > 0) {
      fanData.manualOverride = false;
      fanData.fanspeed = (float)map(
          constrain((int)sensorData.Temperature_avg, 20, 30), 20, 30, 0, 100);
      esp_now_send(tofAddress, (uint8_t *)&fanData, sizeof(fanData));
    }
  }
}

void reset_position() {
  pwm.setPWM(0, 0, angleToPulse_sm(90));
  pwm.setPWM(1, 0, angleToPulse_ti(90));
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ledcWrite(SERVO_CHANNELS[i], US_TO_DUTY(angleToPulse_hi(90)));
  }
}

void WiFi_host(u8_t id) {
  WiFi.mode(WIFI_AP_STA);
  if (id == 1) {
    WiFi.begin(wifi_name_private, wifi_password_private);
    Serial.println("Wifi_mode is now mobilewifi");
  } else if (id == 2) {
    WiFi.begin(wifi_name_home, wifi_password_home);
    Serial.println("Wifi_mode is homewifi");
  } else if (id == 3) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)wifi2_username, strlen(wifi2_username));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)wifi2_username, strlen(wifi2_username));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)wifi2_password, strlen(wifi2_password));
    esp_wifi_sta_wpa2_ent_enable();
    WiFi.begin(wifi2_name);
    Serial.println("WiFi_mode is WiFiname, username and password");
  } else {
    Serial.println("WiFi_host id was set wrong in source code");
  }
}

void WiFi_connection_ping() {
  Serial.print("Connecting to home wifi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nHome wifi IP: " + WiFi.localIP().toString());
  WiFi.softAP(ssid, password);
  Serial.println("AP IP: " + WiFi.softAPIP().toString());
}

void init_socket_server() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", webpage);
  });
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("Server started — WebSocket on ws://[IP]/ws");
}

void init_esp_now() {
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, tofAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ToF peer");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  INVERSE KINEMATICS  —  paste these blocks into Robot_main.cpp
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ARM DIMENSIONS (measured from your robot):
//    L1  = 80  mm   tilt1-pivot  →  tilt2-pivot
//    L2  = 100 mm   tilt2-pivot  →  tilt3-pivot
//    L3  = 200 mm   tilt3-pivot  →  gripper tip  (fingers included)
//    BASE_H = 130 mm   tilt1-pivot height above floor
//
// ─── 1. ADD THESE CONSTANTS near the top of Robot_main.cpp ─────────────────
 
static constexpr float IK_L1       = 80.0f;
static constexpr float IK_L2       = 100.0f;
static constexpr float IK_L3       = 200.0f;
static constexpr float IK_BASE_H   = 130.0f;
static constexpr float IK_MIN_R    = 100.0f;   // minimum outward reach (mm)
 
// ── Servo calibration ──────────────────────────────────────────────────────
//  OFFSET  = servo value that puts the link in the listed reference pose
//  SIGN    = +1 if increasing the servo moves the link in the POSITIVE direction
//            -1 if increasing the servo moves in the NEGATIVE direction
//
//  tilt1 reference: lower arm pointing STRAIGHT UP  (servo=90 = vertical)
//  tilt2 reference: upper arm CO-LINEAR with lower  (servo=90 = fully extended)
//  tilt3 reference: wrist pointing STRAIGHT DOWN    (servo=90 = wrist down)
//
//  If the arm consistently overshoots/undershoots, flip the matching SIGN.
//  If it's offset by a fixed amount, tweak the matching OFFSET.
 
static constexpr float IK_T1_OFFSET = 90.0f;   // tune if needed
static constexpr float IK_T1_SIGN   =  1.0f;   // +1 → increasing servo tilts arm forward
static constexpr float IK_T2_OFFSET = 90.0f;
static constexpr float IK_T2_SIGN   =  1.0f;   // +1 → increasing servo bends elbow
static constexpr float IK_T3_OFFSET = 90.0f;
static constexpr float IK_T3_SIGN   =  1.0f;   // +1 → increasing servo tilts wrist forward
 
// ─── 2. ADD THESE FUNCTIONS just before setup() ─────────────────────────────
 
struct IKResult { bool valid; int t1, t2, t3; };
 
// Solve 3-link planar IK.
//   reach_mm       = horizontal distance from base centre to tip (≥ 100 mm)
//   tip_height_mm  = desired height of gripper tip above floor  (0 = floor)
//
// The solver keeps the wrist pointing straight down and uses the elbow-up solution.
IKResult solveIK(float reach_mm, float tip_height_mm) {
    IKResult res = { false, 90, 90, 90 };
 
    reach_mm = max(reach_mm, IK_MIN_R);
 
    // Wrist-pivot (tilt3) target when wrist points straight down
    // vertical component is measured from the tilt1 pivot (positive = up)
    float Rw = reach_mm;
    float Hw = (tip_height_mm - IK_BASE_H) + IK_L3;
    // e.g. tip at floor → Hw = (0 - 130) + 200 = +70 mm above tilt1
 
    float d2 = Rw*Rw + Hw*Hw;
    float d  = sqrtf(d2);
 
    // ── Clamp to reachable range ──
    float maxR = IK_L1 + IK_L2 - 1.0f;
    float minR = fabsf(IK_L1 - IK_L2) + 1.0f;
 
    if (d < minR) {
        Serial.println("IK: target too close to base");
        return res;
    }
    if (d > maxR) {
        float s = maxR / d;          // scale back, keep direction
        Rw *= s;  Hw *= s;
        d2 = Rw*Rw + Hw*Hw;
        d  = sqrtf(d2);
    }
 
    // ── 2-link IK (law of cosines, elbow-up) ──
    float cosA2 = (IK_L1*IK_L1 + IK_L2*IK_L2 - d2) / (2.0f * IK_L1 * IK_L2);
    cosA2 = constrain(cosA2, -1.0f, 1.0f);
    float A2_rad = acosf(cosA2);            // bend angle at tilt2 joint
 
    float alpha = atan2f(Hw, Rw);           // elevation of wrist target from tilt1
    float cosB  = (IK_L1*IK_L1 + d2 - IK_L2*IK_L2) / (2.0f * IK_L1 * d);
    cosB = constrain(cosB, -1.0f, 1.0f);
    float beta  = acosf(cosB);
 
    // θ1 = angle of lower arm from horizontal (elbow-up selects alpha + beta)
    float theta1_rad    = alpha + beta;
    // Direction of upper arm (from horizontal)
    float thetaUp_rad   = theta1_rad - ((float)M_PI - A2_rad);
    // tilt3 correction to keep wrist pointing straight down (−π/2 from horizontal)
    float theta3_rad    = -(float)M_PI / 2.0f - thetaUp_rad;
 
    float t1_deg = theta1_rad * 180.0f / (float)M_PI;
    float A2_deg = A2_rad     * 180.0f / (float)M_PI;
    float t3_deg = theta3_rad * 180.0f / (float)M_PI;
 
    // ── Map geometry angles → servo values ──
    //  tilt1 : reference = vertical  → delta from 90°
    int s1 = (int)roundf(IK_T1_OFFSET + IK_T1_SIGN * (t1_deg - 90.0f));
    //  tilt2 : reference = co-linear → pure bend angle
    int s2 = (int)roundf(IK_T2_OFFSET + IK_T2_SIGN * A2_deg);
    //  tilt3 : reference = wrist down → correction angle
    int s3 = (int)roundf(IK_T3_OFFSET + IK_T3_SIGN * t3_deg);
 
    res.t1    = constrain(s1, 0, 180);
    res.t2    = constrain(s2, 0, 180);
    res.t3    = constrain(s3, 0, 180);
    res.valid = true;
 
    Serial.printf("IK reach=%.0f h=%.0f → θ1=%.1f° A2=%.1f° θ3_corr=%.1f°  servo: %d %d %d\n",
                  reach_mm, tip_height_mm, t1_deg, A2_deg, t3_deg, res.t1, res.t2, res.t3);
    return res;
}
 
// Call to move the arm:  reach = mm outward from base, tip_h = mm above floor
void lowerArm(float reach_mm = 150.0f, float tip_h_mm = 0.0f) {
    IKResult ik = solveIK(reach_mm, tip_h_mm);
    if (!ik.valid) return;
    applyCommand(1, ik.t1);
    applyCommand(2, ik.t2);
    applyCommand(3, ik.t3);
    broadcastPositions();
}
 
 
// ═══════════════════════════════════════════════════════════════════════════════
//  INVERSE KINEMATICS  —  paste these blocks into Robot_main.cpp
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ARM DIMENSIONS (measured from your robot):
//    L1  = 80  mm   tilt1-pivot  →  tilt2-pivot
//    L2  = 100 mm   tilt2-pivot  →  tilt3-pivot
//    L3  = 200 mm   tilt3-pivot  →  gripper tip  (fingers included)
//    BASE_H = 130 mm   tilt1-pivot height above floor
//
// ─── 1. ADD THESE CONSTANTS near the top of Robot_main.cpp ─────────────────

static constexpr float IK_L1       = 80.0f;
static constexpr float IK_L2       = 100.0f;
static constexpr float IK_L3       = 200.0f;
static constexpr float IK_BASE_H   = 130.0f;
static constexpr float IK_MIN_R    = 100.0f;   // minimum outward reach (mm)

// ── Servo calibration ──────────────────────────────────────────────────────
//  OFFSET  = servo value that puts the link in the listed reference pose
//  SIGN    = +1 if increasing the servo moves the link in the POSITIVE direction
//            -1 if increasing the servo moves in the NEGATIVE direction
//
//  tilt1 reference: lower arm pointing STRAIGHT UP  (servo=90 = vertical)
//  tilt2 reference: upper arm CO-LINEAR with lower  (servo=90 = fully extended)
//  tilt3 reference: wrist pointing STRAIGHT DOWN    (servo=90 = wrist down)
//
//  If the arm consistently overshoots/undershoots, flip the matching SIGN.
//  If it's offset by a fixed amount, tweak the matching OFFSET.

static constexpr float IK_T1_OFFSET = 90.0f;   // tune if needed
static constexpr float IK_T1_SIGN   =  1.0f;   // +1 → increasing servo tilts arm forward
static constexpr float IK_T2_OFFSET = 90.0f;
static constexpr float IK_T2_SIGN   =  1.0f;   // +1 → increasing servo bends elbow
static constexpr float IK_T3_OFFSET = 90.0f;
static constexpr float IK_T3_SIGN   =  1.0f;   // +1 → increasing servo tilts wrist forward

// ─── 2. ADD THESE FUNCTIONS just before setup() ─────────────────────────────

struct IKResult { bool valid; int t1, t2, t3; };

// Solve 3-link planar IK.
//   reach_mm       = horizontal distance from base centre to tip (≥ 100 mm)
//   tip_height_mm  = desired height of gripper tip above floor  (0 = floor)
//
// The solver keeps the wrist pointing straight down and uses the elbow-up solution.
IKResult solveIK(float reach_mm, float tip_height_mm) {
    IKResult res = { false, 90, 90, 90 };

    reach_mm = max(reach_mm, IK_MIN_R);

    // Wrist-pivot (tilt3) target when wrist points straight down
    // vertical component is measured from the tilt1 pivot (positive = up)
    float Rw = reach_mm;
    float Hw = (tip_height_mm - IK_BASE_H) + IK_L3;
    // e.g. tip at floor → Hw = (0 - 130) + 200 = +70 mm above tilt1

    float d2 = Rw*Rw + Hw*Hw;
    float d  = sqrtf(d2);

    // ── Clamp to reachable range ──
    float maxR = IK_L1 + IK_L2 - 1.0f;
    float minR = fabsf(IK_L1 - IK_L2) + 1.0f;

    if (d < minR) {
        Serial.println("IK: target too close to base");
        return res;
    }
    if (d > maxR) {
        float s = maxR / d;          // scale back, keep direction
        Rw *= s;  Hw *= s;
        d2 = Rw*Rw + Hw*Hw;
        d  = sqrtf(d2);
    }

    // ── 2-link IK (law of cosines, elbow-up) ──
    float cosA2 = (IK_L1*IK_L1 + IK_L2*IK_L2 - d2) / (2.0f * IK_L1 * IK_L2);
    cosA2 = constrain(cosA2, -1.0f, 1.0f);
    float A2_rad = acosf(cosA2);            // bend angle at tilt2 joint

    float alpha = atan2f(Hw, Rw);           // elevation of wrist target from tilt1
    float cosB  = (IK_L1*IK_L1 + d2 - IK_L2*IK_L2) / (2.0f * IK_L1 * d);
    cosB = constrain(cosB, -1.0f, 1.0f);
    float beta  = acosf(cosB);

    // θ1 = angle of lower arm from horizontal (elbow-up selects alpha + beta)
    float theta1_rad    = alpha + beta;
    // Direction of upper arm (from horizontal)
    float thetaUp_rad   = theta1_rad - ((float)M_PI - A2_rad);
    // tilt3 correction to keep wrist pointing straight down (−π/2 from horizontal)
    float theta3_rad    = -(float)M_PI / 2.0f - thetaUp_rad;

    float t1_deg = theta1_rad * 180.0f / (float)M_PI;
    float A2_deg = A2_rad     * 180.0f / (float)M_PI;
    float t3_deg = theta3_rad * 180.0f / (float)M_PI;

    // ── Map geometry angles → servo values ──
    //  tilt1 : reference = vertical  → delta from 90°
    int s1 = (int)roundf(IK_T1_OFFSET + IK_T1_SIGN * (t1_deg - 90.0f));
    //  tilt2 : reference = co-linear → pure bend angle
    int s2 = (int)roundf(IK_T2_OFFSET + IK_T2_SIGN * A2_deg);
    //  tilt3 : reference = wrist down → correction angle
    int s3 = (int)roundf(IK_T3_OFFSET + IK_T3_SIGN * t3_deg);

    res.t1    = constrain(s1, 0, 180);
    res.t2    = constrain(s2, 0, 180);
    res.t3    = constrain(s3, 0, 180);
    res.valid = true;

    Serial.printf("IK reach=%.0f h=%.0f → θ1=%.1f° A2=%.1f° θ3_corr=%.1f°  servo: %d %d %d\n",
                  reach_mm, tip_height_mm, t1_deg, A2_deg, t3_deg, res.t1, res.t2, res.t3);
    return res;
}

// Call to move the arm:  reach = mm outward from base, tip_h = mm above floor
void lowerArm(float reach_mm = 150.0f, float tip_h_mm = 0.0f) {
    IKResult ik = solveIK(reach_mm, tip_h_mm);
    if (!ik.valid) return;
    applyCommand(1, ik.t1);
    applyCommand(2, ik.t2);
    applyCommand(3, ik.t3);
    broadcastPositions();
}


// ─── 3. INSIDE handleSerial() — add this before the generic "joint,value" block

    // lower,<reach_mm>          e.g.  "lower,150"  lowers to floor 150 mm out
    // lower,<reach_mm>,<h_mm>   e.g.  "lower,120,30" tip 30 mm above floor
    if (cmd.equalsIgnoreCase("lower")) {
        float reach = (commaIndex > 0) ? serialData.substring(commaIndex + 1).toFloat() : 150.0f;
        // optional second comma for height
        int comma2 = serialData.indexOf(',', commaIndex + 1);
        float height = (comma2 > 0) ? serialData.substring(comma2 + 1).toFloat() : 0.0f;
        lowerArm(reach, height);
        return;
    }


// ─── 4. INSIDE onWsEvent(), replace the WS_EVT_DATA block with this version ─

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            StaticJsonDocument<128> doc;
            DeserializationError err = deserializeJson(doc, (char *)data);
            if (err) {
                Serial.print("JSON parse error: "); Serial.println(err.c_str());
                return;
            }

            // ── NEW: lower_arm IK command ──────────────────────────────────
            if (doc.containsKey("cmd")) {
                const char *cmd_ws = doc["cmd"];
                if (strcmp(cmd_ws, "lower_arm") == 0) {
                    float reach  = doc["reach"]  | 150.0f;
                    float height = doc["height"] | 0.0f;
                    lowerArm(reach, height);
                }
                // future commands can go here
                return;
            }

            // ── existing: individual joint command ─────────────────────────
            int joint = doc["joint"];
            int value = doc["value"];
            applyCommand(joint, value);
            broadcastPositions();
        }
    }

// ════════════════════════════════════════════════════════════════════════════
//  CALIBRATION GUIDE
//  -----------------
//  1. Set reach=150, height=0.  Send "lower,150" from Serial monitor.
//  2. Watch where the gripper tip actually ends up.
//
//  Tip is too FAR out  → decrease IK_T1_SIGN (try -1) or increase IK_T1_OFFSET
//  Tip is too CLOSE in → increase IK_T1_SIGN or decrease IK_T1_OFFSET
//  Elbow folds WRONG way → flip IK_T2_SIGN  (-1 ↔ +1)
//  Wrist not pointing down → tweak IK_T3_OFFSET or flip IK_T3_SIGN
//
//  Repeat with a few reach values (100, 150, 175) to confirm consistency.
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PS2_TX_PIN, -1);
  Wire.begin(SERVO_SDA, SERVO_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);
  buzzer_init();
  servo_init();
  reset_position();
  WiFi_host(2);
  WiFi_connection_ping();
  init_socket_server();
  init_esp_now();
  Serial.println("Ready");
  Serial.printf("WiFi channel: %d\n", WiFi.channel());
}

void loop() {
  ws.cleanupClients();
  handleSerial();
  closing();
  moveservos();
  p2c();
  commands_for_p2c();
  //just_for_testing();
  if (alarmbuzzer) alarm();
  send_fanspeed();
  broadcast_2_webpage();
}
