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

// ─── Webserver and socket ──────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

constexpr uint8_t SERVO_SDA = 32;
constexpr uint8_t SERVO_SCL = 33;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ─── PS2 controller ────────────────────────────────────────────────────────
constexpr uint8_t PS2_TX_PIN = 34;
uint8_t buf[12];
uint8_t bufIndex  = 0;
bool    synced    = false;
bool    selectpressed = false;

bool triangle = false, cross  = false, circle = false, square = false;
bool l1 = false, r1 = false, l2 = false, r2 = false;
bool btnselect = false, btnstart = false;
bool btnselect_prev = false;

uint8_t lx = 0, ly = 0, rx_joy = 0, ry = 0;

// ─── Servo pins (Hiwonder / LEDC) ─────────────────────────────────────────
constexpr uint8_t SERVO_1_PIN = 19;
constexpr uint8_t SERVO_2_PIN = 18;
constexpr uint8_t SERVO_3_PIN = 5;
constexpr uint8_t SERVO_4_PIN = 4;
constexpr uint8_t SERVO_5_PIN = 0;
constexpr uint8_t SERVO_6_PIN = 15;

constexpr uint8_t LEDC_FREQ_HZ  = 50;
constexpr uint8_t LEDC_RES_BITS = 16;

#define US_TO_DUTY(us) ((uint32_t)(us) * 65535 / 20000)

constexpr uint8_t SERVO_NUM = 6;

static const uint8_t SERVO_PINS[SERVO_NUM]     = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN,
                                                    SERVO_4_PIN, SERVO_5_PIN, SERVO_6_PIN };
static const uint8_t SERVO_CHANNELS[SERVO_NUM] = { 2, 3, 4, 5, 6, 7 };

constexpr uint16_t min_Duty_HI = 500;
constexpr uint16_t max_Duty_HI = 2500;

unsigned long previousservo_ada[2] = {};

// ─── Joint target values ───────────────────────────────────────────────────
int baseroatatiion  = 90;
int tilt1           = 90;
int tilt2           = 90;
int tilt3           = 90;
int Rotationgripper = 90;
int Gripperclosing  = 90;

// ─── Soft-AP credentials ───────────────────────────────────────────────────
constexpr const char* ssid     = "MyESP";
constexpr const char* password = "formy_self";

// ─── Buzzer ────────────────────────────────────────────────────────────────
constexpr uint8_t buzzerpin  = 27;
constexpr uint8_t buzzerpin2 = 13;

// ─── Current servo positions (smoothed) ───────────────────────────────────
uint16_t roationBC = 90;
uint16_t gripperC  = 90;
uint16_t roationGC = 90;
uint16_t tilt1C    = 90;
uint16_t tilt2C    = 90;
uint16_t tilt3C    = 90;

// ─── Gripper auto-close ────────────────────────────────────────────────────
bool Close             = false;
int  newpostiongripper = 0;
uint8_t SERVO_DELAY    = 30;

// ─── Buzzer state ──────────────────────────────────────────────────────────
bool Buzzer      = false;
bool alarmbuzzer = false;

enum State { Firsttone, Lasttone, off };
State alarmstate = off;

// ─── Serial flags ──────────────────────────────────────────────────────────
bool tempread    = false;
bool postionread = false;
bool idread      = false;

// ─── Timing ────────────────────────────────────────────────────────────────
unsigned long previousserialmessage = 0;
unsigned long previousservo_hi[SERVO_NUM] = {};
unsigned long previousWsBroadcast   = 0;
unsigned long previousalarm         = 0;
unsigned long previoustest          = 0;
unsigned long previousFanSend       = 0;
unsigned long previousselect        = 0;

constexpr uint16_t WS_BROADCAST_INTERVAL = 200;
uint16_t           buzzerDelay           = 500;
constexpr uint16_t FAN_SEND_INTERVAL     = 1000;

// ─── ESP-NOW structs ───────────────────────────────────────────────────────
struct ToF_to_Arm {
  float DistanceA, DistanceB, ObjectSize, Temperature_avg, Humidity_avg;
};
struct Arm_to_ToF {
  bool  manualOverride;
  float fanspeed;
};

bool fanover_ride    = false;
bool sensorDataValid = false;

ToF_to_Arm sensorData;
Arm_to_ToF fanData;

uint8_t tofAddress[] = { 0xD4, 0xE9, 0xF4, 0xFB, 0x19, 0x88 };
esp_now_peer_info_t peerInfo;

// ══════════════════════════════════════════════════════════════════════════════
//  Forward declarations
// ══════════════════════════════════════════════════════════════════════════════
void applyCommand(int joint, int value);
void broadcastPositions();

// ══════════════════════════════════════════════════════════════════════════════
//  PS2 CONTROLLER PARSER
// ══════════════════════════════════════════════════════════════════════════════
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
      lx = buf[8]; ly = buf[9]; rx_joy = buf[10]; ry = buf[11];
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

// ══════════════════════════════════════════════════════════════════════════════
//  SERVO HELPERS
// ══════════════════════════════════════════════════════════════════════════════
void servo_init() {
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ledcSetup(SERVO_CHANNELS[i], LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_PINS[i], SERVO_CHANNELS[i]);
  }
}

void buzzer_init() {
  ledcSetup(8, 2000, 8);  ledcAttachPin(buzzerpin,  8);
  ledcSetup(9, 2000, 8);  ledcAttachPin(buzzerpin2, 9);
  ledcWriteTone(8, 0);    ledcWriteTone(9, 0);
}

int angleToPulse_sm(int angle) { return map(constrain(angle, 0, 180), 0, 180, 150, 550); }
int angleToPulse_ti(int angle) { return map(constrain(angle, 0, 180), 0, 180, 150, 550); }
int angleToPulse_hi(int angle) { return map(constrain(angle, 0, 180), 0, 180, min_Duty_HI, max_Duty_HI); }

void servo_move_ada(uint8_t channel, uint16_t &currentPos, uint16_t targetPos, uint8_t change) {
  if (currentPos == targetPos) return;
  if (millis() - previousservo_ada[channel] >= SERVO_DELAY) {
    previousservo_ada[channel] = millis();
    if (currentPos < targetPos) currentPos++; else currentPos--;
    if (change == 1) pwm.setPWM(channel, 0, angleToPulse_sm(currentPos));
    else             pwm.setPWM(channel, 0, currentPos);
  }
}

void servo_move_HI(uint8_t servo_id, uint16_t &currentpos, uint16_t targetpos) {
  if (servo_id >= SERVO_NUM) { Serial.println("ERROR: servo_id out of range"); return; }
  if (targetpos > 180)       { Serial.println("ERROR: angle > 180°");          return; }
  if (currentpos == targetpos) return;
  if (millis() - previousservo_hi[servo_id] >= SERVO_DELAY) {
    previousservo_hi[servo_id] = millis();
    if (currentpos < targetpos) currentpos++; else currentpos--;
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

void moveservos() {
  servo_move_HI(0, roationBC, baseroatatiion);
  servo_move_HI(1, tilt1C,    tilt1);
  servo_move_HI(2, tilt2C,    tilt2);
  servo_move_HI(3, tilt3C,    tilt3);
  servo_move_HI(4, roationGC, Rotationgripper);
  servo_move_ada(0, gripperC, Gripperclosing, 1);
}

// ══════════════════════════════════════════════════════════════════════════════
//  PS2 COMMAND MAPPING (UPDATED FOR VELOCITY CONTROL)
// ══════════════════════════════════════════════════════════════════════════════
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
    // Left Stick X -> Base Rotation (Velocity Mode: Stops when released)
    if (abs((int)lx - 128) > JOY_DEADZONE)
      baseroatatiion = constrain(baseroatatiion + ((lx > 128) ? 1 : -1) * JOY_STEP, 0, 180);
    
    // Left Stick Y -> Tilt 1 (Velocity Mode: Stops when released)
    if (abs((int)ly - 128) > JOY_DEADZONE)
      tilt1 = constrain(tilt1 + ((ly > 128) ? 1 : -1) * JOY_STEP, 0, 180);
    
    // Right Stick Y -> Tilt 2 (Velocity Mode: Stops when released)
    if (abs((int)ry - 128) > JOY_DEADZONE)
      tilt2 = constrain(tilt2 + ((ry > 128) ? 1 : -1) * JOY_STEP, 0, 180);
    
    // Right Stick X -> Tilt 3 (Velocity Mode: Stops when released)
    if (abs((int)rx_joy - 128) > JOY_DEADZONE)
      tilt3 = constrain(tilt3 + ((rx_joy > 128) ? 1 : -1) * JOY_STEP, 0, 180);

    // Gripper Controls
    if (l1) Rotationgripper = constrain(Rotationgripper - BTN_STEP, 0, 180);
    if (r1) Rotationgripper = constrain(Rotationgripper + BTN_STEP, 0, 180);
    if (l2) Gripperclosing  = constrain(Gripperclosing  - BTN_STEP, 0, 180);
    if (r2) Gripperclosing  = constrain(Gripperclosing  + BTN_STEP, 0, 180);

    // HOME BUTTON (Triangle): Resets all joints to 90 degrees slowly
    if (triangle) {
      baseroatatiion  = 90;
      tilt1           = 90;
      tilt2           = 90;
      tilt3           = 90;
      Rotationgripper = 90;
      Gripperclosing  = 90;
    }

    // PANIC RESET (Square): Snap back to default stance
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

// ══════════════════════════════════════════════════════════════════════════════
//  APPLY COMMAND  (joint index → variable)
// ══════════════════════════════════════════════════════════════════════════════
void applyCommand(int joint, int value) {
  switch (joint) {
    case 0: baseroatatiion  = value; Serial.printf("Base:    %d\xC2\xB0\n", value); break;
    case 1: tilt1           = value; Serial.printf("Tilt1:   %d\xC2\xB0\n", value); break;
    case 2: tilt2           = value; Serial.printf("Tilt2:   %d\xC2\xB0\n", value); break;
    case 3: tilt3           = value; Serial.printf("Tilt3:   %d\xC2\xB0\n", value); break;
    case 4: Rotationgripper = value; Serial.printf("Twist:   %d\xC2\xB0\n", value); break;
    case 5:
      Gripperclosing    = value;
      newpostiongripper = value;
      Serial.printf("Gripper: %d\xC2\xB0\n", value);
      break;
    default:
      Serial.println("Unknown joint in applyCommand");
      break;
  }
}

// ══════════════════════════════════════════════════════════════════════════════
//  WEBSOCKET BROADCAST
// ══════════════════════════════════════════════════════════════════════════════
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

// ══════════════════════════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLER
// ══════════════════════════════════════════════════════════════════════════════
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
      StaticJsonDocument<128> doc;
      DeserializationError err = deserializeJson(doc, (char *)data);
      if (err) {
        Serial.print("JSON parse error: "); Serial.println(err.c_str());
        return;
      }

      // ── Individual joint command ───────────────────────────────────────
      int joint = doc["joint"];
      int value = doc["value"];
      applyCommand(joint, value);
      broadcastPositions();
    }

  } else if (type == WS_EVT_ERROR) {
    Serial.printf("WS error on client #%u\n", client->id());
  }
}

// ══════════════════════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACKS
// ══════════════════════════════════════════════════════════════════════════════
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

// ══════════════════════════════════════════════════════════════════════════════
//  PERIODIC TASKS
// ══════════════════════════════════════════════════════════════════════════════
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
        ledcWriteTone(8, 750); ledcWriteTone(9, 750);
        alarmstate = Lasttone; buzzerDelay = 350;
        break;
      case Lasttone:
        ledcWriteTone(8, 625); ledcWriteTone(9, 625);
        alarmstate = Firsttone; buzzerDelay = 500;
        break;
      default:
        Serial.println("Alarm state error");
        break;
    }
  }
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

// ══════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLER
// ══════════════════════════════════════════════════════════════════════════════
String serialData = "";

void handleSerial() {
  if (!Serial.available()) return;
  serialData = Serial.readStringUntil('\n');
  serialData.trim();

  // ── Simple keyword commands ────────────────────────────────────────────
  if (serialData.equalsIgnoreCase("alarm"))        { alarmbuzzer  = true;  return; }
  if (serialData.equalsIgnoreCase("offalarm"))     { alarmbuzzer  = false; return; }
  if (serialData.equalsIgnoreCase("automatic fan")){ fanover_ride = false; return; }
  if (serialData.equalsIgnoreCase("close"))        { Close        = true;  return; }

  // ── Commands with parameters ──────────────────────────────────────────
  int commaIndex = serialData.indexOf(',');
  String cmd = (commaIndex > 0) ? serialData.substring(0, commaIndex) : serialData;

  // fanspeed,<0-100>
  if (cmd.equalsIgnoreCase("fanspeed")) {
    int param = (commaIndex > 0) ? serialData.substring(commaIndex + 1).toInt() : 0;
    if (param < 0 || param > 100) {
      Serial.println("Fanspeed must be 0-100");
    } else {
      fanover_ride     = true;
      fanData.fanspeed = param;
      Serial.printf("Fanspeed set to: %.0f%%\n", fanData.fanspeed);
    }
    return;
  }

  // <joint>,<value>  e.g. "1,45"
  if (commaIndex <= 0) { Serial.println("Use: joint,value"); return; }
  applyCommand(cmd.toInt(), serialData.substring(commaIndex + 1).toInt());
  broadcastPositions();
}

// ══════════════════════════════════════════════════════════════════════════════
//  INIT HELPERS
// ══════════════════════════════════════════════════════════════════════════════
void reset_position() {
  pwm.setPWM(0, 0, angleToPulse_sm(90));
  pwm.setPWM(1, 0, angleToPulse_ti(90));
  for (uint8_t i = 0; i < SERVO_NUM; i++)
    ledcWrite(SERVO_CHANNELS[i], US_TO_DUTY(angleToPulse_hi(90)));
}

void WiFi_host(uint8_t id) {
  WiFi.mode(WIFI_AP_STA);
  if (id == 1) {
    WiFi.begin(wifi_name_private, wifi_password_private);
    Serial.println("WiFi: mobile hotspot");
  } else if (id == 2) {
    WiFi.begin(wifi_name_home, wifi_password_home);
    Serial.println("WiFi: home network");
  } else if (id == 3) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)wifi2_username, strlen(wifi2_username));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)wifi2_username, strlen(wifi2_username));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)wifi2_password, strlen(wifi2_password));
    esp_wifi_sta_wpa2_ent_enable();
    WiFi.begin(wifi2_name);
    Serial.println("WiFi: WPA2-Enterprise");
  } else {
    Serial.println("WiFi_host: invalid id");
  }
}

void WiFi_connection_ping() {
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nHome WiFi IP: " + WiFi.localIP().toString());
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    Serial.println("Failed to add ToF peer");
}

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
