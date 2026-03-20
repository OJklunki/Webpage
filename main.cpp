#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include "Webpage.h"
#include <Adafruit_PWMServoDriver.h>
#include "esp_wpa2.h"

// ===================== SHARED STRUCT =====================
// Must be identical in both ESPs
struct ToF_to_Arm {
  float DistanceA;
  float DistanceB;
  float ObjectSize;
  float Temperature_avg;
  float Humidity_avg;
};

struct Arm_to_ToF {
  float fanspeed;  // 0-100 percent
};

ToF_to_Arm sensorData;   // data received from ToF ESP
Arm_to_ToF fanData;      // data we send to ToF ESP

// Replace with the MAC address of your ToF ESP32
uint8_t tofAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // <-- fill in ToF MAC

esp_now_peer_info_t peerInfo;

// ===================== WEBSERVER =====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===================== ADAFRUIT PCA9685 =====================
constexpr uint8_t SERVO_SDA = 33;
constexpr uint8_t SERVO_SCL = 32;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ===================== PS2 CONTROLLER =====================
constexpr uint8_t PS2_TX_PIN = 34;
uint8_t buf[12];
uint8_t bufIndex = 0;
bool synced       = false;
bool selectpressed = false;
uint8_t servoindexp2c = 0;
bool selectedservo = false;

bool triangle  = false;
bool cross     = false;
bool circle    = false;
bool square    = false;
bool l1        = false;
bool r1        = false;
bool l2        = false;
bool r2        = false;
bool btnselect = false;
bool btnstart  = false;  // FIX: renamed from 'start' which is a reserved name on ESP32

uint8_t lx = 0;
uint8_t ly = 0;
uint8_t rx_joy = 0;  // FIX: renamed from 'rx' to avoid conflict with ESP-NOW rx
uint8_t ry = 0;

// ===================== SERVO PINS & LEDC =====================
constexpr uint8_t SERVO_1_PIN = 19;
constexpr uint8_t SERVO_2_PIN = 18;
constexpr uint8_t SERVO_3_PIN = 5;
constexpr uint8_t SERVO_4_PIN = 4;
constexpr uint8_t SERVO_5_PIN = 0;
constexpr uint8_t SERVO_6_PIN = 15;

constexpr uint8_t LEDC_FREQ_HZ   = 50;
constexpr uint8_t LEDC_RES_BITS  = 16;

#define US_TO_DUTY(us) ((uint32_t)(us) * 65535 / 20000)

constexpr uint8_t SERVO_NUM = 6;

static const uint8_t SERVO_PINS[SERVO_NUM]     = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN,
                                                    SERVO_4_PIN, SERVO_5_PIN, SERVO_6_PIN };
static const uint8_t SERVO_CHANNELS[SERVO_NUM] = { 2, 3, 4, 5, 6, 7 };

constexpr uint16_t min_Duty_HI = 500;
constexpr uint16_t max_Duty_HI = 2500;

// ===================== ARM STATE =====================
int baseroatatiion  = 90;
int tilt1           = 90;
int tilt2           = 90;
int Rotationgripper = 90;
int Gripperclosing  = 90;

uint16_t roationBC = 90;
uint16_t gripperC  = 90;
uint16_t roationGC = 90;
uint16_t tilt1C    = 90;
uint16_t tilt2C    = 90;

bool Close           = false;
int  newpostiongripper = 0;
uint8_t SERVO_DELAY  = 30;

// ===================== WIFI =====================
constexpr const char* ssid     = "MyESP";
constexpr const char* password = "formy_self";

// ===================== BUZZER =====================
constexpr uint8_t buzzerpin = 27;
bool Buzzer = false;
bool alarmbuzzer = false;
uint16_t buzzerDelay = 500;

enum State { Firsttone, Lasttone, off };
State alarmstate = off;

// ===================== TIMING =====================
unsigned long previousclosing       = 0;
unsigned long previousserialmessage = 0;
unsigned long previousservo_ada     = 0;
unsigned long previousservo_hi[SERVO_NUM] = {};
unsigned long previousWsBroadcast   = 0;
unsigned long previousalarm         = 0;
unsigned long previoustest          = 0;
unsigned long previousFanSend       = 0;

constexpr uint16_t WS_BROADCAST_INTERVAL = 200;
constexpr uint16_t FAN_SEND_INTERVAL     = 1000;

// ===================== ESP-NOW =====================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  if (millis() - previousserialmessage >= 2000) {
    previousserialmessage = millis();
    Serial.printf("ToF: A=%.1fmm B=%.1fmm Obj=%.1fmm Temp=%.1fC Hum=%.1f%%\n",
                  sensorData.DistanceA, sensorData.DistanceB, sensorData.ObjectSize,
                  sensorData.Temperature_avg, sensorData.Humidity_avg);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("Fan send failed");
}

// ===================== SERVO HELPERS =====================
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

void servo_init() {
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ledcSetup(SERVO_CHANNELS[i], LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_PINS[i], SERVO_CHANNELS[i]);
  }
}

void buzzer_init() {
  ledcSetup(8, 2000, 8);
  ledcAttachPin(buzzerpin, 8);
}

void servo_move_ada(uint8_t temp, uint16_t &currentPos, uint16_t targetPos, uint8_t change) {
  uint8_t channel = temp - 4;
  if (currentPos == targetPos) return;
  if (millis() - previousservo_ada >= SERVO_DELAY) {
    previousservo_ada = millis();
    if (currentPos < targetPos) currentPos++;
    else currentPos--;
    if (change == 1) pwm.setPWM(channel, 0, angleToPulse_sm(currentPos));
    else             pwm.setPWM(channel, 0, currentPos);
  }
}

void servo_move_HI(uint8_t servo_id, uint16_t &currentpos, uint16_t targetpos) {
  if (servo_id >= SERVO_NUM) { Serial.println("ERROR: servo_id out of range"); return; }
  if (targetpos > 180)       { Serial.println("ERROR: angle too large (max 180)"); return; }
  if (currentpos == targetpos) return;
  if (millis() - previousservo_hi[servo_id] >= SERVO_DELAY) {
    previousservo_hi[servo_id] = millis();
    if (currentpos < targetpos) currentpos++;
    else currentpos--;
    ledcWrite(SERVO_CHANNELS[servo_id], US_TO_DUTY(angleToPulse_hi(currentpos)));
  }
}

void servo_move_2_hi(uint16_t &currentpos, uint16_t targetpos) {
  if (targetpos > 180) { Serial.println("ERROR: angle too large (max 180)"); return; }
  if (currentpos == targetpos) return;
  if (millis() - previousservo_hi[1] >= SERVO_DELAY) {
    previousservo_hi[1] = millis();
    if (currentpos < targetpos) currentpos++;
    else currentpos--;
    ledcWrite(1, US_TO_DUTY(angleToPulse_hi(currentpos)));
    ledcWrite(2, US_TO_DUTY(angleToPulse_hi(180 - currentpos)));
  }
}

void moveservos() {
  servo_move_HI(0, roationBC, baseroatatiion);
  servo_move_2_hi(tilt1C, tilt1);
  servo_move_HI(3, tilt2C, tilt2);
  servo_move_HI(4, roationGC, Rotationgripper);
  servo_move_ada(5, gripperC, Gripperclosing, 1);
}

// ===================== PS2 CONTROLLER =====================
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
      // DEBUG - remove once working
      Serial.print("RAW: ");
      for (int i = 0; i < 12; i++) Serial.printf("%02X ", buf[i]);
      Serial.println();

      // buf[0..1] = 0x55 0x55 headers
      // buf[2..11] = recbuff[0..9] from Hiwonder reference
      // buttons are in recbuff[3] = buf[5], recbuff[4] = buf[6]
      triangle  =  (buf[5] & 0xFF) & 1;
      circle    = ((buf[5] & 0xFF) >> 1) & 1;
      cross     = ((buf[5] & 0xFF) >> 2) & 1;
      square    = ((buf[5] & 0xFF) >> 3) & 1;
      l1        = ((buf[5] & 0xFF) >> 4) & 1;
      r1        = ((buf[5] & 0xFF) >> 5) & 1;
      l2        = ((buf[5] & 0xFF) >> 6) & 1;
      r2        = ((buf[5] & 0xFF) >> 7) & 1;
      btnselect = ( buf[6] & 0xFF) & 1;
      btnstart  = ((buf[6] & 0xFF) >> 1) & 1;
      lx        = buf[8];
      ly        = buf[9];
      rx_joy    = buf[10];
      ry        = buf[11];

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
      lx, ly, rx_joy, ry
    );
  }
}

// ===================== COMMANDS =====================
void applyCommand(int joint, int value) {
  switch (joint) {
    case 0: baseroatatiion  = value; Serial.printf("Base: %d\n", value);    break;
    case 1: tilt1           = value; Serial.printf("Tilt1: %d\n", value);   break;
    case 2: tilt2           = value; Serial.printf("Tilt2: %d\n", value);   break;
    case 3: Rotationgripper = value; Serial.printf("GripRot: %d\n", value); break;
    case 4: Gripperclosing  = value; newpostiongripper = value;              break;
    default: Serial.println("Unknown joint"); break;
  }
}

void commands_for_p2c() {
  if (btnstart) {  // FIX: was 'start'
    baseroatatiion  = 90; tilt1 = 90; tilt2 = 90;
    Rotationgripper = 90; Gripperclosing = 180;
  }

  if (btnselect) selectpressed = true;

  if (selectpressed) {
    // FIX: removed the servoindexp2c = 0 that was resetting on every loop,
    // making R1/L1 increment impossible
    if (r1) {
      servoindexp2c++;
      if (servoindexp2c >= SERVO_NUM) {
        servoindexp2c = SERVO_NUM - 1;
        Serial.println("Servo index at max");
      }
    } else if (l1) {
      if (servoindexp2c > 0) servoindexp2c--;
      else Serial.println("Servo index at min");
    } else if (cross) {
      selectpressed = false;
    }
  }
}

// ===================== WEBSERVER =====================
void broadcastPositions() {
  if (ws.count() == 0) return;
  StaticJsonDocument<128> doc;
  doc["type"]    = "positions";
  doc["base"]    = baseroatatiion;
  doc["tilt1"]   = tilt1;
  doc["tilt2"]   = tilt2;
  doc["twist"]   = Rotationgripper;
  doc["gripper"] = Gripperclosing;
  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected\n", client->id());
    broadcastPositions();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      StaticJsonDocument<64> doc;
      if (deserializeJson(doc, (char *)data)) return;
      applyCommand(doc["joint"], doc["value"]);
      broadcastPositions();
    }
  }
}

// ===================== MISC =====================
void alarm() {
  if (millis() - previousalarm >= buzzerDelay) {
    previousalarm = millis();
    switch (alarmstate) {
      case Firsttone: 
      ledcWriteTone(8, 750); 
      alarmstate = Lasttone; buzzerDelay = 350; 
      break;
      case Lasttone:  
      ledcWriteTone(8, 625); 
      alarmstate = Firsttone; 
      buzzerDelay = 500; 
      break;
      default:
      Serial.println("Something wrong happend inside the function alarm");
      break;
    }
  }
}

String serialData = "";
void handleSerial() {
  if (!Serial.available()) return;
  serialData = Serial.readStringUntil('\n');
  serialData.trim();
  if (serialData.equalsIgnoreCase("alarm"))
    alarmbuzzer = true;  return;
  if (serialData.equalsIgnoreCase("offalarm")) { 
    alarmbuzzer = false;
    return; 
  }
  if (serialData.equalsIgnoreCase("close")){ 
    Close = true;
    return;
  }
  int commaIndex = serialData.indexOf(',');
  if (commaIndex <= 0){ 
    Serial.println("Use: joint,value"); 
    return; 
  }
  applyCommand(serialData.substring(0, commaIndex).toInt(), serialData.substring(commaIndex + 1).toInt());
  broadcastPositions();
}

void closing() {
  if (!Close) return;
  if (gripperC == 0) { 
    Close = false; 
    return; 
  }
  if (sensorData.DistanceA <= 18 && sensorData.DistanceB <= 18) {
    Close = false;
    Serial.println("Gripper closed to object");
    return;
  }
  newpostiongripper = 0;  
  servo_move_ada(5, gripperC, newpostiongripper, 1);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PS2_TX_PIN, -1); 

  Wire.begin(SERVO_SDA, SERVO_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);

  buzzer_init();
  servo_init();

  // WiFi — enterprise + soft AP
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);

  constexpr const char* home_ssid     = "mrfylke-sikker";
  constexpr const char* home_username = "oleklu19@skole.mrfylke.no";
  constexpr const char* home_password = "TAE1122addi@esp32a4988";

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)home_username, strlen(home_username));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)home_username, strlen(home_username));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)home_password, strlen(home_password));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(home_ssid);

  Serial.print("Connecting to home wifi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nHome wifi IP: " + WiFi.localIP().toString());

  WiFi.softAP(ssid, password);
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  // ESP-NOW
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, tofAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ToF peer");
  }

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", webpage);
  });
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("Server started");

  // Initial servo positions
  baseroatatiion = 90; tilt1 = 90; tilt2 = 90;
  Rotationgripper = 90; Gripperclosing = 90;
  pwm.setPWM(0, 0, angleToPulse_sm(90));
  pwm.setPWM(1, 0, angleToPulse_ti(90));

  Serial.println("Ready");
}

// ===================== LOOP =====================
void loop() {
  ws.cleanupClients();
  handleSerial();
  closing();
  moveservos();
  p2c();
  commands_for_p2c();
  just_for_testing();
  if (alarmbuzzer) alarm();

  // Send fanspeed to ToF ESP every second
  if (millis() - previousFanSend >= FAN_SEND_INTERVAL) {
    previousFanSend = millis();
    // Derive fanspeed from temperature received from ToF ESP
    if (sensorData.Temperature_avg > 0) {
      fanData.fanspeed = (float)map(
        constrain((int)sensorData.Temperature_avg, 20, 30), 20, 30, 0, 100);
    } else {
      fanData.fanspeed = 0.0f;
    }
    esp_now_send(tofAddress, (uint8_t *)&fanData, sizeof(fanData));
  }

  if (millis() - previousWsBroadcast >= WS_BROADCAST_INTERVAL) {
    previousWsBroadcast = millis();
    broadcastPositions();
  }
}
