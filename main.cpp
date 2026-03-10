#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <AsyncWebSocket.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include "webpage.h"

// ─── WEB SERVER + WEBSOCKET ────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─── JOINT VALUES ──────────────────────────────────────────────────────────
int baseroatatiion   = 0;
int tilt1            = 0;
int tilt2            = 0;
int Rotationgripper  = 0;
int Gripperclosingmm = 0;

// ─── WIFI CREDENTIALS ─────────────────────────────────────────────────────
constexpr const char* ssid     = "MyESP";
constexpr const char* password = "formy_self";

// ─── GRIPPER SERVO ────────────────────────────────────────────────────────
Servo gripper;
constexpr u8_t gripperpin   = 4;
bool Close                  = false;
int  gripperCurrent         = 0;
constexpr u8_t SERVO_DELAY  = 30;

// ─── SERIAL READ FLAGS ────────────────────────────────────────────────────
bool tempread    = false;
bool postionread = false;
bool idread      = false;

// ─── TIMING ───────────────────────────────────────────────────────────────
unsigned long previousservowrite     = 0;
unsigned long previousclosing        = 0;
unsigned long previousserialmessage  = 0;
unsigned long previousservoidread    = 0
unsigned long previousreadtempsensor = 0;
unsigned long previouspostionread    = 0;
unsigned long previousWsBroadcast    = 0;
unsingned long previousidread = 0;
constexpr unsigned long WS_BROADCAST_INTERVAL = 200; // ms between position broadcasts

// ─── ESP-NOW ──────────────────────────────────────────────────────────────
struct struct_message { float a; float b; float os; };
struct_message myData;

// ─── HELPERS ──────────────────────────────────────────────────────────────
int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 150, 550);
}

void servosmooth(u8_t angle) {
  if (angle == gripperCurrent) return;
  if (millis() - previousservowrite >= SERVO_DELAY) {
    previousservowrite = millis();
    gripperCurrent += (angle > gripperCurrent) ? 1 : -1;
    gripper.write(angleToPulse(gripperCurrent));
  }
}

// ─── BUS SERVO PROTOCOL ───────────────────────────────────────────────────
void Busservowrite(HardwareSerial &serial, u8_t id, u16_t position, u16_t time_ms) {
  u8_t len      = 7;
  u8_t cmd      = 0x01;
  u8_t posLow   = position & 0xFF;
  u8_t posHigh  = (position >> 8) & 0xFF;
  u8_t timeLow  = time_ms & 0xFF;
  u8_t timeHigh = (time_ms >> 8) & 0xFF;
  u8_t checksum = ~(id + len + cmd + posLow + posHigh + timeLow + timeHigh) & 0xFF;

  serial.write(0x55); serial.write(0x55);
  serial.write(id);   serial.write(len);   serial.write(cmd);
  serial.write(posLow); serial.write(posHigh);
  serial.write(timeLow); serial.write(timeHigh);
  serial.write(checksum);
}

float Readposition(HardwareSerial &monitor, u8_t id) {
  u8_t lenght   = 3;
  u8_t cmd      = 0x1C;
  u8_t checksum = ~(lenght + cmd + id) & 0xFF;

  monitor.write(0x55); monitor.write(0x55);
  monitor.write(id);   monitor.write(lenght);
  monitor.write(cmd);  monitor.write(checksum);

  if (postionread) { previouspostionread = millis(); postionread = false; }
  if (monitor.available() > 0 && millis() - previouspostionread >= 5) {
    u8_t data[7] = {};
    for (int i = 0; i < 7; i++) data[i] = monitor.read();
    u16_t position = data[5] + (data[6] << 8);
    postionread = true;
    return position;
  }
  return -1;
}

float Readtempservo(HardwareSerial &monitor, u8_t id) {
  u8_t lenght   = 3;
  u8_t cmd      = 0x1A;
  u8_t checksum = ~(lenght + cmd + id) & 0xFF;

  monitor.write(0x55); monitor.write(0x55);
  monitor.write(id);   monitor.write(lenght);
  monitor.write(cmd);  monitor.write(checksum);

  if (tempread) { previousreadtempsensor = millis(); tempread = false; }
  if (monitor.available() > 0 && millis() - previousreadtempsensor >= 5) {
    u8_t data[7] = {};
    for (int i = 0; i < 7; i++) data[i] = monitor.read();
    u16_t temprature = data[5] + (data[6] << 8);
    tempread = true;
    return temprature;
  }
  return -1;
}

void on_off_servo(HardwareSerial &monitor, u8_t id, u8_t state) {

  
  u8_t lenght   = 7;
  u8_t cmd      = 0x1F;
  u8_t checksum = ~(lenght + cmd + id + state) & 0xFF;
  monitor.write(0x55); monitor.write(0x55);
  monitor.write(id);   monitor.write(lenght);
  monitor.write(cmd);  monitor.write(state);
  monitor.write(checksum);
}
u16_t servoidread(HardwareSerial &monitor, u8_t id){
  u8_t lenght = 3;
  u8_t cmd = 0x0E;
  u8_t checksum = ~(lenght + cmd + id) & 0xFF;

  monitor.write(0x55);
  monitor.write(0x55);
  monitor.write(id);
  monitor.write(lenght);
  monitor.write(cmd);
  monitor.write(checksum);

  if (idread) { previousidread = millis(); postionread = false; }

  if(monitor.available() > 0 && millis() - previousidread >= 5){
    u8_t data[] = {};
    for (int i = 0; i < 7; i++){
      data[i] = Serial.read();
    }
    u8_t id = data[5] + (data[6]*256);
    idread = true;
    return id;
  }
  return -1;
}

// void servo id write
void servoidwrite(HardwareSerial &monitor, u8_t newid){
  u8_t lenght = 4;
  u8_t cmd = 0x0D;
  u8_t checksum = ~(lenght + cmd + 0xFE + newid) & 0xFF;

  monitor.write(0x55);
  monitor.write(0x55);
  monitor.write(0xFE);
  monitor.write(lenght);
  monitor.write(cmd);
  monitor.write(newid);
  monitor.write(checksum);
}

// ─── APPLY COMMAND TO SERVOS ──────────────────────────────────────────────
// Called by both WebSocket handler and Serial handler
void applyCommand(int joint, int value) {
  switch (joint) {
    case 0:
      baseroatatiion = value;
      Busservowrite(Serial1, /*base id*/ 1, value, 1000);
      break;
    case 1:
      tilt1 = value;
      Busservowrite(Serial1, /*tilt1 id*/ 2, value, 1000);
      break;
    case 2:
      tilt2 = value;
      Busservowrite(Serial1, /*tilt2 id*/ 3, value, 1000);
      break;
    case 3:
      Rotationgripper = value;
      Busservowrite(Serial1, /*twist id*/ 4, value, 1000);
      break;
    case 4:
      Gripperclosingmm = value;
      servosmooth(map(value, 0, 62, 0, 180));
      break;
    default:
      Serial.println("Unknown joint in applyCommand");
      break;
  }
}

// ─── BROADCAST POSITIONS TO ALL WS CLIENTS ────────────────────────────────
// Pushes current joint state as JSON — browser receives this and updates
// the 3D view without needing to poll
void broadcastPositions() {
  if (ws.count() == 0) return; // nobody connected, skip

  StaticJsonDocument<128> doc;
  doc["type"]    = "positions";
  doc["base"]    = baseroatatiion;
  doc["tilt1"]   = tilt1;
  doc["tilt2"]   = tilt2;
  doc["twist"]   = Rotationgripper;
  doc["gripper"] = Gripperclosingmm;

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

// ─── WEBSOCKET EVENT HANDLER ──────────────────────────────────────────────
// This is the core of the WebSocket upgrade — instead of separate HTTP routes
// per joint, all commands arrive here as JSON messages on one connection
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
    // Send current positions immediately on connect so UI syncs up
    broadcastPositions();

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    // Only handle complete, text frames
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0; // null-terminate

      StaticJsonDocument<64> doc;
      DeserializationError err = deserializeJson(doc, (char *)data);
      if (err) {
        Serial.print("JSON parse error: ");
        Serial.println(err.c_str());
        return;
      }

      // Expected format: {"joint": 0, "value": 90}
      int joint = doc["joint"];
      int value = doc["value"];
      applyCommand(joint, value);

      // Echo back updated positions to all clients (including sender)
      broadcastPositions();
    }

  } else if (type == WS_EVT_ERROR) {
    Serial.printf("WS error on client #%u\n", client->id());
  }
}

// ─── ESP-NOW CALLBACK ─────────────────────────────────────────────────────
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (millis() - previousserialmessage >= 2000) {
    previousserialmessage = millis();
    Serial.println("Incoming from ToF");
    Serial.printf(" A: %.1f mm  B: %.1f mm\n", myData.a, myData.b);
    if (myData.os == 0) Serial.println("No object detected");
    else Serial.printf("Object size: %.1f mm\n", myData.os);
  }
}

// ─── SERIAL INPUT (unchanged from original) ───────────────────────────────
String serialData = "";

void handleSerial() {
  if (!Serial.available()) return;
  serialData = Serial.readStringUntil('\n');
  serialData.trim();

  if (serialData.equalsIgnoreCase("close")) {
    Close = true;
    return;
  }
  int commaIndex = serialData.indexOf(',');
  if (commaIndex <= 0) { Serial.println("Use: joint,value"); return; }

  int joint = serialData.substring(0, commaIndex).toInt();
  int value = serialData.substring(commaIndex + 1).toInt();

  applyCommand(joint, value);
  broadcastPositions(); // keep browser in sync when commanded via serial too
}

// ─── GRIPPER AUTO-CLOSE ───────────────────────────────────────────────────
void closing() {
  if (!Close) return;
  if (gripperCurrent == 180) { Close = false; return; }
  if (millis() - previousclosing >= SERVO_DELAY) {
    previousclosing = millis();
    gripperCurrent--;
    gripper.write(angleToPulse(gripperCurrent));
    if (myData.a <= 18 && myData.b <= 18) {
      Close = false;
      Serial.println("Gripper closed to object");
    }
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  gripper.attach(gripperpin);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin("YourHomeWifiName", "YourHomeWifiPassword");
  Serial.print("Connecting to home wifi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nHome wifi IP: " + WiFi.localIP().toString());

  WiFi.softAP(ssid, password);
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  // Read starting positions
  baseroatatiion   = Readposition(Serial1, /*base id*/   1);
  tilt1            = Readposition(Serial1, /*tilt1 id*/  2);
  tilt2            = Readposition(Serial1, /*tilt2 id*/  3);
  Rotationgripper  = Readposition(Serial1, /*twist id*/  4);
  Gripperclosingmm = 62;

  // Attach WebSocket handler and add to server
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve the webpage
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", webpage);
  });

  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Server started — WebSocket on ws://[IP]/ws");

  // ESP-NOW
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Ready");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────
void loop() {
  ws.cleanupClients(); // drop stale connections, important for AsyncWebSocket
  handleSerial();
  closing();

  // Broadcast positions on a timer so the browser stays in sync
  // even if something moves the arm outside of a WS command (e.g. serial)
  if (millis() - previousWsBroadcast >= WS_BROADCAST_INTERVAL) {
    previousWsBroadcast = millis();
    broadcastPositions();
  }
}

