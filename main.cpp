#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
// REMOVED: #include <ESP32Servo.h> — conflicts with manual LEDC servo control
#include <ArduinoJson.h>
#include "Webpage.h"
#include <Adafruit_PWMServoDriver.h>
#include "esp_wpa2.h"

// Webserver and socket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// pins for servo driver adafruit and object
constexpr uint8_t SERVO_SDA = 33;
constexpr uint8_t SERVO_SCL = 32;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// tx pin for p2c and retriving variables
constexpr uint8_t PS2_TX_PIN = 34;
uint8_t buf[12];
uint8_t bufIndex = 0;
bool synced = false;
bool selectpressed = false;
u8_t servoindexp2c = 0;
bool selectedservo = false;

// varibels from p2 controller
bool triangle = false; 
bool cross = false;    
bool circle = false;      
bool square = false;    
bool l1 = false;        
bool r1 = false;        
bool l2 = false;       
bool r2 = false;   
bool btnselect = false;    
bool btnstart = false;  // renamed from 'start' (reserved name on ESP32)

uint8_t lx = 0;  // left joystick X
uint8_t ly = 0;  // left joystick Y
uint8_t rx_joy = 0;  // right joystick X  renamed from 'rx' (conflicts with ESP-NOW)
uint8_t ry = 0; // right joystick Y

// servo pins from hiwonder controller 
constexpr u8_t SERVO_1_PIN = 19;
constexpr u8_t SERVO_2_PIN = 18;
constexpr u8_t SERVO_3_PIN = 5;
constexpr u8_t SERVO_4_PIN = 4;
constexpr u8_t SERVO_5_PIN = 0;
constexpr u8_t SERVO_6_PIN = 15;

// LEDC setting
constexpr u8_t LEDC_FREQ_HZ = 50;     // 50Hz = 20ms period (standard servo)
constexpr u8_t LEDC_RES_BITS = 16;     // 16-bit = 0–65535

// convertion 
#define US_TO_DUTY(us)  ((uint32_t)(us) * 65535 / 20000)

constexpr u8_t SERVO_NUM = 6;

static const uint8_t SERVO_PINS[SERVO_NUM] = {
    SERVO_1_PIN,
    SERVO_2_PIN,
    SERVO_3_PIN,
    SERVO_4_PIN,
    SERVO_5_PIN,
    SERVO_6_PIN
};

static const uint8_t SERVO_CHANNELS[SERVO_NUM] = { 2,3,4,5,6,7};

constexpr u16_t min_Duty_HI = 500;
constexpr u16_t max_Duty_HI = 2500;


// Servo arm values — initialised to 90 to match current positions on boot
int baseroatatiion = 90;
int tilt1 = 90;
int tilt2 = 90;
int Rotationgripper = 90;
int Gripperclosing = 90;

// Wifi name and password
constexpr const char* ssid = "MyESP";
constexpr const char* password = "formy_self";

// buzzer pin
constexpr u8_t buzzerpin = 27;

// currentpostionf for servo
u16_t  roationBC = 90;
u16_t  gripperC = 90;
u16_t  roationGC = 90;
u16_t  tilt1C = 90;
u16_t  tilt2C = 90;


// Gripper servo
bool Close = false;
int newpostiongripper = 0;
u8_t SERVO_DELAY = 30;

// Buzzer flag and statefunction
bool Buzzer = false;
bool alarmbuzzer = false;

enum State {
  Firsttone,
  Lasttone,
  off
};
State alarmstate = off;


// Serial read flags
bool tempread    = false;
bool postionread = false;
bool idread      = false;


// Timing
unsigned long previousclosing = 0;
unsigned long previousserialmessage = 0;
unsigned long previousservo_ada = 0;
unsigned long previousservo_hi[SERVO_NUM] = {};
unsigned long previousreadtempsensor = 0;
unsigned long previousWsBroadcast= 0;
unsigned long previousalarm = 0;
unsigned long previoustest = 0;
unsigned long previousFanSend = 0;

// interval
constexpr u16_t WS_BROADCAST_INTERVAL = 200; 
u16_t buzzerDelay = 500;
constexpr uint16_t FAN_SEND_INTERVAL = 1000;

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


void p2c(){
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (!synced) {
      if (bufIndex == 0 && b == 0x55) { buf[bufIndex++] = b; }
      else if (bufIndex == 1 && b == 0x55) { buf[bufIndex++] = b; synced = true; }
      else { bufIndex = 0; } 
      continue;
    }
    buf[bufIndex++] = b;
    if (bufIndex == 12) {
      // DEBUG - remove once working
      Serial.print("RAW: ");
      for(int i = 0; i < 12; i++) Serial.printf("%02X ", buf[i]);
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
      btnselect =  (buf[6] & 0xFF) & 1;
      btnstart  = ((buf[6] & 0xFF) >> 1) & 1;
      lx = buf[8]; // left joystick X
      ly = buf[9]; // left joystick Y
      rx_joy = buf[10]; // right joystick X
      ry = buf[11]; // right joystick Y
      bufIndex = 0;
      synced   = false;
    }
  }
}

void just_for_testing(){
  if(millis() - previoustest >= 1000){
    previoustest = millis();
    Serial.printf(
    "BTN: tri=%d crs=%d cir=%d sqr=%d L1=%d L2=%d R1=%d R2=%d sel=%d sta=%d | "
    "LJ: x=%3d y=%3d  RJ: x=%3d y=%3d\n",
    triangle, cross, circle, square, l1, l2, r1, r2, btnselect, btnstart,
    lx, ly, rx_joy, ry
);
  }
}


void servo_init() {
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ledcSetup(SERVO_CHANNELS[i], LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_PINS[i], SERVO_CHANNELS[i]);
  }
}

void buzzer_init(){
  ledcSetup(8, 2000, 8);
  ledcAttachPin(buzzerpin, 8);
  ledcWrite(8, 0);  // ensure buzzer starts silent
}


// Helper for sg servo 
int angleToPulse_sm(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 150, 550);
}

int angleToPulse_ti(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 150, 550);
}


// helper for hiwonder servo
int angleToPulse_hi(int angle){
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, min_Duty_HI, max_Duty_HI);
}


void servo_move_ada(uint8_t temp, u16_t &currentPos, u16_t targetPos, uint8_t change) {
  u8_t channel = temp-4;
  if (currentPos == targetPos) return;
    if (millis() - previousservo_ada >= SERVO_DELAY){
      previousservo_ada = millis();
      if (currentPos < targetPos){
        currentPos++;
      } else {
        currentPos--;
      }
      if (change == 1){
      pwm.setPWM(channel, 0, angleToPulse_sm(currentPos));
      } else {
        pwm.setPWM(channel, 0, /* angle to puls for titankongrc*/ (currentPos));
      }
    }
  }


void servo_move_HI(uint8_t servo_id, uint16_t &currentpos, u16_t targetpos) {
  if (servo_id >= SERVO_NUM) {
    Serial.println("ERROR: servo_id out of range (1-6)");
    return;
    } 
    if (targetpos > 180){
      Serial.println("ERROR: The angle is to large (Max180\xC2\xB0)");
      return;
    }
    if (currentpos == targetpos) return;

    if (millis() - previousservo_hi[servo_id] >= SERVO_DELAY){
      previousservo_hi[servo_id] = millis();
    if (currentpos < targetpos){
      currentpos++;
    } else {
      currentpos--;
    }
    ledcWrite(SERVO_CHANNELS[servo_id], US_TO_DUTY(angleToPulse_hi(currentpos)));
  }
}

void servo_move_2_hi(uint16_t &currentpos, u16_t targetpos){ 
    if (targetpos > 180){
      Serial.println("ERROR: The angle is to large (Max180\xC2\xB0)");
      return;
    }
    if (currentpos == targetpos) return;

    if (millis() - previousservo_hi[1] >= SERVO_DELAY){
      previousservo_hi[1] = millis();
    if (currentpos < targetpos){
      currentpos++;
    } else {
      currentpos--;
    }
    // FIX: was ledcWrite(1,...) and ledcWrite(2,...) which are uninitialised/wrong channels
    ledcWrite(SERVO_CHANNELS[1], US_TO_DUTY(angleToPulse_hi(currentpos)));
    ledcWrite(SERVO_CHANNELS[2], US_TO_DUTY(angleToPulse_hi(180 - currentpos)));
  }
}

void moveservos(){
  servo_move_HI(0, roationBC, baseroatatiion);
  servo_move_2_hi(tilt1C, tilt1);
  servo_move_HI(3, tilt2C, tilt2);
  servo_move_HI(4, roationGC, Rotationgripper);
  servo_move_ada(5, gripperC, Gripperclosing, 1);
}

void commands_for_p2c(){
  if (btnstart){  // renamed from 'start'
    baseroatatiion = 90;
    tilt1 = 90;
    tilt2 = 90;
    Rotationgripper = 90;
    Gripperclosing = 180;
    // FIX: was servo_move_HI(1,...) and servo_move_HI(2,...) — wrong function for tilt pair
    servo_move_HI(0, roationBC, baseroatatiion);
    servo_move_2_hi(tilt1C, tilt1);
    servo_move_HI(3, tilt2C, tilt2);
    servo_move_HI(4, roationGC, Rotationgripper);
    servo_move_ada(5, gripperC, Gripperclosing, 1);
  }
  if (btnselect) selectpressed = true;  // renamed from 'select'
  if(selectpressed){
    // removed servoindexp2c = 0 here which was resetting on every loop
    if(r1){
      servoindexp2c++;
      if(servoindexp2c > 5){
        servoindexp2c = 0;
        Serial.print("Servoindex was to HIGH, (1-6)");
        Serial.println("Index was reset back to 0");
      }
    } else if (l1){
      servoindexp2c--;
      if(servoindexp2c < 1){
        servoindexp2c = 0;
        Serial.print("Servoindex was to LOW, (1-6)");
        Serial.println("Index was reset back to 0");
      }
    } else if (cross){
      selectpressed = false;

    }
  } else if (selectedservo){

  }

}

void applyCommand(int joint, int value) {
  switch (joint) {
    case 0:
      baseroatatiion = value;
      Serial.print("Arm rotaed: ");
      Serial.print(value);
      Serial.println("\xC2\xB0");
      break;
    case 1:
      tilt1 = value;
      Serial.print("Arm tilted at 1: ");
      Serial.print(value);
      Serial.println("\xC2\xB0");
      break;
    case 2:
      tilt2 = value;
      Serial.print("Arm tilted at 2: ");
      Serial.print(value);
      Serial.println("\xC2\xB0");
      break;
    case 3:
      Rotationgripper = value;
      Serial.print("Gripper was roated: ");
      Serial.print(value);
      Serial.println("\xC2\xB0");
      break;

    case 4:
      Gripperclosing = value;
      newpostiongripper = value;
      break;
    default:
      Serial.println("Unknown joint in applyCommand");
      break;
  }
}

// Broadcast postion to webserver
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

// Esp now callbacks
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
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

// buzzzer alarm really loud
void alarm(){
  if (millis() - previousalarm >= buzzerDelay){
    previousalarm = millis();
    switch (alarmstate){
      case Firsttone:
      ledcWriteTone(8, 750);
      alarmstate = Lasttone;
      buzzerDelay = 350;
      break;
      case Lasttone: 
      ledcWriteTone(8, 625);
      alarmstate = Firsttone;
      buzzerDelay = 500;
      break;
      default:
      Serial.println("Something wrong happend with alarm state");
      break;
    }
  }
}

// Serial input 
String serialData = "";

void handleSerial() {
  if (!Serial.available()) return;
  serialData = Serial.readStringUntil('\n');
  serialData.trim();

  if(serialData.equalsIgnoreCase("alarm")) { alarmbuzzer = true; return; }
  if (serialData.equalsIgnoreCase("offalarm")) { alarmbuzzer = false; return; }

  int commaIndex = serialData.indexOf(',');
  String cmd = (commaIndex > 0) ? serialData.substring(0, commaIndex) : serialData;
  int param  = (commaIndex > 0) ? serialData.substring(commaIndex + 1).toInt() : 0;

  if (serialData.equalsIgnoreCase("close")) { Close = true; return; }

  if (commaIndex <= 0) { Serial.println("Use: joint,value"); return; }
  applyCommand(cmd.toInt(), param);
  broadcastPositions();
}

// Gripper auto close
void closing() {
  if (!Close) return;
  if (gripperC == 0) { Close = false; return; }
  if (sensorData.DistanceA <= 18 && sensorData.DistanceB <= 18) {
    Close = false;
    Serial.println("Gripper closed to object");
    return;
  }
  newpostiongripper = 0;
  servo_move_ada(5, gripperC, newpostiongripper, 1);
}

// Setup
void setup() {
  Serial.begin(115200); 
  Serial2.begin(9600, SERIAL_8N1, PS2_TX_PIN, -1);
  Wire.begin(SERVO_SDA, SERVO_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);
  // setting up buzzer and servo
  buzzer_init();
  servo_init();

  WiFi.mode(WIFI_AP_STA);
  // used when only needed wifi name and passwor : 
  WiFi.begin("ADDIS_INGEDA", "FULLAfarta2020");
  // this is used when you need an username to
  /*
  constexpr const char* home_ssid     = "mrfylke-sikker";
  constexpr const char* home_username = "oleklu19@skole.mrfylke.no";
  constexpr const char* home_password = "TAE1122addi@esp32a4988";

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)home_username, strlen(home_username));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)home_username, strlen(home_username));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)home_password, strlen(home_password));
  esp_wifi_sta_wpa2_ent_enable();

  WiFi.begin(home_ssid);
// end
*/
  Serial.print("Connecting to home wifi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nHome wifi IP: " + WiFi.localIP().toString());

  WiFi.softAP(ssid, password);
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  // Read starting positions
  baseroatatiion = 90;
  tilt1 = 90;
  tilt2 = 90;
  Rotationgripper = 90;
  Gripperclosing = 90;

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
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, tofAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ToF peer");
  }

  Serial.println("Ready");

  // setting the angles to 90
  pwm.setPWM(0, 0, angleToPulse_sm(90));
  pwm.setPWM(1, 0, angleToPulse_ti(90));
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
  ledcWrite(SERVO_CHANNELS[i], US_TO_DUTY(angleToPulse_hi(90)));
  }
}

// Loop
void loop() {
  ws.cleanupClients();
  handleSerial();
  closing();
  moveservos();
  p2c();
  commands_for_p2c();
  //just_for_testing();
  if (alarmbuzzer) alarm();

  // Send fanspeed to ToF ESP every second
  if (millis() - previousFanSend >= FAN_SEND_INTERVAL) {
    previousFanSend = millis();
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
