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

// Webserver and socket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// pins for servo driver adafruit adn object
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
bool start = false;     

uint8_t lx = 0;  // left joystick X
uint8_t ly =  0;// left joystick Y
uint8_t rx = 0; // right joystick X
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


// Servo arm values
int baseroatatiion = 0;
int tilt1 = 0;
int tilt2 = 0;
int Rotationgripper = 0;
int Gripperclosing = 0;

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
unsigned long previousservo_hi = 0;
unsigned long previousreadtempsensor = 0;
unsigned long previousWsBroadcast= 0;
unsigned long previousdht1 = 0;
unsigned long previousdht2 = 0;
unsigned long previousdht3 = 0;
unsigned long previousalarm = 0;
unsigned long previoustest = 0;

// interval
constexpr u16_t WS_BROADCAST_INTERVAL = 200; 
u16_t buzzerDelay = 500; 

// ESPNOW
struct struct_message { float a; float b; float os; };
struct_message myData;


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
      triangle = (buf[3] >> 0) & 1;
      cross    = (buf[3] >> 2) & 1;
      circle   = (buf[3] >> 1) & 1;
      square   = (buf[3] >> 3) & 1;
      l1       = (buf[3] >> 4) & 1;
      r1       = (buf[3] >> 5) & 1;
      l2       = (buf[3] >> 6) & 1;
      r2       = (buf[3] >> 7) & 1;

      btnselect   = (buf[4] >> 0) & 1;
      start    = (buf[4] >> 1) & 1;

      lx = buf[6]; // left joystick X
      ly = buf[7]; // left joystick Y
      rx = buf[8]; // right joystick X
      ry = buf[9]; // right joystick Y
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
    triangle, cross, circle, square, l1, l2, r1, r2, btnselect, start,
    lx, ly, rx, ry
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
  u8_t channel = temp-3;
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

    if (millis() - previousservo_hi >= SERVO_DELAY){
      previousservo_hi = millis();
    if (currentpos < targetpos){
      currentpos++;
    } else {
      currentpos--;
    }
    ledcWrite(SERVO_CHANNELS[servo_id], US_TO_DUTY(angleToPulse_hi(currentpos)));
  }
}

void moveservos(){
  servo_move_HI(0, roationBC, baseroatatiion);
  servo_move_HI(1, tilt1C, tilt1);
  servo_move_HI(2, tilt2C, tilt2);
  servo_move_HI(3, roationGC, Rotationgripper);
  servo_move_ada(4, gripperC, Gripperclosing, 1);
}

void commands_for_p2c(){
  if (start){
    baseroatatiion = 90;
    tilt1 = 90;
    tilt2 = 90;
    Rotationgripper = 90;
    Gripperclosing = 180;
    servo_move_HI(0, roationBC, baseroatatiion);
    servo_move_HI(1, tilt1C, tilt1);
    servo_move_HI(2, tilt2C, tilt2);
    servo_move_HI(3, roationGC, Rotationgripper);
    servo_move_ada(4, gripperC, Gripperclosing, 1);
  }
  if (btnselect)selectpressed = true;
  if(selectpressed){
    servoindexp2c = 0;
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

// Esp now callback function
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
// buzzzer alarm really loud
void alarm(){
  if (millis() - previousalarm >= buzzerDelay){
    previousalarm = millis();
    switch (alarmstate){
      case 0:
      ledcWriteTone(8, 750);
      alarmstate = Lasttone;
      buzzerDelay = 350;
      break;
      case 1: 
      ledcWriteTone(8, 625);
      alarmstate = Lasttone;
      buzzerDelay = 500;
      break;
      default:
      Serial.println("Something wrong happend with alarm state");
      break;
    }
  }
}

bool alarmbuzzer = false;

// Serial input 
String serialData = "";

void handleSerial() {
  if (!Serial.available()) return;
  serialData = Serial.readStringUntil('\n');
  serialData.trim();

  if(serialData.equalsIgnoreCase("alarm")) alarmbuzzer = true;
  if (serialData.equalsIgnoreCase("offalarm")) alarmbuzzer = false;

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
  if (gripperC == 180) { Close = false; return; }
  if (millis() - previousclosing >= SERVO_DELAY) {
    previousclosing = millis();
    gripperC--;
    servo_move_ada(0, gripperC, newpostiongripper, 1);
    if (myData.a <= 18 && myData.b <= 18) {
      Close = false;
      Serial.println("Gripper closed to object");
    }
  }
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
  // used when only needed wifi name and passwor : WiFi.begin("ADDIS_INGEDA", "FULLAfarta2020");
  // this is used when you need an username to
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
  Serial.println("Ready");

  // setting the angles to 90
  pwm.setPWM(0, 0, angleToPulse_sm(90));
  pwm.setPWM(1, 0, angleToPulse_ti(90));
}

// Loop
void loop() {
  ws.cleanupClients();
  handleSerial();
  closing();
  moveservos();
  p2c();
  commands_for_p2c();
  just_for_testing();
  if (alarmbuzzer) alarm();


  if (millis() - previousWsBroadcast >= WS_BROADCAST_INTERVAL) {
    previousWsBroadcast = millis();
    broadcastPositions();
  }
}
