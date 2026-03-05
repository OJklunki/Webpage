#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_now.h>
#include <WebServer.h>
#include <WiFi.h>
#include "webpage.h"
// 

// starting webserver
WebServer server(80);

// web values
int baseroatatiion = 0;
int tilt1 = 0;
int tilt2 = 0;
int Rotationgripper = 0;
int Gripperclosingmm = 0;

// name and password wifi
constexpr const char* ssid = "MyESP";
constexpr const char* password = "formy_self";

// webpages values
bool buttonState = false;
int slidervalue = 0;

// Pins I2C servo driver
constexpr uint8_t SERVO_SDA = 26;
constexpr uint8_t SERVO_SCL = 27;

// Servo delay
constexpr int SERVO_DELAY = 30; 

// GRIPPER value
bool Close = false;


// Adafruit object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// Servo Positions 
// Change these to match where your servos physically are at power-on
int currentPos0 = 90; // channel 0 - gripper
int currentPos1 = 90; // channel 1 - rotation of gripper
int currentPos2 = 90; // channel 2 - tilting of gripper
int targetposition0 = 90;
int targetposition1 = 90;
int targetposition2 = 90;

// Serial Input 
String serialData = "";

// callbacks and sending functions 
void handleRoot() {
  server.send(200, "text/html", webpage);
}

void handlePress() {
  buttonState = !buttonState;
  server.send(200, "text/plain", buttonState ? "ON" : "OFF");

}

void handleSlider(){
  slidervalue = server.arg("val").toInt();
  Serial.print("Slidervalue: ");
  Serial.println(slidervalue);
  server.send(200, "text/plain", "Messagesent");
}



// Angle to pulse converte for sg90 servo
int angleToPulse_sg90(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 150, 550);
}

// Angle to pulse converte for sm 90 servo
int angleToPulse_sm(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, 100, 500);
}

// timing varibels for milllis()
unsigned long previousservo = 0;
unsigned long previous2servo = 0;
unsigned long previousclosing = 0;
unsigned long previousserialmessage = 0;
unsigned long previousprint = 0;


// Smooth Servo Move single
void moveServoSmooth(int channel, int &currentPos, int targetPos, int change) {
  if (currentPos == targetPos) return;
    if (millis() - previousservo >= SERVO_DELAY){
      previousservo = millis();
      if (currentPos < targetPos){
        currentPos++;
      } else {
        currentPos--;
      }
      if (change == 1){
      pwm.setPWM(channel, 0, angleToPulse_sm(currentPos));
      } else {
        pwm.setPWM(channel, 0, angleToPulse_sg90(currentPos));
      }
    }
  }

  // Smooth Servo Move dobble
void move2ServoSmooth(int &currentPos, int targetPos){
  if (currentPos == targetPos) return;
    if (millis() - previous2servo >= SERVO_DELAY){
      previous2servo = millis();
      if (currentPos < targetPos){
        currentPos++;
      } else {
        currentPos--;
     }
      pwm.setPWM(1, 0, angleToPulse_sg90(currentPos));
      pwm.setPWM(2, 0, angleToPulse_sg90(180 - currentPos));
    }
  }

//  Esp now struck message - must be exsact as transmitter
struct struct_message {
  float a;
  float b;
  float os;
};
 
// object called myData
struct_message myData;


// recive function 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (millis() - previousserialmessage >= 2000){

  Serial.println("Incoming information from tof ");
  Serial.print(" ---- Distance A: ");
  Serial.print(myData.a);
  Serial.print(" mm ---- Distance B ");
  Serial.println(myData.b);
  Serial.println(" mm ---- ");
  if (myData.os == 0){
    Serial.println("No object was detected correctly");
    Serial.println("");
  } else {
    Serial.print("Object size: ");
    Serial.print(myData.os);
    Serial.println(" mm");
    Serial.println("");
    }
  }
}

//  Setup 
void setup() {
  Serial.begin(115200);
  Wire.begin(SERVO_SDA, SERVO_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);

  // WiFi mode innisilation
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin("YourHomeWifiName", "YourHomeWifiPassword");

  Serial.print("Connecting to home wifi");
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
}
Serial.println("");
Serial.print("Home wifi IP: ");
Serial.println(WiFi.localIP());
  WiFi.softAP(ssid, password);
  Serial.print("Web ip: ");
  Serial.println(WiFi.softAPIP());
  server.begin();

  server.on("/", [](){
    server.send(200, "text/html", webpage);
  });
  server.on("/stepper", [](){
    baseroatatiion = server.arg("val").toInt();
    server.send(200, "text/plain", "OK");
  });




  // debug function for webpage
  server.onNotFound([](){
  Serial.println("Not found: " + server.uri());
  server.send(404, "text/plain", "Not found");
  });

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  Serial.println("ESP-NOW Receiver Ready");
  esp_now_register_recv_cb(OnDataRecv);

  // setting defualt posistion 
  pwm.setPWM(0, 0, angleToPulse_sm(90));
  pwm.setPWM(1, 0, angleToPulse_sg90(90));
  pwm.setPWM(2, 0, angleToPulse_sg90(90));
  pwm.setPWM(3, 0, angleToPulse_sg90(90));

    Serial.println("Gripper Control Ready!");
}


// Serial Handling 
void handleSerial() {
  if (Serial.available() > 0) {
    serialData = Serial.readStringUntil('\n');
    serialData.trim();

    if (serialData == "CLOSE" || serialData == "Close" || serialData == "close"){
      Close = true;
      return;
    }

    int commaIndex = serialData.indexOf(',');
    if (commaIndex <= 0) {
      Serial.println("Error: Use '1,angle' or '2,angle' or ");
      return;
    }

    String firstValue  = serialData.substring(0, commaIndex);
    String secondValue = serialData.substring(commaIndex + 1);
    int val1 = firstValue.toInt();
    int temp = secondValue.toInt();

    if (temp < 0 || temp > 180) {
      Serial.println("Error: Angle must be 0-180");
      return;
    }
    switch(val1) {
      case 1:
      targetposition0 = temp;
      Serial.print("Gripper -> ");
      Serial.println(targetposition0);
      break;
      
      case 2:
      targetposition1 = temp;
      Serial.print("Gripper was rotated ");
      Serial.println(targetposition1);
      break;

      case 3:
      targetposition2 = temp;
      Serial.print("Arm was tilted: ");
      Serial.print(targetposition2);
      break;

      default:
      Serial.println("Something was not correct in servo movment");
      break;
    }
  }
}

void closing(){
  if (Close){
    if (currentPos0 <= 0){
      Close = false;
      Serial.println("Set the gripper first to an angle");
      return;
    }
    if (millis() - previousclosing >= SERVO_DELAY){
      previousclosing = millis();
      currentPos0--;
      pwm.setPWM(0, 0, angleToPulse_sm(currentPos0));
      if (myData.a <= 18  && myData.b <= 18){
        Close = false;
        Serial.println("Gripper is closed to object");
      }
    }
  }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  handleSerial();
  closing();
  if (!Close) moveServoSmooth(0, currentPos0, targetposition0, 1);
  moveServoSmooth(1, currentPos1, targetposition1, 0);
  move2ServoSmooth(currentPos2, targetposition2);
}
