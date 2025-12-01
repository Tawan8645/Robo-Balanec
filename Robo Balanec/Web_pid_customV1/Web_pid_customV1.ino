/****************************************************
 *  BALANCE ROBOT WITH WEB PID TUNING + SAVE CONFIG
 *  ------------------------------------------------
 *  Features:
 *  - Adjust PID (Kp, Ki, Kd) via Web UI
 *  - Adjust Max Motor Power via Web UI
 *  - Save values using ESP32 NVS
 *  - Real-time angle / speed / PWM feedback
 ****************************************************/

#include <Wire.h>
#include "MPU6050.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>     // ใช้สำหรับบันทึกค่าใน NVS

Preferences prefs;           // NVS object
WebServer server(80);
WebSocketsServer webSocket(81);

MPU6050 mpu;

// ================= WiFi CONFIG ==================
const char* ssid = "BalanceBot_AP";
const char* password = "12345678";

// ================= PID PARAMETERS ==================
// ค่า default (จะถูก override โดยค่าที่จำไว้ใน NVS ตอนเริ่มระบบ)
float Kp_balance = 55.0;
float Ki_balance = 2.2;
float Kd_balance = 2.5;

// Motor Power Limit (บันทึกใน NVS เช่นเดียวกัน)
int maxMotorPower = 195;

// Speed PID
float Kp_speed = 0.2, Ki_speed = 0.02, Kd_speed = 0.0;

// ================= PIN DEFINITIONS ==================
#define L_IN1 17
#define L_IN2 5
#define L_PWM 26
#define R_IN1 18
#define R_IN2 19
#define R_PWM 25

#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 32
#define R_ENC_B 33

// ================= GLOBAL VARIABLES ==================
volatile long leftCount = 0, rightCount = 0;
float angle = 0.0, gyroRate = 0.0, accelAngle = 0.0;
float balance_output = 0.0, speed_output = 0.0;
float target_angle = -0.4, target_speed = 0.0, turn_offset = 0.0;
float filteredAngle = 0.0, bias = 0.0;
float dt = 0.0;
unsigned long lastTime = 0;
bool balanceEnabled = false;

// Kalman Filter Parameters
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.05;
float P[2][2] = {{1,0},{0,1}};


// ================= INTERRUPTS ==================
void IRAM_ATTR leftEncoderA() {
  int b = digitalRead(L_ENC_B);
  leftCount += (b == HIGH) ? 1 : -1;
}
void IRAM_ATTR rightEncoderA() {
  int b = digitalRead(R_ENC_B);
  rightCount += (b == HIGH) ? 1 : -1;
}


// ================= KALMAN FILTER ==================
float kalmanFilter(float newAngle, float newRate, float dt){
  float rate = newRate - bias;
  filteredAngle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;

  float S = P[0][0] + R_angle;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - filteredAngle;
  filteredAngle += K[0] * y;
  bias += K[1] * y;

  float P00 = P[0][0], P01 = P[0][1];
  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;

  return filteredAngle;
}


// ================= MOTOR CONTROL ==================
void setMotor(int pin1, int pin2, int pwmPin, int pwmChannel, int speed){
  speed = constrain(speed, -maxMotorPower, maxMotorPower);

  if(speed >= 0){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    ledcWrite(pwmChannel, speed);
  }else{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    ledcWrite(pwmChannel, -speed);
  }
}


// ================= PID ==================
float pid_balance(float angle){
  static float lastError = 0, integral = 0;
  float error = target_angle - angle;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;
  return (Kp_balance*error + Ki_balance*integral + Kd_balance*derivative);
}

float pid_speed(float speed){
  static float lastError = 0, integral = 0;
  float error = target_speed - speed;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;
  return (Kp_speed*error + Ki_speed*integral + Kd_speed*derivative);
}


// ================= WEB PAGE ===================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { background:#0d1117; color:#eee; font-family:sans-serif; text-align:center; }
input { width:80px; margin:5px; font-size:16px; }
button { padding:8px 20px; margin:5px; }
#joystick { width:160px; height:160px; background:#222; border-radius:50%; margin:20px auto; position:relative; }
#stick { width:60px; height:60px; background:#0f0; border-radius:50%; position:absolute; left:50px; top:50px; }
.info { font-size:18px; margin-top:10px; }
</style></head>
<body>

<h2>🤖 BalanceBot Dashboard</h2>

<!-- JOYSTICK CONTROL -->
<div id="joystick">
  <div id="stick"></div>
</div>

<div class="info">Angle: <span id="angle">0</span>°</div>
<div class="info">Speed: <span id="speed">0</span></div>
<div class="info">PWM: <span id="pwm">0</span></div>

<hr>

<h3>PID Balance Control</h3>
Kp: <input id="kp" type="number" step="0.1"><br>
Ki: <input id="ki" type="number" step="0.01"><br>
Kd: <input id="kd" type="number" step="0.1"><br>

<h3>Motor Configuration</h3>
Max Motor Power: <input id="maxpwm" type="number" step="1"><br>

<button onclick="updatePID()">Update</button>
<button onclick="savePID()">Save to ESP32</button>

<script>
let ws = new WebSocket("ws://" + location.hostname + ":81/");
let joy = document.getElementById("joystick");
let stick = document.getElementById("stick");
let rect = joy.getBoundingClientRect();
let center = {x:rect.width/2, y:rect.height/2};
let active=false;

joy.addEventListener("touchstart", ()=> active=true);
joy.addEventListener("touchend", ()=>{
  active=false;
  stick.style.left="50px"; stick.style.top="50px";
  ws.send(JSON.stringify({fwd:0,turn:0}));
});
joy.addEventListener("touchmove", e=>{
  if(!active) return;
  let t = e.touches[0];
  let dx = t.clientX - rect.left - center.x;
  let dy = t.clientY - rect.top  - center.y;
  let dist = Math.sqrt(dx*dx + dy*dy);
  let max = center.x - 30;
  if(dist > max){ dx = dx*max/dist; dy = dy*max/dist; }
  stick.style.left = (center.x+dx-30)+"px";
  stick.style.top  = (center.y+dy-30)+"px";
  ws.send(JSON.stringify({fwd:-dy/max, turn:dx/max}));
});

// ====== WEB SOCKET RECEIVE ======
ws.onmessage = e=>{
  let d = JSON.parse(e.data);

  if(d.angle !== undefined){
    angle.innerHTML = d.angle.toFixed(1);
    speed.innerHTML = d.speed.toFixed(1);
    pwm.innerHTML   = d.pwm.toFixed(0);
  }

  if(d.pid){
    kp.value = d.pid.kp;
    ki.value = d.pid.ki;
    kd.value = d.pid.kd;
  }

  if(d.motor){
    maxpwm.value = d.motor.max;
  }
};

// ===== SEND UPDATE (not save) =====
function updatePID(){
  ws.send(JSON.stringify({
    update:{ 
      kp: parseFloat(kp.value), 
      ki: parseFloat(ki.value), 
      kd: parseFloat(kd.value),
      max: parseInt(maxpwm.value)
    }
  }));
}

// ===== SAVE to NVS =====
function savePID(){
  ws.send(JSON.stringify({
    save:{
      kp: parseFloat(kp.value), 
      ki: parseFloat(ki.value), 
      kd: parseFloat(kd.value),
      max: parseInt(maxpwm.value)
    }
  }));
}
</script>
</body></html>
)rawliteral";


// ================== WEBSOCKET EVENT ===================
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
  if(type == WStype_TEXT){

    DynamicJsonDocument doc(300);
    deserializeJson(doc, payload);

    // joystick
    if(doc.containsKey("fwd")){
      target_speed = doc["fwd"].as<float>() * 100;
      turn_offset  = doc["turn"].as<float>() * 60;
    }

    // ========== UPDATE PID TEMPORARILY ==========
    if(doc.containsKey("update")){
      Kp_balance = doc["update"]["kp"];
      Ki_balance = doc["update"]["ki"];
      Kd_balance = doc["update"]["kd"];
      maxMotorPower = doc["update"]["max"];
    }

    // ========== SAVE PID TO NVS ==========
    if(doc.containsKey("save")){
      Kp_balance = doc["save"]["kp"];
      Ki_balance = doc["save"]["ki"];
      Kd_balance = doc["save"]["kd"];
      maxMotorPower = doc["save"]["max"];

      prefs.putFloat("kp", Kp_balance);
      prefs.putFloat("ki", Ki_balance);
      prefs.putFloat("kd", Kd_balance);
      prefs.putInt("max", maxMotorPower);

      Serial.println("Saved PID + Motor Config to NVS!");
    }
  }
}


// ================== SETUP ===================
void setupWeb(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", [](){ server.send_P(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void setup(){
  Serial.begin(115200);
  Wire.begin();

  // Load saved PID
  prefs.begin("pid", false);
  Kp_balance = prefs.getFloat("kp", Kp_balance);
  Ki_balance = prefs.getFloat("ki", Ki_balance);
  Kd_balance = prefs.getFloat("kd", Kd_balance);
  maxMotorPower = prefs.getInt("max", maxMotorPower);

  // Init MPU
  mpu.initialize();
  if(!mpu.testConnection()){
    Serial.println("MPU6050 Error!");
    while(1);
  }

  // Encoders
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderA, CHANGE);

  // Motors
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  ledcAttachPin(L_PWM, 0); ledcAttachPin(R_PWM, 1);
  ledcSetup(0, 20000, 8); ledcSetup(1, 20000, 8);

  setupWeb();
  lastTime = micros();

  Serial.println("Loaded settings:");
  Serial.println(Kp_balance);
  Serial.println(Ki_balance);
  Serial.println(Kd_balance);
  Serial.println(maxMotorPower);
}


// ================== LOOP ===================
void loop(){
  server.handleClient();
  webSocket.loop();

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  if(dt < 0.003) return;
  lastTime = now;

  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  accelAngle = atan2(ay, az) * 180/PI;
  gyroRate = gx / 131.0;
  angle = kalmanFilter(accelAngle, gyroRate, dt);

  // wait until upright
  if(!balanceEnabled){
    if(abs(angle) < 15){
      balanceEnabled = true;
      Serial.println("Balanced!");
    }else{
      setMotor(L_IN1,L_IN2,L_PWM,0,0);
      setMotor(R_IN1,R_IN2,R_PWM,1,0);
      return;
    }
  }

  // encoder speed
  static long lastL=0, lastR=0;
  long Ld = leftCount - lastL;
  long Rd = rightCount - lastR;
  lastL = leftCount;
  lastR = rightCount;

  float speed = (Ld + Rd) / 2.0;

  speed_output = pid_speed(speed);
  balance_output = pid_balance(angle + speed_output);

  float leftPower = balance_output - turn_offset;
  float rightPower = balance_output + turn_offset;

  setMotor(L_IN1, L_IN2, L_PWM, 0, leftPower);
  setMotor(R_IN1, R_IN2, R_PWM, 1, rightPower);

  // SEND TELEMETRY
  StaticJsonDocument<128> d;
  d["angle"] = angle;
  d["speed"] = speed;
  d["pwm"]   = balance_output;

  String msg;
  serializeJson(d, msg);
  webSocket.broadcastTXT(msg);

  // SEND PID + MOTOR EVERY 1s
  static unsigned long lastSend = 0;
  if(millis() - lastSend > 1000){
    lastSend = millis();

    StaticJsonDocument<128> cfg;
    cfg["pid"]["kp"] = Kp_balance;
    cfg["pid"]["ki"] = Ki_balance;
    cfg["pid"]["kd"] = Kd_balance;
    cfg["motor"]["max"] = maxMotorPower;

    String out;
    serializeJson(cfg, out);
    webSocket.broadcastTXT(out);
  }
}
