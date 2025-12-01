#include <Wire.h>
#include "MPU6050.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

const char* ssid = "BalanceBot_AP";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer webSocket(81);

Preferences prefs;

// PID & Speed (ไม่มีค่าเดิม)
float Kp_b, Ki_b, Kd_b;
float Kp_s, Ki_s, Kd_s;
float maxSpeed;

float lastErr_b=0, integral_b=0;
float lastErr_s=0, integral_s=0;

// PINs
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

MPU6050 mpu;
volatile long leftCount=0,rightCount=0;

float angle=0, gyroRate=0, accelAngle=0;
float balance_out=0, speed_out=0;
float target_angle=-0.4, target_speed=0, turn_offset=0;
float filteredAngle=0, bias=0;
float dt=0;
unsigned long lastTime=0;
bool balanceEnabled=false;

float Q_angle=0.001, Q_gyro=0.003, R_angle=0.05;
float P[2][2]={{1,0},{0,1}};

void IRAM_ATTR leftEncoderA(){ leftCount += (digitalRead(L_ENC_B)?1:-1); }
void IRAM_ATTR rightEncoderA(){ rightCount += (digitalRead(R_ENC_B)?1:-1); }

float kalmanFilter(float newAngle,float newRate,float dt){
  float rate=newRate-bias;
  filteredAngle+=dt*rate;
  P[0][0]+=dt*(dt*P[1][1]-P[0][1]-P[1][0]+Q_angle);
  P[0][1]-=dt*P[1][1];
  P[1][0]-=dt*P[1][1];
  P[1][1]+=Q_gyro*dt;
  float S=P[0][0]+R_angle;
  float K[2]; K[0]=P[0][0]/S; K[1]=P[1][0]/S;
  float y=newAngle-filteredAngle;
  filteredAngle+=K[0]*y;
  bias+=K[1]*y;
  float P00_temp=P[0][0], P01_temp=P[0][1];
  P[0][0]-=K[0]*P00_temp; P[0][1]-=K[0]*P01_temp;
  P[1][0]-=K[1]*P00_temp; P[1][1]-=K[1]*P01_temp;
  return filteredAngle;
}

void setMotor(int pin1,int pin2,int pwmCh,int speed){
  speed=constrain(speed,-maxSpeed,maxSpeed);
  if(speed>=0){ digitalWrite(pin1,HIGH); digitalWrite(pin2,LOW); ledcWrite(pwmCh,speed); }
  else{ digitalWrite(pin1,LOW); digitalWrite(pin2,HIGH); ledcWrite(pwmCh,-speed); }
}

float pid_balance(float a){
  float err=target_angle-a;
  integral_b+=err*dt;
  float d=(err-lastErr_b)/dt;
  lastErr_b=err;
  return Kp_b*err + Ki_b*integral_b + Kd_b*d;
}

float pid_speed(float s){
  float err=target_speed-s;
  integral_s+=err*dt;
  float d=(err-lastErr_s)/dt;
  lastErr_s=err;
  return Kp_s*err + Ki_s*integral_s + Kd_s*d;
}

// ================= Save/Load ====================
void saveParams(){
  prefs.begin("bbot", false);
  prefs.putFloat("Kp_b", Kp_b); prefs.putFloat("Ki_b", Ki_b); prefs.putFloat("Kd_b", Kd_b);
  prefs.putFloat("Kp_s", Kp_s); prefs.putFloat("Ki_s", Ki_s); prefs.putFloat("Kd_s", Kd_s);
  prefs.putFloat("spd", maxSpeed);
  prefs.end();
}

void loadParams(){
  prefs.begin("bbot", true);
  Kp_b=prefs.getFloat("Kp_b", NAN); Ki_b=prefs.getFloat("Ki_b", NAN); Kd_b=prefs.getFloat("Kd_b", NAN);
  Kp_s=prefs.getFloat("Kp_s", NAN); Ki_s=prefs.getFloat("Ki_s", NAN); Kd_s=prefs.getFloat("Kd_s", NAN);
  maxSpeed=prefs.getFloat("spd", NAN);
  prefs.end();
}

// ================= Web Page =====================
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;}input{width:60px;}</style>
</head>
<body>
<h2>BalanceBot PID Control</h2>

<h3>Balance PID</h3>
Kp <input id="Kp_b" type="range" min="0" max="200" step="0.1"><span id="vKp_b"></span><br>
Ki <input id="Ki_b" type="range" min="0" max="50" step="0.01"><span id="vKi_b"></span><br>
Kd <input id="Kd_b" type="range" min="0" max="50" step="0.1"><span id="vKd_b"></span><br>

<h3>Speed PID</h3>
Kp <input id="Kp_s" type="range" min="0" max="10" step="0.01"><span id="vKp_s"></span><br>
Ki <input id="Ki_s" type="range" min="0" max="1" step="0.001"><span id="vKi_s"></span><br>
Kd <input id="Kd_s" type="range" min="0" max="5" step="0.01"><span id="vKd_s"></span><br>

<h3>Motor Max Speed</h3>
<input id="spd" type="range" min="50" max="255" step="1"><span id="vspd"></span><br>

<h3>Realtime Data</h3>
Angle: <span id="angle">0</span> <br>
Speed: <span id="speed">0</span> <br>
PWM: <span id="pwm">0</span> <br>

<script>
let ws=new WebSocket("ws://"+location.hostname+":81/");
ws.onmessage=e=>{
  let d=JSON.parse(e.data);
  document.getElementById("angle").innerText=d.angle.toFixed(2);
  document.getElementById("speed").innerText=d.speed.toFixed(2);
  document.getElementById("pwm").innerText=d.pwm.toFixed(0);
};

// Load saved values
fetch("/params").then(r=>r.json()).then(p=>{
  for(let k in p){ 
    let el=document.getElementById(k); 
    if(el){ el.value=p[k]; document.getElementById("v"+k).innerText=p[k]; }
  }
});

// Update value display and send to ESP32
function sendVal(id){ 
  let val=document.getElementById(id).value;
  document.getElementById("v"+id).innerText=val;
  let data={}; data[id]=parseFloat(val); ws.send(JSON.stringify(data));
}

document.querySelectorAll("input[type=range]").forEach(el=>el.oninput=()=>sendVal(el.id));
</script>
</body>
</html>
)rawliteral";

// ================= WebSocket Event =================
void onWebSocketEvent(uint8_t num,WStype_t type,uint8_t* payload,size_t len){
  if(type==WStype_TEXT){
    DynamicJsonDocument doc(200);
    deserializeJson(doc,payload);
    if(doc.containsKey("Kp_b")) Kp_b=doc["Kp_b"];
    if(doc.containsKey("Ki_b")) Ki_b=doc["Ki_b"];
    if(doc.containsKey("Kd_b")) Kd_b=doc["Kd_b"];
    if(doc.containsKey("Kp_s")) Kp_s=doc["Kp_s"];
    if(doc.containsKey("Ki_s")) Ki_s=doc["Ki_s"];
    if(doc.containsKey("Kd_s")) Kd_s=doc["Kd_s"];
    if(doc.containsKey("spd")) maxSpeed=doc["spd"];
    saveParams(); // save every change
  }
}

// ================= SETUP =========================
void setup(){
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  if(!mpu.testConnection()){ Serial.println("MPU6050 Error"); while(1); }

  pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT);
  pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT);
  pinMode(L_ENC_A,INPUT_PULLUP); pinMode(L_ENC_B,INPUT_PULLUP);
  pinMode(R_ENC_A,INPUT_PULLUP); pinMode(R_ENC_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderA, CHANGE);

  ledcAttachPin(L_PWM,0); ledcAttachPin(R_PWM,1);
  ledcSetup(0,20000,8); ledcSetup(1,20000,8);

  loadParams();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);
  server.on("/", [](){ server.send_P(200,"text/html",htmlPage); });
  server.on("/params", [](){
    DynamicJsonDocument doc(200);
    doc["Kp_b"]=Kp_b; doc["Ki_b"]=Ki_b; doc["Kd_b"]=Kd_b;
    doc["Kp_s"]=Kp_s; doc["Ki_s"]=Ki_s; doc["Kd_s"]=Kd_s;
    doc["spd"]=maxSpeed;
    String out; serializeJson(doc,out); server.send(200,"application/json",out);
  });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  lastTime=micros();
  Serial.println("System Ready. Waiting to balance...");
}

// ================= LOOP =========================
void loop(){
  server.handleClient();
  webSocket.loop();

  unsigned long now=micros();
  dt=(now-lastTime)/1000000.0;
  if(dt<0.003) return;
  lastTime=now;

  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  accelAngle=atan2(ay,az)*180/PI;
  gyroRate=gx/131.0;
  angle=kalmanFilter(accelAngle,gyroRate,dt);

  if(!balanceEnabled){
    if(abs(angle)<15){ balanceEnabled=true; Serial.println("Balancing started"); }
    else{ setMotor(L_IN1,L_IN2,0,0); setMotor(R_IN1,R_IN2,1,0); return; }
  }

  static long lastL=0,lastR=0;
  long Ld=leftCount-lastL, Rd=rightCount-lastR;
  lastL=leftCount; lastR=rightCount;
  float speed=(Ld+Rd)/2.0;

  speed_out=pid_speed(speed);
  balance_out=pid_balance(angle+speed_out);
  float leftMotor=balance_out-turn_offset;
  float rightMotor=balance_out+turn_offset;

  setMotor(L_IN1,L_IN2,0,leftMotor);
  setMotor(R_IN1,R_IN2,1,rightMotor);

  StaticJsonDocument<128> data;
  data["angle"]=angle; data["speed"]=speed; data["pwm"]=balance_out;
  String json; serializeJson(data,json);
  webSocket.broadcastTXT(json);
}
