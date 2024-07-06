#include <Wire.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <EEPROM.h>

MPU6050 mpu6050(Wire);

const char* ssid = "Gerialkot-Home";
const char* password = "4dNDtndm4AN";
const char* pc_ip = "192.168.8.134";
const int pc_port = 8888;

WiFiUDP udp;
WebServer server(80);

float pitchFiltered = 0, rollFiltered = 0, yawFiltered = 0;

float alphaPitch = 0.5f; // Szűrési erősség pitch tengelyhez
float alphaRoll = 0.5f;  // Szűrési erősség roll tengelyhez
float alphaYaw = 0.5f;   // Szűrési erősség yaw tengelyhez

bool sendPitch = true, sendRoll = true, sendYaw = true;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5; // 5ms-enként küld adatot

void setup() {
  Serial.begin(115200);
  Wire.begin(15, 22);  // SDA, SCL
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  EEPROM.begin(512);
  loadSettings();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("IP address: " + WiFi.localIP().toString());

  setupWebServer();
}

void loop() {
  server.handleClient();

  mpu6050.update();
  
  float pitch = mpu6050.getAngleX();
  float roll = mpu6050.getAngleY();
  float yaw = -mpu6050.getAngleZ(); // Invert yaw

  pitchFiltered = applyLowPassFilter(pitch, pitchFiltered, alphaPitch);
  rollFiltered = applyLowPassFilter(roll, rollFiltered, alphaRoll);
  yawFiltered = applyLowPassFilter(yaw, yawFiltered, alphaYaw);

  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    sendFREEDData(pitchFiltered, rollFiltered, yawFiltered);
  }
}

float applyLowPassFilter(float input, float &estimate, float alpha) {
  estimate = alpha * input + (1.0f - alpha) * estimate;
  return estimate;
}

void sendFREEDData(float pitch, float roll, float yaw) {
  uint8_t freedMessage[29] = {0xD1}; // Start byte
  freedMessage[1] = 0x00; // Camera ID

  int32_t pan = sendPitch ? static_cast<int32_t>(pitch * 32768) : 0;
  int32_t tilt = sendYaw ? static_cast<int32_t>(yaw * 32768) : 0;
  int32_t rollVal = sendRoll ? static_cast<int32_t>(roll * 32768) : 0;

  freedMessage[2] = (tilt >> 16) & 0xFF;
  freedMessage[3] = (tilt >> 8) & 0xFF;
  freedMessage[4] = tilt & 0xFF;
  freedMessage[5] = (pan >> 16) & 0xFF;
  freedMessage[6] = (pan >> 8) & 0xFF;
  freedMessage[7] = pan & 0xFF;
  freedMessage[8] = (rollVal >> 16) & 0xFF;
  freedMessage[9] = (rollVal >> 8) & 0xFF;
  freedMessage[10] = rollVal & 0xFF;

  // Position, Zoom, Focus data remains unchanged

  uint8_t checksum = 0x40;
  for (int i = 0; i < 26; i++) {
    checksum -= freedMessage[i];
  }
  checksum %= 256;
  freedMessage[26] = checksum;

  freedMessage[27] = 0;
  freedMessage[28] = 0;

  udp.beginPacket(pc_ip, pc_port);
  udp.write(freedMessage, sizeof(freedMessage));
  udp.endPacket();
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/angles", HTTP_GET, handleAngles);
  server.begin();
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32 MPU6050 VPCAM by Gerialkot</h1>";
  html += "<form method='POST' action='/update'>";
  html += "Alpha Pitch: (Lower value is smoother motion) <input type='range' min='0.01' max='0.99' step='0.01' name='alphaPitch' value='" + String(alphaPitch) + "'><br>";
  html += "Alpha Roll: (Lower value is smoother motion) <input type='range' min='0.01' max='0.99' step='0.01' name='alphaRoll' value='" + String(alphaRoll) + "'><br>";
  html += "Alpha Yaw: (Lower value is smoother motion) <input type='range' min='0.01' max='0.99' step='0.01' name='alphaYaw' value='" + String(alphaYaw) + "'><br>";
  html += "Send Pitch: <input type='checkbox' name='pitch' " + String(sendPitch ? "checked" : "") + "><br>";
  html += "Send Roll: <input type='checkbox' name='roll' " + String(sendRoll ? "checked" : "") + "><br>";
  html += "Send Yaw: <input type='checkbox' name='yaw' " + String(sendYaw ? "checked" : "") + "><br>";
  html += "<input type='submit' value='Update'>";
  html += "</form>";
  html += "<div id='angles'></div>";
  html += "<script>"
          "setInterval(function() {"
          "fetch('/angles').then(response => response.json()).then(data => {"
          "document.getElementById('angles').innerHTML = 'Pitch: ' + data.pitch + '<br>Roll: ' + data.roll + '<br>Yaw: ' + data.yaw;"
          "});"
          "}, 100);"
          "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleUpdate() {
  if (server.hasArg("alphaPitch")) alphaPitch = server.arg("alphaPitch").toFloat();
  if (server.hasArg("alphaRoll")) alphaRoll = server.arg("alphaRoll").toFloat();
  if (server.hasArg("alphaYaw")) alphaYaw = server.arg("alphaYaw").toFloat();
  sendPitch = server.hasArg("pitch");
  sendRoll = server.hasArg("roll");
  sendYaw = server.hasArg("yaw");
  
  saveSettings();
  
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleAngles() {
  String json = "{\"pitch\":" + String(pitchFiltered) + ",\"roll\":" + String(rollFiltered) + ",\"yaw\":" + String(yawFiltered) + "}";
  server.send(200, "application/json", json);
}

void loadSettings() {
  EEPROM.get(0, alphaPitch);
  EEPROM.get(4, alphaRoll);
  EEPROM.get(8, alphaYaw);
  EEPROM.get(12, sendPitch);
  EEPROM.get(13, sendRoll);
  EEPROM.get(14, sendYaw);
}

void saveSettings() {
  EEPROM.put(0, alphaPitch);
  EEPROM.put(4, alphaRoll);
  EEPROM.put(8, alphaYaw);
  EEPROM.put(12, sendPitch);
  EEPROM.put(13, sendRoll);
  EEPROM.put(14, sendYaw);
  EEPROM.commit();
}
