#include <Wire.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <EEPROM.h>

MPU6050 mpu6050(Wire);

const char* ssid = "WIFI HÁLÓZAT NEVE";
const char* password = "WIFI JELSZÓ";
const char* pc_ip = "FOGADÓ PC IP CÍME";
const int pc_port = 8888;

WiFiUDP udp;
WebServer server(80);

const int buzzerPin = 14;
const int redLedPin = 33; // Piros LED pin
const int greenLedPin = 26; // Zöld LED pin

bool systemReady = false;
unsigned long ledBlinkInterval = 500; // LED villogási intervallum (ms)
unsigned long lastLedToggleTime = 0;

// Hangfrekvenciák
const int NOTE_C5 = 523;
const int NOTE_D5 = 587;
const int NOTE_E5 = 659;
const int NOTE_G5 = 784;

class KalmanFilter {
private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias;
    float P[2][2];

public:
    KalmanFilter() {
        Q_angle = 0.001f;
        Q_bias = 0.003f;
        R_measure = 0.03f;

        angle = 0.0f;
        bias = 0.0f;

        P[0][0] = 0.0f;
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 0.0f;
    }

    float update(float newAngle, float newRate, float dt) {
        // Prediction
        angle += dt * (newRate - bias);
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Measurement update
        float y = newAngle - angle;
        float S = P[0][0] + R_measure;
        float K[2] = {P[0][0] / S, P[1][0] / S};

        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }

    void setQangle(float newQ_angle) {
        Q_angle = newQ_angle;
    }

    void setQbias(float newQ_bias) {
        Q_bias = newQ_bias;
    }

    void setRmeasure(float newR_measure) {
        R_measure = newR_measure;
    }
};

KalmanFilter kalmanPitch, kalmanRoll, kalmanYaw;

float pitchFiltered = 0, rollFiltered = 0, yawFiltered = 0;

float kalmanStrengthPitch = 0.5f;
float kalmanStrengthRoll = 0.5f;
float kalmanStrengthYaw = 0.5f;

bool sendPitch = true, sendRoll = true, sendYaw = true;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5; // 5ms-enként küld adatot

bool kalmanCalibrated = false;
unsigned long kalmanStartTime = 0;
const unsigned long kalmanCalibrationTime = 5000; // 5 másodperc kalibrálási idő

float zuptThreshold = 0.01; // ZUPT küszöbérték
bool zuptEnabled = true;

// Aluláteresztő szűrő paraméterei
const int numReadings = 10; // A mozgóátlag minta száma
float gyroPitchReadings[numReadings];
float gyroRollReadings[numReadings];
float gyroYawReadings[numReadings];

int readIndex = 0;
float totalPitch = 0;
float totalRoll = 0;
float totalYaw = 0;

float averagePitch = 0;
float averageRoll = 0;
float averageYaw = 0;

bool useLowPassFilter = true;

void playDJISound() {
    tone(buzzerPin, NOTE_G5, 100);
    delay(150);
    tone(buzzerPin, NOTE_E5, 100);
    delay(150);
    tone(buzzerPin, NOTE_C5, 100);
    delay(150);
    noTone(buzzerPin);
}

void playWifiConnectingSound() {
    tone(buzzerPin, NOTE_C5, 100);
    delay(200);
    noTone(buzzerPin);
}

void playWifiConnectedSound() {
    tone(buzzerPin, NOTE_C5, 100);
    delay(100);
    tone(buzzerPin, NOTE_E5, 100);
    delay(100);
    tone(buzzerPin, NOTE_G5, 200);
    delay(200);
    noTone(buzzerPin);
}

void playCalibrationSound() {
    tone(buzzerPin, NOTE_C5, 50);
    delay(100);
    noTone(buzzerPin);
}

void playReadySound() {
    tone(buzzerPin, NOTE_C5, 100);
    delay(100);
    tone(buzzerPin, NOTE_E5, 100);
    delay(100);
    tone(buzzerPin, NOTE_G5, 100);
    delay(100);
    tone(buzzerPin, NOTE_C5, 200);
    delay(200);
    noTone(buzzerPin);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(15, 22);  // SDA, SCL
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    EEPROM.begin(512);
    loadSettings();

    pinMode(buzzerPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);

    delay(1000);
    playDJISound();

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        playWifiConnectingSound();
        delay(1000);
    }
    playWifiConnectedSound();

    Serial.println("IP address: " + WiFi.localIP().toString());

    setupWebServer();

    kalmanStartTime = millis();

    // Initialize readings arrays
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
        gyroPitchReadings[thisReading] = 0;
        gyroRollReadings[thisReading] = 0;
        gyroYawReadings[thisReading] = 0;
    }
}

void loop() {
    server.handleClient();

    mpu6050.update();

    float accPitch = atan2(mpu6050.getAccY(), mpu6050.getAccZ()) * RAD_TO_DEG;
    float accRoll = atan2(-mpu6050.getAccX(), sqrt(mpu6050.getAccY() * mpu6050.getAccY() + mpu6050.getAccZ() * mpu6050.getAccZ())) * RAD_TO_DEG;

    float gyroPitch = mpu6050.getGyroX();
    float gyroRoll = mpu6050.getGyroY();
    float gyroYaw = -mpu6050.getGyroZ(); // Invertáljuk a gyro yaw értéket

    // Aluláteresztő szűrő alkalmazása, ha be van kapcsolva
    if (useLowPassFilter) {
        totalPitch = totalPitch - gyroPitchReadings[readIndex];
        totalRoll = totalRoll - gyroRollReadings[readIndex];
        totalYaw = totalYaw - gyroYawReadings[readIndex];

        gyroPitchReadings[readIndex] = gyroPitch;
        gyroRollReadings[readIndex] = gyroRoll;
        gyroYawReadings[readIndex] = gyroYaw;

        totalPitch = totalPitch + gyroPitch;
        totalRoll = totalRoll + gyroRoll;
        totalYaw = totalYaw + gyroYaw;

        readIndex = readIndex + 1;

        if (readIndex >= numReadings) {
            readIndex = 0;
        }

        averagePitch = totalPitch / numReadings;
        averageRoll = totalRoll / numReadings;
        averageYaw = totalYaw / numReadings;
    } else {
        averagePitch = gyroPitch;
        averageRoll = gyroRoll;
        averageYaw = gyroYaw;
    }

    static unsigned long lastTime = 0;
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    if (zuptEnabled && abs(averagePitch) < zuptThreshold && abs(averageRoll) < zuptThreshold && abs(averageYaw) < zuptThreshold) {
        // ZUPT alkalmazása
        pitchFiltered = kalmanPitch.update(accPitch, 0, dt);
        rollFiltered = kalmanRoll.update(accRoll, 0, dt);
        yawFiltered = kalmanYaw.update(-mpu6050.getAngleZ(), 0, dt); // Invertáljuk a yaw szöget is
    } else {
        pitchFiltered = kalmanPitch.update(accPitch, averagePitch, dt);
        rollFiltered = kalmanRoll.update(accRoll, averageRoll, dt);
        yawFiltered = kalmanYaw.update(-mpu6050.getAngleZ(), averageYaw, dt); // Invertáljuk a yaw szöget is
    }

    if (!kalmanCalibrated) {
        if (millis() - kalmanStartTime < kalmanCalibrationTime) {
            if (millis() % 500 == 0) {  // Minden 500 ms-ben
                playCalibrationSound();
            }
        } else {
            kalmanCalibrated = true;
            systemReady = true;
            playReadySound();
        }
    }

    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= sendInterval) {
        lastSendTime = currentTime;
        sendFREEDData(pitchFiltered, rollFiltered, yawFiltered);
    }

    updateLedStatus(); // Update LED status in each loop
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
    html += "Kalman Alpha Pitch: <input type='range' min='0.01' max='0.99' step='0.01' name='kalmanStrengthPitch' value='" + String(kalmanStrengthPitch) + "'><br>";
    html += "Kalman Alpha Roll: <input type='range' min='0.01' max='0.99' step='0.01' name='kalmanStrengthRoll' value='" + String(kalmanStrengthRoll) + "'><br>";
    html += "Kalman Alpha Yaw: <input type='range' min='0.01' max='0.99' step='0.01' name='kalmanStrengthYaw' value='" + String(kalmanStrengthYaw) + "'><br>";
    html += "Send Pitch: <input type='checkbox' name='pitch' " + String(sendPitch ? "checked" : "") + "><br>";
    html += "Send Roll: <input type='checkbox' name='roll' " + String(sendRoll ? "checked" : "") + "><br>";
    html += "Send Yaw: <input type='checkbox' name='yaw' " + String(sendYaw ? "checked" : "") + "><br>";
    html += "ZUPT Threshold: <input type='number' step='0.001' name='zuptThreshold' value='" + String(zuptThreshold) + "'><br>";
    html += "Enable ZUPT: <input type='checkbox' name='zuptEnabled' " + String(zuptEnabled ? "checked" : "") + "><br>";
    html += "Use Low Pass Filter: <input type='checkbox' name='useLowPassFilter' " + String(useLowPassFilter ? "checked" : "") + "><br>";
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
    if (server.hasArg("kalmanStrengthPitch")) {
        kalmanStrengthPitch = server.arg("kalmanStrengthPitch").toFloat();
        kalmanPitch.setRmeasure(1.0f / kalmanStrengthPitch);
    }
    if (server.hasArg("kalmanStrengthRoll")) {
        kalmanStrengthRoll = server.arg("kalmanStrengthRoll").toFloat();
        kalmanRoll.setRmeasure(1.0f / kalmanStrengthRoll);
    }
    if (server.hasArg("kalmanStrengthYaw")) {
        kalmanStrengthYaw = server.arg("kalmanStrengthYaw").toFloat();
        kalmanYaw.setRmeasure(1.0f / kalmanStrengthYaw);
    }
    if (server.hasArg("zuptThreshold")) {
        zuptThreshold = server.arg("zuptThreshold").toFloat();
    }
    zuptEnabled = server.hasArg("zuptEnabled");
    useLowPassFilter = server.hasArg("useLowPassFilter");

    sendPitch = server.hasArg("pitch");
    sendRoll = server.hasArg("roll");
    sendYaw = server.hasArg("yaw");

    saveSettings();

    // LED állapotfrissítés webUI frissítéskor
    digitalWrite(redLedPin, HIGH);
    delay(1000);
    digitalWrite(redLedPin, LOW);

    server.sendHeader("Location", "/");
    server.send(303);
}

void handleAngles() {
    String json = "{\"pitch\":" + String(pitchFiltered) + ",\"roll\":" + String(rollFiltered) + ",\"yaw\":" + String(yawFiltered) + "}";
    server.send(200, "application/json", json);
}

void loadSettings() {
    EEPROM.get(0, kalmanStrengthPitch);
    EEPROM.get(4, kalmanStrengthRoll);
    EEPROM.get(8, kalmanStrengthYaw);
    EEPROM.get(12, sendPitch);
    EEPROM.get(13, sendRoll);
    EEPROM.get(14, sendYaw);
    EEPROM.get(15, zuptThreshold);
    EEPROM.get(19, zuptEnabled);
    EEPROM.get(20, useLowPassFilter);

    kalmanPitch.setRmeasure(1.0f / kalmanStrengthPitch);
    kalmanRoll.setRmeasure(1.0f / kalmanStrengthRoll);
    kalmanYaw.setRmeasure(1.0f / kalmanStrengthYaw);
}

void saveSettings() {
    EEPROM.put(0, kalmanStrengthPitch);
    EEPROM.put(4, kalmanStrengthRoll);
    EEPROM.put(8, kalmanStrengthYaw);
    EEPROM.put(12, sendPitch);
    EEPROM.put(13, sendRoll);
    EEPROM.put(14, sendYaw);
    EEPROM.put(15, zuptThreshold);
    EEPROM.put(19, zuptEnabled);
    EEPROM.put(20, useLowPassFilter);
    EEPROM.commit();
}

void updateLedStatus() {
    unsigned long currentMillis = millis();

    if (!systemReady) {
        if (currentMillis - lastLedToggleTime >= ledBlinkInterval) {
            lastLedToggleTime = currentMillis;
            int state = digitalRead(redLedPin);
            digitalWrite(redLedPin, !state); // Piros LED villogtatása
        }
        digitalWrite(greenLedPin, LOW);
    } else {
        digitalWrite(redLedPin, LOW);
        digitalWrite(greenLedPin, HIGH); // Zöld LED világít
    }
}
