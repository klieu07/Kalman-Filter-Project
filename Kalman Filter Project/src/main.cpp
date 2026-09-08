#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SparkFun_ISM330DHCX.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

// ---------- WiFi ----------
// const char* WIFI_SSID = "SSID";          
// const char* WIFI_PASS = "PASSWORD";
const uint16_t PORT = 3333;
WiFiServer server(PORT);
WiFiClient client;

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- IMU ----------
SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData, gyroData;

// ---------- GPS ----------
SFE_UBLOX_GNSS myGNSS;
#define GPS_RX 16
#define GPS_TX 17

// ---------- SD ----------
#define SD_CS 33
#define SD_SCK 14
#define SD_MOSI 13
#define SD_MISO 27
SPIClass hspi(HSPI);
File logFile;
bool sdOK = false;

// ---------- attitude (complementary filter) ----------
float roll = 0, pitch = 0, yaw = 0;
float biasGx = 0, biasGy = 0, biasGz = 0;
unsigned long lastMicros = 0;
const float ALPHA = 0.98f;
const float R2D = 57.29578f, D2R = 0.0174533f;

// ---------- GPS cache ----------
byte gFix = 0, gSiv = 0;
double gLat = 0, gLon = 0, gAlt = 0;
unsigned long lastGps = 0, lastFlush = 0;
unsigned long rowCount = 0;

// ---------- isometric arrow drawing ----------
const int CX = 96, CY = 40;
const float SCALE = 12.0f;

void projectVec(float wx, float wy, float wz, int &sx, int &sy) {
    float ix = (wx - wy) * 0.866f;
    float iy = (wx + wy) * 0.5f - wz;
    sx = CX + (int)(SCALE * ix);
    sy = CY + (int)(SCALE * iy);
}

void drawScreen() {
    float cr=cos(roll*D2R), sr=sin(roll*D2R);
    float cp=cos(pitch*D2R), sp=sin(pitch*D2R);
    float cyw=cos(yaw*D2R), syw=sin(yaw*D2R);

    float Xx=cyw*cp,           Xy=syw*cp,           Xz=-sp;
    float Yx=cyw*sp*sr-syw*cr, Yy=syw*sp*sr+cyw*cr, Yz=cp*sr;
    float Zx=cyw*sp*cr+syw*sr, Zy=syw*sp*cr-cyw*sr, Zz=cp*cr;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // left column: text status
    display.setCursor(0, 0);
    const char* fixStr = (gFix>=3)?"3D":(gFix==2)?"2D":"--";
    display.printf("%s S%d", fixStr, gSiv);
    display.setCursor(0, 10);
    display.printf("R%4.0f", roll);
    display.setCursor(0, 20);
    display.printf("P%4.0f", pitch);
    display.setCursor(0, 30);
    display.printf("Y%4.0f", yaw);
    display.setCursor(0, 44);
    display.printf("Log");
    display.setCursor(0, 54);
    display.printf("%lu", rowCount);

    // right side: 3D orientation tripod, forward axis = arrow
    int ox, oy, tx, ty;
    projectVec(0,0,0, ox,oy);
    projectVec(Yx*0.7f, Yy*0.7f, Yz*0.7f, tx,ty);
    display.drawLine(ox,oy,tx,ty, SSD1306_WHITE);
    projectVec(Zx*0.7f, Zy*0.7f, Zz*0.7f, tx,ty);
    display.drawLine(ox,oy,tx,ty, SSD1306_WHITE);
    projectVec(Xx, Xy, Xz, tx,ty);
    display.drawLine(ox,oy,tx,ty, SSD1306_WHITE);
    display.fillCircle(tx,ty,2, SSD1306_WHITE);

    // wifi indicator
    if (client && client.connected()) { display.setCursor(120,0); display.print("W"); }

    display.display();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Joining WiFi");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected! ESP32 IP: ");
        Serial.println(WiFi.localIP());     // <-- WRITE THIS IP DOWN
    } else {
        Serial.println("WiFi FAILED - check 2.4GHz, SSID, password");
    }
    server.begin();

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Booting...");
    display.display();

    if (!myISM.begin()) { Serial.println("IMU FAIL"); while(1) delay(10); }
    myISM.deviceReset();
    while (!myISM.getDeviceReset()) delay(1);
    myISM.setDeviceConfig();
    myISM.setBlockDataUpdate();
    myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
    myISM.setAccelFullScale(ISM_4g);
    myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
    myISM.setGyroFullScale(ISM_500dps);
    myISM.setAccelFilterLP2();
    myISM.setGyroFilterLP1();

    // GPS
    Serial2.begin(38400, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100);
    if (!myGNSS.begin(Serial2)) {
        Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
        delay(100);
        if (!myGNSS.begin(Serial2)) Serial.println("GPS not found (continuing)");
    }
    myGNSS.setUART1Output(COM_TYPE_UBX);

    // SD (optional — don't halt if missing)
    hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS, hspi)) {
        char fname[24];
        int n = 0;
        do { sprintf(fname, "/log%03d.csv", n++); } while (SD.exists(fname) && n < 1000);
        logFile = SD.open(fname, FILE_WRITE);
        if (logFile) {
            logFile.println("millis,ax,ay,az,gx,gy,gz,roll,pitch,yaw,fix,sats,lat,lon,alt");
            logFile.flush();
            sdOK = true;
            Serial.printf("Logging to %s\n", fname);
        }
    } else {
        Serial.println("SD not mounted (continuing without logging)");
    }

    // gyro bias calibration — HOLD STILL
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Calibrating gyro");
    display.println("HOLD STILL...");
    display.display();
    const int N = 300;
    float sgx=0,sgy=0,sgz=0;
    for (int i=0;i<N;i++){
        while(!myISM.checkStatus()){}
        myISM.getGyro(&gyroData);
        sgx+=gyroData.xData/1000.0f; sgy+=gyroData.yData/1000.0f; sgz+=gyroData.zData/1000.0f;
        delay(5);
    }
    biasGx=sgx/N; biasGy=sgy/N; biasGz=sgz/N;

    // seed level from gravity
    myISM.getAccel(&accelData);
    {
        float ax=accelData.xData/1000.0f, ay=accelData.yData/1000.0f, az=accelData.zData/1000.0f;
        roll  = atan2(ay, az)*R2D;
        pitch = atan2(-ax, sqrt(ay*ay+az*az))*R2D;
        yaw   = 0;
    }
    lastMicros = micros();
}

void loop() {
    unsigned long nowMs = millis();

    // accept WiFi client + send header
    if (!client || !client.connected()) {
        client = server.available();
        if (client) client.println("millis,ax,ay,az,gx,gy,gz,roll,pitch,yaw,fix,sats,lat,lon,alt");
    }

    // IMU
    if (myISM.checkStatus()) {
        myISM.getAccel(&accelData);
        myISM.getGyro(&gyroData);
    }

    // attitude update
    unsigned long nowUs = micros();
    float dt = (nowUs - lastMicros)/1e6f;
    lastMicros = nowUs;
    if (dt<=0 || dt>0.5f) dt = 0.01f;

    float ax=accelData.xData/1000.0f, ay=accelData.yData/1000.0f, az=accelData.zData/1000.0f;
    float gx=gyroData.xData/1000.0f - biasGx;
    float gy=gyroData.yData/1000.0f - biasGy;
    float gz=gyroData.zData/1000.0f - biasGz;

    float rollAcc  = atan2(ay, az)*R2D;
    float pitchAcc = atan2(-ax, sqrt(ay*ay+az*az))*R2D;
    roll  = ALPHA*(roll  + gx*dt) + (1-ALPHA)*rollAcc;
    pitch = ALPHA*(pitch + gy*dt) + (1-ALPHA)*pitchAcc;
    yaw  += gz*dt;
    if (yaw<0) yaw+=360; if (yaw>=360) yaw-=360;

    // GPS at 1 Hz
    if (nowMs - lastGps > 1000) {
        gFix = myGNSS.getFixType();
        gSiv = myGNSS.getSIV();
        gLat = myGNSS.getLatitude()/1e7;
        gLon = myGNSS.getLongitude()/1e7;
        gAlt = myGNSS.getAltitudeMSL()/1000.0;   // meters
        lastGps = nowMs;
    }

    // build row -> SD + WiFi
    char row[200];
    snprintf(row, sizeof(row),
        "%lu,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%d,%d,%.7f,%.7f,%.1f",
        nowMs, accelData.xData, accelData.yData, accelData.zData,
        gyroData.xData, gyroData.yData, gyroData.zData,
        roll, pitch, yaw, gFix, gSiv, gLat, gLon, gAlt);

    if (sdOK) { logFile.println(row); rowCount++; }
    if (client && client.connected()) client.println(row);

    if (sdOK && nowMs - lastFlush > 1000) { logFile.flush(); lastFlush = nowMs; }

    drawScreen();
    delay(20);   // ~50 Hz
}
