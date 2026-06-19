#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <Wire.h> 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// --- 🛠️ พินฮาร์ดแวร์ตรงรุ่น ESP32-2432S028 จอกระจกพอร์ตคู่ ---
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1  
#define TFT_BL    21 

#define TOUCH_SDA 22
#define TOUCH_SCL 27  
#define RX2_PIN   16 
#define TX2_PIN   17 

// 🎨 รหัสสีเส้นกรอบทั้ง 6 ช่องตามแบบรูปภาพสเก็ตช์ดินสอประจำตัวพี่ช่าง (RGB565)
#define FRAME_SPD     0x001F // 🔵 สีน้ำเงินเข้ม (Blue)
#define FRAME_BOOST   0xF800 // 🔴 สีแดงสด (Red)
#define FRAME_ECT     0x07FF // 🔵 สีฟ้าอ่อน (Cyan)
#define FRAME_FUEL    0x07E0 // 🟢 สีเขียว (Green)
#define FRAME_BATT    0xF81F // 🟣 สีชมพู/ม่วง (Magenta)
#define FRAME_RANGE   0xFFE0 // 🟡 สีเหลือง (Yellow)
#define COLOR_GRAY    0x7BEF // สีเทาสำหรับข้อความหัวข้อเล็กและหน่วย

const float REVO_BARO_KPA = 101.0; 
const int32_t ECT_MAX_WARN = 98;    
const float BOOST_MAX_WARN = 30.0;  
const int32_t RPM_MAX = 4500;
const int32_t RPM_WARN = 3500;

// ค่ากำหนดสำหรับคำนวณปริมาณน้ำมัน
const float TANK_CAPACITY_LITERS = 80.0; 
const float AVG_CONSUMPTION_KM_L = 12.5;  

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
BLEAdvertisedDevice myDevice;

bool flashState = false; 
bool isConnectedOBD = false;
bool deviceFound = false;
unsigned long lastFlashTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastQueryTime = 0;

int32_t tempRPM = 0;
int32_t tempSpeed = 0;
int32_t tempECT = 0;
int32_t tempFuelPercent = 0; 
int32_t calculatedRangeKM = 0;  
float boostPSI = 0.0; 
float batteryVolt = 14.1;

int32_t lastSpeed = -1, lastRPM = -1, lastECT = -1, lastFuel = -1, lastRange = -1;
float lastBoost = -1.0, lastVolt = -1.0;
bool isFirstDraw = true; 

// รหัสเลขฐาน Service UUID แท้ประจำกล่อง OBDBLE ของพี่ช่าง
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb"); 
static BLEUUID charUUID("0000fff1-0000-1000-8000-00805f9b34fb");    

void parseOBDData(uint8_t* pData, size_t length) {
  if (length < 4) return;
  String msg = "";
  for (size_t i = 0; i < length; i++) { msg += (char)pData[i]; }
  msg.replace(" ", ""); msg.toUpperCase();

  // ถอดรหัสพัสดุรอบเครื่องยนต์
  int rIdx = msg.indexOf("410C");
  if (rIdx != -1 && msg.length() >= rIdx + 8) {
    int A = strtol(msg.substring(rIdx + 4, rIdx + 6).c_str(), NULL, 16);
    int B = strtol(msg.substring(rIdx + 6, rIdx + 8).c_str(), NULL, 16);
    tempRPM = ((A * 256) + B) / 4;
  }
  // ถอดรหัสพัสดุความเร็วรถ
  int sIdx = msg.indexOf("410D");
  if (sIdx != -1 && msg.length() >= sIdx + 6) {
    tempSpeed = strtol(msg.substring(sIdx + 4, sIdx + 6).c_str(), NULL, 16);
  }
  // ถอดรหัสพัสดุความร้อนหม้อน้ำ
  int eIdx = msg.indexOf("4105");
  if (eIdx != -1 && msg.length() >= eIdx + 6) {
    tempECT = strtol(msg.substring(eIdx + 4, eIdx + 6).c_str(), NULL, 16) - 40;
  }
  // ถอดรหัสระดับน้ำมันเปอร์เซ็นต์
  int fIdx = msg.indexOf("412F");
  if (fIdx != -1 && msg.length() >= fIdx + 6) {
    int fuelRaw = strtol(msg.substring(fIdx + 4, fIdx + 6).c_str(), NULL, 16);
    tempFuelPercent = (fuelRaw * 100) / 255; 
    float currentFuelLiters = (tempFuelPercent / 100.0) * TANK_CAPACITY_LITERS;
    calculatedRangeKM = currentFuelLiters * AVG_CONSUMPTION_KM_L;
  }
  // ถอดรหัสบูสเทอร์โบจากค่าแรงดัน MAP
  int mIdx = msg.indexOf("410B");
  if (mIdx != -1 && msg.length() >= mIdx + 6) {
    int mapKpa = strtol(msg.substring(mIdx + 4, mIdx + 6).c_str(), NULL, 16);
    boostPSI = (mapKpa - REVO_BARO_KPA) * 0.145038;
    if (boostPSI < 0) boostPSI = 0;
  }
  // ถอดรหัสแรงดันไฟแบตเตอรี่รถยนต์
  int vIdx = msg.indexOf("4142");
  if (vIdx != -1 && msg.length() >= vIdx + 8) {
    int A = strtol(msg.substring(vIdx + 4, vIdx + 6).c_str(), NULL, 16);
    int B = strtol(msg.substring(vIdx + 6, vIdx + 8).c_str(), NULL, 16);
    batteryVolt = ((A * 256) + B) / 1000.0;
  }
}

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  parseOBDData(pData, length);
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice;
      deviceFound = true;
    }
  }
};

bool connectToServer() {
  pClient = BLEDevice::createClient();
  if(!pClient->connect(&myDevice)) return false;
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) { pClient->disconnect(); return false; }
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) { pClient->disconnect(); return false; }
  if(pRemoteCharacteristic->canNotify()) { pRemoteCharacteristic->registerForNotify(notifyCallback); }
  return true;
}

// 💡 แก้ไขปลดล็อกค่า 0: เติมสัญลักษณ์ปิดท้ายด้วยกุญแจ \r เพื่อสั่งให้ชิปรถยนต์ตอบสตรีมค่ากลับมา [📌]
void queryPID(String cmd) {
  if (pRemoteCharacteristic != nullptr) {
    cmd += "\r"; 
    pRemoteCharacteristic->writeValue((uint8_t*)cmd.c_str(), cmd.length(), false);
  }
}

void drawShiftLightDots(int32_t current_rpm) {
  int numDots = map(current_rpm, 0, RPM_MAX, 0, 14);
  if (numDots > 14) numDots = 14; if (numDots < 0) numDots = 0;
  for (int i = 0; i < 14; i++) {
    int cx = 20 + (i * 21); int cy = 15; int r = 5; uint16_t color = 0x2104; 
    if (i < numDots) {
      if (i < 8) color = ILI9341_GREEN;       
      else if (i < 11) color = ILI9341_YELLOW; 
      else color = ILI9341_RED;                
    }
    if (current_rpm >= RPM_WARN && i >= 11) { if (!flashState) color = ILI9341_BLACK; }
    tft.fillCircle(cx, cy, r, color);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH); 
  
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
  tft.begin(16000000); 
  tft.setRotation(3); // ล็อคคำสั่งหมุนจอกลับหัว 180 องศาแนวซิ่งหน้ารถของพี่ช่าง
  tft.fillScreen(ILI9341_BLACK); 

  tft.setTextSize(2); tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(15, 100); tft.print("DIRECT HARDWARE LINK...");
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(1.5);
  tft.setCursor(15, 140); tft.print("CONNECTING TO OBDBLE CHIP...");

  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  BLEDevice::init("ESP32_ZD_GAUGE");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastFlashTime >= 450) {
    flashState = !flashState;
    lastFlashTime = currentMillis;
  }

  // ระบบ Auto Scan เกาะพิกัดตรงชิปกล่องรถยนต์ของพี่ช่าง
  if (!isConnectedOBD) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    pBLEScan->start(1, false); 
    
    int count = pBLEScan->getResults()->getCount();
    for (int i = 0; i < count; i++) {
      BLEAdvertisedDevice device = pBLEScan->getResults()->getDevice(i);
      if (device.haveServiceUUID() && device.isAdvertisingService(serviceUUID)) {
        myDevice = device;
        pBLEScan->stop();
        
        if (connectToServer()) {
          queryPID("ATZ");   delay(1500);
          queryPID("ATE0");  delay(300);
          queryPID("ATL0");  delay(300);
          queryPID("ATS0");  delay(300);
          queryPID("ATH0");  delay(300);
          queryPID("ATSP0"); delay(1500);
          queryPID("0100");  delay(1000); // Auto detect protocol
          isConnectedOBD = true;
          break;
        }
      }
    }
    pBLEScan->clearResults();
  }

  // 💡 ปรับช่วงเวลายิงคิวถามข้อมูลเซนเซอร์เว้นระยะ 160ms เพื่อป้องกันบัฟเฟอร์ชิป OBD ค้างส่งข้อมูลไม่ทัน [📌]
  if (isConnectedOBD && (currentMillis - lastQueryTime >= 160)) {
    lastQueryTime = currentMillis;
    static int pollStep = 0;
    if (pollStep == 0) { queryPID("010C"); pollStep = 1; }      // ยิงถามรอบเครื่องยนต์
    else if (pollStep == 1) { queryPID("010D"); pollStep = 2; } // ยิงถามความเร็วรถ
    else if (pollStep == 2) { queryPID("0105"); pollStep = 3; } // ยิงถามความร้อนน้ำ ECT
    else if (pollStep == 3) { queryPID("012F"); pollStep = 4; } // ยิงถามระดับเปอร์เซ็นต์น้ำมัน
    else if (pollStep == 4) { queryPID("010B"); pollStep = 5; } // ยิงถามบูสไอดี MAP
    else if (pollStep == 5) { queryPID("0142"); pollStep = 0; } // ยิงถามโวลต์แบตเตอรี่
  }

  // ส่วนของการวาดโครงสร้างแสดงผล 6 ช่องสีแบบหงาย 180 องศาตรงตามแบบกระดาษสเก็ตช์
  if (isConnectedOBD && (currentMillis - lastDisplayTime >= 150)) {
    lastDisplayTime = currentMillis;
    
    if(isFirstDraw) {
      tft.fillScreen(ILI9341_BLACK);
      tft.drawRect(8, 115, 98, 52, FRAME_SPD);    
      tft.drawRect(111, 115, 98, 52, FRAME_BOOST); 
      tft.drawRect(214, 115, 98, 52, FRAME_ECT);   
      tft.drawRect(8, 175, 98, 52, FRAME_FUEL);    
      tft.drawRect(111, 175, 98, 52, FRAME_BATT);  
      tft.drawRect(214, 175, 98, 52, FRAME_RANGE); 

      tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(18, 122);  tft.print("SPD"); tft.setCursor(118, 122); tft.print("BOOST"); tft.setCursor(224, 122); tft.print("ECT");
      tft.setCursor(18, 182);  tft.print("FUEL"); tft.setCursor(125, 182); tft.print("BAT"); tft.setCursor(220, 182); tft.print("RANGE");

      tft.setTextSize(1); tft.setTextColor(COLOR_GRAY);
      tft.setCursor(84, 154);  tft.print("km"); tft.setCursor(184, 154); tft.print("PSI"); tft.setCursor(292, 154); tft.print("C"); 
      tft.setCursor(90, 214);  tft.print("%"); tft.setCursor(196, 214); tft.print("v"); tft.setCursor(290, 214); tft.print("km");
      isFirstDraw = false;
    }

    drawShiftLightDots(tempRPM);

    // ⚪ 1. รอบเครื่องยนต์ขนาดยักษ์ตรงกลางบน (Size 5)
    if (tempRPM != lastRPM) {
      tft.fillRect(100, 35, 110, 38, ILI9341_BLACK); tft.setTextColor(ILI9341_WHITE); tft.setTextSize(5); 
      tft.setCursor(105, 35); tft.print(tempRPM);
      tft.fillRect(215, 53, 30, 15, ILI9341_BLACK); tft.setTextSize(1.5); tft.setTextColor(COLOR_GRAY); tft.setCursor(215, 53); tft.print("RPM");
    }
  }
}
