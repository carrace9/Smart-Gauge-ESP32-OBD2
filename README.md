#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// --- สลับพินหน้าจอสำหรับบอร์ด ESP32-2432S028 รุ่นจอกระจก (Capacitive) ---
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1 
#define TFT_BL   21  // พินเปิด-ปิดไฟ Backlight จอภาพ [1]

// ประกาศไลบรารีสถาปัตยกรรมแบบเจาะจงสัญญาณ Hardware SPI ของบอร์ด CYD จอกระจก [1]
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// --- ตั้งค่าพินอ่านแรงดันแบตเตอรี่ (ADC) ---
#define BAT_ADC_PIN 34
const float VOLTAGE_DIVIDER_RATIO = 2.0; 
const float ADC_CALIBRATION_FACTOR = 3.3 / 4095.0; 

// --- ตั้งค่า UUID และชื่ออุปกรณ์ OBD2 BLE ---
const String TARGET_BLE_NAME = "OBDBLE";
static BLEUUID SERVICE_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_UUID   ("0000fff1-0000-1000-8000-00805f9b34fb");

// --- ตัวแปรจัดการสถานะ BLE ---
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

// --- คิวเก็บชุดคำสั่ง PID สำหรับลูปวนถาม (OBD Poll Timing 350ms) ---
const char* obdPIDs[] = {"010C", "010D", "010B", "0105", "012F"}; 
int currentPidIndex = 0;
unsigned long lastPollTime = 0;
const unsigned long POLL_INTERVAL = 350; 

// --- ตัวแปรเก็บค่าพารามิเตอร์สำหรับแสดงผล ---
int rpm = 0;
int speedKmh = 0;
float boostBar = 0.0;
int ectCelsius = 0;
int fuelPercent = 0;
float vBat = 0.0;
int rangeKm = 0;

// ฟังก์ชัน Callback ดักจับและถอดรหัสข้อมูลสตรีมตอบกลับจากกล่อง OBD2 BLE
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    
    String rawResponse = "";
    for (size_t i = 0; i < length; i++) {
        rawResponse += (char)pData[i];
    }
    
    // ระบบลบเว้นวรรค
    rawResponse.replace(" ", "");
    rawResponse.replace("\r", "");
    rawResponse.replace("\n", "");
    rawResponse.trim();
    
    // แปลงข้อมูลเป็นตัวพิมพ์ใหญ่ทั้งหมดเพื่อป้องกัน Error และเปรียบเทียบค่าได้แม่นยำ
    rawResponse.toUpperCase();

    // แสดงค่าดิบออก Serial Monitor เพื่อวิเคราะห์การตอบกลับของกล่อง
    Serial.print("OBD Data: ");
    Serial.println(rawResponse);

    // ดักจับข้อมูลและแปลงค่า (ตรวจสอบเฉพาะตัวพิมพ์ใหญ่)
    if (rawResponse.indexOf("410C") != -1) { 
        int idx = rawResponse.indexOf("410C");
        if (rawResponse.length() >= idx + 8) {
            String hexVal = rawResponse.substring(idx + 4, idx + 8);
            long rawHex = strtol(hexVal.c_str(), NULL, 16);
            rpm = rawHex / 4;
        }
    } 
    else if (rawResponse.indexOf("410D") != -1) {
        int idx = rawResponse.indexOf("410D");
        if (rawResponse.length() >= idx + 6) {
            String hexVal = rawResponse.substring(idx + 4, idx + 6);
            speedKmh = strtol(hexVal.c_str(), NULL, 16);
        }
    } 
    else if (rawResponse.indexOf("410B") != -1) {
        int idx = rawResponse.indexOf("410B");
        if (rawResponse.length() >= idx + 6) {
            String hexVal = rawResponse.substring(idx + 4, idx + 6);
            int mapKpa = strtol(hexVal.c_str(), NULL, 16);
            boostBar = (mapKpa - 101) * 0.01; 
            if (boostBar < 0) boostBar = 0.0;
        }
    } 
    else if (rawResponse.indexOf("4105") != -1) {
        int idx = rawResponse.indexOf("4105");
        if (rawResponse.length() >= idx + 6) {
            String hexVal = rawResponse.substring(idx + 4, idx + 6);
            ectCelsius = strtol(hexVal.c_str(), NULL, 16) - 40;
        }
    } 
    else if (rawResponse.indexOf("412F") != -1) {
        int idx = rawResponse.indexOf("412F");
        if (rawResponse.length() >= idx + 6) {
            String hexVal = rawResponse.substring(idx + 4, idx + 6);
            fuelPercent = (strtol(hexVal.c_str(), NULL, 16) * 100) / 255;
            float currentLiters = (fuelPercent / 100.0) * 80.0;
            rangeKm = currentLiters * 12.0; 
        }
    }
}

// คลาสจัดการเมื่อตรวจเจออุปกรณ์ BLE
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == TARGET_BLE_NAME.c_str()) {
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = true;
        }
    }
};

// ฟังก์ชันเริ่มเชื่อมต่อเครื่องอ่าน OBD2 BLE + คำสั่งเปิดโปรโตคอลรถดีเซล
bool connectToServer() {
    BLEClient*  pClient  = BLEDevice::createClient();
    if (!pClient->connect(myDevice)) return false;
    
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        pClient->disconnect();
        return false;
    }
    
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHAR_UUID);
    if (pRemoteCharacteristic == nullptr) {
        pClient->disconnect();
        return false;
    }
    
    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }
    
    // ล้างบัฟเฟอร์และสั่งเปิดโปรโตคอลอัตโนมัติป้องกันค่าไม่วิ่ง
    pRemoteCharacteristic->writeValue("ATZ\r", true);      delay(400);
    pRemoteCharacteristic->writeValue("ATE0\r", true);     delay(200);
    pRemoteCharacteristic->writeValue("ATL0\r", true);     delay(200);
    pRemoteCharacteristic->writeValue("ATSP0\r", true);    delay(400);
    
    connected = true;
    return true;
}

void setup() {
    Serial.begin(115200);
    
    // เริ่มทำงานขาสัญญาณพินสื่อสาร SPI หลักของระบบบอร์ดจอกระจก [1]
    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
    
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH); // สั่งเปิดสัญญาณไฟแบ็คไลท์ [1]
    
    tft.begin();
    tft.setRotation(3); // ล็อคพินหมุนกลับหัว 180 องศา
    tft.fillScreen(ILI9341_BLACK); // ย้อมพื้นหลังดำสนิท ไร้กรอบ
    
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(40, 110);
    tft.print("SCANNING FOR OBDBLE...");
    
    BLEDevice::init("ESP32_OBD_GAUGE");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
}

void drawShiftLight(int currentRpm) {
    int totalDots = 14;
    int dotRadius = 5;
    int startX = 12; 
    int spacing = 22; 
    int yPos = 12;     

    for (int i = 0; i < totalDots; i++) {
        uint16_t color = ILI9341_DARKGREEN; 
        int triggerRpm = 1000 + (i * 250);   

        if (currentRpm >= triggerRpm) {
            if (i < 6)        color = ILI9341_GREEN;  
            else if (i < 11)  color = ILI9341_YELLOW; 
            else              color = ILI9341_RED;    
        }
        tft.fillCircle(startX + (i * spacing), yPos, dotRadius, color);
    }
}

void updateBatteryVoltage() {
    int rawAdc = analogRead(BAT_ADC_PIN);
    vBat = ((float)rawAdc * ADC_CALIBRATION_FACTOR) * VOLTAGE_DIVIDER_RATIO;
}

void printDynamicValue(int x, int y, String val, uint16_t color, int size, int clearWidth) {
    tft.fillRect(x, y, clearWidth, size * 8, ILI9341_BLACK);
    tft.setTextColor(color);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    tft.print(val);
}

void updateDisplay() {
    drawShiftLight(rpm);
    printDynamicValue(100, 35, String(rpm), ILI9341_WHITE, 5, 130);
    printDynamicValue(15, 95, "S:" + String(speedKmh), ILI9341_CYAN, 4, 120);
    printDynamicValue(115, 140, "B:" + String(boostBar, 2), ILI9341_RED, 4, 140);
    printDynamicValue(185, 95, "W:" + String(ectCelsius), ILI9341_YELLOW, 4, 120);
    printDynamicValue(15, 195, "F:" + String(fuelPercent) + "%", ILI9341_MAGENTA, 4, 140);
    printDynamicValue(115, 195, String(vBat, 1) + "V", ILI9341_ORANGE, 4, 110);
    
    uint16_t goldColor = tft.color565(255, 180, 0); 
    printDynamicValue(195, 195, "R:" + String(rangeKm), goldColor, 4, 120);
}

void loop() {
    if (doConnect == true) {
        if (connectToServer()) {
            tft.fillScreen(ILI9341_BLACK); 
        }
        doConnect = false;
    }

    if (connected) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastPollTime >= POLL_INTERVAL) {
            lastPollTime = currentMillis;
            updateBatteryVoltage();
            
            String cmd = String(obdPIDs[currentPidIndex]) + "\r";
            pRemoteCharacteristic->writeValue(cmd.c_str(), true);
            
            currentPidIndex++;
            if (currentPidIndex >= 5) {
                currentPidIndex = 0;
            }
            updateDisplay();
        }
    } else if (doScan) {
        BLEDevice::getScan()->start(0);  
    }
}
