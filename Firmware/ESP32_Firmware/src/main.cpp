#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define LED_PIN 2  // Onboard LED for many ESP32 dev boards
#define RX_PIN 16
#define TX_PIN 17

#pragma pack(push, 1) // ensure no padding
struct PacketData
{
    double latitude;    // 8 bytes
    double longitude;   // 8 bytes
    int32_t timestamp;  // 4 bytes (if 32-bit is sufficient)
};
#pragma pack(pop)

// Create a HardwareSerial object on UART2
HardwareSerial MySerial(2);

// Custom Service and Characteristic UUIDs
// #define SERVICE_UUID        "ABF0"
#define SERVICE_UUID        "ABF2"
#define CHARACTERISTIC_UUID "ABF1"

// Task handles
TaskHandle_t ledTaskHandle = NULL; 
TaskHandle_t ledTaskConnectedHandle = NULL; 

// Used to signal the tasks to stop
int ledTaskCanEnd = false; 
int ledTaskCanEndConnected = false; 

// Basic blinking tasks
void LEDTask(void *params);
void LEDTaskConnected(void *params);

// Create a global pointer to the characteristic so we can set the callback
BLECharacteristic *pCharacteristic;

void sendData(double lat, double lon, int32_t currentUnix)
{
    PacketData packet;
    packet.latitude = lat;
    packet.longitude = lon;
    packet.timestamp = currentUnix;

    // Convert struct to a raw byte array
    uint8_t *bytes = (uint8_t*)&packet;
    size_t size = sizeof(packet); // 20 bytes

    uint8_t header[2] = {0xAA,0x55};
    MySerial.write(header,2);

    MySerial.write(bytes, size);  // write raw data
    MySerial.flush();             // ensure it's sent
}


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("Connection established!");

    // 1. Stop the "fast blink" task
    ledTaskCanEnd = true;
    // Give it a moment to exit
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 2. Start the "connected blink" task
    ledTaskCanEndConnected = false;
    xTaskCreate(LEDTaskConnected, "Connected Indicator", 1000, &ledTaskCanEndConnected, 1, &ledTaskConnectedHandle);
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("Client disconnected!");

    // 1. Stop the "connected blink" task
    ledTaskCanEndConnected = true;
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 2. Restart the "fast blink" to indicate not connected
    ledTaskCanEnd = false;
    xTaskCreate(LEDTask, "Startup Indicator", 1000, &ledTaskCanEnd, 1, &ledTaskHandle);

    // Optionally resume advertising
    pServer->startAdvertising();
    Serial.println("Advertising restarted. LED is blinking fast again...");
  }
};

// Callback class to handle writes
class CoordinatesCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("Data received: ");
      Serial.println(rxValue.c_str());
      // MySerial.println(rxValue.c_str());

      // Expected format: "latitude,longitude,unixTime"
      // e.g. "42.123456,-71.654321,1679812345"
      // Parse by splitting on commas
      // A quick approach (not super robust, but works for example):
      String data = String(rxValue.c_str());
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        String latStr = data.substring(0, firstComma);
        String lonStr = data.substring(firstComma+1, secondComma);
        String timeStr = data.substring(secondComma+1);

        double lat = latStr.toDouble();
        double lon = lonStr.toDouble();
        long unixTime = timeStr.toInt(); // toInt64 if available, else parse manually

        Serial.printf("Parsed lat=%.6f, lon=%.6f, time=%ld\n", lat, lon, unixTime);
        sendData(lat,lon,unixTime);
        // Now do something with these values
      }
    }
  }
};

// Task Function - to blink the LED until BLE connects
void LEDTask(void *params) {

  // get a pointer to the changing variable
  int *pEndTask = (int*)params;

  while(*pEndTask == 0) {
    digitalWrite(LED_PIN, HIGH); 
    vTaskDelay(50/portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW); 
    vTaskDelay(50/portTICK_PERIOD_MS);
  }

  // ensure LED is off at end of task
  digitalWrite(LED_PIN, LOW); 
  // Never exit a task without deleting it first
  vTaskDelete(NULL);
}

// Task Function - to blink the LED in different frequency once connected
void LEDTaskConnected(void *params) {
  int *pEndTask = (int*)params;

  while (*pEndTask == 0) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }

  digitalWrite(LED_PIN, LOW);
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  MySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Start a blinking LED to indicate "setup / not connected"
  xTaskCreate(LEDTask, "Startup Indicator", 1000, &ledTaskCanEnd, 1, &ledTaskHandle);

  // 1. Initialize BLE
  BLEDevice::init("ESP32_BLE_Device");

  // 2. Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();

  // Optional: set a custom BLEServerCallbacks to handle connect/disconnect
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. Create custom service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 4. Create characteristic
  // pCharacteristic = pService->createCharacteristic(
  //   CHARACTERISTIC_UUID,
  //   BLECharacteristic::PROPERTY_READ |
  //   BLECharacteristic::PROPERTY_WRITE
  // );
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    "ABF3",
    BLECharacteristic::PROPERTY_WRITE
  );

  // 5. Set callback to handle writes (toggle LED)
  // pCharacteristic->setCallbacks(new LEDCharacteristicCallbacks());
  pCharacteristic->setCallbacks(new CoordinatesCallbacks());

  // 6. Start the service
  pService->start();

  // 7. Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE Advertising started. Ready to toggle LED!");
}

void loop() {
  // MySerial.println("Hello from ESP32!");
  // Serial.println("Message sent");
  // Nothing needed here; BLE events happen via callbacks
  delay(1000);
}
