#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa Pin Definitions
#define CS_PIN 18
#define IRQ_PIN 26
#define RST_PIN 14
#define SCK_PIN 5
#define MISO 19
#define MOSI 27

// LoRa frequency and sync word
#define LORA_FREQUENCY 915E6
#define LORA_SYNC_WORD 0x12

unsigned long currentMillis = millis();
unsigned long previousHzCount = 0;
const long intervalHzCount = 5000;
float validatedMsgsHz = 0.0;
int validatedMsgsQty = 0;

unsigned long lastPrintMillis = 0;
int accumulatedRSSI = 0;
int messageCount = 0;

#define START_MARKER 0xAB
#define END_MARKER 0xDE

// Data structure definition
struct RadioControlStruct {
  float steering_val;
  float throttle_val;
  float voltage;
  byte estop;
  byte control_mode;
  unsigned long counter;
  byte checksum;  
} RadioControlData;

struct TractorDataStruct{
  float speed;
  float heading;
  float voltage;
  int8_t gps_rtk_status;  
  unsigned long counter;
  byte checksum;
} TractorData;


template <typename T>
byte calculateChecksum(T& data) {
  byte* dataPtr = (byte*)&data;
  byte checksum = 0;
  for (size_t i = 0; i < sizeof(T); i++) {
    checksum ^= dataPtr[i];
  }
  return checksum;
}

/*
byte calculateChecksum(const RadioControlStruct& data) {
  byte* dataPtr = (byte*)&data;
  byte checksum = 0;
  for (size_t i = 0; i < sizeof(RadioControlStruct); i++) {  // Include checksum field
    checksum ^= dataPtr[i];
  }
  return checksum;
}
*/

/*
byte calculateRadioChecksum(const RadioControlStruct& data) {
  byte* dataPtr = (byte*)&data;
  byte checksum = 0;
  //for (size_t i = 0; i < sizeof(RadioControlStruct) - 1; i++) {  // for the transmit side
  for (size_t i = 0; i < sizeof(RadioControlStruct); i++) {  // for the receive side - calculates zero
    checksum ^= dataPtr[i];
  }
  return checksum;
}

*/
byte calculateRadioChecksum(const RadioControlStruct& data) {
  byte* dataPtr = (byte*)&data;
  byte checksum = 0;
  for (size_t i = 0; i < sizeof(RadioControlStruct) - 1; i++) {  // Exclude checksum field
    checksum ^= dataPtr[i];
  }
  return checksum;
}


void calcQtyValidatedMsgs(){  // calculate the qty of validated messages and save the results as frequency (i.e. Hz)
    if (currentMillis - previousHzCount >= intervalHzCount) {
      validatedMsgsHz = validatedMsgsQty / (intervalHzCount/1000);
      validatedMsgsQty = 0; // reset the counter every 5 seconds
      previousHzCount = currentMillis;
  }
}

void getTractorData() {
  TractorData.speed = 30.0; // replace with actual data
  TractorData.heading = 90.0; // replace with actual data
  TractorData.voltage = 12.0; // replace with actual data
  TractorData.gps_rtk_status = 1; // replace with actual data
  //TractorData.checksum = calculateChecksum(TractorData); // calculate the checksum
}
/*
void getLoRaMsg() {
  if (LoRa.parsePacket()) {
    int rssi = LoRa.packetRssi();  // capture RSSI
    accumulatedRSSI += rssi;
    messageCount++;

    while (LoRa.available()) {
      LoRa.readBytes((byte*)&RadioControlData, sizeof(RadioControlStruct));
    }
    
    //if (calculateChecksum(RadioControlData)==RadioControlData.checksum) {  // calculateRadioChecksum
    if (calculateRadioChecksum(RadioControlData)==RadioControlData.checksum) { 
      validatedMsgsQty++;  // increment the count of incoming messages
      calcQtyValidatedMsgs();  // calculate the frequency of validated messages
    } else {
        Serial.print("Checksum mismatch: ");
        Serial.print("Received checksum (");
        Serial.print(RadioControlData.checksum);
        Serial.print(") Calculated checksum (");
        Serial.print(calculateChecksum(RadioControlData));
        
        Serial.print(" | Vltg: ");
        Serial.print(RadioControlData.voltage);
        Serial.print(" | E-stp: ");
        Serial.print(RadioControlData.estop);
        Serial.print(" | Cntrl Mode: ");
        Serial.print(RadioControlData.control_mode);
        
        Serial.print(") | Cntr: ");
        Serial.println(RadioControlData.counter);
    }
  }
}
*/

void getLoRaMsg() {
  if (LoRa.parsePacket()) {
    int rssi = LoRa.packetRssi();  // capture RSSI
    accumulatedRSSI += rssi;
    messageCount++;

    // Check for start marker
    if (LoRa.read() != START_MARKER) {
      Serial.println("Missing start marker.");
      return;
    }

    // Read the data
    LoRa.readBytes((byte*)&RadioControlData, sizeof(RadioControlStruct));

    // Check for end marker
    if (LoRa.read() != END_MARKER) {
      Serial.println("Missing end marker.");
      return;
    }

    // Verify the checksum
    //if (calculateChecksum(RadioControlData)==RadioControlData.checksum) {  // calculateRadioChecksum
    if (calculateRadioChecksum(RadioControlData)==RadioControlData.checksum) { 
      validatedMsgsQty++;  // increment the count of incoming messages
      calcQtyValidatedMsgs();  // calculate the frequency of validated messages
    } else {
      Serial.print("Checksum mismatch: ");
      Serial.print("Received checksum (");
      Serial.print(RadioControlData.checksum);
      Serial.print(") Calculated checksum (");
      Serial.print(calculateChecksum(RadioControlData));
      /*
      Serial.print(" | Vltg: ");
      Serial.print(RadioControlData.voltage);
      Serial.print(" | E-stp: ");
      Serial.print(RadioControlData.estop);
      Serial.print(" | Cntrl Mode: ");
      Serial.print(RadioControlData.control_mode);
      */
      Serial.print(") | Cntr: ");
      Serial.println(RadioControlData.counter);
    }
  }
}


void sendLoRaMsg() {
  TractorData.counter++; // increment the counter each time the function is called
  TractorData.checksum = calculateChecksum(TractorData);
  LoRa.beginPacket();
  LoRa.write((byte*)&TractorData, sizeof(TractorDataStruct));
  LoRa.endPacket();
}

void InitLoRa() {
  SPI.begin(SCK_PIN, MISO, MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed. Check your connections!");
    while (1);
  } else {
    Serial.println("LoRa initialization successful");
  }
  LoRa.setSyncWord(LORA_SYNC_WORD);
}

void printInfoMsg() {
  if (currentMillis - lastPrintMillis >= 10000) {  // 10 seconds
    float avgRSSI = 0;
    if (messageCount > 0) {
      avgRSSI = (float)accumulatedRSSI / messageCount;
    }
    accumulatedRSSI = 0;
    messageCount = 0;
    Serial.print("Average RSSI: ");
    Serial.print(avgRSSI);
    Serial.print(" dBm | Rx Hz: ");
    Serial.print(validatedMsgsHz);
    Serial.print(" | Tx counter: ");
    Serial.println(TractorData.counter);    
    lastPrintMillis = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  InitLoRa();
}

void loop() {
  currentMillis = millis();
  getTractorData(); // get the data
 // sendLoRaMsg(); // send the data via LoRa  
  getLoRaMsg();
  printInfoMsg();
 // delay(100); // Adjust delay as needed to achieve the desired transmission rate
}
