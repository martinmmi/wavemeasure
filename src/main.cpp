//////////////////////////////////////////////////////////////////////
////////////////// Wavemeasure by Martin Mittrenga ///////////////////
//////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <U8g2lib.h>
#include <LoRa.h>
#include <Pangodream_18650_CL.h>
#include <bitmaps.h>
#include <DFRobot_BMI160.h>



#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

const int csPin = 18;         // LoRa radio chip select
const int resetPin = 23;      // LoRa radio reset
const int irqPin = 26;        // Change for your board; must be a hardware interrupt pin

String rx_adr, tx_adr, incoming, outgoing, rssi;
String name = "Wavemeasure";      // Device Name

char buf_rx[12];
char buf_tx[12];
char buf_bV[5];
char buf_bL[4];
char buf_localAddress[5];
char buf_rxAdr[5];
char buf_txAdr[5];

///////////////////////////////////////////////
////////// CHANGE for each Receiver ///////////

byte localAddress = 0xff;                 // Address of this device   
String string_localAddress = "ff";                                    
byte destination = 0xaa;                  // Destination to send to              
String string_destinationAddress = "aa";          

///////////////////////////////////////////////
///////////////////////////////////////////////

byte msgKey1 = 0x2a;                      // Key of outgoing messages
byte msgKey2 = 0x56;
byte msgCount = 0;                        // Count of outgoing messages
byte byte_rssi;
byte byte_bL;

long lastGetBattery = 0;
long lastSend = 0;
long lastDisplayPrint = 0;

int defaultBrightnessDisplay = 150;   // value from 1 to 255
int bL = 0;
double bV = 0;

bool batteryAttention = LOW;
bool batteryAttentionState = LOW;
bool initBattery = LOW;

DFRobot_BMI160 bmi160;
  
const int8_t i2c_addr = 0x69;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

#define ADC_PIN             35
#define CONV_FACTOR       1.75      //1.7 is fine for the right voltage
#define READS               20
#define batteryWidth        29
#define batteryHeight       15
#define lineWidth            2
#define lineHeight          10
#define LED_PIN_INTERNAL    25

Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);

//////////////////////////////////////////////////////////////////////

void sendMessage(String message) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgKey1);                  // add message KEY
  LoRa.write(msgKey2);                  // add message KEY
  LoRa.write(byte_rssi);
  LoRa.write(byte_bL);
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(message.length());         // add payload length
  LoRa.print(message);                  // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

//////////////////////////////////////////////////////////////////////

void onReceive(int packetSize, String *ptr_rx_adr, String *ptr_tx_adr, String *ptr_incoming, String *ptr_rssi, String *ptr_snr) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgKey1 = LoRa.read();   // incoming msg KEY1
  byte incomingMsgKey2 = LoRa.read();   // incoming msg KEY2
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xff) {
    Serial.println("This Message is not for me.");
    return;                             // skip rest of function
  }

  if ((incomingMsgKey1 != msgKey1 && incomingMsgKey2 != msgKey2) && (recipient == localAddress || recipient == 0xff)) {
    Serial.println("Error: Message key is false.");
    return;                             // skip rest of function
  }

  if ((incomingLength != incoming.length()) && (recipient == localAddress || recipient == 0xff)) {   // check length for error
    Serial.println("Error: Message length does not match length.");
    return;                             // skip rest of function
  }

  *ptr_rx_adr = String(recipient, HEX);
  *ptr_tx_adr = String(sender, HEX);
  *ptr_incoming = incoming;
  *ptr_rssi = String(LoRa.packetRssi());
  *ptr_snr = String(LoRa.packetSnr());

  return;

}

//////////////////////////////////////////////////////////////////////

void printDisplay() {

  sprintf(buf_tx, "%s", outgoing);
  sprintf(buf_rx, "%s", incoming);  
  sprintf(buf_localAddress, "%x", localAddress);         // byte
  sprintf(buf_rxAdr, "%s", string_destinationAddress);   // x = byte
  sprintf(buf_txAdr, "%s", tx_adr);

  if ((millis() - lastGetBattery > 10000) || (initBattery == LOW)) {
    bV = BL.getBatteryVolts();
    bL = BL.getBatteryChargeLevel();
    snprintf(buf_bV, 5, "%f", bV);
    snprintf(buf_bL, 4, "%d", bL);

    byte_bL = bL;

    initBattery = HIGH;
    lastGetBattery = millis();
  }

  u8g2.clearBuffer();					      // clear the internal memory

  //Battery Level Indicator
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.setDrawColor(1);
  u8g2.drawStr(67,12,buf_bL);
  u8g2.drawStr(87,12,"%");

  //Address Indicator
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.setDrawColor(1);
  u8g2.drawXBM(20, 3, lineWidth, lineHeight, line1);
  u8g2.setDrawColor(1);
  u8g2.drawStr(3,12,buf_localAddress);

  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.setDrawColor(0);

  //Battery Indicator
  if ((bL >= 0) && (bL <= 10)) {
    batteryAttention = HIGH;
  }
  if ((bL >= 11) && (bL <= 25)) {
    u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery1);
    batteryAttention = LOW;
  }
  if ((bL >= 26) && (bL <= 50)) {
    u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery2);
    batteryAttention = LOW;
    }
  if ((bL >= 51) && (bL <= 75)) {
    u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery3);
    batteryAttention = LOW;
  }
  if ((bL >= 76) && (bL <= 100)) {
    u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery4);
    batteryAttention = LOW;
  }

  //Battery Attention Indicator
  if ((batteryAttention == HIGH)) {
    batteryAttentionState = !batteryAttentionState;
    if ((batteryAttentionState == HIGH)) {
      u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery0);
    }
    if ((batteryAttentionState == LOW)) {
      u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery1);
    } 
  }

  u8g2.sendBuffer();

  lastDisplayPrint = millis();
}

//////////////////////////////////////////////////////////////////////

void setup() {

  setCpuFrequencyMhz(80);               // Set CPU Frequenz 240, 160, 80, 40, 20, 10 Mhz
  
  Serial.begin(115200);                 // initialize serial
  while (!Serial);

  //init the hardware bmin160  
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("reset false");
    while(1);
  }
  
  //set and init the bmi160 i2c address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("init false");
    while(1);
  }

  Serial.println("");
  Serial.println(name);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setContrast(defaultBrightnessDisplay);  

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin
  LoRa.setTxPower(17);  //2-20 default 17
  LoRa.setSpreadingFactor(7);    //6-12 default 7
  LoRa.setSignalBandwidth(125E3);   //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 default 125E3
  LoRa.setCodingRate4(5);   //5-8 default 5
  LoRa.setPreambleLength(8);    //6-65535 default 8
  LoRa.begin(868E6);  //set Frequenz 915E6 or 868E6

  if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");

  pinMode(LED_PIN_INTERNAL, OUTPUT);

  Serial.println("Outputs init succeeded.");

  u8g2.sendBuffer();

  printDisplay();

}

//////////////////////////////////////////////////////////////////////

void loop() {

  printDisplay();
  
  int i = 0;
  int rslt;
  int16_t accelGyro[6]={0}; 
  
  //get both accel and gyro data from bmi160
  //parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if(rslt == 0){
    for(i=0;i<6;i++){
      if (i<3){
        //the first three are gyro data
        Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
      }else{
        //the following three data are accel data
        Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
      }
    }
    Serial.println();
  }else{
    Serial.println("err");
  }

  delay(2000);

  /*
   * //only read accel data from bmi160
   * int16_t onlyAccel[3]={0};
   * bmi160.getAccelData(onlyAccel);
   */

  /*
   * ////only read gyro data from bmi160
   * int16_t onlyGyro[3]={0};
   * bmi160.getGyroData(onlyGyro);
   */

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////