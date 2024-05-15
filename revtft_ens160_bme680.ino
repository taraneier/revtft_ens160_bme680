// SPDX-FileCopyrightText: 2022 Limor Fried for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#include <Arduino.h>
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_BME280.h>
#include <Adafruit_ST7789.h> 
#include <Fonts/FreeSans12pt7b.h>

// Adafruit_BME280 bme280; // I2C
// bool bmefound = false;
extern Adafruit_TestBed TB;

Adafruit_MAX17048 lipo;
// Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

/*ENS160 - Digital Air Quality Sensor*/
#include <Wire.h>
int ArduinoLED = 13;

#include "ScioSense_ENS160.h"  // ENS160 library
// ScioSense_ENS160      ens160(ENS160_I2CADDR_0);
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

/*BME680 - Bosch Sensor*/
#include "Adafruit_BME680.h"
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

Adafruit_BME680 bme; // I2C


const char *ssid = "Endor";
const char *password = "nubnubnub";

WebServer server(80);


void handleNotFound() {
  // digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
  // digitalWrite(led, 0);
}

void setup() {
  Serial.begin(115200);
  //while (! Serial) delay(10);
  
  delay(100);
  
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1; 
  TB.begin();
  TB.setColor(WHITE);

  display.init(135, 240);           // Init ST7789 240x135
  display.setRotation(3);
  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE); 

  if (!lipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
    
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(lipo.getChipID(), HEX);

  // if (TB.scanI2CBus(0x77)) {
  //   Serial.println("BME280 address");

  //   unsigned status = bme280.begin();  
  //   if (!status) {
  //     Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  //     Serial.print("SensorID was: 0x"); Serial.println(bme280.sensorID(),16);
  //     Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //     Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //     Serial.print("        ID of 0x60 represents a BME 280.\n");
  //     Serial.print("        ID of 0x61 represents a BME 680.\n");
  //     return;
  //   }
  //   Serial.println("BME280 found OK");
  //   bmefound = true;
  // }

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);

  /*ENS160 - Digital Air Quality Sensor*/
  //Switch on LED for init
  pinMode(ArduinoLED, OUTPUT);
  digitalWrite(ArduinoLED, LOW);

  Serial.println("------------------------------------------------------------");
  Serial.println("ENS160 - Digital air quality sensor");
  Serial.println();
  Serial.println("Sensor readout in standard mode");
  Serial.println();
  Serial.println("------------------------------------------------------------");
  delay(1000);
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
  // ESP32 is kinda odd in that secondary ports must be manually
  // assigned their pins with setPins()!
  Serial.println("Setting up Wire1 for ESP32S2");
  Wire1.setPins(SDA1, SCL1);
  #endif
  Wire1.begin();
  


  Serial.print("ENS160...");
  ens160.begin();
  Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available()) {
    // Print ENS160 versions
    Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());
  
    Serial.print("\tStandard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
  }
  /* END ENS160 - Digital Air Quality Sensor*/


  /*BME680 - Bosch Sensor*/
  Serial.println("Adafruit BME680 Test");
   if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  Serial.println("Found BME680 sensor");
  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  server.on("/bme", getReadings);
  server.on("/bme/csv", getReadingsCSV);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

}

uint8_t j = 0;

void loop() {
  server.handleClient();
  Serial.println("**********************");
  /*ENS160 - Digital Air Quality Sensor*/
  if (ens160.available()) {
    ens160.measure(true);
    ens160.measureRaw(true);
  
    Serial.print("AQI: ");Serial.print(ens160.getAQI());Serial.print("\t");
    Serial.print("TVOC: ");Serial.print(ens160.getTVOC());Serial.print("ppb\t");
    Serial.print("eCO2: ");Serial.print(ens160.geteCO2());Serial.print("ppm\t");
    Serial.print("R HP0: ");Serial.print(ens160.getHP0());Serial.print("Ohm\t");
    Serial.print("R HP1: ");Serial.print(ens160.getHP1());Serial.print("Ohm\t");
    Serial.print("R HP2: ");Serial.print(ens160.getHP2());Serial.print("Ohm\t");
    Serial.print("R HP3: ");Serial.print(ens160.getHP3());Serial.println("Ohm");
  }
  /* END ENS160 - Digital Air Quality Sensor*/
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  TB.printI2CBusScan();

  if (j % 2 == 0) {
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(0, 17);
    
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.print("VOC:");
    canvas.print(ens160.getTVOC());
    canvas.print("  AQI:");
    canvas.println(ens160.getAQI());
    canvas.print("CO2:");
    canvas.print(ens160.geteCO2());
    canvas.setTextColor(ST77XX_BLUE); 
    canvas.print("  GAS:");
    canvas.println(bme.gas_resistance / 1000);
    canvas.print("TMP:");
    canvas.print(bme.temperature * 1.8 + 32);
    canvas.print(" HUM:");
    canvas.println(bme.humidity);
    // canvas.print("CO2:");
    // canvas.print(iaqSensor.co2Equivalent);
    // canvas.print("  VOC:");
    // canvas.println(iaqSensor.breathVocEquivalent);
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("Battery: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.print(lipo.cellVoltage(), 1);
    canvas.print(" V  /  ");
    canvas.print(lipo.cellPercent(), 0);
    canvas.println("%");
    canvas.setTextColor(ST77XX_BLUE); 
    canvas.print("IP: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.print(WiFi.localIP());
    // for (uint8_t a=0x01; a<=0x7F; a++) {
    //   if (TB.scanI2CBus(a, 0))  {
    //     canvas.print("0x");
    //     canvas.print(a, HEX);
    //     canvas.print(", ");
    //   }
    // }
    // canvas.println("");
    // canvas.print("Buttons: ");
    // Serial.println(digitalRead(0));
    // Serial.println(digitalRead(1));
    // Serial.println(digitalRead(2));
    // if (!digitalRead(0)) {
    //   canvas.print("D0, ");
    // }
    // if (digitalRead(1)) {
    //   canvas.print("D1, ");
    // }
    // if (digitalRead(2)) {
    //   canvas.print("D2, ");
    // }
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
  }
  
  TB.setColor(TB.Wheel(j++));
  delay(10);
  return;
}


void getReadings() {
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  if (ens160.available()) {
    ens160.measure(true);
    ens160.measureRaw(true);
  }
  
  String out = "";
//  char temp[100];
  out += "{";
  out += "\"aqi:\":";
  out += ens160.getAQI();
  out += ",";
  out += "\"tvoc\":";
  out += ens160.getTVOC();
  out += ",";
  out += "\"eCO2\":";
  out += ens160.geteCO2();
  out += ",";
  out += "\"temperature\":";
  out += bme.temperature;
  out += ",";
  out += "\"humidity\":";
  out += bme.humidity;
  out += ",";
  out += "\"pressure\":";
  out += bme.pressure;
  out += ",";
  out += "\"gas_resistance\":";
  out += bme.gas_resistance;
  out += ",";
  out += "\"batt_v\":";
  out += lipo.cellVoltage();
  out += ",";
  out += "\"batt_p\":";
  out += lipo.cellPercent();
  // out += ",";
  // out += "\"batt_temp\":";
  // out += lipo.getCellTemperature();
  out += "}";
  // updateDisplay();
  server.send(200, "application/json", out);
}


void getReadingsCSV() {
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  if (ens160.available()) {
    ens160.measure(true);
    ens160.measureRaw(true);
  }
  
  String out = "";
  out += "aqi, tvoc, eco2, temp,hum,press,kohm,bv,bp,bt\n";
  out += ens160.getAQI();
  out += ",";
  out += ens160.getTVOC();
  out += ",";
  out += ens160.geteCO2();
  out += ",";
  out += bme.temperature;
  out += ",";
  out += bme.humidity;
  out += ",";
  out += bme.pressure;
  out += ",";
  out += bme.gas_resistance;
  out += ",";
  out += lipo.cellVoltage();
  out += ",";
  out += lipo.cellPercent();
  // out += ","
  // out += lipo.getCellTemperature();
  out += "\n";
  // updateDisplay();


  server.send(200, "text/csv", out);
}
