//
//  Progetto MILANO SMART PARK - Parco Segantini
//  ILARIA Firmware per ESP32_WROVER_B
//  Versioni originali by Luca Crotti
//  Versioni MOD by THE ILARIA TEAM (Norman Mulinacci, Lorenzo Romeo, Diego Porras)
//
//  Librerie richieste: Pacchetto esp32 per Arduino, NTPClient, Adafruit Unified Sensor, Adafruit BME680 Library, MiCS6814-I2C, U8g2
//

#define VERSION "MOD 26"

#include <FS.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>

// librerie per gestione RETE WIFI
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <Adafruit_Sensor.h>
#include <MiCS6814-I2C.h>
#include <Arduino.h>

//#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>

// librerie per gestione display OLED
#include <U8g2lib.h>

// strutture e istanze BME280
//#define SEALEVELPRESSURE_HPA (1013.25)
//#define BME280_ADD 0x76
//Adafruit_BME280 bme; // I2C///////Adafruit_BME280 bme(I2C_SDA, I2C_SCL);

//struttura per sensore MICS6814
MiCS6814 gas;

//strutture e istanze per BME680
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme680; // I2C

// Istanza monitor OLED 1.3" controller SH1106... monitor 0.96" invece con SSD1306
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);   // ESP32 Thing, HW I2C with pin remapping
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);   // ESP32 Thing, HW I2C with pin remapping

// seriale per SDS021 o per PMS5003
HardwareSerial PMSerial(2);  // seriale1  (UART0=0; UART1=1; UART2=2)

//§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
// definizioni per deep sleep
#define uS_TO_S_FACTOR 1000000 				 	//Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 1200    				  //Time ESP32 will go to sleep (in seconds) 20 minutes = 1200 secs
//definizioni per media
#define AVG_MEASUREMENTS_AMOUNT 10	 	  //Amount of measurements
#define AVG_MEASUREMENTS_DELAY	3000	 	//Amount of delay between measurements (in milliseconds)
#define PREHEAT_TIME 10                 //Preheat time in minutes, 10-30 minutes is recommended
//§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§

RTC_DATA_ATTR int bootCount = 0;        //contatore di riavvii dopo lo sleep

//++++++variabile per distinguere tra centralina mobile (true) o fissa (false)++++++++
bool mobile_unit = true;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

File root;
WiFiClient client;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Define Leap Year for getDateTimeStamps()
#define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ) )

//++++++++++++ CREDENZIALI PER SITO WEB   +++++++++++++++++++++
//sito web: https://api.milanosmartpark.net
const char* server = "api.milanosmartpark.net";
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

String command;
String ssid = "";
String pwd = "";
String codice = "";
String splash = "";
String logpath = "";
float hum = 0.0;
float temp = 0.0;
float pre = 0.0;
float VOC = 0.0;
float PM1 = 0.0;
float PM10 = 0.0;
float PM25 = 0.0;
int NOx = 0;
int COx = 0;
uint8_t buffer;

// per sensore PMS5003
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

// per SDS SENSORE PM
//int len = 0;
//int value;
//int pm10_serial = 0;
//int pm25_serial = 0;
//int checksum_is;
//int checksum_ok = 0;
//int oterror = 1;

// per sensore MICS-6814 multichannel
float c1, c2, c3, c4, c5, c6, c7, c8;
float MICS6814_NH3   = 0.0;
float MICS6814_CO   = 0.0;
float MICS6814_NO2   = 0.0;
float MICS6814_C3H8  = 0.0;
float MICS6814_C4H10 = 0.0;
float MICS6814_CH4   = 0.0;
float MICS6814_H2    = 0.0;
float MICS6814_C2H5OH = 0.0;

//// gestione dei DIP-SWITCH per settare indirizzi di trasmissione
//int codice = 0;
//int pin_1=0;
//int pin_2=0;
//int pin_3=0;
//int pin_4=0;

// variabili di stato
bool SD_ok = false;
bool rete_ok = false;
bool ssid_ok = false;
bool connesso_ok = false;
bool dataora_ok = false;
bool invio_ok = false;

// variabili per gestione orario da server UDP
String formattedDate = "";
String dayStamp = "";
String timeStamp = "";
String logvalue = "";

unsigned long check_wifi = 30000;
unsigned long timeout_connect = 0;

// sensore CO e calcolo PPM
float VoltLev = 0.0;
const float Res1 = 5000.0;
const float rangeV = 4096;
float Rs = 0.0;
float ratio = 0.0;
float LogCOppm = 0.0;
float COppm = 0.0;

String MACA = "";

//configurazione sensori e funzionalità
bool MICS6814_run;
bool BME680_run;
bool PMS_run = true;
bool MQ7_run = false;
//bool MICS4514_run = false;
//bool SDS_run = false;
//bool BME280_run = false;

byte mac[6];

//++++++++++++++++++++++++++++funzionalità di DEBUG+++++++++++++++++++++
bool DEBBUG = false;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++  I C O N E  D I  S I S T E M A  +++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char wifi1_icon16x16[] = {
  0b00000000, 0b00000000, //
  0b11100000, 0b00000111, //      ######
  0b11111000, 0b00011111, //    ##########
  0b11111100, 0b00111111, //   ############
  0b00001110, 0b01110000, //  ###        ###
  0b11100110, 0b01100111, //  ##  ######  ##
  0b11110000, 0b00001111, //     ########
  0b00011000, 0b00011000, //    ##      ##
  0b11000000, 0b00000011, //       ####
  0b11100000, 0b00000111, //      ######
  0b00100000, 0b00000100, //      #    #
  0b10000000, 0b00000001, //        ##
  0b10000000, 0b00000001, //        ##
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char arrow_up_icon16x16[] = {
  0b10000000, 0b00000001, //        ##
  0b11000000, 0b00000011, //       ####
  0b11100000, 0b00000111, //      ######
  0b11110000, 0b00001111, //     ########
  0b01111000, 0b00011110, //    ####  ####
  0b00111100, 0b00111100, //   ####    ####
  0b00011110, 0b01111000, //  ####      ####
  0b00111111, 0b11111100, // ######    ######
  0b00111111, 0b11111100, // ######    ######
  0b00111110, 0b01111100, //  #####    #####
  0b00111000, 0b00011100, //    ###    ###
  0b00111000, 0b00011100, //    ###    ###
  0b00111000, 0b00011100, //    ###    ###
  0b11111000, 0b00011111, //    ##########
  0b11111000, 0b00011111, //    ##########
  0b11110000, 0b00001111, //     ########
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char blank_icon16x16[] = {
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char nocon_icon16x16[] = {
  0b00000000, 0b00000000, //
  0b11100000, 0b00000011, //       #####
  0b11111000, 0b00001111, //     #########
  0b11111100, 0b00011111, //    ###########
  0b00111110, 0b00111110, //   #####   #####
  0b01111110, 0b00111000, //   ###    ######
  0b11111111, 0b01110000, //  ###    ########
  0b11110111, 0b01110001, //  ###   ##### ###
  0b11000111, 0b01110011, //  ###  ####   ###
  0b10000111, 0b01110111, //  ### ####    ###
  0b00001110, 0b00111111, //   ######    ###
  0b00011110, 0b00111110, //   #####    ####
  0b11111100, 0b00011111, //    ###########
  0b11111000, 0b00001111, //     #########
  0b11100000, 0b00000011, //       #####
  0b00000000, 0b00000000, //
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char sd_icon16x16[] = {
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b11111110, 0b01111111, //  ##############
  0b11111111, 0b11111111, // ################
  0b11111111, 0b11000011, // ##    ##########
  0b11111111, 0b11111111, // ################
  0b11111111, 0b11000011, // ##    ##########
  0b11111111, 0b11111111, // ################
  0b11111111, 0b11000011, // ##    ##########
  0b11111111, 0b11111111, // ################
  0b11111111, 0b11000011, // ##    ##########
  0b11111111, 0b11111111, // ################
  0b11111111, 0b01111111, //  ###############
  0b00111111, 0b00000111, //      ###  ######
  0b00011110, 0b00000011, //       ##   ####
  0b00000000, 0b00000000, //
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char clock_icon16x16[] = {
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b11100000, 0b00000011, //       #####
  0b11110000, 0b00000111, //      #######
  0b00011000, 0b00001100, //     ##     ##
  0b00001100, 0b00011000, //    ##       ##
  0b00000110, 0b00110000, //   ##         ##
  0b00000110, 0b00110000, //   ##         ##
  0b11111110, 0b00110000, //   ##    #######
  0b10000110, 0b00110000, //   ##    #    ##
  0b10000110, 0b00110000, //   ##    #    ##
  0b10001100, 0b00011000, //    ##   #   ##
  0b00011000, 0b00001100, //     ##     ##
  0b11110000, 0b00000111, //      #######
  0b11100000, 0b00000011, //       #####
  0b00000000, 0b00000000, //
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//*****************************************************************
//************** F U N Z I O N I   A U S I L I A R I E ************
//*****************************************************************

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void drawScrHead() { //disegna l'header dello schermo con tutte le icone

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);

  // stato del sistema
  u8g2.drawStr(0, 13, "#      #");
  u8g2.drawStr(7, 13, codice.c_str());

  if (dataora_ok) {
    u8g2.drawXBMP(52, 0, 16, 16, clock_icon16x16);
  } else {
    u8g2.drawXBMP(52, 0, 16, 16, blank_icon16x16);
  }
  if (SD_ok) {
    u8g2.drawXBMP(72, 0, 16, 16, sd_icon16x16);
  } else {
    u8g2.drawXBMP(72, 0, 16, 16, blank_icon16x16);
  }
  if (connesso_ok) {
    u8g2.drawXBMP(112, 0, 16, 16, wifi1_icon16x16);
  } else {
    u8g2.drawXBMP(112, 0, 16, 16, nocon_icon16x16);
  }

  u8g2.drawLine(0, 17, 127, 17);

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
String floatToComma(float value) {    //Converts float values in strings with the decimal part separated from the integer part by a comma

  String convert = String(value);
  convert.replace(".", ",");
  return convert;

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void displayMeasures() { //prints data on the U8g2 screen, on four pages

  if (DEBBUG) Serial.println("...aggiorno i dati del display...");

  // pagina 1
  drawScrHead();
  u8g2.setCursor(5, 28); u8g2.print("Temp: " + floatToComma(temp));
  u8g2.setCursor(5, 39); u8g2.print("Hum: " + floatToComma(hum));
  u8g2.setCursor(5, 50); u8g2.print("Pre: " + floatToComma(pre));
  u8g2.setCursor(5, 61); u8g2.print("CO: " + floatToComma(COppm));
  u8g2.sendBuffer();
  delay(5000);

  // pagina 2
  drawScrHead();
  u8g2.setCursor(5, 28); u8g2.print("P25: " + floatToComma(PM25));
  u8g2.setCursor(5, 39); u8g2.print("P10: " + floatToComma(PM10));
  u8g2.setCursor(5, 50); u8g2.print("PM1: " + floatToComma(PM1));
  u8g2.setCursor(5, 61); u8g2.print("VOC: " + floatToComma(VOC));
  u8g2.sendBuffer();
  delay(5000);

  // pagina 3
  drawScrHead();
  u8g2.setCursor(5, 28); u8g2.print("COx: " + floatToComma(MICS6814_CO));
  u8g2.setCursor(5, 39); u8g2.print("NOx: " + floatToComma(MICS6814_NO2));
  u8g2.setCursor(5, 50); u8g2.print("NH3: " + floatToComma(MICS6814_NH3));
  u8g2.setCursor(5, 61); u8g2.print("H2: " + floatToComma(MICS6814_H2));
  u8g2.sendBuffer();
  delay(5000);

  // pagina 4
  drawScrHead();
  u8g2.setCursor(5, 28); u8g2.print("CH4: " + floatToComma(MICS6814_CH4));
  u8g2.setCursor(5, 39); u8g2.print("C3H8: " + floatToComma(MICS6814_C3H8));
  u8g2.setCursor(5, 50); u8g2.print("C4H10: " + floatToComma(MICS6814_C4H10));
  u8g2.setCursor(5, 61); u8g2.print("C2H5OH: " + floatToComma(MICS6814_C2H5OH));
  u8g2.sendBuffer();
  delay(5000);

}// end of displayMeasures()
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void appendFile(fs::FS &fs, String path, String message) {

  if (DEBBUG) Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    if (DEBBUG) Serial.println("Failed to open file for appending");
    return;
  }
  if (file.println(message)) {
    if (DEBBUG) Serial.println("Message appended");
  } else {
    if (DEBBUG) Serial.println("Append failed");
  }
  file.close();

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void readFile(fs::FS &fs, const char * path) {

  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void parseCommand(String comm) {

  String part1;
  String part2;
  String part3;
  String part4;
  String part5;


  if (DEBBUG) Serial.println("comando letto: ");
  if (DEBBUG) Serial.println(comm);

  part1 = comm.substring(comm.indexOf("#ssid"), comm.indexOf("#pwd"));
  part2 = comm.substring(comm.indexOf("#pwd"), comm.indexOf("#codice"));
  part3 = comm.substring(comm.indexOf("#codice"), comm.indexOf("#splash"));
  part4 = comm.substring(comm.indexOf("#splash"), comm.indexOf("\n\r"));
  if (DEBBUG) Serial.print("part1: "); Serial.println(part1);
  if (DEBBUG) Serial.print("part2: "); Serial.println(part2);
  if (DEBBUG) Serial.print("part3: "); Serial.println(part3);
  if (DEBBUG) Serial.print("part4: "); Serial.println(part4);
  Serial.println();

  if (part1.startsWith("#ssid", 0)) {
    ssid = part1.substring(part1.indexOf("#ssid") + 6, part1.length());
    if (DEBBUG) Serial.print("SSID = ");
    if (DEBBUG) Serial.println(ssid);
  } else {
    Serial.println("comando non riconosciuto");
  }

  if (part2.startsWith("#pwd", 0)) {
    pwd = part2.substring(part2.indexOf("#pwd") + 5, part2.length());
    if (DEBBUG) Serial.print("pwd = ");
    if (DEBBUG) Serial.println(pwd);
  } else {
    Serial.println("comando non riconosciuto");
  }

  if (part3.startsWith("#codice", 0)) {
    codice = part3.substring(part3.indexOf("#codice") + 8, part3.length());
    if (DEBBUG) Serial.print("codice = ");
    if (DEBBUG) Serial.println(codice);
  } else {
    Serial.println("comando non riconosciuto");
  }

  if (part4.startsWith("#splash", 0)) {
    splash = part4.substring(part4.indexOf("#splash") + 8, part4.length());
    if (DEBBUG) Serial.print("splash = ");
    if (DEBBUG) Serial.println(splash);
  } else {
    Serial.println("comando non riconosciuto");
  }

}// fine parseCommand
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
boolean readPMSdata(Stream *s) {  //per PMS5003

  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;

}// fine readPMSdata()
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void print_wakeup_reason() {

  //Method to print the reason by which ESP32 has been awaken from sleep
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }

}//////////////
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool initializeSD() {

  if (!SD.begin()) {
    Serial.println("Errore lettore SD CARD!");
    return false;
  } else {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("LETTORE SD CARD inizializzato, CARD non presente - controllare!");
      return false;  // se attivo il return, in caso ci arrivo il codice non va avanti
    }
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("TIPO CARD NON IDENTIFICATO - formattare la SD CARD");
    }
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("Dimensione SD CARD: %lluMB\n", cardSize);
    return true;
  }// fine else

  delay(300);

}// fine funzione initializeSD()
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void scansioneWifi() {
  for (int k = 0; k < 4; k++) {
    Serial.println("scan start");
    drawScrHead();
    u8g2.drawStr(0, 30, "SCANSIONE della rete:");
    u8g2.sendBuffer();
    delay(100);
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    delay(300);
    u8g2.setFont(u8g2_font_6x13_tf); u8g2.drawStr(3, 42, "SCANSIONE COMPLETATA"); u8g2.sendBuffer();
    delay(1000);
    if (n == 0) {
      Serial.println("no networks found");
      Serial.printf("%d retryes left\n", 3 - k);
      u8g2.setFont(u8g2_font_6x13_tf); u8g2.drawStr(8, 55, "NESSUNA RETE!"); u8g2.sendBuffer();
      if (k < 3) {
        Serial.println("Trying again...");
        drawScrHead();
        u8g2.drawStr(35, 45, "RIPROVO...");
        u8g2.sendBuffer();
        delay(2000);
        continue;
      }
      delay(1000);
    } else {
      Serial.print(n);
      Serial.println(" networks found");
      drawScrHead();
      delay(200);
      drawScrHead();
      u8g2.setCursor(5, 35); u8g2.print("RETI TROVATE:"); u8g2.setCursor(95, 35); u8g2.print(String(n));
      u8g2.sendBuffer();
      delay(100);
      for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1); Serial.print(": ");
        Serial.print(WiFi.SSID(i)); Serial.print(" ("); Serial.print(WiFi.RSSI(i)); Serial.print(")");
        Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "---OPEN---" : "***");
        if (WiFi.SSID(i) == ssid) {
          ssid_ok = true;
        }
        delay(100);
      }
      Serial.println();
      break;
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool connessioneWifi() {

  Serial.print("MI CONNETTO A: ");
  Serial.print(ssid); Serial.print(" @ "); Serial.println(pwd);

  drawScrHead();
  u8g2.drawStr(5, 30, "Connessione a: "); u8g2.drawStr(8, 42, ssid.c_str());
  u8g2.sendBuffer();

  int ritento = 1;
  int num_volte = 1;

  //Serial.print("MI CONNETTOOOOOOOOOOOOO A: ");
  //Serial.print(ssid.c_str()); Serial.print("@"); Serial.println(pwd.c_str());

  // -------CONNESSIONE WIFI----------
  WiFi.begin(ssid.c_str() + '\0', pwd.c_str() + '\0');
  delay(500);

  // ciclo di attesa connessione...
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.print(".");
    u8g2.drawStr((7 * num_volte) + (2 * ritento), 54, ". "); u8g2.sendBuffer();
    ritento = ritento + 1;

    if (ritento >= 5) {  // qui attendo fino a 2 secondi di avere connessione
      WiFi.disconnect();
      delay(2000);
      WiFi.begin(ssid.c_str(), pwd.c_str());
      ritento = 1;
      num_volte = num_volte + 1;
      delay(3000);
    }// fine if ritenta

    if (num_volte >= 5) {
      Serial.println("***** impossibile connettersi al wifi - riprovo tra 1 minuto...");
      drawScrHead();
      u8g2.drawStr(5, 30, "WiFi NON Connesso"); u8g2.drawStr(5, 42, "riprovo tra poco...");
      u8g2.sendBuffer();
      ritento = 1;
      num_volte = 1;
      break;
    }// fine IF TIMEOUT ARRIVATO

  }// fine WHILE esco da loop se wifi connesso... o per timeout

  // aggiorno lo stato se WIFI connesso
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi CONNESSO....");
    drawScrHead();
    u8g2.drawStr(5, 30, "WiFi Connesso"); u8g2.drawStr(8, 42, "TX OK !!");
    u8g2.sendBuffer();
    delay(500);
    return true;
  } else {
    Serial.println("");
    Serial.println("WiFi NON Connesso.");
    drawScrHead();
    u8g2.drawStr(5, 30, "WiFi NON Connesso"); u8g2.drawStr(8, 42, "TX KO :(");
    u8g2.sendBuffer();
    delay(500);
    dataora_ok = false;
    return false;
  }

}// fine funzione connessioneWifi
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
String getMacAdr(byte* mc) {

  String macAdr = "";
  WiFi.macAddress(mc);
  macAdr += String(mc[0], HEX); macAdr += ":";
  macAdr += String(mc[1], HEX); macAdr += ":";
  macAdr += String(mc[2], HEX); macAdr += ":";
  macAdr += String(mc[3], HEX); macAdr += ":";
  macAdr += String(mc[4], HEX); macAdr += ":";
  macAdr += String(mc[5], HEX);
  macAdr.toUpperCase();


  if (DEBBUG) {
    Serial.print("MAC: ");
    Serial.print(mc[0], HEX);
    Serial.print(":");
    Serial.print(mc[1], HEX);
    Serial.print(":");
    Serial.print(mc[2], HEX);
    Serial.print(":");
    Serial.print(mc[3], HEX);
    Serial.print(":");
    Serial.print(mc[4], HEX);
    Serial.print(":");
    Serial.println(mc[5], HEX);
    Serial.println(macAdr);
  }
  MACA = macAdr;
  return macAdr;

} //fine getMacAdr
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initDateTime() {

  if (connesso_ok) {
    // Inizializzo il clientNTPClient per sincronizzare data/ora
    if (DEBBUG) Serial.println("...inizializzo il server dell'ora...");
    timeClient.begin();
    timeClient.setTimeOffset(3600); // Set offset time in seconds to adjust...// GMT +1 = 3600
    dataora_ok = true;
    upDateTime();
  } else {
    Serial.println("Errore nell'inizializzazione del server orario, connessione assente!");
    dataora_ok = false;
  }
  delay(500);

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void upDateTime() {

  dayStamp = "";
  timeStamp = "";
  long start = millis();
  if (connesso_ok && dataora_ok) {
    if (DEBBUG) Serial.println("...aggiorno data e ora...");
    // aggiorno data e ora ogni LOOP
    while ((!timeClient.update()) && (millis() - start <= 10000)) {
      if (DEBBUG) Serial.println("...aspetto l'ora aggiornata...");
      timeClient.forceUpdate();
      delay(300);
    }
    getDateTimeStamps();
    delay(500);
  } else {
    Serial.println("Errore nell'aggiornamento di data e ora!");
  }

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void getDateTimeStamps() {    // Made from original getFormattedDate() from previous NTPClient package

  unsigned long rawTime = timeClient.getEpochTime() / 86400L;  // in days
  unsigned long days = 0, year = 1970;
  uint8_t month;
  static const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  //next part performs year, month and day calculations from Epoch Time
  while ((days += (LEAP_YEAR(year) ? 366 : 365)) <= rawTime)
    year++;
  rawTime -= days - (LEAP_YEAR(year) ? 366 : 365); // now it is days in this year, starting at 0
  days = 0;
  for (month = 0; month < 12; month++) {
    uint8_t monthLength;
    if (month == 1) { // february
      monthLength = LEAP_YEAR(year) ? 29 : 28;
    } else {
      monthLength = monthDays[month];
    }
    if (rawTime < monthLength) break;
    rawTime -= monthLength;
  }
  String monthStr = ++month < 10 ? "0" + String(month) : String(month); // jan is month 1
  String dayStr = ++rawTime < 10 ? "0" + String(rawTime) : String(rawTime); // day of month
  //bulding final dayStamp and timeStamp strings
  dayStamp = String(year) + "-" + monthStr + "-" + dayStr;//Serial.print("DATE: ");//Serial.println(dayStamp);
  timeStamp = timeClient.getFormattedTime();//Serial.print("HOUR: ");//Serial.println(timeStamp);

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//*****************************************************************
//******************* S E T U P ***********************************
//*****************************************************************

void setup() {

  //INIZIALIZZO DIP-SWITCH ++++++++++++++++++++++++++++++++++++
  //pinMode(33,INPUT);
  //pinMode(25,INPUT);
  //pinMode(26,INPUT);
  //pinMode(27,INPUT);


  // INIZIALIZZO SCHERMO OLED E SERIALE++++++++++++++++++++++++++++++++++++
  u8g2.begin();
  Serial.begin(115200);
  delay(1500);// serve per dare tempo alla seriale di attivarsi


  // SPLASH INIZIO ++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // HELLO via seriale
  Serial.println();
  Serial.println("THE ILARIA TEAM");
  Serial.print("MILANO SMART PARK VERSIONE "); Serial.println(VERSION);
  // SPLASH a schermo
  u8g2.firstPage();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(5, 15, "MILANO SMART PARK");
  u8g2.drawStr(5, 30, "SW VER."); u8g2.drawStr(50, 30, VERSION); //^^^ versione corrente ^^^
  u8g2.drawStr(5, 45, "THE ILARIA TEAM '20");
  u8g2.sendBuffer();
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // Inizializzo SD CARD+++++++++++++++++++++++++++++++++++++++++++++++++
  SD_ok = initializeSD();
  delay(100);
  if (SD_ok == true) {
    // parsing del file per ricavare SSID e PWD....
    command = "";
    root = SD.open("/RETE.txt");// apri il file con le indicazioni per la rete WIFI
    if (root) {
      SD_ok = true;
      Serial.println("RETE.txt: aperto");
      char c;
      while (root.available()) {      // read from the file until there's nothing else in it:
        command += root.readStringUntil('\n*');
      }// fine WHILE
      root.close();
      if (DEBBUG) Serial.println(command);
      parseCommand(command);
      rete_ok = true;
      u8g2.setFont(u8g2_font_t0_17b_mf);
      u8g2.drawStr(25, 60, splash.c_str()); u8g2.sendBuffer();
      delay(3000);
    } else { //qui se errore nell'aprire il file
      delay(3000);
      Serial.println("Errore in apertura del file RETE.txt!");
      drawScrHead();
      u8g2.drawStr(5, 35, "ERRORE APERTURA FILE!");
      u8g2.drawStr(25, 55, "..NO WEB..");
      u8g2.sendBuffer();
      delay(1000);
      rete_ok = false;
    }
  }// fine IF SD PRESENTE
  else { // qui se sd non presente...
    delay(3000);
    Serial.println("Errore: nessuna scheda SD inserita!");
    drawScrHead();
    u8g2.drawStr(5, 35, "NESSUNA SCHEDA SD!");
    u8g2.drawStr(25, 55, "..NO WEB..");
    u8g2.sendBuffer();
    delay(1000);
    SD_ok = false;
    rete_ok = false;
  }// fine IF ramo ELSE+++++++++++++++++++++++++++++++++++++++


  // fase di SCANSIONE DELLA RETE +++++++++++++++++++++++++++++++++++++++++++++++++++
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int wifipow = WiFi.getTxPower();
  Serial.printf("WiFi transmission power is %d [MIN: -4(-1dBm); MAX: 78(19.5dBm)]\n", wifipow);
  delay(1000);
  if (rete_ok) {
    for (int m = 0; m < 4; m++) {
      scansioneWifi();
      if (rete_ok && ssid_ok) {
        Serial.print(ssid); Serial.println(" trovata!");
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.setCursor(5, 55); u8g2.print(ssid + " OK!");
        u8g2.sendBuffer();
        delay(4000);
        connesso_ok = connessioneWifi(); //lancio la connessione wifi
        initDateTime(); //lancio connessione a server orario e sincronizzo l'ora
        break;
      } else if (!ssid_ok) {
        Serial.print("Errore! "); Serial.print(ssid); Serial.println(" non trovata!");
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.setCursor(5, 55); u8g2.print("NO " + ssid + "!");
        u8g2.sendBuffer();
        delay(4000);
        if (m < 3) {
          Serial.println("Riprovo...");
          drawScrHead();
          u8g2.drawStr(35, 45, "RIPROVO...");
          u8g2.sendBuffer();
          delay(3000);
          continue;
        }
        Serial.println("Nessuna connessione!"); Serial.println();
        drawScrHead();
        u8g2.setCursor(5, 45); u8g2.print("NESSUNA CONNESSIONE!");
        u8g2.sendBuffer();
        delay(3000);
      }
    }
  }//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // controllo presenza del csv di log su SD, se non c'è lo creo e lo intesto +++++++++++++++++++++++++++++++++++++
  if (SD_ok == true) {
    logpath = "/LOG_ILARIA_" + codice + ".csv";
    if (!SD.exists(logpath)) {
      Serial.println("File di log non presente, lo creo...");
      File filecsv = SD.open(logpath, FILE_WRITE);
      if (filecsv) {
        filecsv.close();
        String headertext = "File di log della centralina: " + codice + " | Versione firmware: " + VERSION;
        appendFile(SD, logpath, headertext);
        appendFile(SD, logpath, "Date;Time;Temp;Hum;Pre;CO;COx;NOx;PM25;PM10;PM1;VOC;NH3;C3H8;C4H10;CH4;H2;C2H5OH");
        Serial.println("File di log creato!");
        Serial.println();
      } else {
        Serial.println("Errore nel creare il file di log!");
        Serial.println();
      }
    } else {
      Serial.println("File di log presente!");
      Serial.println();
    }
  }//fine controllo


  //+++++++GESTIONE SLEEP MODE++++++++++++++++++++++++++++++++++++++++++++
  //- Incrementa boot_number ad ogni reboot+++++++++
  if (!mobile_unit) {
    ++bootCount;
    Serial.println("Boot_number: " + String(bootCount));
    //visualizza la ragione del reboot
    print_wakeup_reason();
    //  First we configure the wake up source
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " seconds");
    Serial.println();
  } else {
    Serial.println("Centralina configurata come mobile, disattivo lo sleep...");
    Serial.println();
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  /*
    // +++++++  ATTIVO ALIMENTAZIONE  TUTTI I SENSORI+++++++++++++++
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH); // accendo il MOSFET
    delay(500);
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  */

  // +++++++ INIZIALIZZO TUTTI I SENSORI, SE PRESENTI +++++++++++++++
  // inizializzo BME680++++++++++++++++++++++++++++++++++++++++++
  if (bme680.begin()) {
    Serial.println("Sensore BME680 connesso");
    Serial.println();
    BME680_run = true;
    bme680.setGasHeater(200, 150); // 200*C for 150 ms
  } else {
    Serial.println("Sensore BME680 non connesso");
    Serial.println();
    BME680_run = false;
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //inizializzo SENSORE MICS6814++++++++++++++++++++++++++++++++++++
  if (gas.begin()) { // Connect to sensor using default I2C address (0x04)
    Serial.println("Sensore MICS6814 connesso, inizializzo...");
    MICS6814_run = true;
    // accensione riscaldatore e led
    gas.powerOn();
    gas.ledOn();
    Serial.println("MICS6814: valori delle resistenze di base:");
    Serial.print("OX: "); Serial.print(gas.getBaseResistance(CH_OX));
    Serial.print(" ; RED: "); Serial.print(gas.getBaseResistance(CH_RED));
    Serial.print(" ; NH3: "); Serial.println(gas.getBaseResistance(CH_NH3));
    Serial.println();
  } else {
    Serial.println("Sensore MICS6814 non connesso");
    Serial.println();
    MICS6814_run = false;
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //inizializzo SENSORE PM++++++++++++++++++++++++++++++++++++++++++
  //definizione: void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false);
  // SERIALE PER PMS: con WROVER NON USARE SERIAL 2 sui pin 16 e 17, va in crash!
  if (PMS_run) {
    Serial.println("Sensore PMS5003 abilitato, inizializzo...");
    Serial.println();
    PMSerial.begin(9600, SERIAL_8N1, 14, 12); // baud, type, ESP_RX, ESP_TX
    delay(1500);
  } else {
    Serial.println("Sensore PMS5003 disabilitato");
    Serial.println();
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // messaggio MQ-7++++++++++++++++++++++++++++++++++++++++++++
  if (MQ7_run) {
    Serial.println("Sensore MQ-7 abilitato");
    Serial.println();
  } else {
    Serial.println("Sensore MQ-7 disabilitato");
    Serial.println();
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  /*
    // messaggio SDS++++++++++++++++++++++++++++++++++++++++++++
    if (SDS_run) {
      Serial.println("sensore SDS abilitato");
    } else {
      Serial.println("sensore SDS disabilitato");
    }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  */
  /*
    // inizializzo BME280++++++++++++++++++++++++++++++++++++++++++++
    if (!bme.begin(BME280_ADD)) {
      Serial.println("sensore BME280 non connesso");
      BME280_run = false;
    } else {
      Serial.println("sensore BME280 connesso, abilito...");
      BME280_run = true;
    }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  */
  /*
    // MICS4514 - GESTIONE riscaldatore: PRE --> D34++++++++++++++++++++
    if (MICS4514_run) {
      Serial.println("sensore MICS4514 abilitato");
      pinMode(34, OUTPUT);
      digitalWrite(34, HIGH); // accendo il riscaldatore
    } else {
      Serial.println("sensore MICS4514 disabilitato");
    }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  */

  // PRERISCALDAMENTO SENSORI +++++++++++++++++++++++++++++++++++++++++++
  // MESSAGGIO seriale
  Serial.println("Preriscaldamento sensori in corso, attendere...");
  // ATTESA
  for (int i = 60 * PREHEAT_TIME; i > 0; i--) {
    Serial.print(i / 60);
    Serial.print(":");
    Serial.println(i % 60);
    drawScrHead();
    u8g2.setCursor(5, 35); u8g2.print("Preriscaldamento...");
    u8g2.setCursor(8, 55); u8g2.print("ATTENDI " + String(i / 60) + ":" + String(i % 60) + " MIN.");
    u8g2.sendBuffer();
    delay(1000);
  }
  Serial.println();
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}// fine SETUP



//*************************************************************************
//*************************  L O O P  *************************************
//*************************************************************************
void loop() {

  delay(100);

  if (DEBBUG) {
    Serial.print("codice: --> "); Serial.println(codice);
    Serial.print("SD_ok: --> "); Serial.println(SD_ok);
    Serial.print("rete_ok: --> "); Serial.println(rete_ok);
    Serial.print("connesso_ok: --> "); Serial.println(connesso_ok);
    Serial.print("dataora_ok: --> "); Serial.println(dataora_ok);
    Serial.print("invio_ok: --> "); Serial.println(invio_ok);
    //Serial.print("stato pin_1 (GPIO33): ");Serial.println(pin_1);
    //Serial.print("stato pin_2 (GPIO25): ");Serial.println(pin_2);
    //Serial.print("stato pin_3 (GPIO26): ");Serial.println(pin_3);
    //Serial.print("stato pin_4 (GPIO27): ");Serial.println(pin_4);
  }


  //------------------------------------------------------------------------
  //++++++++++++++++  AGGIORNAMENTO SENSORI  ++++++++++++++++++++++++++++++
  //messaggio seriale
  Serial.println("Sto effettuando le misurazioni...");
  //messaggio a schermo
  drawScrHead();
  u8g2.drawStr(15, 35, "Sto effettuando");
  u8g2.drawStr(15, 55, "le misurazioni...");
  u8g2.sendBuffer();

  //ciclo di campionamento+++++++++++++++++++++++++++++++++++++++++++++++++
  for (int z = 0; z < AVG_MEASUREMENTS_AMOUNT; z++) {
    if (DEBBUG) Serial.printf("Misurazione numero %d\n", z + 1);


    //+++++++++  AGGIORNAMENTO SENSORE CO MQ7  ++++++++++++
    if (MQ7_run) {
      if (DEBBUG) Serial.println("...campiono MQ7...");
      // gestione dato ANALOG da sensore CO MQ7
      COx = analogRead(32);   //Serial.print("Livello CO-MQ7 = "); Serial.println(CO); (32)
      delay(10);

      // calcolo PPM di CO
      VoltLev = COx / rangeV;               // calcolo il livello di tensione letto
      Rs = Res1 * (5 - VoltLev) / VoltLev;  // calcolo la resistenza del sensore
      ratio = Rs / Res1;
      float k1 = -1.3;                      // k1 entità che pesa la linearità (minore = meno lineare)
      float k2 = 4.084;                     // k2 fattore moltiplicativo esponenziale
      LogCOppm = (log10(ratio) * (k1)) + k2;
      COppm += pow(10, LogCOppm);
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++


    //+++++++++  AGGIORNAMENTO SENSORE MICS6814  ++++++++++++
    if (MICS6814_run) {
      if (DEBBUG) Serial.println("...campiono MICS6814...");
      // gestione dati sensore su I2C
      c1 = gas.measureCO();     if (c1 >= 0) MICS6814_CO += c1;
      c2 = gas.measureNO2();    if (c2 >= 0) MICS6814_NO2 += c2;
      c3 = gas.measureNH3();    if (c3 >= 0) MICS6814_NH3 += c3;
      c4 = gas.measureC3H8();   if (c4 >= 0) MICS6814_C3H8 += c4;
      c5 = gas.measureC4H10();  if (c5 >= 0) MICS6814_C4H10 += c5;
      c6 = gas.measureCH4();    if (c6 >= 0) MICS6814_CH4 += c6;
      c7 = gas.measureH2();     if (c7 >= 0) MICS6814_H2 += c7;
      c8 = gas.measureC2H5OH(); if (c8 >= 0) MICS6814_C2H5OH += c8;
    }
    delay (300);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++


    //+++++++++  AGGIORNAMENTO SENSORE PMS5003  +++++++++++
    if (PMS_run) {
      if (DEBBUG) Serial.println("...campiono PMS5003...");
      if (readPMSdata(&PMSerial)) {
        // reading data was successful!
        PM1 += data.pm10_standard;
        PM25 += data.pm25_standard;
        PM10 += data.pm100_standard;
        if (DEBBUG) {
          Serial.print(data.pm10_standard); Serial.print("\t");
          Serial.print(data.pm25_standard); Serial.print("\t");
          Serial.print(data.pm100_standard); Serial.print("\t");
          Serial.print(data.particles_03um); Serial.print("\t");
          Serial.print(data.particles_05um); Serial.print("\t");
          Serial.print(data.particles_10um); Serial.print("\t");
          Serial.print(data.particles_25um); Serial.print("\t");
          Serial.print(data.particles_50um); Serial.print("\t");
          Serial.println(data.particles_100um);
          delay(300);
        }// fine IF DEBBUG
      }//fine IF read...
    }
    //++++++++++++++++++++++++++++++++++++++++++++++


    //+++++++++  AGGIORNAMENTO SENSORE BME680  ++++++++++++
    if (BME680_run) {
      if (DEBBUG) Serial.println("...campiono BME680...");
      // Set up oversampling and filter initialization
      bme680.setTemperatureOversampling(BME680_OS_8X);
      bme680.setHumidityOversampling(BME680_OS_2X);
      bme680.setPressureOversampling(BME680_OS_4X);
      bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme680.setGasHeater(200, 150); // 200*C for 150 ms
      delay(300);
      if (! bme680.performReading()) {
        Serial.println("Failed to perform reading :(");
        //return;
      }
      temp += bme680.temperature;
      hum += bme680.humidity;
      pre += bme680.pressure / 100.0;
      VOC += bme680.gas_resistance / 1000.0;
      if (DEBBUG) {
        Serial.print("Temperature = "); Serial.print(bme680.temperature); Serial.println(" *C");
        Serial.print("Pressure = "); Serial.print(bme680.pressure / 100.0); Serial.println(" hPa");
        Serial.print("Humidity = "); Serial.print(bme680.humidity); Serial.println(" %");
        Serial.print("Gas = "); Serial.print(bme680.gas_resistance / 1000.0); Serial.println(" KOhms");
        Serial.println();
      }
    }
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /*
        //+++++++++  AGGIORNAMENTO SENSORE MICS4514 +++++++++++++++
        if (MICS4514_run) {
          if (DEBBUG) Serial.println("...campiono MICS4514...");
          // gestione dato da MICS4514
          NOx += analogRead(36);//  Serial.print("Livello NOx = "); Serial.println(NOx);
          COx += analogRead(39);//  Serial.print("Livello COx = "); Serial.println(COx);
          delay(10);
        }
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    */
    /*
        //+++++++++  AGGIORNAMENTO SENSORE PM SDS021 ++++++++++
        if (SDS_run) {
          if (DEBBUG) Serial.println("...campiono SDS021...");
          len = 0;
          pm10_serial = 0;
          pm25_serial = 0;
          checksum_is;
          checksum_ok = 0;
          oterror = 1;
          // controllo il sensore PM...
          while ((PMSerial.available() > 0) && (PMSerial.available() >= (10 - len))) {
            buffer = PMSerial.read();
            value = int(buffer);
            switch (len) {
              case (0): if (value != 170) {
                  len = -1;
                }; break;
              case (1): if (value != 192) {
                  len = -1;
                }; break;
              case (2): pm25_serial = value; checksum_is = value; break;
              case (3): pm25_serial += (value << 8); checksum_is += value; break;
              case (4): pm10_serial = value; checksum_is += value; break;
              case (5): pm10_serial += (value << 8); checksum_is += value; break;
              case (6): checksum_is += value; break;
              case (7): checksum_is += value; break;
              case (8): if (value == (checksum_is % 256)) {
                  checksum_ok = 1;
                } else {
                  len = -1;
                }; break;
              case (9): if (value != 171) {
                  len = -1;
                }; break;
            }// fine switch case
            len++;
            if (len == 10 && checksum_ok == 1) {
              PM1 += (float)0.0;
              PM10 += (float)pm10_serial / 10.0; //Serial.print("PM10 = "); Serial.println(PM10);
              PM25 += (float)pm25_serial / 10.0; //Serial.print("PM2.5 = "); Serial.println(PM25);
              len = 0; checksum_ok = 0;
              pm10_serial = 0.0;
              pm25_serial = 0.0;
              checksum_is = 0;
              oterror = 0;
              //buffer=0;
            }// fine IF
          }// fine while

        }// fine IF SDS
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    */
    /*
        //+++++++++  AGGIORNAMENTO SENSORE BME280  +++++++++++++++++++++
        if (BME280_run) {
          if (DEBBUG) Serial.println("...campiono BME280...");
          // gestione sensore BME280
          temp += bme.readTemperature();
          hum += bme.readHumidity();
          pre += bme.readPressure() / 100.0F;
          alt += bme.readAltitude(SEALEVELPRESSURE_HPA + 5);
        }
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    */

    if (z < AVG_MEASUREMENTS_AMOUNT - 1) {
      if (DEBBUG) Serial.printf("Aspetto %d secondi per fare la prossima misurazione\n", AVG_MEASUREMENTS_DELAY / 1000);
      delay(AVG_MEASUREMENTS_DELAY);
    }

  }// fine ciclo di campionamento
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //-----------------------------------------------------------------------------------------------


  //------------------------------------------------------------------------
  //+++++++++  CALCOLO MEDIE VALORI  +++++++++++++++++++++++++++++++++++++++
  if (DEBBUG) Serial.println("...calcolo le medie di tutti i valori...");

  //+++++++++  MEDIA SENSORE CO MQ7  ++++++++++++
  if (MQ7_run) {
    COppm /= AVG_MEASUREMENTS_AMOUNT;
  }
  //+++++++++  MEDIA SENSORE MICS6814  ++++++++++++
  if (MICS6814_run) {
    MICS6814_CO /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_NO2 /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_NH3 /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_C3H8 /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_C4H10 /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_CH4 /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_H2 /= AVG_MEASUREMENTS_AMOUNT;
    MICS6814_C2H5OH /= AVG_MEASUREMENTS_AMOUNT;
  }
  //+++++++++  MEDIA SENSORE BME680  ++++++++++++
  if (BME680_run) {
    temp /= AVG_MEASUREMENTS_AMOUNT;
    hum /= AVG_MEASUREMENTS_AMOUNT;
    pre /= AVG_MEASUREMENTS_AMOUNT;
    VOC /= AVG_MEASUREMENTS_AMOUNT;
  }
  //+++++++++  MEDIA SENSORE PMS5003  ++++++++++++
  if (PMS_run) {
    PM1 /= AVG_MEASUREMENTS_AMOUNT;
    PM25 /= AVG_MEASUREMENTS_AMOUNT;
    PM10 /= AVG_MEASUREMENTS_AMOUNT;
  }
  /*
    //+++++++++  MEDIA SENSORE MICS4514  ++++++++++++
    NOx = ;
    COx = ;
    //+++++++++  MEDIA SENSORE PM SDS021  ++++++++++++
    PM1 = ;
    PM10 = ;
    PM25 = ;
    //+++++++++  MEDIA SENSORE BME280  ++++++++++++
    temp = ;
    hum = ;
    pre = ;
    alt = ;
  */
  //------------------------------------------------------------------------
  // fine calcolo medie


  //------------------------------------------------------------------------
  //+++++++++++ AGGIORNO DATA E ORA ++++++++++++++++++++++
  upDateTime();
  if (DEBBUG) Serial.println(dayStamp);
  if (DEBBUG) Serial.println(timeStamp);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  //+++++++++++++ LOG SU SERIALE  +++++++++++++++++++++++
  if (DEBBUG) Serial.println("...log su seriale...");

  // preparo schermata per LOG su SERIALE +++++++++++++++++++++++++++++++++++++
  Serial.println("\n--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("Date          Time        Temp     Hum      Pre        CO      COx     NOx     PM25    PM10     PM1     VOC       NH3     C3H8     C4H10    CH4       H2     C2H5OH");
  Serial.println("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Serial.print(dayStamp);
  Serial.print("    "); Serial.print(timeStamp);
  Serial.print("    "); Serial.print(floatToComma(temp));
  Serial.print("    "); Serial.print(floatToComma(hum));
  Serial.print("    "); Serial.print(floatToComma(pre));
  Serial.print("    "); Serial.print(floatToComma(COppm));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_CO));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_NO2));
  Serial.print("    "); Serial.print(floatToComma(PM25));
  Serial.print("    "); Serial.print(floatToComma(PM10));
  Serial.print("    "); Serial.print(floatToComma(PM1));
  Serial.print("    "); Serial.print(floatToComma(VOC));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_NH3));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_C3H8));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_C4H10));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_CH4));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_H2));
  Serial.print("    "); Serial.print(floatToComma(MICS6814_C2H5OH));
  Serial.println("\n");
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  //+++++++++++++ LOG SU SD CARD  ++++++++++++++++++++++++++++++++++++++++++
  SD.end(); //ricontrollo presenza effettiva della scheda SD
  delay(100);
  SD_ok = false;
  SD_ok = SD.begin();
  delay(100);

  if (SD_ok) {
    if (DEBBUG) Serial.println("...log su scheda SD...");
    //"Date;Time;Temp;Hum;Pre;CO;COx;NOx;PM25;PM10;PM1;VOC;NH3;C3H8;C4H10;CH4;H2;C2H5OH"
    logvalue = "";
    logvalue += dayStamp; logvalue += ";";
    logvalue += timeStamp; logvalue += ";";
    logvalue += floatToComma(temp); logvalue += ";";
    logvalue += floatToComma(hum); logvalue += ";";
    logvalue += floatToComma(pre); logvalue += ";";
    logvalue += floatToComma(COppm); logvalue += ";";
    logvalue += floatToComma(MICS6814_CO); logvalue += ";";
    logvalue += floatToComma(MICS6814_NO2); logvalue += ";";
    logvalue += floatToComma(PM25); logvalue += ";";
    logvalue += floatToComma(PM10); logvalue += ";";
    logvalue += floatToComma(PM1); logvalue += ";";
    logvalue += floatToComma(VOC); logvalue += ";";
    logvalue += floatToComma(MICS6814_NH3); logvalue += ";";
    logvalue += floatToComma(MICS6814_C3H8); logvalue += ";";
    logvalue += floatToComma(MICS6814_C4H10); logvalue += ";";
    logvalue += floatToComma(MICS6814_CH4); logvalue += ";";
    logvalue += floatToComma(MICS6814_H2); logvalue += ";";
    logvalue += floatToComma(MICS6814_C2H5OH);
    File filelog = SD.open(logpath, FILE_APPEND);
    delay(100);
    if (!filelog) {
      Serial.println("errore su apertura scheda SD...");
      return;
    }
    if (!filelog.println(logvalue)) {
      Serial.println("Append FALLITO");
    }
    filelog.close();
  } else {
    Serial.println("Errore lettore SD CARD!");
    u8g2.drawXBMP(72, 0, 16, 16, blank_icon16x16);
    u8g2.sendBuffer();
  }
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  //+++++++++++ AGGIORNO DISPLAY  ++++++++++++++++++++++++++++++++++++++++++
  displayMeasures();
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  //+++++++++++ CHECK connessione - RICONNESSIONE  +++++++++++++++++++++
  if (WiFi.status() != WL_CONNECTED && rete_ok) {  // dentro se devo ritentare una connessione
    if (DEBBUG) Serial.println("...ritento la connessione...");
    connesso_ok = false;
    dataora_ok = false;
    WiFi.disconnect();
    u8g2.drawXBMP(52, 0, 16, 16, blank_icon16x16);
    u8g2.drawXBMP(112, 0, 16, 16, nocon_icon16x16);
    u8g2.sendBuffer();
    delay(2000);
    if (DEBBUG) Serial.println("disconnesso e aspetto...");

    // -------RICONNESSIONE WIFI----------
    WiFi.begin(ssid.c_str(), pwd.c_str());
    int ritento = 1;
    int riprovato = 1;
    if (DEBBUG) Serial.println("ri-connessione...");
    delay(3000);

    // ciclo di attesa connessione...
    while (WiFi.status() != WL_CONNECTED) {
      delay(3000);
      ritento = ritento + 1;
      if (DEBBUG) Serial.print(ritento); Serial.print("-");
      if (ritento >= 5) {  // qui attendo fino a 3 secondi di avere connessione
        WiFi.disconnect();
        delay(300);
        WiFi.begin(ssid.c_str(), pwd.c_str());
        delay(3000);
        ritento = 1;
        riprovato = riprovato + 1; if (DEBBUG) Serial.print(riprovato); Serial.print("*");
      }// fine if ritenta

      if (riprovato >= 5) {
        if (DEBBUG) Serial.println("***** impossibile connettersi al wifi - riprovo tra 1 minuto...");
        //ritento=1;
        //riprovato=1;
        if (DEBBUG) Serial.println("uso il break..");
        break; //esco dal while...
      }// fine IF TIMEOUT ARRIVATO
    }// fine WHILE esco da loop se wifi connesso... o per timeout
    if (DEBBUG) Serial.println("fuori da while...");

    // aggiorno lo stato se WIFI connesso
    if (WiFi.status() == WL_CONNECTED) {
      connesso_ok = true;
      u8g2.drawXBMP(112, 0, 16, 16, wifi1_icon16x16);
      u8g2.sendBuffer();
      initDateTime();
      if (dataora_ok) {
        u8g2.drawXBMP(52, 0, 16, 16, clock_icon16x16);
        u8g2.sendBuffer();
      }
    } else {
      connesso_ok = false;
      dataora_ok = false;
    }
  }// FINE If not connected
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  //++++++ AGGIORNAMENTO SERVER ++++++++++++++++++++++++++++++++++++++++++++
  if (connesso_ok && dataora_ok) { // dentro se connesso al wifi e data e ora sono ok
    if (DEBBUG) Serial.println("...upload dei dati sul SERVER");
    u8g2.drawXBMP(92, 0, 16, 16, arrow_up_icon16x16);
    u8g2.sendBuffer();
    if (DEBBUG) Serial.println("...disegnata icona...");
    // trasmissione su THINGSPEAK
    if (DEBBUG) Serial.print(temp);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(hum);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(pre);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(VOC);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(MICS6814_CO);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(PM1);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(PM25);
    if (DEBBUG) Serial.print("-");
    if (DEBBUG) Serial.print(PM10);
    if (DEBBUG) Serial.println("---;");


    if (client.connect(server, 80)) {    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      String postStr = "apikey=";
      postStr += codice;
      postStr += "&temp=";
      postStr += String(temp);
      postStr += "&hum=";
      postStr += String(hum);
      postStr += "&pre=";
      postStr += String(pre);
      postStr += "&voc=";
      postStr += String(VOC);
      postStr += "&co=";
      postStr += String(MICS6814_NH3);
      postStr += "&cox=";
      postStr += String(MICS6814_CO);
      postStr += "&nox=";
      postStr += String(MICS6814_NO2);
      postStr += "&pm1=";
      postStr += String(PM1);
      postStr += "&pm25=";
      postStr += String(PM25);
      postStr += "&pm10=";
      postStr += String(PM10);
      postStr += "&mac=";
      postStr += getMacAdr(mac);
      postStr += "&data=";
      postStr += dayStamp;
      postStr += "&ora=";
      postStr += timeStamp;


      if (DEBBUG) Serial.println("...aggiunte tutte le stringhe a POSTSTR...");

      client.print("POST /api/channels/writelog HTTP/1.1\r\n");
      client.print("Host: api.milanosmartpark.net\r\n");
      client.print("Connection: close\r\n");
      client.print("User-Agent: Wondermade\r\n");
      client.print("Content-Type: application/x-www-form-urlencoded\r\n");
      client.print("Content-Length: ");
      client.print(postStr.length());
      client.print("\r\n\r\n");
      client.print(postStr);
      if (DEBBUG) Serial.println("STRINGA DI POST INVIATA..........");
      if (DEBBUG) Serial.println(postStr);
      if (DEBBUG) Serial.println("...fine print dei CLIENT.PRINT...");
      postStr = "";
    } else {
      Serial.println(".............errore........");
      Serial.println(".............non connesso........");
      Serial.println(".............non connesso........");
    }

    // Read all the lines of the reply from server and print them to Serial
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    delay(1000);
    client.stop();

    u8g2.drawXBMP(92, 0, 16, 16, blank_icon16x16);
    u8g2.sendBuffer();

  } // fine if connesso OK
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
  delay(900);// ritardo su ciclo di circa 1 secondo
  MACA = getMacAdr(mac);


  // ++++++++ CONTROLLO DEEP SLEEP +++++++++++++++
  if (!mobile_unit) {
    Serial.println();
    Serial.println("Going to sleep now...");
    // ++++++++  DISATTIVO ALIMENTAZIONE DI TUTTI I SENSORI    +++++++++++++++
    // spengo il sensore MICS6814
    gas.powerOff();
    gas.ledOff();
    // spengo il MOSFET
    //digitalWrite(4, LOW);
    delay(1000);
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    drawScrHead();
    u8g2.drawStr(5, 32, "MAC:");
    u8g2.setCursor(17, 44);
    u8g2.print(MACA.c_str());
    u8g2.drawXBMP(52, 0, 16, 16, blank_icon16x16);
    u8g2.drawXBMP(112, 0, 16, 16, nocon_icon16x16);
    u8g2.setFont(u8g2_font_6x13_tf); u8g2.drawStr(25, 62, "SLEEP ATTIVO");
    u8g2.sendBuffer();
    esp_deep_sleep_start();   //§§§   GOOD NIGHT ! §§
    Serial.println("This will never be printed");
  } else {
    Serial.println("Prossime misurazioni tra 2 minuti");
    for (int w = 0; w < 6; w++) { //stampa a rotazione sul display le misure per 120 secondi
      displayMeasures();
    }
  }

}// fine LOOP
