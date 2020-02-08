/*
   LUCA CROTTI @ 2019
   PROGETTO SEGANTINI SMART PARK
   FILE:   ILARIA_PCB
   MODIFICATO DA NORMAN MULINACCI @ 2020

      HW: 
      - PCB: ILARIA_PCBV4
      - MONITOR OLED 1.3" 128x64 SSD1306 interfaccia I2C
      - ESP32 ESP32-WROVER-B
                         ____________________
                        |                     |
               3V3--- o-|-3V3             GND-|-o
                      o-|-EN              D23-|-o --- MOSI SDCARD
      MICS4514-COx -- o-|-VP(D36)         D22-|-o --- SCL OLED+BME+MICS6814
      MICS4514-NOx -- o-|-VN(D39)     (D1)TX0-|-o
      MICS4514-pre -- o-|-D34         (D3)RX0-|-o
    MQ-7(COx)A0 ----- o-|-D35             D21-|-o --- SDA OLED+BME+MICS6814
    MQ-7(COx)D0 ----- o-|-D32             GND-|-o
          dip-1 ----- o-|-D33             D19-|-o --- MISO SDCARD u8
          dip-2 ----- o-|-D25             D18-|-o --- SCK SDCARD
          dip-3 ----- o-|-D26             D5 -|-o --- CS SDCARD
          dip-4 ----- o-|-D27             D17-|-o  -x-x- non usare! protetto
        RX SENS PM--- o-|-D14             D16-|-o  -x-x- non usare! protetto
        TX SENS PM--- o-|-D12             D4 -|-o --- PIN ATTIVAZ. MOSFET
                      o-|-GND             D0 -|-o
                      o-|-D13             D2 -|-o
                      o-|-SD2  ---xxx     D15-|-o  -x-x- non usare! protetto
                      o-|-SD3  ---xxx---  SD1-|-o  -x-x- non usare! protetto
                      o-|-CMD  ---xxx---  SD0-|-o  -x-x- non usare! protetto
       VCC-SD+OLED--- o-|-5Vin    xxx---  CLK-|-o  -x-x- non usare! protetto
                        |                     |
                        |      _______        |
                        |     |       |       |
                        |     |       |       |
                        |_____|       |_______|
                              |_______|

    VERSIONE 21: 
      - aggiunta schermata di MAC ADDRESS alla fine del LOOP
      - spedizione su nuovo sito Segantini (accordi con Carlo per la parte
        di protocollo)
        
    VERSIONE 22: 
      - aggiunta del sensore MICS-6814 grove
      - il segnale del CO è relativo ora a questo nuovo sensore. 
      - aggiunta schermata 2 dove compaiono anche gli altri valori letti dal sensore
      
    VERSIONE 23: 
      - test con diverse librerie, ottimizzato l'ordine delle chiamate
      - corretto richiamo libreria NTPClient (serve libreria specifica, modificata in riga 48)
      - modificata versione per tracciabilità

    VERSIONE 24MOD (Norman Mulinacci 2020):
      - compilazione con versione antecedente di ESP32 (1.0.0) -versioni successive crashano-
      - aggiunto delay di preriscaldamento dei sensori all'avvio
      - modificata modalità di LOG su seriale e scheda SD (per maggiore conformità col CSV)
      - aggiunto invio dati del sensore MICS6814 al server
      - aggiunta distinzione tra centralina fissa o mobile (tramite variabile booleana)
      - risolto bug di misura COppm con MQ-7 (corretto pin35 e variabile di upload su server)
      - modificata versione per tracciabilità
     VERSIONE 25MOD (Lorenzo Romeo 2020):
      -Risolto il problema della compilazione con le nuove versioni
      -Modificato il nome della funzione getFormattedDate() in getFormattedTime(), per rendere
       compatibile l'ultima versione della centralina
*/
#define VERSION "25MOD"

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>

// librerie per gestione RETE WIFI
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <Adafruit_Sensor.h>
#include <MutichannelGasSensor.h>
#include <Arduino.h>

#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>

// librerie per gestione display OLED
#include <U8g2lib.h>

// strutture e istanze BME280
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADD 0x76
Adafruit_BME280 bme; // I2C///////Adafruit_BME280 bme(I2C_SDA, I2C_SCL);

//strutture e istanze per BME680
//#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme680; // I2C

// Istanza monitor OLED 1.3" controller SH1106... monitor 0.96" invece con SSD1306
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);   // ESP32 Thing, HW I2C with pin remapping
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);   // ESP32 Thing, HW I2C with pin remapping

// seriale per SDS021 o per PMS5003
HardwareSerial PMSerial(2);  // seriale1  (UART0=0; UART1=1; UART2=2)

//§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
// definizioni per deep sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1500     /* Time ESP32 will go to sleep (in seconds) 25 minutes = 1500 secs */
RTC_DATA_ATTR int bootCount = 0;
//§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§

File root;
WiFiClient client;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

//++++++++++++ CREDENZIALI PER SITO WEB   +++++++++++++++++++++
//sito web: https://api.milanosmartpark.net
const char* server = "api.milanosmartpark.net";    
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

String command;
String ssid="";
String pwd="";
String codice="";
String splash="";
float hum, temp, pre, alt, VOC;
float PM1, PM10, PM25;
int NOx;
int COx;
//int luce;
//int UVlight;
uint8_t buffer;


int tick = 0;
int logtick = 0;


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
int len = 0;
int value;
int pm10_serial = 0;
int pm25_serial = 0;
int checksum_is;
int checksum_ok = 0;
int oterror = 1;

// per sensore MICS-6814 multichannel
float c1,c2,c3,c4,c5,c6,c7,c8,c9;
float MICS6814_NH3   =0.0;
float MICS6814_COB   =0.0;
float MICS6814_NO2   =0.0;
float MICS6814_C3H8  =0.0;
float MICS6814_C4H10 =0.0;
float MICS6814_CH4   =0.0;
float MICS6814_H2    =0.0;
float MICS6814_C2H5OH=0.0;


//// gestione dei DIP-SWITCH per settare indirizzi di trasmissione
//int codice = 0;
//int pin_1=0;
//int pin_2=0;
//int pin_3=0;
//int pin_4=0;

// variabili di stato
bool SD_ok = false;
bool rete_ok = false;
bool connesso_ok = false;
bool dataora_ok = false;
bool invio_ok = false;


// variabili per gestione orario da server UDP
String formattedDate;
String dayStamp;
String timeStamp;
String logvalue;

unsigned long check_wifi = 30000;
unsigned long timeout_connect = 0;

int FASE = 0;

// sensore CO e calcolo PPM
float VoltLev = 0.0;
const float Res1 = 5000.0;
const float rangeV = 4096;
float Rs = 0.0;
float ratio = 0.0;
float LogCOppm = 0.0;
float COppm = 0.0;
String MACA="";

//configurazione sensori e funzionalità
bool MQ7_run = true;
bool MICS6814_run = true;
bool MICS4514_run = false;
bool PMS_run = true;
bool SDS_run = false;
bool BME280_run = false;
bool BME680_run = true;
//bool light_run = false;
//bool UV_run = false;

bool mobile_unit = false; //variabile per distinguere tra centralina mobile (true) o fissa (false)


byte mac[6];

//++++++++++++++++++++++++++++funzionalità di DEBUG+++++++++++++++++++++
bool DEBBUG = false;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//*****************************************************************
//************** F U N Z I O N I   A U S I L I A R I E ************
//*****************************************************************

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
unsigned char tool_icon16x16[] = {
  0b00000000, 0b00000000, //
  0b01100000, 0b00000000, //          ##
  0b11100000, 0b00000000, //         ###
  0b11000000, 0b00000001, //        ###
  0b11000000, 0b00000001, //        ###
  0b11100110, 0b00000001, //        ####  ##
  0b11111110, 0b00000011, //       #########
  0b11111100, 0b00000111, //      #########
  0b11111000, 0b00001111, //     #########
  0b11000000, 0b00011111, //    #######
  0b10000000, 0b00111111, //   #######
  0b00000000, 0b01111111, //  #######
  0b00000000, 0b11111110, // #######
  0b00000000, 0b11111100, // ######
  0b00000000, 0b11111000, // #####
  0b00000000, 0b01110000, //  ###
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char speak_icon16x16[] = {
  0b11111100, 0b00111111, //   ############
  0b11111110, 0b01111111, //  ##############
  0b11111111, 0b11111111, // ################
  0b00001111, 0b11110000, // ####        ####
  0b00000111, 0b11100000, // ###          ###
  0b00000111, 0b11100000, // ###          ###
  0b00000111, 0b11100000, // ###          ###
  0b00000111, 0b11100000, // ###          ###
  0b00000111, 0b11100000, // ###          ###
  0b00000111, 0b11100000, // ###          ###
  0b00001111, 0b11110000, // ####        ####
  0b11111110, 0b11110001, // ####   ########
  0b11111100, 0b01111011, //  #### ########
  0b11111000, 0b00111111, //   ###########
  0b00000000, 0b00011110, //    ####
  0b00000000, 0b00001100, //     ##
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
void appendFile(fs::FS &fs, const char * path, String message) {
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


  if(DEBBUG) Serial.println("comando letto: ");
  if(DEBBUG) Serial.println(comm);

  part1 = comm.substring(comm.indexOf("#ssid"),comm.indexOf("#pwd"));
  part2 = comm.substring(comm.indexOf("#pwd"),comm.indexOf("#codice"));
  part3 = comm.substring(comm.indexOf("#codice"),comm.indexOf("#splash"));
  part4 = comm.substring(comm.indexOf("#splash"),comm.indexOf("\n\r"));
  if(DEBBUG) Serial.print("part1: ");Serial.println(part1);
  if(DEBBUG) Serial.print("part2: ");Serial.println(part2);
  if(DEBBUG) Serial.print("part3: ");Serial.println(part3);
  if(DEBBUG) Serial.print("part4: ");Serial.println(part4);
  Serial.println();

  if (part1.startsWith("#ssid",0)) {
    ssid = part1.substring(part1.indexOf("#ssid")+6,part1.length());
    if(DEBBUG) Serial.print("SSID = ");
    if(DEBBUG) Serial.println(ssid);
  } else {Serial.println("comando non riconosciuto");}

  if (part2.startsWith("#pwd",0)) {
    pwd = part2.substring(part2.indexOf("#pwd")+5,part2.length());
    if(DEBBUG) Serial.print("pwd = ");
    if(DEBBUG) Serial.println(pwd);
  } else {Serial.println("comando non riconosciuto");}

  if (part3.startsWith("#codice",0)) {
    codice = part3.substring(part3.indexOf("#codice")+8,part3.length());
    if(DEBBUG) Serial.print("codice = ");
    if(DEBBUG) Serial.println(codice);
  } else {Serial.println("comando non riconosciuto");}

  if (part4.startsWith("#splash",0)) {
    splash = part4.substring(part4.indexOf("#splash")+8,part4.length());
    if(DEBBUG) Serial.print("splash = ");
    if(DEBBUG) Serial.println(splash);
  } else {Serial.println("comando non riconosciuto");}


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

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

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
void print_wakeup_reason(){
//Method to print the reason by which ESP32 has been awaken from sleep
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}//////////////

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool sdcard_init(){
    if (!SD.begin()) {
      Serial.println("ERRORE su inizializzazione SD CARD!!! ");
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

}// fine funzione sdcard_init()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool connessioneWifi(){

    Serial.print("MI CONNETTO A: ");
    Serial.print(ssid); Serial.print(" @ "); Serial.println(pwd);

    u8g2.clearBuffer();u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(0, 20, "Connessione a: ");u8g2.drawStr(5, 40, ssid.c_str());
    u8g2.sendBuffer();

    int ritento = 1;
    int num_volte = 1;

    Serial.print("MI CONNETTOOOOOOOOOOOOO A: ");
    Serial.print(ssid.c_str()); Serial.print("@"); Serial.println(pwd.c_str());

    // -------CONNESSIONE WIFI----------
    WiFi.begin(ssid.c_str()+'\0', pwd.c_str()+'\0');
    delay(500);

    // ciclo di attesa connessione...
    while (WiFi.status() != WL_CONNECTED) {
      delay(5000);
      Serial.print(".");
      u8g2.drawStr((7*num_volte)+(2*ritento), 55, ". ");u8g2.sendBuffer();
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
        //Serial.println("***** impossibile connettersi al wifi - riprovo tra 1 minuto...");
        u8g2.clearBuffer(); u8g2.setFont(u8g_font_helvB10);
        u8g2.drawStr(0, 20, "WiFi NON Connesso");u8g2.drawStr(20, 20, "riprovo tra poco...");
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
      u8g2.clearBuffer();u8g2.setFont(u8g_font_helvB10);
      u8g2.drawStr(0, 20, "WiFi Connesso");u8g2.drawStr(0, 40, "TX OK ! !");
      u8g2.sendBuffer();
      delay(500);
      return true;
    } else {
      Serial.println("");
      Serial.println("WiFi NON Connesso.");
      u8g2.clearBuffer();u8g2.setFont(u8g_font_helvB10);
      u8g2.drawStr(0, 20, "WiFi NON Connesso");u8g2.drawStr(0, 40, "TX KO");
      u8g2.sendBuffer();
      delay(500);
      dataora_ok = false;
      return false;
    }
      
}// fine funzione connessioneWifi


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
String getMacAdr(byte* mc){
    String macAdr ="";
    WiFi.macAddress(mc);
    macAdr+=String(mc[0],HEX);macAdr+=":";
    macAdr+=String(mc[1],HEX);macAdr+=":";
    macAdr+=String(mc[2],HEX);macAdr+=":";
    macAdr+=String(mc[3],HEX);macAdr+=":";
    macAdr+=String(mc[4],HEX);macAdr+=":";
    macAdr+=String(mc[5],HEX);
    macAdr.toUpperCase();
    

 if (DEBBUG){
    Serial.print("MAC: ");
    Serial.print(mc[0],HEX);
    Serial.print(":");
    Serial.print(mc[1],HEX);
    Serial.print(":");
    Serial.print(mc[2],HEX);
    Serial.print(":");
    Serial.print(mc[3],HEX);
    Serial.print(":");
    Serial.print(mc[4],HEX);
    Serial.print(":");
    Serial.println(mc[5],HEX);
    Serial.println(macAdr);
 }
 MACA=macAdr;
 return macAdr;
} //fine getMacAdr



//*****************************************************************
//******************* S E T U P ***********************************
//*****************************************************************
void setup() {
  //inizializzo variabili ++++++++++++++++++++++++++++++++++
  COx = 0;
  COppm = 0;
  temp = 0;
  hum = 0;
  pre = 0;
  alt = 0;
  VOC = 0;
  PM1 = 0;
  PM25 = 0;
  PM10 = 0; 



  // INIZIALIZZO DIP-SWITCH ++++++++++++++++++++++++++++++++++++
//  pinMode(33,INPUT);
//  pinMode(25,INPUT);
//  pinMode(26,INPUT);
//  pinMode(27,INPUT);
//  codice=0;
   
  // INIZIALIZZO SCHERMO OLED E SERIALE++++++++++++++++++++++++++++++++++++
  u8g2.begin();
  Serial.begin(115200);
  delay(1500);// serve per dare tempo alla seriale di attivarsi


  // Inizializzo SD CARD+++++++++++++++++++++++++++++++++++++++++++++++++
  SD_ok = sdcard_init();  
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
        Serial.println(command);
        parseCommand(command);
        rete_ok = true;
     }
    // se è il primo avvio, preparo schermata per LOG su SD
    if (bootCount == 0) {
      String TestoInit = "\nLOG FILE della centralina: ";
      TestoInit += codice;
      TestoInit += "_";
      TestoInit += splash;
      appendFile(SD, "/LOG_ILARIA.txt", TestoInit);
      appendFile(SD, "/LOG_ILARIA.txt", "------------------------------------------------------------------------------------------------------------");
      appendFile(SD, "/LOG_ILARIA.txt", "Temp\tHum\tPre\tCO(ppm)\tCOx\tNOx\tPM25\tPM10\tPM1\tO3\tVOC\tCreated"); //"\t\t\tNH3\tC3H8\t\tC4H10\t\tCH4\t\tH2\tC2H5OH" non inseriti
      appendFile(SD, "/LOG_ILARIA.txt", "------------------------------------------------------------------------------------------------------------\n");
    }
  }// fine IF SD PRESENTE
  else { // qui se file non trovato...
      Serial.println("errore in apertura del file RETE.txt");
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x13_tf);
      u8g2.drawStr(0, 20, "ERRORE APERTURA FILE!");
      u8g2.drawStr(10, 40, "..NO WEB..");
      u8g2.sendBuffer();
      delay(1000);
      SD_ok = false;
      rete_ok = false;
  } // fine IF ramo ELSE+++++++++++++++++++++++++++++++++++++++




  //+++++++GESTIONE SLEEP MODE++++++++++++++++++++++++++++++++++++++++++++
  //- Incrementa boot_number ad ogni reboot+++++++++
  if (!mobile_unit) {
    ++bootCount;
    Serial.println("Boot_number: " + String(bootCount));
    //visualizza la ragione del reboot 
    print_wakeup_reason();
    //  First we configure the wake up source
    //  We set our ESP32 to wake up every 5 seconds
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  } else {
    Serial.println("Centralina configurata come mobile, disattivo lo sleep...");
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // SPLASH INIZIO ++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // HELLO via seriale
  Serial.println();
  Serial.println("WONDERMADE");
  Serial.print("MILANO SMART PARK V");Serial.println(VERSION);
  // SPLASH a schermo
  u8g2.firstPage();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0, 15, "MILANO SMART PARK");
  u8g2.drawStr(0, 30, "SW VER."); //^^^ ver. 24MOD ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  u8g2.drawStr(50, 30, VERSION);
  u8g2.sendBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0, 48, "L.C.'19/N.M.'20");
  u8g2.setFont(u8g2_font_t0_17b_mf);
  u8g2.drawStr(10, 64, splash.c_str());  u8g2.sendBuffer();
  delay(3000);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // +++++++  ATTIVO ALIMENTAZIONE  TUTTI I SENSORI+++++++++++++++
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH); // accendo il MOSFET
    delay(500);
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  Serial.println();


  //inizializzo SENSORE PM++++++++++++++++++++++++++++++++++++++++++
  //definizione: void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false);
  // SERIALE PER PMS: con WROVER NON USARE SERIAL 2 sui pin 16 e 17, va in crash!
  if (PMS_run) {
    Serial.println("sensore PMS5003 abilitato, inizializzo...");
    PMSerial.begin(9600, SERIAL_8N1,14,12); // baud, type, ESP_RX,ESP_TX)
    delay(1500);
  } else {
    Serial.println("sensore PMS5003 disabilitato");
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // messaggio SDS++++++++++++++++++++++++++++++++++++++++++++
  if (SDS_run) {
    Serial.println("sensore SDS abilitato");
  } else {
    Serial.println("sensore SDS disabilitato");
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // messaggio MQ-7++++++++++++++++++++++++++++++++++++++++++++
  if (MQ7_run) {
    Serial.println("sensore MQ-7 abilitato");
  } else {
    Serial.println("sensore MQ-7 disabilitato");
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // inizializzo BME680++++++++++++++++++++++++++++++++++++++++++
  if (!bme680.begin()) {
    Serial.println("sensore BME680 non connesso");
    BME680_run = false;
  } else {
    Serial.println("sensore BME680 connesso, abilito...");
    BME680_run = true;
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // inizializzo BME280++++++++++++++++++++++++++++++++++++++++++++
  if (!bme.begin(BME280_ADD)) {
    Serial.println("sensore BME280 non connesso");
    BME280_run = false;
  } else {
    Serial.println("sensore BME280 connesso, abilito...");
    BME280_run = true;
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  //inizializzo SENSORE MICS6814++++++++++++++++++++++++++++++++++++
  if (MICS6814_run) {
    Serial.println("sensore MICS6814 abilitato, inizializzo...");
    gas.begin(0x04);//the default I2C address of the slave is 0x04
    gas.powerOn();
    delay(500);
  } else {
    Serial.println("sensore MICS6814 disabilitato");
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // MICS4514 - GESTIONE riscaldatore: PRE --> D34++++++++++++++++++++
  if (MICS4514_run) {
    Serial.println("sensore MICS4514 abilitato");
    pinMode(34, OUTPUT);
    digitalWrite(34, HIGH); // accendo il riscaldatore
  } else {
    Serial.println("sensore MICS4514 disabilitato");
  }//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  Serial.println();


  // fase di SCANSIONE DELLA RETE +++++++++++++++++++++++++++++++++++++++++++++++++++

    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(1000);
    Serial.println("scan start");
    
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(0, 20, "SCANSIONE della rete:");
    u8g2.sendBuffer();
    delay(100);

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    delay(300);
    u8g2.setFont(u8g2_font_6x13_tf);u8g2.drawStr(0, 40, "SCANSIONE COMPLETATA:");u8g2.sendBuffer();
    delay(1000);
    if (n == 0) {
        Serial.println("no networks found");
        u8g2.setFont(u8g2_font_6x13_tf);u8g2.drawStr(0, 60, "NESSUNA RETE");u8g2.sendBuffer();
        delay(1000);

    } else {
        Serial.print(n);
        Serial.println(" networks found");
        u8g2.clearBuffer();
        delay(200);
        u8g2.setFont(u8g2_font_6x13_tf);u8g2.drawStr(0, 15, "RETI trovate:");u8g2.sendBuffer();
        delay(100);
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);Serial.print(": ");
            Serial.print(WiFi.SSID(i));Serial.print(" (");Serial.print(WiFi.RSSI(i));Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?"---OPEN---":"***");
            delay(100);
            u8g2.setFont(u8g2_font_6x13_tf);u8g2.drawStr(0, ((i+2)*13), WiFi.SSID(i).c_str());u8g2.sendBuffer();
        }
    }
    Serial.println("");

    // Wait a bit before scanning again
    delay(2000);

    // SPLASH a schermo del testo letto su SD
    u8g2.firstPage();
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(0, 15, "mi connetto a:");
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawUTF8(0, 30, ssid.c_str());
    u8g2.drawStr(5, 48, pwd.c_str());
    u8g2.sendBuffer();



  // fase di CONNESSIONE ALLA RETE, solo se ho dati di connessione++++++++++++++++++++

  if (rete_ok) {
      connesso_ok = connessioneWifi();
  }// fine if se rete presente+++++++++++++++++++++++++++++++++++++++++++++


  //+++++++++++++++++++ lancio connessione a server orario
  if (connesso_ok) {
    dataora_ok = true;
    // Inizializzo il clientNTPClient per sincronizzare data/ora
    timeClient.begin();
    timeClient.setTimeOffset(3600); // Set offset time in seconds to adjust...// GMT +1 = 3600
  } else {
    dataora_ok = false;  // fine IF rete presente
  }//+++++++++++++++++++++++++++++++++++++++++++++++++++++++

  FASE = 5;
  delay(500);



  // PRERISCALDAMENTO SENSORI +++++++++++++++++++++++++++++++++++++++++++
  // MESSAGGIO seriale
  Serial.println();
  Serial.println("ATTENDERE 5 min, preriscaldamento sensori in corso...");
  // MESSAGGIO a schermo
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0, 15, "Preriscaldamento...");
  u8g2.drawStr(5, 48, "ATTENDERE 5 MINUTI");
  u8g2.sendBuffer();
  // ATTESA
  delay(300000);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



  // preparo schermata per LOG su SERIALE+++++++++++++++++++++++++++++++++++++
  Serial.println("\n--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("Temp\tHum\tPre\tCO(ppm)\tCOx\tNOx\tPM25\tPM10\tPM1\tO3\tVOC\tCreated\t\t\tNH3\tC3H8\t\tC4H10\t\tCH4\t\tH2\tC2H5OH");
  Serial.println("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  
}// fine SETUP








//*************************************************************************
//*************************************************************************
//*************************************************************************
void loop() {

  delay(100);

  
  if (DEBBUG) {
    Serial.print("SD_ok: --> "); Serial.println(SD_ok);
    Serial.print("rete_ok: --> "); Serial.println(rete_ok);
    Serial.print("connesso_ok: --> "); Serial.println(connesso_ok);
    Serial.print("dataora_ok: --> "); Serial.println(dataora_ok);
    Serial.print("invio_ok: --> "); Serial.println(invio_ok);
//    Serial.print("stato pin_1 (GPIO33): ");Serial.println(pin_1);
//    Serial.print("stato pin_2 (GPIO25): ");Serial.println(pin_2);
//    Serial.print("stato pin_3 (GPIO26): ");Serial.println(pin_3);
//    Serial.print("stato pin_4 (GPIO27): ");Serial.println(pin_4);
//    Serial.print("stato codice: ");Serial.println(codice);
  }

  // gestione a FASI

  //------------------------------------------------------------------------
  if (FASE == 5) {  //+++++++++++ AGGIORNO DATA e ORA  +++++++++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 5...");
    dayStamp = "na";
    timeStamp = "na";
    
    long start = millis();
    if ((dataora_ok) && (connesso_ok)) {
      // aggiorno data e ora ogni LOOP
      while ((!timeClient.update())&& (millis()-start <= 10000)) {
        if (DEBBUG) Serial.println("...aspetto l'ora aggiornata");
        timeClient.forceUpdate();
        delay(300);
      }
      formattedDate = timeClient.getFormattedTime();// formato: 2018-05-28T16:00:13Z
      if (DEBBUG) Serial.println(formattedDate);

      // formattazione data e ora da stringa ottenuta
      int splitT = formattedDate.indexOf("T");
      dayStamp = formattedDate.substring(0, splitT);//Serial.print("DATE: ");//Serial.println(dayStamp);
      timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1); //Serial.print("HOUR: ");//Serial.println(timeStamp);
      delay(500);
    }// fine if dataora

    FASE = 10;
  }// fine FASE 5   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------





  //------------------------------------------------------------------------
  if (FASE == 10) {  //+++++++++  AGGIORNAMENTO SENSORE CO MQ7  ++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 10...");

    if (MQ7_run) {
      // gestione dato ANALOG da sensore CO MQ7
      COx = analogRead(35);   //Serial.print("Livello CO-MQ7 = "); Serial.println(CO);
      delay(10);

      // calcolo PPM di CO
      VoltLev = COx / rangeV;               // calcolo il livello di tensione letto
      Rs = Res1 * (5 - VoltLev) / VoltLev; // calcolo la resistenza del sensore
      ratio = Rs / Res1;
      float k1 = -1.3;                      // k1 entità che pesa la linearità (minore = meno lineare)
      float k2 = 4.084;                     // k2 fattore moltiplicativo esponenziale
      LogCOppm = (log10(ratio) * (k1)) + k2;
      COppm = pow(10, LogCOppm);
    }

    FASE = 15;

  }// fine FASE 10   //+++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------





  //------------------------------------------------------------------------
  if (FASE == 15) {  //+++++++++  AGGIORNAMENTO SENSORE MICS6814  ++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 15...");

    if (MICS6814_run) {
      // gestione dati sensore su I2C
      c1 = gas.measure_NH3();    if(c1>=0) MICS6814_NH3=c1;   
      c2 = gas.measure_CO();     if(c2>=0) MICS6814_COB=c2;    
      c3 = gas.measure_NO2();    if(c3>=0) MICS6814_NO2=c3;
      c4 = gas.measure_C3H8();   if(c4>=0) MICS6814_C3H8=c4;  
      c5 = gas.measure_C4H10();  if(c5>=0) MICS6814_C4H10=c5;
      c6 = gas.measure_CH4();    if(c6>=0) MICS6814_CH4=c6;
      c7 = gas.measure_H2();     if(c7>=0) MICS6814_H2=c7;
      c8 = gas.measure_C2H5OH(); if(c8>=0) MICS6814_C2H5OH=c8;
    }
    delay (100);
    FASE = 20;

  }// fine FASE 15   //+++++++++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------




  //------------------------------------------------------------------------
  if (FASE == 20) {  //+++++++++  AGGIORNAMENTO SENSORE MICS4514 +++++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 20...");

    if (MICS4514_run) {
      //  // gestione dato da MICS4514
      NOx = analogRead(36);//  Serial.print("Livello NOx = "); Serial.println(NOx);
      COx = analogRead(39);//  Serial.print("Livello COx = "); Serial.println(COx);
      delay(10);
    }
    FASE = 30;
  }// fine FASE 20   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------




  //------------------------------------------------------------------------
  if (FASE == 30) {  //+++++++++  AGGIORNAMENTO SENSORE PM SDS021 ++++++++++
    if (DEBBUG) Serial.println("...sono in fase 30...");
    len = 0;
    pm10_serial = 0;
    pm25_serial = 0;
    checksum_is;
    checksum_ok = 0;
    oterror = 1;
    if (SDS_run) {
      if (DEBBUG) Serial.println(SDS_run);
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
          PM1 = (float)0.0; 
          PM10 = (float)pm10_serial / 10.0; //Serial.print("PM10 = "); Serial.println(PM10);
          PM25 = (float)pm25_serial / 10.0; //Serial.print("PM2.5 = "); Serial.println(PM25);
          len = 0; checksum_ok = 0;
          pm10_serial = 0.0;
          pm25_serial = 0.0;
          checksum_is = 0;
          oterror = 0;
          //buffer=0;
        }// fine IF
      }// fine while

    }// fine IF SDS

    if (PMS_run) {
      FASE = 35;
    } else {
      FASE = 40;
    }

  }// fine FASE 30   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------





  //------------------------------------------------------------------------
  if (FASE == 35) {  //+++++++++  AGGIORNAMENTO SENSORE PMS5003  +++++++++++
    if (DEBBUG) Serial.println("...sono in fase 35...");

    if (readPMSdata(&PMSerial)) {
        // reading data was successful!
        PM1=data.pm10_standard;
        PM25=data.pm25_standard;
        PM10=data.pm100_standard;
        if(DEBBUG){
            Serial.print(data.pm10_standard);Serial.print("\t"); 
            Serial.print(data.pm25_standard);Serial.print("\t");
            Serial.print(data.pm100_standard);Serial.print("\t");
            Serial.print(data.particles_03um);Serial.print("\t");
            Serial.print(data.particles_05um);Serial.print("\t");
            Serial.print(data.particles_10um);Serial.print("\t");
            Serial.print(data.particles_25um);Serial.print("\t");
            Serial.print(data.particles_50um);Serial.print("\t");
            Serial.println(data.particles_100um);
            delay(300);
        }// fine IF DEBBUG
    }//fine IF read...
  
    FASE = 40;

  }// fine FASE 35   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------






  //------------------------------------------------------------------------
  if (FASE == 40) {  //+++++++++  AGGIORNAMENTO SENSORE BME280  ++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 40...");

    if (BME280_run) {
      if (DEBBUG) Serial.println("bme280..");
      // gestione sensore BME280
      temp = bme.readTemperature();
      hum = bme.readHumidity();
      pre = bme.readPressure() / 100.0F;
      alt = bme.readAltitude(SEALEVELPRESSURE_HPA + 5);
    }

    if (BME680_run) {
      FASE = 45;
    } else {
      FASE = 50;
    }
  }// fine FASE 40   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------





  //------------------------------------------------------------------------
  if (FASE == 45) {  //+++++++++  AGGIORNAMENTO SENSORE BME680  ++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 45...");

    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(200, 150); // 320*C for 150 ms
    delay(300);
    if (! bme680.performReading()) {
      Serial.println("Failed to perform reading :(");
      //return;
    }
    
    temp = bme680.temperature;
    hum = bme680.humidity;
    pre = bme680.pressure / 100.0;
    VOC = bme680.gas_resistance / 1000.0;
    
    if(DEBBUG){
        Serial.print("Temperature = "); Serial.print(bme680.temperature); Serial.println(" *C");
        Serial.print("Pressure = "); Serial.print(bme680.pressure / 100.0); Serial.println(" hPa");
        Serial.print("Humidity = "); Serial.print(bme680.humidity); Serial.println(" %");
        Serial.print("Gas = "); Serial.print(bme680.gas_resistance / 1000.0); Serial.println(" KOhms");
        Serial.println();
    }


    FASE = 60;
  }// fine FASE 40   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------





/*
  //------------------------------------------------------------------------
  if (FASE == 50) {  //+++++++++  AGGIORNAMENTO SENSORE luce e UV  +++++++++
    if (DEBBUG) Serial.println("...sono in fase 50...");
    luce = 333;
    UVlight = 444;
    /////////////////
    FASE = 60;
  }// fine FASE 50   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------
*/





  //------------------------------------------------------------------------
  if (FASE == 60) {  //+++++++++++++ LOG SU SERIALE  +++++++++++++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 60...");

  //"Temp\tHum\tPre\tCO(ppm)\tCOx\tNOx\tPM25\tPM10\tPM1\tO3\tVOC\tCreated\t\t\tNH3\tC3H8\t\tC4H10\t\tCH4\t\tH2\tC2H5OH"
    Serial.print(temp);
    Serial.print("\t"); Serial.print(hum);
    Serial.print("\t"); Serial.print(pre);
    Serial.print("\t"); Serial.print(COppm);
    Serial.print("\t"); Serial.print(MICS6814_COB);
    Serial.print("\t"); Serial.print(MICS6814_NO2);
    Serial.print("\t"); Serial.print(PM25);
    Serial.print("\t"); Serial.print(PM10);
    Serial.print("\t"); Serial.print(PM1);
    Serial.print("\t"); Serial.print("N.D.");
    Serial.print("\t"); Serial.print(VOC);
    Serial.print("\t"); Serial.print(dayStamp);
    Serial.print(" "); Serial.print(timeStamp);
    Serial.print("\t"); Serial.print(MICS6814_NH3);
    Serial.print("\t"); Serial.print(MICS6814_C3H8);
    Serial.print("\t"); Serial.print(MICS6814_C4H10);
    Serial.print("\t\t"); Serial.print(MICS6814_CH4);
    Serial.print("\t"); Serial.print(MICS6814_H2);
    Serial.print("\t"); Serial.print(MICS6814_C2H5OH);
    Serial.println();

    FASE = 70;
  }// fine FASE 60   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  if (FASE == 70) {  //+++++++++++++ LOG SU SD CARD  +++++++++++++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 70...");

    if (SD_ok) {
          logvalue = "";
          logvalue += String(temp); logvalue += "\t";
          logvalue += String(hum); logvalue += "\t";
          logvalue += String(pre); logvalue += "\t";
          logvalue += String(COppm); logvalue += "\t";
          logvalue += String(MICS6814_COB); logvalue += "\t";
          logvalue += String(MICS6814_NO2); logvalue += "\t";
          logvalue += String(PM25); logvalue += "\t";
          logvalue += String(PM10); logvalue += "\t";
          logvalue += String(PM1); logvalue += "\t";
          logvalue += String("N.D."); logvalue += "\t";
          logvalue += String(VOC); logvalue += "\t";
          logvalue += String(dayStamp); logvalue += " ";
          logvalue += String(timeStamp); logvalue += "\n";
          //logvalue += String(MICS6814_NH3); logvalue += "\t";
          //logvalue += String(MICS6814_C3H8); logvalue += "\t";
          //logvalue += String(MICS6814_C4H10); logvalue += "\t";
          //logvalue += String(MICS6814_CH4); logvalue += "\t";
          //logvalue += String(MICS6814_H2); logvalue += "\t";
          //logvalue += String(MICS6814_C2H5OH); logvalue += "\t";
    
          File filelog = SD.open("/LOG_ILARIA.txt", FILE_APPEND);
          delay(100);
          if (!filelog) {
            Serial.println("errore su apertura scheda SD...");
            return;
          }
          if (filelog.println(logvalue)) {
            //Serial.println("Message appended");
          } else {
            Serial.println("Append FALLITO");
          }
          filelog.close();
    }// fine IF SD ok
    FASE = 80;
  }// fine FASE 70   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  if (FASE == 80) {  //+++++++++++ AGGIORNO DISPLAY  +++++++++++++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 80...");

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x13_tf);

    // stato del sistema
    u8g2.drawStr(0, 13, "#      #");
    u8g2.drawStr(7, 13, codice.c_str());

    if (dataora_ok) {u8g2.drawXBMP(52, 0, 16, 16, clock_icon16x16);
    } else {u8g2.drawXBMP(52, 0, 16, 16, blank_icon16x16); 
    }

    if (SD_ok) {u8g2.drawXBMP(72, 0, 16, 16, speak_icon16x16);
    } else {u8g2.drawXBMP(72, 0, 16, 16, blank_icon16x16); 
    }

    if (connesso_ok) {u8g2.drawXBMP(112, 0, 16, 16, wifi1_icon16x16);
    } else {u8g2.drawXBMP(112, 0, 16, 16, nocon_icon16x16); 
    }

// pagina 1
    u8g2.drawLine(0, 17, 127, 17);
    u8g2.drawStr(0, 28, "Te:");   u8g2.drawStr(68, 28, "HR:");
    u8g2.drawStr(0, 40, "hPa:");  u8g2.drawStr(68, 40, "VOC:");
    u8g2.drawStr(0, 52, "CO:");   u8g2.drawStr(68, 52, "PM1:");
    u8g2.drawStr(0, 64, "P25:");  u8g2.drawStr(68, 64, "P10:");
    u8g2.setCursor(25, 28);  u8g2.print(temp);    u8g2.setCursor(93, 28);  u8g2.print(hum);
    u8g2.setCursor(25, 40);  u8g2.print(pre);     u8g2.setCursor(93, 40);  u8g2.print(VOC);
    u8g2.setCursor(25, 52);  u8g2.print(COppm);   u8g2.setCursor(93, 52);  u8g2.print(PM1);
    u8g2.setCursor(25, 64);  u8g2.print(PM25);    u8g2.setCursor(93, 64);  u8g2.print(PM10);
    u8g2.sendBuffer();
    delay(3000);

    
// pagina 2
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x13_tf);
    // stato del sistema
    u8g2.drawStr(0, 13, "#      #");
    u8g2.drawStr(7, 13, codice.c_str());

    if (dataora_ok) {u8g2.drawXBMP(52, 0, 16, 16, clock_icon16x16);
    } else {u8g2.drawXBMP(52, 0, 16, 16, blank_icon16x16); 
    }

    if (SD_ok) {u8g2.drawXBMP(72, 0, 16, 16, speak_icon16x16);
    } else {u8g2.drawXBMP(72, 0, 16, 16, blank_icon16x16); 
    }

    if (connesso_ok) {u8g2.drawXBMP(112, 0, 16, 16, wifi1_icon16x16);
    } else {u8g2.drawXBMP(112, 0, 16, 16, nocon_icon16x16); 
    }

    u8g2.drawLine(0, 17, 127, 17);
    u8g2.drawStr(0, 28, "COx:");
    u8g2.drawStr(0, 40, "NOx:");
    u8g2.drawStr(0, 52, "NH3:");
    u8g2.drawStr(0, 64, "H2:");
       
      
    u8g2.setCursor(35, 28);  u8g2.print(MICS6814_COB);    
    u8g2.setCursor(35, 40);  u8g2.print(MICS6814_NO2);     
    u8g2.setCursor(35, 52);  u8g2.print(MICS6814_NH3);   
    u8g2.setCursor(35, 64);  u8g2.print(MICS6814_H2);    
    u8g2.sendBuffer();
    delay(4000);
    FASE = 90;

  }// fine FASE 80   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  if (FASE == 90) {  //+++++++++++ CHECK connessione - RICONNESSIONE  ++++++
    if (DEBBUG) Serial.println("...sono in fase 90...");

    // ciclo di verifica connessione...
    if (WiFi.status() != WL_CONNECTED) {  // dentro se devo ritentare una connessione
      connesso_ok = false;
      dataora_ok = false;
      WiFi.disconnect();
      delay(2000);
      if (DEBBUG) Serial.println("disconnesso e aspetto...");

      // -------RICONNESSIONE WIFI----------
      WiFi.begin(ssid.c_str(), pwd.c_str());
      int ritento = 1;
      int riprovato = 1;
      if (DEBBUG) Serial.println("ri-connesso...");
      delay(3000);
      
      // ciclo di attesa connessione...
      while (WiFi.status() != WL_CONNECTED) {
          delay(3000);
          ritento = ritento + 1;
          if (DEBBUG) Serial.print(ritento);Serial.print("-");
          if (ritento >= 5) {  // qui attendo fino a 3 secondi di avere connessione
              WiFi.disconnect();
              delay(300);
              WiFi.begin(ssid.c_str(), pwd.c_str());
              delay(3000);
              ritento = 1;
              riprovato = riprovato + 1; if (DEBBUG) Serial.print(riprovato);Serial.print("*");
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
    }// fine ramo IF

    // aggiorno lo stato se WIFI connesso
    if (WiFi.status() == WL_CONNECTED) {
        connesso_ok = true;
        FASE = 100;
        timeClient.begin();
        timeClient.setTimeOffset(3600); // Set offset time in seconds to adjust...// GMT +1 = 3600
        dataora_ok = true;
    } else {
        connesso_ok = false;
        dataora_ok = false;
        FASE = 5;
    }

  }// fine FASE 90   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------


  //------------------------------------------------------------------------
  if (FASE == 100) {  //++++++ AGGIORNAMENTO SERVER +++++++++++++++++++++++++
    if (DEBBUG) Serial.println("...sono in fase 100...");

    if (connesso_ok) {
        if (DEBBUG) Serial.println("...dentro IF TICK...");
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
        if (DEBBUG) Serial.print(MICS6814_COB);
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
          postStr += String(COppm);
          postStr += "&cox=";
          postStr += String(MICS6814_COB);
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
    while(client.available()) {
        String line = client.readStringUntil('\r');
        Serial.print(line);
    }
        delay(1000);
        client.stop();

    } // fine if connesso OK

    u8g2.drawXBMP(92, 0, 16, 16, blank_icon16x16);
    u8g2.sendBuffer();

    FASE = 5;

  }// fine FASE 100   //++++++++++++++++++++++++++++++++++++++++++++++
  //------------------------------------------------------------------------

  
  //§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
  delay(900);// ritardo su ciclo di circa 1 secondo
  MACA=getMacAdr(mac);
  delay(400);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(0, 25, "MAC:");
  u8g2.sendBuffer();
  delay(500);
  u8g2.setCursor(5, 45);
  u8g2.print(MACA.c_str());
  u8g2.sendBuffer();

  // ++++++++ CONTROLLO DEEP SLEEP +++++++++++++++
  if (!mobile_unit) {
    Serial.println();
    Serial.println("Going to sleep now");
    // ++++++++  DISATTIVO ALIMENTAZIONE DI TUTTI I SENSORI    +++++++++++++++
    digitalWrite(4, LOW); // accendo il MOSFET
    delay(1000);
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    esp_deep_sleep_start();                       //§§§   GOOD NIGHT ! §§
    Serial.println("This will never be printed");
  } else {
    delay(TIME_TO_SLEEP * 1000); //se è configurata come centralina mobile, attende TIME_TO_SLEEP secondi prima della prossima misura
  }
  

}// fine LOOP
