#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>

#include <Wire.h>  
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>

/* ------------------------------ Config -------------------------------- */

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     125
#endif
#ifndef LORA_SF
  #define LORA_SF     9
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef HELTEC_LORA_V3
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;

#else
  #error "need to provide a 'board' object"
#endif

#define SDA_OLED 17
#define SCL_OLED 18
#define Vext 36

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     21 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

/* ------------------------------ Code -------------------------------- */

// just for the display ...

class MyMesh : public mesh::Mesh {

protected:
  char gps_info[50];
  char sens_info[50];
  char mvmt_info[50];
  int mvmts[128] = {0};
  unsigned int mvmts_idx=0;

  void display_infos() {
      display.clearDisplay();
      display.setCursor(0,0);
      display.println((char *)sens_info + 5);
      display.setCursor(0,10);
      display.println((char *)gps_info + 4);
      display.setCursor(0,20);
      display.println((char *)mvmt_info + 4);

      unsigned int idx = mvmts_idx;
      int d = 0;
      bool u = false; // last point is up
      for (int i = 127; i >= 0 ; i--) {
        if (d <= 0) {
          d = mvmts[idx];
          idx = (idx - 1) % 128;
          u = !u;
        }
        if (d != 0) { // won't draw if there was a 0 in the array
          display.drawPixel(i, u ? 35 : 45, SSD1306_WHITE);
          if (--d == 0) {
            display.drawLine(i, 35, i, 45, SSD1306_WHITE);
          }
        }
      }
      display.display();
  }

  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override {
    if (memcmp(app_data, "SENS", 4) == 0) {
      memcpy(sens_info, app_data, app_data_len);
      sens_info[app_data_len] = 0;
      Serial.print("Received adv from a sensor : ");
      Serial.println((char *)sens_info);
      display_infos();
    } else if (memcmp(app_data, "LOC", 3) == 0) {
      memcpy(gps_info, app_data, app_data_len);
      gps_info[app_data_len] = 0;
      Serial.print("Received loc from a beacon : ");
      Serial.println((char *)gps_info);
      display_infos();
    } else if (memcmp(app_data, "MVMT", 4) == 0) {
      memcpy(mvmt_info, app_data, app_data_len);
      mvmt_info[app_data_len] = 0;
      Serial.print("Received adv from a accel : ");
      Serial.println((char *)mvmt_info);
      int s, m;
      sscanf(mvmt_info, "MVMT %d %d", &s, &m);
      if (m == 1) { // first mvt, store stalled count
        mvmts_idx = (mvmts_idx + 1) % 128;
        mvmts[mvmts_idx] = s;
        mvmts_idx = (mvmts_idx + 1) % 128;
      }
      mvmts[mvmts_idx] = m;
     display_infos();
    }
  }

public:
  MyMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {

  }

};

SPIClass spi;
StdRNG fast_rng;
SimpleMeshTables tables;
SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
MyMesh the_mesh(*new RadioLibWrapper(radio, board), fast_rng, *new VolatileRTCClock(), tables);
unsigned long nextPing;

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

  board.begin();

  Wire.begin(SDA_OLED, SCL_OLED);

  VextON();
  delay(100);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.display();
  
#if defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8);
#else
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8);
#endif
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }
  fast_rng.begin(radio.random(0x7FFFFFFF));
  the_mesh.begin();

  RadioNoiseListener true_rng(radio);
  the_mesh.self_id = mesh::LocalIdentity(&true_rng);  // create new random identity

}

void loop() {
  the_mesh.loop();
}
