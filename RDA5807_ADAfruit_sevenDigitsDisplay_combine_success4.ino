//#define DEBUG_MODE

#include <Arduino.h>
#include <EEPROM.h>
#include <AceCommon.h> // incrementMod()
#include <AceSPI.h>
#include <AceSegment.h> // HybridModule

//#include <string> // std::stringを使用するためのヘッダー

#if defined(ARDUINO_ARCH_AVR) || defined(EPOXY_DUINO)
#include <digitalWriteFast.h>
#include <ace_spi/HardSpiFastInterface.h>
#include <ace_spi/SimpleSpiFastInterface.h>
#endif

using ace_common::incrementMod;
using ace_common::incrementModOffset;
using ace_common::TimingStats;
using ace_spi::SimpleSpiInterface;
using ace_spi::HardSpiInterface;
using ace_spi::SimpleSpiFastInterface;
using ace_spi::HardSpiFastInterface;
using ace_segment::LedModule;
using ace_segment::HybridModule;
using ace_segment::kActiveHighPattern;

// Select interface protocol.
#define INTERFACE_TYPE_SIMPLE_SPI 0
#define INTERFACE_TYPE_SIMPLE_SPI_FAST 1
#define INTERFACE_TYPE_HARD_SPI 2
#define INTERFACE_TYPE_HARD_SPI_FAST 3
#define INTERFACE_TYPE_SIMPLE_TMI 4
#define INTERFACE_TYPE_SIMPLE_TMI_FAST 5

#define DEFAULT_VOLUME 6
#define STORE_TIME  150    // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).
#define STORE_TIME2 10000
#define STORE_TIME3 1000

const uint8_t app_id = 43;  // Useful to check the EEPROM content before processing useful data
const int eeprom_address = 0;
//const char* RadioStation = "";


const char station12[] PROGMEM = "bayfm";
const char station13[] PROGMEM = "FM NACK5";
const char station1[] PROGMEM = "TokyoFM";
const char station2[] PROGMEM = "J-wave";
const char station3[] PROGMEM = "NHK-FM";
const char station4[] PROGMEM = "KamakuraFM";
const char station5[] PROGMEM = "FM-Totsuka";
const char station6[] PROGMEM = "FM-Yokohama";
const char station7[] PROGMEM = "interfm";
const char station8[] PROGMEM = "TBSradio";
const char station9[] PROGMEM = " Bunka\n  Housou";
const char station10[] PROGMEM = " Radio\n  Nippon";
const char station11[] PROGMEM = " Nippon\n  Housou";
const char stationDefault[] PROGMEM = "no Data";


long storeTime = millis();
bool digit4state = false;
uint16_t pushcount = 0;
uint16_t count3 = 0;
uint16_t pushcount4 = 0;

//----------------------------------------------------------------------------
// Hardware configuration.
//----------------------------------------------------------------------------

// Configuration for Arduino IDE
#if ! defined(EPOXY_DUINO) && ! defined(AUNITER)
  #define AUNITER_MICRO_CUSTOM_SINGLE
#endif

#if defined(EPOXY_DUINO)
  #define INTERFACE_TYPE INTERFACE_TYPE_HARD_SPI_FAST
  SPIClass& spiInstance = SPI;

#elif defined(AUNITER_MICRO_CUSTOM_SINGLE)
  #define INTERFACE_TYPE INTERFACE_TYPE_HARD_SPI_FAST
  SPIClass& spiInstance = SPI;

#else
  #error Unknown environment
#endif

const uint8_t NUM_DIGITS = 5;
const uint8_t NUM_SEGMENTS = 8;
const uint8_t LATCH_PIN = 10;
const uint8_t DATA_PIN = MOSI;
const uint8_t CLOCK_PIN = SCK;
const uint8_t DIGIT_PINS[NUM_DIGITS] = {5, 4, 3, 2, 6}; // ケタの順番を修正
const uint8_t COLON_PIN = 6; // コロンのピンを追加

const uint8_t FRAMES_PER_SECOND = 60;
const uint8_t NUM_SUBFIELDS = 16;

uint16_t currentFrequency;
uint16_t previousFrequency;
uint16_t previousFrequency2;
uint16_t currentVolume;
uint16_t previousVolume;
uint16_t previousVolume2;

uint8_t rssi = 0;
uint8_t volume = DEFAULT_VOLUME;
uint16_t STC;

#if INTERFACE_TYPE == INTERFACE_TYPE_SIMPLE_SPI
  using SpiInterface = SimpleSpiInterface;
  SpiInterface spiInterface(LATCH_PIN, DATA_PIN, CLOCK_PIN);
#elif INTERFACE_TYPE == INTERFACE_TYPE_SIMPLE_SPI_FAST
  using SpiInterface = SimpleSpiFastInterface<LATCH_PIN, DATA_PIN, CLOCK_PIN>;
  SpiInterface spiInterface;
#elif INTERFACE_TYPE == INTERFACE_TYPE_HARD_SPI
  using SpiInterface = HardSpiInterface<SPIClass>;
  SpiInterface spiInterface(spiInstance, LATCH_PIN);
#elif INTERFACE_TYPE == INTERFACE_TYPE_HARD_SPI_FAST
  using SpiInterface = HardSpiFastInterface<SPIClass, LATCH_PIN>;
  SpiInterface spiInterface(spiInstance);
#else
  #error Unknown INTERFACE_TYPE
#endif

// Common Cathode, with transistors on Group pins
HybridModule<SpiInterface, NUM_DIGITS, NUM_SUBFIELDS> ledModule(
    spiInterface,
    kActiveHighPattern /*segmentOnPattern*/,
    kActiveHighPattern /*digitOnPattern*/,
    FRAMES_PER_SECOND,
    DIGIT_PINS
);

// LED patterns 0~19 and more
const uint8_t PATTERNS[26] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b10111111, // 0.
  0b10000110, // 1.
  0b11011011, // 2.
  0b11001111, // 3.
  0b11100110, // 4.
  0b11101101, // 5.
  0b11111101, // 6.
  0b10000111, // 7.
  0b11111111, // 8.
  0b11101111, // 9.
  0b00000001, // : upper single dot (20)
  0b00000010, // : lower single dot (21)
  0b00000011, // : (22)
  0b00000100, // ' (23)
  0b00000111, // :'(24)
  0b00000000  // blank (25)
};

void setupAceSegment() {
#if INTERFACE_TYPE == INTERFACE_TYPE_HARD_SPI \
    || INTERFACE_TYPE == INTERFACE_TYPE_HARD_SPI_FAST
  spiInstance.begin();
#endif

  spiInterface.begin();
  ledModule.begin();
  pinMode(COLON_PIN, OUTPUT);
  digitalWrite(COLON_PIN, LOW);
}

//----------------------------------------------------------------------------

// loop() state variables
TimingStats stats;
uint8_t digitIndex = 0;
uint8_t brightnessIndex = 0;
uint16_t prevUpdateMillis = 0;

#if ENABLE_SERIAL_DEBUG >= 1
uint16_t prevStatsMillis = 0;
#endif

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <RDA5807.h> // It is a minimal receicer with two push buttons (ATmega328 - Uno, Nano etc)
RDA5807 rx; 

void saveAllReceiverInformation() {
  EEPROM.update(eeprom_address, app_id);
  EEPROM.update(eeprom_address + 1, rx.getVolume());           // stores the current Volume
  EEPROM.update(eeprom_address + 2, currentFrequency >> 8);    // stores the current Frequency HIGH byte for the band
  EEPROM.update(eeprom_address + 3, currentFrequency & 0xFF);  // stores the current Frequency LOW byte for the band

  digit4state = true;
}

void readAllReceiverInformation() {
  rx.setVolume(EEPROM.read(eeprom_address + 1));
  currentFrequency = EEPROM.read(eeprom_address + 2) << 8;
  currentFrequency |= EEPROM.read(eeprom_address + 3);
  previousFrequency = currentFrequency;
  rx.setFrequency(currentFrequency);
}

void saveInformation(){
  if (((currentFrequency = rx.getFrequency()) != previousFrequency2) || (currentVolume != previousVolume2)) {
    if ((millis() - storeTime) > STORE_TIME2) {
      saveAllReceiverInformation();
      storeTime = millis();
      previousFrequency2 = currentFrequency;
      previousVolume2 = currentVolume;
    }
  }
}

void setup() {
  
  delay(500);

  #ifdef DEBUG_MODE
  Serial.begin(9600);
  #endif

  //delay(1000);

  #if ENABLE_SERIAL_DEBUG >= 1
  Serial.begin(9600);
  while (!Serial);
  #endif

  setupAceSegment();

  //Serial.begin(9600);
  pinMode(8, INPUT_PULLUP); // Arduino pin 4 - Seek station down
  pinMode(9, INPUT_PULLUP); // Arduino pin 5 - Seek station up
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  rx.setup(); // Starts the receiver with default parameters
  rx.setBand(RDA_FM_BAND_WORLD);
  rx.setVolume(DEFAULT_VOLUME);
  if (EEPROM.read(eeprom_address) == app_id) {
    readAllReceiverInformation();
  }
  currentFrequency = rx.getFrequency();

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  //display.setTextSize(1);      // Normal 1:1 pixel scale
  //display.setFont(&FreeMono9pt7b);
  //display.setTextColor(SSD1306_WHITE); // Draw white text
  //display.setCursor(0, 0);     // Start at top-left corner
  //display.print("Kent Kimura");
  //display.display();
  //delay(500);
}

void loop() {
  #ifdef DEBUG_MODE
  Serial.print(previousFrequency);
  #endif

  sevenSegmentDisplay();

  OLEDdisplay();

  getButtonState();

  saveInformation();

  //delay(200);
}

void OLEDdisplay(){
  if (( currentFrequency != previousFrequency ) || (currentVolume != previousVolume)){
    currentFrequency = rx.getFrequency();
    currentVolume = rx.getVolume();
    previousFrequency = currentFrequency;
    previousVolume = currentVolume;

    char radioStationBuffer[20]; // 一時バッファ
    getRadioStation(currentFrequency, radioStationBuffer);

    display.clearDisplay();
    display.setFont(&FreeMono9pt7b);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 10);    // Start at top-left corner
    display.print(F("FM "));
    display.setCursor(34, 10);
    display.print(rx.getFrequency()/100.0);
    display.setCursor(95, 10);
    display.print(F("MHz"));
    display.setCursor(0, 24);    // Start at top-left corner
    display.print(radioStationBuffer);
    display.setCursor(0, 57);
    display.print(F("Volume"));
    display.setCursor(70, 57);
    display.print(rx.getVolume());
    display.display();
  }
}

void showStatus(){
  //意図的に中はカラ。
}

void getButtonState(){
  if (digitalRead(8) == LOW){
    if ((millis() - storeTime) > STORE_TIME){
      storeTime = millis();
      rx.seek(RDA_SEEK_WRAP,RDA_SEEK_DOWN,showStatus);
      currentFrequency = rx.getFrequency();
    }
  }
  if (digitalRead(9) == LOW){
    if ((millis() - storeTime) > STORE_TIME){
      storeTime = millis();
      rx.seek(RDA_SEEK_WRAP,RDA_SEEK_UP,showStatus);
      currentFrequency = rx.getFrequency();
    }
  }
  if (digitalRead(A0) == LOW){
    if ((millis() - storeTime) > STORE_TIME){
      storeTime = millis();
      rx.setVolumeDown();
      currentVolume = rx.getVolume();
    }
  }
  if (digitalRead(A1) == LOW){
    if ((millis() - storeTime) > STORE_TIME){
      storeTime = millis();
      rx.setVolumeUp();
      currentVolume = rx.getVolume();
    }
  }
  if (digitalRead(A2) == LOW){
    if ((millis() - storeTime) > STORE_TIME){
      storeTime = millis();
      rx.setFrequencyDown();
      currentFrequency = rx.getFrequency();
    }
  }
  if (digitalRead(A3) == LOW){
    if ((millis() - storeTime) > STORE_TIME){
      storeTime = millis();
      rx.setFrequencyUp();
      currentFrequency = rx.getFrequency();
    }
  }
}

void sevenSegmentDisplay() {
  currentVolume = rx.getVolume();
  uint8_t digits[NUM_DIGITS];
  
  digits[0] = (currentFrequency / 1000) % 10;
  digits[1] = (currentFrequency / 100) % 10 + 10;
  digits[2] = (currentFrequency / 10) % 10;
  digits[3] = currentVolume % 10;
  if (currentVolume > 9) digits[4] = 23;
  else digits[4] = 25;

  if (digit4state == true){
    if (currentVolume > 9){
      digits[4] = 24;
    }
    else digits[4] = 22;
    if ((millis() - storeTime) > STORE_TIME3) {
    storeTime = millis();
    digit4state = false;
    }
  }

  if (digit4state == false){
    if (currentVolume > 9){
      digits[4] = 23;
    }
    else digits[4] = 25;
  }
  
  for (uint8_t i = 0; i < NUM_DIGITS; ++i) {
    uint8_t pattern = PATTERNS[digits[i]];
    ledModule.setPatternAt(i, pattern);
  }
  ledModule.renderFieldWhenReady();
  /*
  if (seekDirection = 0){
    currentFrequency -= 0.1;
  }

  if (seekDirection = 1){
    currentFrequency += 0.1;
  }*/
}


void getRadioStation(uint16_t frequency, char* buffer) {
    switch (frequency) {
        case 7800:
            strcpy_P(buffer, station12);
            break;
        case 7950:
            strcpy_P(buffer, station13);
            break;
        case 8000:
            strcpy_P(buffer, station1);
            break;
        case 8130:
            strcpy_P(buffer, station2);
            break;
        case 8190:
        case 8250:
            strcpy_P(buffer, station3);
            break;
        case 8280:
            strcpy_P(buffer, station4);
            break;
        case 8370:
            strcpy_P(buffer, station5);
            break;
        case 8470:
        case 8700:
            strcpy_P(buffer, station6);
            break;
        case 7610:
        case 8970:
            strcpy_P(buffer, station7);
            break;
        case 9050:
            strcpy_P(buffer, station8);
            break;
        case 9160:
            strcpy_P(buffer, station9);
            break;
        case 9240:
            strcpy_P(buffer, station10);
            break;
        case 9300:
            strcpy_P(buffer, station11);
            break;
        default:
            strcpy_P(buffer, stationDefault);
            break;
    }
}