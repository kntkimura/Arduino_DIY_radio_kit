/*
  Arduino Radio + Air Sensor / No Clock Version

  Combined from:
  - Arduino_Radio.ino radio UI concept
  - AirSensor_Clock_only_from_ArduinoRadio.ino sensor code

  Removed:
  - RTC / clock functions
  - Snake game
  - GPS

  Features:
  - FM Radio (RDA5807)
  - AHT temperature/humidity sensor
  - ENS160 air quality sensor
  - OLED SSD1306 128x64 (U8g2)
  - 5-digit 7-segment display (AceSegment + 74HC595)
  - Analog buttons on A0 / A1
  - TimerOne 7-segment refresh
*/

#include <avr/io.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <RDA5807.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <AceSPI.h>
#include <AceSegment.h>
#include <TimerOne.h>

using ace_segment::HybridModule;
using ace_segment::kActiveHighPattern;
using ace_spi::HardSpiInterface;

// ================== I2C bus recovery ==================
void i2cBusRecover() {
  // AVR ATmega328P: SDA=A4=PC4, SCL=A5=PC5
  DDRC  &= ~(_BV(4) | _BV(5));
  PORTC |=  (_BV(4) | _BV(5));

  if (!(PINC & _BV(4))) {
    for (uint8_t i = 0; i < 15; i++) {
      DDRC  |= _BV(5);
      PORTC |= _BV(5);
      asm volatile("nop\nnop\nnop\nnop\n");
      PORTC &= ~_BV(5);
      asm volatile("nop\nnop\nnop\nnop\n");
      DDRC  &= ~_BV(5);
      PORTC |= _BV(5);
    }
  }
}

// ================== OLED ==================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// ================== Radio ==================
RDA5807 rx;
#define DEFAULT_VOLUME 3
const uint8_t app_id = 50;

uint16_t currentFrequency = 0;  // 10kHz unit. 8130 = 81.3MHz
uint16_t currentVolume = DEFAULT_VOLUME;
volatile bool seekup = false;
volatile bool seekdown = false;

// EEPROM layout
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_VOLUME_ADDR = 1;
const int EEPROM_FREQ_L_ADDR = 2;
const int EEPROM_FREQ_H_ADDR = 3;

unsigned long lastUserActionTime = 0;
bool eepromSavePending = false;
bool oledNeedsUpdate = true;

// ================== Sensor ==================
Adafruit_AHTX0 aht;
Adafruit_Sensor *aht_humidity = nullptr;
Adafruit_Sensor *aht_temp = nullptr;
bool aht_ok = false;
sensors_event_t humidityEvent, tempEvent;
float lastTempC = NAN;
float lastHum = NAN;
unsigned long lastSensorRead = 0;

// ENS160 address: if your module is 0x52, change 0x53 to 0x52.
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
bool ens160_ok = false;
uint8_t lastAQI = 0;
uint16_t lastTVOC = 0;
uint16_t lastECO2 = 0;

// ================== Mode / menu ==================
bool radioMode = true;
bool sensorMode = false;
bool inConsole = false;
bool consoleNeedsRedraw = false;
uint8_t consoleIndex = 0;

const char menuItem0[] PROGMEM = "Radio";
const char menuItem1[] PROGMEM = "Sensor";
const char* const menuItems[] PROGMEM = { menuItem0, menuItem1 };
const uint8_t NUM_MENU_ITEMS = sizeof(menuItems) / sizeof(menuItems[0]);

// ================== Buttons ==================
#define STORE_TIME 100
#define LONG_PRESS_START 420
#define LONG_PRESS_INTERVAL 90
unsigned long storeTime = 0;

const uint8_t BTN_NONE = 0;
const uint8_t BTN_1 = 1;
const uint8_t BTN_2 = 2;
const uint8_t BTN_3 = 3;
const uint8_t BTN_4 = 4;

const unsigned long BUTTON_DEBOUNCE_MS = 18;

enum ButtonRepeatIndex {
  BTN_A0_1 = 0,
  BTN_A0_2,
  BTN_A0_3,
  BTN_A0_4,
  BTN_A1_1,
  BTN_A1_2,
  BTN_A1_3,
  BTN_A1_4,
  BTN_REPEAT_COUNT
};

unsigned long buttonPressStart[BTN_REPEAT_COUNT] = {0};
unsigned long buttonLastRepeat[BTN_REPEAT_COUNT] = {0};
bool buttonPrevState[BTN_REPEAT_COUNT] = {false};

bool seekUpLockedUntilRelease = false;
bool seekDownLockedUntilRelease = false;
unsigned long seekIgnoreUntilMs = 0;
const unsigned long SEEK_AFTER_IGNORE_MS = 220;

// Prevent A1_3 long-press repeat from reopening the menu immediately
// after selecting Radio/Sensor. The button must be released once.
bool menuLockedUntilRelease = false;

bool buttonRepeatTriggered(bool pressed, uint8_t index, unsigned long now) {
  if (!pressed) {
    buttonPrevState[index] = false;
    buttonPressStart[index] = 0;
    buttonLastRepeat[index] = 0;
    return false;
  }

  if (!buttonPrevState[index]) {
    buttonPrevState[index] = true;
    buttonPressStart[index] = now;
    buttonLastRepeat[index] = now;
    return true;
  }

  if (now - buttonPressStart[index] < LONG_PRESS_START) return false;

  if (now - buttonLastRepeat[index] >= LONG_PRESS_INTERVAL) {
    buttonLastRepeat[index] = now;
    return true;
  }

  return false;
}

// ================== 7-segment display ==================
const uint8_t NUM_DIGITS = 5;
const uint8_t LATCH_PIN = 10;
const uint8_t DIGIT_PINS[NUM_DIGITS] = { 7, 6, 5, 4, 8 };

const uint8_t FRAMES_PER_SECOND = 200;
const uint8_t NUM_SUBFIELDS = 16;

SPIClass& spiInstance = SPI;
using SpiInterface = HardSpiInterface<SPIClass>;
SpiInterface spiInterface(spiInstance, LATCH_PIN);
HybridModule<SpiInterface, NUM_DIGITS, NUM_SUBFIELDS> ledModule(
  spiInterface,
  kActiveHighPattern,
  kActiveHighPattern,
  FRAMES_PER_SECOND,
  DIGIT_PINS
);

const uint8_t PATTERNS[28] = {
  0b00111111,  // 0
  0b00000110,  // 1
  0b01011011,  // 2
  0b01001111,  // 3
  0b01100110,  // 4
  0b01101101,  // 5
  0b01111101,  // 6
  0b00000111,  // 7
  0b01111111,  // 8
  0b01101111,  // 9
  0b10111111,  // 0.
  0b10000110,  // 1.
  0b11011011,  // 2.
  0b11001111,  // 3.
  0b11100110,  // 4.
  0b11101101,  // 5.
  0b11111101,  // 6.
  0b10000111,  // 7.
  0b11111111,  // 8.
  0b11101111,  // 9.
  0b00000001,  // upper colon
  0b00000010,  // lower colon
  0b00000011,  // colon
  0b00000100,  // '
  0b00000111,  // :'
  0b00111001,  // C
  0b01110011,  // P
  0b00000000   // blank
};

volatile uint8_t segDigits[NUM_DIGITS] = { 27, 27, 27, 27, 27 };

// ================== Station names ==================
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
const char station9[] PROGMEM = " Bunka\nHousou";
const char station10[] PROGMEM = " Radio\nNippon";
const char station11[] PROGMEM = " Nippon\nHousou";
const char stationDefault[] PROGMEM = "no Data";

// ================== Prototypes ==================
void setupAceSegment();
void sevenSegmentDisplay();
void applySegDigitsToModule();
void updateSegForRadio();
void updateSegForSensor();
void blankSevenSegment();
void getButtonState();
void readAllReceiverInformation();
void saveAllReceiverInformation();
void readSensors();
void OLEDdisplayRadio();
void OLEDdisplaySensor();
void drawConsoleScreen();
void applyConsoleSelection();
void getRadioStation(uint16_t frequency, char* buffer);
void drawStrWithNewlines(int x, int y, int lineH, const char* s);
void formatFreq(uint16_t f10kHz, char* out);
uint16_t readAnalogAveraged(uint8_t pin);
uint8_t classifyAnalogButton(uint16_t v);
uint8_t stableButton(uint8_t channelIndex, uint8_t raw, unsigned long now);
void onSeekStep();
void enterRadioMode();
void enterSensorMode();

void setup() {
  i2cBusRecover();
  delay(300);
  i2cBusRecover();

  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  ADCSRA = (ADCSRA & 0xf8) | 0x04;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  delay(50);

  // Radio init
  rx.setup();
  delay(100);
  rx.setBand(RDA_FM_BAND_WORLD);
  rx.setVolume(DEFAULT_VOLUME);
  readAllReceiverInformation();
  rx.setVolume(currentVolume);
  rx.setFrequency(currentFrequency);
  currentFrequency = rx.getFrequency();
  currentVolume = rx.getVolume();

  // Sensor init
  aht_ok = aht.begin();
  if (aht_ok) {
    aht_temp = aht.getTemperatureSensor();
    aht_humidity = aht.getHumiditySensor();
  }
  delay(50);

  ens160_ok = (ens160.begin() == NO_ERR);
  if (ens160_ok) {
    ens160.setPWRMode(ENS160_STANDARD_MODE);
    ens160.setTempAndHum(25.0, 50.0);
  }
  delay(50);

  // OLED
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFlipMode(0);

  // 7-segment
  setupAceSegment();
  ledModule.setBrightness(15);
  Timer1.initialize(50);
  Timer1.attachInterrupt(sevenSegmentDisplay);

  readSensors();
  enterRadioMode();
  oledNeedsUpdate = false;
}

void loop() {
  getButtonState();
  unsigned long now = millis();

  if (inConsole) {
    if (consoleNeedsRedraw) {
      drawConsoleScreen();
      consoleNeedsRedraw = false;
    }
    return;
  }

  if (sensorMode) {
    if (now - lastSensorRead >= 1000UL) {
      readSensors();
      updateSegForSensor();
      oledNeedsUpdate = true;
    }
  }

  const unsigned long OLED_IDLE_MS = 150;
  if (oledNeedsUpdate && (millis() - lastUserActionTime >= OLED_IDLE_MS)) {
    if (radioMode) OLEDdisplayRadio();
    else if (sensorMode) OLEDdisplaySensor();
    oledNeedsUpdate = false;
  }

  if (radioMode && eepromSavePending && (millis() - lastUserActionTime >= 5000UL)) {
    saveAllReceiverInformation();
    eepromSavePending = false;
  }
}

// ================== EEPROM ==================
void readAllReceiverInformation() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == app_id) {
    currentVolume = EEPROM.read(EEPROM_VOLUME_ADDR);
    currentFrequency = EEPROM.read(EEPROM_FREQ_L_ADDR) | (EEPROM.read(EEPROM_FREQ_H_ADDR) << 8);
    if (currentFrequency < 7600 || currentFrequency > 9500) currentFrequency = 8130;
    if (currentVolume > 15) currentVolume = DEFAULT_VOLUME;
  } else {
    currentVolume = DEFAULT_VOLUME;
    currentFrequency = 8130;
  }
}

void saveAllReceiverInformation() {
  EEPROM.update(EEPROM_MAGIC_ADDR, app_id);
  EEPROM.update(EEPROM_VOLUME_ADDR, rx.getVolume());
  uint16_t f = rx.getFrequency();
  EEPROM.update(EEPROM_FREQ_L_ADDR, (uint8_t)(f & 0xFF));
  EEPROM.update(EEPROM_FREQ_H_ADDR, (uint8_t)(f >> 8));
}

// ================== Sensor ==================
void readSensors() {
  if (aht_ok) {
    aht_humidity->getEvent(&humidityEvent);
    aht_temp->getEvent(&tempEvent);
    lastTempC = tempEvent.temperature - 3.5; 
    lastHum = humidityEvent.relative_humidity;
  }

  if (ens160_ok) {
    if (aht_ok && !isnan(lastTempC) && !isnan(lastHum)) {
      ens160.setTempAndHum(lastTempC, lastHum);
    }
    lastAQI = ens160.getAQI();
    lastTVOC = ens160.getTVOC();
    lastECO2 = ens160.getECO2();
  }

  lastSensorRead = millis();
}

// ================== SEEK callback ==================
void onSeekStep() {
  currentFrequency = rx.getFrequency();
  updateSegForRadio();

  if ((currentFrequency > 9500) && seekup) {
    rx.setFrequency(7600);
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStep);
  }

  if ((currentFrequency <= 7600) && seekdown) {
    rx.setFrequency(9500);
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStep);
  }
}

// ================== 7-segment ==================
void setupAceSegment() {
  spiInstance.begin();
  spiInterface.begin();
  ledModule.begin();
}

void sevenSegmentDisplay() {
  ledModule.renderFieldWhenReady();
}

void applySegDigitsToModule() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    uint8_t d = segDigits[i];
    if (d > 27) d = 27;
    ledModule.setPatternAt(i, PATTERNS[d]);
  }
  interrupts();
}

void updateSegForRadio() {
  noInterrupts();
  segDigits[0] = currentFrequency / 1000 % 10;
  segDigits[1] = currentFrequency / 100 % 10 + 10;  // decimal point
  segDigits[2] = currentFrequency / 10 % 10;
  segDigits[3] = currentVolume % 10;
  segDigits[4] = (currentVolume > 9) ? 23 : 27;
  interrupts();
  applySegDigitsToModule();
}

// Sensor 7-seg: show eCO2 ppm as 3 digits + P + P marker.
void updateSegForSensor() {
  if (!ens160_ok || lastECO2 == 0) {
    blankSevenSegment();
    return;
  }

  uint16_t co2 = lastECO2;
  if (co2 > 9999) co2 = 9999;

  uint8_t d0 = (co2 / 100) % 10;
  uint8_t d1 = (co2 / 10) % 10;
  uint8_t d2 = co2 % 10;

  if (d0 == 0) d0 = 27;
  if (d0 == 27 && d1 == 0) d1 = 27;
  if (d0 == 27 && d1 == 27 && d2 == 0) d2 = 27;

  noInterrupts();
  segDigits[0] = d0;
  segDigits[1] = d1;
  segDigits[2] = d2;
  segDigits[3] = 26;  // P
  segDigits[4] = 23;  // " ' " for the expression of " 'P "
  interrupts();
  applySegDigitsToModule();
}

void blankSevenSegment() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) segDigits[i] = 27;
  interrupts();
  applySegDigitsToModule();
}

// ================== Buttons ==================
uint16_t readAnalogAveraged(uint8_t pin) {
  uint16_t sum = 0;
  analogRead(pin);  // discard first read after mux change
  for (uint8_t i = 0; i < 4; i++) {
    sum += analogRead(pin);
    delayMicroseconds(120);
  }
  return sum / 4;
}

uint8_t classifyAnalogButton(uint16_t v) {
  // Representative values: 0 / 510 / 679 / 768 / no press 1023
  if (v < 190) return BTN_1;
  if (v >= 370 && v <= 605) return BTN_2;
  if (v >= 610 && v <= 735) return BTN_3;
  if (v >= 740 && v <= 875) return BTN_4;
  return BTN_NONE;
}

uint8_t stableButton(uint8_t channelIndex, uint8_t raw, unsigned long now) {
  static uint8_t lastRaw[2] = {BTN_NONE, BTN_NONE};
  static uint8_t stable[2] = {BTN_NONE, BTN_NONE};
  static unsigned long changedAt[2] = {0, 0};

  if (raw != lastRaw[channelIndex]) {
    lastRaw[channelIndex] = raw;
    changedAt[channelIndex] = now;
  }

  if ((now - changedAt[channelIndex]) >= BUTTON_DEBOUNCE_MS) {
    stable[channelIndex] = raw;
  }

  return stable[channelIndex];
}

void getButtonState() {
  unsigned long now = millis();

  uint16_t a0v = readAnalogAveraged(A0);
  uint16_t a1v = readAnalogAveraged(A1);

  uint8_t a0b = stableButton(0, classifyAnalogButton(a0v), now);
  uint8_t a1b = stableButton(1, classifyAnalogButton(a1v), now);

  bool a0_1 = (a0b == BTN_1);  // radio freq down / menu up
  bool a0_2 = (a0b == BTN_2);  // radio freq up / menu down
  bool a0_3 = (a0b == BTN_3);  // seek up
  bool a0_4 = (a0b == BTN_4);  // seek down

  bool a1_1 = (a1b == BTN_1);  // volume down / menu down
  bool a1_2 = (a1b == BTN_2);  // volume up / menu up
  bool a1_3 = (a1b == BTN_3);  // menu / enter
  bool a1_4 = (a1b == BTN_4);  // cancel

  if (!a1_3) menuLockedUntilRelease = false;

  if (!a0_3) seekUpLockedUntilRelease = false;
  if (!a0_4) seekDownLockedUntilRelease = false;

  bool trigA0_1 = buttonRepeatTriggered(a0_1, BTN_A0_1, now);
  bool trigA0_2 = buttonRepeatTriggered(a0_2, BTN_A0_2, now);
  bool trigA0_3 = buttonRepeatTriggered(a0_3, BTN_A0_3, now);
  bool trigA0_4 = buttonRepeatTriggered(a0_4, BTN_A0_4, now);
  bool trigA1_1 = buttonRepeatTriggered(a1_1, BTN_A1_1, now);
  bool trigA1_2 = buttonRepeatTriggered(a1_2, BTN_A1_2, now);
  bool trigA1_3 = buttonRepeatTriggered(a1_3, BTN_A1_3, now);
  bool trigA1_4 = buttonRepeatTriggered(a1_4, BTN_A1_4, now);

  bool anyPressed = false;

  if (inConsole) {
    if (trigA0_1 || trigA1_2) {
      consoleIndex = (consoleIndex + NUM_MENU_ITEMS - 1) % NUM_MENU_ITEMS;
      consoleNeedsRedraw = true;
      anyPressed = true;
    } else if (trigA0_2 || trigA1_1) {
      consoleIndex = (consoleIndex + 1) % NUM_MENU_ITEMS;
      consoleNeedsRedraw = true;
      anyPressed = true;
    } else if (trigA1_3 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      applyConsoleSelection();
      inConsole = false;
      menuLockedUntilRelease = true;
      anyPressed = true;
    } else if (trigA1_4 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      inConsole = false;
      menuLockedUntilRelease = true;
      oledNeedsUpdate = true;
      anyPressed = true;
    }

    if (anyPressed) lastUserActionTime = millis();
    return;
  }

  if (trigA1_3 && !menuLockedUntilRelease && (now - storeTime) > STORE_TIME) {
    storeTime = now;
    inConsole = true;
    consoleIndex = radioMode ? 0 : 1;
    consoleNeedsRedraw = true;
    anyPressed = true;
  }

  if (radioMode) {
    if (trigA0_1 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      currentFrequency = rx.getFrequency();
      if (currentFrequency <= 7600) currentFrequency = 9500;
      else currentFrequency -= 10;
      rx.setFrequency(currentFrequency);
      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      eepromSavePending = true;
      anyPressed = true;

    } else if (trigA0_2 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      currentFrequency = rx.getFrequency();
      if (currentFrequency >= 9500) currentFrequency = 7600;
      else currentFrequency += 10;
      rx.setFrequency(currentFrequency);
      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      eepromSavePending = true;
      anyPressed = true;

    } else if (trigA0_3 && !seekUpLockedUntilRelease && now >= seekIgnoreUntilMs) {
      seekUpLockedUntilRelease = true;
      seekIgnoreUntilMs = now + SEEK_AFTER_IGNORE_MS;
      seekup = true;
      seekdown = false;
      rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStep);
      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      eepromSavePending = true;
      anyPressed = true;

    } else if (trigA0_4 && !seekDownLockedUntilRelease && now >= seekIgnoreUntilMs) {
      seekDownLockedUntilRelease = true;
      seekIgnoreUntilMs = now + SEEK_AFTER_IGNORE_MS;
      seekup = false;
      seekdown = true;
      rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStep);
      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      eepromSavePending = true;
      anyPressed = true;

    } else if (trigA1_1 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      currentVolume = rx.getVolume();
      if (currentVolume > 0) currentVolume--;
      rx.setVolume(currentVolume);
      currentVolume = rx.getVolume();
      updateSegForRadio();
      oledNeedsUpdate = true;
      eepromSavePending = true;
      anyPressed = true;

    } else if (trigA1_2 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      currentVolume = rx.getVolume();
      if (currentVolume < 15) currentVolume++;
      rx.setVolume(currentVolume);
      currentVolume = rx.getVolume();
      updateSegForRadio();
      oledNeedsUpdate = true;
      eepromSavePending = true;
      anyPressed = true;
    }
  }

  if (anyPressed) lastUserActionTime = millis();
}

// ================== Mode switching ==================
void enterRadioMode() {
  radioMode = true;
  sensorMode = false;
  rx.powerUp();
  rx.setBand(RDA_FM_BAND_WORLD);
  rx.setFrequency(currentFrequency);
  rx.setVolume(currentVolume);
  currentFrequency = rx.getFrequency();
  currentVolume = rx.getVolume();
  updateSegForRadio();
  OLEDdisplayRadio();
  oledNeedsUpdate = false;
}

void enterSensorMode() {
  radioMode = false;
  sensorMode = true;
  currentFrequency = rx.getFrequency();
  currentVolume = rx.getVolume();
  rx.setVolume(0);
  rx.powerDown();  // sensor mode reduces radio noise/current
  readSensors();
  updateSegForSensor();
  OLEDdisplaySensor();
  oledNeedsUpdate = false;
}

void applyConsoleSelection() {
  if (consoleIndex == 0) enterRadioMode();
  else enterSensorMode();
}

// ================== OLED ==================
void drawConsoleScreen() {
  u8g2.setPowerSave(0);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(0, 12, "Mode Select");

    for (uint8_t i = 0; i < NUM_MENU_ITEMS; i++) {
      char buf[12];
      strcpy_P(buf, (PGM_P)pgm_read_word(&menuItems[i]));
      int y = 30 + i * 16;

      if (i == consoleIndex) {
        u8g2.drawBox(0, y - 12, 128, 15);
        u8g2.setDrawColor(0);
        u8g2.drawStr(4, y, buf);
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(4, y, buf);
      }
    }
  } while (u8g2.nextPage());
}

void formatFreq(uint16_t f10kHz, char* out) {
  uint8_t mhz = f10kHz / 100;
  out[0] = (mhz >= 100) ? '1' : ' ';
  out[1] = char('0' + (mhz / 10) % 10);
  out[2] = char('0' + mhz % 10);
  out[3] = '.';
  out[4] = char('0' + ((f10kHz % 100) / 10));
  out[5] = '\0';
}

void getRadioStation(uint16_t frequency, char* buffer) {
  switch (frequency) {
    case 7800: strcpy_P(buffer, station12); break;
    case 7950: strcpy_P(buffer, station13); break;
    case 8000: strcpy_P(buffer, station1); break;
    case 8130: strcpy_P(buffer, station2); break;
    case 8190:
    case 8250: strcpy_P(buffer, station3); break;
    case 8280: strcpy_P(buffer, station4); break;
    case 8370: strcpy_P(buffer, station5); break;
    case 8470:
    case 8700: strcpy_P(buffer, station6); break;
    case 7650:
    case 8970: strcpy_P(buffer, station7); break;
    case 9050: strcpy_P(buffer, station8); break;
    case 9160: strcpy_P(buffer, station9); break;
    case 9240: strcpy_P(buffer, station10); break;
    case 9300: strcpy_P(buffer, station11); break;
    default: strcpy_P(buffer, stationDefault); break;
  }
}

void drawStrWithNewlines(int x, int y, int lineH, const char* s) {
  const char* p = s;
  int line = 0;
  while (*p) {
    char buf[22];
    int i = 0;
    while (*p && *p != '\n' && i < (int)sizeof(buf) - 1) {
      buf[i++] = *p++;
    }
    buf[i] = '\0';
    u8g2.drawStr(x, y + line * lineH, buf);
    if (*p == '\n') {
      p++;
      line++;
    }
  }
}

void OLEDdisplayRadio() {
  u8g2.setPowerSave(0);

  char fbuf[8];
  char vbuf[4];
  char station[18];
  currentFrequency = rx.getFrequency();
  currentVolume = rx.getVolume();
  formatFreq(currentFrequency, fbuf);
  getRadioStation(currentFrequency, station);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso16_tn);
    int fw = u8g2.getStrWidth(fbuf);
    u8g2.drawStr(0, 18, fbuf);

    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(fw + 2, 18, "MHz");

    u8g2.setFont(u8g2_font_10x20_tf);
    int sw = u8g2.getStrWidth(station);
    int sx = (128 - sw) / 2;
    drawStrWithNewlines(sx, 42, 20, station);

    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(80, 64, "Vol");
    u8g2.setFont(u8g2_font_logisoso16_tn);
    vbuf[0] = (currentVolume >= 10) ? char('0' + currentVolume / 10) : ' ';
    vbuf[1] = char('0' + currentVolume % 10);
    vbuf[2] = '\0';
    u8g2.drawStr(100, 64, vbuf);
  } while (u8g2.nextPage());
}

void OLEDdisplaySensor() {
  u8g2.setPowerSave(0);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);
    int y = 16;

    if (aht_ok && !isnan(lastTempC) && !isnan(lastHum)) {
      int t10 = (int)(lastTempC * 10.0f + (lastTempC >= 0 ? 0.5f : -0.5f));
      char signT = (t10 < 0) ? '-' : ' ';
      if (t10 < 0) t10 = -t10;

      u8g2.setCursor(0, y);
      u8g2.print(F("Temp:"));
      u8g2.print(signT);
      u8g2.print(t10 / 10);
      u8g2.print('.');
      u8g2.print(t10 % 10);
      u8g2.print(F(" C"));
      y += 16;

      int h10 = (int)(lastHum * 10.0f + 0.5f);
      if (h10 < 0) h10 = 0;
      u8g2.setCursor(0, y);
      u8g2.print(F("Hum :"));
      u8g2.print(h10 / 10);
      u8g2.print('.');
      u8g2.print(h10 % 10);
      u8g2.print(F(" %"));
      y += 16;
    } else {
      u8g2.setCursor(0, y);
      u8g2.print(F("AHT ERROR"));
      y += 16;
    }

    if (ens160_ok) {
      u8g2.setCursor(0, y);
      u8g2.print(F("eCO2:"));
      u8g2.print(lastECO2);
      u8g2.print(F(" ppm"));
      y += 16;

      u8g2.setCursor(0, y);
      u8g2.print(F("AQI:"));
      u8g2.print(lastAQI);
      u8g2.print(F(" TVOC:"));
      u8g2.print(lastTVOC);
    } else {
      u8g2.setCursor(0, y);
      u8g2.print(F("ENS160 ERR"));
    }
  } while (u8g2.nextPage());
}
