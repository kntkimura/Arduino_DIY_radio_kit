// Merged for user request:
// - Based on ArduinoRadio_MultiFunction_6_longpress_darkmode_scrolled(1).ino
// - Sensor functions removed
// - RTC/Clock/Alarm functions removed
// - GPS screen added from MiniGPS_selfFix(1).ino
// - Modified: GPS mode shows current latitude/longitude instead of bitmap/map/distance screen.
//
#include <avr/io.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <RDA5807.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <TinyGPS.h>


void i2cBusRecover() {
  // AVR(ATmega328P): SDA=A4=PC4, SCL=A5=PC5
  // Pull-ups ON
  DDRC  &= ~(_BV(4) | _BV(5));   // inputs
  PORTC |=  (_BV(4) | _BV(5));   // enable pull-ups

  // If SDA is being held LOW, pulse SCL ~9 times to try to release it
  if (!(PINC & _BV(4))) {
    for (uint8_t i = 0; i < 15; i++) {
      DDRC  |= _BV(5);           // SCL output
      PORTC |= _BV(5);           // SCL high
      asm volatile("nop\nnop\nnop\nnop\n");
      PORTC &= ~_BV(5);          // SCL low
      asm volatile("nop\nnop\nnop\nnop\n");
      DDRC  &= ~_BV(5);          // release SCL
      PORTC |= _BV(5);           // keep pull-up
    }
  }
}


#include <AceSPI.h>
#include <AceSegment.h>
#include <FlexiTimer2.h>

using ace_segment::HybridModule;
using ace_segment::kActiveHighPattern;
using ace_spi::HardSpiInterface;

// ================== OLED (SSD1306 128x64) ==================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// ================== RDA5807 ラジオ ==================
RDA5807 rx;
#define DEFAULT_VOLUME 3
#define STORE_TIME 100  // 単発ボタン用の最小間隔(ms)
#define LONG_PRESS_START 500     // 長押し連続開始までの時間(ms)
#define LONG_PRESS_INTERVAL 120  // 長押し連続送りの間隔(ms)

const uint8_t app_id = 50;  // EEPROM用識別ID
const int eeprom_address = 0;

uint16_t currentFrequency = 0;  // 10kHz単位 (例: 8970 = 89.7MHz)
uint16_t currentVolume = DEFAULT_VOLUME;

// 操作・表示関連
unsigned long storeTime = 0;
unsigned long colonStartMs = 0;

// EEPROM 遅延セーブ用
unsigned long lastUserActionTime = 0;  // 最後にボタン操作があった時刻
bool eepromSavePending = false;        // セーブ待ちフラグ

// OLED 遅延更新用
bool oledNeedsUpdate = true;  // 何か変化があり、OLEDを再描画すべきか

// コンソール描画フラグ
bool consoleNeedsRedraw = false;

// ==== SEEK制御用フラグ ====
volatile bool seekup = false;
volatile bool seekdown = false;

// ================== モード管理 ==================
bool gpsMode = false;     // GPS画面
bool darkMode = false;    // ラジオ動作中に表示だけ消すダークモード
bool inConsole = false;   // コンソール画面中か
uint8_t consoleIndex = 0;


// コンソールメニュー
const char menuItem0[] PROGMEM = "Radio";
const char menuItem1[] PROGMEM = "GPS";
const char menuItem2[] PROGMEM = "Dark";
const char* const menuItems[] PROGMEM = {
  menuItem0,
  menuItem1,
  menuItem2,
};
const uint8_t NUM_MENU_ITEMS =
  sizeof(menuItems) / sizeof(menuItems[0]);

// ================== GPS ==================
// ハードウェアシリアル版です。
// GPS TX -> Arduino D0/RX、GPS RX -> Arduino D1/TX に接続します。
// ※D0/D1を使うため、書き込み時やSerialモニタ使用時はGPSのTX/RX配線が干渉する場合があります。
TinyGPS gps;
bool gpsSerialActive = false;  // GPSモード中だけSerialを有効にする
bool radioPoweredDownForGps = false;  // GPSモード中はRDA5807を電源OFFにする

const float TARGET_LAT = 35.319209;   // 鎌倉市役所の北緯
const float TARGET_LON = 139.546962;  // 鎌倉市役所の東経
const uint16_t RADIUS_METERS = 50;
// 鎌倉周辺での概算: 緯度1度=約111.32km、経度1度=約91.2km
const float LAT_M_PER_DEG = 111320.0;
const float LON_M_PER_DEG = 91200.0;

float currentLat = TinyGPS::GPS_INVALID_F_ANGLE;
float currentLon = TinyGPS::GPS_INVALID_F_ANGLE;
unsigned long currentAge = TinyGPS::GPS_INVALID_AGE;
bool hasFix = false;
bool inRange = false;
uint16_t distanceMeters = 9999;
unsigned long lastGpsDisplayUpdate = 0;
const unsigned long GPS_DISPLAY_INTERVAL_MS = 500;


// 長押し連続入力管理
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
    return true;  // 押した瞬間に1回
  }

  if (now - buttonPressStart[index] < LONG_PRESS_START) {
    return false;
  }

  if (now - buttonLastRepeat[index] >= LONG_PRESS_INTERVAL) {
    buttonLastRepeat[index] = now;
    return true;
  }

  return false;
}

// ================== 7セグ (AceSegment) ==================
const uint8_t NUM_DIGITS = 5;
const uint8_t NUM_SEGMENTS = 8;
const uint8_t LATCH_PIN = 10;
const uint8_t DATA_PIN = MOSI;
const uint8_t CLOCK_PIN = SCK;
const uint8_t DIGIT_PINS[NUM_DIGITS] = { 7, 6, 5, 4, 8 };
//const uint8_t COLON_PIN = 8;  // 物理コロンLED用（RTC割り込みD3と競合しないよう変更）

const uint8_t FRAMES_PER_SECOND = 200;
const uint8_t NUM_SUBFIELDS = 16;

bool digit4state = false;

// SPI インスタンス
SPIClass& spiInstance = SPI;

// AceSPI インターフェイス
using SpiInterface = HardSpiInterface<SPIClass>;
SpiInterface spiInterface(spiInstance, LATCH_PIN);

// 7セグモジュール
HybridModule<SpiInterface, NUM_DIGITS, NUM_SUBFIELDS> ledModule(
  spiInterface,
  kActiveHighPattern,
  kActiveHighPattern,
  FRAMES_PER_SECOND,
  DIGIT_PINS);

// 7セグ パターン
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
  0b00000001,  // : upper (20)
  0b00000010,  // : lower (21)
  0b00000011,  // : (22) 両方
  0b00000100,  // '(23)
  0b00000111,  // :'(24)
  0b00111001,  // C (25)
  0b01110011,  // P (26)
  0b00000000   // blank (27)
};

// ISR から表示するための digit 配列（中身の更新は loop 側）
volatile uint8_t segDigits[NUM_DIGITS] = { 27, 27, 27, 27, 27 };

// ================== 局名（PROGMEM） ==================
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

// ================== EEPROM 配置 ==================
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_VOLUME_ADDR = 1;
const int EEPROM_FREQ_L_ADDR = 2;
const int EEPROM_FREQ_H_ADDR = 3;

// ================== プロトタイプ ==================
void setupAceSegment();
void sevenSegmentDisplay();
void getButtonState();

void OLEDdisplayRadio();
void OLEDdisplayGps();
void OLEDdisplayDark();

void drawConsoleScreen();
void applyConsoleSelection();
void readAllReceiverInformation();
void saveAllReceiverInformation();

void updateSegForRadio();
void updateSegForGps();
void updateSegForDark();
void blankSevenSegmentNow();
void enterGpsMode();
void exitGpsMode();
void onSeekStep();

void formatFreq(uint16_t f10kHz, char* out);
void getRadioStation(uint16_t frequency, char* buffer);
void drawStrWithNewlines(int x, int y, int lineH, const char* s);
void drawRadioScreen(const char* station, uint8_t vol, uint16_t f10kHz);
uint16_t calcDistanceMetersFast(float lat, float lon);
bool readDataFromGps();
void updateGpsData();
void processGps();
void drawGpsScreen();


// ================== EEPROM 読み書き ==================

void readAllReceiverInformation() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == app_id) {
    currentVolume = EEPROM.read(EEPROM_VOLUME_ADDR);
    currentFrequency = EEPROM.read(EEPROM_FREQ_L_ADDR)
                    | (EEPROM.read(EEPROM_FREQ_H_ADDR) << 8);
  } else {
    currentVolume = DEFAULT_VOLUME;
    currentFrequency = 8130;  // 81.3MHz
  }
}

void saveAllReceiverInformation() {
  EEPROM.update(EEPROM_MAGIC_ADDR, app_id);
  EEPROM.update(EEPROM_VOLUME_ADDR, rx.getVolume());
  uint16_t f = rx.getFrequency();
  EEPROM.update(EEPROM_FREQ_L_ADDR, (uint8_t)(f & 0xFF));
  EEPROM.update(EEPROM_FREQ_H_ADDR, (uint8_t)(f >> 8));

  digit4state = true;
  colonStartMs = millis();
  updateSegForRadio();
}

// ================== ステーション名取得（PROGMEM→RAM） ==================
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

// ================== 文字列描画（改行対応） ==================
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

// ================== 周波数文字列整形 ==================
void formatFreq(uint16_t f10kHz, char* out, size_t n) {
  int mhz = f10kHz / 100;  // 10kHz単位→MHz整数
  int dec1 = (f10kHz % 100) / 10;
  snprintf(out, n, "%2d.%1d", mhz, dec1);
}

// ================== ラジオ画面描画 ==================
void drawRadioScreen(const char* station, uint8_t vol, uint16_t f10kHz) {
  char fbuf[8];
  char vbuf[4];
  formatFreq(f10kHz, fbuf, sizeof(fbuf));

  u8g2.firstPage();
  do {
    // 周波数（大数字）
    u8g2.setFont(u8g2_font_logisoso16_tn);  // 数字専用・軽量
    int fw = u8g2.getStrWidth(fbuf);
    u8g2.drawStr(0, 18, fbuf);

    // "MHz"
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(fw + 2, 18, "MHz");

    // 局名（改行対応：8x13）
    u8g2.setFont(u8g2_font_10x20_tf);
    int sw = u8g2.getStrWidth(station);  // 1行目の幅を基準
    int sx = (128 - sw) / 2;
    drawStrWithNewlines(sx, 38, 13, station);

    // Vol
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(80, 64, "Vol");
    u8g2.setFont(u8g2_font_logisoso16_tn);
    snprintf(vbuf, sizeof(vbuf), "%2u", vol);
    u8g2.drawStr(100, 64, vbuf);
  } while (u8g2.nextPage());
}

// ================== SEEK コールバック ==================
void onSeekStep() {
  // 今の周波数を更新
  currentFrequency = rx.getFrequency();

  // シーク中の 7セグアニメ（0.1MHzごとにカウントしていく）
  updateSegForRadio();

  // 上方向シーク中に 95.0MHz を超えたら 76.0MHz へ巻き戻し
  if ((currentFrequency > 9500) && seekup) {
    rx.setFrequency(7600);
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStep);
  }

  // 下方向シーク中に 76.0MHz 未満になったら 95.0MHz へ巻き戻し
  if ((currentFrequency <= 7600) && seekdown) {
    rx.setFrequency(9500);
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStep);
  }
}

// ================== setup / loop ==================
void setup() {
  i2cBusRecover();
  delay(500);
  i2cBusRecover();
  Wire.begin();
  Wire.setClock(100000);  // まずは安全に100kHz
  delay(300);

  // ADC分周（元コード踏襲）
  ADCSRA = (ADCSRA & 0xf8) | 0x04;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  //pinMode(COLON_PIN, OUTPUT);
  //digitalWrite(COLON_PIN, LOW);
  delay(50);

  // ラジオ初期化
  rx.setup();
  delay(100);
  rx.setBand(RDA_FM_BAND_WORLD);
  rx.setVolume(DEFAULT_VOLUME);
  readAllReceiverInformation();
  rx.setVolume(currentVolume);
  rx.setFrequency(currentFrequency);
  currentFrequency = rx.getFrequency();
  currentVolume = rx.getVolume();
  delay(50);

  // GPSは通常時OFF。GPSモードに入った時だけハードウェアSerialを開始する。

  // OLED
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFlipMode(0);

  // AceSegment
  setupAceSegment();
  ledModule.setBrightness(15);
  FlexiTimer2::set(2, 1.0 / 10000, sevenSegmentDisplay);
  FlexiTimer2::start();

  // 初期表示はラジオ
  updateSegForRadio();
  OLEDdisplayRadio();
  oledNeedsUpdate = false;
}

void loop() {
  getButtonState();
  if (gpsMode) {
    processGps();
  }
  unsigned long now = millis();

  if (inConsole) {
    if (consoleNeedsRedraw) {
      drawConsoleScreen();
      consoleNeedsRedraw = false;
    }
  } else if (gpsMode) {
    if (now - lastGpsDisplayUpdate >= GPS_DISPLAY_INTERVAL_MS) {
      lastGpsDisplayUpdate = now;
      oledNeedsUpdate = true;
    }
  } else if (darkMode) {
    // ダークモード：表示は消したまま、ラジオ操作のみ許可
  } else {
    // ラジオモード
  }

  // ===== OLED 共通遅延描画処理 =====
  const unsigned long OLED_IDLE_MS = 200;
  if (!inConsole && oledNeedsUpdate && (millis() - lastUserActionTime >= OLED_IDLE_MS)) {
    if (gpsMode) {
      OLEDdisplayGps();
    } else if (darkMode) {
      OLEDdisplayDark();
    } else {
      OLEDdisplayRadio();
    }
    oledNeedsUpdate = false;
  }

  // ===== EEPROM 遅延セーブ（最後の操作から5秒後に1回だけ） =====
  if (!gpsMode && eepromSavePending && (millis() - lastUserActionTime >= 5000)) {
    saveAllReceiverInformation();
    eepromSavePending = false;
  }

  // コロン点灯の自動消灯（操作がなくても1秒で消す）
  if (!gpsMode) {
    if (digit4state && (millis() - colonStartMs >= 1000UL)) {
      colonStartMs = millis();
      digit4state = false;
      updateSegForRadio();
    }
  }
}

// ================== 7セグ描画（ISR側は「表示だけ」） ==================
void sevenSegmentDisplay() {
  // ハードウェアシリアルでは受信時に割り込みを長く止めないため、
  // GPSモード中も7セグ多重表示を継続する。
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    uint8_t d = segDigits[i];
    if (d > 27) d = 27;
    ledModule.setPatternAt(i, PATTERNS[d]);
  }
  ledModule.renderFieldWhenReady();
}

// Radio モード用 digit 設定
void updateSegForRadio() {
  if (darkMode) {
    updateSegForDark();
    return;
  }

  noInterrupts();
  segDigits[0] = currentFrequency / 1000 % 10;
  segDigits[1] = currentFrequency / 100 % 10 + 10;  // ドット付き
  segDigits[2] = currentFrequency / 10 % 10;
  segDigits[3] = currentVolume % 10;
  segDigits[4] = (currentVolume > 9) ? 23 : 27;  // ' or blank
  if (digit4state) {                             //digit4stateはセーブ時のフラグ
    segDigits[4] = (currentVolume > 9) ? 24 : 22;
  }
  interrupts();
}

// GPS モード用 digit 設定
void updateSegForGps() {
  // GPSモードではOLEDに現在座標を表示するため、7セグは消灯する。
  // ここで距離表示を行わないことで、GPS画面の意味を座標表示に統一する。
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) segDigits[i] = 27;
  interrupts();
  //digitalWrite(COLON_PIN, LOW);
}

void updateSegForDark() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    segDigits[i] = 27;
  }
  interrupts();

  //digitalWrite(COLON_PIN, LOW);
}

void blankSevenSegmentNow() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    segDigits[i] = 27;
  }
  interrupts();

  // 念のため、現在点灯中の桁を消してからタイマーを止める
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    ledModule.setPatternAt(i, PATTERNS[27]);
  }
  for (uint8_t i = 0; i < 8; i++) {
    ledModule.renderFieldWhenReady();
  }
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    pinMode(DIGIT_PINS[i], OUTPUT);
    digitalWrite(DIGIT_PINS[i], LOW);  // digit pinはActive HighなのでLOWで消灯
  }
}

void enterGpsMode() {
  gpsMode = true;
  darkMode = false;
  lastGpsDisplayUpdate = 0;

  // GPSモードではラジオをOFFにする。
  // 戻るときに復帰できるよう、現在値を保持してからRDA5807をPower Downする。
  if (!radioPoweredDownForGps) {
    currentFrequency = rx.getFrequency();
    currentVolume = rx.getVolume();
    rx.setVolume(0);
    rx.powerDown();
    radioPoweredDownForGps = true;
  }

  // GPSモード中も7セグを表示する。
  // 測位できるまでは7セグを消灯し、測位後は距離を表示する。
  updateSegForGps();

  if (!gpsSerialActive) {
    Serial.begin(9600);         // GPSモードに入った時だけハードウェアシリアル開始
    while (Serial.available()) Serial.read();  // 残っている文字を捨てる
    gpsSerialActive = true;
  }

  oledNeedsUpdate = true;
}

void exitGpsMode() {
  if (gpsSerialActive) {
    Serial.end();               // 通常時はGPS用Serialを止める
    gpsSerialActive = false;
  }

  // GPSモードでOFFにしたラジオを復帰する。
  if (radioPoweredDownForGps) {
    rx.powerUp();
    delay(80);
    rx.setBand(RDA_FM_BAND_WORLD);
    rx.setFrequency(currentFrequency);
    rx.setVolume(currentVolume);
    currentFrequency = rx.getFrequency();
    currentVolume = rx.getVolume();
    radioPoweredDownForGps = false;
  }

  gpsMode = false;
  updateSegForRadio();          // ラジオ表示へ戻す
}

// ================== AceSegment 初期化 ==================
void setupAceSegment() {
  spiInstance.begin();
  spiInterface.begin();
  ledModule.begin();
}

// ================== ボタン処理 ==================
void getButtonState() {
  uint16_t a0 = analogRead(A0);
  uint16_t a1 = analogRead(A1);
  unsigned long now = millis();

  bool a0_1 = (a0 < 200);               // 周波数↓ / メニュー↑
  bool a0_2 = (a0 >= 200 && a0 < 560);  // 周波数↑ / メニュー↓
  bool a0_3 = (a0 >= 560 && a0 < 710);  // シークUP
  bool a0_4 = (a0 >= 710 && a0 < 800);  // シークDOWN

  bool a1_1 = (a1 < 200);               // 音量↓
  bool a1_2 = (a1 >= 200 && a1 < 560);  // 音量↑
  bool a1_3 = (a1 >= 560 && a1 < 710);  // コンソール起動 / 決定
  bool a1_4 = (a1 >= 710 && a1 < 800);  // コンソールキャンセル

  bool trigA0_1 = buttonRepeatTriggered(a0_1, BTN_A0_1, now);
  bool trigA0_2 = buttonRepeatTriggered(a0_2, BTN_A0_2, now);
  bool trigA0_3 = buttonRepeatTriggered(a0_3, BTN_A0_3, now);
  bool trigA0_4 = buttonRepeatTriggered(a0_4, BTN_A0_4, now);
  bool trigA1_1 = buttonRepeatTriggered(a1_1, BTN_A1_1, now);
  bool trigA1_2 = buttonRepeatTriggered(a1_2, BTN_A1_2, now);
  bool trigA1_3 = buttonRepeatTriggered(a1_3, BTN_A1_3, now);
  bool trigA1_4 = buttonRepeatTriggered(a1_4, BTN_A1_4, now);

  bool anyPressed = false;

  // ===== コンソール中 =====
  if (inConsole) {
    if (trigA1_2) {
      consoleIndex = (consoleIndex + NUM_MENU_ITEMS - 1) % NUM_MENU_ITEMS;
      consoleNeedsRedraw = true;
      anyPressed = true;

    } else if (trigA1_1) {
      consoleIndex = (consoleIndex + 1) % NUM_MENU_ITEMS;
      consoleNeedsRedraw = true;
      anyPressed = true;

    } else if (trigA1_3 && (now - storeTime) > STORE_TIME) {
      // 決定：ここで即モードを適用し、OLEDも切り替える
      storeTime = now;
      applyConsoleSelection();
      inConsole = false;
      anyPressed = true;

    } else if (trigA1_4 && (now - storeTime) > STORE_TIME) {
      // キャンセル：コンソールを閉じて元のモード画面に戻す
      storeTime = now;
      inConsole = false;
      anyPressed = true;
      oledNeedsUpdate = true;  // idle描画で元画面を再描画
    }

    if (anyPressed) {
      lastUserActionTime = millis();
      // コンソール操作では EEPROM 保存不要なので eepromSavePending は触らない
    }
    return;
  }

  // ===== 通常のラジオ操作 =====
  // GPSモード中はRDA5807をPower Downしているため、周波数・音量操作は無効にする。
  if (!gpsMode) {
    if (trigA0_1) {  // 周波数↓
      rx.setFrequencyDown();
      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA0_2) {  // 周波数↑
      rx.setFrequencyUp();
      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA0_3 && (now - storeTime) > STORE_TIME) {  // シークUP
      storeTime = now;
      seekup = true;
      rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStep);
      seekup = false;

      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA0_4 && (now - storeTime) > STORE_TIME) {  // シークDOWN
      storeTime = now;
      seekdown = true;
      rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStep);
      seekdown = false;

      currentFrequency = rx.getFrequency();
      updateSegForRadio();
      oledNeedsUpdate = true;
      anyPressed = true;
    }

    if (trigA1_1) {  // 音量↓
      rx.setVolumeDown();
      currentVolume = rx.getVolume();
      updateSegForRadio();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA1_2) {  // 音量↑
      rx.setVolumeUp();
      currentVolume = rx.getVolume();
      updateSegForRadio();
      oledNeedsUpdate = true;
      anyPressed = true;
    }
  }

  // コンソール起動
  if (trigA1_3 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      inConsole = true;
      consoleNeedsRedraw = true;  // 入った瞬間だけ描画
      anyPressed = true;
  }

  // 何かボタンが押されたら「最後の操作時刻」を更新
  if (anyPressed) {
    lastUserActionTime = millis();
    eepromSavePending = true;
  }
}

// ================== コンソール描画 / 選択 ==================
void drawConsoleScreen() {
  u8g2.setPowerSave(0);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(0, 12, "Mode Select");

    const uint8_t visibleItems = 3;
    uint8_t startIndex = 0;
    if (NUM_MENU_ITEMS > visibleItems && consoleIndex >= visibleItems) {
      startIndex = consoleIndex - visibleItems + 1;
    }

    for (uint8_t row = 0; row < visibleItems; row++) {
      uint8_t i = startIndex + row;
      if (i >= NUM_MENU_ITEMS) break;

      char buf[16];
      strcpy_P(buf, (PGM_P)pgm_read_word(&menuItems[i]));
      int y = 28 + row * 14;

      if (i == consoleIndex) {
        u8g2.drawBox(0, y - 11, 128, 14);
        u8g2.setDrawColor(0);
        u8g2.drawStr(4, y, buf);
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(4, y, buf);
      }
    }
  } while (u8g2.nextPage());
}

void applyConsoleSelection() {
  // モード切り替えと7セグ更新
  if (consoleIndex == 0) {  // Radio
    exitGpsMode();
    darkMode = false;
    updateSegForRadio();
    OLEDdisplayRadio();

  } else if (consoleIndex == 1) {  // GPS
    enterGpsMode();
    OLEDdisplayGps();

  } else if (consoleIndex == 2) {  // Dark
    exitGpsMode();
    darkMode = true;
    updateSegForDark();
    OLEDdisplayDark();
  }

  // ここで直接描画したので、idle描画は不要
  oledNeedsUpdate = false;
}

// ================== OLED 画面描画 ==================
void OLEDdisplayRadio() {
  if (darkMode) {
    OLEDdisplayDark();
    return;
  }

  u8g2.setPowerSave(0);
  char stationBuf[22];
  getRadioStation(currentFrequency, stationBuf);
  drawRadioScreen(stationBuf, currentVolume, currentFrequency);
}

uint16_t calcDistanceMetersFast(float lat, float lon) {
  float dy = (lat - TARGET_LAT) * LAT_M_PER_DEG;
  float dx = (lon - TARGET_LON) * LON_M_PER_DEG;
  float d2 = dx * dx + dy * dy;
  uint16_t d = 0;
  while ((uint32_t)d * (uint32_t)d < (uint32_t)d2 && d < 9999) d++;
  return d;
}

bool readDataFromGps() {
  bool decoded = false;
  while (Serial.available()) {
    if (gps.encode(Serial.read())) decoded = true;
  }
  return decoded;
}

void updateGpsData() {
  gps.f_get_position(&currentLat, &currentLon, &currentAge);

  hasFix = (currentLat != TinyGPS::GPS_INVALID_F_ANGLE) &&
           (currentLon != TinyGPS::GPS_INVALID_F_ANGLE) &&
           (currentAge != TinyGPS::GPS_INVALID_AGE);

  if (hasFix) {
    distanceMeters = calcDistanceMetersFast(currentLat, currentLon);
    inRange = (distanceMeters <= RADIUS_METERS);
  } else {
    distanceMeters = 9999;
    inRange = false;
  }

  if (gpsMode) {
    updateSegForGps();
  }
}

void processGps() {
  if (!gpsMode || !gpsSerialActive) return;

  if (readDataFromGps()) {
    updateGpsData();
    oledNeedsUpdate = true;
  }
}

void drawGpsScreen() {
  char latBuf[16];
  char lonBuf[16];
  char ageBuf[10];

  if (hasFix) {
    // 小数6桁で表示。1桁は約0.11m相当なのでGPS精度としては十分です。
    dtostrf(currentLat, 10, 6, latBuf);
    dtostrf(currentLon, 10, 6, lonBuf);

    if (currentAge < 10000UL) {
      ultoa(currentAge / 1000UL, ageBuf, 10);
    } else {
      strcpy(ageBuf, ">9");
    }
  } else {
    strcpy(latBuf, "---");
    strcpy(lonBuf, "---");
    strcpy(ageBuf, "-");
  }

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(0, 12, "GPS Position");

    if (!hasFix) {
      u8g2.drawStr(0, 34, "Waiting fix");
      u8g2.drawStr(0, 52, "Lat/Lon: ---");
    } else {
      u8g2.drawStr(0, 28, "LAT");
      u8g2.drawStr(32, 28, latBuf);

      u8g2.drawStr(0, 44, "LON");
      u8g2.drawStr(32, 44, lonBuf);

      u8g2.drawStr(0, 62, "Age");
      u8g2.drawStr(32, 62, ageBuf);
      u8g2.drawStr(48, 62, "s");
    }
  } while (u8g2.nextPage());
}

void OLEDdisplayGps() {
  u8g2.setPowerSave(0);
  drawGpsScreen();
}

void OLEDdisplayDark() {
  u8g2.setPowerSave(1);
}


