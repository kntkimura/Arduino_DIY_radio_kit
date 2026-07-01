// Merged for user request:
// - Based on ArduinoRadio_MultiFunction_6_longpress_darkmode_scrolled(1).ino
// - Sensor functions removed
// - Alarm mode imported/adapted from ArduinoRadio_AlarmClock_scrolled_v3_setdate_longpress.ino
// - In Alarm mode, A1_3 starts standby until the set time, then radio resumes
//
#include <avr/io.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <RDA5807.h>
#include <RtcDS3231.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


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
bool alarmMode = false;   // アラーム設定画面
bool clockMode = false;   // 時計モード
bool darkMode = false;    // ラジオ動作中に表示だけ消すダークモード
bool inConsole = false;   // コンソール画面中か
uint8_t consoleIndex = 0;

uint8_t alarmHour = 7;
uint8_t alarmMinute = 0;
bool alarmEnabled = false;        // EEPROM保存されるアラーム有効/無効
bool alarmStandbyActive = false;  // 指定時刻まで待機中
bool alarmTriggeredToday = false;
uint8_t alarmEditField = 0;  // 0=ON/OFF, 1=hour, 2=minute
uint8_t volumeBeforeAlarm = DEFAULT_VOLUME;
bool radioPoweredDownForAlarm = false;

const uint8_t RTC_INTERRUPT_PIN = 2;  // DS3231 SQW/INT -> D2
volatile bool rtcAlarmInterruptFired = false;
bool mcuSleepArmedForAlarm = false;


// コンソールメニュー
const char menuItem0[] PROGMEM = "Radio";
const char menuItem1[] PROGMEM = "Alarm";
const char menuItem2[] PROGMEM = "Clock";
const char menuItem3[] PROGMEM = "Dark";
const char* const menuItems[] PROGMEM = {
  menuItem0,
  menuItem1,
  menuItem2,
  menuItem3,
};
const uint8_t NUM_MENU_ITEMS =
  sizeof(menuItems) / sizeof(menuItems[0]);


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

// ================== RTC（DS3231 I2C） ==================
RtcDS3231<TwoWire> Rtc(Wire);

// ================== 7セグ (AceSegment) ==================
const uint8_t NUM_DIGITS = 5;
const uint8_t NUM_SEGMENTS = 8;
const uint8_t LATCH_PIN = 10;
const uint8_t DATA_PIN = MOSI;
const uint8_t CLOCK_PIN = SCK;
const uint8_t DIGIT_PINS[NUM_DIGITS] = { 7, 6, 5, 4, 8 };
const uint8_t COLON_PIN = 9;  // 物理コロンLED用（RTC割り込みD3と競合しないよう変更）

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
const int EEPROM_ALARM_ENABLED_ADDR = 4;
const int EEPROM_ALARM_HOUR_ADDR = 5;
const int EEPROM_ALARM_MIN_ADDR = 6;

// ================== プロトタイプ ==================
void setupAceSegment();
void sevenSegmentDisplay();
void getButtonState();

void OLEDdisplayRadio();
void OLEDdisplayAlarm();
void OLEDdisplayClock();
void OLEDdisplayDark();

void drawConsoleScreen();
void applyConsoleSelection();
void readAllReceiverInformation();
void saveAllReceiverInformation();

void updateSegForRadio();
void updateSegForAlarm();
void updateSegForClock();
void updateSegForDark();
void onSeekStep();

void formatFreq(uint16_t f10kHz, char* out, size_t n);
void getRadioStation(uint16_t frequency, char* buffer);
void drawStrWithNewlines(int x, int y, int lineH, const char* s);
void drawRadioScreen(const char* station, uint8_t vol, uint16_t f10kHz);
void armAlarmStandby();
void cancelAlarmStandby(bool keepEnabled = true);
void checkAlarmTrigger();
void enterAlarmHardwareStandby();
void restoreFromAlarmHardwareStandby();
void setupRtcAlarmInterrupt();
void clearRtcAlarmFlags();
void programRtcAlarmForNextTrigger();
void enterMcuPowerDownSleep();
void handleRtcAlarmWake();
void rtcAlarmISR();


void rtcAlarmISR() {
  rtcAlarmInterruptFired = true;
}

void setupRtcAlarmInterrupt() {
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);
  clearRtcAlarmFlags();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcAlarmISR, FALLING);
}

void clearRtcAlarmFlags() {
  rtcAlarmInterruptFired = false;
  Rtc.LatchAlarmsTriggeredFlags();
}

void programRtcAlarmForNextTrigger() {
  RtcDateTime now = Rtc.GetDateTime();
  RtcDateTime target(now.Year(), now.Month(), now.Day(), alarmHour, alarmMinute, 0);

  if (target < now ||
      (target.Year() == now.Year() && target.Month() == now.Month() && target.Day() == now.Day() &&
       target.Hour() == now.Hour() && target.Minute() == now.Minute() && target.Second() <= now.Second())) {
    target = target + 24 * 60 * 60;
  }

  DS3231AlarmOne alarm1(
    target.Day(),
    target.Hour(),
    target.Minute(),
    target.Second(),
    DS3231AlarmOneControl_HoursMinutesSecondsMatch);

  Rtc.SetAlarmOne(alarm1);
  clearRtcAlarmFlags();
}

void enterMcuPowerDownSleep() {
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  if (rtcAlarmInterruptFired) {
    interrupts();
    sleep_disable();
    return;
  }
  interrupts();
  sleep_cpu();
  sleep_disable();
}

void handleRtcAlarmWake() {
  if (!alarmStandbyActive) return;
  if (!rtcAlarmInterruptFired) return;

  DS3231AlarmFlag flags = Rtc.LatchAlarmsTriggeredFlags();
  rtcAlarmInterruptFired = false;

  if (!(flags & DS3231AlarmFlag_Alarm1)) return;

  alarmTriggeredToday = true;
  alarmStandbyActive = false;
  mcuSleepArmedForAlarm = false;
  alarmMode = false;
  darkMode = false;
  restoreFromAlarmHardwareStandby();
  currentFrequency = rx.getFrequency();
  updateSegForRadio();
  OLEDdisplayRadio();
  oledNeedsUpdate = false;
}

// ================== EEPROM 読み書き ==================

void readAllReceiverInformation() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == app_id) {
    currentVolume = EEPROM.read(EEPROM_VOLUME_ADDR);
    currentFrequency = EEPROM.read(EEPROM_FREQ_L_ADDR)
                    | (EEPROM.read(EEPROM_FREQ_H_ADDR) << 8);
    alarmEnabled = EEPROM.read(EEPROM_ALARM_ENABLED_ADDR) ? true : false;
    alarmHour = EEPROM.read(EEPROM_ALARM_HOUR_ADDR);
    alarmMinute = EEPROM.read(EEPROM_ALARM_MIN_ADDR);

    if (currentVolume > 15) currentVolume = DEFAULT_VOLUME;
    if (currentFrequency < 7600 || currentFrequency > 10800) currentFrequency = 8130;
    if (alarmHour > 23) alarmHour = 7;
    if (alarmMinute > 59) alarmMinute = 0;
  } else {
    currentVolume = DEFAULT_VOLUME;
    currentFrequency = 8130;  // 81.3MHz
    alarmEnabled = false;
    alarmHour = 7;
    alarmMinute = 0;
  }
}

void saveAllReceiverInformation() {
  EEPROM.update(EEPROM_MAGIC_ADDR, app_id);
  EEPROM.update(EEPROM_VOLUME_ADDR, rx.getVolume());
  uint16_t f = rx.getFrequency();
  EEPROM.update(EEPROM_FREQ_L_ADDR, (uint8_t)(f & 0xFF));
  EEPROM.update(EEPROM_FREQ_H_ADDR, (uint8_t)(f >> 8));
  EEPROM.update(EEPROM_ALARM_ENABLED_ADDR, alarmEnabled ? 1 : 0);
  EEPROM.update(EEPROM_ALARM_HOUR_ADDR, alarmHour);
  EEPROM.update(EEPROM_ALARM_MIN_ADDR, alarmMinute);

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
  pinMode(COLON_PIN, OUTPUT);
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  digitalWrite(COLON_PIN, LOW);
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

  // RTC 初期化（DS3231）
  Rtc.Begin();

  // 一度だけ時刻合わせ
  //RtcDateTime compiled(__DATE__, __TIME__);
  //Rtc.SetDateTime(compiled + 40 );
  //時刻合わせのための書き込みをしたら上に行をコメントアウトして書き込みをやり直す

  if (!Rtc.GetIsRunning()) {
    Rtc.SetIsRunning(true);
  }

  setupRtcAlarmInterrupt();

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
  handleRtcAlarmWake();
  getButtonState();
  unsigned long now = millis();

  handleRtcAlarmWake();

  if (inConsole) {
    if (consoleNeedsRedraw) {
      drawConsoleScreen();
      consoleNeedsRedraw = false;
    }

  } else if (alarmMode) {
    if (alarmStandbyActive && mcuSleepArmedForAlarm) {
      enterMcuPowerDownSleep();
      handleRtcAlarmWake();
    }

    if (!alarmStandbyActive) {
      static unsigned long lastAlarmDrawTick = 0;
      if (now - lastAlarmDrawTick >= 500) {
        lastAlarmDrawTick = now;
        updateSegForAlarm();
        oledNeedsUpdate = true;
      }
    }
  } else if (clockMode) {

    // 時計モード：1秒ごとに更新
    static unsigned long lastClockDrawTick = 0;
    if (now - lastClockDrawTick >= 1000) {
      lastClockDrawTick = now;
      updateSegForClock();
      oledNeedsUpdate = true;
    }
  } else if (darkMode) {
    // ダークモード：表示は消したまま、ラジオ操作のみ許可
  } else {
    // ラジオモード：7セグはボタン操作時に即更新、
    // OLED は idle 時にまとめて更新する（下の共通処理で描画）
  }

  // ===== OLED 共通遅延描画処理 =====
  const unsigned long OLED_IDLE_MS = 200;
  if (!inConsole && !alarmStandbyActive && oledNeedsUpdate && (millis() - lastUserActionTime >= OLED_IDLE_MS)) {

    if (alarmMode) {
      OLEDdisplayAlarm();
    } else if (clockMode) {
      OLEDdisplayClock();
    } else if (darkMode) {
      OLEDdisplayDark();
    } else {
      OLEDdisplayRadio();
    }
    oledNeedsUpdate = false;
  }

  // ===== EEPROM 遅延セーブ（最後の操作から5秒後に1回だけ） =====
  if (eepromSavePending && (millis() - lastUserActionTime >= 5000)) {
    saveAllReceiverInformation();
    eepromSavePending = false;
  }

  // コロン点灯の自動消灯（操作がなくても1秒で消す）
  if (!alarmMode && !clockMode) {  // ラジオ/ダークモード中だけ
    if (digit4state && (millis() - colonStartMs >= 1000UL)) {
      colonStartMs = millis();
      digit4state = false;
      updateSegForRadio();  // 消灯を表示に反映
    }
  }
}

// ================== 7セグ描画（ISR側は「表示だけ」） ==================
void sevenSegmentDisplay() {
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

// Alarm モード用 digit 設定（HH:MM）
void updateSegForAlarm() {
  noInterrupts();
  segDigits[0] = alarmHour / 10;
  segDigits[1] = alarmHour % 10;
  segDigits[2] = alarmMinute / 10;
  segDigits[3] = alarmMinute % 10;
  segDigits[4] = 22;
  interrupts();

  digitalWrite(COLON_PIN, LOW);
}

// Clock モード用 digit 設定
void updateSegForClock() {
  RtcDateTime now = Rtc.GetDateTime();
  uint8_t hh = now.Hour();
  uint8_t mm = now.Minute();

  noInterrupts();
  segDigits[0] = hh / 10;
  segDigits[1] = hh % 10;  // ドット無し
  segDigits[2] = mm / 10;
  segDigits[3] = mm % 10;
  segDigits[4] = 22;  // コロン（上下2ドットパターン）
  interrupts();

  digitalWrite(COLON_PIN, LOW);
}

void updateSegForDark() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    segDigits[i] = 27;
  }
  interrupts();

  digitalWrite(COLON_PIN, LOW);
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

  // ===== 通常（ラジオ操作）は Radio/Dark/clockMode モードのときだけ =====
  if (!alarmMode) {
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

  if (alarmMode) {
    if (trigA0_1) {
      if (alarmStandbyActive) {
        cancelAlarmStandby(true);
      }
      if (alarmEditField == 0) {
        alarmEnabled = !alarmEnabled;
      } else if (alarmEditField == 1) {
        alarmHour = (alarmHour + 23) % 24;
      } else {
        alarmMinute = (alarmMinute + 59) % 60;
      }
      updateSegForAlarm();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA0_2) {
      if (alarmStandbyActive) {
        cancelAlarmStandby(true);
      }
      if (alarmEditField == 0) {
        alarmEnabled = !alarmEnabled;
      } else if (alarmEditField == 1) {
        alarmHour = (alarmHour + 1) % 24;
      } else {
        alarmMinute = (alarmMinute + 1) % 60;
      }
      updateSegForAlarm();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA1_1) {
      alarmEditField = (alarmEditField + 2) % 3;
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA1_2) {
      alarmEditField = (alarmEditField + 1) % 3;
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA1_3 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      if (alarmStandbyActive) {
        cancelAlarmStandby(true);
      } else if (alarmEnabled) {
        armAlarmStandby();
      }
      updateSegForAlarm();
      oledNeedsUpdate = true;
      anyPressed = true;

    } else if (trigA1_4 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      if (alarmStandbyActive) {
        cancelAlarmStandby(true);
      }
      alarmMode = false;
      updateSegForRadio();
      OLEDdisplayRadio();
      oledNeedsUpdate = false;
      anyPressed = true;
    }
  } else {
    // コンソール起動（Alarm画面以外）
    if (trigA1_3 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      inConsole = true;
      consoleNeedsRedraw = true;  // 入った瞬間だけ描画
      anyPressed = true;
    }
  }

  // 何かボタンが押されたら「最後の操作時刻」を更新
  if (anyPressed) {
    lastUserActionTime = millis();
    if (!clockMode) {
      eepromSavePending = true;
    }
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
    cancelAlarmStandby(true);
    alarmMode = false;
    clockMode = false;
    darkMode = false;
    updateSegForRadio();
    OLEDdisplayRadio();

  } else if (consoleIndex == 1) {  // Alarm
    alarmMode = true;
    clockMode = false;
    darkMode = false;
    updateSegForAlarm();
    OLEDdisplayAlarm();

  } else if (consoleIndex == 2) {  // Clock
    cancelAlarmStandby(true);
    alarmMode = false;
    clockMode = true;
    darkMode = false;
    updateSegForClock();
    OLEDdisplayClock();

  } else if (consoleIndex == 3) {  // Dark
    cancelAlarmStandby(true);
    alarmMode = false;
    clockMode = false;
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

void enterAlarmHardwareStandby() {
  rx.setMute(true);
  rx.setAudioOutputHighImpedance(true);
  rx.powerDown();
  radioPoweredDownForAlarm = true;

  // 先に表示内容を blank にする
  updateSegForDark();

  // blank を実表示へ数回反映させる
  for (uint8_t n = 0; n < 10; n++) {
    sevenSegmentDisplay();
    delay(2);
  }

  // その後で表示を止める
  ledModule.setBrightness(0);
  u8g2.setPowerSave(1);
}

void restoreFromAlarmHardwareStandby() {
  if (!radioPoweredDownForAlarm) return;

  // ラジオICを復帰して、直前の状態へ戻す。
  rx.powerUp();
  delay(120);
  rx.setBand(RDA_FM_BAND_WORLD);
  rx.setFrequency(currentFrequency);
  rx.setAudioOutputHighImpedance(false);
  rx.setMute(false);
  rx.setVolume(volumeBeforeAlarm > 0 ? volumeBeforeAlarm : DEFAULT_VOLUME);
  currentFrequency = rx.getFrequency();
  currentVolume = rx.getVolume();
  radioPoweredDownForAlarm = false;

  // 表示系を再開。
  u8g2.setPowerSave(0);
  ledModule.setBrightness(15);
}

void armAlarmStandby() {
  if (!alarmEnabled) return;
  alarmStandbyActive = true;
  alarmTriggeredToday = false;
  volumeBeforeAlarm = currentVolume;
  programRtcAlarmForNextTrigger();
  enterAlarmHardwareStandby();
  mcuSleepArmedForAlarm = true;
}

void cancelAlarmStandby(bool keepEnabled) {
  bool wasStandbyActive = alarmStandbyActive;
  alarmStandbyActive = false;
  alarmTriggeredToday = false;
  mcuSleepArmedForAlarm = false;
  if (!keepEnabled) alarmEnabled = false;

  clearRtcAlarmFlags();

  if (wasStandbyActive) {
    restoreFromAlarmHardwareStandby();
  }
  currentVolume = rx.getVolume();
}

void checkAlarmTrigger() {
  handleRtcAlarmWake();
}

void OLEDdisplayAlarm() {
  u8g2.setPowerSave(0);
  char buf[24];
  RtcDateTime now = Rtc.GetDateTime();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);

    if (alarmStandbyActive) {
      u8g2.drawStr(0, 12, "Alarm standby");
    } else {
      u8g2.drawStr(0, 12, "Set Alarm");
    }

    snprintf(buf, sizeof(buf), "Now   %02u:%02u", now.Hour(), now.Minute());
    u8g2.drawStr(0, 28, buf);

    snprintf(buf, sizeof(buf), "%s %02u:%02u", alarmEnabled ? "ON " : "OFF", alarmHour, alarmMinute);
    u8g2.drawStr(20, 50, buf);

    // 選択マーク
    if (alarmEditField == 0) {
      u8g2.drawTriangle(30, 41, 26, 36, 35, 36);
    } else if (alarmEditField == 1) {
      u8g2.drawTriangle(60, 41, 56, 36, 65, 36);
    } else {
      u8g2.drawTriangle(83, 41, 79, 36, 88, 36);
    }

    if (alarmStandbyActive) {
      u8g2.drawStr(0, 63, "A1_3:Stop  A1_4:Exit");
    } else {
      u8g2.drawStr(0, 63, "A1_3:Go A1_4:Exit");
    }
  } while (u8g2.nextPage());
}

void OLEDdisplayDark() {
  u8g2.setPowerSave(1);
}

void OLEDdisplayClock() {
  u8g2.setPowerSave(0);
  char buf[24];

  RtcDateTime now = Rtc.GetDateTime();
  uint8_t hh = now.Hour();
  uint8_t mm = now.Minute();
  uint8_t ss = now.Second();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", hh, mm, ss);
    int w = u8g2.getStrWidth(buf);
    u8g2.drawStr((128 - w) / 2, 24, buf);

    snprintf(buf, sizeof(buf), "%04u-%02u-%02u",
             now.Year(), now.Month(), now.Day());
    w = u8g2.getStrWidth(buf);
    u8g2.drawStr((128 - w) / 2, 44, buf);

  } while (u8g2.nextPage());
}
