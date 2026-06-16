/*
  Arduino Radio
  Copyright (c) 2026 Kent Kimura

  Released under the MIT License.
  https://opensource.org/licenses/MIT

  --------------------------------------------------------------------
  Hardware Notes / ハードウェアについて
  --------------------------------------------------------------------

  English:

  This project is designed for an ATmega328P-PU running from the
  internal 8 MHz RC oscillator without an external crystal.

  During development, the internal clock configuration produced
  lower RF noise and better FM reception than some external crystal
  configurations.

  ICSP programming is recommended.

  Recommended Arduino IDE settings:

    Board Package:
      ATtiny/ATmega Standalone

    Board:
      ATmega328P (ISP)(Int. 8 MHz)(Shift!)

    Programmer:
      Arduino as ISP

  Japanese:

  本プロジェクトはATmega328P-PUの内部8MHz RCクロックでの
  動作を前提として設計されています。

  開発時の検証では、外部クリスタルを使用しない構成の方が
  FMラジオ受信時のノイズが少なく、良好な受信性能が
  得られました。

  書き込みはICSP方式を推奨します。

  推奨Arduino IDE設定:

    ボードパッケージ:
      ATtiny/ATmega Standalone

    ボード:
      ATmega328P (ISP)(Int. 8 MHz)(Shift!)

    書き込み装置:
      Arduino as ISP

  --------------------------------------------------------------------
  Features
  --------------------------------------------------------------------

  - FM Radio (RDA5807)
  - OLED Display (U8g2)
  - 4-Digit 7-Segment Display (AceSegment)
  - Analog Multi-Button Input
  - Snake Game
  - Dark Mode
  - Internal RC Oscillator Operation
  - TimerOne Display Refresh

  --------------------------------------------------------------------
  Libraries
  --------------------------------------------------------------------

  - U8g2
  - RDA5807
  - AceSegment
  - TimerOne

*/

// Version 1.0
// Author: Kent Kimura
// Project: Arduino Radio

#include <avr/io.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <RDA5807.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>


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
#include <TimerOne.h>

using ace_segment::HybridModule;
using ace_segment::kActiveHighPattern;
using ace_spi::HardSpiInterface;

// ================== OLED (SSD1306 128x64) ==================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// ================== RDA5807 ラジオ ==================
RDA5807 rx;
#define DEFAULT_VOLUME 3
#define STORE_TIME 100  // 単発ボタン用の最小間隔(ms)
#define LONG_PRESS_START 420     // 長押し連続開始までの時間(ms)
#define LONG_PRESS_INTERVAL 90   // 長押し連続送りの間隔(ms)

const uint8_t app_id = 50;  // EEPROM用識別ID
const int eeprom_address = 0;

uint16_t currentFrequency = 0;  // 10kHz単位 (例: 8970 = 89.7MHz)
uint16_t currentVolume = DEFAULT_VOLUME;

// スネークモード中のラジオ電源管理
// ゲームに入った瞬間はRDA5807をPower Down。
// ゲーム中はA1_1だけでラジオON/OFFを切り替える。
bool radioPoweredDownForGame = false;
bool gameRadioToggleLockedUntilRelease = false;

// スネークのフルーツ取得音
// A1_2でON/OFF切り替え。ONの時だけフルーツ取得時に「ピロッ♪」を鳴らす。
const uint8_t BEEP_PIN = A2;
bool fruitSoundEnabled = true;
bool fruitSoundToggleLockedUntilRelease = false;

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
bool gameMode = false;    // スネークゲーム画面
bool darkMode = false;    // ラジオ動作中に表示だけ消すダークモード
bool inConsole = false;   // コンソール画面中か
uint8_t consoleIndex = 0;

// コンソールメニュー
const char menuItem0[] PROGMEM = "Radio";
const char menuItem1[] PROGMEM = "Snake";
const char menuItem2[] PROGMEM = "Dark";
const char* const menuItems[] PROGMEM = {
  menuItem0,
  menuItem1,
  menuItem2,
};
const uint8_t NUM_MENU_ITEMS =
  sizeof(menuItems) / sizeof(menuItems[0]);

// ================== スネークゲーム ==================
const uint8_t CELL = 4;                  // 1マス 4x4px
const uint8_t GW = 128 / CELL;           // 32
const uint8_t GH = 64 / CELL;            // 16
const uint8_t MAX_SNAKE = 96;            // SRAM節約
const uint8_t CLEAR_LEN = 20;            // ここまで伸びたらクリア
const uint8_t FOOD_MARGIN = 1;

uint8_t sx[MAX_SNAKE], sy[MAX_SNAKE];
uint8_t slen = 0;
int8_t dx = 1, dy = 0;
uint8_t foodX = 10, foodY = 8;
bool gameOver = false;
bool gameClear = false;
bool paused = false;
uint8_t lastGameDirButton = 0;  // BTN_NONE相当。定義前なので0で初期化
unsigned long lastTick = 0;
uint16_t tickMs = 140;
unsigned long lastGameDraw = 0;
const unsigned long GAME_DRAW_INTERVAL_MS = 80;  // OLED描画負荷を下げ、速度の揺れを抑える

// ================== ボタン誤作動対策 ==================
// 抵抗分圧の値が境界付近で揺れると誤作動するため、
// 1) 複数回ADCを読んで平均化
// 2) ボタン判定に「無効帯」を入れる
// 3) 同じ判定が連続して一定時間続いた時だけ確定
// の3段構えにしています。
const uint8_t BTN_NONE = 0;
const uint8_t BTN_1 = 1;
const uint8_t BTN_2 = 2;
const uint8_t BTN_3 = 3;
const uint8_t BTN_4 = 4;

const unsigned long BUTTON_DEBOUNCE_MS = 18;  // ラジオ通常操作用
const unsigned long GAME_BUTTON_DEBOUNCE_MS = 0;  // スネーク中は即反応。方向キーの取りこぼしを減らす

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

// SEEKボタン専用の二重入力防止。
// rx.seek() は処理に時間がかかるため、処理完了後も同じ押下が残って
// 「長押しリピート」として再発火することがある。
// そのため SEEK UP/DOWN だけは、押した瞬間に1回だけ実行し、
// いったんボタンを離すまで次のSEEKを受け付けない。
bool seekUpLockedUntilRelease = false;
bool seekDownLockedUntilRelease = false;
unsigned long seekIgnoreUntilMs = 0;
const unsigned long SEEK_AFTER_IGNORE_MS = 220;

void resetButtonRepeatState(uint8_t index) {
  buttonPrevState[index] = false;
  buttonPressStart[index] = 0;
  buttonLastRepeat[index] = 0;
}

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
void OLEDdisplayGame();
void OLEDdisplayDark();

void drawConsoleScreen();
void applyConsoleSelection();
void readAllReceiverInformation();
void saveAllReceiverInformation();

void updateSegForRadio();
void updateSegForGame();
void updateSegForDark();
void applySegDigitsToModule();
void blankSevenSegmentNow();
void enterGameMode();
void exitGameMode();
void radioOffForGame();
void radioOnFromGameButton();
void onSeekStep();

void formatFreq(uint16_t f10kHz, char* out);
void getRadioStation(uint16_t frequency, char* buffer);
void drawStrWithNewlines(int x, int y, int lineH, const char* s);
void drawRadioScreen(const char* station, uint8_t vol, uint16_t f10kHz);
void gamePlaceFood();
void gameReset();
bool gameStep();
void gameUpdate();
void gameDraw();
uint16_t readAnalogAveraged(uint8_t pin);
uint8_t classifyAnalogButton(uint16_t v);
uint8_t stableButton(uint8_t channelIndex, uint8_t raw, unsigned long now);


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
void formatFreq(uint16_t f10kHz, char* out) {
  uint8_t mhz = f10kHz / 100;
  out[0] = (mhz >= 100) ? '1' : ' ';
  out[1] = char('0' + (mhz / 10) % 10);
  out[2] = char('0' + mhz % 10);
  out[3] = '.';
  out[4] = char('0' + ((f10kHz % 100) / 10));
  out[5] = '\0';
}

// ================== ラジオ画面描画 ==================
void drawRadioScreen(const char* station, uint8_t vol, uint16_t f10kHz) {
  char fbuf[8];
  char vbuf[4];
  formatFreq(f10kHz, fbuf);

  u8g2.firstPage();
  do {
    // 周波数（大数字）
    u8g2.setFont(u8g2_font_logisoso16_tn);  // 数字専用・軽量
    int fw = u8g2.getStrWidth(fbuf);
    u8g2.drawStr(0, 18, fbuf);

    // "MHz"
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(fw + 2, 18, "MHz");

    // 局名（大きめフォント）
    u8g2.setFont(u8g2_font_10x20_tf);
    int sw = u8g2.getStrWidth(station);
    int sx = (128 - sw) / 2;
    drawStrWithNewlines(sx, 42, 20, station);

    // Vol
    u8g2.setFont(u8g2_font_8x13_tr);
    u8g2.drawStr(80, 64, "Vol");
    u8g2.setFont(u8g2_font_logisoso16_tn);
    vbuf[0] = (vol >= 10) ? char('0' + vol / 10) : ' ';
    vbuf[1] = char('0' + vol % 10);
    vbuf[2] = '\0';
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
  pinMode(BEEP_PIN, OUTPUT);
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

  // スネークゲーム用乱数
  randomSeed(analogRead(A3));

  // OLED
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFlipMode(0);

  // AceSegment
  setupAceSegment();
  ledModule.setBrightness(15);
  // 7セグの多重表示はTimer1へ移動。tone()はTimer2を使うため衝突しない。
  // 200usはこの内部クロック環境でチラつきが少なかった値。必要なら250〜500も試す。
  Timer1.initialize(50);
  Timer1.attachInterrupt(sevenSegmentDisplay);

  // 初期表示はラジオ
  updateSegForRadio();
  OLEDdisplayRadio();
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
  } else if (gameMode) {
    gameUpdate();
    // スネーク中は「操作後200ms待ってから描画」の共通処理を使わない。
    // ここで一定周期描画にすることで、押している間に画面更新が止まる症状を防ぐ。
    if (now - lastGameDraw >= GAME_DRAW_INTERVAL_MS) {
      lastGameDraw = now;
      OLEDdisplayGame();
      oledNeedsUpdate = false;
    }
  } else if (darkMode) {
    // ダークモード：表示は消したまま、ラジオ操作のみ許可
  } else {
    // ラジオモード
  }

  // ===== OLED 共通遅延描画処理 =====
  const unsigned long OLED_IDLE_MS = 200;
  if (!inConsole && !gameMode && oledNeedsUpdate && (millis() - lastUserActionTime >= OLED_IDLE_MS)) {
    if (darkMode) {
      OLEDdisplayDark();
    } else {
      OLEDdisplayRadio();
    }
    oledNeedsUpdate = false;
  }

  // ===== EEPROM 遅延セーブ（最後の操作から5秒後に1回だけ） =====
  if (!gameMode && eepromSavePending && (millis() - lastUserActionTime >= 5000)) {
    saveAllReceiverInformation();
    eepromSavePending = false;
  }

  // コロン点灯の自動消灯（操作がなくても1秒で消す）
  if (!gameMode) {
    if (digit4state && (millis() - colonStartMs >= 1000UL)) {
      colonStartMs = millis();
      digit4state = false;
      updateSegForRadio();
    }
  }
}

// ================== 7セグ描画（ISR側は「表示だけ」） ==================
void sevenSegmentDisplay() {
  // ISR内では桁の多重表示だけ行う。
  // setPatternAt()をここで毎回呼ぶと割り込み処理が重くなり、
  // ボタン二度押し・チラつき・ラジオノイズの原因になる。
  ledModule.renderFieldWhenReady();
}

void applySegDigitsToModule() {
  // 表示内容が変わった時だけloop側からAceSegmentへ反映する。
  // renderFieldWhenReady()と同時に内部バッファを書き換えないよう、
  // 短時間だけ割り込みを止める。
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    uint8_t d = segDigits[i];
    if (d > 27) d = 27;
    ledModule.setPatternAt(i, PATTERNS[d]);
  }
  interrupts();
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
  applySegDigitsToModule();
}

// ゲームモード用 digit 設定
void updateSegForGame() {
  noInterrupts();
  // 7セグにはスネークのポイントを 3桁 + apostrophe + P で表示。例: 004'P
  segDigits[0] = (slen / 100) % 10;
  segDigits[1] = (slen / 10) % 10;
  segDigits[2] = slen % 10;
  segDigits[3] = 26;  // P
  segDigits[4] = 23;  // '  ※物理的にはコロン/アポストロフィ位置
  interrupts();
  applySegDigitsToModule();
}

void updateSegForDark() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    segDigits[i] = 27;
  }
  interrupts();
  applySegDigitsToModule();
}

void blankSevenSegmentNow() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    segDigits[i] = 27;
  }
  interrupts();

  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    ledModule.setPatternAt(i, PATTERNS[27]);
  }
  for (uint8_t i = 0; i < 8; i++) {
    ledModule.renderFieldWhenReady();
  }
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    pinMode(DIGIT_PINS[i], OUTPUT);
    digitalWrite(DIGIT_PINS[i], LOW);
  }
}

void radioOffForGame() {
  // スネーク開始時はラジオをOFF。
  // 復帰時に同じ周波数・音量へ戻せるように現在値を保持する。
  if (!radioPoweredDownForGame) {
    currentFrequency = rx.getFrequency();
    currentVolume = rx.getVolume();
    rx.setVolume(0);
    rx.powerDown();
    radioPoweredDownForGame = true;
  }
}

void radioOnFromGameButton() {
  // ゲーム中、A1_1操作でラジオを復帰する。
  // 既に復帰済みなら何もしない。
  if (radioPoweredDownForGame) {
    rx.powerUp();
    rx.setBand(RDA_FM_BAND_WORLD);
    rx.setFrequency(currentFrequency);
    rx.setVolume(currentVolume);
    currentFrequency = rx.getFrequency();
    currentVolume = rx.getVolume();
    radioPoweredDownForGame = false;
  }
}

void playFruitGetSound() {
  if (!fruitSoundEnabled) return;
  // フルーツ取得時の短い「ピロッ♪」。短時間だけ待って2音目へつなぐ。
  tone(BEEP_PIN, 2000, 15);
  tone(BEEP_PIN, 2800, 22);
}

void playFruitSoundToggleBeep() {
  // A1_2で効果音ON/OFFを切り替えた時の確認音。
  if (fruitSoundEnabled) {
    tone(BEEP_PIN, 3000, 30);  // ON: 高め
  } else {
    tone(BEEP_PIN, 1200, 50);  // OFF: 低め
  }
}

void enterGameMode() {
  gameMode = true;
  darkMode = false;
  gameRadioToggleLockedUntilRelease = false;
  fruitSoundToggleLockedUntilRelease = false;
  radioOffForGame();
  gameReset();
  updateSegForGame();
  oledNeedsUpdate = true;
}

void exitGameMode() {
  // ゲーム終了してラジオモードへ戻る時は復帰する。
  gameRadioToggleLockedUntilRelease = false;
  fruitSoundToggleLockedUntilRelease = false;
  radioOnFromGameButton();
  gameMode = false;
  paused = false;
  updateSegForRadio();
  oledNeedsUpdate = true;
}

// ================== AceSegment 初期化 ==================
void setupAceSegment() {
  spiInstance.begin();
  spiInterface.begin();
  ledModule.begin();
}

// ================== ボタン処理 ==================
uint16_t readAnalogAveraged(uint8_t pin) {
  uint16_t sum = 0;
  // 1回目はMUX切替直後の捨て読み
  analogRead(pin);

  // スネーク中は入力取りこぼしを避けるため軽めに読む。
  // ラジオ中は従来通り少し平均化して誤作動を抑える。
  uint8_t samples = gameMode ? 1 : 4;
  uint8_t waitUs = gameMode ? 0 : 120;

  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(waitUs);
  }
  return sum / samples;
}

uint8_t classifyAnalogButton(uint16_t v) {
  // 通常操作用。代表値: 0 / 510 / 679 / 768 / 無押下1023
  // ラジオ操作では誤作動防止を優先して、境界に少し無効帯を残す。
  if (v < 190) return BTN_1;
  if (v >= 370 && v <= 605) return BTN_2;
  if (v >= 610 && v <= 735) return BTN_3;
  if (v >= 740 && v <= 875) return BTN_4;
  return BTN_NONE;
}

uint8_t classifyAnalogButtonGame(uint16_t v) {
  // スネーク専用。短いタップを拾うため、無効帯をほぼ無くして敏感にする。
  // 中間点で区切る: 0 / 510 / 679 / 768 / 1023
  if (v < 260) return BTN_1;
  if (v < 595) return BTN_2;
  if (v < 724) return BTN_3;
  if (v < 900) return BTN_4;
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

  unsigned long debounceMs = gameMode ? GAME_BUTTON_DEBOUNCE_MS : BUTTON_DEBOUNCE_MS;
  if ((now - changedAt[channelIndex]) >= debounceMs) {
    stable[channelIndex] = raw;
  }

  return stable[channelIndex];
}

// ================== ボタン処理 ==================
void getButtonState() {
  unsigned long now = millis();

  uint16_t a0v = readAnalogAveraged(A0);
  uint16_t a1v = readAnalogAveraged(A1);

  uint8_t a0raw = gameMode ? classifyAnalogButtonGame(a0v) : classifyAnalogButton(a0v);
  uint8_t a1raw = gameMode ? classifyAnalogButtonGame(a1v) : classifyAnalogButton(a1v);

  // スネーク中は方向キーの短いタップを拾うため、デバウンス確定を待たず即反映。
  // ラジオ中は従来通りstableButton()で誤作動を抑える。
  uint8_t a0b = gameMode ? a0raw : stableButton(0, a0raw, now);
  uint8_t a1b = gameMode ? a1raw : stableButton(1, a1raw, now);

  bool a0_1 = (a0b == BTN_1);  // 周波数↓ / ゲーム上
  bool a0_2 = (a0b == BTN_2);  // 周波数↑ / ゲーム右
  bool a0_3 = (a0b == BTN_3);  // シークUP / ゲーム下
  bool a0_4 = (a0b == BTN_4);  // シークDOWN / ゲーム左

  bool a1_1 = (a1b == BTN_1);  // 音量↓
  bool a1_2 = (a1b == BTN_2);  // 音量↑
  bool a1_3 = (a1b == BTN_3);  // コンソール起動 / 決定 / ゲームPAUSE
  bool a1_4 = (a1b == BTN_4);  // キャンセル / ゲーム終了

  // SEEKボタンは「離した」ことを確認してから次回を許可する。
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
      storeTime = now;
      applyConsoleSelection();
      inConsole = false;
      anyPressed = true;

    } else if (trigA1_4 && (now - storeTime) > STORE_TIME) {
      storeTime = now;
      inConsole = false;
      anyPressed = true;
      oledNeedsUpdate = true;
    }

    if (anyPressed) lastUserActionTime = now;
    return;
  }

  // ===== ゲーム中 =====
  if (gameMode) {
    // ゲーム中はA0方向キーではラジオを起動しない。
    // A1_1だけでラジオON/OFFをトグルする。
    // 押しっぱなしで連続トグルしないよう、離すまでロックする。
    if (!a1_1) {
      gameRadioToggleLockedUntilRelease = false;
    }
    if (a1_1 && !gameRadioToggleLockedUntilRelease) {
      gameRadioToggleLockedUntilRelease = true;
      if (radioPoweredDownForGame) {
        radioOnFromGameButton();
      } else {
        radioOffForGame();
      }
      anyPressed = true;
      lastUserActionTime = now;
    }

    // A1_2でフルーツ取得音のON/OFFを切り替える。
    if (!a1_2) {
      fruitSoundToggleLockedUntilRelease = false;
    }
    if (a1_2 && !fruitSoundToggleLockedUntilRelease) {
      fruitSoundToggleLockedUntilRelease = true;
      fruitSoundEnabled = !fruitSoundEnabled;
      playFruitSoundToggleBeep();
      oledNeedsUpdate = true;
      anyPressed = true;
      lastUserActionTime = now;
    }

    // スネークの方向操作は、平均化・デバウンスを最小にして即反映。
    // 速度安定化はgameUpdate()側で行い、入力はここでできるだけ取りこぼさない。
    uint8_t dirButton = BTN_NONE;
    if (a0_1) dirButton = BTN_1;       // 上
    else if (a0_2) dirButton = BTN_2;  // 右
    else if (a0_3) dirButton = BTN_3;  // 下
    else if (a0_4) dirButton = BTN_4;  // 左

    // スネーク中は、別方向に変わった瞬間も新しい入力として即受け付ける。
    bool dirPressedNow = (dirButton != BTN_NONE && dirButton != lastGameDirButton);
    lastGameDirButton = dirButton;

    bool directionAccepted = false;
    if (dirPressedNow) {
      if (dirButton == BTN_1 && dy !=  1) { dx =  0; dy = -1; directionAccepted = true; } // 上
      else if (dirButton == BTN_2 && dx != -1) { dx =  1; dy =  0; directionAccepted = true; } // 右
      else if (dirButton == BTN_3 && dy != -1) { dx =  0; dy =  1; directionAccepted = true; } // 下
      else if (dirButton == BTN_4 && dx !=  1) { dx = -1; dy =  0; directionAccepted = true; } // 左
    }

    if (directionAccepted) {
      // 方向ボタンを押した瞬間に1マス進める。
      // その直後に通常tickが重なって二重移動しないよう、基準時刻もリセットする。
      if (!gameOver && !gameClear && !paused) {
        gameStep();
        lastTick = now;
        OLEDdisplayGame();   // 押した瞬間の移動をすぐ画面に出す
        lastGameDraw = now;
      }
      updateSegForGame();
      anyPressed = true;
    }

    if (trigA1_3 && (now - storeTime) > 200) {
      storeTime = now;
      if (gameOver || gameClear) gameReset();
      else paused = !paused;
      updateSegForGame();
      oledNeedsUpdate = true;
      anyPressed = true;
    }

    if (trigA1_4 && (now - storeTime) > 200) {
      storeTime = now;
      exitGameMode();
      anyPressed = true;
    }

    if (anyPressed) {
      lastUserActionTime = now;
      oledNeedsUpdate = true;
    }
    return;
  }

  // ===== 通常のラジオ操作 =====
  if (trigA0_1) {
    rx.setFrequencyDown();
    currentFrequency = rx.getFrequency();
    updateSegForRadio();
    oledNeedsUpdate = true;
    anyPressed = true;

  } else if (trigA0_2) {
    rx.setFrequencyUp();
    currentFrequency = rx.getFrequency();
    updateSegForRadio();
    oledNeedsUpdate = true;
    anyPressed = true;

  } else if (a0_3 && !seekUpLockedUntilRelease && (now >= seekIgnoreUntilMs) && (now - storeTime) > STORE_TIME) {
    storeTime = now;
    seekUpLockedUntilRelease = true;
    resetButtonRepeatState(BTN_A0_3);

    seekup = true;
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStep);
    seekup = false;

    currentFrequency = rx.getFrequency();
    updateSegForRadio();
    oledNeedsUpdate = true;
    anyPressed = true;
    seekIgnoreUntilMs = millis() + SEEK_AFTER_IGNORE_MS;

  } else if (a0_4 && !seekDownLockedUntilRelease && (now >= seekIgnoreUntilMs) && (now - storeTime) > STORE_TIME) {
    storeTime = now;
    seekDownLockedUntilRelease = true;
    resetButtonRepeatState(BTN_A0_4);

    seekdown = true;
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStep);
    seekdown = false;

    currentFrequency = rx.getFrequency();
    updateSegForRadio();
    oledNeedsUpdate = true;
    anyPressed = true;
    seekIgnoreUntilMs = millis() + SEEK_AFTER_IGNORE_MS;
  }

  if (trigA1_1) {
    rx.setVolumeDown();
    currentVolume = rx.getVolume();
    updateSegForRadio();
    oledNeedsUpdate = true;
    anyPressed = true;

  } else if (trigA1_2) {
    rx.setVolumeUp();
    currentVolume = rx.getVolume();
    updateSegForRadio();
    oledNeedsUpdate = true;
    anyPressed = true;
  }

  // コンソール起動
  if (trigA1_3 && (now - storeTime) > STORE_TIME) {
    storeTime = now;
    inConsole = true;
    consoleNeedsRedraw = true;
    anyPressed = true;
  }

  if (anyPressed) {
    lastUserActionTime = now;
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
    exitGameMode();
    darkMode = false;
    updateSegForRadio();
    OLEDdisplayRadio();

  } else if (consoleIndex == 1) {  // Snake
    enterGameMode();
    OLEDdisplayGame();

  } else if (consoleIndex == 2) {  // Dark
    exitGameMode();
    darkMode = true;
    updateSegForDark();
    OLEDdisplayDark();
  }

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

void gamePlaceFood() {
  uint8_t xmin = FOOD_MARGIN;
  uint8_t xmax = GW - 1 - FOOD_MARGIN;
  uint8_t ymin = FOOD_MARGIN + 3;  // 上部スコア表示を避ける
  uint8_t ymax = GH - 1 - FOOD_MARGIN;

  for (uint16_t tries = 0; tries < 200; tries++) {
    uint8_t fx = random(xmin, (uint8_t)(xmax + 1));
    uint8_t fy = random(ymin, (uint8_t)(ymax + 1));

    bool hit = false;
    for (uint8_t i = 0; i < slen; i++) {
      if (sx[i] == fx && sy[i] == fy) { hit = true; break; }
    }
    if (!hit) { foodX = fx; foodY = fy; return; }
  }

  for (uint8_t y = ymin; y <= ymax; y++) {
    for (uint8_t x = xmin; x <= xmax; x++) {
      bool hit = false;
      for (uint8_t i = 0; i < slen; i++) {
        if (sx[i] == x && sy[i] == y) { hit = true; break; }
      }
      if (!hit) { foodX = x; foodY = y; return; }
    }
  }
}

void gameReset() {
  slen = 4;
  uint8_t cx = GW / 2;
  uint8_t cy = GH / 2;
  for (uint8_t i = 0; i < slen; i++) {
    sx[i] = cx - i;
    sy[i] = cy;
  }
  dx = 1;
  dy = 0;
  gameOver = false;
  gameClear = false;
  paused = false;
  lastGameDirButton = BTN_NONE;
  tickMs = 140;
  gamePlaceFood();
  lastTick = millis();
  lastGameDraw = 0;
}

bool gameStep() {
  if (gameOver || gameClear || paused) return false;

  int nx = (int)sx[0] + dx;
  int ny = (int)sy[0] + dy;

  if (nx < 0 || nx >= GW || ny < 0 || ny >= GH) {
    gameOver = true;
    return true;
  }

  bool grow = ((uint8_t)nx == foodX && (uint8_t)ny == foodY);
  uint8_t oldTailX = sx[slen - 1];
  uint8_t oldTailY = sy[slen - 1];

  for (int i = slen - 1; i > 0; i--) {
    sx[i] = sx[i - 1];
    sy[i] = sy[i - 1];
  }
  sx[0] = (uint8_t)nx;
  sy[0] = (uint8_t)ny;

  for (uint8_t i = 1; i < slen; i++) {
    if (sx[0] == sx[i] && sy[0] == sy[i]) {
      gameOver = true;
      return true;
    }
  }

  if (grow) {
    if (slen < MAX_SNAKE) {
      sx[slen] = oldTailX;
      sy[slen] = oldTailY;
      slen++;
    }

    playFruitGetSound();

    if (slen >= CLEAR_LEN) {
      gameClear = true;
      updateSegForGame();
      return true;
    }

    gamePlaceFood();
    if (tickMs > 60) tickMs -= 4;
    updateSegForGame();
  }

  return true;
}

void gameUpdate() {
  if (gameOver || gameClear || paused) return;

  unsigned long now = millis();
  if (now - lastTick < tickMs) return;

  // lastTick = now にすると、OLED描画などでループが遅れた分だけ
  // 次の移動周期も後ろへずれて、スネークの速度が不安定に見える。
  // 予定時刻に tickMs を足す方式にして、内部クロックでも周期を安定させる。
  lastTick += tickMs;
  if (now - lastTick > tickMs) {
    // 大きく遅れた場合だけ破綻防止で現在時刻に寄せる。
    lastTick = now;
  }

  gameStep();
}

void gameDraw() {
  u8g2.setPowerSave(0);
  u8g2.firstPage();
  do {
    if (gameOver || gameClear) {
      u8g2.setFont(u8g2_font_10x20_tf);
      const char* title = gameOver ? "GAME OVER" : "GAME CLEAR!";
      int tw = u8g2.getStrWidth(title);
      u8g2.drawStr((128 - tw) / 2, 26, title);

      u8g2.setFont(u8g2_font_6x12_tr);
      const char* l2a = "A1-3: restart";
      const char* l2b = "A1-4: radio";
      int w2a = u8g2.getStrWidth(l2a);
      int w2b = u8g2.getStrWidth(l2b);
      u8g2.drawStr((128 - w2a) / 2, 42, l2a);
      u8g2.drawStr((128 - w2b) / 2, 55, l2b);

    } else {
      u8g2.drawFrame(0, 0, 128, 64);
      u8g2.drawBox(foodX * CELL, foodY * CELL, CELL, CELL);

      for (uint8_t i = 0; i < slen; i++) {
        u8g2.drawBox(sx[i] * CELL, sy[i] * CELL, CELL, CELL);
      }

      u8g2.setFont(u8g2_font_6x12_tr);
      char sbuf[20];
      snprintf(sbuf, sizeof(sbuf), "Point:%u/%u", slen, CLEAR_LEN);
      int sw = u8g2.getStrWidth(sbuf);
      u8g2.drawStr(128 - sw - 2, 12, sbuf);

      // A1_2で切り替えるフルーツ取得音の状態表示
      u8g2.drawStr(3, 12, fruitSoundEnabled ? "SND" : "MUTE");

      if (paused) {
        const char* p = "PAUSE";
        int pw = u8g2.getStrWidth(p);
        u8g2.drawStr((128 - pw) / 2, 36, p);
      }
    }
  } while (u8g2.nextPage());
}

void OLEDdisplayGame() {
  gameDraw();
}


void OLEDdisplayDark() {
  u8g2.setPowerSave(1);
}

