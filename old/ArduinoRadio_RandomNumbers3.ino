#include <Wire.h>
#include <U8g2lib.h>
#include <RDA5807.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <AceCommon.h> // incrementMod()
#include <AceSPI.h>
#include <AceSegment.h> // HybridModule
#include <FlexiTimer2.h>

// ================== OLED (SSD1306 128x64) ==================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// ================== RDA5807 ==================
RDA5807 rx;
#define DEFAULT_VOLUME 0
#define STORE_TIME     100  // ボタンのオートリピート間隔(ms)

const uint8_t app_id = 43;     // EEPROM識別
const int     eeprom_address = 0;

uint16_t currentFrequency;
uint16_t previousFrequency;
uint16_t previousFrequency2;
uint16_t currentVolume;
uint16_t previousVolume;
uint16_t previousVolume2;
// ==== SEEK表示制御 ====
volatile bool seeking = false;           // シーク中フラグ（OLED用）
volatile bool seekup = false;
volatile bool seekdown = false;
volatile unsigned long lastSeekRedraw = 0; // 間引き用

unsigned long storeTime = 0;
unsigned long storeTime2 = 0;

// ================== 局名（PROGMEM） ==================
const char station12[] PROGMEM = "bayfm";
const char station13[] PROGMEM = "FM NACK5";
const char station1[]  PROGMEM = "TokyoFM";
const char station2[]  PROGMEM = "J-wave";
const char station3[]  PROGMEM = "NHK-FM";
const char station4[]  PROGMEM = "KamakuraFM";
const char station5[]  PROGMEM = "FM-Totsuka";
const char station6[]  PROGMEM = "FM-Yokohama";
const char station7[]  PROGMEM = "interfm";
const char station8[]  PROGMEM = "TBSradio";
const char station9[]  PROGMEM = " Bunka\nHousou";
const char station10[] PROGMEM = " Radio\nNippon";
const char station11[] PROGMEM = " Nippon\nHousou";
const char stationDefault[] PROGMEM = "no Data";

// ================== モード管理 ==================
bool gameMode = false;   // false=ラジオ, true=ゲーム
bool paused  = false;    // ゲーム一時停止
bool randomMode = false;

// ================== スネーク（超軽量実装） ==================
const uint8_t CELL   = 4;                 // 1マス 4x4 ピクセル
const uint8_t GW     = 128 / CELL;        // 32
const uint8_t GH     =  64 / CELL;        // 16
const uint8_t MAX_SNAKE = 96;             // RAM節約（x[]/y[]で約192B）

uint8_t sx[MAX_SNAKE], sy[MAX_SNAKE];
uint8_t slen = 0;
int8_t  dx = 1, dy = 0;                   // 右移動開始
uint8_t foodX = 10, foodY = 8;
// ===== スネーク状態（既存の近くに追加） =====
bool gameOver  = false;
bool gameClear = false;

// 「クリア」の目標長（お好みで調整）
const uint8_t CLEAR_LEN = 20;   // 例: 長さ40でクリア

unsigned long lastTick = 0;
uint16_t tickMs = 140;                    // 進行速度（短いほど速い）
uint16_t a0 = 0;

// ================== 共通ユーティリティ ==================
void resetI2C() {
  delay(10);
  Wire.end();
  delay(30);
  Wire.begin();
  delay(30);
  Wire.setClock(100000);
}

// -------------------- 7セグディスプレイ --------------------
bool digit4state = false;
bool sevensegmentState = true;

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
const uint8_t DIGIT_PINS[NUM_DIGITS] = { 7, 6, 5, 4, 8 };
const uint8_t COLON_PIN = 8;

const uint8_t FRAMES_PER_SECOND = 200;
const uint8_t NUM_SUBFIELDS = 16;

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
const uint8_t PATTERNS[27] = {
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
  0b00000100, //  '(23)
  0b00000111, // :'(24)
  0b00111001, // C (25)
  0b00000000  // blank (26)
};

void setupAceSegment() {
#if INTERFACE_TYPE == INTERFACE_TYPE_HARD_SPI \
    || INTERFACE_TYPE == INTERFACE_TYPE_HARD_SPI_FAST
  spiInstance.begin();
#endif

  spiInterface.begin();
  ledModule.begin();
  pinMode(COLON_PIN, OUTPUT);
}

void sevenSegmentDisplay() {
  static bool isFirst = true;
  static uint8_t digits[NUM_DIGITS];
  volatile unsigned char digitsNum = 0;
  if (gameMode == false){
    if (( currentFrequency != previousFrequency ) || (currentVolume != previousVolume) && (randomMode == false)) {
      digits[0] = currentFrequency / 1000 % 10;
      digits[1] = currentFrequency / 100 % 10 + 10; // ドット付き
      digits[2] = currentFrequency / 10 % 10;
      digits[3] = currentVolume % 10;
      previousFrequency = currentFrequency;
      previousVolume = currentVolume;
    }
    // ランダムモード中：4桁は常にランダムだが、1秒ごとに左→右へ1桁だけ本物の周波数を出す
    else if (randomMode == true) {
      // ランダム演出用の独
      //新規スケッチ
      //立タイマ（storeTimeは使わない）
      static uint8_t      digitsNum = 0;       // 0→1→2→3→0…
      static unsigned long tReveal  = 0;       // 1秒で桁送り
      static unsigned long tNoise   = 0;       // 30msでランダム更新

      // 1秒ごとに表示する桁を進める（積み上げ式でドリフト抑制）
      unsigned long now = millis();
      if (now - tReveal >= 1000UL) {
        tReveal += 1000UL;                     // ← now=millis() 代入より積み上げが安定
        digitsNum = (digitsNum + 1) & 0x03;    // 0..3 の環状
      }

      // 30msごとにランダム更新
      if (now - tNoise >= 30UL) {
        tNoise = now;

        // 周波数の各桁（例: 87.9 → [8][7.][9] のイメージで 4桁を作る）
        uint8_t f0 = (currentFrequency / 1000) % 10;
        uint8_t f1 = (currentFrequency / 100)  % 10 + 10;  // ドット付き
        uint8_t f2 = (currentFrequency / 10)   % 10;
        uint8_t f3 =  currentVolume % 10;
        uint8_t f4 = (currentVolume > 9) ? 23 : 26;

        // まず全桁ランダム
        static char increment = 0;
        if (increment == 0) {
          digits[0] = random(0, 10);
          digits[1] = random(0, 10);
          digits[2] = random(0, 10);
          digits[3] = random(0, 10);
          digits[4] = 26;
        }
        else if (increment == 1){
          
          digits[0] = 26;
          digits[1] = 26;
          digits[2] = 26;
          digits[3] = 26;
          digits[4] = 26;
        }
        else if (increment == 2){
          
          digits[0] = 26;
          digits[1] = 26;
          digits[2] = 26;
          digits[3] = 26;
          digits[4] = 26;
        }
        increment += 1;
        if (increment == 2) {
          increment = 0;
        }

        // digitsNum に該当する桁だけ “本物” を上書き
        switch (digitsNum) {
          case 0: digits[0] = f0; break;
          case 1: digits[1] = f1; break;
          case 2: digits[2] = f2; break;
          case 3: digits[3] = f3; 
                  digits[4] = f4; break;
        }

        // 必要ならここで previous* を更新（必須ではない）
        // previousFrequency = currentFrequency;
        // previousVolume    = currentVolume;
      }
    }

    if (!randomMode){
      digits[4] = (currentVolume > 9) ? 23 : 26;
      if (digit4state) {
          digits[4] = (currentVolume > 9) ? 24 : 22;
          if (millis() - storeTime > 1000) {
              storeTime = millis();
              digit4state = false;
          }
      }
    }
  }/*
  else if (gameMode == true) {
    static int oldT10 = 32767;
    static bool IsFirst = true;

    // 温度×10を四捨五入して整数化（-0.1 など負数も正しく丸め）
    int t10 = (temp.temperature >= 0)
                ? (int)(temp.temperature * 10.0f + 0.5f)
                : (int)(temp.temperature * 10.0f - 0.5f);

    if ((t10 != oldT10) || IsFirst) {
      oldT10 = t10;
      IsFirst = false;

      int at10 = (t10 < 0) ? -t10 : t10; // 表示は絶対値（必要なら符号パターンを用意）

      // 例: 23.4°C → [2][3.][4][C][°]
      digits[0] = (at10 / 100) % 10;       // 10の位（23の"2"）
      if ( digits[0] == 0) digits[0] = 26;
      digits[1] = (at10 / 10)  % 10 + 10;  // 1の位＋ドット（"3."）※+10で“ドット付き数字”
      digits[2] =  at10 % 10;              // 小数1桁（"4"）
      digits[3] = 25;                      // 'C' のパターン
      digits[4] = 23;                      // 右端に ° っぽい記号（手持ちで代用）
    }
  }*/
  for (uint8_t i = 0; i < NUM_DIGITS; ++i) {
      ledModule.setPatternAt(i, PATTERNS[digits[i]]);
  }
  ledModule.renderFieldWhenReady();
}

void saveInformation() {
  if (((currentFrequency = rx.getFrequency()) != previousFrequency2) || (currentVolume != previousVolume2)) {
    if ((millis() - storeTime) > 10000) {
      saveAllReceiverInformation();
      storeTime = millis();
      previousFrequency2 = currentFrequency;
      previousVolume2 = currentVolume;
    }
  }
}

// -------------------- EEPROM → 受信機設定復元 --------------------
void readAllReceiverInformation() {
  rx.setVolume(EEPROM.read(eeprom_address + 1));
  currentFrequency = EEPROM.read(eeprom_address + 2) << 8;
  currentFrequency |= EEPROM.read(eeprom_address + 3);
  //previousFrequency = currentFrequency;
  rx.setFrequency(currentFrequency);
}

void saveAllReceiverInformation() {
  EEPROM.update(eeprom_address, app_id);
  EEPROM.update(eeprom_address + 1, rx.getVolume());           // stores the current Volume
  EEPROM.update(eeprom_address + 2, currentFrequency >> 8);    // stores the current Frequency HIGH byte for the band
  EEPROM.update(eeprom_address + 3, currentFrequency & 0xFF);  // stores the current Frequency LOW byte for the band

  digit4state = true;
}

// -------------------- ステーション名取得（PROGMEM→RAM） --------------------
void getRadioStation(uint16_t frequency, char* buffer) {
  switch (frequency) {
    case 7800:  strcpy_P(buffer, station12); break;
    case 7950:  strcpy_P(buffer, station13); break;
    case 8000:  strcpy_P(buffer, station1);  break;
    case 8130:  strcpy_P(buffer, station2);  break;
    case 8190:
    case 8250:  strcpy_P(buffer, station3);  break;
    case 8280:  strcpy_P(buffer, station4);  break;
    case 8370:  strcpy_P(buffer, station5);  break;
    case 8470:
    case 8700:  strcpy_P(buffer, station6);  break;
    case 7650:
    case 8970:  strcpy_P(buffer, station7);  break;
    case 9050:  strcpy_P(buffer, station8);  break;
    case 9160:  strcpy_P(buffer, station9);  break;
    case 9240:  strcpy_P(buffer, station10); break;
    case 9300:  strcpy_P(buffer, station11); break;
    default:    strcpy_P(buffer, stationDefault); break;
  }
}

// -------------------- 文字列描画（改行対応） --------------------
void drawStrWithNewlines(int x, int y, int lineH, const char* s) {
  const char* p = s;
  int line = 0;
  while (*p) {
    char buf[22];
    int i = 0;
    while (*p && *p != '\n' && i < (int)sizeof(buf)-1) { buf[i++] = *p++; }
    buf[i] = '\0';
    u8g2.drawStr(x, y + line*lineH, buf);
    if (*p == '\n') { p++; line++; }
  }
}

void formatFreq(uint16_t f10kHz, char* out, size_t n) {
  int mhz  = f10kHz / 100;      // 10kHz単位→MHz整数
  int dec1 = (f10kHz % 100) / 10;
  snprintf(out, n, "%2d.%1d", mhz, dec1);
}

// ================== ラジオ画面描画 ==================
void drawRadioScreen(const char* station, uint8_t vol, uint16_t f10kHz) {
  char fbuf[8];
  formatFreq(f10kHz, fbuf, sizeof(fbuf));

  u8g2.firstPage();
  do {
    // 周波数（大数字）
    u8g2.setFont(u8g2_font_logisoso16_tn);     // 数字専用・軽量
    int fw = u8g2.getStrWidth(fbuf);
    u8g2.drawStr(0, 18, fbuf);

    // "MHz"
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(fw + 2, 18, "MHz");

    // 局名（改行対応、やや大きめ）
    u8g2.setFont(u8g2_font_10x20_tf);
    int sw = u8g2.getStrWidth(station);  // 1行目の幅を基準
    int sx = (128 - sw) / 2 - 10;
    drawStrWithNewlines(sx, 40, 20, station);

    // Vol
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(87, 64, "Vol");
    u8g2.setFont(u8g2_font_logisoso16_tn);
    char vbuf[3];
    snprintf(vbuf, sizeof(vbuf), "%2u", vol);
    u8g2.drawStr(100, 64, vbuf);
  } while (u8g2.nextPage());
}

// ================== スネーク：ゲームロジック ==================
// 壁から何マス離すか（1なら外周1マスは候補外）
const uint8_t FOOD_MARGIN = 1;

// 壁沿いを回避してフードを置く
void gamePlaceFood() {
  // 許容範囲（[xmin..xmax], [ymin..ymax]）
  uint8_t xmin = FOOD_MARGIN;
  uint8_t xmax = GW - 1 - FOOD_MARGIN;
  uint8_t ymin = FOOD_MARGIN+3;
  uint8_t ymax = GH - 1 - FOOD_MARGIN;

  // まずはランダム試行（軽量）
  for (uint16_t tries = 0; tries < 200; tries++) {
    uint8_t fx = random(xmin, (uint8_t)(xmax + 1));
    uint8_t fy = random(ymin, (uint8_t)(ymax + 1));

    bool hit = false;
    for (uint8_t i = 0; i < slen; i++) {
      if (sx[i] == fx && sy[i] == fy) { hit = true; break; }
    }
    if (!hit) { foodX = fx; foodY = fy; return; }
  }

  // フォールバック：内側を走査して最初に空いている場所に置く
  for (uint8_t y = ymin; y <= ymax; y++) {
    for (uint8_t x = xmin; x <= xmax; x++) {
      bool hit = false;
      for (uint8_t i = 0; i < slen; i++) {
        if (sx[i] == x && sy[i] == y) { hit = true; break; }
      }
      if (!hit) { foodX = x; foodY = y; return; }
    }
  }

  // それでも置けない＝内側が埋まった → クリア扱いにしてもOK
  // gameClear = true;
}


void gameReset() {
  slen = 4;
  uint8_t cx = GW/2, cy = GH/2;
  for (uint8_t i = 0; i < slen; i++) { sx[i] = cx - i; sy[i] = cy; }
  dx = 1; dy = 0;
  gameOver  = false;     // ★追加
  gameClear = false;     // ★追加
  paused    = false;
  tickMs    = 140;
  gamePlaceFood();
  lastTick  = millis();
}


void gameUpdate() {
  if (gameOver || gameClear || paused) return;

  unsigned long now = millis();
  if (now - lastTick < tickMs) return;
  lastTick = now;

  // 次の頭位置を計算（※負数を扱うので int を使う）
  int nx = (int)sx[0] + dx;
  int ny = (int)sy[0] + dy;

  // ★ 壁判定：範囲外ならゲームオーバー
  if (nx < 0 || nx >= GW || ny < 0 || ny >= GH) {
    gameOver = true;
    return;
  }

  // 身体を後方から詰める
  for (int i = slen - 1; i > 0; i--) {
    sx[i] = sx[i-1];
    sy[i] = sy[i-1];
  }
  // 頭を新しい位置へ（※ラップしない）
  sx[0] = (uint8_t)nx;
  sy[0] = (uint8_t)ny;

  // 自己衝突 → ゲームオーバー
  for (uint8_t i = 1; i < slen; i++) {
    if (sx[0] == sx[i] && sy[0] == sy[i]) { gameOver = true; return; }
  }

  // 食べた？
  if (sx[0] == foodX && sy[0] == foodY) {
    if (slen < MAX_SNAKE) slen++;
    sx[slen-1] = sx[slen-2];
    sy[slen-1] = sy[slen-2];

    if (slen >= CLEAR_LEN) { gameClear = true; return; }
    gamePlaceFood();
    if (tickMs > 60) tickMs -= 4;
  }
}



// ================== スネーク：描画 ==================
void gameDraw() {
  u8g2.firstPage();
  do {
    if (gameOver || gameClear) {
      // --- 終了画面：全消去してから文字だけ描画 ---
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 0, 128, 64);
      u8g2.setDrawColor(1);

      // タイトル
      u8g2.setFont(u8g2_font_10x20_tf);
      const char* title = gameOver ? "GAME OVER" : "GAME CLEAR!";
      int tw = u8g2.getStrWidth(title);
      u8g2.drawStr((128 - tw) / 2, 26, title);

      // 案内メッセージ（2行・上詰め位置）
      u8g2.setFont(u8g2_font_6x12_tr);
      const char* l2a = "Press PAUSE";
      const char* l2b = gameOver ? "to restart" : "to replay";
      int w2a = u8g2.getStrWidth(l2a);
      int w2b = u8g2.getStrWidth(l2b);
      u8g2.drawStr((128 - w2a) / 2, 40, l2a); // 1行目（上詰め）
      u8g2.drawStr((128 - w2b) / 2, 50, l2b); // 2行目（上詰め）

      // ★ゲームクリア時だけシークレットメッセージを追記（小さいフォントで）
      if (gameClear) {
        u8g2.setFont(u8g2_font_5x8_tr);      // 小さめで3行目を確保
        const char* secret = "Secret MASSAGE XYZ";
        int ws = u8g2.getStrWidth(secret);
        u8g2.drawStr((128 - ws) / 2, 62, secret);    // 64px内に収まる位置
      }

    } else {
      // --- プレイ中：従来通り ---
      u8g2.drawFrame(0, 0, 128, 64);
      // フード
      u8g2.drawBox(foodX * CELL, foodY * CELL, CELL, CELL);
      // スネーク本体
      for (uint8_t i = 0; i < slen; i++) {
        int x = sx[i] * CELL, y = sy[i] * CELL;
        u8g2.drawBox(x, y, CELL, CELL);
      }
      // スコア等
      u8g2.setFont(u8g2_font_6x12_tr);
      char sbuf[20];
      snprintf(sbuf, sizeof(sbuf), "Len:%u/%u", slen, CLEAR_LEN);
      int sw = u8g2.getStrWidth(sbuf);
      u8g2.drawStr(128 - sw - 2, 12, sbuf);
    }
  } while (u8g2.nextPage());
}


// -------------------- 画面更新トリガ --------------------
void OLEDdisplay() {
  static bool isFirst = true;
  if ((currentFrequency != previousFrequency) || (currentVolume != previousVolume) || (isFirst == true)) {
    isFirst = false;
    char station[20];
    getRadioStation(currentFrequency, station);
    drawRadioScreen(station, currentVolume, currentFrequency);
    previousFrequency = currentFrequency;
    previousVolume    = currentVolume;
  }
}

// ハードシークの途中でRDA5807ライブラリから呼ばれる
void onSeekStepOLED() {
  // 今の周波数を読み直して、60ms以上あいたら描画
  currentFrequency = rx.getFrequency();
  if ((currentFrequency > 9500) && (seekup == true)){
    rx.setFrequency(7600);
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStepOLED);
  }
  if ((currentFrequency <= 7600) && (seekdown == true)){
    rx.setFrequency(9500);
    rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStepOLED);
  }
  unsigned long now = millis();
  if (now - lastSeekRedraw >= 60 && !gameMode) { // ゲームモード中は描かない
    // ここで“SEEK中”表記が出るように描く
    //OLEDdisplay();
    lastSeekRedraw = now;
  }
}

// ================== 入力 ==================
void getButtonState() {

  static unsigned long lastButtonTime = 0; // 最後にボタンが押された時刻
  unsigned long now = millis();

           a0 = analogRead(A0);
  uint16_t a1 = analogRead(A1);

  // しきい値は既存機材に合わせて調整可
  bool a0_1 = (a0 < 200);              //周波数下
  bool a0_2 = (a0 >= 200 && a0 < 560); //周波数上
  bool a0_3 = (a0 >= 560 && a0 < 710); //シーク上
  bool a0_4 = (a0 >= 710 && a0 < 800); //シーク下

  bool a1_1 = (a1 < 200);              //音量下
  bool a1_2 = (a1 >= 200 && a1 < 560); //音量上
  bool a1_3 = (a1 >= 560 && a1 < 710); //モード切替
  bool a1_4 = (a1 >= 710 && a1 < 800); //未対応

  if (!gameMode) {
    // ---------- ラジオ操作 ----------
    if (a0_1 && (millis() - storeTime) > STORE_TIME) { // 周波数↓
      storeTime = millis();
      rx.setFrequencyDown();
      //currentFrequency = rx.getFrequency();
      OLEDdisplay();
    } else if (a0_2 && (millis() - storeTime) > STORE_TIME) { // 周波数↑
      storeTime = millis();
      rx.setFrequencyUp();
      //currentFrequency = rx.getFrequency();
      OLEDdisplay();
    } else if (a0_3 && (millis() - storeTime) > STORE_TIME) { // シークUP
      storeTime = millis();
      seekup = true;
      rx.seek(RDA_SEEK_WRAP, RDA_SEEK_UP, onSeekStepOLED);
      seekup = false;
      //currentFrequency = rx.getFrequency();
      if (!gameMode) {
        previousFrequency = 0xFFFF;  // 念のため強制再描画
        //currentFrequency = rx.getFrequency();
        OLEDdisplay();
      }
    } else if (a0_4 && (millis() - storeTime) > STORE_TIME) { // シークDOWN
      storeTime = millis();
      seekdown = true;
      rx.seek(RDA_SEEK_WRAP, RDA_SEEK_DOWN, onSeekStepOLED);
      seekdown = false;
      //currentFrequency = rx.getFrequency();
      if (!gameMode) {
        previousFrequency = 0xFFFF;  // 念のため強制再描画
        //currentFrequency = rx.getFrequency();
        OLEDdisplay();
      }
    }

    if (a1_1 && (millis() - storeTime) > STORE_TIME) { // 音量↓
      storeTime = millis();
      rx.setVolumeDown();
      currentVolume = rx.getVolume();
      OLEDdisplay();
    } else if (a1_2 && (millis() - storeTime) > STORE_TIME) { // 音量↑
      storeTime = millis();
      rx.setVolumeUp();
      currentVolume = rx.getVolume();
      OLEDdisplay();
    } else if (a1_3 && (millis() - storeTime) > STORE_TIME) { // モード切替→ゲーム
      storeTime = millis();
      gameMode = !gameMode;
      gameReset();
    } else if (a1_4 && (millis() - storeTime) > STORE_TIME) { // random display起動
      storeTime = millis();
      randomMode = !randomMode;
    }

  } else {
    // ---------- ゲーム操作 ----------
    // 方向：A0=左右, A1=上下（逆走禁止）
    if (a0_1 && dy !=  1) { dx =  0; dy =-1; } //上移動
    if (a0_2 && dx != -1) { dx =  1; dy = 0; } //右移動
    if (a0_3 && dy != -1) { dx =  0; dy = 1; } //上移動
    if (a0_4 && dx !=  1) { dx = -1; dy = 0; } //左移動

    // 一時停止/再開 or リセット
    if (a1_3 && (millis() - storeTime) > 200) { // pause / resume / reset
      storeTime = millis();
      if (gameOver) gameReset();
      else paused = !paused;
    }

    // ラジオへ戻る
    if (a1_4 && (millis() - storeTime) > 200) {
      storeTime = millis();
      gameMode = false;
      previousFrequency = 0xFFFF; // 強制再描画用
      previousVolume    = 0xFFFF;
    }
  }
  if ((now - lastButtonTime) > 3000) {
    currentFrequency = rx.getFrequency();
    OLEDdisplay();
    lastButtonTime = now;  // 連続実行しないようにリセット
  }
}

// ================== setup / loop ==================
void setup() {
  resetI2C();
  // ADCサンプリング分周（元コード踏襲）
  ADCSRA = ADCSRA & 0xf8;  // 分周比を決めるビット(ADPS2:0)を000へ
  ADCSRA = (ADCSRA & 0xF8) | 0x04;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // 乱数（ゲーム用）
  randomSeed(analogRead(A3));

  // ラジオ
  rx.setup();
  rx.setBand(RDA_FM_BAND_WORLD);      // 必要なら _JAPAN_WIDE に
  //rx.setSpace(RDA_FM_SPACE_100K);     // 0.1MHzステップ
  rx.setVolume(DEFAULT_VOLUME);
  if (EEPROM.read(eeprom_address) == app_id) {
    readAllReceiverInformation();
  }

  currentFrequency = rx.getFrequency();
  currentVolume    = rx.getVolume();

  setupAceSegment();
  ledModule.setBrightness(15);

  FlexiTimer2::set(2, 1.0/10000, sevenSegmentDisplay); // call every 500 1ms "ticks"
  FlexiTimer2::start();

  rx.setFrequency(currentFrequency);

  // OLED
  u8g2.begin();
  u8g2.setFlipMode(0);

  rx.setFrequency(currentFrequency);

  OLEDdisplay();
}

void loop() {

  if (!gameMode) {
    // ---- ラジオ画面 ----
    //currentFrequency = rx.getFrequency();
    //currentVolume    = rx.getVolume();

  } else {
    // ---- ゲーム ----
    gameUpdate();
    gameDraw();
  }
  
  getButtonState();

  saveInformation();
}
