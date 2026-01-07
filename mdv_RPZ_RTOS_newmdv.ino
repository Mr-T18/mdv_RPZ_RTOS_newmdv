/*
 * モータドライバ Ver 22-RTOS
 * RP2040 FreeRTOS (デュアルコア) 対応版
 *
 * Core 0 (リアルタイム処理):
 * - canISR (HW割り込み): _CAN_INTピンでトリガー
 * - canReadTask (優先度3): ISRからのセマフォで起動し、CAN.readMsgBuf() を実行
 * - motorTask (優先度2): 50ms (WDT1_TIMEOUT) 周期で Motor() を実行
 * - loop (優先度1): Control() ロジックとボタン監視を実行
 *
 * Core 1 (低優先度処理):
 * - loop1 (優先度1): 250ms (WDT2_TIMEOUT) 周期で SetDisplay() と showStatus() を実行
 */

#define Ver "22-RTOS"

// --- FreeRTOS関連のヘッダ ---
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <mcp_can.h>
#include <EEPROM.h>

#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(U8X8_PIN_NONE);
#include "imported_font.h"
#define font_n u8x8_font_amstrad_cpc_extended_r
#define font_c imported_font

// デバッグシリアルを有効にする場合はコメント解除
#define Seri

// --- ID定義 ---
#define ECU_ID 100
#define TPIP_ID 1600
unsigned char txBuf[8];

#define MM 5 // エンコーダで速度データを扱うときの移動平均の個数

// --- ピン定義 ---
#define SPI_CS_PIN 5
#define SW_L 12
#define SW_R 7
#define LMT_H 13
#define LMT_L 14
#define MOT_DIR 28
#define MOT_OnF 29
#define MOT_PWM 15
#define _CAN_INT 8
#define _CAN_CS 5
#define TX_LED 30
#define RX_LED 17
#define EDIT_SETTINGS 26

// --- パラメータ定義 ---
#define MAN_DUTY 200 // ボタンで動かすときのDuty
#define APP_DUT 4.0
#define APP_SPD 4.0
#define APP_POS 4.0
#define ENC_STEP 3.75

// --- タイムアウト定義 ---
#define WDT_TIMEOUT 1000 // CAN通信のWDT
#define WDT1_TIMEOUT 50  // Motorタスクの実行周期 (ms)
#define WDT2_TIMEOUT 250 // Displayタスクの実行周期 (ms)

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

// --- グローバル変数 (オリジナルコードより) ---
INT32U can_id, ID_SET, ID_RCV;
INT8U can_err;
unsigned char len = 0;
unsigned char rxBuf[8];
unsigned char Buf[2];
unsigned long wdt = 1000;
unsigned int duty;
unsigned char dir, pre_dir;
unsigned char ctrl_mode = 99, pre_mode;
float app = 1.0, spd_ref, pos_ref, spd_enc, pos_enc, pre_enc;
float mm_spd[MM];
unsigned char mm = 0;
unsigned int RFlag, SpdModeFlag, Rdat;
unsigned char flg;
unsigned char EncS, preEncS;
unsigned int EncTimer;
unsigned char EncFlag, EncErr;
unsigned char rcv_flg, flagRecv;
unsigned char dsp_flg1, dsp_flg2, dsp_flg3, dsp_flg4, dsp_flg5, dsp_flg6;
unsigned char dsp_duty;
float Pos_duty;
unsigned int PDuty, PPos;
unsigned char lmt_flg_A;
unsigned char lmt_flg_B;
unsigned char wdt_flg;
unsigned char m_duty, m_dir, m_rev, rev;
unsigned char pre_m_duty, pre_m_dir, pre_m_rev;
unsigned long wdt0, wdt1, wdt2, wdt3;

unsigned char rot_reverse; // 0:順転,1:反転
volatile bool isEditMode = false;

// --- FreeRTOSハンドル ---
SemaphoreHandle_t xCanInterruptSemaphore; // CAN ISRがCANタスクを起こすためのセマフォ
SemaphoreHandle_t xMotorDataMutex;        // Core 0とCore 1のデータ共有を保護するMutex

// タスクハンドル
TaskHandle_t xMotorTaskHandle = NULL;
TaskHandle_t xDisplayTaskHandle = NULL;
TaskHandle_t xCanTaskHandle = NULL;

// =========================================================
//  CAN処理 (ISR と タスク)
// =========================================================

/**
 * @brief CAN割り込みサービスルーチン (ISR)
 * _CAN_INT (GPIO 8) ピンがFALLING (H->L) になった時に呼ばれる。
 * ISRは超高速に！
 */
void canISR()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // CAN Read Taskに「メッセージが来たよ」と通知する
  xSemaphoreGiveFromISR(xCanInterruptSemaphore, &xHigherPriorityTaskWoken);
  // もし優先度の高いタスクが起床したら、ISRの直後にタスクを切り替える
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief CAN読み取りタスク (Core 0で高優先度実行)
 * ISRからセマフォを受け取ると起動し、SPI経由でCANデータを読み取る。
 */
void canReadTask(void *pvParameters)
{
  for (;;)
  {
    // ISRからセマフォがGiveされるまで、ここで待機 (CPU消費ゼロ)
    if (xSemaphoreTake(xCanInterruptSemaphore, portMAX_DELAY) == pdTRUE)
    {

      // --- CAN_RCV() の内容をここに展開 ---
      // (SPI通信はISR内ではNGなので、タスクで実行する)
      CAN.readMsgBuf(&ID_RCV, &len, rxBuf);

      // 読み取ったデータをグローバル変数に反映
      // 複数の変数を更新するため、クリティカルセクションで保護
      // (Core 0のloopタスク、Core 1のDisplayタスクと競合するため)
      taskENTER_CRITICAL();
      if (ID_RCV == can_id)
      {
        digitalWrite(TX_LED, LOW);
        wdt0 = millis(); // wdt0はControlタスクも参照
        Buf[0] = rxBuf[0];
        Buf[1] = rxBuf[1];
        flagRecv = 1; // flagRecvはControlタスク(loop)も参照
        rcv_flg = 1;  // rcv_flgはDisplayタスク(loop1)も参照
      }

      if (ID_RCV == 0)
      {
        rcv_flg = 1;
      }
      digitalWrite(TX_LED, HIGH);
      taskEXIT_CRITICAL();
    }
  }
}

// =========================================================
//  初期化関数 (オリジナル)
// =========================================================

void init_can()
{
  while (CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) != CAN_OK)
    ;

  INT32U ID_mask;
  ID_mask = can_id << 16;
  CAN.init_Mask(0, 0, 0x007F0000);
  CAN.init_Filt(0, 0, ID_mask);
  CAN.init_Filt(1, 0, 0x00000000);

  CAN.init_Mask(1, 0, 0x007F0000);
  CAN.init_Filt(2, 0, 0x00000000);
  CAN.init_Filt(3, 0, 0x00000000);
  CAN.init_Filt(4, 0, 0x00000000);
  CAN.init_Filt(5, 0, 0x00000000);

  CAN.setMode(MCP_NORMAL);

  delay(50);
  return;
}

/* ここから新機能
 */

// canIDの設定のあと，順転or反転の設定を行う

/*
順転 or 反転の設定画面(従来の設定が順転の場合)


  ◎Forward
    Reverse

選んだほうのフォントを白黒反転して選択状態を見やすいようにする
SW_Lを押すと切り替え，
SW_Rを押すと決定

*/

bool initialDisplay()
{
  int flg = 0;
  int cnt_d = 4;
  char temp[10];

  u8x8.clear();

  can_id = EEPROM.read(0);
  rot_reverse = EEPROM.read(1);

  u8x8.draw2x2String(0, 0, "ID: ");
  sprintf(temp, "%d", int(can_id));
  u8x8.draw2x2String(8, 0, temp);
  u8x8.drawString(0, 2, "set > push 2 btn");
  u8x8.drawString(0, 3, "Ver.");
  u8x8.drawString(5, 3, Ver);

  while ((cnt_d > 0) && flg == 0) // 起動時のcanID設定モードon
  {
    if ((digitalRead(SW_L) == HIGH) && (digitalRead(SW_R) == HIGH))
      cnt_d--;
    else
    {
      flg = 1;
      cnt_d = 0;
    }

    sprintf(temp, "%d", cnt_d);
    u8x8.drawString(15, 3, temp);
    delay(1000);
  }

  delay(500);
  if (flg == 0)
  {
    // ID_SET = can_id << 16; <- こいつ元からおるけど必要ないよね？
    return false;
  }
  else
  {
    return true;
  }
}

void settingsEdit()
{
  int cur = 0;
  int flg = 0;
  int data[3];
  char temp[10];

  char rot_temp = rot_reverse;
  char select = 255;

  u8x8.draw2x2String(0, 0, "ID: ");
  sprintf(temp, "%d", int(can_id));
  u8x8.draw2x2String(8, 0, temp);

  data[0] = can_id / 100;
  data[1] = (can_id - (data[0] * 100)) / 10;
  data[2] = can_id % 10;

  u8x8.drawString(0, 2, "set > push 2 btn");

  u8x8.drawString(0, 3, "Ver.");
  u8x8.drawString(5, 3, Ver);

  u8x8.clear();

  flg = 1;

  while (flg != 99)
  {

    if (flg == 1)
    {

      u8x8.drawString(13, 3, "Ent");
      u8x8.drawString(9, 3, ">");
      u8x8.draw2x2String(0, 0, "ID:");

      if (cur == 0)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      sprintf(temp, "%d", data[0]);
      u8x8.draw2x2String(6, 0, temp);

      if (cur == 1)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      sprintf(temp, "%d", data[1]);
      u8x8.draw2x2String(8, 0, temp);

      if (cur == 2)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      sprintf(temp, "%d", data[2]);
      u8x8.draw2x2String(10, 0, temp);

      if (cur == 3)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      can_id = ((data[0] * 100) + (data[1] * 10) + data[2]);
      if ((can_id > 254) || (can_id < 1))
        u8x8.draw2x2String(12, 0, "_");
      else
        u8x8.draw2x2String(12, 0, "*");

      u8x8.setInverseFont(0);
    }

    if (digitalRead(SW_L) == LOW) // 左ボタン押下時．カーソルを動かす
      flg = 2;
    if ((digitalRead(SW_L) == HIGH) && flg == 2) // ボタンのチャタリング防止
    {
      cur++;
      if (cur > 3)
        cur = 0;
      flg = 1;
    }

    if (digitalRead(SW_R) == LOW) // 右ボタン押下時．カーソルのある場所の数字を変えるorエンター
      flg = 3;
    if ((digitalRead(SW_R) == HIGH) && flg == 3)
    {
      if (cur != 3) // 数字を変える
      {
        data[cur]++;
        if (data[cur] > 9)
          data[cur] = 0;
        if ((cur == 0) && (data[cur] > 2))
          data[cur] = 0;
        delay(50);
        flg = 1;
      }
      else // エンター
      {
        can_id = (data[0] * 100) + (data[1] * 10) + data[2];
        if ((can_id <= 254) && (can_id >= 1))
          flg = 99;
        else
          flg = 1;
      }
      delay(50);
    }
  }

  flg = 1;
  cur = rot_reverse; // 最初のカーソルの位置は従来の設定の位置
  u8x8.clear();

  while (flg != 99)
  {
    if (flg == 1)
    {
      u8x8.drawString(0, 0, "Rotation:");
      u8x8.drawString(0, 3, "Enter");


      if (rot_reverse == 0) // 現在の順転or反転の設定を示す
      {
        u8x8.setCursor(2, 1);
        u8x8.setFont(font_c);
        u8x8.print("\x45");
        u8x8.setFont(font_n);
      }
      else if (rot_reverse == 1)
      {
        u8x8.setCursor(2, 2);
        u8x8.setFont(font_c);
        u8x8.print("\x45");
        u8x8.setFont(font_n);
      }

      if (cur == 0)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      u8x8.drawString(3, 1, "Forward");

      if (cur == 1)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      u8x8.drawString(3, 2, "Reverse");

      if (cur == 2)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      u8x8.drawString(0, 3, "Enter");

      u8x8.setInverseFont(0);
      if(select == 0)
      {
        u8x8.drawString(10, 1, "<<");
        u8x8.drawString(10, 2, "  ");
      }
      else if(select == 1)
      {
        u8x8.drawString(10, 1, "  ");
        u8x8.drawString(10, 2, "<<");
      }
      else
        ;
      flg = 10;
    }

    if (digitalRead(SW_L) == LOW) // 左ボタン押下時．カーソルを動かす
      flg = 2;
    if ((digitalRead(SW_L) == HIGH) && flg == 2) // ボタンのチャタリング防止
    {
      cur++;
      if (cur > 2)
        cur = 0;
      flg = 1;
      rot_temp = cur;
    }

    if (digitalRead(SW_R) == LOW) // 右ボタン押下時．エンター
    {
      flg = 3;
    }
    if ((digitalRead(SW_R) == HIGH) && flg == 3)
    {
      if(cur < 2){
        select = cur;
        flg = 1;
        delay(50);
      }
      else{
        flg = 99;
        if(select != 255) rot_reverse = select;
        delay(50);
      }
    }
    
  }

  ID_SET = can_id << 16; // <- この変数ID_SET，別に有効活用してない.

  EEPROM.write(0, can_id);
  EEPROM.write(1, rot_reverse);
  EEPROM.end();

  init_can();

  return;
}

void set_CANID()
{
  int cnt_d = 4;
  int cur = 0;
  int flg = 0;
  int data[3];
  char temp[10];

  can_id = EEPROM.read(0);

  u8x8.draw2x2String(0, 0, "ID: ");
  sprintf(temp, "%d", int(can_id));
  u8x8.draw2x2String(8, 0, temp);

  data[0] = can_id / 100;
  data[1] = (can_id - (data[0] * 100)) / 10;
  data[2] = can_id % 10;

  u8x8.drawString(0, 2, "set > push 2 btn");

  u8x8.drawString(0, 3, "Ver.");
  u8x8.drawString(5, 3, Ver);

  while ((cnt_d > 0) && flg == 0) // 起動時のcanID設定モードon
  {
    // ここで起動時にボタン押下を読み取る
    // この分岐の処理と，canIDをカーソルで編集している処理を分ければ，
    // サブボタン押下時にset_CANID関数を再利用できる
    if ((digitalRead(SW_L) == HIGH) || (digitalRead(SW_R) == HIGH))
      cnt_d--;
    else
    {
      flg = 1;
      cnt_d = 0;
    }

    sprintf(temp, "%d", cnt_d);
    u8x8.drawString(15, 3, temp);
    delay(1000);
  }

  delay(500);
  if (flg == 0)
  {
    ID_SET = can_id << 16;
    return;
  }

  u8x8.clear();

  flg = 1;

  while (flg != 99)
  {

    if (flg == 1)
    {

      u8x8.drawString(13, 3, "Ent");
      u8x8.drawString(9, 3, ">");

      u8x8.draw2x2String(0, 0, "ID:");
      if (cur == 0)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      sprintf(temp, "%d", data[0]);
      u8x8.draw2x2String(6, 0, temp);
      if (cur == 1)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      sprintf(temp, "%d", data[1]);
      u8x8.draw2x2String(8, 0, temp);
      if (cur == 2)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);
      sprintf(temp, "%d", data[2]);
      u8x8.draw2x2String(10, 0, temp);
      if (cur == 3)
        u8x8.setInverseFont(1);
      else
        u8x8.setInverseFont(0);

      can_id = ((data[0] * 100) + (data[1] * 10) + data[2]);
      if ((can_id > 254) || (can_id < 1))
        u8x8.draw2x2String(12, 0, "_");
      else
        u8x8.draw2x2String(12, 0, "*");

      u8x8.setInverseFont(0);
    }

    if (digitalRead(SW_L) == LOW) // 左ボタン押下時．カーソルを動かす
      flg = 2;
    if ((digitalRead(SW_L) == HIGH) && flg == 2) // ボタンのチャタリング防止
    {
      cur++;
      if (cur > 3)
        cur = 0;
      flg = 1;
    }

    if (digitalRead(SW_R) == LOW) // 右ボタン押下時．カーソルのある場所の数字を変えるorエンター
      flg = 3;
    if ((digitalRead(SW_R) == HIGH) && flg == 3)
    {
      if (cur != 3)
      {
        data[cur]++;
        if (data[cur] > 9)
          data[cur] = 0;
        if ((cur == 0) && (data[cur] > 2))
          data[cur] = 0;
        delay(50);
        flg = 1;
      }
      else
      {
        can_id = (data[0] * 100) + (data[1] * 10) + data[2];
        if ((can_id <= 254) && (can_id >= 1))
          flg = 99;
        else
          flg = 1;
      }
      delay(50);
    }
  }

  EEPROM.write(0, can_id);
  EEPROM.end();
  ID_SET = can_id << 16;

  return;
}

// =========================================================
//  メインロジック (Core 0 / loopタスク)
// =========================================================

char Direction(unsigned char _dir)
{
  unsigned char temp = 0;
  temp = _dir ^ rot_reverse; // xor
  return temp;
}

/**
 * @brief メインの制御ロジック (オリジナル関数)
 * dly10() を vTaskDelay() に置き換え。
 * CHK() は削除 (CANタスクが処理するため)。
 */
void Control()
{
  wdt = millis() - wdt0;

  // Manualモード(ctrl_mode = 9)
  if (digitalRead(SW_L) == LOW)
  {
    vTaskDelay(pdMS_TO_TICKS(10)); // dly10() を置き換え
    if ((digitalRead(SW_L) == LOW))
    {
      flg = 1;
      ctrl_mode = 9;
      wdt0 = millis();
    }
  }
  else if ((digitalRead(SW_L) == HIGH) && (flg == 1))
  {
    flg = 5;
  }

  if (digitalRead(SW_R) == LOW)
  {
    vTaskDelay(pdMS_TO_TICKS(10)); // dly10() を置き換え
    if ((digitalRead(SW_R) == LOW))
    {
      flg = 2;
      ctrl_mode = 9;
      wdt0 = millis();
    }
  }
  else if ((digitalRead(SW_R) == HIGH) && (flg == 2))
  {
    flg = 5;
    wdt0 = millis();
  }

  // コントロールモードごとの処理
  if (ctrl_mode != 9)
  {

    // WDTの処理
    if (wdt > WDT_TIMEOUT)
    {
      // ここに停止処理を書く
      duty = 0;
      rev = 0;
      ctrl_mode = 99;
      wdt0 = millis();
      wdt_flg = 1;
    }

    if (RFlag != 0)
    { // RFlagはloop()によって立てられる

      // wdt0 = millis(); // wdt0の更新はCANReadTaskが担当する

      // Buf[] の読み取り
      // CANタスクが書き込み中かもしれないので、クリティカルセクションで読み取る
      unsigned char localBuf[2];
      taskENTER_CRITICAL();
      localBuf[0] = Buf[0];
      localBuf[1] = Buf[1];
      taskEXIT_CRITICAL();

      // 指令値代入処理の間はCAN受信割り込みを停止
      if ((localBuf[0] & 0x01) != 0)
        // dir = 0; // 命令1バイト目が xxxx xx01 ならCW
        dir = Direction(0);
      if ((localBuf[0] & 0x02) != 0)
        // dir = 1; // 命令1バイト目が xxxx xx10 ならCCW
        dir = Direction(1);

      ctrl_mode = localBuf[0] >> 6; // 命令1バイト目の上位2ビットを制御モードとして代入

      Rdat = localBuf[1];

      RFlag = 0; // RFlagを消費
    }

    // Dutyモードの処理
    if (ctrl_mode == 0)
    {
      // RFlagは既に上記で処理済み
      duty = map((int)Rdat, 0, 100, 0, 250);
      rev = 1;
    }

    // 位置制御モードの処理
    if (ctrl_mode == 1)
    {
      duty = 0;
      rev = 0;
    }

    // 速度制御モードの処理
    if (ctrl_mode == 2)
    {
      duty = 0;
      rev = 0;
    }

    if (duty < 5)
      duty = 0;
    if (duty == 0)
      rev = 0;
  }
  else
  { // ここからマニュアルモードの出力処理

    if ((flg == 1) || (flg == 2))
    {
      duty = MAN_DUTY;
      rev = 1;

      if (flg == 1)
      {
        // ここにCWの処理を書く
        // dir = 0;
        dir = Direction(0);
      }
      else
      {
        // ここにCCWの処理を書く
        // dir = 1;
        dir = Direction(1);
      }
    }

    if (flg == 5)
    {
      flg = 0;
      ctrl_mode = 99;
      // ここに停止の処理を書く
      duty = 0;
      rev = 0;
    }
  }

  if ((dir == 0) && (digitalRead(LMT_H) == LOW))
  {
    // ここに停止処理を書く
    duty = 0;
    ctrl_mode = 99;
    rev = 0;
  }

  if ((dir == 1) && (digitalRead(LMT_L) == LOW))
  {
    // ここに停止処理を書く
    duty = 0;
    ctrl_mode = 99;
    rev = 0;
  }

  // m_duty などの更新
  // このデータはCore 1 (Display) と Core 0 (Motor) が読むので、Mutexで保護する
  if (xSemaphoreTake(xMotorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    m_duty = duty;
    dsp_duty = int(100 * m_duty / 250); // dsp_dutyもここで計算
    m_dir = dir;
    m_rev = rev;

    // Mutexを解放
    xSemaphoreGive(xMotorDataMutex);
  }
}

// =========================================================
//  モーター処理 (Core 0 / motorTask)
// =========================================================

/**
 * @brief モータードライバへの出力 (オリジナル関数)
 * dly10() を vTaskDelay() に置き換え。
 * CHK() は削除。
 */
void Motor()
{
  // Core 1と共有する変数を読み出すためにMutexを取得
  unsigned char local_m_dir, local_m_rev, local_m_duty;
  unsigned char local_ctrl_mode;

  if (xSemaphoreTake(xMotorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    local_m_dir = m_dir;
    local_m_rev = m_rev;
    local_m_duty = m_duty;
    local_ctrl_mode = ctrl_mode; // ctrl_modeも保護対象

    xSemaphoreGive(xMotorDataMutex);
  }
  else
  {
    // Mutex取れなければ今回はスキップ
    return;
  }

  if (local_ctrl_mode != pre_mode)
  {
    pre_mode = local_ctrl_mode;
  }

  if (local_m_dir != pre_m_dir)
  {
    pre_m_dir = local_m_dir;
    pre_m_duty = 0;
    analogWrite(MOT_PWM, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); // dly10() を置き換え
    if (local_m_dir == 0)
      digitalWrite(MOT_DIR, LOW);
    else
      digitalWrite(MOT_DIR, HIGH);
  }

  if (local_m_rev != pre_m_rev)
  {
    pre_m_rev = local_m_rev;
    pre_m_duty = 0;
    if (local_m_rev == 1)
    {
      digitalWrite(MOT_OnF, HIGH);
    }
    else
    {
      analogWrite(MOT_PWM, 0);
      vTaskDelay(pdMS_TO_TICKS(10)); // dly10() を置き換え
      digitalWrite(MOT_OnF, LOW);
    }
  }

  if (pre_m_duty == 0)
  {
    pre_m_duty = local_m_duty;
    if (local_m_duty != 0)
    {
      vTaskDelay(pdMS_TO_TICKS(10)); // dly10() を置き換え
      analogWrite(MOT_PWM, local_m_duty);
    }
  }
  else
  {
    pre_m_duty = local_m_duty;
    analogWrite(MOT_PWM, local_m_duty);
  }
}

/**
 * @brief モーター周期実行タスク (Core 0で中優先度実行)
 */
void motorTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {

    if (isEditMode)
    {
      analogWrite(MOT_PWM, 0);
      digitalWrite(MOT_OnF, LOW);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(WDT1_TIMEOUT));
    }

    // Motor() 関数を実行
    Motor();

    // 50ms (WDT1_TIMEOUT) 周期で正確に実行
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(WDT1_TIMEOUT));
  }
}

// =========================================================
//  ディスプレイ処理 (Core 1 / loop1タスク)
// =========================================================

void BaseDisplay()
{
  char temp[10];

  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("I D:");

  u8x8.setCursor(5, 0);
  sprintf(temp, "%-3d", int(can_id));
  u8x8.print(temp);

  u8x8.setFont(font_c);
  u8x8.setCursor(9, 3);
  u8x8.print(" \x41 ");

  u8x8.setCursor(13, 3);
  u8x8.print(" \x43 ");
  u8x8.setFont(font_n);

  u8x8.setCursor(0, 1);
  u8x8.print("Mod:");

  u8x8.setCursor(9, 1);
  u8x8.print("Pwr:");

  u8x8.setCursor(0, 2);
  u8x8.print("Dir:");

  u8x8.setCursor(9, 2);
  u8x8.print("Rev:");

  u8x8.setCursor(0, 3);
  u8x8.print("Lmt:");
}

/**
 * @brief ディスプレイ更新 (オリジナル関数)
 * CHK() は削除。
 * Core 0と共有する変数は、Mutex保護下でローカル変数にコピーしてから使用する。
 */
void SetDisplay()
{
  // Core 1 (Display) と Core 0 (Control) が共有するデータを
  // 読み取る前に Mutex を取得する

  // このディスプレイ処理で必要な共有変数を格納するローカル変数
  unsigned char local_ctrl_mode;
  unsigned char local_dsp_duty;
  unsigned char local_m_dir;
  unsigned char local_m_rev;
  float local_app;
  unsigned char local_rcv_flg;
  INT32U local_ID_RCV;
  unsigned char local_wdt_flg;

  if (xSemaphoreTake(xMotorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    // --- Mutex保護区間 ---
    // ここで共有データをローカル変数にコピー
    local_ctrl_mode = ctrl_mode;
    local_dsp_duty = dsp_duty;
    local_m_dir = m_dir;
    local_m_rev = m_rev;
    local_app = app;
    local_rcv_flg = rcv_flg;
    local_ID_RCV = ID_RCV;
    local_wdt_flg = wdt_flg;

    // 消費するフラグはここで倒す
    rcv_flg = 0;
    wdt_flg = 0;

    // Mutexを解放
    xSemaphoreGive(xMotorDataMutex);
    // --- Mutex保護区間 終了 ---
  }
  else
  {
    // Mutexが取得できなかった (Core 0がビジー)
    // 今回のディスプレイ更新はスキップ
    return;
  }

  // --- これ以降はローカル変数を使ってディスプレイ処理を行う ---
  // これにより、重い描画処理(35ms)の間にCore 0がデータを変更しても
  // 表示がチグハグになることを防げる

  char temp[10];

  u8x8.setCursor(0, 0);
  if (local_rcv_flg == 1)
  { // ローカル変数を使用
    if (local_ID_RCV != 0)
    {
      u8x8.setInverseFont(1);
      u8x8.print("Rcv");
      u8x8.setInverseFont(0);
    }
    else
    {
      u8x8.setCursor(9, 0);
      u8x8.print("*");
    }
    dsp_flg1 = 1;
    // rcv_flg = 0; // Mutex内で処理済み
  }
  else if (dsp_flg1 == 1)
  {
    u8x8.print("I D");
    u8x8.setCursor(9, 0);
    u8x8.print(" ");
    dsp_flg1 = 0;
  }

  // WDTの処理
  u8x8.setCursor(14, 0);
  if (local_wdt_flg == 1) // ローカル変数を使用
  {
    u8x8.print("W");
    // wdt_flg = 0; // Mutex内で処理済み
    dsp_flg2 = 1;
  }
  else if (dsp_flg2 == 1)
  {
    u8x8.print(" ");
    dsp_flg2 = 0;
  }

  u8x8.setCursor(4, 1);
  switch (local_ctrl_mode)
  { // ローカル変数を使用
  case 0:
    u8x8.print("DTY");
    break;
  case 9:
    u8x8.print("Btn");
    if (flg == 1)
    { // 'flg' はCore 0のControlタスク専用のためMutex不要
      u8x8.setCursor(9, 3);
      u8x8.setFont(font_c);
      u8x8.print(" \x40 ");
      u8x8.setFont(font_n);
    }
    else
    {
      u8x8.setCursor(13, 3);
      u8x8.setFont(font_c);
      u8x8.print(" \x42 ");
      u8x8.setFont(font_n);
    }
    dsp_flg3 = 1;
    break;
  case 99:
    u8x8.print("Stp");
    break;
  }

  if ((local_ctrl_mode != 9) && (dsp_flg3 == 1))
  {
    u8x8.setFont(font_c);
    u8x8.setCursor(9, 3);
    u8x8.print(" \x41 ");
    u8x8.setCursor(13, 3);
    u8x8.print(" \x43 ");
    u8x8.setFont(font_n);
    dsp_flg3 = 0;
  }

  u8x8.setCursor(13, 1);
  u8x8.setFont(font_c);
  if (local_dsp_duty == 0)
    u8x8.print("   "); // ローカル変数
  if ((local_dsp_duty > 0) && (local_dsp_duty < 20))
    u8x8.print("\x46  ");
  if ((local_dsp_duty >= 20) && (local_dsp_duty < 40))
    u8x8.print("\x47  ");
  if ((local_dsp_duty >= 40) && (local_dsp_duty < 60))
    u8x8.print("\x47\x46 ");
  if ((local_dsp_duty >= 60) && (local_dsp_duty < 80))
    u8x8.print("\x47\x47 ");
  if ((local_dsp_duty >= 80) && (local_dsp_duty < 100))
    u8x8.print("\x47\x47\x46");
  if (local_dsp_duty == 100)
    u8x8.print("\x47\x47\x47");
  u8x8.setFont(font_n);

  u8x8.setCursor(15, 0);
  if (local_app != 1.0)
  {
    u8x8.print("A");
  } // ローカル変数
  else
  {
    u8x8.print(" ");
  }

  u8x8.setFont(font_c);

  u8x8.setCursor(4, 2);
  if (local_m_dir == 0)
  {
    if (local_m_rev == 0)
    {
      u8x8.print(" \x41 ");
    }
    else
    {
      u8x8.print(" \x40 ");
    }
  } // ローカル変数
  else
  {
    if (local_m_rev == 0)
    {
      u8x8.print(" \x43 ");
    }
    else
    {
      u8x8.print(" \x42 ");
    }
  }

  u8x8.setCursor(13, 2);
  if (local_m_rev == 0)
  {
    u8x8.print(" \x45 ");
  } // ローカル変数
  else
  {
    u8x8.print(" \x44 ");
  }

  u8x8.setFont(font_n);

  // digitalReadはどちらのコアからでも安全
  if (digitalRead(LMT_L) == LOW)
  {
    u8x8.setCursor(4, 3);
    u8x8.setFont(font_c);
    u8x8.print("\x42\x42\x42");
    u8x8.setFont(font_n);
    dsp_flg4 = 1;
    if (lmt_flg_A == 0)
      send_can(0b00000010); // send_can()はCore 1から呼んでOK
  }
  else
  {
    lmt_flg_A = 0;
    if (dsp_flg4 == 1)
    {
      u8x8.setCursor(4, 3);
      u8x8.print("   ");
      dsp_flg4 = 0;
    }
  }

  if (digitalRead(LMT_H) == LOW)
  {
    u8x8.setCursor(4, 3);
    u8x8.print("X");
    u8x8.setFont(font_c);
    u8x8.print("\x40");
    u8x8.setFont(font_n);
    u8x8.print("X");
    dsp_flg5 = 1;
    if (lmt_flg_B == 0)
      send_can(0b00000001); // send_can()はCore 1から呼んでOK
  }
  else
  {
    lmt_flg_B = 0;
    if (dsp_flg5 == 1)
    {
      u8x8.setCursor(4, 3);
      u8x8.print("   ");
      dsp_flg5 = 0;
    }
  }

  // CAN通信エラーの場合のリセット処理
  can_err = CAN.checkError(); // Core 1からSPI (CAN)にアクセス

  if (can_err != 0)
  {
    digitalWrite(RX_LED, LOW);
    // flagRecv = 0; // Core 0と競合する変数は触らない
    // duty = 0;
    // rev = 0;
    init_can(); // Core 1からSPI (CAN)にアクセス
    u8x8.setCursor(9, 0);
    u8x8.print("# ERR #");
    can_err = 0;
    digitalWrite(RX_LED, HIGH);
    dsp_flg6 = 1;
  }
  else if (dsp_flg6 == 1)
  {
    u8x8.setCursor(9, 0);
    u8x8.print("       ");
    dsp_flg6 = 0;
  }
}

// dly10() は vTaskDelay(pdMS_TO_TICKS(10)) に置き換えられたので削除

void send_can(unsigned char buf)
{
  // この関数はCore 1 (DisplayTask) から呼ばれるが、
  // CANReadTask (Core 0) と同時にCAN (SPI)にアクセスすると
  // 競合する可能性がある。
  // 厳密にはここもSPIバスのMutexで保護すべきだが、
  // リミットスイッチヒット (稀) と CAN受信 (頻繁) の
  // タイミングが重なることは稀として、一旦そのままにする。

  txBuf[0] = can_id;
  txBuf[1] = buf;
  CAN.sendMsgBuf(ECU_ID, 0, 2, txBuf); // sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);
  txBuf[0] = ECU_ID;
  txBuf[1] = can_id;
  CAN.sendMsgBuf(TPIP_ID, 0, 2, txBuf);
  lmt_flg_A = 1;
  lmt_flg_B = 1;
}

void showStatus()
{
#ifdef Seri
  // この関数もCore 1で実行されるため、Mutexで保護
  unsigned char local_m_dir, local_m_rev, local_ctrl_mode;
  INT32U local_ID_RCV;

  if (xSemaphoreTake(xMotorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    // 共有データをローカル変数にコピー
    local_ID_RCV = ID_RCV;
    local_m_dir = m_dir;
    local_m_rev = m_rev;
    local_ctrl_mode = ctrl_mode;

    xSemaphoreGive(xMotorDataMutex);

    // ローカル変数を使ってシリアル出力
    Serial.print("ID_RCV: ");
    Serial.print(local_ID_RCV);
    Serial.print("   ");
    Serial.print("  CAN_ID: ");
    Serial.print(can_id);
    Serial.print("m_dir: ");
    Serial.print(local_m_dir);
    Serial.print(",  m_rev: ");
    Serial.print(local_m_rev);
    Serial.print("   ");
    Serial.print(", ctrl_mode: ");
    Serial.print(local_ctrl_mode);
    Serial.print("   ");
    Serial.println("");
    Serial.println("");
  }
#endif
}

// =========================================================
//  Core 0 Setup & Loop
// =========================================================

void setup()
{
#ifdef Seri
  Serial.begin(115200);
  delay(1000); // シリアルモニタ起動待ち
  Serial.println("--- RP2040 FreeRTOS Motor Driver Start ---");
#endif

  EEPROM.begin(256);

  Wire.setSDA(0);
  Wire.setSCL(1);

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFlipMode(1);
  u8x8.setFont(font_n);

  pinMode(SW_L, INPUT);
  pinMode(SW_R, INPUT);
  pinMode(LMT_H, INPUT);
  pinMode(LMT_L, INPUT);
  pinMode(MOT_DIR, OUTPUT);
  pinMode(MOT_OnF, OUTPUT);
  pinMode(MOT_PWM, OUTPUT);

  pinMode(_CAN_INT, INPUT_PULLUP); // INTピンはプルアップ推奨
  pinMode(_CAN_CS, OUTPUT);

  pinMode(TX_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);

  pinMode(EDIT_SETTINGS, INPUT);

  digitalWrite(TX_LED, HIGH);
  digitalWrite(RX_LED, HIGH);

  // set_CANID();
  if (initialDisplay()) // initialDisplay() -> true or false
  {
    settingsEdit(); // set_CANID()の代わりに呼び出す．init_can()はsettingsEdit()の中で実行．
  }
  else
  {
    init_can();
  }
  // init_can();

  BaseDisplay();

// --- FreeRTOS オブジェクトの作成 ---
#ifdef Seri
  Serial.println("Core 0: Creating Semaphores and Mutex...");
#endif
  xCanInterruptSemaphore = xSemaphoreCreateBinary();
  xMotorDataMutex = xSemaphoreCreateMutex(); // データ保護用Mutex

  if (xCanInterruptSemaphore == NULL || xMotorDataMutex == NULL)
  {
#ifdef Seri
    Serial.println("Core 0: FAILED to create RTOS objects!");
#endif
    while (1)
      ; // 起動失敗
  }

  // CAN割り込みピンの設定
  attachInterrupt(digitalPinToInterrupt(_CAN_INT), canISR, FALLING);
#ifdef Seri
  Serial.println("Core 0: Attached CAN Interrupt.");
#endif

  // --- タスクの作成 (Core 0) ---

  // CAN読み取りタスク (高優先度)
  xTaskCreate(
      canReadTask,      // タスク関数
      "CANReadTask",    // 名前
      512,              // スタックサイズ (bytes) - 少し余裕を持たせる
      NULL,             // パラメータ
      3,                // 優先度 (高い)
      &xCanTaskHandle); // タスクハンドル

  // モーター制御タスク (中優先度)
  xTaskCreate(
      motorTask,          // タスク関数
      "MotorTask",        // 名前
      512,                // スタックサイズ
      NULL,               // パラメータ
      2,                  // 優先度 (中)
      &xMotorTaskHandle); // タスクハンドル

#ifdef Seri
  Serial.println("Core 0: CANReadTask and MotorTask created.");
  Serial.println("Core 0: Handing over to RTOS scheduler.");
#endif

// loop() タスク (Controlタスク) は自動的に優先度1で起動する
#ifdef Seri
  Serial.println("Core 0: Creating DisplayTask on Core 1...");
#endif

  // ディスプレイタスク (低優先度、Core 1に固定)
  xTaskCreateAffinitySet(
      displayTask,          // タスク関数
      "DisplayTask",        // 名前
      1024,                 // ★スタックサイズ (words) = 4096 bytes. これで十分なはず
      NULL,                 // パラメータ
      1,                    // 優先度 (低)
      (1 << 1),             // ★アフィニティマスク (1 << 1) = Core 1に固定
      &xDisplayTaskHandle); // タスクハンドル

#ifdef Seri
  Serial.println("Core 0: Handing over to RTOS scheduler.");
#endif
}

/**
 * @brief Core 0 のメインループ (Controlタスク 優先度1)
 * CANタスクから受信フラグを受け取り、メインロジック(Control)を実行する。
 */
void loop()
{
  // CANデータが来ていたらフラグを処理
  taskENTER_CRITICAL();
  if (flagRecv == 1)
  {
    flagRecv = 0; // フラグを消費
    RFlag = 1;    // Control() 関数用のフラグを立てる
  }
  taskEXIT_CRITICAL();

  if (digitalRead(EDIT_SETTINGS) == LOW)
  {
    isEditMode = true;
    delay(50);
    settingsEdit();

    // canバッファにたまった受信内容を掃除
    xSemaphoreTake(xCanInterruptSemaphore, 0); // 待ち時間0でTakeして、フラグを強制的にDownさせる
    while (CAN.checkReceive() == CAN_MSGAVAIL)
    {
      CAN.readMsgBuf(&ID_RCV, &len, rxBuf); // 読み出すだけで何もしない
    }
    wdt0 = millis();
    flagRecv = 0;
    RFlag = 0;
    BaseDisplay();
    delay(50);

    isEditMode = false;
  }

  if (!isEditMode)
  {
    Control();
  }

#ifdef Seri
  // Serial.println("[Core 0] loop() is running.");
#endif

  vTaskDelay(pdMS_TO_TICKS(1));
}

// =========================================================
//  Core 1 Setup & Loop
// =========================================================

/**
 * @brief Core 1 のメインループ (Displayタスク 優先度1)
 * 時間のかかるディスプレイ処理とシリアル出力を担当する。
 * (loop1() の代わりに手動で作成)
 */
void displayTask(void *pvParameters)
{
#ifdef Seri
  // Core 1 が起動したことをCore 0のシリアルに表示
  Serial.println("[Core 1] displayTask is ALIVE!");
#endif

  // 周期実行のための最終起動時間を記録
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  { // タスクは必ず無限ループにする

    if (isEditMode) // 設定編集中なら待ち状態にする
    {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(WDT2_TIMEOUT));
      continue;
    }
    SetDisplay();
    showStatus();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(WDT2_TIMEOUT));
  }
}
