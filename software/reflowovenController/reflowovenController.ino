/*

  U32.pde

  LiquidCrystal 16x2 example

  m2tklib = Mini Interative Interface Toolkit Library

  Copyright (C) 2011  olikraus@gmail.com

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <LiquidCrystal.h>
#include "M2tk.h"
#include "utility/m2ghlc.h"
#include <PID_v1.h>
#include <SPI.h>
#include <avr/wdt.h>

// LCD(D2-D7)
#define LCDrsPin 2
#define LCDenablePin 3
#define LCDd4Pin 4
#define LCDd5Pin 5
#define LCDd6Pin 6
#define LCDd7Pin 7
#include <EEPROM.h>

#define READY 0
#define START_WAIT 1
#define START_UP 2
#define RISE_FOR_PREHEAT 3
#define PREHEAT 4
#define RISE_FOR_PEAK_SETUP 5
#define RISE_FOR_PEAK 6
#define PEAK 7
#define COOLDOWN 8
#define WAIT_FINISH 9

//SSR
#define RELAYPIN 14 //ヒーター１
#define RELAYPIN2 15 //ヒーター２

//BUTTON
#define STARTBUTTON 8 //スタートボタン
#define NEXTBUTTON 9 //スタートボタン

//THEMAL CUPPLE
#define SLAVE 10 //熱電対ＣＳ



struct MyObject {

  uint32_t preheatTemperature[4] ;
  uint32_t preheatTime[4] ; /* sec */
  uint32_t heatTemperature[4] ;
  uint32_t heatTime[4]; /* sec */

  uint8_t rise_for_ph_cf;
  uint8_t preheat_delay_time;
  uint32_t rfph_kp; //rise for preheat pram
  uint32_t rfph_ki; //rise for preheat pram *1000
  uint32_t rfph_kd; //rise for preheat pram
  uint32_t ph_kp; //preheat pram
  uint32_t ph_ki; //preheat pram *1000
  uint32_t ph_kd; //preheat pram
  uint32_t rfp_kp; //rise for peak pram
  uint32_t rfp_ki; //rise for peak pram *1000
  uint32_t rfp_kd; //rise for peak pram

};


//デフォルト設定
MyObject setting_eeprom = {
  {160, 140, 160, 150},
  {70, 60, 120, 60},
  {220, 180, 240, 200},
  {20, 30, 60, 30},
  10, 3,
  150, 50, 60, // stage3 kp ki*1000 kd
  80, 25, 300, // stage4 kp ki*1000 kd
  150, 50, 100 // stage5 kp ki*1000 kd

};


//=================================================

uint8_t u8num = 0; //セーブ＆ロードデータ番号
uint32_t u32num = 0;
//uint8_t number = 0;

//プロファイル用データ
uint32_t preheatTemperature = 160 ;
uint32_t preheatTime = 70 ; /* sec */
uint32_t heatTemperature = 220 ;
uint32_t heatTime = 20; /* sec */
double currentTemperature;
//カウントダウン用
volatile uint32_t heatTimeC = 0; /* sec */
volatile uint32_t preheatTimeC = 0; /* sec */
int STATE = 0;

//ステージ時間測定
uint32_t stageMills[10];

volatile int wdt_Cycle = 0;
volatile long wdt_count = 0;
unsigned long time_DBG = 0;
int heaterOn = 0; // for debug

double SetPoint, Output, Input;
int windowSize = 2000;
unsigned long windowStartTime;
LiquidCrystal lcd(LCDrsPin, LCDenablePin, LCDd4Pin, LCDd5Pin, LCDd6Pin, LCDd7Pin);
PID myPID(&Input, &Output, &SetPoint, 90, 0.025, 25, DIRECT);
//=================================================
// Forward declaration of the toplevel element
M2_EXTERN_ALIGN(top_el_menu);


// show screen with my A0 pin value
M2_LABEL(el_a0_label, NULL, "preheatTemperature");
M2_U32NUM(el_a0_u32, "c3", &preheatTemperature);//M2_U32NUM(el_a0_u32, "c3.2", &preheatTemperature);
M2_ROOT(el_a0_ok, NULL, " ok ", &top_el_menu);
M2_LIST(list_a0) = { &el_a0_label, &el_a0_u32, &el_a0_ok };
M2_VLIST(el_a0_vlist, NULL, list_a0);
M2_ALIGN(top_el_a0, "-1|1W64H64", &el_a0_vlist);


// show screen with my A1 pin value
M2_LABEL(el_a1_label, NULL, "preheatTime");
M2_U32NUM(el_a1_u32, "c3", &preheatTime);
M2_ROOT(el_a1_ok, NULL, " ok ", &top_el_menu);
M2_LIST(list_a1) = { &el_a1_label, &el_a1_u32, &el_a1_ok };
M2_VLIST(el_a1_vlist, NULL, list_a1);
M2_ALIGN(top_el_a1, "-1|1W64H64", &el_a1_vlist);

M2_LABEL(el_a2_label, NULL, "heatTemperature");
M2_U32NUM(el_a2_u32, "c3", &heatTemperature);
M2_ROOT(el_a2_ok, NULL, " ok ", &top_el_menu);
M2_LIST(list_a2) = { &el_a2_label, &el_a2_u32, &el_a2_ok };
M2_VLIST(el_a2_vlist, NULL, list_a2);
M2_ALIGN(top_el_a2, "-1|1W64H64", &el_a2_vlist);

M2_LABEL(el_a3_label, NULL, "heatTime");
M2_U32NUM(el_a3_u32, "c3", &heatTime);
M2_ROOT(el_a3_ok, NULL, " ok ", &top_el_menu);
M2_LIST(list_a3) = { &el_a3_label, &el_a3_u32, &el_a3_ok };
M2_VLIST(el_a3_vlist, NULL, list_a3);
M2_ALIGN(top_el_a3, "-1|1W64H64", &el_a3_vlist);

//=========================================
//advanced menu
//=========================================
M2_LABEL(el_a4_label, NULL, "PH_COF_TEMP ");
M2_U8NUM(el_a4_u8, "c2", 0, 50, &setting_eeprom.rise_for_ph_cf);
M2_LABEL(el_a4_label_2, NULL, "PH_DELY_TIME ");
M2_U8NUM(el_a4_u8_2, "c2", 0, 10, &setting_eeprom.preheat_delay_time);
M2_ROOT(el_a4_ok, NULL, "ok", &top_el_menu);
M2_LIST(list_a4) = {
  //  &el_a4_label_3,NULL,
  &el_a4_label, &el_a4_u8,
  &el_a4_label_2, &el_a4_u8_2,
  &el_a4_ok
};
M2_GRIDLIST(el_a4_glist, "c2", list_a4);
M2_ALIGN(top_el_a4, NULL, &el_a4_glist);

M2_LABEL(el_a5_label, NULL, "Kp");
M2_U32NUM(el_a5_u32, "c4", &setting_eeprom.rfph_kp);
M2_LABEL(el_a5_label_2, NULL, "Ki");
M2_U32NUM(el_a5_u32_2, "c4.3", &setting_eeprom.rfph_ki);
M2_LABEL(el_a5_label_3, NULL, "Kd");
M2_U32NUM(el_a5_u32_3, "c4", &setting_eeprom.rfph_kd);
//same a4
M2_LIST(list_a5) = {
  //  &el_a5_label_3,NULL,
  &el_a5_label, &el_a5_u32,
  &el_a5_label_2, &el_a5_u32_2,
  &el_a5_label_3, &el_a5_u32_3,
  &el_a4_ok
};
M2_GRIDLIST(el_a5_glist, "c2", list_a5);
M2_ALIGN(top_el_a5, NULL, &el_a5_glist);

M2_U32NUM(el_a6_u32, "c4", &setting_eeprom.ph_kp);
M2_U32NUM(el_a6_u32_2, "c4.3", &setting_eeprom.ph_ki);
M2_U32NUM(el_a6_u32_3, "c4", &setting_eeprom.ph_kd);
M2_LIST(list_a6) = {
  &el_a5_label, &el_a6_u32,
  &el_a5_label_2, &el_a6_u32_2,
  &el_a5_label_3, &el_a6_u32_3,
  &el_a4_ok
};
M2_GRIDLIST(el_a6_glist, "c2", list_a6);
M2_ALIGN(top_el_a6, NULL, &el_a6_glist);

M2_U32NUM(el_a7_u32, "c4", &setting_eeprom.rfp_kp);
M2_U32NUM(el_a7_u32_2, "c4.3", &setting_eeprom.rfp_ki);
M2_U32NUM(el_a7_u32_3, "c4", &setting_eeprom.rfp_kd);
M2_LIST(list_a7) = {
  &el_a5_label, &el_a7_u32,
  &el_a5_label_2, &el_a7_u32_2,
  &el_a5_label_3, &el_a7_u32_3,
  &el_a4_ok
};
M2_GRIDLIST(el_a7_glist, "c2", list_a7);
M2_ALIGN(top_el_a7, NULL, &el_a7_glist);

M2_LABEL(el_b0_label, NULL, "LOAD: ");
M2_U8NUM(el_b0_u8, "c1", 0, 2, &u8num);
M2_ROOT(el_b0_cancel, NULL, "back", &top_el_menu);
M2_BUTTON(el_b0_ok, "", " ok ", fn_load);
M2_LIST(list_b0) = {
  &el_b0_label, &el_b0_u8,
  &el_b0_cancel, &el_b0_ok
};
M2_GRIDLIST(el_b0_glist, "c2", list_b0);
M2_ALIGN(top_el_b0, NULL, &el_b0_glist);

M2_LABEL(el_b1_label, NULL, "SAVE: ");
//same as b0
M2_BUTTON(el_b1_ok, "", " ok ", fn_save);
//same as b0
M2_LIST(list_b1) = {
  &el_b1_label, &el_b0_u8,
  &el_b0_cancel, &el_b1_ok
};
M2_GRIDLIST(el_b1_glist, "c2", list_b1);
M2_ALIGN(top_el_b1, NULL, &el_b1_glist);

M2_LABEL(el_c0_label, NULL, "START");
M2_BUTTON(el_c0_ok, "", " ok ", fn_start);
//same as b0
M2_LIST(list_c0) = {
  &el_c0_label, &el_b0_cancel, &el_c0_ok
};
M2_HLIST(el_c0_hlist, "c2", list_c0);
M2_ALIGN(top_el_c0, NULL, &el_c0_hlist);

M2_ROOT(el_switch_to_graphics, NULL, "Show Graphics", &m2_null_element);
//=================================================



// other voids that show any values

// Left entry: Menu name. Submenus must have a '.' at the beginning
// Right entry: Reference to the target dialog box (In this example all menus call the toplevel element again
m2_menu_entry m2_2lmenu_data[] =
{

  { "* D.S.S.T MENU *", &top_el_menu },
  { "START", &top_el_c0},

  { "SETTING", NULL },
  { ". PREHEAT TEMP", &top_el_a0 },
  { ". PREHEAT TIME", &top_el_a1},
  { ". PEAK TEMP", &top_el_a2},
  { ". PEAK TIME", &top_el_a3},

  { "SAVE & LOAD", NULL },
  { ". LOAD SETTING", &top_el_b0},
  { ". SAVE SETTING", &top_el_b1},


  { "ADVANCED OPTION",     NULL },
  { ". RiseForPreHeat", &top_el_a5},
  { ". RFPH option", &top_el_a4},
  { ". Preheat", &top_el_a6},
  { ". RiseForPeak", &top_el_a7},
  // etc..
  { NULL, NULL },
};

// The first visible line and the total number of visible lines.
// Both values are written by M2_2LMENU and read by M2_VSB
uint8_t m2_2lmenu_first;
uint8_t m2_2lmenu_cnt;

// M2_2LMENU definition
// Option l4 = four visible lines
// Option e15 = first column has a width of 15 pixel
// Option W43 = second column has a width of 43/64 of the display width

M2_2LMENU(el_2lmenu, "l4w17", &m2_2lmenu_first, &m2_2lmenu_cnt, m2_2lmenu_data, '+', '-', '\0');
M2_SPACE(el_space, "W1h1");
M2_VSB(el_vsb, "l4W2r1", &m2_2lmenu_first, &m2_2lmenu_cnt);
M2_LIST(list_2lmenu) = { &el_2lmenu, &el_space, &el_vsb };
M2_HLIST(el_hlist, NULL, list_2lmenu);
M2_ALIGN(top_el_menu, "-1|1W64H64", &el_hlist);

// m2 object and constructor
M2tk m2(&top_el_menu, m2_es_arduino, m2_eh_2bs, m2_gh_lc);
//シリアルポートを使う場合はこちら
//M2tk m2(&top_el_menu, m2_es_arduino_serial, m2_eh_4bs, m2_gh_arduino_serial);
void setup() {
  Serial.begin(9600);
  m2_SetLiquidCrystal(&lcd, 20, 4);

  m2.setPin(M2_KEY_SELECT, STARTBUTTON);
  m2.setPin(M2_KEY_NEXT,  NEXTBUTTON);
  //  m2.setPin(M2_KEY_PREV, uiKeyUpPin);
  //  m2.setPin(M2_KEY_EXIT, uiKeyExitPin);
  reflowSetup();
}

void loop() {


  m2.checkKey();
  if ( m2.handleKey() ) {
    m2.draw();
  }
}

//===================================
//EEPROM
//===================================

int load_eeprom(int num) {
  float eeprom_key ; //eeprom check key
  EEPROM.get( 0, eeprom_key );
  if (eeprom_key == 654.321f) {
    int eeAddress = sizeof(float);
    EEPROM.get( eeAddress, setting_eeprom);
    preheatTemperature = setting_eeprom.preheatTemperature[num] ;
    preheatTime = setting_eeprom.preheatTime[num] ; /* sec */
    heatTemperature = setting_eeprom.heatTemperature[num] ;
    heatTime = setting_eeprom.heatTime[num]; /* sec */
    return 0;
  } else {
    return 1;

  }


}

//EEPROM SAVE LOAD
int save_eeprom(int num) {
  float eeprom_key = 654.321f;
  EEPROM.put( 0, eeprom_key );
  setting_eeprom.preheatTemperature[num] = preheatTemperature ;
  setting_eeprom.preheatTime[num] = preheatTime ; /* sec */
  setting_eeprom.heatTemperature[num] = heatTemperature;
  setting_eeprom.heatTime[num] = heatTime; /* sec */
  EEPROM.put( sizeof(float), setting_eeprom );
  return 0;

}
//===================================

//===================================
// MENU FUNCTION
//===================================
void fn_load(m2_el_fnarg_p fnarg) {
  /* do something with the number */

  lcd.clear();
  if (load_eeprom(u8num) == 0) {
    lcd.print(F("load complete"));
  } else {
    lcd.print(F("invalid data"));

  }
  delay(2000);

  m2.setRoot(&top_el_menu);

}
void fn_save(m2_el_fnarg_p fnarg) {

  lcd.clear();
  if (save_eeprom(u8num) == 0) {
    lcd.print(F("save complete"));
  } else {
    lcd.print(F("save failed"));

  }
  delay(2000);
  m2.setRoot(&top_el_menu);

}

void fn_start(m2_el_fnarg_p fnarg) {
  save_eeprom(3);

  reflowMain();
  m2.setRoot(&top_el_menu);
}

//===================================
//reflow
//===================================
void reflowSetup() {
  STATE = READY;
  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW);
  pinMode(RELAYPIN2, OUTPUT);
  digitalWrite(RELAYPIN2, LOW);
  pinMode(SLAVE, OUTPUT);
  digitalWrite(SLAVE, HIGH);
  windowStartTime = millis();
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);
load_eeprom(3);

}
void reflowMain() {
  m2.setRoot(&el_switch_to_graphics);
  lcd.clear();
  STATE = START_UP;
  do {
    currentTemperature = getTemperature();
    Output = 0;
    switch (STATE) {
      case START_UP: // setup

        time_DBG = 0;
        preheatTimeC = preheatTime ; /* sec */
        heatTimeC = heatTime; /* sec */
        wdt_disable();

#ifdef TFTGRAPH
        TFTscreen.loop_start_up(preheatTemperature, heatTemperature);
#endif
        delayWDT_setup(6);
        windowStartTime = millis();
        STATE ++;
        break;
      case RISE_FOR_PREHEAT:
        SetPoint = preheatTemperature - setting_eeprom.rise_for_ph_cf;
        myPID.SetTunings((double)setting_eeprom.rfph_kp, (double)setting_eeprom.rfph_ki / 1000, (double)setting_eeprom.rfph_kd);
        keepHeat(currentTemperature);
        if (currentTemperature >= SetPoint) {
          //遅れを吸収（かっこ悪い方法）
          digitalWrite(RELAYPIN, LOW);
          digitalWrite(RELAYPIN2, LOW);
          myPID.SetMode(MANUAL);
          delay(setting_eeprom.preheat_delay_time);

          myPID.SetTunings((double)setting_eeprom.ph_kp, (double)setting_eeprom.ph_ki / 1000, (double)setting_eeprom.ph_kd);
          myPID.SetMode(AUTOMATIC);
          SetPoint = preheatTemperature;

          STATE ++;
        }
        break;
      case PREHEAT: // keep preheat time. main logic is countDown()


        keepHeat(currentTemperature);
        break;
      case RISE_FOR_PEAK_SETUP:
        SetPoint = heatTemperature;
        myPID.SetTunings((double)setting_eeprom.rfp_kp, (double)setting_eeprom.rfp_ki / 1000, (double)setting_eeprom.rfp_kd);
        STATE ++;
        break;

      case RISE_FOR_PEAK:


        keepHeat(currentTemperature);
        if (currentTemperature >= SetPoint) {
          STATE ++;
        }
        break;
      case PEAK:
        keepHeat(currentTemperature);
        break;
      case COOLDOWN: //

        digitalWrite(RELAYPIN, LOW);
        digitalWrite(RELAYPIN2, LOW);
        lcd.setCursor(3, 3);
        lcd.print(F("FINISH! "));
#ifdef TFTGRAPH

        TFTscreen.loop_cooldown();
#endif

        STATE ++;
        break;

      case WAIT_FINISH: //

        if (digitalRead(STARTBUTTON) == LOW ) {
          STATE = 0;
          wdt_disable();

        }
        break;

    }
    //    char str[32];
    //    char temp[16];
    //    char temp2[16];
    //    char temp3[16];
    //    sprintf(str, "%ld, %s, %s, %s, %s, %d",
    //            time_DBG,
    //            dtostrf(currentTemperature, 5, 2, temp),
    //            dtostrf(Output, 5, 2, temp2),
    //            dtostrf(millis() - windowStartTime, 5, 2, temp3),
    //            heaterOn ? "ON" : "OFF",
    //            STATE);
    //    Serial.println(str);

    time_DBG++;

  } while ( STATE != 0);


}
double getTemperature()
{
  unsigned int thermocouple; // 14-Bit Thermocouple Temperature Data + 2-Bit
  unsigned int internal; // 12-Bit Internal Temperature Data + 4-Bit
//#define DUMMY_SENS
#ifdef DUMMY_SENS
  return (analogRead(A6)) * 0.25;

#else
  digitalWrite(SLAVE, LOW);  //  Enable the chip
  thermocouple = (unsigned int)SPI.transfer(0x00) << 8;  //  Read high byte thermocouple
  thermocouple |= (unsigned int)SPI.transfer(0x00);  //  Read low byte thermocouple
  internal = (unsigned int)SPI.transfer(0x00) << 8;  //  Read high byte internal
  internal |= (unsigned int)SPI.transfer(0x00);  //  Read low byte internal
  digitalWrite(SLAVE, HIGH);  //  Disable the chip

  if ((thermocouple & 0x0001) != 0) {
    Serial.println("ERROR: ");
  } else {
    if ((thermocouple & 0x8000) == 0) { // above 0 Degrees Celsius
      return (thermocouple >> 2) * 0.25;
    } else {  // below zero
      return (0x3fff - (thermocouple >> 2) + 1)  * -0.25;
    }
  }
#endif

}
void keepHeat(double currentTemperature)
{
  Input = currentTemperature;
  myPID.Compute();

  if ((millis() - windowStartTime) > windowSize) {
    windowStartTime += windowSize;
  }

  if (Output > (millis() - windowStartTime)) {
    heaterOn = 1;

    digitalWrite(RELAYPIN, HIGH);


    if (STATE != RISE_FOR_PREHEAT) {
      digitalWrite(RELAYPIN2, HIGH);


    }
  } else {
    heaterOn = 0;
    digitalWrite(RELAYPIN, LOW);
    digitalWrite(RELAYPIN2, LOW);



  }
}



void delayWDT_setup(unsigned int ii) {  // ウォッチドッグタイマーをセット。
  // 引数はWDTCSRにセットするWDP0-WDP3の値。設定値と動作時間は概略下記
  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
  // 6=1sec, 7=2sec, 8=4sec, 9=8sec
  byte bb;
  if (ii > 9 ) {                        // 変な値を排除
    ii = 9;
  }
  bb = ii & 7;                          // 下位3ビットをbbに
  if (ii > 7) {                         // 7以上（7.8,9）なら
    bb |= (1 << 5);                     // bbの5ビット目(WDP3)を1にする
  }
  bb |= ( 1 << WDCE );

  MCUSR &= ~(1 << WDRF);                // MCU Status Reg. Watchdog Reset Flag ->0
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);   // ウォッチドッグ変更許可（WDCEは4サイクルで自動リセット）
  // set new watchdog timeout value
  WDTCSR = bb;                          // 制御レジスタを設定
  WDTCSR |= _BV(WDIE);
}

ISR(WDT_vect) {                         // WDTがタイムアップした時に実行される処理
  // wdt_Cycle++;                        // 必要ならコメントアウトを外す

  char str[32]; char temp[8];



  //  dtostrf(myPID.GetKp(), 5, 0, temp)
  //  dtostrf(myPID.GetKi(), 5,0, temp)
  //  dtostrf(myPID.GetKd(), 5, 0, temp)

  lcd.setCursor(0, 0);

  sprintf(str, " %s%-2u", "STATE:", STATE);
  lcd.print(str);
  lcd.setCursor(0, 1);
  sprintf(str, "%s%s", dtostrf(currentTemperature, 5, 2, temp), "C");
  lcd.print(str); lcd.print((char)223);
  sprintf(str, "/%s%s", dtostrf(SetPoint, 5, 2, temp), "C");
  lcd.print(str); lcd.print((char)223);

  switch (STATE) {
    case PREHEAT:
      lcd.setCursor(10, 0);
      sprintf(str, "%s%s", dtostrf(preheatTimeC, 3, 0, temp), "s");
      lcd.print(str);

      if (preheatTimeC <= 0) {
        lcd.setCursor(10, 0);
        lcd.print(F("      "));
        STATE++;
      }
      preheatTimeC--;

      break;
    case PEAK:
      lcd.setCursor(10, 0);
      sprintf(str, "%s%s", dtostrf(heatTimeC, 3, 0, temp), "s");
      lcd.print(str);
      if (heatTimeC <= 0) {
        lcd.setCursor(10, 0);
        lcd.print(F("      "));
        STATE++;
      }
      heatTimeC--;
      //Serial.println(heatTime);
      break;
  }



  //  if (wdt_Cycle >= 4) {
  //#ifdef TFTGRAPH
  //
  //    TFTscreen.wdt_graph(currentTemperature, STATE);
  //#endif
  //    wdt_Cycle = 0;
  //  }
}




