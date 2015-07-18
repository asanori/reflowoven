#include <PID_v1.h>
#include <SPI.h>
#include <avr/wdt.h>


//#define TFTGRAPH //TFTGRAPH ENABLE

#ifdef TFTGRAPH
#include <TFTG.h>
#endif


// STATE

#define READY 0
#define START_WAIT 1
#define START_UP 2
#define RISE_FOR_PREHEAT 3
#define PREHEAT 4
#define RISE_FOR_PEAK 5
#define PEAK 6
#define COOLDOWN 7
#define WAIT_FINISH 8


#ifdef TFTGRAPH

#define sclk A3
#define mosi A2
#define dc   A1
#define cs   A0
#define rst  0

#endif

#define RELAYPIN 5 //ヒーター１
#define RELAYPIN2 6 //ヒーター２
#define STARTBUTTON 8 //スタートボタン
#define SLAVE 10 //熱電対ＣＳ

#define  PREHEAT_TEMPERATURE  160
#define  PREHEAT_TIME 70
#define  HEAT_TEMPERATURE  220
#define  HEAT_TIME 20

int STATE = 0;
volatile int wdt_Cycle = 0;

unsigned long time_DBG = 0;
double preheatTemperature ;
volatile long preheatTime ; /* sec */
double heatTemperature ;
volatile long heatTime; /* sec */
double currentTemperature;

double SetPoint, Output, Input;
int windowSize = 2000;
unsigned long windowStartTime;


////////////////////////////////////////////////////////////TFTG.h
#ifdef TFTGRAPH
class TFTG : public Adafruit_ST7735 {
  public:
    TFTG(uint8_t CS, uint8_t RS, uint8_t RST);
    TFTG(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST);
    void begin();
    volatile int xPos ;
    void loop_ready();
    void loop_start_up(double lowerline,double upperline);
    void loop_cooldown();
    void wdt_graph(double drawtemperature,int state);


};

///////////////////////////////////////////////////////////tftg.cpp

TFTG::TFTG(uint8_t CS, uint8_t RS, uint8_t RST)
  : Adafruit_ST7735(CS, RS, RST)
{
  // as we already know the orientation (landscape, therefore rotated),
  // set default width and height without need to call begin() first.
  _width = ST7735_TFTHEIGHT;
  _height = ST7735_TFTWIDTH;
}
TFTG::TFTG(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST)
  : Adafruit_ST7735(CS, RS, SID, SCLK, RST)
{
  // as we already know the orientation (landscape, therefore rotated),
  // set default width and height without need to call begin() first.
  _width = ST7735_TFTHEIGHT;
  _height = ST7735_TFTWIDTH;
}
void TFTG::begin() {
  initR(INITR_REDTAB);
  initG();
  setRotation(1);
  setTextSize(1);
  setRotation(2);
  xPos = 0;
}
void TFTG::loop_ready() {
  xPos = 0;

  background(250, 16, 200);
  setCursor(0, 0);
  println("PUSH START");
}
void TFTG::loop_start_up(double lowerline,double upperline) {
      background(250, 16, 200);
      xPos = 0;
      stroke(0, 0, 255);
      line(0, height() - map(lowerline, 0, 300, 0, height()), width(), height() - map(lowerline, 0, 300, 0, height()));
      stroke(0,  255, 0);
      line(0, height() - map(upperline, 0, 300, 0, height()), width(), height() - map(upperline, 0, 300, 0, height()));
}
void TFTG::loop_cooldown(){
      stroke(255,  255, 255);

      println("FINISH! PUSH START");
  
}
void TFTG::wdt_graph(double drawtemperature,int state){
      switch (state) {
      //switch color
      case RISE_FOR_PREHEAT:stroke( 50, 180, 10); break;
      case PREHEAT:         stroke( 10, 50, 180); break;
      case RISE_FOR_PEAK:   stroke(180, 10, 10); break;
      case PEAK:            stroke( 10, 150, 180); break;
      default:              stroke(250, 180, 10); break;
    }
    int drawHeight = map(drawtemperature, 0, 300, 0, height());
    line(xPos,height() - drawHeight,xPos,height());
    if (xPos >= width()) {
      xPos = 0;
      background(250, 16, 200);
    }
    else {
      // increment the horizontal position:
      xPos++;
    }
  
}

//
/////////////////////////////////////////////////////////////////

TFTG TFTscreen = TFTG(cs, dc, mosi, sclk, rst);
#endif
PID myPID(&Input, &Output, &SetPoint, 90, 0.025, 25, DIRECT);


int heaterOn = 0; // for debug
void setup() {
  STATE = READY;

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW);
  pinMode(RELAYPIN2, OUTPUT);
  digitalWrite(RELAYPIN2, LOW);
  pinMode(SLAVE, OUTPUT);
  digitalWrite(SLAVE, HIGH);
  pinMode(STARTBUTTON, INPUT_PULLUP);
  windowStartTime = millis();
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);

#ifdef TFTGRAPH
  TFTscreen.begin();
#endif

}

void loop() {
  currentTemperature = getTemperature();
  Output = 0;

  switch (STATE) {
    case READY: // wait for start-switch
      digitalWrite(RELAYPIN, LOW);
      digitalWrite(RELAYPIN2, LOW);

#ifdef TFTGRAPH
      TFTscreen.loop_ready();
#endif

      delayWDT_setup(6);
      STATE++;
      break;

    case START_WAIT: // wait for start-switch

      if (digitalRead(STARTBUTTON) == LOW) {

        STATE++;
      }

      break;

    case START_UP: // setup

      time_DBG = 0;
      preheatTemperature = PREHEAT_TEMPERATURE ;
      preheatTime = PREHEAT_TIME ; /* sec */
      heatTemperature = HEAT_TEMPERATURE ;
      heatTime = HEAT_TIME; /* sec */
     wdt_disable();

#ifdef TFTGRAPH
      TFTscreen.loop_start_up(preheatTemperature,heatTemperature);
#endif
      delayWDT_setup(6);

      windowStartTime = millis();
      STATE ++;
      break;

    case RISE_FOR_PREHEAT:
      SetPoint = preheatTemperature - 10;
      myPID.SetTunings(150, 0.05, 60);
      keepHeat(currentTemperature);
      if (currentTemperature >= SetPoint) {
        //遅れを吸収（かっこ悪い方法）
        digitalWrite(RELAYPIN, LOW);
        digitalWrite(RELAYPIN2, LOW);
        myPID.SetMode(MANUAL);
        delay(3000);

        myPID.SetTunings(80, 0.025, 300);
        myPID.SetMode(AUTOMATIC);

        STATE ++;
      }
      break;
    case PREHEAT: // keep preheat time. main logic is countDown()

      SetPoint = preheatTemperature;

      keepHeat(currentTemperature);
      break;
    case RISE_FOR_PEAK:

      SetPoint = heatTemperature;
      myPID.SetTunings(150, 0.05, 100);
      keepHeat(currentTemperature);
      if (currentTemperature >= SetPoint) {
        STATE = PEAK;
      }
      break;
    case PEAK:
    keepHeat(currentTemperature);
      break;
    case COOLDOWN: //

      digitalWrite(RELAYPIN, LOW);
      digitalWrite(RELAYPIN2, LOW);
#ifdef TFTGRAPH

      TFTscreen.loop_cooldown();
#endif

      STATE ++;
      break;

    case WAIT_FINISH: //

      if (digitalRead(STARTBUTTON) == LOW) {
        STATE = 0;


      }
      break;

  }

  char str[32];
  char temp[16];
  char temp2[16];
  char temp3[16];
  sprintf(str, "%ld, %s, %s, %s, %s, %d",
          time_DBG,
          dtostrf(currentTemperature, 5, 2, temp),
          dtostrf(Output, 5, 2, temp2),
          dtostrf(millis() - windowStartTime, 5, 2, temp3),
          heaterOn ? "ON" : "OFF",
          STATE);
  Serial.println(str);

  time_DBG++;
}

double getTemperature()
{
  unsigned int thermocouple; // 14-Bit Thermocouple Temperature Data + 2-Bit
  unsigned int internal; // 12-Bit Internal Temperature Data + 4-Bit
#define DUMMY_SENS
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
  wdt_Cycle++;                        // 必要ならコメントアウトを外す
  // keep "SetPoint" temperature until preheatTime/heatTime
  switch (STATE) {
    case PREHEAT:
      if (preheatTime <= 0) {
        STATE++;
      }
      preheatTime--;
      Serial.println(preheatTime);
      break;
    case PEAK:
      if (heatTime <= 0) {
        STATE++;
      }
      heatTime--;
      //Serial.println(heatTime);
      break;
  }



  if (wdt_Cycle >= 4) {
#ifdef TFTGRAPH

    TFTscreen.wdt_graph(currentTemperature,STATE);
#endif
    wdt_Cycle = 0;
  }
}



